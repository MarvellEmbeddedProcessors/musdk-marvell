/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/**
 * @file pp2_txsched.c
 *
 * Port and txq rate limiting and txq arbitration
 */

#include "std_internal.h"
#include "pp2_types.h"
#include "pp2.h"
#include "pp2_port.h"

#define PP2_AMPLIFY_FACTOR_MTU		(3)
#define PP2_WRR_WEIGHT_UNIT		(8)

/* Calculate period and tokens accordingly with required rate and accuracy [kbps] */
static int pp2_txsched_rate_calc(u32 rate, u32 accuracy, u32 *pperiod, u32 *ptokens)
{
	/* Calculate refill tokens and period - rate [kbps] = tokens [bits] * 1000 / period [usec] */
	/* Assume:  Tclock [MHz] / BasicRefillNoOfClocks = 1 */
	u32 period, tokens, calc;
	s32 var;

	if (rate == 0) {
		/* Disable traffic from the port: tokens = 0 */
		if (pperiod != NULL)
			*pperiod = 1000;

		if (ptokens != NULL)
			*ptokens = 0;

		return 0;
	}

	/* Find values of "period" and "tokens" match "rate" and "accuracy" when period is minimal */
	for (period = 1; period <= 1000; period++) {
		tokens = 1;
		while (true) {
			calc = (tokens * 1000) / period;
			var = ((calc - rate) * 100) / 100;
			var = (var > 0) ? var : -var;
			if (var <= accuracy) {
				if (pperiod != NULL)
					*pperiod = period;
				if (ptokens != NULL)
					*ptokens = tokens;

				return 0;
			}
			if (calc > rate)
				break;

			tokens++;
		}
	}
	return -EDOM;
}

/* Set bandwidth limitation for TX port
 *   rate [Kbps]    - steady state TX bandwidth limitation
 */
static int pp2_txsched_port_rate_set(struct pp2_port *port, int rate)
{
	int	rc;
	u32	regVal;
	u32	tokens, period, txPortNum, accuracy = 0;

	if (!port->enable_port_rate_limit)
		return 0;

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	rc = pp2_txsched_rate_calc(rate, accuracy, &period, &tokens);
	if (rc) {
		pr_err("%s: Can't provide rate of %d [Kbps] with accuracy of %d [%%]\n",
		       __func__, rate, accuracy);
		return rc;
	}
	if (tokens > MVPP2_TXP_REFILL_TOKENS_MAX)
		tokens = MVPP2_TXP_REFILL_TOKENS_MAX;

	if (period > MVPP2_TXP_REFILL_PERIOD_MAX)
		period = MVPP2_TXP_REFILL_PERIOD_MAX;

	regVal = pp2_reg_read(port->cpu_slot, MVPP2_TXP_SCHED_REFILL_REG);

	regVal &= ~MVPP2_TXP_REFILL_TOKENS_ALL_MASK;
	regVal |= MVPP2_TXP_REFILL_TOKENS_MASK(tokens);

	regVal &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
	regVal |= MVPP2_TXP_REFILL_PERIOD_MASK(period);

	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_REFILL_REG, regVal);

	return 0;
}

/* Set maximum burst size for TX port
 *   burst [bytes] - number of bytes to be sent with maximum possible TX rate,
 *                    before TX rate limitation will take place.
 */
static int pp2_txsched_port_burst_set(struct pp2_port *port, int burst)
{
	u32 size, mtu;
	u32 txPortNum;

	if (!port->enable_port_rate_limit)
		return 0;

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	/* Caclulate Token Bucket Size */
	size = 8 * burst;

	if (size > MVPP2_TXP_TOKEN_SIZE_MAX)
		size = MVPP2_TXP_TOKEN_SIZE_MAX;

	/* Token bucket size must be larger then MTU. We set it to max. */
	mtu = MVPP2_TX_MTU_MAX;
	if (mtu > size) {
		pr_err("%s Error: Bucket size (%d bytes) < MTU (%d bytes)\n",
		       __func__, (size / 8), (mtu / 8));
		return -EINVAL;
	}
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, size);

	return 0;
}

/* Set bandwidth limitation for TXQ
 *   rate  [Kbps]  - steady state TX rate limitation
 */
static int pp2_txsched_queue_rate_set(struct pp2_port *port, int txq, int rate)
{
	u32 regVal;
	u32 txPortNum, period, tokens, accuracy = 0;
	int rc;

	if (!port->txq_config[txq].rate_limit_enable)
		return 0;

	rc = pp2_txsched_rate_calc(rate, accuracy, &period, &tokens);
	if (rc) {
		pr_err("%s: Can't provide rate of %d [Kbps] with accuracy of %d [%%]\n",
		       __func__, rate, accuracy);
		return rc;
	}

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	if (tokens > MVPP2_TXQ_REFILL_TOKENS_MAX)
		tokens = MVPP2_TXQ_REFILL_TOKENS_MAX;

	if (period > MVPP2_TXQ_REFILL_PERIOD_MAX)
		period = MVPP2_TXQ_REFILL_PERIOD_MAX;

	regVal = pp2_reg_read(port->cpu_slot, MVPP2_TXQ_SCHED_REFILL_REG(txq));

	regVal &= ~MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
	regVal |= MVPP2_TXQ_REFILL_TOKENS_MASK(tokens);

	regVal &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
	regVal |= MVPP2_TXQ_REFILL_PERIOD_MASK(period);

	pp2_reg_write(port->cpu_slot, MVPP2_TXQ_SCHED_REFILL_REG(txq), regVal);

	return 0;
}

/* Set maximum burst size for TX port
 *   burst [bytes] - number of bytes to be sent with maximum possible TX rate,
 *                    before TX bandwidth limitation will take place.
 */
static int pp2_txsched_queue_burst_set(struct pp2_port *port, int txq, int burst)
{
	u32 size, mtu;
	int txPortNum;

	if (!port->txq_config[txq].rate_limit_enable)
		return 0;

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	/* Calculate Tocket Bucket Size */
	size = 8 * burst;

	if (size > MVPP2_TXQ_TOKEN_SIZE_MAX)
		size = MVPP2_TXQ_TOKEN_SIZE_MAX;

	/* Token bucket size must be larger then MTU. We set it to max. */
	mtu = MVPP2_TX_MTU_MAX;
	if (mtu > size) {
		pr_err("%s Error: Bucket size (%d bytes) < MTU (%d bytes)\n",
		       __func__, (size / 8), (mtu / 8));
		return -EINVAL;
	}

	pp2_reg_write(port->cpu_slot, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq), size);

	return 0;
}

/* Performs linear mapping of the 1-255 range given to the user to the
 *   weight_min-255 range restricted by the current MTU.
 */
static u8 pp2_txsched_weight_remap(u32 weight, u32 weight_min)
{
	if (weight_min > MVPP2_TXQ_WRR_WEIGHT_MAX)
		return MVPP2_TXQ_WRR_WEIGHT_MAX;

	return weight_min + ((MVPP2_TXQ_WRR_WEIGHT_MAX - weight_min) * weight) / MVPP2_TXQ_WRR_WEIGHT_MAX;
}

/* Set TXQ to work in WRR mode and set relative weight. */
/*   Weight range [1..N] */
static int pp2_txsched_queue_wrr_set(struct pp2_port *port, u8 txq, u8 weight)
{
	u32 regVal, mtu, mtu_aligned, weight_min;
	int txPortNum;

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	/* Weight * 256 bytes * 8 bits must be larger then MTU [bits] */
	mtu = pp2_reg_read(port->cpu_slot, MVPP2_TXP_SCHED_MTU_REG);

	/* WA for wrong Token bucket update: Set MTU value = 3*real MTU value, now get read MTU*/
	mtu /= PP2_AMPLIFY_FACTOR_MTU;
	mtu /= BITS_PER_BYTE; /* move to bytes */
	mtu_aligned = ALIGN(mtu, PP2_WRR_WEIGHT_UNIT);
	weight_min = mtu_aligned / PP2_WRR_WEIGHT_UNIT;

	weight = pp2_txsched_weight_remap(weight, weight_min);

	regVal = pp2_reg_read(port->cpu_slot, MVPP2_TXQ_SCHED_WRR_REG(txq));

	regVal &= ~MVPP2_TXQ_WRR_WEIGHT_ALL_MASK;
	regVal |= MVPP2_TXQ_WRR_WEIGHT_MASK(weight);
	pp2_reg_write(port->cpu_slot, MVPP2_TXQ_SCHED_WRR_REG(txq), regVal);

	regVal = pp2_reg_read(port->cpu_slot, MVPP2_TXP_SCHED_FIXED_PRIO_REG);
	regVal &= ~(1 << txq);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_FIXED_PRIO_REG, regVal);

	return 0;
}

/* Set TXQ to work in FIX priority mode */
static int pp2_txsched_queue_fixed_prio_set(struct pp2_port *port, u8 txq)
{
	u32 regVal;
	int txPortNum;

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	regVal = pp2_reg_read(port->cpu_slot, MVPP2_TXP_SCHED_FIXED_PRIO_REG);
	regVal |= (1 << txq);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_FIXED_PRIO_REG, regVal);

	return 0;
}

static int pp2_txsched_queue_arbitration_set(struct pp2_port *port, u8 txq,
					     enum pp2_ppio_outq_sched_mode mode, u8 weight)
{
	if (mode == PP2_PPIO_SCHED_M_WRR)
		return pp2_txsched_queue_wrr_set(port, txq, weight);

	if (mode == PP2_PPIO_SCHED_M_SP)
		return pp2_txsched_queue_fixed_prio_set(port, txq);

	pr_err("%s Error: Invalid egress arbitration mode on p%dq%d: %d.\n",
	       __func__, port->id, txq, (int) mode);

	return -EINVAL;
}

/* Initialize port and queue rate limits and txq arbitration */
int pp2_port_config_txsched(struct pp2_port *port)
{
	int rc, txq;
	/* Store hardware state */

	/* Set/verify scheduler period */

	/* Set port rate limit and burst size */
	rc = pp2_txsched_port_rate_set(port, port->rate_limit_params.cir);
	if (rc)
		return rc;

	rc = pp2_txsched_port_burst_set(port, port->rate_limit_params.cbs * 1000);
	if (rc)
		return rc;

	/* Set txq rate limits, burst sizes, arbitration mode and WRR weight */
	for (txq = 0; txq < port->num_tx_queues; txq++) { /* This only works in logical ports post reprioritization */

		rc = pp2_txsched_queue_rate_set(port, txq, port->txq_config[txq].rate_limit_params.cir);
		if (rc)
			return rc;

		rc = pp2_txsched_queue_burst_set(port, txq, port->txq_config[txq].rate_limit_params.cbs * 1000);
		if (rc)
			return rc;

		if (port->num_tx_queues > 1 && port->id != PP2_LOOPBACK_PORT) {
			rc = pp2_txsched_queue_arbitration_set(port, txq, port->txq_config[txq].sched_mode,
							       port->txq_config[txq].weight);
			if (rc)
				return rc;
		}
	}

	return 0;
}

void pp2_port_deinit_txsched(struct pp2_port *port)
{
}
