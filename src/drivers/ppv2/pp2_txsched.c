/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_txsched.c
 *
 * Port and txq rate limiting and txq arbitration
 */

#include "std_internal.h"
#include "pp2_types.h"
#include "pp2.h"
#include "pp2_port.h"

#define PP2_AMPLIFY_FACTOR_MTU			(3)
#define PP2_WRR_WEIGHT_UNIT			(256)
#define MVPP2_TXP_MAX_CONFIGURABLE_BUCKET_SIZE	(MVPP2_TXP_TOKEN_SIZE_MAX - MVPP2_TXP_REFILL_TOKENS_MAX)
#define MVPP2_TXQ_MAX_CONFIGURABLE_BUCKET_SIZE	(MVPP2_TXQ_TOKEN_SIZE_MAX - MVPP2_TXQ_REFILL_TOKENS_MAX)
#define MVPP2_TXP_REFILL_PERIOD_MIN		(1)
#define MVPP2_TXQ_REFILL_PERIOD_MIN		(1)

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

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	if (port->enable_port_rate_limit) {

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
	} else {
		period = MVPP2_TXP_REFILL_PERIOD_MIN;
		tokens = MVPP2_TXP_REFILL_TOKENS_MAX;
	}

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
		burst = MVPP2_TXP_MAX_CONFIGURABLE_BUCKET_SIZE / 8;

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

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	if (port->txq_config[txq].rate_limit_enable) {

		rc = pp2_txsched_rate_calc(rate, accuracy, &period, &tokens);
		if (rc) {
			pr_err("%s: Can't provide rate of %d [Kbps] with accuracy of %d [%%]\n",
			       __func__, rate, accuracy);
			return rc;
		}

		if (tokens > MVPP2_TXQ_REFILL_TOKENS_MAX)
			tokens = MVPP2_TXQ_REFILL_TOKENS_MAX;

		if (period > MVPP2_TXQ_REFILL_PERIOD_MAX)
			period = MVPP2_TXQ_REFILL_PERIOD_MAX;
	} else {
		tokens = MVPP2_TXQ_REFILL_TOKENS_MAX;
		period = MVPP2_TXQ_REFILL_PERIOD_MIN;
	}

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
		burst = MVPP2_TXQ_MAX_CONFIGURABLE_BUCKET_SIZE / 8;

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

/* Set TXQ to work in WRR mode and set relative weight. */
/*   Weight range [1..N] */
static int pp2_txsched_queue_wrr_set(struct pp2_port *port, u8 txq, u8 weight)
{
	u32 regVal;
	int txPortNum;

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

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

/* Sets the register called MTU.
 *   Although, it really sets it to value of [frame_tx_size-CRC], which is
 *   the frame_size as viewed by the tx_scheduler.
 */
static void pp2_port_txsched_set_mtu(struct pp2_port *port)
{
	u32 val, mtu;
	u32 tx_port_num;
	uintptr_t cpu_slot = port->cpu_slot;

	mtu = (port->port_mtu + ETH_HLEN) * 8;

	/* WA for wrong Token bucket update: Set MTU value = 3*real MTU value */
	mtu = 3 * mtu;

	if (mtu > MVPP2_TXP_MTU_MAX)
		mtu = MVPP2_TXP_MTU_MAX;

	/* Indirect access to registers */
	tx_port_num = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	/* Set MTU */
	val = pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_MTU_REG);
	val &= ~MVPP2_TXP_MTU_MAX;
	val |= mtu;
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_MTU_REG, val);
}

/* If the user requested weight range does cannot be fitted into the dynamic range allowed by the hardware after
 * MTU restrictions, it has to be compressed under the following terms --
 * - User supplied 0/1 weight must be remapped to the minimum allowed by hw,
 * - User supplied 255 weight must be remapped to 255,
 * - The slope of the mapping near the minimum must be equal to the minimum, so that if the user requests 1,2, the
 *   result will be e.g. 8,16.
 * This is accomplished using a first order rational function (Ax + B) / (Cx + D)
 */
static u8 pp2_txsched_rational_weight_remap(u32 weight, u32 min, u32 max)
{
	/* The solution for minimum weight of 1 contains square roots, but doesn't for minimum weight of 0, so we
	 * offset the parameters by -1 and offset the result back.
	 */
	weight -= 1;
	min -= 1;
	max -= 1;
	return ((min * max * max + min - max) * weight + min * max * (max - 1)) /
	       ((min * max + min - max) * weight + max * (max - 1)) + 1;
}

/* The user supplied WRR weights need to be remapped to comply with hardware requirements --
 * - The weights must be larger than the MTU/256,
 * - The smallest weight must be equal to MTU/256
 * We perform a linear mapping if there is enough dynamic range, or a non-linear one if there isn't.
 */
static void pp2_txsched_remap_weights(struct pp2_port *port, u8 remapped_weights[])
{
	u32 hw_min, user_min = 0xff, user_max = 0x0;
	u8 txq;
	u32 mtu;
	int txPortNum;
	int accommodating_dynamic_range; /* Can user requested range be met after MTU restriction */

	txPortNum = MVPP2_TX_PORT_NUM(port->id);
	pp2_reg_write(port->cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	/* Weight * 256 bytes * 8 bits must be larger then MTU [bits] */
	mtu = pp2_reg_read(port->cpu_slot, MVPP2_TXP_SCHED_MTU_REG);
	mtu /= PP2_AMPLIFY_FACTOR_MTU;
	mtu /= BITS_PER_BYTE; /* move to bytes */
	mtu = ALIGN(mtu, PP2_WRR_WEIGHT_UNIT);
	hw_min = mtu / PP2_WRR_WEIGHT_UNIT;

	for (txq = 0; txq < port->num_tx_queues; txq++) {

		if (port->txq_config[txq].sched_mode == PP2_PPIO_SCHED_M_WRR) {

			if (port->txq_config[txq].weight == 0)
				port->txq_config[txq].weight = 1;

			if (port->txq_config[txq].weight > user_max)
				user_max = port->txq_config[txq].weight;

			if (port->txq_config[txq].weight < user_min)
				user_min = port->txq_config[txq].weight;
		}
	}

	if (user_min > user_max) /* WRR unused */
		return;

	if ((user_max / user_min) < (MVPP2_TXQ_WRR_WEIGHT_MAX / hw_min))
		accommodating_dynamic_range = 1;
	else
		accommodating_dynamic_range = 0;

	for (txq = 0; txq < port->num_tx_queues; txq++) {

		if (port->txq_config[txq].sched_mode == PP2_PPIO_SCHED_M_WRR) {

			if (accommodating_dynamic_range)
				remapped_weights[txq] = port->txq_config[txq].weight * hw_min / user_min;
			else
				remapped_weights[txq] =
					pp2_txsched_rational_weight_remap(port->txq_config[txq].weight,
									  hw_min, MVPP2_TXQ_WRR_WEIGHT_MAX);
		}
	}
}

/* Initialize port and queue rate limits and txq arbitration */
int pp2_port_config_txsched(struct pp2_port *port)
{
	int rc, txq;
	u8 remapped_weights[MVPP2_MAX_TXQ];

	/* Store hardware state */

	/* Set port MTU (which is used later in the initialization) */
	pp2_port_txsched_set_mtu(port);

	/* Set port rate limit and burst size */
	rc = pp2_txsched_port_rate_set(port, port->rate_limit_params.cir);
	if (rc)
		return rc;

	rc = pp2_txsched_port_burst_set(port, port->rate_limit_params.cbs * 1024);
	if (rc)
		return rc;

	pp2_txsched_remap_weights(port, remapped_weights);

	/* Set txq rate limits, burst sizes, arbitration mode and WRR weight */
	for (txq = 0; txq < port->num_tx_queues; txq++) { /* This only works in logical ports post reprioritization */

		rc = pp2_txsched_queue_rate_set(port, txq, port->txq_config[txq].rate_limit_params.cir);
		if (rc)
			return rc;

		rc = pp2_txsched_queue_burst_set(port, txq, port->txq_config[txq].rate_limit_params.cbs * 1024);
		if (rc)
			return rc;

		if (port->num_tx_queues > 1 && NOT_LPBK_PORT(port)) {
			rc = pp2_txsched_queue_arbitration_set(port, txq, port->txq_config[txq].sched_mode,
							       remapped_weights[txq]);
			if (rc)
				return rc;
		}
	}

	return 0;
}

void pp2_port_deinit_txsched(struct pp2_port *port)
{
}
