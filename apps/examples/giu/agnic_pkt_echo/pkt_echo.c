/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "lib/mv_pme.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "utils.h"
#include "mv_agnic_pfio.h"


/*#define VERBOSE_CHECKS_FOR_SF_BUG*/

#define PKT_ECHO_APP_MAX_BURST_SIZE		64

#define AGNIC_DEFAULT_NUM_TCS			1
#define AGNIC_DEFAULT_PKT_OFFS			0
#define AGNIC_BPOOLS_BUFF_SIZE			2048

#define PKT_ECHO_APP_TX_Q_SIZE			512
#define PKT_ECHO_APP_RX_Q_SIZE			(PKT_ECHO_APP_TX_Q_SIZE)

#define PKT_ECHO_APP_MAX_NUM_CORES		4
#define PKT_ECHO_APP_MAX_NUM_PORTS		1

#define PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_ECHO_APP_MAX_NUM_QS_PER_TC		PKT_ECHO_APP_MAX_NUM_CORES

#define PKT_ECHO_APP_DMA_MEM_SIZE		(80 * 1024 * 1024)

#define PKT_ECHO_APP_STATS_DFLT_THR		1000
#define PKT_ECHO_APP_CTRL_TRD_THRESH		200

#define PKT_ECHO_APP_PREFETCH_SHIFT		7

#define AGNIC_PA_DEVICE_WATERMARK	0xcafe000000000000
#define AGNIC_COOKIE_DEVICE_WATERMARK	0xcafecafe
#define AGNIC_COOKIE_DRIVER_WATERMARK	0xdeaddead

#define AGNIC_PCI_VENDOR		"Marvell Technology"
#define AGNIC_PCI_PRODUCT		"Device 7080"
#define AGNIC_PCI_MMAP_FILE_NAME	"/dev/mem"

#define next_q_idx(_idx, _size) (((_idx+1) == _size) ? 0 : (_idx+1))
#define q_occupancy(prod, cons, q_size)	\
	(((prod) - (cons) + (q_size)) & ((q_size) - 1))
#define q_space(prod, cons, q_size)	\
	((q_size) - q_occupancy((prod), (cons), (q_size)) - 1)


/*
 * Tx shadow queue
 */
struct shadow_q {
	u8			 tc;
	u8			 qid;
	u16			 read_ind;	/* read index */
	u16			 write_ind;	/* write index */

	struct agnic_buff_inf	*buffs_inf;	/* pointer to the buffer object */
};

/*
 * Local thread port parameters
 */
struct lcl_port_desc {
	int			 id;		/* Local port ID */
	int			 lcl_id;	/* Local thread ID */
	int			 pfio_id;	/* PFIO port ID */
	struct agnic_pfio	*pfio;		/* PFIO object returned by agnic_pfio_init() */
	int			 num_shadow_qs;	/* Number of shadow queues */
	int			 shadow_q_size;	/* Size of shadow queue */
	struct shadow_q		*shadow_qs;	/* shadow queue */
};

struct glob_arg {
	struct glb_common_args		 cmn_args; /* Keep first */

	u16				rxq_size;
	u16				txq_size;

	struct agnic_pfio		*pfio;

	eth_addr_t			 eth_addr;

	void				*pci_bar;
	void				*buffs_addr;
};

struct local_arg {
	struct local_common_args	 cmn_args; /* Keep first */

	struct lcl_port_desc		 ports_desc[PKT_ECHO_APP_MAX_NUM_PORTS];
};


static struct glob_arg garg = {};


static inline u16 free_multi_buffers(struct lcl_port_desc	*rx_port,
				     struct lcl_port_desc	*tx_port,
				     u16			 start_idx,
				     u16			 num,
				     u8				 tc,
				     u8				 tc_qid,
				     u8				 sh_qid)
{
	u16			idx = start_idx;
	u16			cont_in_shadow, req_num;
	struct shadow_q		*shadow_q;

	shadow_q = &tx_port->shadow_qs[sh_qid];

	cont_in_shadow = tx_port->shadow_q_size - start_idx;

	if (num <= cont_in_shadow) {
		req_num = num;
		agnic_pfio_inq_put_buffs(rx_port->pfio, tc, tc_qid, &shadow_q->buffs_inf[idx], &req_num);
		idx = idx + num;
		if (idx == tx_port->shadow_q_size)
			idx = 0;
	} else {
		req_num = cont_in_shadow;
		agnic_pfio_inq_put_buffs(rx_port->pfio, tc, tc_qid, &shadow_q->buffs_inf[idx], &req_num);

		req_num = num - cont_in_shadow;
		agnic_pfio_inq_put_buffs(rx_port->pfio, tc, tc_qid, &shadow_q->buffs_inf[0], &req_num);
		idx = num - cont_in_shadow;
	}

	return idx;
}

static inline void free_sent_buffers(struct lcl_port_desc	*rx_port,
				     struct lcl_port_desc	*tx_port,
				     u8				 tc,
				     u8				 tc_qid,
				     u8				 sh_qid)
{
	u16			idx;
	u16 tx_num = 0;

	agnic_pfio_get_num_outq_done(tx_port->pfio, tc, tc_qid, &tx_num);

	if (unlikely(!tx_num))
		return;

	idx = free_multi_buffers(rx_port, tx_port, tx_port->shadow_qs[sh_qid].read_ind, tx_num, tc, tc_qid, sh_qid);
	tx_port->shadow_qs[sh_qid].read_ind = idx;
}

static int do_echo(struct local_arg	*larg,
		   u8			 rx_ppio_id,
		   u8			 tx_ppio_id,
		   u8			 tc,
		   u8			 tc_qid,
		   u8			 sh_qid,
		   u16			 num)
{
	struct agnic_pfio_desc	 descs[PKT_ECHO_APP_MAX_BURST_SIZE];
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;
	struct lcl_port_desc	*rx_lcl_port_desc = &(larg->ports_desc[rx_ppio_id]);
	struct lcl_port_desc	*tx_lcl_port_desc = &(larg->ports_desc[tx_ppio_id]);
	struct shadow_q		*shadow_q;
	int			 shadow_q_size;
	u16			 i, tx_num, write_ind, tmp_num;
	int			 echo, prefetch_shift = larg->cmn_args.prefetch_shift;

	echo = larg->cmn_args.echo;

	shadow_q = &tx_lcl_port_desc->shadow_qs[sh_qid];
	shadow_q_size = tx_lcl_port_desc->shadow_q_size;

	/* Work with local copy */
	write_ind = shadow_q->write_ind;
	tmp_num = q_space(write_ind, shadow_q->read_ind, shadow_q_size);
	if (unlikely(num > tmp_num))
		num = tmp_num;

	agnic_pfio_recv(rx_lcl_port_desc->pfio, tc, tc_qid, descs, &num);
	if (!num)
		return 0;
/*printf("recv %d pkts for echo (TH:%d, tc:%d, q:%d, sh:%d)\n", num, larg->cmn_args.id, tc, tc_qid, sh_qid);
*/

	perf_cntrs->rx_cnt += num;

	tmp_num = 0;
	for (i = 0; i < num; i++) {
		char		*buff = (char *)agnic_pfio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = agnic_pfio_inq_desc_get_phys_addr(&descs[i]);
		u16		 len = agnic_pfio_inq_desc_get_pkt_len(&descs[i]);
		u16		 pkt_offset = AGNIC_DEFAULT_PKT_OFFS;
		u8		 md = agnic_pfio_inq_desc_get_md(&descs[i]);
		u32		 tmp_flags, rx_flags;

		if (unlikely(((pa & AGNIC_PA_DEVICE_WATERMARK) == AGNIC_PA_DEVICE_WATERMARK) ||
			((uintptr_t)buff == AGNIC_COOKIE_DEVICE_WATERMARK) ||
			((uintptr_t)buff == AGNIC_COOKIE_DRIVER_WATERMARK))) {
			pr_info("\r[ERROR] AGNIC: Illegal descriptor (PA:0x%"PRIx64", cookie:%p)!!!!\n", pa, buff);
			continue;
		}

#ifdef VERBOSE_CHECKS_FOR_SF_BUG
		/* The following code assumes that the frame is ETH/IPv4 */
		if (likely(((buff[sizeof(struct ether_header)] & 0xf0) == 0x40) &&
			swab16(*(u16 *)(&buff[sizeof(struct ether_header)-2])) == 0x0800)) {
			struct iphdr *ipv4hdr = (struct iphdr *)(buff + sizeof(struct ether_header));
			u16 tot_len = swab16(ipv4hdr->tot_len) + sizeof(struct ether_header);
			u16 exp_len = len;
			u32 watermark;

			if (md == 1)
				exp_len -= AGNIC_PFIO_MD_MODE_16B_LEN;

			/* TODO: is this a BUG??? */
			if (tot_len < 60)
				tot_len = 60;

			watermark = *(u32 *)&buff[sizeof(struct ether_header) +
						sizeof(struct ip) +
						sizeof(struct udphdr)];
			if (unlikely(watermark == MVAPPS_PLD_WATERMARK)) {
				struct agnic_buff_inf buffs_inf;
				u16 req_num = 1;

				pr_info("\r[ERROR] AGNIC: Illegal buffer content (0x%08x)!\n", watermark);
				buffs_inf.cookie = (uintptr_t)buff;
				buffs_inf.addr = pa;
				agnic_pfio_inq_put_buffs(rx_lcl_port_desc->pfio, tc, tc_qid, &buffs_inf, &req_num);
				continue;
			}

			if (unlikely(tot_len != exp_len)) {
				struct agnic_buff_inf buffs_inf;
				u16 req_num = 1;

				pr_info("\r[ERROR] AGNIC: length mismatch (frame %u vs rx-desc %u @%d)!\n",
					tot_len, exp_len, write_ind);
				buffs_inf.cookie = (uintptr_t)buff;
				buffs_inf.addr = pa;
				agnic_pfio_inq_put_buffs(rx_lcl_port_desc->pfio, tc, tc_qid, &buffs_inf, &req_num);
				continue;
			}
		}
#endif /* VERBOSE_CHECKS_FOR_SF_BUG */

/*
 *printf("received packet (va:%p, pa 0x%"PRIx64", len %d):\n", buff, pa, len);
 *mem_disp(buff, len);
*/

		if (likely(echo)) {
			char *tmp_buff;

			if (num - i > prefetch_shift) {
				tmp_buff = (char *)agnic_pfio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += pkt_offset;
				prefetch(tmp_buff);
			}
			tmp_buff = buff;
			pr_debug("buff(%p)\n", tmp_buff);
			tmp_buff += pkt_offset;
			if (md == 1)
				tmp_buff += AGNIC_PFIO_MD_MODE_16B_LEN;
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}

		rx_flags = descs[i].cmds[0];
		agnic_pfio_outq_desc_reset(&descs[i]);
		agnic_pfio_outq_desc_set_phys_addr(&descs[i], pa);
		agnic_pfio_outq_desc_set_pkt_offset(&descs[i], pkt_offset);
		agnic_pfio_outq_desc_set_pkt_len(&descs[i], len);
		agnic_pfio_outq_desc_set_md_mode(&descs[i], md);
		/* set the L3/L4 information */
		AGNIC_OUT_D_L3_OFFSET_SET(descs[i].cmds[0], AGNIC_IN_D_L3_OFFSET_GET(rx_flags));
		tmp_flags = AGNIC_IN_D_L3_INFO_GET(rx_flags);
		if ((tmp_flags >= AGNIC_IN_D_L3_TYPE_IPV4_NO_OPTS) &&
			(tmp_flags <= AGNIC_IN_D_L3_TYPE_IPV4_OTHER)) { /* IPv4 types */
			tmp_flags = AGNIC_OUT_D_L3_INFO_IPV4;
			if (unlikely(tmp_flags > AGNIC_IN_D_L3_TYPE_IPV4_NO_OPTS))
				tmp_flags |= AGNIC_OUT_D_GEN_L4_CSUM_NOT;
		} else if ((tmp_flags == AGNIC_IN_D_L3_TYPE_IPV6_NO_EXT) ||
			(tmp_flags == AGNIC_IN_D_L3_TYPE_IPV6_EXT)) { /* IPv6 types */
			tmp_flags = AGNIC_OUT_D_L3_INFO_IPV6;
			if (unlikely(tmp_flags > AGNIC_IN_D_L3_TYPE_IPV6_NO_EXT))
				tmp_flags |= AGNIC_OUT_D_GEN_L4_CSUM_NOT;
		} else
			tmp_flags = AGNIC_OUT_D_L3_INFO_OTHER;
		descs[i].cmds[0] |= tmp_flags;
		if (likely(tmp_flags != AGNIC_OUT_D_L3_INFO_OTHER)) {
			AGNIC_OUT_D_IP_HDR_LEN_SET(descs[i].cmds[0], AGNIC_IN_D_IP_HDR_LEN_GET(rx_flags));
			tmp_flags = AGNIC_IN_D_L4_TYPE_GET(rx_flags);
			if (tmp_flags == AGNIC_IN_D_L4_TYPE_TCP) /* TCP */
				tmp_flags = AGNIC_OUT_D_L4_TCP;
			else if (tmp_flags == AGNIC_IN_D_L4_TYPE_UDP) /* UDP */
				tmp_flags = AGNIC_OUT_D_L4_UDP;
			else
				tmp_flags = AGNIC_OUT_D_L4_OTHER | AGNIC_OUT_D_GEN_L4_CSUM_NOT;
			descs[i].cmds[0] |= tmp_flags;
		} else
			descs[i].cmds[0] |= AGNIC_OUT_D_GEN_IPV4_CSUM_DIS;
		shadow_q->buffs_inf[write_ind].cookie = (uintptr_t)buff;
		shadow_q->buffs_inf[write_ind].addr = pa;
		write_ind = next_q_idx(write_ind, shadow_q_size);
		tmp_num++;
	}
	shadow_q->write_ind = write_ind;
	num = tmp_num;

	if (num) {
		u16 desc_idx = 0, cnt = 0, orig_num = num;

		do {
			tx_num = num;
			agnic_pfio_send(tx_lcl_port_desc->pfio, tc, tc_qid, &descs[desc_idx], &tx_num);
			if (num > tx_num)
				cnt++;
			desc_idx += tx_num;
			num -= tx_num;
		} while (num);

		tx_num = orig_num;
		perf_cntrs->tx_cnt += tx_num;
	}
	free_sent_buffers(rx_lcl_port_desc, tx_lcl_port_desc, tc, tc_qid, sh_qid);
/*printf("sent %d pkts from echo\n", tx_num);*/

	return tx_num;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;
	int			 err;
	u16			 num;
	u8			 tc = 0, tc_qid = 0, sh_qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			tc_qid++;
			if (tc_qid == larg->cmn_args.garg->cmn_args.cpus) {
				tc_qid = 0;
				tc++;
				if (tc == PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * larg->cmn_args.garg->cmn_args.cpus) + tc_qid))));
		sh_qid = (tc * larg->cmn_args.garg->cmn_args.cpus) + tc_qid;

		err = do_echo(larg, 0, 0, tc, tc_qid, sh_qid, num);
		if (err < 0) {
			*running = 0;
			return err;
		}
	}

	return 0;
}

static int ctrl_cb(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	agnic_pfio_poll_mgmt(garg->pfio);

	return 0;
}

static void recv_custom_msg_cb(void *arg, u8 code, u64 cookie, void *msg, u16 len)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return;
	}

	if (code != 2) /* Don't print KA (too many) */
		pr_info("AGNIC: got a message: code:%d\n", code);
}

static int app_build_bpool(struct glob_arg *garg,
			struct agnic_pfio *pfio,
			int tc,
			int qid,
			int num_of_buffs,
			int buff_len)
{
	struct agnic_buff_inf *buffs_inf;
	u64 buff_phys_addr;
	int i, err;

	num_of_buffs -= 1;

	pr_debug("Adding (%d Bytes) buffers into BPOOL.\n", buff_len);

	buffs_inf = (struct agnic_buff_inf *)malloc(num_of_buffs * sizeof(struct agnic_buff_inf));
	if (buffs_inf == NULL) {
		pr_err("Failed to allocate buffs_inf\n");
		return -ENOMEM;
	}

	garg->buffs_addr = mv_sys_dma_mem_alloc(buff_len * num_of_buffs, 4);
	if (!garg->buffs_addr) {
		pr_err("failed to allocate AGNIC bpool mem!\n");
		return -ENOMEM;
	}

	buff_phys_addr = mv_sys_dma_mem_virt2phys(garg->buffs_addr);

	for (i = 0; i < num_of_buffs; i++) {
		u16 num = 1;

		buffs_inf[i].addr = buff_phys_addr + (i * buff_len);
		buffs_inf[i].cookie = (u64)garg->buffs_addr + (i * buff_len);

		err = agnic_pfio_inq_put_buffs(pfio, tc, qid, &buffs_inf[i], &num);
		if (err || !num) {
			pr_err("failed to put buff (%d) in AGNIC BPool (%d,%d)!\n", i, tc, qid);
			return -EFAULT;
		}
	}

	return 0;
}

static int extract_pci_bar_inf(dma_addr_t *pa, size_t *size)
{
	FILE *fp;
	char ret_str[1024];
	int ret = -ENOENT;

	/* Open the command for reading. */
	fp = popen("lspci -v", "r");
	if (fp == NULL) {
		pr_err("Failed to run command\n");
		return -EFAULT;
	}

	/* Read the output a line at a time - output it. */
	while (fgets(ret_str, sizeof(ret_str)-1, fp) != NULL) {
		if (!((strstr(ret_str, "Ethernet controller") > 0) && /* this is a NIC */
			(strstr(ret_str, AGNIC_PCI_VENDOR) > 0) && /* This is MV product */
			(strstr(ret_str, AGNIC_PCI_PRODUCT) > 0))) /* this is MV AG-NIC */
			continue;
		while (fgets(ret_str, sizeof(ret_str)-1, fp) != NULL) {
			if (strstr(ret_str, "Memory at") > 0) {
				char *token, *mem_str, *size_str = NULL;

				if (strstr(ret_str, "size=") > 0)
					size_str = strstr(ret_str, "size=");
				mem_str = strstr(ret_str, "Memory at");
				token = strtok(mem_str, " ");
				token = strtok(NULL, " ");
				token = strtok(NULL, " ");
				*pa = strtol(token, NULL, 16);
				if (size_str) {
					int mult = 1;

					token = strtok(size_str, "=");
					token = strtok(NULL, "]");
					if (token[strlen(token)-1] == 'K') {
						token[strlen(token)-1] = '\0';
						mult = 1024;
					} else if (token[strlen(token)-1] == 'M') {
						token[strlen(token)-1] = '\0';
						mult = 1024*1024;
					}
					*size = atoi(token) * mult;
				} else
					/* in case we didn't find the BAR size, let's give it a
					 * default size of 4KB
					 */
					*size = 4096;
				pr_debug("Found PCI based AGNIC BAR0 @ 0x%"PRIx64", size 0x%"PRIx64"\n",
					*pa, *size);
				ret = 0;
				break;
			}
		}
	}

	/* close */
	pclose(fp);

	return ret;
}

static int init_pfio_pci_mem(struct glob_arg *garg, struct agnic_pfio_init_params *params)
{
	int	 fd, err;

	err = extract_pci_bar_inf(&params->pci_bar_inf.pa, &params->pci_bar_inf.size);
	if (err) {
		pr_err("Failed to find any PCI based AGNIC device!\n");
		return err;
	}

	fd = open(AGNIC_PCI_MMAP_FILE_NAME, O_RDWR);
	if (fd < 0) {
		pr_err("UIO file open (%s) = %d (%s)\n",
			AGNIC_PCI_MMAP_FILE_NAME, -errno, strerror(errno));
		return -EFAULT;
	}

	params->pci_bar_inf.va = mmap(NULL,
		params->pci_bar_inf.size,
		PROT_READ | PROT_WRITE,
		MAP_SHARED,
		fd,
		(off_t)params->pci_bar_inf.pa);
	close(fd);
	if (unlikely(params->pci_bar_inf.va == MAP_FAILED)) {
		pr_err("mmap() of 0x%016llx = %d (%s)\n",
			(unsigned long long int)params->pci_bar_inf.pa, -errno, strerror(errno));
		return -EFAULT;
	}
	params->pci_mode = 1;
	garg->pci_bar = params->pci_bar_inf.va;

	pr_debug("AGNIC %d BAR0 address: pa 0x%"PRIx64", va %p\n",
		0, params->pci_bar_inf.pa, params->pci_bar_inf.va);

	return 0;
}

static int init_global(void *arg)
{
	struct glob_arg			*garg = (struct glob_arg *)arg;
	struct agnic_pfio_init_params	 pfio_params;
	int				 err, i, j;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_ECHO_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pfio_params, 0, sizeof(pfio_params));
	if (garg->pci_bar) {
		err = init_pfio_pci_mem(garg, &pfio_params);
		if (err)
			return err;
	}
	pfio_params.num_in_tcs = AGNIC_DEFAULT_NUM_TCS;
	pfio_params.num_out_tcs = AGNIC_DEFAULT_NUM_TCS;
	pfio_params.num_qs_per_tc = garg->cmn_args.cpus;
	pfio_params.in_qs_size = garg->rxq_size;
	pfio_params.out_qs_size = garg->txq_size;
	pfio_params.buff_size = AGNIC_BPOOLS_BUFF_SIZE;
	pfio_params.pkt_offset = garg->cmn_args.pkt_offset;
	if (garg->cmn_args.cpus > 1)
		pfio_params.hash_type = AGNIC_PFIO_HASH_T_2_TUPLE;

	err = agnic_pfio_init(&pfio_params, &garg->pfio);
	if (err) {
		pr_err("Failed to initialize AGNIC Port!\n");
		return err;
	}

	err = agnic_pfio_register_custom_msg_cb(garg->pfio, 0, garg, recv_custom_msg_cb);
	if (err) {
		pr_err("Failed to register AGNIC notification CB!\n");
		return err;
	}

	for (i = 0; i < pfio_params.num_in_tcs; i++)
		for (j = 0; j < pfio_params.num_qs_per_tc; j++) {
			err = app_build_bpool(garg, garg->pfio, i, j, pfio_params.in_qs_size, pfio_params.buff_size);
			if (err) {
				pr_err("Failed to fill AGNIC BPool (%d,%d)!\n", i, j);
				return err;
			}
		}

	usleep(500000); /* Let the FW some time to get ready */
	err = agnic_pfio_set_mtu(garg->pfio, garg->cmn_args.mtu);
	if (err) {
		pr_err("Failed to set MTU for AGNIC!\n");
		return err;
	}
	if (garg->eth_addr[0] || garg->eth_addr[1] || garg->eth_addr[2] ||
		garg->eth_addr[3] || garg->eth_addr[4] || garg->eth_addr[5]) {
		err = agnic_pfio_set_mac_addr(garg->pfio, garg->eth_addr);
		if (err) {
			pr_err("Failed to set MTU for AGNIC!\n");
			return err;
		}
	}
	err = agnic_pfio_enable(garg->pfio);
	if (err) {
		pr_err("Failed to enable AGNIC!\n");
		return err;
	}

	pr_info("done\n");
	return 0;
}

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;

	/* we need first to disable the port */
	agnic_pfio_disable(garg->pfio);
	/* wait and let all traffic sattle-down */
	mdelay(100);

	agnic_pfio_deinit(garg->pfio);

	/* free the pkt buffer memory */
	mv_sys_dma_mem_free(garg->buffs_addr);

	/* This part of Musdk deinit, should be done in the end of all deinit */
	mv_sys_dma_mem_destroy();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	int			 i, j;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	pr_info("Local TH%d initializations ...\n", id);

	larg = (struct local_arg *)malloc(sizeof(struct local_arg));
	if (!larg) {
		pr_err("No mem for local arg obj!\n");
		return -ENOMEM;
	}
	memset(larg, 0, sizeof(struct local_arg));

	larg->cmn_args.id		= id;
	larg->cmn_args.garg		= garg;
	larg->cmn_args.num_ports	= garg->cmn_args.num_ports;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.busy_wait	= garg->cmn_args.busy_wait;
	larg->cmn_args.echo		= garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * larg->cmn_args.id);
	pr_debug("thread %d (cpu %d) mapped to Qs %llx\n",
		larg->cmn_args.id, sched_getcpu(), (unsigned int long long)larg->cmn_args.qs_map);

	/* TODO: for future multiple ports */
	i = 0;
	larg->ports_desc[i].id		= 0;
	larg->ports_desc[i].lcl_id	= larg->cmn_args.id;
	larg->ports_desc[i].pfio_id	= 0;
	larg->ports_desc[i].pfio	= garg->pfio;
	larg->ports_desc[i].num_shadow_qs = AGNIC_DEFAULT_NUM_TCS * larg->cmn_args.garg->cmn_args.cpus;
	larg->ports_desc[i].shadow_q_size = garg->txq_size;

	larg->ports_desc[i].shadow_qs =
		(struct shadow_q *)malloc(larg->ports_desc[i].num_shadow_qs*sizeof(struct shadow_q));
	if (!larg->ports_desc[i].shadow_qs) {
		pr_err("no mem for shadow-Qs obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc[i].shadow_qs, 0, larg->ports_desc[i].num_shadow_qs*sizeof(struct shadow_q));

	for (j = 0; j < larg->ports_desc[i].num_shadow_qs; j++) {
		larg->ports_desc[i].shadow_qs[j].buffs_inf =
			(struct agnic_buff_inf *)
				malloc(larg->ports_desc[i].shadow_q_size*sizeof(struct agnic_buff_inf));
		if (!larg->ports_desc[i].shadow_qs[j].buffs_inf) {
			pr_err("no mem for shadow-Q %d obj!\n", j);
			return -ENOMEM;
		}
	}

	*_larg = larg;
	pr_info("done\n");
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg	*larg = (struct local_arg *)arg;

	if (!larg)
		return;
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK AGNIC packet-echo application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\tTODO\n"
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-b <size>                Burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t-c, --cores <number>     Number of CPUs to use\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t--rxq <size>             Size of rx_queue (default is %d)\n"
	       "\t--txq <size>             Size of tx_queue (default is %d)\n"
	       "\t--mtu <mtu>              Set MTU (default is %d)\n"
	       "\t--addr <eth-addr>        Set Ethernet Address\n"
	       "\t--pci                    AGNIC is in PCI mode\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       PKT_ECHO_APP_MAX_BURST_SIZE,
	       PKT_ECHO_APP_RX_Q_SIZE,
	       PKT_ECHO_APP_TX_Q_SIZE,
	       DEFAULT_MTU
	       );
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = MVAPPS_INVALID_AFFINITY;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->cmn_args.burst = PKT_ECHO_APP_MAX_BURST_SIZE;
	garg->rxq_size = PKT_ECHO_APP_RX_Q_SIZE;
	garg->txq_size = PKT_ECHO_APP_TX_Q_SIZE;
	garg->cmn_args.busy_wait = 0;
	garg->cmn_args.echo = 1;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.pkt_offset = AGNIC_DEFAULT_PKT_OFFS;
	garg->cmn_args.prefetch_shift = PKT_ECHO_APP_PREFETCH_SHIFT;
	garg->cmn_args.ctrl_thresh = PKT_ECHO_APP_STATS_DFLT_THR;

	/* TODO: init hardcoded ports?!?!?! */
	garg->cmn_args.num_ports = 1;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		} else if (strcmp(argv[i], "--mtu") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->cmn_args.mtu = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--addr") == 0) {
			int rv;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			rv = sscanf(argv[i + 1], "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
				    &garg->eth_addr[0], &garg->eth_addr[1], &garg->eth_addr[2],
				&garg->eth_addr[3], &garg->eth_addr[4], &garg->eth_addr[5]);
			if (rv != 6) {
				pr_err("Failed to parse -S parameter (%d)\n", rv);
				return -EINVAL;
			}
			pr_debug("Set eth_addr to: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
				garg->eth_addr[0], garg->eth_addr[1], garg->eth_addr[2],
				garg->eth_addr[3], garg->eth_addr[4], garg->eth_addr[5]);
			break;
			i += 2;
		} else if (strcmp(argv[i], "--rxq") == 0) {
			garg->rxq_size = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--txq") == 0) {
			garg->txq_size = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-b") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->cmn_args.burst = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-c") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->cmn_args.cpus = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-a") == 0) {
			garg->cmn_args.affinity = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-m") == 0) {
			int rv;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			rv = sscanf(argv[i + 1], "%x:%x", (unsigned int *)&garg->cmn_args.qs_map,
				    &garg->cmn_args.qs_map_shift);
			if (rv != 2) {
				pr_err("Failed to parse -m parameter\n");
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--pci") == 0) {
			garg->pci_bar = (void *)MAP_FIXED;
			i += 1;
		} else if (strcmp(argv[i], "--no-echo") == 0) {
			garg->cmn_args.echo = 0;
			i += 1;
		} else if (strcmp(argv[i], "--cli") == 0) {
			garg->cmn_args.cli = 1;
			i += 1;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (garg->cmn_args.burst > PKT_ECHO_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, PKT_ECHO_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cmn_args.cpus > PKT_ECHO_APP_MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cmn_args.cpus, PKT_ECHO_APP_MAX_NUM_CORES);
		return -EINVAL;
	}
	if ((garg->cmn_args.affinity != -1) &&
	    ((garg->cmn_args.cpus + garg->cmn_args.affinity) > MVAPPS_MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cmn_args.cpus, garg->cmn_args.affinity, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}

	if (garg->cmn_args.qs_map &&
	    (PKT_ECHO_APP_MAX_NUM_QS_PER_TC == 1) &&
	    (PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = 1;
	} else if (!garg->cmn_args.qs_map) {
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
	}

	if ((garg->cmn_args.cpus != 1) &&
	    (garg->cmn_args.qs_map & (garg->cmn_args.qs_map << garg->cmn_args.qs_map_shift))) {
		pr_err("Invalid queues-mapping (ovelapping CPUs)!\n");
		return -EINVAL;
	}

	return 0;
}

static void init_app_params(struct mvapp_params *mvapp_params, u64 cores_mask)
{
	memset(mvapp_params, 0, sizeof(struct mvapp_params));
	mvapp_params->use_cli		= garg.cmn_args.cli;
	mvapp_params->num_cores		= garg.cmn_args.cpus;
	mvapp_params->cores_mask	= cores_mask;
	mvapp_params->global_arg	= (void *)&garg;
	mvapp_params->init_global_cb	= init_global;
	mvapp_params->deinit_global_cb	= deinit_global;
	mvapp_params->init_local_cb	= init_local;
	mvapp_params->deinit_local_cb	= deinit_local;
	mvapp_params->main_loop_cb	= main_loop_cb;
	mvapp_params->ctrl_cb		= ctrl_cb;
	mvapp_params->ctrl_cb_threshold	= PKT_ECHO_APP_CTRL_TRD_THRESH;
}


int main(int argc, char *argv[])
{
	struct mvapp_params		 mvapp_params;
	u64				 cores_mask;
	int				 err;

	setbuf(stdout, NULL);

	pr_info("AGNIC pkt-echo is started\n");
	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);

	init_app_params(&mvapp_params, cores_mask);

	return mvapp_go(&mvapp_params);
}
