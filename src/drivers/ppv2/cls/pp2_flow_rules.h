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


/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_H_
#define _PP2_CLS_H_


/******************************************************************************/
/*                                 MACROS                                     */
/******************************************************************************/
#define MVPP2_CLS_LOG_FLOW_LUID_MAX	(20)				/* max lookup ID per flow	*/
#define MVPP2_CLS_FLOW_RULE_MAX		(96)				/* max flow rules		*/
#define MVPP2_CLS_FREE_LOG2OFF		(0)				/* 1st entry is the free index	*/
#define MVPP2_CLS_LOG2OFF_START		(1)				/* 1st entry is the free index	*/
#define MVPP2_CLS_LOG2OFF_TBL_SIZE	(MVPP2_CLS_FLOWS_TBL_SIZE+MVPP2_CLS_FREE_LOG2OFF)
									/* logical to offset table size	*/
#define MVPP2_CLS_FL_RND_SIZE		(25)				/* max rule hits per CLS round	*/
#define MVPP2_CLS_C2_RND_MAX		(16)				/* max C2 per CLS round		*/
#define MVPP2_CLS_C3_RND_MAX		(8)				/* max C3 per CLS round		*/
#define MVPP2_CLS_C4_RND_MAX		(8)				/* max C4 per CLS round		*/
#define MVPP2_CLS_FLOW_RND_MAX		(2)				/* max CLS rounds		*/
#define MVPP2_CLS_UNDF_FL_LOG_ID		(0)				/* CLS undefined logical rule ID*/
#define MVPP2_CLS_FREE_FL_LOG		(0xFFFF)			/* free flow rule value		*/
#define MVPP2_CLS_FL_OFF_INV		(0xFFFF)			/* rule invalid offset		*/
#define MVPP2_PORT_TYPE_INV		(0xFFFF)			/* invalid port type flag	*/
#define MVPP2_PORT_BM_INV			(0xFFFF)			/* invalid port BM flag		*/
#define MVPP2_CLS_DEF_MTU			(0xFFFF)			/* default MTU size		*/
#define MVPP2_CLS_DEF_FLOW_LEN		20

/******************************************************************************/
/*                              ENUMERATIONS                                  */
/******************************************************************************/
enum pp2_cls_rl_mrg_state_t {
	MVPP2_MRG_NOT_NEW			= 0x0000,			/* merged rule no new		*/
	MVPP2_MRG_NEW			= 0x0001,			/* merged rule new		*/
	MVPP2_MRG_NEW_EXISTS		= 0x0002			/* merged rule exists		*/
};

enum pp2_cls_rl_cnt_op_t {
	MVPP2_CNT_INC			= 0x0000,			/* increment engine counter	*/
	MVPP2_CNT_DEC			= 0x0001			/* decrement engine counter	*/
};

/*
 * define the value of RX queue high from:
 * 0: The value of QueueHigh is as defined by the Classifier
 * 1: The value of QueueHigh is as defined by corresponding register
 */
enum pp2_cls_rx_qh_from_t {
	MVPP2_RX_QH_FROM_CLS = 0,
	MVPP2_RX_QH_FROM_REG,
};

/* Define the RX queue high for port, only valid in PP21 */
enum pp2_cls_rx_qh_t {
	MVPP2_GMAC0_RX_QH = 0,
	MVPP2_GMAC1_RX_QH,
	MVPP2_LPBK_RX_QH,
	MVPP2_PMAC_RX_QH,
};
/******************************************************************************/
/*                               STRUCTURES                                   */
/******************************************************************************/
struct pp2_cls_luid_conf_t {
	u8			luid;				/* Lookup ID			*/
};

struct pp2_cls_fl_eng_cnt_t {
	u8			c2;				/* C2 engine count		*/
	u8			c3;				/* C3 engine count		*/
	u8			c4;				/* C4 engine count		*/
};

struct pp2_cls_lkp_dcod_entry_t {
	u8			cpu_q;				/* CPU queue			*/
	u8			way;				/* entry way			*/
	u8			flow_len;			/* flow length			*/
	u16			flow_log_id;			/* flow logical ID		*/
	u16			luid_num;			/* Lookup ID number		*/
	struct pp2_cls_luid_conf_t	luid_list[MVPP2_CLS_LOG_FLOW_LUID_MAX];/* Lookup ID list		*/
};

struct pp2_cls_fl_rule_entry_t {
	u16			fl_log_id;	/* flow logical id              */
	u16			rl_log_id;			/* rule logical id              */
	u16			port_type;			/* port type			*/
	u16			port_bm;			/* port bitmap			*/
	u16			lu_type;	/* lookup type                  */
	u8			enabled;	/* enable flag		*/
	u8			prio;	/* HW priority                  */
	u8			engine;	/* engine to use                */
	u8			seq_ctrl;
	u8			field_id_cnt;			/* field ID count		*/
	u8			field_id[MVPP2_FLOW_FIELD_COUNT_MAX];/* field IDs			*/
};

struct pp2_cls_fl_rule_list_t {
	struct pp2_cls_fl_rule_entry_t	fl[MVPP2_CLS_FLOW_RULE_MAX];	/* flow rules			*/
	u16			fl_len;				/* flow length			*/
};

struct pp2_cls_rl_entry_t {
	u16			rl_log_id;	/* rule logical id		*/
	u16			rl_off;				/* rule offset			*/
	u16			ref_cnt[MVPP2_MAX_NUM_GMACS];	/* rule reference count         */
	u16			port_type;			/* port type			*/
	u16			port_bm;			/* port bitmap			*/
	u16			lu_type;	/* lookup type                  */
	u8			prio;	/* HW priority                  */
	u8			engine;	/* engine to use                */
	u8			enabled;	/* enable flag			*/
	u8			skip;				/* skip flag			*/
	u8			seq_ctrl;
	u8			field_id_cnt;			/* field ID count		*/
	u8			field_id[MVPP2_FLOW_FIELD_COUNT_MAX];/* field IDs			*/
	enum pp2_cls_rl_mrg_state_t	state;				/* rule state			*/
};

struct pp2_cls_fl_t {
	u16			fl_log_id;			/* flow logical id              */
	u16			fl_len;				/* flow length			*/
	struct pp2_cls_rl_entry_t	fl[MVPP2_CLS_FLOW_RULE_MAX];	/* flow rules			*/
	struct pp2_cls_fl_eng_cnt_t	eng_cnt;			/* flow rules engine count	*/
};

/* cli functions */
int pp2_cli_cls_lkp_dcod_entry_set(void *arg, int argc, char *argv[]);
int pp2_cli_cls_lkp_dcod_luid_set(void *arg, int argc, char *argv[]);
int pp2_cli_cls_lkp_dcod_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_lkp_dcod_add(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rule_set(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rule_add(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rule_ena(void *arg, int argc, char *argv[]);
int pp2_cli_cls_lkp_dcod_ena(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rule_dis(void *arg, int argc, char *argv[]);
int pp2_cli_cls_lkp_hits_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_hits_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rls_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_log_rls_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rule_init(void *arg, int argc, char *argv[]);

/******************************************************************************/
/*                                PROTOTYPE                                   */
/******************************************************************************/
/* decode table */
int pp2_cls_lkp_dcod_enable(uintptr_t cpu_slot, u16 flow_log_id);
int pp2_cls_lkp_dcod_set(struct pp2_cls_lkp_dcod_entry_t *lkp_dcod_conf);

/* CLS flow table */
int pp2_cls_fl_rule_add(uintptr_t cpu_slot, struct pp2_cls_fl_rule_list_t *fl_rls);
int pp2_cls_fl_rule_enable(uintptr_t cpu_slot, struct pp2_cls_fl_rule_list_t *fl_rls);
int pp2_cls_fl_rule_disable(uintptr_t cpu_slot, u16 *rl_log_id,
				u16 rl_log_id_len,
				struct pp2_cls_class_port_t *src_port);
int pp2_cls_init(uintptr_t cpu_slot);

#endif /* _PP2_CLS_H_ */
