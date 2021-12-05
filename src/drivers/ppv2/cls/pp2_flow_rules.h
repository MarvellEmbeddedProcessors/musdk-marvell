/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_H_
#define _PP2_CLS_H_

/******************************************************************************/
/*                                 MACROS                                     */
/******************************************************************************/
#define MVPP2_CLS_LOG_FLOW_LUID_MAX	(20)				/* max lookup ID per flow	*/
#define MVPP2_CLS_FLOW_RULE_MAX		(256)				/* max flow rules		*/
#define MVPP2_CLS_FREE_LOG2OFF		(0)				/* 1st entry is the free index	*/
#define MVPP2_CLS_LOG2OFF_START		(1)				/* 1st entry is the free index	*/
#define MVPP2_CLS_LOG2OFF_TBL_SIZE	(MVPP2_CLS_FLOWS_TBL_SIZE + MVPP2_CLS_FREE_LOG2OFF)
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
#define MVPP2_CLS_DEF_WAY		0
#define MVPP2_CLS_DEF_RXQ		0
#define MVPP2_CLS_DEF_SEQ_CTRL		0

/******************************************************************************/
/*                              ENUMERATIONS                                  */
/******************************************************************************/
enum pp2_cls_rl_udf7_t {
	MVPP2_CLS_KERNEL_UDF7 = 1,
	MVPP2_CLS_MUSDK_NIC_UDF7 = 1,
	MVPP2_CLS_MUSDK_LOG_UDF7 = 2,
};

enum pp2_cls_rl_prio_t {
	MVPP2_CLS_MUSDK_CLS_PRIO = 0,
	MVPP2_CLS_MUSDK_DSCP_PRIO,
	MVPP2_CLS_MUSDK_VLAN_PRIO,
	MVPP2_CLS_MUSDK_DEF_PRIO,
	MVPP2_CLS_MUSDK_HASH_PRIO,
	MVPP2_CLS_KERNEL_DSCP_PRIO,
	MVPP2_CLS_KERNEL_VLAN_PRIO,
	MVPP2_CLS_KERNEL_DEF_PRIO,
	MVPP2_CLS_KERNEL_HASH_PRIO
};

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
	u8			udf7;
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
	u16			ref_cnt[PP2_NUM_PORTS];	/* rule reference count         */
	u16			port_type;			/* port type			*/
	u16			port_bm;			/* port bitmap			*/
	u16			lu_type;	/* lookup type                  */
	u8			prio;	/* HW priority                  */
	u8			engine;	/* engine to use                */
	u8			enabled;	/* enable flag			*/
	u8			skip;				/* skip flag			*/
	u8			udf7;
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
int pp2_cli_cls_flow_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rls_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_log_rls_dump(void *arg, int argc, char *argv[]);
int pp2_cli_cls_fl_rule_init(void *arg, int argc, char *argv[]);
int pp2_cls_print_rxq_counters(void *arg, int argc, char *argv[]);
int pp2_cli_cls_set_rss_mode(void *arg, int argc, char *argv[]);

/******************************************************************************/
/*                                PROTOTYPE                                   */
/******************************************************************************/
/* decode table */
int pp2_cls_lkp_dcod_disable(struct pp2_inst *inst, u16 fl_log_id);
int pp2_cls_lkp_dcod_set_and_disable(struct pp2_inst *inst,  u16 fl_log_id);
int pp2_cls_lkp_dcod_enable(struct pp2_inst *inst, u16 flow_log_id);
int pp2_cls_lkp_dcod_set(struct pp2_inst *inst,
			 struct pp2_cls_lkp_dcod_entry_t *lkp_dcod_conf);

/* CLS flow table */
int pp2_cls_fl_rule_add(struct pp2_inst *inst, struct pp2_cls_fl_rule_list_t *fl_rls);
int pp2_cls_fl_rule_enable(struct pp2_inst *inst, struct pp2_cls_fl_rule_list_t *fl_rls);
int pp2_cls_fl_rule_disable(struct pp2_inst *inst, u16 *rl_log_id,
			    u16 rl_log_id_len,
			    u32 port_id);
int pp2_cls_rss_mode_flows_set(struct pp2_port *port, int rss_mode);
int pp2_cls_udf_field_add(struct pp2_inst *inst, u8 udf_num, u8 offset, u8 size);
int pp2_cls_udf_field_remove(struct pp2_inst *inst, u8 udf_num);
int pp2_cls_rule_disable(struct pp2_port *port, struct pp2_cls_fl_rule_entry_t *fl);
int pp2_cls_init(struct pp2_inst *inst);

#endif /* _PP2_CLS_H_ */
