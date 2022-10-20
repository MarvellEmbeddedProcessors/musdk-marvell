/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _MV_NSS_DP_H_
#define _MV_NSS_DP_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __MV_NSS_OS_BASIC_TYPES_UNDEFINED__

typedef signed char        int8_t;
typedef unsigned char      uint8_t;
typedef short              int16_t;
typedef unsigned short     uint16_t;
typedef int                int32_t;
typedef unsigned int       uint32_t;
typedef signed long long   int64_t;
typedef unsigned long long uint64_t;
typedef uint32_t           size_t;
typedef int32_t            ssize_t;

#ifndef NULL
#define NULL (0)
#endif

#endif  /* __MV_NSS_OS_BASIC_TYPES_UNDEFINED__ */

/* Flow parameters bit-masks */
#define MV_NSS_DP_PORT_DST_BIT          (0)
#define MV_NSS_DP_PORT_SRC_BIT          (1)
#define MV_NSS_DP_L2_ADDR_DST_BIT       (2)
#define MV_NSS_DP_L2_ADDR_SRC_BIT       (3)
#define MV_NSS_DP_ETH_TYPE_BIT          (4)
#define MV_NSS_DP_LLC_SSAP_BIT          (5)
#define MV_NSS_DP_LLC_DSAP_BIT          (6)
#define MV_NSS_DP_SNAP_BIT              (7)
#define MV_NSS_DP_IP_DST_BIT            (8)
#define MV_NSS_DP_IP_SRC_BIT            (9)
#define MV_NSS_DP_L4_PORT_DST_BIT       (10)
#define MV_NSS_DP_L4_PORT_SRC_BIT       (11)
#define MV_NSS_DP_L4_PROTO_BIT          (12)
#define MV_NSS_DP_FLBL_BIT              (13)
#define MV_NSS_DP_PRIO_BIT              (14)
#define MV_NSS_DP_OPAQUE_BIT            (15)
#define MV_NSS_DP_PRIO_ACTION_BIT       (16)

#define MV_NSS_DP_PORT_DST              (1UL << MV_NSS_DP_PORT_DST_BIT)
#define MV_NSS_DP_PORT_SRC              (1UL << MV_NSS_DP_PORT_SRC_BIT)
#define MV_NSS_DP_L2_ADDR_DST           (1UL << MV_NSS_DP_L2_ADDR_DST_BIT)
#define MV_NSS_DP_L2_ADDR_SRC           (1UL << MV_NSS_DP_L2_ADDR_SRC_BIT)
#define MV_NSS_DP_ETH_TYPE              (1UL << MV_NSS_DP_ETH_TYPE_BIT)
#define MV_NSS_DP_LLC_SSAP              (1UL << MV_NSS_DP_LLC_SSAP_BIT)
#define MV_NSS_DP_LLC_DSAP              (1UL << MV_NSS_DP_LLC_DSAP_BIT)
#define MV_NSS_DP_SNAP                  (1UL << MV_NSS_DP_SNAP_BIT)
#define MV_NSS_DP_IP_DST                (1UL << MV_NSS_DP_IP_DST_BIT)
#define MV_NSS_DP_IP_SRC                (1UL << MV_NSS_DP_IP_SRC_BIT)
#define MV_NSS_DP_FLBL                  (1UL << MV_NSS_DP_FLBL_BIT)
#define MV_NSS_DP_L4_PORT_DST           (1UL << MV_NSS_DP_L4_PORT_DST_BIT)
#define MV_NSS_DP_L4_PORT_SRC           (1UL << MV_NSS_DP_L4_PORT_SRC_BIT)
#define MV_NSS_DP_L4_PROTO              (1UL << MV_NSS_DP_L4_PROTO_BIT)
#define MV_NSS_DP_PRIO                  (1UL << MV_NSS_DP_PRIO_BIT)
#define MV_NSS_DP_OPAQUE                (1UL << MV_NSS_DP_OPAQUE_BIT)
#define MV_NSS_DP_PRIO_ACTION           (1UL << MV_NSS_DP_PRIO_ACTION_BIT)

/* Number of intervals of hash value in hash profile */
#define MV_NSS_DP_HASH_KNEES            (3)

/* Unknown hash profile ID */
#define MV_NSS_DP_HASH_PROF_ID_NONE     (0xFFFFFFFF)

/* Number of supported queuing priorities */
#define MV_NSS_DP_PRIO_NUM              (1 << 4)

/* Number of supported scheduling priorities */
#define MV_NSS_DP_SCHED_PRIO_NUM        (1 << 3)

/* Unknown or any VLAN ID */
#define MV_NSS_DP_VLAN_ANY              (0xFFF)

/* Maximum number of characters in the system message including the trailing '\x0' */
#define MV_NSS_DP_SYSTEM_MSG_SIZE       (0x20)

/* Unknown or invalid port ID */
#define MV_NSS_DP_PORT_ID_NONE           (0x00)

/* All virtual ports indication */
#define MV_NSS_DP_PORT_ALL               (0xFF)

/* Unknown or invalid flow ID */
#define MV_NSS_DP_FLOW_ID_NONE           (0xFFFFFFFE)

/* All flow IDs indication */
#define MV_NSS_DP_FLOW_ID_ALL            (0xFFFFFFFF)

/* DTLS version 1.0 */
#define MV_NSS_DP_DTLS_VER_1_0           (10)

/* DTLS version 1.2 */
#define MV_NSS_DP_DTLS_VER_1_2           (12)

/* Ethernet link physical duplex modes */
#define MV_NSS_DP_DUPLEX_HALF             0x00
#define MV_NSS_DP_DUPLEX_FULL             0x01
#define MV_NSS_DP_DUPLEX_UNKNOWN          0xFF

/* Ethernet link physical speeds */
#define MV_NSS_DP_SPEED_10                10
#define MV_NSS_DP_SPEED_100               100
#define MV_NSS_DP_SPEED_1000              1000
#define MV_NSS_DP_SPEED_2500              2500
#define MV_NSS_DP_SPEED_10000             10000
#define MV_NSS_DP_SPEED_UNKNOWN           0xFFFF

/* Unknown or invalid client ID */
#define MV_NSS_DP_CLIENT_ID_NONE          (0x0000)

/* Base type used for bitwise operations */
typedef uint32_t mv_nss_dp_bitmask_t;

/* Virtual port ID */
typedef uint8_t mv_nss_dp_port_id_t;

/* Virtual port index per type */
typedef uint8_t mv_nss_dp_port_type_idx_t;

/* Packet flow ID */
typedef uint32_t mv_nss_dp_flow_id_t;

/* Hash profile ID */
typedef uint32_t mv_nss_dp_hash_profile_id_t;

/* Status codes */
typedef enum {
	MV_NSS_DP_OK = 0x00,           /* Operation succeeded */
	MV_NSS_DP_FAILED,              /* Operation failed */
	MV_NSS_DP_TOO_MANY_REQUESTS,   /* Number of concurrent requests exceeded the limit */
	MV_NSS_DP_END_OF_LIST,         /* No more records are available */
	MV_NSS_DP_INVALID_PARAM,       /* One or more parameters are invalid or out of range */
	MV_NSS_DP_OUT_OF_RESOURCES,    /* System resources utilization exceeded the limit */
	MV_NSS_DP_ITEM_NOT_FOUND,      /* Requested item does not exist */
	MV_NSS_DP_API_EXEC_TIMEOUT,    /* Request execution exceeded timeout */
	MV_NSS_DP_SHUTDOWN,            /* Can't execute a request due to shutdown in progress */
	MV_NSS_DP_NOT_SUPPORTED        /* Operation or feature is not supported */
} mv_nss_dp_status_t;


/*
 * typedef: enum mv_nss_dp_event_param_t
 *
 * Description:
 *       Types of events generated by NSS.
 *
 * Enumerations:
 *        MV_NSS_DP_EVT_NOTIFY_CODE            - System notification code.
 *        MV_NSS_DP_EVT_NOTIFY_MSG             - System notification with a message.
 *        MV_NSS_DP_EVT_INIT                   - Corresponds to mv_nss_dp_init request.
 *        MV_NSS_DP_EVT_SHUTDOWN               - Corresponds to mv_nss_dp_shutdown request.
 *        MV_NSS_DP_EVT_HASH_PROFILE_SET       - Corresponds to mv_nss_dp_hash_profile_set request.
 *        MV_NSS_DP_EVT_HASH_PROFILE_DELETE    - Corresponds to mv_nss_dp_hash_profile_delete request.
 *        MV_NSS_DP_EVT_DTLS_SET               - Corresponds to mv_nss_dp_dtls_set request.
 *        MV_NSS_DP_EVT_DTLS_DELETE            - Corresponds to mv_nss_dp_dtls_delete request.
 *        MV_NSS_DP_EVT_DTLS_GET               - Corresponds to mv_nss_dp_dtls_get request.
 *        MV_NSS_DP_EVT_PORT_SET               - Corresponds to mv_nss_dp_set_port request.
 *        MV_NSS_DP_EVT_PORT_DELETE            - Corresponds to mv_nss_dp_delete_port request.
 *        MV_NSS_DP_EVT_PORT_GET               - Corresponds to mv_nss_dp_get_port request.
 *        MV_NSS_DP_EVT_PORT_STATS_GET         - Corresponds to mv_nss_dp_port_stats_get or
 *                                               mv_nss_dp_port_bulk_stats_get request.
 *        MV_NSS_DP_EVT_PORT_STATS_RESET       - Corresponds to mv_nss_dp_port_stats_reset request.
 *        MV_NSS_DP_EVT_PORT_DST_SET           - Corresponds to mv_nss_dp_port_dst_set request.
 *        MV_NSS_DP_EVT_VLAN_CFG_SET           - Corresponds to mv_nss_dp_vlan_cfg_set request.
 *        MV_NSS_DP_EVT_VLAN_CFG_DELETE        - Corresponds to mv_nss_dp_vlan_cfg_delete request.
 *        MV_NSS_DP_EVT_VLAN_CFG_GET           - Corresponds to mv_nss_dp_vlan_cfg_get request.
 *        MV_NSS_DP_EVT_MC_BRIDGED_CFG_SET     - Corresponds to mv_nss_dp_mc_bridged_cfg_set request.
 *        MV_NSS_DP_EVT_MC_BRIDGED_CFG_DELETE  - Corresponds to mv_nss_dp_mc_bridged_cfg_delete request.
 *        MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET     - Corresponds to mv_nss_dp_mc_bridged_cfg_get request.
 *        MV_NSS_DP_EVT_MC_TUNNELED_CFG_SET    - Corresponds to mv_nss_dp_mc_tunneled_cfg_set request.
 *        MV_NSS_DP_EVT_MC_TUNNELED_CFG_DELETE - Corresponds to mv_nss_dp_mc_tunneled_cfg_delete request.
 *        MV_NSS_DP_EVT_MC_TUNNELED_CFG_GET    - Corresponds to mv_nss_dp_mc_tunneled_cfg_get request.
 *        MV_NSS_DP_EVT_FLOW_SET               - Corresponds to mv_nss_dp_flow_set request.
 *        MV_NSS_DP_EVT_FLOW_DELETE            - Corresponds to mv_nss_dp_flow_delete or
 *                                               mv_nss_dp_flow_delete_all request.
 *        MV_NSS_DP_EVT_FLOW_GET               - Corresponds to mv_nss_dp_flow_get request.
 *        MV_NSS_DP_EVT_FLOW_STATUS_SET        - Corresponds to mv_nss_dp_flow_status_set request.
 *        MV_NSS_DP_EVT_FLOW_STATUS_GET        - Corresponds to mv_nss_dp_flow_status_get request.
 *        MV_NSS_DP_EVT_FLOW_STATS_GET         - Corresponds to mv_nss_dp_flow_stats_get or mv_nss_dp_flow_bulk_stats_get request.
 *        MV_NSS_DP_EVT_FLOW_STATS_GET         - Corresponds to mv_nss_dp_flow_stats_get or
 *                                               mv_nss_dp_flow_bulk_stats_get request.
 *        MV_NSS_DP_EVT_FLOW_COUNT_GET         - Corresponds to mv_nss_dp_flow_count_get request.
 *        MV_NSS_DP_EVT_INGRESS_Q_CFG_SET      - Corresponds to mv_nss_dp_ingress_queue_cfg_set request.
 *        MV_NSS_DP_EVT_INGRESS_Q_CFG_GET      - Corresponds to mv_nss_dp_ingress_queue_cfg_get request.
 *        MV_NSS_DP_EVT_INGRESS_PRIO_CFG_SET   - Corresponds to mv_nss_dp_ingress_prio_cfg_set request.
 *        MV_NSS_DP_EVT_INGRESS_PRIO_CFG_GET   - Corresponds to mv_nss_dp_ingress_prio_cfg_get request.
 *        MV_NSS_DP_EVT_INGRESS_QOS_POLICY_SET - Corresponds to mv_nss_dp_ingress_qos_policy_set request.
 *        MV_NSS_DP_EVT_INGRESS_QOS_POLICY_GET - Corresponds to mv_nss_dp_ingress_qos_policy_get request.
 *        MV_NSS_DP_EVT_EGRESS_Q_CFG_SET       - Corresponds to mv_nss_dp_egress_queue_cfg_set request.
 *        MV_NSS_DP_EVT_EGRESS_Q_CFG_GET       - Corresponds to mv_nss_dp_egress_queue_cfg_get request.
 *        MV_NSS_DP_EVT_EGRESS_PRIO_CFG_SET    - Corresponds to mv_nss_dp_egress_prio_cfg_set request.
 *        MV_NSS_DP_EVT_EGRESS_PRIO_CFG_GET    - Corresponds to mv_nss_dp_egress_prio_cfg_get request.
 *        MV_NSS_DP_EVT_EGRESS_QOS_POLICY_SET  - Corresponds to mv_nss_dp_egress_qos_policy_set request.
 *        MV_NSS_DP_EVT_EGRESS_QOS_POLICY_GET  - Corresponds to mv_nss_dp_egress_qos_policy_get request.
 *        MV_NSS_DP_EVT_CLIENT_SET             - Corresponds to mv_nss_dp_client_set request.
 *        MV_NSS_DP_EVT_CLIENT_DELETE          - Corresponds to mv_nss_dp_client_delete request.
 *        MV_NSS_DP_EVT_CLIENT_GET             - Corresponds to mv_nss_dp_client_get request.
 *        MV_NSS_DP_EVT_HASH_PROFILE_GET       - Corresponds to mv_nss_dp_hash_profile_get request.
 *        MV_NSS_DP_EVT_ETH_LINK_STATE_GET     - Corresponds to mv_nss_dp_eth_link_state_get request.
 *        MV_NSS_DP_EVT_INGRESS_Q_STATS_GET    - Corresponds to mv_nss_dp_ingress_queue_stats_get request.
 *        MV_NSS_DP_EVT_INGRESS_Q_STATS_RESET  - Corresponds to mv_nss_dp_ingress_queue_stats_reset request.
 *        MV_NSS_DP_EVT_EGRESS_Q_STATS_GET     - Corresponds to mv_nss_dp_egress_queue_stats_get request.
 *        MV_NSS_DP_EVT_EGRESS_Q_STATS_RESET   - Corresponds to mv_nss_dp_egress_queue_stats_reset request.
 *        MV_NSS_DP_EVT_BYPASS_STATE_SET       - Corresponds to mv_nss_dp_bypass_state_set request.
 *        MV_NSS_DP_EVT_BYPASS_STATE_GET       - Corresponds to mv_nss_dp_bypass_state_get request.
 *        MV_NSS_DP_EVT_VXLAN_VNI_CFG_SET      - Corresponds to mv_nss_dp_vxlan_vni_cfg_set request.
 *        MV_NSS_DP_EVT_VXLAN_VNI_CFG_GET      - Corresponds to mv_nss_dp_vxlan_vni_cfg_get request.
 *        MV_NSS_DP_EVT_VXLAN_VNI_CFG_DELETE   - Corresponds to mv_nss_dp_vxlan_vni_cfg_delete request.
 */
typedef enum {
	MV_NSS_DP_EVT_NOTIFY_CODE,
	MV_NSS_DP_EVT_NOTIFY_MSG,
	MV_NSS_DP_EVT_INIT,
	MV_NSS_DP_EVT_SHUTDOWN,
	MV_NSS_DP_EVT_HASH_PROFILE_SET,
	MV_NSS_DP_EVT_HASH_PROFILE_DELETE,
	MV_NSS_DP_EVT_DTLS_SET,
	MV_NSS_DP_EVT_DTLS_DELETE,
	MV_NSS_DP_EVT_DTLS_GET,
	MV_NSS_DP_EVT_PORT_SET,
	MV_NSS_DP_EVT_PORT_DELETE,
	MV_NSS_DP_EVT_PORT_GET,
	MV_NSS_DP_EVT_PORT_STATS_GET,
	MV_NSS_DP_EVT_PORT_STATS_RESET,
	MV_NSS_DP_EVT_PORT_DST_SET,
	MV_NSS_DP_EVT_VLAN_CFG_SET,
	MV_NSS_DP_EVT_VLAN_CFG_DELETE,
	MV_NSS_DP_EVT_VLAN_CFG_GET,
	MV_NSS_DP_EVT_MC_BRIDGED_CFG_SET,
	MV_NSS_DP_EVT_MC_BRIDGED_CFG_DELETE,
	MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET,
	MV_NSS_DP_EVT_MC_TUNNELED_CFG_SET,
	MV_NSS_DP_EVT_MC_TUNNELED_CFG_DELETE,
	MV_NSS_DP_EVT_MC_TUNNELED_CFG_GET,
	MV_NSS_DP_EVT_FLOW_SET,
	MV_NSS_DP_EVT_FLOW_DELETE,
	MV_NSS_DP_EVT_FLOW_GET,
	MV_NSS_DP_EVT_FLOW_STATUS_SET,
	MV_NSS_DP_EVT_FLOW_STATUS_GET,
	MV_NSS_DP_EVT_FLOW_STATS_GET,
	MV_NSS_DP_EVT_FLOW_STATS_RESET,
	MV_NSS_DP_EVT_FLOW_COUNT_GET,
	MV_NSS_DP_EVT_INGRESS_Q_CFG_SET,
	MV_NSS_DP_EVT_INGRESS_Q_CFG_GET,
	MV_NSS_DP_EVT_INGRESS_PRIO_CFG_SET,
	MV_NSS_DP_EVT_INGRESS_PRIO_CFG_GET,
	MV_NSS_DP_EVT_INGRESS_QOS_POLICY_SET,
	MV_NSS_DP_EVT_INGRESS_QOS_POLICY_GET,
	MV_NSS_DP_EVT_EGRESS_Q_CFG_SET,
	MV_NSS_DP_EVT_EGRESS_Q_CFG_GET,
	MV_NSS_DP_EVT_EGRESS_PRIO_CFG_SET,
	MV_NSS_DP_EVT_EGRESS_PRIO_CFG_GET,
	MV_NSS_DP_EVT_EGRESS_QOS_POLICY_SET,
	MV_NSS_DP_EVT_EGRESS_QOS_POLICY_GET,
	MV_NSS_DP_EVT_CLIENT_SET,
	MV_NSS_DP_EVT_CLIENT_DELETE,
	MV_NSS_DP_EVT_CLIENT_GET,
	MV_NSS_DP_EVT_HASH_PROFILE_GET,
	MV_NSS_DP_EVT_ETH_LINK_STATE_GET,
	MV_NSS_DP_EVT_INGRESS_Q_STATS_GET,
	MV_NSS_DP_EVT_INGRESS_Q_STATS_RESET,
	MV_NSS_DP_EVT_EGRESS_Q_STATS_GET,
	MV_NSS_DP_EVT_EGRESS_Q_STATS_RESET,
	MV_NSS_DP_EVT_BYPASS_STATE_SET,
	MV_NSS_DP_EVT_BYPASS_STATE_GET,
	MV_NSS_DP_EVT_VXLAN_VNI_CFG_SET,
	MV_NSS_DP_EVT_VXLAN_VNI_CFG_GET,
	MV_NSS_DP_EVT_VXLAN_VNI_CFG_DELETE,
} mv_nss_dp_event_type_t;

/*
 * typedef: enum mv_nss_dp_notifiy_t
 *
 * Description:
 *       Types of notifications generated by NSS.
 */
typedef enum {
	MV_NSS_DP_NOTIFY_NONE = 0x00,
	MV_NSS_DP_NOTIFY_DTLS_SEQID_ROLLOVER, /* DTLS sequence ID roll over 0 */
	MV_NSS_DP_NOTIFY_DTLS_ANTI_REPLAY,    /* DTLS anti replay */
	MV_NSS_DP_NOTIFY_DOWN,                /* NSS is down */
	MV_NSS_DP_NOTIFY_NOT_SUPPORTED
} mv_nss_dp_notifiy_t;

/*
 * typedef: enum mv_nss_dp_port_type_t
 *
 * Description:
 *       Virtual port type.
 *
 * Enumerations:
 *       MV_NSS_DP_PORT_NONE    - Invalid virtual port type.
 *       MV_NSS_DP_PORT_ETH     - Ethernet virtual port.
 *       MV_NSS_DP_PORT_ETH_LAG - LAG Ethernet virtual port.
 *       MV_NSS_DP_PORT_CPU     - CPU virtual port.
 *       MV_NSS_DP_PORT_CAPWAP  - CAPWAP tunnel endpoint virtual port.
 */
typedef enum {
	 MV_NSS_DP_PORT_NONE = 0,
	 MV_NSS_DP_PORT_ETH,
	 MV_NSS_DP_PORT_ETH_LAG,
	 MV_NSS_DP_PORT_CPU,
	 MV_NSS_DP_PORT_CAPWAP,
	 MV_NSS_DP_PORT_VXLAN
} mv_nss_dp_port_type_t;

/*
 * typedef: enum mv_nss_dp_port_state_t
 *
 * Description:
 *       Virtual port receive/transmit operational state.
 *
 * Enumerations:
 *       MV_NSS_DP_PORT_STATE_DOWN - Virtual port is inactive.
 *       MV_NSS_DP_PORT_STATE_UP   - Virtual port is active.
 */
typedef enum {
	MV_NSS_DP_PORT_STATE_DOWN = 0x00,
	MV_NSS_DP_PORT_STATE_UP,
} mv_nss_dp_port_state_t;

/*
 * typedef: enum mv_nss_dp_l4_proto_t
 *
 * Description:
 *       Layer 4 protocol types.
 *
 * Enumerations:
 *       MV_NSS_DP_PROTO_UDP      - UDP protocol.
 *       MV_NSS_DP_PROTO_UDP_LITE - UDP-Lite protocol.
 *       MV_NSS_DP_PROTO_TCP      - TCP protocol.
 */
typedef enum {
	MV_NSS_DP_PROTO_UDP,
	MV_NSS_DP_PROTO_UDP_LITE,
	MV_NSS_DP_PROTO_TCP
} mv_nss_dp_l4_proto_t;

/*
 * typedef: enum mv_nss_dp_vlan_policy_t
 *
 * Description:
 *       VLAN admission policy.
 *
 * Enumerations:
 *       MV_NSS_DP_VLAN_TAG_ALLOWED     - Both 802.1Q tagged and untagged frames allowed.
 *       MV_NSS_DP_VLAN_TAG_NOT_ALLOWED - Untagged frames only allowed.
 *       MV_NSS_DP_VLAN_TAG_REQUIRED    - 802.1Q tagged frames only allowed.
 */
typedef enum {
	MV_NSS_DP_VLAN_TAG_ALLOWED,
	MV_NSS_DP_VLAN_TAG_NOT_ALLOWED,
	MV_NSS_DP_VLAN_TAG_REQUIRED
} mv_nss_dp_vlan_policy_t;

/*
 * typedef: enum mv_nss_dp_qos_policy_type_t
 *
 * Description:
 *       QoS policy type.
 *
 * Enumerations:
 *       MV_NSS_DP_QOS_POLICY_L2_L3 - Precedence is L2 QoS, then L3 QoS, then default priority.
 *       MV_NSS_DP_QOS_POLICY_L3_L2 - Precedence is L2 QoS, then L3 QoS, then default priority.
 *       MV_NSS_DP_QOS_POLICY_L2    - Precedence is L2 QoS, then default priority.
 *       MV_NSS_DP_QOS_POLICY_L3    - Precedence is L3 QoS, then default priority.
 */
typedef enum {
	MV_NSS_DP_INGRESS_QOS_POLICY_L2_L3,
	MV_NSS_DP_INGRESS_QOS_POLICY_L3_L2,
	MV_NSS_DP_INGRESS_QOS_POLICY_L2,
	MV_NSS_DP_INGRESS_QOS_POLICY_L3
} mv_nss_dp_qos_policy_type_t;


/*
 * typedef: enum mv_nss_dp_prio_action_t
 *
 * Description:
 *       Priority action for calculating per flow queying priority.
 *
 * Enumerations:
 *       MV_DP_NSS_PRIO_ACTION_NONE     - Priority value is default.
 *       MV_DP_NSS_PRIO_ACTION_FIXED    - Priority value is fixed.
 *       MV_DP_NSS_PRIO_ACTION_QOS_PLCY - Priority value is based on ingress
 *                                        QoS policy.
 */
typedef enum {
	MV_DP_NSS_PRIO_ACTION_NONE,
	MV_DP_NSS_PRIO_ACTION_FIXED,
	MV_DP_NSS_PRIO_ACTION_QOS_PLCY,
} mv_nss_dp_prio_action_t;

/*
 * typedef: enum mv_nss_dp_dtls_mode_t
 *
 * Description:
 *       DTLS cipher suite.
 *
 * Enumerations:
 *       MV_NSS_DP_AES_128_CBC_HMAC_SHA_1   - AES with key length 128 using HMAC SHA-1.
 *       MV_NSS_DP_AES_256_CBC_HMAC_SHA_1   - AES with key length 128 using HMAC SHA-1.
 *       MV_NSS_DP_AES_256_CBC_HMAC_SHA_256 - AES with key length 256 using HMAC SHA-256.
 *       MV_NSS_DP_AES_128_GCM              - AES with key 128 in GCM mode.
 *       MV_NSS_DP_AES_256_GCM              - AES with key 256 in GCM mode.
 */
typedef enum {
	MV_NSS_DP_AES_128_CBC_HMAC_SHA_1,
	MV_NSS_DP_AES_256_CBC_HMAC_SHA_1,
	MV_NSS_DP_AES_256_CBC_HMAC_SHA_256,
	MV_NSS_DP_AES_128_GCM,
	MV_NSS_DP_AES_256_GCM

} mv_nss_dp_dtls_mode_t;

/*
 * typedef: enum mv_nss_dp_entity_type_t
 *
 * Description:
 *       NSS entity types.
 *
 * Enumerations:
 *       MV_NSS_DP_VIRT_PORT        - Virtual port, physical and non-physical.
 *       MV_NSS_DP_VLAN             - VLAN configuration record.
 *       MV_NSS_DP_CLIENT           - Client record.
 *       MV_NSS_DP_INGRESS_QOS_PLCY - Ingress QoS policy record.
 *       MV_NSS_DP_EGRESS_QOS_PLCY  - Egress QoS policy record.
 *       MV_NSS_DP_HASH_PROFILE     - Hash profile.
 *       MV_NSS_DP_TUNNEL_CAPWAP    - CAPWAP tunnel instance.
 *       MV_NSS_DP_DTLS             - DTLS context.
 *       MV_NSS_DP_L2_FLOW          - Layer 2 flow.
 *       MV_NSS_DP_L4_FLOW          - Layer 4 flow.
 *       MV_NSS_DP_INGRESS_QUEUE    - Ingress virtual queue.
 *       MV_NSS_DP_EGRESS_QUEUE     - Egress virtual queue.
 *       MV_NSS_DP_TUNNEL_VXLAN     - VXLAN tunnel instance.
 */
typedef enum {
	MV_NSS_DP_VIRT_PORT = 0x00,
	MV_NSS_DP_VLAN,
	MV_NSS_DP_CLIENT,
	MV_NSS_DP_INGRESS_QOS_PLCY,
	MV_NSS_DP_EGRESS_QOS_PLCY,
	MV_NSS_DP_HASH_PROFILE,
	MV_NSS_DP_TUNNEL_CAPWAP,
	MV_NSS_DP_DTLS,
	MV_NSS_DP_L2_FLOW,
	MV_NSS_DP_L4_FLOW,
	MV_NSS_DP_INGRESS_QUEUE,
	MV_NSS_DP_EGRESS_QUEUE,
	MV_NSS_DP_TUNNEL_VXLAN
} mv_nss_dp_entity_type_t;

/*
 * typedef: enum mv_nss_dp_state_t
 *
 * Description:
 *       Network sub-system state.
 *
 * Enumerations:
 *       MV_NSS_DP_STATE_NONINIT  - Network sub-system is uninitialized.
 *       MV_NSS_DP_STATE_LIVE     - Network sub-system is initialized and running successfully.
 *       MV_NSS_DP_STATE_INACTIVE - Network sub-system is non-functioning and needs to be restarted.
 */
typedef enum {
	MV_NSS_DP_STATE_NONINIT,
	MV_NSS_DP_STATE_LIVE,
	MV_NSS_DP_STATE_INACTIVE
} mv_nss_dp_state_t;

/*
 * typedef: struct mv_nss_dp_l2_addr_t
 *
 * Description:
 *       Layer 2 address.
 *
 * Fields:
 *       addr - Layer 2 address bytes.
 */
typedef struct {
	uint8_t addr[6];
} mv_nss_dp_l2_addr_t;

/*
 * typedef: struct mv_nss_dp_ip_addr_t
 *
 * Description:
 *       IPv4 or IPv6 address.
 *
 * Fields:
 *       ip  - IP address:
 *              word[0] - IPv4 address.
 *              words[0-3] - IPv6 address.
 *       ver - IP version nunber (4 or 6).
 */
typedef struct {
	uint32_t ip[4];
	uint8_t  ver;
} mv_nss_dp_ip_addr_t;

/*
 * typedef: struct mv_nss_dp_client_t
 *
 * Description:
 *       Client station record.
 *
 * Fields:
 *       client_id        - Unique client ID for an existing client or MV_NSS_DP_CLIENT_ID_NONE for new client.
 *                          The client IDs for new clients are generated by packet processor.
 *       cookie           - application cookie.
 *       l2_addr          - Unicast Layer 2 address.
 *       type             - port type (ETH/CAPWAP/VXLAN).
 *       u                - port id or index specific to the port type.
 *                          port_id - relevant if port-type is CAPWAP/VXLAN
 *                          index - relevant if port-type is ETH/ETH_LAG/CPU/etc.
 *       prio_policy      - Ingress priority Policy.
 *       vlan             - Ingress bridged VLAN ID allowed for this client or
 *                          MV_NSS_DP_VLAN_ANY if any VLAN ID is allowed.
 *       qcf_needed       - Set if QCF Needed for this client.
 *       dscp_needed      - Set if DSCP update needed for this client.
 *       bssid            - 4 LSb of client BSSID.
 *       radio_id         - Client radio ID.
 *       is_my_mac        - Set if it's My Mac.
 */
typedef struct {
	uint16_t            client_id;
	uint16_t            cookie;
	mv_nss_dp_l2_addr_t l2_addr;
	mv_nss_dp_port_type_t  type;
	union {
		mv_nss_dp_port_id_t    port_id;
		mv_nss_dp_port_type_idx_t index;
	} u;

	uint32_t            prio_policy : 8;
	uint32_t            vlan : 12;
	uint32_t            qcf_needed : 1;
	uint32_t            dscp_needed : 1;
	uint32_t            bssid : 4;
	uint32_t            radio_id : 2;
	uint32_t            is_my_mac : 1;
} mv_nss_dp_client_t;

/*
 * typedef: struct mv_nss_dp_params_t
 *
 * Description:
 *       Parameters used in flow specification.
 *
 * Fields:
 *       ip_addr_dst - Destination IP address.
 *                     Valid if MV_NSS_DP_IP_DST bit is set in .present.
 *       ip_addr_src - Source IP address.
 *                     Valid if MV_NSS_DP_IP_SRC bit is set in .present.
 *       l2_addr_dst - Destination Layer 2 address.
 *                     Valid if MV_NSS_DP_L2_ADDR_DST bit is set in .present.
 *       l2_addr_src - Destination Layer 2 address.
 *                     Valid if MV_NSS_DP_L2_ADDR_SRC bit is set in .present.
 *       present     - Bit-mask of the valid fields in the structure.
 *       flbl        - IPv6 Flow Label.
 *                     Valid if MV_NSS_DP_FLBL bit is set in .present.
 *       opaque      - Opaque data.
 *                     Valid if MV_NSS_DP_OPAQUE bit is set in .present.
 *       eth_type    - DIX Ethernet Type (protocol ID).
 *                     Valid if MV_NSS_DP_ETH_TYPE bit is set in .present.
 *       l4_port_dst - Layer 4 destination port.
 *                     Valid if MV_NSS_DP_L4_PORT_DST bit is set in .present.
 *       l4_port_src - Layer 4 source port.
 *                     Valid if MV_NSS_DP_L4_PORT_SRC bit is set in .present.
 *       llc_ssap    - Link Layer Control Source Service Access Point.
 *                     Valid if MV_NSS_DP_LLC_SSAP bit is set in .present.
 *       llc_dsap    - Link Layer Control Destination Service Access Point.
 *                     Valid if MV_NSS_DP_LLC_DSAP bit is set in .present.
 *       port_src    - Source virtual port ID.
 *                     Valid if MV_NSS_DP_PORT_SRC bit is set in .present.
 *       port_dst    - Destination virtual port ID.
 *                     Valid if MV_NSS_DP_PORT_DST bit is set in .present.
 *       snap        - SNAP header: OUI (3-octets) + Ethernet Type (2-octets).
 *                     Valid if MV_NSS_DP_SNAP bit is set in .present.
 *       l4_proto    - Layer 4 protocol.
 *                     Valid if MV_NSS_DP_L4_PROTO bit is set in .present.
 *       prio_action - Priority action.
 *                     Valid if MV_NSS_DP_PRIO_ACTION bit is set in .present.
 *       prio_val    - Fixed priority (4 LSbs) if .prio_action is
 *                     MV_DP_NSS_PRIO_ACTION_FIXED or ingress QoS policy
 *                     number (5 LSbs) if .prio_action is
 *                     MV_DP_NSS_PRIO_ACTION_QOS_PLCY.
 *                     Valid if MV_NSS_DP_PRIO bit is set in .present.
 */
typedef struct {
	mv_nss_dp_ip_addr_t     ip_addr_dst;
	mv_nss_dp_ip_addr_t     ip_addr_src;
	mv_nss_dp_l2_addr_t     l2_addr_dst;
	mv_nss_dp_l2_addr_t     l2_addr_src;
	mv_nss_dp_bitmask_t     present;
	uint32_t                flbl;
	uint32_t                opaque;
	uint16_t                eth_type;
	uint16_t                l4_port_dst;
	uint16_t                l4_port_src;
	uint8_t                 llc_ssap;
	uint8_t                 llc_dsap;
	mv_nss_dp_port_id_t     port_dst;
	mv_nss_dp_port_id_t     port_src;
	uint8_t                 snap[5];
	mv_nss_dp_l4_proto_t    l4_proto;
	mv_nss_dp_prio_action_t prio_action;
	uint8_t                 prio_val;
} mv_nss_dp_params_t;

/*
 * typedef: struct mv_nss_dp_eth_link_state_t
 *
 * Description:
 *       Ethernet physical linik state.
 *
 * Fields:
 *       speed   - Physical speed, 10/100/1000/2500/10000
 *       is_up   - Physical state: true - up, false - down.
 *       port_id - Ethernet virtual port ID.
 *       duplex  - Duplex mode:
 *                   MV_NSS_DP_DUPLEX_HALF   - Half duplex
 *                   MV_NSS_DP_DUPLEX_FULL   - Full duplex
 *                   MV_NSS_DP_SPEED_UNKNOWN - Unknown
 */
typedef struct {
	uint16_t            speed;
	mv_nss_dp_port_id_t port_id;
	uint8_t             is_up;
	uint8_t             duplex;
} mv_nss_dp_eth_link_state_t;

/*
 * typedef: struct mv_nss_dp_dtls_t
 *
 * Description:
 *       DTLS specification.
 *
 * Fields:
 *       dtls_id              - DTLS record index starting from 0.
 *       read_mac_secret      - SHA key for downstream.
 *       read_mac_secret_len  - SHA key length for downstream, bytes.
 *       read_key             - Key for downstream.
 *       write_mac_secret     - SHA key for upstream.
 *       write_mac_secret_len - SHA key length for upstream, bytes.
 *       write_key            - Key for upstream.
 *       epoch                - Egress epoch number.
 *       seq_id               - Pointer to 48-bit sequence ID which together with
 *                              epoch represents 64-bit DTLS sequence ID.
 *       nonce                - NONCE vector (relevant for GCM mode only).
 *       mode                 - Algorithm type.
 *       version              - DTLS version number: MV_NSS_DP_DTLS_VER_1_0 or MV_NSS_DP_DTLS_VER_1_2.
 */
typedef struct {
	uint16_t              dtls_id;
	uint8_t               read_mac_secret[64];
	uint16_t              read_mac_secret_len;
	uint8_t               read_key[32];
	uint8_t               write_mac_secret[64];
	uint16_t              write_mac_secret_len;
	uint8_t               write_key[32];
	uint16_t              epoch;
	uint8_t               seq_id[6];
	uint8_t               nonce[4];
	mv_nss_dp_dtls_mode_t mode;
	uint8_t               version;
} mv_nss_dp_dtls_t;

/*
 * typedef: struct mv_nss_dp_hash_profile_t
 *
 * Description:
 *       Hash-based specification used in load balancing scenarios.
 *
 *
 *       The hash is calculated based on traffic type:
 *        a. TCP/UDP
 *        b. IP, but not TCP/UDP or fragmented or with IP options
 *        c. Non-IP
 *
 *       A hash profile allows to program a set of packet header fields
 *       to use in hash calculation for each of traffic type above.
 *       The packet header fields matching the programmed bit-mask
 *       are concatenated into a single bit sequence over which the hash
 *       is calculated. The 32-bit hash value is mapped to a number
 *       0, 1, 2, ..., MV_NSS_DP_HASH_KNEES using the algorithm
 *       illustrated in the following example:
 *
 *       LAG port = {0, 2, 3} (EMAC1 is excluded)
 *       result = {80, 0, 160}
 *
 *       Algorithm:
 *         if (hash < 80)
 *             output = 0;
 *         else if (hash < 0) // < 0 Excluded case
 *             output = 1;
 *         else if (hash < 160)
 *             output = 2;
 *         else
 *             output = 3;
 *
 * Fields:
 *       tcp_udp    - Packet fields to be used for calculating the hash for
 *                    TCP, UDP and UDP-Lite packets.
 *       ip         - Packet fields to be used for calculating the hash for
 *                    IP packets, but none of the above.
 *       non_ip     - Packet fields to be used for calculating the hash for
 *                    non-IP packets.
 *       result     - Output function: result[n] specifies the output for the hash value n.
 *       profile_id - Hash profile ID starting from 0.
 */
typedef struct {
	mv_nss_dp_bitmask_t         tcp_udp;
	mv_nss_dp_bitmask_t         ip;
	mv_nss_dp_bitmask_t         non_ip;
	mv_nss_dp_hash_profile_id_t profile_id;
	uint32_t                    result[MV_NSS_DP_HASH_KNEES];
} mv_nss_dp_hash_profile_t;

/*
 * typedef: struct mv_nss_dp_flow_status_t
 *
 * Description:
 *       Packet flow status.
 *
 * Fields:
 *       flow_id - ID of existing flow.
 *       status  - Flow activity status:
 *                   'false' - Flow is inactive.
 *                   'true'  - Flow is active.
 */
typedef struct {
	mv_nss_dp_flow_id_t flow_id;
	uint8_t             status;
} mv_nss_dp_flow_status_t;

/*
 * typedef: struct mv_nss_dp_flow_t
 *
 * Description:
 *       Packet flow descriptor.
 *       The supported sets of parameters for classification and actions
 *       are implementation-specific.
 *
 * Fields:
 *       flow_id      - Unique flow ID for an existing flow or MV_NSS_DP_FLOW_ID_NONE for new flow.
 *                      The flow IDs for new flows are generated by packet processor.
 *       cls          - Parameters for flow classification.
 *       act          - Parameters for actions to be performed once a flow is classified.
 *       idle_timeout - Idle timeout, seconds, after the flow is inactivated or 0
 *                      not to inactivate the flow based on idle timeout.
 *       status       - Flow activity status:
 *                       'false' - Flow is inactive.
 *                       'true'  - Flow is active.
 */
typedef struct {
	mv_nss_dp_flow_id_t     flow_id;
	mv_nss_dp_params_t      cls;
	mv_nss_dp_params_t      act;
	uint32_t                idle_timeout;
	uint8_t                 status;
} mv_nss_dp_flow_t;

/*
 * typedef: struct mv_nss_dp_port_eth_t
 *
 * Description:
 *       Ethernet virtual port specification.
 *
 * Fields:
 *       options - Ethernet port options:
 *                 bit[0] - Promiscuous mode:
 *                   0 - Don't allow any unicast DA
 *                   1 - Allow any unicast DA
 *                 bit[1] - Broadcast frame admission:
 *                   0 - Don't allow
 *                   1 - Allow
 *                 bit[2] - Multicast non-IP frame admission:
 *                   0 - Don't allow
 *                   1 - Allow
 *                 bit[3] - Multicast link local IP frame admission:
 *                   0 - Don't allow
 *                   1 - Allow
 *                 bit[4] - Multicast non-link local IP frame admission:
 *                   0 - Don't allow
 *                   1 - Allow
 *                 bit[5] - Unknown VLAN admission:
 *                   0 - Don't allow
 *                   1 - Allow
 * mtu           - Port MTU.
 * tpid          - TPID to use for 802.1Q tagged frames.
 * native_vlan   - Native VLAN ID.
 * policy        - VLAN admission policy.
 */
typedef struct {
	uint32_t                options;
	uint16_t                mtu;
	uint16_t                tpid;
	uint16_t                native_vlan;
	mv_nss_dp_vlan_policy_t policy;
} mv_nss_dp_port_eth_t;

/*
 * typedef: struct mv_nss_dp_port_eth_lag_t
 *
 * Description:
 *       LAG Ethernet virtual port specification.
 *
 *       The actual Ethernet port used to receive or transmit
 *       a packet is indicated by the metadata.port_link.
 *       On egress, if metadata.port_link is equal to 0xFF,
 *       actual Ethernet port to transmit a packet is selected
 *       automatically based on the LAG port associated hash profile.
 *
 * Fields:
 *       links     - Bit-mask of Ethernet links participating in the LAG.
 *                   At least one Ethernet link must be present.
 *       hash_prof - Associated hash profile ID for selecting the egress link.
 *
 */
typedef struct {
	mv_nss_dp_bitmask_t         links;
	mv_nss_dp_hash_profile_id_t hash_prof_id;
} mv_nss_dp_port_eth_lag_t;

/*
 * typedef: struct mv_nss_dp_port_cpu_t
 *
 * Description:
 *       CPU and CPU RSS virtual port specification.
 *
 * Fields:
 *
 * cores        - Bit-mask of physical CPU cores.
 * hash_prof_id - Associated hash profile ID for selecting the ingress CPU core
 *                or MV_NSS_DP_HASH_PROF_ID_NONE.
 *                In case of a single core specified in 'cores' field,
 *                the hash profile is ignored.
 */
typedef struct {
	mv_nss_dp_bitmask_t         cores;
	mv_nss_dp_hash_profile_id_t hash_prof_id;
} mv_nss_dp_port_cpu_t;

/*
 * typedef: struct mv_nss_dp_port_capwap_t
 *
 * Description:
 *       CAPWAP tunnel endpoint virtual port specification.
 *
 * Fields:
 *       remote_mac         - Remote MAC address. Ignored when matching the tunnel.
 *       local_mac          - Local MAC address.
 *       bssid              - Associated BSSID.
 *       remote_ip          - Remote IP address.
 *       local_ip           - Local IP address.
 *       port_id            - Associated Ethernet, Ethernet LAG, CPU or
 *                            MACsec virtual port ID.
 *       proto              - Layer 4 protocol.
 *       ttl                - TTL value.
 *       flow_lbl           - Flow Label for IPv6.
 *       dtls_index         - Index of a valid DTLS record or 0xFFFF for
 *                            a non-encrypted tunnel.
 *       local_port         - Local Layer 4 port.
 *       remote_port        - Remote Layer 4 port.
 *       pmtu               - Path MTU.
 *       vlan_id            - VLAN ID associated with a tunnel.
 *       options            - CAPWAP tunnel options:
 *                            bit[0] - Reserved.
 *                            bit[1] - Reserved.
 *                            bit[2] - Unicast: Allow any client on tunnel.
 *                             0 - Don't allow.
 *                             1 - Allow.
 *                            bit[3] - Multicast: Allow any MGID on tunnel.
 *                             0 - Don't allow.
 *                             1 - Allow.
 *       uc_qos_policy      - Unicast ingress QoS policy number.
 *       mc_qos_policy      - Multicast ingress QoS policy number.
 *       calc_cs            - Specifies if checksum must be calculated.
 */
typedef struct {
	mv_nss_dp_l2_addr_t         remote_mac;
	mv_nss_dp_l2_addr_t         local_mac;
	mv_nss_dp_l2_addr_t         bssid;
	mv_nss_dp_ip_addr_t         remote_ip;
	mv_nss_dp_ip_addr_t         local_ip;
	mv_nss_dp_port_id_t         port_id;
	mv_nss_dp_l4_proto_t        proto;
	uint32_t                    ttl;
	uint32_t                    flow_lbl;
	uint16_t                    dtls_index;
	uint16_t                    remote_port;
	uint16_t                    local_port;
	uint16_t                    pmtu;
	uint16_t                    vlan_id;
	uint8_t                     options;
	uint8_t                     uc_qos_policy;
	uint8_t                     mc_qos_policy;
	uint8_t                     calc_cs;
} mv_nss_dp_port_capwap_t;

/*
 * typedef: struct mv_nss_dp_port_vxlan_t
 *
 * Description:
 *       VXLAN tunnel endpoint virtual port specification.
 *
 * Fields:
 *       remote_mac         - Remote MAC address. Ignored when matching the tunnel.
 *       local_mac          - Local MAC address.
 *       remote_ip          - Remote IP address.
 *       local_ip           - Local IP address.
 *       port_id            - Associated Ethernet, Ethernet LAG, or CPU
 *                            virtual port ID.
 *       proto              - Layer 4 protocol.
 *       ttl                - TTL value.
 *       flow_lbl           - Flow Label for IPv6.
 *       local_port         - Local Layer 4 port.
 *       remote_port        - Remote Layer 4 port.
 *       vlan_id            - VLAN ID associated with a tunnel.
 *       options            - VXLAN tunnel options:
 *                            bit[0] - VXLAN flags A bit: Policy Applied.
 *                            bit[1] - VXLAN flags D bit: Don't Learn.
 *                            bit[2] - Unicast: Allow any client on tunnel.
 *                             0 - Don't allow.
 *                             1 - Allow.
 *       uc_qos_policy      - Unicast ingress QoS policy number.
 *       mc_qos_policy      - Multicast ingress QoS policy number.
 *       calc_cs            - Specifies if checksum must be calculated.
 */
typedef struct {
	mv_nss_dp_l2_addr_t         remote_mac;
	mv_nss_dp_l2_addr_t         local_mac;
	mv_nss_dp_ip_addr_t         remote_ip;
	mv_nss_dp_ip_addr_t         local_ip;
	mv_nss_dp_port_id_t         port_id;
	mv_nss_dp_l4_proto_t        proto;
	uint32_t                    ttl;
	uint32_t                    flow_lbl;
	uint16_t                    remote_port;
	uint16_t                    local_port;
	uint16_t                    vlan_id;
	uint8_t                     options;
	uint8_t                     uc_qos_policy;
	uint8_t                     mc_qos_policy;
	uint8_t                     calc_cs;
} mv_nss_dp_port_vxlan_t;




/*
 * typedef: struct mv_nss_dp_port_t
 *
 * Description:
 *       Virtual port specification.
 *
 * Fields:
 *       port_id - Unique port ID for an existing flow or MV_NSS_DP_PORT_ID_NONE for new port.
 *                 The port IDs for new ports are generated by packet processor.
 *       index   - Virtual port type index.
 *       type    - Virtual port type.
 *       state   - Operation port state.
 *       params  - Virtual port parameters specific to the port type.
 */
typedef struct {
	mv_nss_dp_port_id_t    port_id;
	mv_nss_dp_port_type_idx_t index;
	mv_nss_dp_port_type_t  type;
	mv_nss_dp_port_state_t state;

	union {
		mv_nss_dp_port_eth_t     eth;
		mv_nss_dp_port_eth_lag_t eth_lag;
		mv_nss_dp_port_cpu_t     cpu;
		mv_nss_dp_port_capwap_t  capwap;
		mv_nss_dp_port_vxlan_t   vxlan;
	} params;
} mv_nss_dp_port_t;

/*
 * typedef: struct mv_nss_dp_port_stats_t
 *
 * Description:
 *       Virtual port counters.
 *
 * Fields:
 *       rx_pkts   - Number of packets received from port.
 *       rx_errors - Number of packets dropped due to receive errors.
 *       tx_pkts   - Number of packets transmitted to port.
 *       tx_errors - Number of packets not transmitted due to errors.
 *       rx_octets - Numbers of octets received from port.
 *       tx_octets - Number of octets transmitted to port.
 *       port_id   - Virtual port ID.
 *       type      - Virtual port type.
 */
typedef struct {
	uint64_t              rx_pkts;
	uint64_t              rx_errors;
	uint64_t              tx_pkts;
	uint64_t              tx_errors;
	uint64_t              rx_octets;
	uint64_t              tx_octets;
	mv_nss_dp_port_id_t   port_id;
	mv_nss_dp_port_type_t type;
} mv_nss_dp_port_stats_t;

/*
 * typedef: struct mv_nss_dp_vlan_cfg_t
 *
 * Description:
 *       Global VLAN configuration.
 *
 * Fields:
 *       vlan_id       - 802.1Q VLAN ID.
 *       uc_qos_policy - Unicast ingress QoS policy number.
 *       mc_qos_policy - Multicast ingress QoS policy number.
 *       ports_mask    - Bit-mask of Ethernet virtual ports for the VLAN.
 */
typedef struct {
	uint16_t vlan_id;
	uint8_t  uc_qos_policy;
	uint8_t  mc_qos_policy;
	uint8_t  ports_mask;
} mv_nss_dp_vlan_cfg_t;

/*
 * typedef: struct mv_nss_dp_ingress_qos_policy_t
 *
 * Description:
 *       Ingress priority policy.
 *
 * Fields:
 *       policy_id    - Priority policy ID.
 *       type         - Policy type.
 *       prio_def     - Default fixed priority.
 *       l2_to_prio   - Maps 3-bit PCP (802.1Q) and UP (802.1e) to priority.
 *       dscp_to_prio - Maps 6-bit DSCP to priority.
 */
typedef struct {
	uint8_t                     policy_id;
	mv_nss_dp_qos_policy_type_t type;
	uint8_t                     prio_def;
	uint8_t                     l2_to_prio[8];
	uint8_t                     dscp_to_prio[64];
} mv_nss_dp_ingress_qos_policy_t;

/*
 * typedef: struct mv_nss_dp_egress_qos_policy_t
 *
 * Description:
 *       Egress priority policy.
 *
 * Fields:
 *       policy_id    - Priority policy ID. Must be 0.
 *       prio_to_up   - Maps priority to 3-bit UP (802.1e) in inner frame.
 *       prio_to_dscp - Maps priority to 6-bit DSCP in outer frame.
 *       prio_to_pcp  - Maps priority to 3-bit PCP (802.1Q) in outer frame.
 */
typedef struct {
	uint8_t policy_id;
	uint8_t prio_to_up[MV_NSS_DP_PRIO_NUM];
	uint8_t prio_to_dscp[MV_NSS_DP_PRIO_NUM];
	uint8_t prio_to_pcp[MV_NSS_DP_PRIO_NUM];
} mv_nss_dp_egress_qos_policy_t;

/*
 * typedef: struct mv_nss_dp_mc_bridged_cfg_t
 *
 * Description:
 *       Configuration for processing bridged multicast packets.
 *
 * Fields:
 *       l2_addr - Multicast Layer 2 address.
 *       opaque  - Opaque data.
 *       vlan_id - Allowed VLAN ID or MV_NSS_DP_VLAN_ANY for any VLAN.
 */
typedef struct {
	mv_nss_dp_l2_addr_t l2_addr;
	uint32_t            opaque;
	uint16_t            vlan_id;
} mv_nss_dp_mc_bridged_cfg_t;

/*
 * typedef: struct mv_nss_dp_mc_tunneled_cfg_t
 *
 * Description:
 *       Configuration for processing tunneled multicast packets.
 *
 * Fields:
 *       mgid    - Multicast group ID.
 *       policy_id - Ingress QoS policy ID.
 */
typedef struct {
	uint16_t mgid;
	uint8_t  policy_id;
} mv_nss_dp_mc_tunneled_cfg_t;

/*
 * typedef: struct mv_nss_dp_meter_t
 *
 * Description:
 *       Traffic meter parameters used for policing or shaping.
 *
 * Fields:
 *       cir  - Committed information rate, Kbps.
 *       eir  - Excess information rate, Kbps.
 *       cbs  - Committed burst size, KB.
 *       ebs  - Excess burst size, KB.
 */
typedef struct {
	uint32_t cir;
	uint32_t eir;
	uint32_t cbs;
	uint32_t ebs;
} mv_nss_dp_meter_t;

/*
 * typedef: struct mv_nss_dp_ingress_queue_cfg_t
 *
 * Description:
 *       Virtual ingress queue configuration.
 *       Each queue belongs to a High or Normal priority group
 *       depending on the static mapping.
 *
 * Fields:
 *       policer           - Policing parameters.
 *       tail_drop_thresh  - Number of packets before the tail drop occurs.
 *       red_thresh        - Maximum number of packets before Random Early Drop
 *                           (RED) occurs.
 *       sched_priority    - Scheduling priority within the queue group in range
 *                           0..MV_NSS_DP_SCHED_PRIO_NUM.
 *       sched_weight      - Scheduling weight within the queue group.
 *       queue             - Virtual queue number starting from 0.
 */
typedef struct {
	mv_nss_dp_meter_t	policer;
	uint16_t		    tail_drop_thresh;
	uint16_t		    red_thresh;
	uint16_t		    sched_priority;
	uint16_t		    sched_weight;
	uint8_t		    queue;
} mv_nss_dp_ingress_queue_cfg_t;

/*
 * typedef: struct mv_nss_dp_ingress_prio_cfg_t
 *
 * Description:
 *       Mapping of priority values to virtual ingress queues.
 *
 * Fields:
 *       prio_mngt - Default priority for ingress IEEE 802.11 management frames.
 *       queue     - queue[N] specifies the ingress queue for priority N.
 */
typedef struct {
	uint8_t prio_mngt;
	uint8_t queue[MV_NSS_DP_PRIO_NUM];
} mv_nss_dp_ingress_prio_cfg_t;

/*
 * typedef: struct mv_nss_dp_egress_queue_cfg_t
 *
 * Description:
 *       Virtual egress queue configuration.
 *       Each queue belongs to a High or Normal priority group
 *       depending on the static mapping.
 *
 * Fields:
 *       policer          - Policing parameters.
 *       shaper           - Shaping parameters.
 *       tail_drop_thresh - Number of packets in before the tail drop occurs.
 *       sched_priority   - Scheduling priority within the queue group in range
 *                          0..MV_NSS_DP_SCHED_PRIO_NUM.
 *       sched_weight     - Scheduling weight within the  queue group.
 *       queue            - Virtual queue number starting from 0.
 */
typedef struct {
	mv_nss_dp_meter_t policer;
	mv_nss_dp_meter_t shaper;
	uint16_t          tail_drop_thresh;
	uint16_t          sched_priority;
	uint16_t          sched_weight;
	uint8_t           queue;
} mv_nss_dp_egress_queue_cfg_t;

/*
 * typedef: struct mv_nss_dp_egress_prio_cfg_t
 *
 * Description:
 *       Mapping of priority values to virtual egress queues.
 *
 * Fields:
 *       queue - queue[N] specifies the egress queue for priority N.
 */
typedef struct {
	uint8_t queue[MV_NSS_DP_PRIO_NUM];
} mv_nss_dp_egress_prio_cfg_t;

/*
 * typedef: struct mv_nss_dp_flow_stats_t
 *
 * Description:
 *       Flow counters.
 *
 * Fields:
 *       rx_pkts    - Number of flow packets received.
 *       rx_octets  - Number of flow octets received.
 *       rx_last_ts - Timestamp of the last received packet.
 *       flow_id    - Flow ID.
 */
typedef struct {
	uint64_t            rx_pkts;
	uint64_t            rx_octets;
	uint32_t            rx_last_ts;
	mv_nss_dp_flow_id_t flow_id;
} mv_nss_dp_flow_stats_t;


/*
 * typedef: struct mv_nss_dp_queue_stats_t
 *
 * Description:
 *       Virtual queue counters.
 *
 * Fields:
 *       pkts   - Number of packets passed.
 *       errors - Number of packets dropped due to errors.
 *       queue  - Virtual queue number.
 */
typedef struct {
	uint64_t              pkts;
	uint64_t              errors;
	uint8_t               queue;
} mv_nss_dp_queue_stats_t;

/*
 * typedef: struct mv_nss_dp_vxlan_vni_cfg_t
 *
 * Description:
 *       VXLAN VNI configuration.
 *
 * Fields:
 *       port_id  - VXLAN port-id.
 *       vni      - VXLAN Network Identifier.
 *       index    - Index value associated with this VNI.
 *                  This is the value that will be returned in Metadata.
 */
typedef struct {
	mv_nss_dp_port_id_t  port_id;
	uint32_t             vni;
	uint8_t              index;
} mv_nss_dp_vxlan_vni_cfg_t;


/*
 * typedef: struct mv_nss_dp_event_t
 *
 * Description:
 *       Notification of a system originated event or completion of an API
 *       request. Please refer to the description of specific requests
 *       for information on the corresponding event fields.
 *
 * Fields:
 *       status - Status code.
 *       xid    - Application provided data in the original request.
 *       cookie - Application provided data in the original request.
 *       count  - Number of items successfully processed by the system.
 *       type   - Event type.
 *       params - Parameters of the event corresponding to event type.
 */
typedef struct {
	mv_nss_dp_status_t      status;
	uint32_t                xid;
	void                   *cookie;
	uint32_t                count;
	mv_nss_dp_event_type_t  type;

	union {
		char                           *notify_msg;
		mv_nss_dp_notifiy_t            *notify_code;
		mv_nss_dp_port_t               *port;
		mv_nss_dp_port_stats_t         *port_stats;
		mv_nss_dp_flow_t               *flow;
		mv_nss_dp_flow_status_t        *flow_status;
		mv_nss_dp_flow_stats_t         *flow_stats;
		uint32_t                       *flow_count;
		mv_nss_dp_ingress_prio_cfg_t   *ingress_prio_cfg;
		mv_nss_dp_ingress_queue_cfg_t  *ingress_queue_cfg;
		mv_nss_dp_ingress_qos_policy_t *ingress_qos_policy;
		mv_nss_dp_egress_prio_cfg_t    *egress_prio_cfg;
		mv_nss_dp_egress_queue_cfg_t   *egress_queue_cfg;
		mv_nss_dp_egress_qos_policy_t  *egress_qos_policy;
		mv_nss_dp_vlan_cfg_t           *vlan_cfg;
		mv_nss_dp_mc_bridged_cfg_t     *mc_bridged_cfg;
		mv_nss_dp_mc_tunneled_cfg_t    *mc_tunneled_cfg;
		mv_nss_dp_client_t             *client;
		mv_nss_dp_dtls_t               *dtls;
		mv_nss_dp_hash_profile_t       *hash_prof;
		mv_nss_dp_eth_link_state_t     *eth_link_state;
		mv_nss_dp_queue_stats_t        *queue_stats;
		uint8_t                        *bypass_state;
		mv_nss_dp_vxlan_vni_cfg_t      *vxlan_vni_cfg;
	} params;
} mv_nss_dp_event_t;

/*
 * typedef: mv_nss_dp_evt_handler
 *
 * Description:
 *       Event handler routine for the system originated events.
 *
 * Parameters:
 *       event - System event specification with the following fields:
 *        .type - MV_NSS_DP_EVT_NOTIFY_CODE or MV_NSS_DP_EVT_NOTIFY_MSG.
 *        .count - Equal to 0.
 *        .cookie - Equal to 0.
 *        .xid - Equal to 0.
 *        .status - System status code.
 *        .params.notify_msg - System message if .type is MV_NSS_DP_EVT_NOTIFY_MSG.
 *        .param.notify_code - System message if .type is MV_NSS_DP_EVT_NOTIFY_CODE.
 */
typedef void (*mv_nss_dp_evt_handler)(mv_nss_dp_event_t *event);

/*
 * typedef: struct mv_nss_dp_result_spec_t
 *
 * Description:
 *       Request completion result delivery specification.
 *
 * Fields:
 *       cb     - Application callback routine to be invoked upon request completion.
 *       xid    - Application data to be returned in the callback.
 *       cookie - Application data to be returned in the callback.
 */
typedef struct {
	mv_nss_dp_evt_handler cb;
	uint32_t              xid;
	void                 *cookie;
} mv_nss_dp_result_spec_t;

/*
 * typedef: struct mv_nss_dp_init_cfg_t
 *
 * Description:
 *       Data Path API initialization parameters.
 *
 * Fields:
 *       requests_num    - The maximum number of outstanding API requests issued
 *                         by application before issuing a new request fails
 *                         with the error code MV_NSS_DP_TOO_MANY_REQUESTS.
 *       port_dst        - Default destination virtual port for ingress packets
 *                         not matching any flows.
 *       notify_cb       - Application callback routine to be invoked upon async notification.
 */
typedef struct {
	uint32_t              requests_num;
	mv_nss_dp_port_id_t   port_dst;
	mv_nss_dp_evt_handler notify_cb;
} mv_nss_dp_init_cfg_t;

/*
 * mv_nss_dp_init
 *
 * Description:
 *       Start NSS engine with default NSS configuration.
 *
 *       All pre-defined ports are initially in the down state and
 *       are not able to receive or send traffic.
 *
 *       Application must ensure successful NSS initialization before
 *       executing any further API requests.
 *
 * Parameters:
 *       cfg - API initialization parameters.
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *            The result is returned via event with the following values
 *            of mv_nss_dp_event_t structure fields:
 *	        .type - MV_NSS_DP_EVT_INIT
 *          .cookie - Equal to res->cookie
 *	        .xid - Equal to res->xid
 *	        .status - Result of operation
 *
 *		 Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_init(const mv_nss_dp_init_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_shutdown
 *
 * Description:
 *       Stop NSS engine and reset entire NSS configuration to its initial
 *       state before mv_nss_dp_init.
 *
 *       All created entities, such as flows, virtual ports,
 *       hash profiles, etc are destroyed regardless of their existing
 *       dependencies. Pre-defined virtual ports are brought to down state
 *       and are no longer able to receive or send traffic.
 *
 *       All traffic processing by NSS is stopped in ingress and egress directions.
 *
 *       All pending API requests are cancelled with MV_NSS_DP_CANCELLED status
 *       returned in notifications.
 *
 *       Application must execute mv_nss_dp_init in order to restart NSS engine.
 *
 * Parameters:
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *		.type - MV_NSS_DP_EVT_SHUTDOWN
 *      .cookie - Equal to res->cookie
 *	    .xid - Equal to res->xid
 *		.status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_shutdown(const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_system_dim_get
 *
 * Description:
 *       Return system dimensions.
 *
 * Parameters:
 *       type - Type of system entity.
 *       res  - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *	        .type - MV_NSS_DP_EVT_SYSTEM_DIM_GET
 *          .cookie - Equal to res->cookie
 *	        .xid - Equal to res->xid
 *		    .status - Result of operation
 *          .params.dim - Supported number of instances.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_system_dim_get(mv_nss_dp_entity_type_t type,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_hash_profile_set
 *
 * Description:
 *       Create or update a hash profile.
 *
 * Parameters:
 *       profile - Hash profile specification.
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *	        .type - MV_NSS_DP_EVT_PROFILE_SET
 *          .cookie - Equal to res->cookie
 *	        .xid - Equal to res->xid
 *	        .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_hash_profile_set(const mv_nss_dp_hash_profile_t *profile,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_hash_profile_get
 *
 * Description:
 *       Retrieve hash profile.
 *
 * Parameters:
 *       id - ID of an existing hash profile.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_HASH_PROFILE_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.hash_prof - Hash profile.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_hash_profile_get(mv_nss_dp_hash_profile_id_t profile_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_hash_profile_delete
 *
 * Description:
 *       Delete a hash profile. The request would fail if there are currently
 *       entities dependent on that hash profile (eg virtual ports).
 *
 * Parameters:
 *       profile_id - Hash profile ID.
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *          The result is returned via event with the following values
 *          of mv_nss_dp_event_t structure fields:
 *	        .type - MV_NSS_DP_EVT_PROFILE_DELETE
 *          .cookie - Equal to res->cookie
 *	        .xid - Equal to res->xid
 *	        .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_hash_profile_delete(mv_nss_dp_hash_profile_id_t profile_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_dtls_set
 *
 * Description:
 *       Create or update DTLS record.
 *
 * Parameters:
 *       dtls - Pointer to DTLS record to create or update.
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_DTLS_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_dtls_set(const mv_nss_dp_dtls_t *dtls,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_dtls_delete
 *
 * Description:
 *       Delete a DTLS record.
 *
 * Parameters:
 *       dtls_id - DTLS record ID to delete.
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_DTLS_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_dtls_delete(uint16_t dtls_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_dtls_get
 *
 * Description:
 *       Retrieve DTLS record.
 *
 * Parameters:
 *       id - ID of an existing DTLS record.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_DTLS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.dtls - DTLS record.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_dtls_get(uint16_t dtls_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_port_set
 *
 * Description:
 *       Create or update virtual port.
 *
 * Parameters:
 *       port - Virtual port specification.
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_set(const mv_nss_dp_port_t *port,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_port_delete
 *
 * Description:
 *       Remove a virtual port. The request would fail if there are currently
 *       entities dependent on that port (eg flows) or the port is up.
 *       Removing a pre-defined virtual port (eg Ethernet or CPU) restores its
 *       initial configutaion.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port.
 *       res - Result event specification or NULL for no result event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_delete(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_port_get
 *
 * Description:
 *       Return parameters of a virtual port.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.port - Port parameters.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_get(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_port_stats_get
 *
 * Description:
 *       Get counters of a virtual port.
 *
 * Parameters:
 *       port_id - ID of virtual port.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *           .params.port_stats - Pointer to port counters record
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_stats_get(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_port_bulk_stats_get
 *
 * Description:
 *       Retrieve counters of virtual ports range.
 *
 * Parameters:
 *       index - Index of the first record in range, starting from 0.
 *       count - Total number of records to get.
 *       stats - Application allocated buffer to receive the data.
 *               The buffer size must be greater or equal to
 *               sizeof(mv_nss_dp_port_stats_t) * count. Else, the maximum
 *               number of fitting records is returned.
 *               IMPORTANT: Application must not deallocate the 'stats'
 *               buffer until receiving the result event.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Number of records in params.port_stats.
 *                       Can be smaller than the requested number 'count' in case
 *                       there are fewer records.
 *           .status - Result of operation. In case, 'index' exceeds
 *                        the number of existing records, MV_NSS_DP_END_OF_LIST
 *                        status is returned.
 *           .params.port_stats - Equal to 'stats' argument
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_bulk_stats_get(uint32_t index,
		uint32_t count,
		mv_nss_dp_port_stats_t *stats,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_port_stats_reset
 *
 * Description:
 *       Reset virtual port counters.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port or MV_NSS_DP_PORT_ALL for all ports.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_STATS_RESET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_stats_reset(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_port_dst_set
 *
 * Description:
 *       Set destination virtual port for ingress packets not matching any flows.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_DST_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_dst_set(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_vlan_cfg_set
 *
 * Description:
 *       Set VLAN configuration.
 *
 * Parameters:
 *       cfg - Pointer to array of VLAN configurations.
 *       count - Number of records pointed by 'cfg', greater than or equal to 1.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_VLAN_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *           .status - Result of operation
 *
 *     Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_set(const mv_nss_dp_vlan_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);
/*
 * mv_nss_dp_vlan_cfg_delete
 *
 * Description:
 *       Delete VLAN configuration.
 *
 * Parameters:
 *       vlan_id - Pointer to array of VLAN IDs to delete.
 *       count - Number of records pointed by 'vlan_id', greater than or equal to 1.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_VLAN_CFG_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'vlan_id' array successfully processed
 *           .status - Result of operation
 *
 *     Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_delete(const uint16_t *vlan_id,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_vlan_cfg_get
 *
 * Description:
 *       Get VLAN configuration.
 *
 * Parameters:
 *       index - Configuration record index starting from 0.
 *       res  - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_VLAN_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation. In case, 'index' exceeds
 *                     the number of existing records, MV_NSS_DP_END_OF_LIST
 *                     status is returned.
 *           .params.vlan_cfg - VLAN configuration record.
 *
 *     Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_get(uint16_t index,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_vlan_cfg_get_by_id
 *
 * Description:
 *       Get VLAN configuration matching a VLAN ID.
 *
 * Parameters:
 *       vlan_id - VLAN ID of the record to retrieve.
 *       res     - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_VLAN_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation. In case no matching record is
 *                     found, MV_NSS_DP_ITEM_NOT_FOUND status is returned.
 *           .params.vlan_cfg - VLAN configuration record.
 *
 *     Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_get_by_id(uint16_t vlan_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_mc_bridged_cfg_set
 *
 * Description:
 *       Set bridged multicast configuration.
 *
 * Parameters:
 *       cfg - Pointer to array of bridged multicast configurations.
 *       count - Number of records pointed by 'cfg', greater than or equal to 1.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_MC_BRIDGED_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *           .status - Result of operation
 *
 *     Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_mc_bridged_cfg_set(const mv_nss_dp_mc_bridged_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_mc_bridged_cfg_delete
 *
 * Description:
 *       Delete bridged multicast configuration(s) matching a multicast
 *       Layer 2 address.
 *
 * Parameters:
 *       cfg - Pointer to array of bridged multicast configurations to delete.
 *             The .opaque field is ignored when performing the configuration match.
 *       count - Number of records pointed by 'l2_addr', greater than or equal to 1.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_MC_BRIDGED_CFG_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_mc_bridged_cfg_delete(const mv_nss_dp_mc_bridged_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_mc_bridged_cfg_get
 *
 * Description:
 *       Get bridged multicast configuration.
 *
 * Parameters:
 *       index - Configuration record index starting from 0.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation. In case, 'index' exceeds
 *                     the number of existing records, MV_NSS_DP_END_OF_LIST
 *                     status is returned.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_mc_bridged_cfg_get(uint16_t index,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_mc_tunneled_cfg_set
 *
 * Description:
 *       Set tunneled multicast configuration.
 *
 * Parameters:
 *       cfg - Pointer to array of tunneled multicast configurations.
 *       count - Number of records pointed by 'cfg', greater than or equal to 1.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_MC_TUNNELED_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_mc_tunneled_cfg_set(const mv_nss_dp_mc_tunneled_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_mc_tunneled_cfg_delete
 *
 * Description:
 *       Delete one or more tunneled multicast configurations.
 *       The result is returned in event of type MV_NSS_DP_EVT_ACK.
 *
 * Parameters:
 *       mgid - Pointer to array of multicast group IDs.
 *       count - Number of records pointed by 'mgid', greater than or equal to 1.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_MC_TUNNELED_CFG_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'mgid' array successfully processed
 *           .status - Result of operation.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_mc_tunneled_cfg_delete(const uint16_t *mgid,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_mc_tunneled_cfg_get
 *
 * Description:
 *       Get tunneled multicast configuration.
 *
 * Parameters:
 *       index - Configuration record index starting from 0.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_MC_TUNNELED_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation. In case, 'index' exceeds
 *                     the number of existing records, MV_NSS_DP_END_OF_LIST
 *                     status is returned.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_mc_tunneled_cfg_get(uint16_t index,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_set
 *
 * Description:
 *       Add or update flows.
 *
 * Parameters:
 *       flow - Pointer to array of flows to be added or updated.
 *              For new flows the .flow_id field must be set to
 *              MV_NSS_DP_FLOW_ID_NONE.
 *              The actual flow IDs are auto-generated and returned via
 *              the result event.
 *              IMPORTANT: Application must not deallocate the 'flow' array
 *              until receiving the result event.
 *       count - Number of records pointed by 'flow', greater than or equal to 1.
 *       res  - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .count - The number of records from the beginning of
 *                    the 'flow' array successfully processed
 *           .params.flow - Equal to 'flow'
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_set(mv_nss_dp_flow_t *flow,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_delete
 *
 * Description:
 *       Remove existing flows.
 *
 * Parameters:
 *       flow_id - Pointer to array of flow IDs to remove.
 *       count - Number of records pointed by 'flow_id', greater than or equal to 1.
 *       res     - Result event specification or NULL.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - The number of records from the beginning of
 *                    the 'flow_id' array successfully processed
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_delete(const mv_nss_dp_flow_id_t *flow_id,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_get
 *
 * Description:
 *       Retrieve flow record.
 *
 * Parameters:
 *       flow_id - ID of an existing flow to retrieve.
 *       res     - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_FLOW_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.flow - Flow record retrieved
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_get(mv_nss_dp_flow_id_t flow_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_delete_all
 *
 * Description:
 *       Remove all flows.
 *
 * Parameters:
 *       res - Result event specification or NULL.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .count - Number of deleted flows
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_delete_all(const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_status_set
 *
 * Description:
 *       Set flow activity status.
 *
 * Parameters:
 *       flow_status - Pointer to array specifying flow IDs and their new statui.
 *       count   - Number of items pointed by 'flow_status'.
 *       res     - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_STATUS_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .count - The number of records from the beginning of
 *                    the 'flow_status' array successfully processed
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_status_set(const mv_nss_dp_flow_status_t *flow_status,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_status_get
 *
 * Description:
 *       Retrieve flow activity status.
 *
 * Parameters:
 *       flow_id - ID of an existing flow.
 *       res     - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_STATUS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.flow_status - Flow status
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_status_get(mv_nss_dp_flow_id_t flow_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_stats_get
 *
 * Description:
 *       Retrieve a flow statistics.
 *
 * Parameters:
 *       flow_id - ID of an existing flow.
 *       res     - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.flow_stats - Flow statistics
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_stats_get(mv_nss_dp_flow_id_t flow_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_stats_reset
 *
 * Description:
 *       Reset flow statistics.
 *
 * Parameters:
 *       flow_id - ID of existing flow or MV_NSS_DP_FLOW_ID_ALL for all flows.
 *       res     - Result event specification or NULL.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_STATS_RESET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_stats_reset(mv_nss_dp_flow_id_t flow_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_bulk_stats_get
 *
 * Description:
 *       Retrieve snapshot of flow statistics.
 *
 * Parameters:
 *       index - Index of the first record in range, starting from 0.
 *       count - Total number of records to retrieve.
 *       options - Bit-mask specifying the subset of flows to query. If zero, all flows are implied.
 *               bit [0] - Flow status:
 *                  1 - Flows with inactive status only
 *                  0 - Flows with any activity status
 *       stats - Application allocated buffer to receive the data whose size
 *               must be greater or equal to  sizeof(mv_nss_dp_flow_stats_t) * count.
 *               Else, the maximum number of fitting records is returned.
 *               IMPORTANT: Application must not deallocate the 'stats' buffer
 *               until receiving the result event.
 *
 *       res     - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Number of records in params.flow_stats.
 *                    Can be smaller than the requested number 'count' in case
 *                    there are fewer records.
 *           .status - Result of operation. In case, 'index' exceeds
 *                     the number of existing records, MV_NSS_DP_END_OF_LIST
 *                     status is returned.
 *           .params.flow_stats - Equal to 'stats'.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_bulk_stats_get(uint32_t index,
		uint32_t count,
		uint32_t options,
		mv_nss_dp_flow_stats_t *stats,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_flow_get_count
 *
 * Description:
 *       Retrieve the number of existing flows.
 *
 * Parameters:
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_COUNT
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.flow_count - Current number of flows
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_count_get(const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_queue_cfg_set
 *
 * Description:
 *       Update virtual ingress queue configuration.
 *
 * Parameters:
 *       cfg - Pointer to array of virtual ingress queue configurations.
 *       count - Number of records pointed by 'cfg', greater than or equal to 1.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_INGRESS_Q_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_cfg_set(const mv_nss_dp_ingress_queue_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_queue_cfg_get
 *
 * Description:
 *       Retrieve virtual ingress queue configuration.
 *
 * Parameters:
 *       queue - virtual ingress queue which configuration to retrieve.
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_INGRESS_Q_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .params.ingress_queue_cfg - Ingress queue configuration.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_cfg_get(uint8_t queue,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_prio_cfg_set
 *
 * Description:
 *       Update ingress priority configuration.
 *
 * Parameters:
 *       cfg - Pointer to ingress priority configuration.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_INGRESS_PRIO_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_prio_cfg_set(const mv_nss_dp_ingress_prio_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_prio_cfg_get
 *
 * Description:
 *       Retrieve ingress priority configuration.
 *
 * Parameters:
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_INGRESS_PRIO_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .params.ingress_prio_cfg - Ingress priority configuration.
 *
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_prio_cfg_get(const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_qos_policy_set
 *
 * Description:
 *       Update ingress QoS policy.
 *
 * Parameters:
 *       policy - Pointer to array of priority policy records.
 *       count - Number of records pointed by 'policy', greater than or equal to 1.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_INGRESS_QOS_POLICY_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - The number of records from the beginning of
 *                    the 'policy' array successfully processed
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_qos_policy_set(const mv_nss_dp_ingress_qos_policy_t *policy,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_qos_policy_get
 *
 * Description:
 *       Retrieve ingress QoS policy.
 *
 * Parameters:
 *       policy_id - ID of priority policy to retrieve.
 *       res       - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_INGRESS_QOS_POLICY_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.ingress_qos_cfg - Ingress QoS policy record
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_qos_policy_get(uint8_t policy_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_egress_queue_cfg_set
 *
 * Description:
 *       Update virtual egress queue configuration.
 *
 * Parameters:
 *       cfg - Pointer to array of virtual egress queue configurations.
 *       count - Number of records pointed by 'cfg', greater than or equal to 1.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_Q_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_cfg_set(const mv_nss_dp_egress_queue_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_egress_queue_cfg_get
 *
 * Description:
 *       Retrieve virtual egress queue configuration.
 *
 * Parameters:
 *       queue - Virtual egress queue which configuration to retrieve.
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_Q_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .params.egress_queue_cfg - Ingress queue configuration.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_cfg_get(uint8_t queue,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_egress_prio_cfg_set
 *
 * Description:
 *       Update egress priority configuration.
 *
 * Parameters:
 *       cfg - Pointer to egress priority configuration.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_PRIO_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_prio_cfg_set(const mv_nss_dp_egress_prio_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_egress_prio_cfg_get
 *
 * Description:
 *       Retrieve egress priority configuration.
 *
 * Parameters:
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_PRIO_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .params.ingress_prio_cfg - Egress priority configuration.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_prio_cfg_get(const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_egress_qos_policy_set
 *
 * Description:
 *       Update egress QoS policy.
 *
 * Parameters:
 *       policy - Priority policy record.
 *       count - Number of records pointed by 'policy', greater than or equal to 1.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_ENGRESS_QOS_POLICY_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_qos_policy_set(const mv_nss_dp_egress_qos_policy_t *policy,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_egress_qos_policy_get
 *
 * Description:
 *       Retrieve egress QoS policy.
 *
 * Parameters:
 *       policy_id - ID of the priority policy to retrieve. Must be 0.
 *       res       - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_INGRESS_QOS_POLICY_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.engress_qos_cfg - Ingress QoS policy record
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_qos_policy_get(uint8_t policy_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_client_set
 *
 * Description:
 *       Add or update client record.
 *
 * Parameters:
 *       client - Client record to add or update.
 *       count - Number of records pointed by 'client', must be equal to 1.
 *       res - Result event specification or NULL for no event.
 *
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_CLIENT_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'client' array successfully processed.
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */

mv_nss_dp_status_t mv_nss_dp_client_set(const mv_nss_dp_client_t *client,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_client_get
 *
 * Description:
 *       Retrieve client record.
 *
 * Parameters:
 *       client_id - ID of an existing Client to retrieve.
 *       res   - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .params.client - Client record retrieved.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_client_get(uint16_t client_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_client_delete
 *
 * Description:
 *       Remove an existing client record.
 *
 * Parameters:
 *       client_id - Client ID to remove.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_CLIENT_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - Index of the last successfully processed
 *                    record in the 'l2_addr' array
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_client_delete(uint16_t client_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_eth_link_state_get
 *
 * Description:
 *       Retrieve Ethernet physical link state.
 *
 * Parameters:
 *       port_id - ID of an existing Ethernet virtual port.
 *       res     - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_ETH_LINK_STATE_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.eth_link_state - Ethernet physical link state.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_eth_link_state_get(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_queue_stats_get
 *
 * Description:
 *       Get counters of a virtual Ingress queue.
 *
 * Parameters:
 *       queue - Virtual Ingress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_INGRESS_Q_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *           .params.queue_stats - Pointer to queue counters record
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_stats_get(uint8_t queue,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_queue_stats_reset
 *
 * Description:
 *       Reset counters of a virtual Ingress queue.
 *
 * Parameters:
 *       queue - Virtual Ingress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_INGRESS_Q_STATS_RESET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_stats_reset(uint8_t queue,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_egress_queue_stats_get
 *
 * Description:
 *       Get counters of a virtual Egress queue.
 *
 * Parameters:
 *       queue - Virtual Egress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_EGRESS_Q_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *           .params.egress_queue_stats - Pointer to queue counters record
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_stats_get(uint8_t queue,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_ingress_queue_stats_reset
 *
 * Description:
 *       Reset counters of a virtual Egress queue.
 *
 * Parameters:
 *       queue - Virtual Egress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_EGRESS_Q_STATS_RESET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_stats_reset(uint8_t queue,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_state_get
 *
 * Description:
 *		 Retrieve current network sub-system state.
 *
 *		 The API is working in synchronous mode only and res->cb must be equal to zero.
 *
 * Parameters:
 *       state - pointer for result.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via state pointer with the values
 *           of mv_nss_dp_state_t enum.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_state_get(mv_nss_dp_state_t *state, const mv_nss_dp_result_spec_t *res);


/*
 * mv_nss_dp_bypass_state_set
 *
 * Description:
 *       Set bypass enabled or disabled
 *
 * Parameters:
 *    state  - bypass state.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_BYPASS_STATE_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_bypass_state_set(const uint8_t enable,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_bypass_state_get
 *
 * Description:
 *       Get bypass state
 *
 * Parameters:
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_BYPASS_STATE_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.bypass_state - Bypass state enabled or disabled.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_bypass_state_get(const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_vxlan_vni_cfg_set
 *
 * Description:
 *       Set VXLAN VNI configuration
 *
 * Parameters:
 *    cfg  - Pointer to VXLAN VNI configuration.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_VXLAN_VNI_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vxlan_vni_cfg_set(const mv_nss_dp_vxlan_vni_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_vxlan_vni_cfg_get
 *
 * Description:
 *       Get VXLAN VNI configuration
 *
 * Parameters:
 *       cfg  - Pointer to VXLAN VNI configuration to retrieve.
 *       res  - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_VXLAN_VNI_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.vxlan_vni_cfg - VXLAN VNI configuration.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vxlan_vni_cfg_get(const mv_nss_dp_vxlan_vni_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_vxlan_vni_cfg_delete
 *
 * Description:
 *       Delete VXLAN VNI configuration
 *
 * Parameters:
 *       cfg  - Pointer to VXLAN VNI configuration to delete.
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_VXLAN_VNI_CFG_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vxlan_vni_cfg_delete(const mv_nss_dp_vxlan_vni_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res);

/*
 * mv_nss_dp_register_notify_cb
 *
 * Description:
 *       Register application callback routine.
 *
 * Parameters:
 *    cb       - Application callback routine to be invoked upon async notification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_register_notify_cb(mv_nss_dp_evt_handler cb);

#ifdef __cplusplus
}
#endif


#endif

