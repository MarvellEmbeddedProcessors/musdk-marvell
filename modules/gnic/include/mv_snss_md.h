/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _MV_NSS_METADATA_H_
#define _MV_NSS_METADATA_H_


/* Present fields in egress metadata */
#define MV_NSS_MDAT_EGR_TYPE_BIT         (0)
#define MV_NSS_MDAT_EGR_VLAN_BIT         (1)
#define MV_NSS_MDAT_EGR_RID_BIT          (2)
#define MV_NSS_MDAT_EGR_BSSID_BIT        (3)
#define MV_NSS_MDAT_EGR_WBID_BIT         (4)
#define MV_NSS_MDAT_EGR_OUTER_8021Q_BIT  (5)
#define MV_NSS_MDAT_EGR_OUTER_DSCP_BIT   (6)
#define MV_NSS_MDAT_EGR_INNER_8021E_BIT  (7)
#define MV_NSS_MDAT_EGR_RSSI_SNR_BIT	 (8)
#define MV_NSS_MDAT_EGR_PRIO_BIT         (9)


/* Present fields in ingress metadata */
#define MV_NSS_MDAT_ING_COOKIE_BIT       (0)
#define MV_NSS_MDAT_ING_VLAN_BIT         (1)
#define MV_NSS_MDAT_ING_RID_BIT          (2)
#define MV_NSS_MDAT_ING_BSSID_BIT        (3)
#define MV_NSS_MDAT_ING_WBID_BIT         (4)
#define MV_NSS_MDAT_ING_OUTER_8021Q_BIT  (5)
#define MV_NSS_MDAT_ING_OUTER_DSCP_BIT   (6)
#define MV_NSS_MDAT_ING_INNER_8021E_BIT  (7)
#define MV_NSS_MDAT_ING_INNER_DSCP_BIT   (8)

#define MV_NSS_METADATA_SIZE 16


/* Packet origination conditions */
enum mv_nss_origination_reason_t {
	/*	0x00 .. 0xF - Normal ingress and egress conditions */
	MV_NSS_REASON_UKNOWN = 0x00,                     /* Unspecified reason */
	MV_NSS_REASON_BRIDGE_UC_OTHER,                   /* Ingress bridged unicast, unknown destination client */
	MV_NSS_REASON_BRIDGE_UC_CLIENT,                  /* Ingress bridged unicast, known destination client */
	MV_NSS_REASON_BRIDGE_UC_MYMAC,                   /* Ingress bridged unicast, destination local host */
	MV_NSS_REASON_BRIDGE_MC_OTHER,                   /* Ingress bridged multicast, unknown destination group */
	MV_NSS_REASON_BRIDGE_MC_KNOWN,                   /* Ingress bridged multicast, known destination group */
	MV_NSS_REASON_TUNNEL_MGMT,                       /* Ingress tunneled 802.11 management packet */
	MV_NSS_REASON_TUNNEL_UC_OTHER,                   /* Ingress tunneled unicast, unknown destination client */
	MV_NSS_REASON_TUNNEL_UC_CLIENT,                  /* Ingress tunneled unicast, known destination client */
	MV_NSS_REASON_TUNNEL_MC_MGID,                    /* Ingress tunneled multicast, known MGID */
	MV_NSS_REASON_TUNNEL_MC_OTHER                    /* Ingress tunneled multicast, unknown MGID */
};

enum mv_nss_reason_t {
/*  Ingress drop conditions */
	MV_NSS_REASON_INGRESS_DROP_BASE = 0x0,
	MV_NSS_REASON_INGRESS_DROP_MACSEC_MISSING,       /* MACSEC required, but missing */
	MV_NSS_REASON_INGRESS_DROP_MACSEC_BAD_SA,        /* Unknown MACSEC Security Association */
	MV_NSS_REASON_INGRESS_DROP_MACSEC_BAD_MIC,       /* MACSEC frame has been tampered */
	MV_NSS_REASON_INGRESS_DROP_VLAN_TAG_MISSING,     /* VLAN tag missing, but required */
	MV_NSS_REASON_INGRESS_DROP_VLAN_PORT_BLOCK,      /* VLAN ID not allowed on this port */
	MV_NSS_REASON_INGRESS_DROP_VLAN_UNKNOWN,         /* Unknown VLAN */
	MV_NSS_REASON_INGRESS_DROP_OUTER_SANITY_FAILED,  /* Outer packet consistency checks failed */
	MV_NSS_REASON_INGRESS_DROP_DTLS_MISSING,         /* DTLS required, but missing */
	MV_NSS_REASON_INGRESS_DROP_DTLS_BAD_SA,          /* DTLS present w/o being configured */
	MV_NSS_REASON_INGRESS_DROP_DTLS_BAD_MIC,         /* DTLS frame has been tampered */
	MV_NSS_REASON_INGRESS_DROP_INNER_SANITY_FAILED,  /* Tunneled packet sanity checks failed */
	MV_NSS_REASON_INGRESS_DROP_UCAST_UNKNOWN,        /* Unicast address is unknown */
	MV_NSS_REASON_INGRESS_DROP_UCAST_BRIDGE_BLOCK,   /* Unicast address was tunneled only */
	MV_NSS_REASON_INGRESS_DROP_UCAST_VLAN_BLOCK,     /* Unicast address on wrong VLAN */
	MV_NSS_REASON_INGRESS_DROP_MCAST_TYPE_BLOCK,     /* Multicast type is filtered */
	MV_NSS_REASON_INGRESS_DROP_INNER_UCAST_UNKNOWN,  /* Tunneled Unicast address is unknown */
	MV_NSS_REASON_INGRESS_DROP_INNER_UCAST_BLOCK,    /* Unicast address was bridging only */
	MV_NSS_REASON_INGRESS_DROP_INNER_MGID_UNKNOWN,   /* MGID is unknown */
	MV_NSS_REASON_INGRESS_DROP_CAPWAP_REASM_FAILED,  /* CAPWAP reassembly error */
	MV_NSS_REASON_INGRESS_DROP_UDPLITE_COVRG_LEN,    /* Unexpected value of UDP-Lite coverage length */
	MV_NSS_REASON_INGRESS_DROP_DTLS_PADDING,         /* DTLS padding error */

	/*  Egress drop conditions */
	MV_NSS_REASON_EGRESS_DROP_BASE = 0x1A,
	MV_NSS_REASON_EGRESS_DROP_VLAN_TAG_BLOCK,        /* VLAN tag not allowed */
	MV_NSS_REASON_EGRESS_DROP_UCAST_SRC_UNKNOWN      /* Unknown client in tunnel */
};


/*
 * struct mv_nss_metadata_t
 *
 * Description:
 *       Packet annotation.
 *
 * Fields:
 *       vlan        - Packet VLAN ID.
 *       bssid       - Client BSSID 4 LSbs.
 *       wbid        - CAPWAP WBID field.
 *       outer_8021q - Outer packet PCP for 802.1Q tagged packets.
 *       prio        - Packet queuing priority.
 *       inner_8021e - Inner packet 802.11e priority.
 *       reserved    - Reserved field.
 *       outer_dscp  - Outer packet DSCP for IP packets.
 *       radio_id    - Physical WLAN interface ID.
 *       inner_dscp  - Inner packet DSCP for IP packets.
 *       frm_type    - Frame type: 0 - 802.3, 1 - 802.11.
 *       mgid        - Multicast Group ID.
 *       ulpt        - Up link port type, use mv_nss_dp_port_type_t
 *       dlpt        - Down link port type, use mv_nss_dp_port_type_t
 *       pid         - port ID
 *       pid_hi      - high bits of port ID
 *       pid_lo      - low bits of port ID
 *       port_link   - Physical link ID within source virtual port.
 *       p_info      - Packet info with origination context
 *       reason      - Drop reason
 *       present     - Bit-mask of valid fields in the structure
 *                     (see MV_NSS_DP_MDAT_XXX constants).
 *       ka          - 0 - Data Payload packet.
 *                     1 - Keep-Alive packet.
 *       rssi        - RSSI
 *       snr         - SNR
 *       is_drop     - 1 if packet is dropped otherwise 0
 *       l3_offset   - IPv4 or IPv6 offset in the packet.
 *                                     This field indicates the beginning of
 *                                     Layer3 (in octets).
 *       mac_to_me   - MAC DA lookup results
 *                         0 - MAC DA is unknown (not MAC to me)
 *                         1 - MAC DA is known unicast or multicast MAC address
 *                             (MAC to me)
 *       ip_hdr_len  - IP Header Length in 4-octet words.
 *                                      IPv4: IPv4 header length including options
 *                                            if exist
 *                                      IPv6: IPv6 header length including extension
 *                                            header if exists
 *      l4_info      - L4 parsing results
 *                         0 - Unknown
 *                         1 - TCP
 *                         2 - TCP + checksum error
 *                         3 - UDP
 *                         4 - UDP lite
 *                         5 - UDP + checksum error
 *                         6 - IGMP
 *                         7 - Other
 *       vlan_info    - Number of VLANs in the packet
 *                         0 - Untagged
 *                         1 - Single Tag
 *                         2 - Double tag
 *                         3 - Reserved
 *       mng_pkt      - Management or non-management packet type
 *                         0 - Non-management packet
 *                         1 - Management packet
 *       l2_info      - Indicates type of L2 packet
 *                         0 - Unicast
 *                         1 - Multicast
 *                         2 - IP Multicast
 *                         3 - Broadcast
 *       l3_info      - L3 parsing results
 *                         0 - Unknown
 *                         1 - IPv4
 *                         2 - IPv4 fragment
 *                         3 - IPv4 with options
 *                         4 = IPv4 with errors (checksum, TTL, etc)
 *                         5 = IPv6
 *                         6 = IPv6 with extension(s) header
 *                         7 = ARP
 */

struct mv_nss_metadata_egress_t {
	uint64_t		reserved_0:16;
	uint64_t		frm_type:1;
	uint64_t		reserved_1:1;
	uint64_t		inner_dscp:6;
	uint64_t		radio_id:2;
	uint64_t		outer_dscp:6;
	uint64_t		inner_8021e:3;
	uint64_t		ka:1;
	uint64_t		prio:4;
	uint64_t		outer_8021q:3;
	uint64_t		wbid:5;
	uint64_t		bssid:4;
	uint64_t		vlan:12;

	uint64_t		cookie:16;
	uint64_t		present:10;
	uint64_t		reserved_2:10;
	uint64_t		snr:8;
	uint64_t		rssi:8;
	uint64_t		reserved_3:1;
	uint64_t		pid:5;
	uint64_t		dlpt:3;
	uint64_t		ulpt:3;
};

struct mv_nss_metadata_ingress_t {
	uint64_t		mgid:16;
	uint64_t		frm_type:1;
	uint64_t		reserved_1:1;
	uint64_t		inner_dscp:6;
	uint64_t		radio_id:2;
	uint64_t		outer_dscp:6;
	uint64_t		inner_8021e:3;
	uint64_t		ka:1;
	uint64_t		prio:4;
	uint64_t		outer_8021q:3;
	uint64_t		wbid:5;
	uint64_t		bssid:4;
	uint64_t		vlan:12;

	union {
		struct normal_t {
			uint64_t		cookie:16;
			uint64_t		present:10;
			uint64_t		l3_offset:6;
			uint64_t		ip_hdr_len:5;
			uint64_t		l3_info:3;
			uint64_t		l2_info:2;
			uint64_t		vlan_info:2;
			uint64_t		l4_info:3;
			uint64_t		mng_pkt:1;
			uint64_t		pkt_inf:4;
			uint64_t		is_drop:1; /* must be 0 */
			uint64_t		pid:2;     /* packet ID low bits */
			uint64_t		mac_to_me:1;
			uint64_t		port_link:2;
			uint64_t		dlpt:3;
			uint64_t		ulpt:3;
		} w1_1;

		struct ext_reason_t {
			uint64_t		cookie:16;
			uint64_t		present:10;
			uint64_t		l3_offset:6;
			uint64_t		ip_hdr_len:5;
			uint64_t		l3_info:3;
			uint64_t		l2_info:2;
			uint64_t		vlan_info:2;
			uint64_t		l4_info:3;
			uint64_t		mng_pkt:1;
			uint64_t		reason:4;
			uint64_t		is_drop:1; /* must be 1 */
			uint64_t		pid:2;     /* packet ID low bits */
			uint64_t		mac_to_me:1;
			uint64_t		port_link:2;
			uint64_t		reason_hi:1;
			uint64_t		reserved_2:2;
			uint64_t		ulpt:3;

		} w1_0;
	} u;
};

union mv_nss_metadata_t {
	struct mv_nss_metadata_egress_t  md_egress;
	struct mv_nss_metadata_ingress_t md_ingress;
};

#define MV_NSS_METADATA_LEN sizeof(union mv_nss_metadata_t)

#endif
