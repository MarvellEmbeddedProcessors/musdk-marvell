/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __NET_H__
#define __NET_H__

#define PPP_PROTO_IPV4		0x21
#define PPP_PROTO_IPV6		0x57
#define ARP_PROTO		0x806	/* Address Resolution packet	*/
#define PPPOE_PROTO		0x8864	/* PPPoE packet	*/
#define MV_ETHADDR_LEN		6	/**< Layer 2 ethernet address length */
#define MV_ETHHDR_LEN		14	/**< Ethernet header length */
#define MV_IPV4ADDR_LEN		4	/**< IP version 4 address length */
#define MV_IPV6ADDR_LEN		16	/**< IP version 6 address length */
#define MV_IP_VER_4		4	/**< IP version 4, version number */
#define MV_IP_VER_6		6	/**< IP version 6, version number */
#define MV_IPV4_HL_MIN		5	/**< IP version 4, header length in words */
#define MV_IP_PROTO_NH_LEN	1	/**< IP protocol/next-header length */
#define MV_L4_PORT_LEN		2	/**< L4 port length */

/** IPv4 header */
struct mv_ipv4hdr {
#ifdef __BIG_ENDIAN
	u8	version:4, ihl:4;
#else
	u8	ihl:4, version:4;
#endif
	u8	dscp_ecn;		/**< DSCP / ECN */
	u16	total_len;		/**< Total length */
	u16	id;			/**< ID */
	u16	frag_offset;		/**< Fragmentation offset */
	u8	ttl;			/**< Time to live */
	u8	proto;			/**< Protocol */
	u16	chksum;			/**< Checksum */
	u8	src_addr[MV_IPV4ADDR_LEN];	/**< Source address as array of Bytes */
	u8	dst_addr[MV_IPV4ADDR_LEN];	/**< Destination address as array of Bytes */
} __packed;

/** IPv6 header */
struct mv_ipv6hdr {
#ifdef __BIG_ENDIAN
	u8	version:4, priority:4;
#else
	u8	priority:4, version:4;
#endif
	u8	flow_lbl[3];
	u16	pl_len;
	u8	next_header;
	u8	hop_limit;
	u8	src_addr[MV_IPV6ADDR_LEN];
	u8	dst_addr[MV_IPV6ADDR_LEN];
} __packed;

/** UDP header */
struct mv_udphdr {
	u16	src_port; /**< Source port */
	u16	dst_port; /**< Destination port */
	u16	length;   /**< UDP datagram length in bytes (header+data) */
	u16	chksum;   /**< UDP header and data checksum (0 if not used) */
} __packed;

union mv_ip_addr {
	u32 ip4;	/* IPv4 Address */
	u32 ip6[4];	/* IPv6 Address */
};

struct mv_2tuple {
	u8			ip_ver;		/* IP version (4,6) */
	u8			reserved[3];
	union mv_ip_addr	ip_saddr;	/* Source IP address */
	union mv_ip_addr	ip_daddr;	/* Destination IP address */
};

struct mv_5tuple {
	u8			ip_ver;		/* IP version (4,6) */
	u8			l4_proto;	/* L4 protocol type, udp/tcp */
	u16			reserved;
	u16			l4_sport;	/* Source Port */
	u16			l4_dport;	/* Destination Port */
	union mv_ip_addr	ip_saddr;	/* Source IP address */
	union mv_ip_addr	ip_daddr;	/* Destination IP address */
};

static inline bool mv_check_eaddr_mc(const u8 *eaddr)
{
	u16 e_16 = *(const u16 *)eaddr;
#ifdef __BIG_ENDIAN
	return 0x01 & e_16;
#else
	return 0x01 & (e_16 >> ((sizeof(e_16) * 8) - 8));
#endif
}

static inline bool mv_check_eaddr_uc(const u8 *addr)
{
	return !mv_check_eaddr_mc(addr);
}

static inline bool mv_check_eaddr_bc(const u8 *eaddr)
{
	return (*(const u16 *)(eaddr + 0) &
		*(const u16 *)(eaddr + 2) &
		*(const u16 *)(eaddr + 4)) == 0xffff;
}

static inline int mv_check_eaddr_zero(const u8 *eaddr)
{
	return !(eaddr[0] | eaddr[1] | eaddr[2] | eaddr[3] | eaddr[4] | eaddr[5]);
}

static inline bool mv_eaddr_identical(const u8 *eaddr1, const u8 *eaddr2)
{
	const u16 *e1_16 = (const u16 *)eaddr1;
	const u16 *e2_16 = (const u16 *)eaddr2;

	return ((e1_16[0] ^ e2_16[0]) | (e1_16[1] ^ e2_16[1]) | (e1_16[2] ^ e2_16[2])) == 0;
}

static inline bool mv_eaddr_identical_except_4LSB(const u8 *eaddr1,
						  const u8 *eaddr2)
{
	return (((eaddr1[5] & 0xF0) == (eaddr2[5] & 0xF0)) &
		(eaddr1[4] == eaddr2[4]) & (eaddr1[3] == eaddr2[3]) &
		(eaddr1[2] == eaddr2[2]) & (eaddr1[1] == eaddr2[1]) &
		(eaddr1[0] == eaddr2[0]));
}

static inline int mv_check_eaddr_valid(const u8 *addr)
{
	return !mv_check_eaddr_mc(addr) && !mv_check_eaddr_zero(addr);
}

static inline void mv_cp_eaddr(u8 *dest, const u8 *source)
{
	 u16 *dst_16 = (u16 *)dest;
	 const u16 *src_16 = (const u16 *)source;

	 dst_16[0] = src_16[0];
	 dst_16[1] = src_16[1];
	 dst_16[2] = src_16[2];
}

static inline u16 mv_add_csum16(u16 csum, u16 add)
{
	csum += add;
	return (csum + (csum < add));
}

static inline u16 mv_sub_csum16(u16 csum, u16 sub)
{
	return mv_add_csum16(csum, ~sub);
}

/* Calculate 16 bits checksum optimized for network protocols
 * "buf" must be 16 bits aligned.
 * "shorts" is buffer size in u16 units
 */
static inline u16 mv_calc_csum16(const u16 *buf, u32 shorts)
{
	int i;
	u16 csum = 0;

	for (i = 0; i < shorts; i++) {
		csum = mv_add_csum16(csum, *buf);
		buf++;
	}
	return csum;
}

/* Calculate IPv4 checksum. "ihl" is IP header length in words.
 * To generate new checksum "csum" field in IP header must be cleared before
 * To verify checksum: calculated checksum must be 0
 */
static inline u16 mv_ip4_csum(const u16 *iph, u32 ihl)
{
	return ~mv_calc_csum16(iph, ihl * 2);
}

#endif /* __NET_H__ */
