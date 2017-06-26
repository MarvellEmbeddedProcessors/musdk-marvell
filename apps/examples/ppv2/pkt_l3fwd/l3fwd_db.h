/* Copyright (c) 2016, Linaro Limited
 * All rights reserved.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 */

#ifndef _L3FWD_DB_MV_H_
#define _L3FWD_DB_MV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <utils.h>

#include "mv_std.h"
#include "src/drivers/ppv2/pp2_plat.h"

#define LPM_FRWD
#ifndef LPM_FRWD
#define IPV6_ENABLED
#endif

/*
 * Atomic 32-bit unsigned integer
 */
struct pp2_atomic_u32_s {
	u32 v; /* Actual storage for the atomic variable */
};

typedef struct pp2_atomic_u32_s pp2_atomic_u32_t;

struct pp2_rwlock_s {
	pp2_atomic_u32_t cnt;
/* < lock count
 * 0 lock not taken
 * -1 write lock taken
 * >0 read lock(s) taken
 */
};

typedef struct pp2_rwlock_s pp2_rwlock_t;

static inline void pp2_atomic_store_rel_u32(pp2_atomic_u32_t *atom, u32 val)
{
	__atomic_store_n(&atom->v, val, __ATOMIC_RELEASE);
}

static inline void pp2_atomic_sub_rel_u32(pp2_atomic_u32_t *atom, u32 val)
{
	(void)__atomic_fetch_sub(&atom->v, val, __ATOMIC_RELEASE);
}

static inline u32 pp2_atomic_load_u32(pp2_atomic_u32_t *atom)
{
	return __atomic_load_n(&atom->v, __ATOMIC_RELAXED);
}

static inline int pp2_atomic_cas_acq_u32(pp2_atomic_u32_t *atom,
				   u32 *old_val, u32 new_val)
{
	return __atomic_compare_exchange_n(&atom->v, old_val, new_val,
					   0 /* strong */,
					   __ATOMIC_ACQUIRE,
					   __ATOMIC_RELAXED);
}

static inline void pp2_atomic_init_u32(pp2_atomic_u32_t *atom, u32 val)
{
	__atomic_store_n(&atom->v, val, __ATOMIC_RELAXED);
}

/**
 * Ethernet MAC address
 */
typedef struct __attribute__((__packed__)) {
	u8 addr[ETH_ALEN]; /* Address */
} pp2h_ethaddr_t;

static inline int pp2h_eth_addr_parse(pp2h_ethaddr_t *mac, const char *str)
{
	int byte[ETH_ALEN];
	int i;

	memset(byte, 0, sizeof(byte));

	if (sscanf(str, "%x:%x:%x:%x:%x:%x",
		   &byte[0], &byte[1], &byte[2],
		   &byte[3], &byte[4], &byte[5]) != ETH_ALEN)
		return -1;

	for (i = 0; i < ETH_ALEN; i++)
		if (byte[i] < 0 || byte[i] > 255)
			return -1;

	mac->addr[0] = byte[0];
	mac->addr[1] = byte[1];
	mac->addr[2] = byte[2];
	mac->addr[3] = byte[3];
	mac->addr[4] = byte[4];
	mac->addr[5] = byte[5];

	return 0;
}

/*
 * Ethernet header
 */
typedef struct __attribute__((__packed__)) {
	pp2h_ethaddr_t dst; /* Destination address */
	pp2h_ethaddr_t src; /* Source address */
	u16 type;   /* EtherType */
} pp2h_ethhdr_t;

/* IPv4 header */
typedef struct __attribute__((__packed__)) {
	u8    ver_ihl;     /* Version / Header length */
	u8    tos;         /* Type of service */
	u16 tot_len;    /* Total length */
	u16 id;         /* ID */
	u16 frag_offset;/* Fragmentation offset */
	u8    ttl;         /* Time to live */
	u8    proto;       /* Protocol */
	u16 chksum;    /* Checksum */
	u32 src_addr;   /* Source address */
	u32 dst_addr;   /* Destination address */
} pp2h_ipv4hdr_t;

/** UDP header */
typedef struct __attribute__((__packed__)) {
	u16 src_port; /* Source port */
	u16 dst_port; /* Destination port */
	u16 length;   /* UDP datagram length in bytes (header+data) */
	u16 chksum;   /* UDP header and data checksum (0 if not used)*/
} pp2h_udphdr_t;

/**
 * IPv6 header
 */
typedef struct __attribute__((__packed__)) {
	u32 ver_tc_flow; /* Version / Traffic class / Flow label */
	u16 payload_len; /* Payload length */
	u8    next_hdr;     /* Next header */
	u8    hop_limit;    /* Hop limit */
	u8    src_addr[16]; /* Source address */
	u8    dst_addr[16]; /* Destination address */
} pp2h_ipv6hdr_t;

#define OIF_LEN 32
#define MAX_DB  65536
#define MAX_STRING  32

/*
 * Max number of flows
 */
#define FWD_MAX_FLOW_COUNT	BIT(16)

/*
 * Default hash entries in a bucket
 */
#define FWD_DEF_BUCKET_ENTRIES	4

/*
 * IPv4 hash key size
 */
#define IPV4_5TUPLE_KEY_SIZE (sizeof(s32) + sizeof(s32) + \
						sizeof(short) + sizeof(short) + sizeof(char))

/*
 * IPv6 hash key size
 */
#define IPV6_5TUPLE_KEY_SIZE (5 * sizeof(u64))

/*
 * IP address range (subnet)
 */
typedef struct ip_addr_range_s {
	u32  addr;     /* IP address, host endianness */
	u32  depth;    /* subnet bit width */
} ip_addr_range_t;

typedef struct ipv6_addr_range_s {
	union ipv6_addr_u {
		struct u64_s {
			u64  ipv6_hi;     /* IP address, host endianness */
			u64  ipv6_lo;     /* IP address, host endianness */
		} u64;
		struct u16_s {
			u16 ipv6_u16[8];
		} u16;
		struct u8_s {
			u8 ipv6_u8[16];
		} u8;
	} addr;
	u32  prefix;		/* subnet bit width */
} ipv6_addr_range_t;

/*
 * TCP/UDP flow
 */
typedef struct tuple5_s {
	union tuple5_u {
		struct ipv4_5t_s {
			s32 src_ip;
			s32 dst_ip;
			short src_port;
			short dst_port;
			char  proto;
			char  pad1;
			short pad2;
#ifdef IPV6_ENABLED
			s64 pad3;
			s64 pad4;
			s64 pad5;
#endif
		} ipv4_5t;
		struct ip_5t_s {
			s64 hi64;
			s64 lo64;
#ifdef IPV6_ENABLED
			s64 pad6;
			s64 pad7;
			s64 pad8;
#endif
		} ip_5t;
#ifdef IPV6_ENABLED
		struct ipv6_5t_s {
			u8 src_ipv6[16];
			u8 dst_ipv6[16];
			short src_port;
			short dst_port;
			char  proto;
			char  pad9;
			short pad10;
		} ipv6_5t;
#endif
	} u5t;
	u8 ip_protocol;	/*PP2H_IPV4 or PP2H_IPV6*/
} tuple5_t __attribute__((aligned(L1_CACHE_LINE_BYTES)));

/*
 * Forwarding data base entry
 */
typedef struct fwd_db_entry_s {
	struct fwd_db_entry_s *next;          /* Next entry on list */
	char				oif[OIF_LEN]; /* Output interface name */
	u8				oif_id;	      /* Output interface idx */
	pp2h_ethaddr_t		src_mac;      /* Output source MAC */
	pp2h_ethaddr_t		dst_mac;      /* Output destination MAC */
#ifndef LPM_FRWD
	union ip_hdr_u {
		struct ipv4_s {
			ip_addr_range_t		src_subnet;  /*subnet previously*/     /* Subnet for this router */
			ip_addr_range_t		dst_subnet;
			u16			src_port;
			u16			dst_port;
			u8				protocol;	/*0 - UDP, 1 - TCP */
		} ipv4;
		struct ipv6_s {
			ipv6_addr_range_t	src_subnet;	/*subnet previously*/     /* Subnet for this router */
			ipv6_addr_range_t	dst_subnet;
			u16			src_port;
			u16			dst_port;
			u8				protocol;		/*0 - UDP, 1 - TCP */
		} ipv6;
	} u;
	u8 ip_protocol;	/*PP2H_IPV4 or PP2H_IPV6*/
#else
	ip_addr_range_t		subnet;
#endif
} fwd_db_entry_t;

/**
 * Forwarding data base
 */
typedef struct fwd_db_s {
	u32          index;          /* Next available entry */
	fwd_db_entry_t   *list;           /* List of active routes */
	fwd_db_entry_t    array[MAX_DB];  /* Entry storage */
} fwd_db_t;

/* Global pointer to fwd db */
extern fwd_db_t *fwd_db;

/*
 * Initialize FWD DB
 */
void init_fwd_db(void);

/*
 * Initialize forward lookup cache based on hash
 */
void init_fwd_hash_cache(void);

/*
 * Create a forwarding database entry
 *
 * String is of the format "SubNet,Intf,NextHopMAC"
 *
 * @param input  Pointer to string describing route
 * @param oif  Pointer to out interface name, as a return value
 * @param dst_mac  Pointer to dest mac for output packet, as a return value
 *
 * @return 0 if successful else -1
 */
int create_fwd_db_entry(char *input, char **oif, u8 **dst_mac);

/*
 * Scan FWD DB entries and resolve output queue and source MAC address
 *
 * @param intf   Interface name string
 * @param portid Output queue for packet transmit
 * @param mac    MAC address of this interface
 */
void resolve_fwd_db(char *intf, int portid, u8 *mac);

/*
 * Display one forwarding database entry
 *
 * @param entry  Pointer to entry to display
 */
void dump_fwd_db_entry(fwd_db_entry_t *entry);

/*
 * Display the forwarding database
 */
void dump_fwd_db(void);

/*
 * Find a matching forwarding database entry
 *
 * @param key  ipv4/ipv6 tuple
 *
 * @return pointer to forwarding DB entry else NULL
 */
fwd_db_entry_t *find_fwd_db_entry(tuple5_t *key);

/*
 * Parse text string representing an IPv4 address or subnet
 *
 * String is of the format "XXX.XXX.XXX.XXX(/W)" where
 * "XXX" is decimal value and "/W" is optional subnet length
 *
 * @param ipaddress  Pointer to IP address/subnet string to convert
 * @param addr       Pointer to return IPv4 address, host endianness
 * @param depth      Pointer to subnet bit width
 * @return 0 if successful else -1
 */
int parse_ipv4_string(char *ipaddress, u32 *addr, u32 *depth);

/*
 * Parse text string representing an IPv6 address or subnet
 *
 * String is of the format "XXXX:XXXX:XXXX:XXXX:XXXX:XXXX:XXXX:XXXX(/W)" where
 * "XXXX" is heximal value and "/W" is optional subnet length
 * Or condensed notation: XXXX:XXXX:XXXX:XXXX::(0/W)
 *
 * @param ipaddress  Pointer to IP address/subnet string to convert
 * @param addr_hi    Pointer to return high 64B of IPv6 address, host endianness
 * @param addr_lo    Pointer to return low 64B of IPv6 address, host endianness
 * @param depth      Pointer to subnet bit width
 * @return 0 if successful else -1
 */
int parse_ipv6_string(char *ipaddress, u64 *addr_hi, u64 *addr_lo, u32 *depth);

#ifdef __cplusplus
}
#endif

#endif
