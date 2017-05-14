/* Copyright (c) 2016, Linaro Limited
 * All rights reserved.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "byteorder_inlines.h"
#include "l3fwd_db.h"
#include "xxhash.h"
#include "lib/net.h"

inline void pp2_rwlock_read_lock(pp2_rwlock_t *rwlock)
{
	u32 cnt;
	int  is_locked = 0;

	while (is_locked == 0) {
		cnt = pp2_atomic_load_u32(&rwlock->cnt);
		/* waiting for read lock */
		if ((int32_t)cnt < 0)
			continue;

		is_locked = pp2_atomic_cas_acq_u32(&rwlock->cnt,
						   &cnt, cnt + 1);
	}
}

inline void pp2_rwlock_write_unlock(pp2_rwlock_t *rwlock)
{
	pp2_atomic_store_rel_u32(&rwlock->cnt, 0);
}

inline void pp2_rwlock_write_lock(pp2_rwlock_t *rwlock)
{
	u32 cnt;
	int is_locked = 0;

	while (is_locked == 0) {
		u32 zero = 0;

		cnt = pp2_atomic_load_u32(&rwlock->cnt);
		/* lock acquired, wait */
		if (cnt != 0)
			continue;

		is_locked = pp2_atomic_cas_acq_u32(&rwlock->cnt,
						   &zero, (u32)-1);
	}
}

inline void pp2_rwlock_read_unlock(pp2_rwlock_t *rwlock)
{
	pp2_atomic_sub_rel_u32(&rwlock->cnt, 1);
}

inline void pp2_rwlock_init(pp2_rwlock_t *rwlock)
{
	pp2_atomic_init_u32(&rwlock->cnt, 0);
}

/** Jenkins hash support.
  *
  * Copyright (C) 2006 Bob Jenkins (bob_jenkins@burtleburtle.net)
  *
  * http://burtleburtle.net/bob/hash/
  *
  * These are the credits from Bob's sources:
  *
  * lookup3.c, by Bob Jenkins, May 2006, Public Domain.
  *
  * These are functions for producing 32-bit hashes for hash table lookup.
  * hashword(), hashlittle(), hashlittle2(), hashbig(), mix(), and final()
  * are externally useful functions.  Routines to test the hash are included
  * if SELF_TEST is defined.  You can use this free for any purpose.  It's in
  * the public domain.  It has no warranty.
  *
  * $FreeBSD$
  */
#define JHASH_GOLDEN_RATIO	0x9e3779b9
#define rot(x, k) (((x) << (k)) | ((x) >> (32 - (k))))
#define FWD_BJ3_MIX(a, b, c) \
{ \
	a -= c; a ^= rot(c, 4); c += b; \
	b -= a; b ^= rot(a, 6); a += c; \
	c -= b; c ^= rot(b, 8); b += a; \
	a -= c; a ^= rot(c, 16); c += b; \
	b -= a; b ^= rot(a, 19); a += c; \
	c -= b; c ^= rot(b, 4); b += a; \
}

/**
 * Compute hash value from a flow
 */
static inline
u64 l3fwd_calc_hash(tuple5_t *key)
{
#ifdef LPM_FRWD
	u64 l4_ports = 0;
	u32 dst_ip, src_ip;

	src_ip = key->u5t.ipv4_5t.src_ip;
	dst_ip = key->u5t.ipv4_5t.dst_ip + JHASH_GOLDEN_RATIO;
	FWD_BJ3_MIX(src_ip, dst_ip, l4_ports);
	return l4_ports;
#else
#ifdef IPV6_ENABLED
	int key_size = (key->ip_protocol == IP_VERSION_4) ? IPV4_5TUPLE_KEY_SIZE : IPV6_5TUPLE_KEY_SIZE;

	return XXH_fast32((void *)key, key_size, 0);
#else
	return XXH_fast32((void *)key, IPV4_5TUPLE_KEY_SIZE, 0);
#endif
#endif
}

/**
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
int parse_ipv4_string(char *ipaddress, u32 *addr, u32 *depth)
{
	int b[4];
	int qualifier = 32;
	int converted;
	u32 addr_le;

	if (strchr(ipaddress, '/')) {
		converted = sscanf(ipaddress, "%d.%d.%d.%d/%d",
				   &b[3], &b[2], &b[1], &b[0],
				   &qualifier);
		if (converted != 5)
			return -1;
	} else {
		converted = sscanf(ipaddress, "%d.%d.%d.%d",
				   &b[3], &b[2], &b[1], &b[0]);
		if (converted != 4)
			return -1;
	}

	if ((b[0] > 255) || (b[1] > 255) || (b[2] > 255) || (b[3] > 255))
		return -1;
	if (!qualifier || (qualifier > 32))
		return -1;

	addr_le = b[0] | b[1] << 8 | b[2] << 16 | b[3] << 24;
	*addr = pp2_le_to_cpu_32(addr_le);
	*depth = qualifier;

	return 0;
}

/**
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
int parse_ipv6_string(char *ipaddress, u64 *addr_hi, u64 *addr_lo, u32 *depth)
{
	int group;
	int j;
	char *sptr = ipaddress;
	char *endptr;
	int b[8];
	int double_sep = 0;
	int qualifier = 128;

	for (group = 0; group < 7; group++) {
		b[group] = strtol(sptr, &endptr, 16);
		if ((b[group] < 0) || (b[group] > 0xFFFF) || !(*endptr))
			return -1;

		if (*endptr != ':')
			return -1;

		++endptr;
		if (!(*endptr))
			return -1;

		if (*endptr == ':') {
			/* found '::' */
			double_sep = 1;
			++endptr;
			break;
		}
		sptr = endptr;
	}

	if (double_sep) {
		for (j = group + 1; j < 8; j++)
			b[j] = 0;

		if (*endptr) {
			if (*endptr != '0')
				return -1;

			endptr++;
			if (!(*endptr))
				return -1;
		}
	} else {
		b[7] = strtol(sptr, &endptr, 16);
		if ((b[7] < 0) || (b[7] > 0xFFFF))
			return -1;
	}

	if (*endptr) {
		/* prefix should follow */
		if (*endptr != '/')
			return -1;

		endptr++;
		if (!(*endptr))
			return -1;

		qualifier = strtol(endptr, NULL, 10);
	}

	if ((qualifier < 0) || (qualifier > 128))
		return -1;

	*addr_hi = (u64)pp2_cpu_to_be_16(b[0]) | (u64)pp2_cpu_to_be_16(b[1]) << 16 |
			(u64)pp2_cpu_to_be_16(b[2]) << 32 | (u64)pp2_cpu_to_be_16(b[3]) << 48;
	*addr_lo = (u64)pp2_cpu_to_be_16(b[4]) | (u64)pp2_cpu_to_be_16(b[5]) << 16 |
			(u64)pp2_cpu_to_be_16(b[6]) << 32 | (u64)pp2_cpu_to_be_16(b[7]) << 48;

	*depth = qualifier;
	return 0;
}

/**
 * Generate text string representing IPv4 range/subnet, output
 * in "XXX.XXX.XXX.XXX/W" format
 *
 * @param b     Pointer to buffer to store string
 * @param range Pointer to IPv4 address range
 *
 * @return Pointer to supplied buffer
 */
static inline
char *ipv4_subnet_str(char *b, ip_addr_range_t *range)
{
	sprintf(b, "%d.%d.%d.%d/%d",
		0xFF & ((range->addr) >> 24),
		0xFF & ((range->addr) >> 16),
		0xFF & ((range->addr) >>  8),
		0xFF & ((range->addr) >>  0),
		range->depth);
	return b;
}

/**
 * Generate text string representing IPv6 range/subnet, output
 * in "XXXX:XXXX:XXXX:XXXX:XXXX:XXXX:XXXX:XXXX/W" format
 *
 * @param b     Pointer to buffer to store string
 * @param range Pointer to IPv6 address range
 *
 * @return Pointer to supplied buffer
 */
static inline
char *ipv6_subnet_str(char *b, ipv6_addr_range_t *range)
{
	u64 temp_hi, temp_lo;

	temp_hi = pp2_cpu_to_be_64(range->addr.u64.ipv6_hi);
	temp_lo = pp2_cpu_to_be_64(range->addr.u64.ipv6_lo);

	sprintf(b, "%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x/%d",
		(u32)(0xFFFF & (temp_hi >> 48)),
		(u32)(0xFFFF & (temp_hi >> 32)),
		(u32)(0xFFFF & (temp_hi >> 16)),
		(u32)(0xFFFF & (temp_hi >> 0)),
		(u32)(0xFFFF & (temp_lo >> 48)),
		(u32)(0xFFFF & (temp_lo >> 32)),
		(u32)(0xFFFF & (temp_lo >> 16)),
		(u32)(0xFFFF & (temp_lo >> 0)),
		range->prefix);
	return b;
}

/**
 * Generate text string representing MAC address
 *
 * @param b     Pointer to buffer to store string
 * @param mac   Pointer to MAC address
 *
 * @return Pointer to supplied buffer
 */
static inline
char *mac_addr_str(char *b, pp2h_ethaddr_t *mac)
{
	u8 *byte;

	byte = mac->addr;
	sprintf(b, "%02X:%02X:%02X:%02X:%02X:%02X",
		byte[0], byte[1], byte[2], byte[3], byte[4], byte[5]);
	return b;
}

/**
 * Flow cache table entry
 */
typedef struct flow_entry_s {
	tuple5_t key;		/**< match key */
	struct flow_entry_s *next;	/**< next entry in the bucket */
	fwd_db_entry_t *fwd_entry;	/**< entry info in db */
} flow_entry_t;

/**
 * Flow cache table bucket
 */
typedef struct flow_bucket_s {
	pp2_rwlock_t	lock;	/**< Bucket lock*/
	flow_entry_t	*next;	/**< First flow entry in bucket*/
} flow_bucket_t;

/**
 * Flow hash table, fast lookup cache
 */
typedef struct flow_table_s {
	pp2_rwlock_t flow_lock;	/**< flow table lock*/
	flow_entry_t *flows;	/**< flow store */
	flow_bucket_t *bucket;	/**< bucket store */
	u32 bkt_cnt;
	u32 flow_cnt;
	u32 next_flow;	/**< next available flow in the store */
} flow_table_t;

static flow_table_t fwd_lookup_cache;

static void create_fwd_hash_cache(void)
{
	flow_bucket_t		*bucket;
	flow_entry_t		*flows;
	u32		bucket_count, flow_count, size;
	u32		i;

	flow_count = FWD_MAX_FLOW_COUNT;
	bucket_count = flow_count / FWD_DEF_BUCKET_ENTRIES;

	/* Reserve memory for Routing hash table */
	size = sizeof(flow_bucket_t) * bucket_count +
		sizeof(flow_entry_t) * flow_count;

	bucket = (flow_bucket_t *)malloc(size);
	if (!bucket) {
		/* Try the second time with small request */
		flow_count /= 4;
		bucket_count = flow_count / FWD_DEF_BUCKET_ENTRIES;
		size = sizeof(flow_bucket_t) * bucket_count +
			sizeof(flow_entry_t) * flow_count;

		bucket = (flow_bucket_t *)malloc(size);
		if (!bucket) {
			printf("Error: shared mem alloc failed.\n");
			exit(-1);
		}
	}

	size = sizeof(flow_bucket_t) * bucket_count;
	flows = (flow_entry_t *)(void *)((char *)bucket + size);

	fwd_lookup_cache.bucket = bucket;
	fwd_lookup_cache.bkt_cnt = bucket_count;
	fwd_lookup_cache.flows = flows;
	fwd_lookup_cache.flow_cnt = flow_count;

	/*Initialize bucket locks*/
	for (i = 0; i < bucket_count; i++) {
		bucket = &fwd_lookup_cache.bucket[i];
		pp2_rwlock_init(&bucket->lock);
		bucket->next = NULL;
	}

	memset(flows, 0, sizeof(flow_entry_t) * flow_count);
	pp2_rwlock_init(&fwd_lookup_cache.flow_lock);
	fwd_lookup_cache.next_flow = 0;
}

static inline flow_entry_t *get_new_flow(void)
{
	u32 next;
	flow_entry_t *flow = NULL;

	pp2_rwlock_write_lock(&fwd_lookup_cache.flow_lock);
	next = fwd_lookup_cache.next_flow;
	if (next < fwd_lookup_cache.flow_cnt) {
		flow = &fwd_lookup_cache.flows[next];
		fwd_lookup_cache.next_flow++;
	}
	pp2_rwlock_write_unlock(&fwd_lookup_cache.flow_lock);

	return flow;
}

static inline
int match_key_flow(tuple5_t *key, flow_entry_t *flow)
{
#if defined(LPM_FRWD) || !defined(IPV6_ENABLED)
	if (key->u5t.ip_5t.hi64 == flow->key.u5t.ip_5t.hi64 &&
	    key->u5t.ip_5t.lo64 == flow->key.u5t.ip_5t.lo64)
		return 1;
#else
	if (key->u5t.ip_5t.hi64 == flow->key.u5t.ip_5t.hi64 &&
	    key->u5t.ip_5t.lo64 == flow->key.u5t.ip_5t.lo64 &&
	    key->u5t.ip_5t.pad6 == flow->key.u5t.ip_5t.pad6 &&
	    key->u5t.ip_5t.pad7 == flow->key.u5t.ip_5t.pad7 &&
	    key->u5t.ip_5t.pad8 == flow->key.u5t.ip_5t.pad8)
		return 1;
#endif
	return 0;
}

static inline
flow_entry_t *lookup_fwd_cache(tuple5_t *key, flow_bucket_t *bucket)
{
	flow_entry_t *rst;

	pp2_rwlock_read_lock(&bucket->lock);
	for (rst = bucket->next; rst; rst = rst->next) {
		if (match_key_flow(key, rst))
			break;
	}
	pp2_rwlock_read_unlock(&bucket->lock);

	return rst;
}

static inline
flow_entry_t *insert_fwd_cache(tuple5_t *key,
			       flow_bucket_t *bucket,
			       fwd_db_entry_t *entry)
{
	flow_entry_t *flow;

	if (!entry)
		return NULL;

	flow = get_new_flow();
	if (!flow)
		return NULL;

	flow->key = *key;
	flow->fwd_entry = entry;

	pp2_rwlock_write_lock(&bucket->lock);
	if (bucket->next)
		flow->next = bucket->next;
	bucket->next = flow;
	pp2_rwlock_write_unlock(&bucket->lock);

	return flow;
}

void init_fwd_hash_cache(void)
{
	fwd_db_entry_t *entry;
	flow_entry_t *flow;
	flow_bucket_t *bucket;
	u64 hash;
	u32 i, nb_hosts;
	tuple5_t key;
	int counter = 0;

#ifndef LPM_FRWD
	u32 j, dst_nb_hosts;
#endif
#ifdef IPV6_ENABLED
	u16 src_tmp, dst_tmp;
#endif

	create_fwd_hash_cache();
	/**
	 * warm up the lookup cache with possible hosts.
	 * with millions flows, save significant time during runtime.
	 */

	memset(&key, 0, sizeof(key));

#ifdef LPM_FRWD
	for (entry = fwd_db->list; entry; entry = entry->next) {
		nb_hosts = 1 << (32 - entry->subnet.depth);
		for (i = 0; i < nb_hosts; i++) {
			key.u5t.ipv4_5t.dst_ip = entry->subnet.addr + i;
			hash = l3fwd_calc_hash(&key);
			hash &= fwd_lookup_cache.bkt_cnt - 1;
			bucket = &fwd_lookup_cache.bucket[hash];
			flow = lookup_fwd_cache(&key, bucket);
			if (flow)
				return;

			flow = insert_fwd_cache(&key, bucket, entry);
			if (!flow)
				goto out;
			counter++;

			if (counter >= FWD_MAX_FLOW_COUNT) {
				printf("Reached the maximum number of DB flows\n");
				goto out;
			}
		}
	}
#else
	for (entry = fwd_db->list; entry; entry = entry->next) {
		if (likely(entry->ip_protocol == IP_VERSION_4)) {
			nb_hosts = 1 << (32 - entry->u.ipv4.src_subnet.depth);
			dst_nb_hosts = 1 << (32 - entry->u.ipv4.dst_subnet.depth);
			for (i = 0; i < nb_hosts; i++) {
				for (j = 0; j < dst_nb_hosts; j++) {
					memset(&key, 0, sizeof(key));
					key.u5t.ipv4_5t.src_ip = entry->u.ipv4.src_subnet.addr + i;
					key.u5t.ipv4_5t.dst_ip = entry->u.ipv4.dst_subnet.addr + j;
					key.u5t.ipv4_5t.src_port = entry->u.ipv4.src_port;
					key.u5t.ipv4_5t.dst_port = entry->u.ipv4.dst_port;
					key.u5t.ipv4_5t.proto = entry->u.ipv4.protocol;
					key.ip_protocol = IP_VERSION_4;
					hash = l3fwd_calc_hash(&key);
					hash &= fwd_lookup_cache.bkt_cnt - 1;
					bucket = &fwd_lookup_cache.bucket[hash];
					flow = lookup_fwd_cache(&key, bucket);
					if (flow)
						return;

					flow = insert_fwd_cache(&key, bucket, entry);
					if (!flow)
						goto out;
					counter++;

					if (counter >= FWD_MAX_FLOW_COUNT) {
						printf("Reached the maximum number of DB flows\n");
						goto out;
					}
				}
			}
		} else {
#ifdef IPV6_ENABLED
			/*IP_VERSION_6*/
			nb_hosts = 1 << (128 - entry->u.ipv6.src_subnet.prefix);
			dst_nb_hosts = 1 << (128 - entry->u.ipv6.dst_subnet.prefix);
			for (i = 0; i < nb_hosts; i++) {
				for (j = 0; j < dst_nb_hosts; j++) {
					memcpy(&key.u5t.ipv6_5t.src_ipv6,
					       &entry->u.ipv6.src_subnet.addr.u8.ipv6_u8, IPV6_ADDR_LEN);
					memcpy(&key.u5t.ipv6_5t.dst_ipv6,
					       &entry->u.ipv6.dst_subnet.addr.u8.ipv6_u8, IPV6_ADDR_LEN);

					if (((u32)pp2_cpu_to_be_16(entry->u.ipv6.src_subnet.addr.u16.ipv6_u16[7]) + i) >
					     0xFFFF)
						break;
					if (((u32)pp2_cpu_to_be_16(entry->u.ipv6.dst_subnet.addr.u16.ipv6_u16[7]) + j) >
					     0xFFFF)
						continue;

					src_tmp = pp2_cpu_to_be_16(entry->u.ipv6.src_subnet.addr.u16.ipv6_u16[7]) + i;
					dst_tmp = pp2_cpu_to_be_16(entry->u.ipv6.dst_subnet.addr.u16.ipv6_u16[7]) + j;
					key.u5t.ipv6_5t.src_ipv6[14] = (src_tmp >> 8) & 0xFF;
					key.u5t.ipv6_5t.dst_ipv6[14] = (dst_tmp >> 8) & 0xFF;
					key.u5t.ipv6_5t.src_ipv6[15] = src_tmp & 0xFF;
					key.u5t.ipv6_5t.dst_ipv6[15] = dst_tmp & 0xFF;
					key.u5t.ipv6_5t.src_port = entry->u.ipv6.src_port;
					key.u5t.ipv6_5t.dst_port = entry->u.ipv6.dst_port;
					key.u5t.ipv6_5t.proto = entry->u.ipv6.protocol;
					key.ip_protocol = IP_VERSION_6;

					hash = l3fwd_calc_hash(&key);
					hash &= fwd_lookup_cache.bkt_cnt - 1;
					bucket = &fwd_lookup_cache.bucket[hash];
					flow = lookup_fwd_cache(&key, bucket);
					if (flow)
						return;

					flow = insert_fwd_cache(&key, bucket, entry);
					if (!flow)
						goto out;
					counter++;

					if (counter >= FWD_MAX_FLOW_COUNT) {
						printf("Reached the maximum number of DB flows\n");
						goto out;
					}
				}
			}
#endif
		}
	}
#endif
out:
	printf("init_fwd_hash_cache created %d entries\n", counter);
}

/** Global pointer to fwd db */
fwd_db_t *fwd_db;

void init_fwd_db(void)
{
	fwd_db = (fwd_db_t *)malloc(sizeof(*fwd_db));

	if (!fwd_db) {
		printf("Error: shared mem alloc failed.\n");
		exit(EXIT_FAILURE);
	}
	memset(fwd_db, 0, sizeof(*fwd_db));
}

int create_fwd_db_entry(char *input, char **oif, u8 **dst_mac)
{
	int pos = 0;
	char *local;
	char *str;
	char *save;
	char *token;
#ifndef LPM_FRWD
	int protocol;
#endif
	fwd_db_entry_t *entry = &fwd_db->array[fwd_db->index];

	*oif = NULL;
	*dst_mac = NULL;

	/* Verify we haven't run out of space */
	if (fwd_db->index >= MAX_DB) {
		printf("create_fwd_db_entry: out of space\n");
		return -1;
	}

	/* Make a local copy */
	local = malloc(strlen(input) + 1);
	if (!local) {
		printf("create_fwd_db_entry: malloc failed\n");
		return -1;
	}
	strcpy(local, input);

	/* Setup for using "strtok_r" to search input string */
	str = local;
	save = NULL;

	/* Parse tokens separated by ',' */
	while (NULL != (token = strtok_r(str, ",", &save))) {
		str = NULL;  /* reset str for subsequent strtok_r calls */

		/* Parse token based on its position */
		switch (pos) {
#ifdef LPM_FRWD
		case 0:
			if (parse_ipv4_string(token, &entry->subnet.addr,
					      &entry->subnet.depth) == -1) {
				printf("create_fwd_db_entry: invalid IP address\n");
				return -1;
			}
			break;
		case 1:
			strncpy(entry->oif, token, OIF_LEN - 1);
			entry->oif[OIF_LEN - 1] = 0;
			*oif = entry->oif;
			break;
		case 2:
			if (pp2h_eth_addr_parse(&entry->dst_mac, token) != 0) {
				printf("create_fwd_db_entry: invalid Mac address\n");
				return -1;
			}
			*dst_mac = entry->dst_mac.addr;
			break;
#else
		case 0:
			if (parse_ipv4_string(token, &entry->u.ipv4.src_subnet.addr,
					      &entry->u.ipv4.src_subnet.depth) == -1) {
				if (parse_ipv6_string(token,
						      &entry->u.ipv6.src_subnet.addr.u64.ipv6_hi,
						      &entry->u.ipv6.src_subnet.addr.u64.ipv6_lo,
						      &entry->u.ipv6.src_subnet.prefix) == -1) {
					printf("create_fwd_db_entry: invalid SrcIp\n");
					return -1;
				}
				entry->ip_protocol = IP_VERSION_6;
			} else {
				entry->ip_protocol = IP_VERSION_4;
			}
			break;
		case 1:
			if (entry->ip_protocol == IP_VERSION_4) {
				if (parse_ipv4_string(token, &entry->u.ipv4.dst_subnet.addr,
						      &entry->u.ipv4.dst_subnet.depth) == -1) {
					printf("create_fwd_db_entry: invalid DestIp\n");
					return -1;
				}
			} else {
				if (parse_ipv6_string(token,
						      &entry->u.ipv6.dst_subnet.addr.u64.ipv6_hi,
						      &entry->u.ipv6.dst_subnet.addr.u64.ipv6_lo,
						      &entry->u.ipv6.dst_subnet.prefix) == -1) {
					printf("create_fwd_db_entry: invalid DestIp\n");
					return -1;
				}
			}
			break;
		case 2:
			if (entry->ip_protocol == IP_VERSION_4)
				entry->u.ipv4.src_port = atoi(token);
			else
				entry->u.ipv6.src_port = atoi(token);
			break;
		case 3:
			if (entry->ip_protocol == IP_VERSION_4)
				entry->u.ipv4.dst_port = atoi(token);
			else
				entry->u.ipv6.dst_port = atoi(token);
			break;
		case 4:
			protocol = atoi(token);
			if (protocol > 1) {
				printf("create_fwd_db_entry: Invalid protocol\n");
				return -1;
			}
			protocol = (protocol == 0) ? IP_PROTOCOL_UDP : IP_PROTOCOL_TCP;
			if (entry->ip_protocol == IP_VERSION_4)
				entry->u.ipv4.protocol = protocol;
			else
				entry->u.ipv6.protocol = protocol;
			break;
		case 5:
			strncpy(entry->oif, token, OIF_LEN - 1);
			entry->oif[OIF_LEN - 1] = 0;
			*oif = entry->oif;
			break;
		case 6:
			if (pp2h_eth_addr_parse(&entry->dst_mac, token) != 0) {
				printf("create_fwd_db_entry: invalid Mac address\n");
				return -1;
			}
			*dst_mac = entry->dst_mac.addr;
			break;
#endif
		default:
			printf("ERROR: extra token \"%s\" at position %d\n",
			       token, pos);
			return -1;
		}

		/* Advance to next position */
		pos++;
	}

#ifndef LPM_FRWD
#ifndef IPV6_ENABLED
	if (entry->ip_protocol == IP_VERSION_6) {
		printf("IPv6 support is turned off\n");
		return -1;
	}
#endif
#endif

	/* Add route to the list */
	fwd_db->index++;
	entry->next = fwd_db->list;
	fwd_db->list = entry;

	free(local);
	return 0;
}

void resolve_fwd_db(char *intf, int portid, u8 *mac)
{
	fwd_db_entry_t *entry;

	/* Walk the list and attempt to set output and MAC */
	for (entry = fwd_db->list; entry; entry = entry->next) {
		if (strcmp(intf, entry->oif))
			continue;

		entry->oif_id = portid;
		mv_cp_eaddr(entry->src_mac.addr, mac);
	}
}

void dump_fwd_db_entry(fwd_db_entry_t *entry)
{
	char subnet_str[8 * MAX_STRING];
	char mac_str[MAX_STRING];
#ifndef LPM_FRWD
	char dst_subnet_str[8 * MAX_STRING];
#endif

	mac_addr_str(mac_str, &entry->dst_mac);
#ifdef LPM_FRWD
	printf("%-32s%-32s%-16s\n",
	       ipv4_subnet_str(subnet_str, &entry->subnet),
	       entry->oif, mac_str);
#else
	if (entry->ip_protocol == IP_VERSION_4) {
		printf("%-32s%-32s%-16d%-16d%-8d%-32s%-16s\n",
		       ipv4_subnet_str(subnet_str, &entry->u.ipv4.src_subnet),
		       ipv4_subnet_str(dst_subnet_str, &entry->u.ipv4.dst_subnet),
		       entry->u.ipv4.src_port, entry->u.ipv4.dst_port, entry->u.ipv4.protocol,
		       entry->oif, mac_str);
	} else {
		printf("%-128s%-128s%-16d%-16d%-8d%-32s%-16s\n",
		       ipv6_subnet_str(subnet_str, &entry->u.ipv6.src_subnet),
		       ipv6_subnet_str(dst_subnet_str, &entry->u.ipv6.dst_subnet),
		       entry->u.ipv6.src_port, entry->u.ipv6.dst_port,
		       entry->u.ipv6.protocol, entry->oif, mac_str);
	}
#endif
}

void dump_fwd_db(void)
{
	fwd_db_entry_t *entry;

#ifdef LPM_FRWD
	printf("Routing table\n"
	       "-----------------\n"
	       "%-32s%-32s%-16s\n",
	       "subnet", "next_hop", "dest_mac");
#else
	printf("Routing table\n"
	       "-----------------\n"
	       "%-32s%-32s%-16s%-16s%-8s%-32s%-16s\n",
	       "src_subnet", "dst_subnet", "src_port", "dst_port", "proto", "next_hop", "dest_mac");
#endif
	for (entry = fwd_db->list; entry; entry = entry->next)
		dump_fwd_db_entry(entry);

	printf("\n");
}

fwd_db_entry_t *find_fwd_db_entry(tuple5_t *key)
{
#ifdef LPM_FRWD
	fwd_db_entry_t *entry;
#endif
	flow_entry_t *flow;
	flow_bucket_t *bucket;
	u64 hash;

	/* first find in cache */
	hash = l3fwd_calc_hash(key);
	hash &= fwd_lookup_cache.bkt_cnt - 1;

	bucket = &fwd_lookup_cache.bucket[hash];
	flow = lookup_fwd_cache(key, bucket);
	if (likely(flow))
		return flow->fwd_entry;

#ifdef LPM_FRWD
	for (entry = fwd_db->list; entry; entry = entry->next) {
		u32 mask;

		mask = ((1u << entry->subnet.depth) - 1) <<
			(32 - entry->subnet.depth);

		if (entry->subnet.addr == (key->u5t.ipv4_5t.dst_ip & mask))
			break;
	}

	insert_fwd_cache(key, bucket, entry);

	return entry;
#else
	return NULL;
#endif
}
