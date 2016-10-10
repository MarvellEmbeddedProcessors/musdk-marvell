/**
 * @file PP2_plat.h
 *
 * Packet Processor SDK platform hardware information
 *
 * Used internally and/or as an exportable header to
 * an application or to an external implementation
 *
 */
#ifndef _PP2_PLAT_H_
#define _PP2_PLAT_H_

/******************************************************************************/
/*                 Hardware Resources per Platform                            */


/* Platform specific absolute hardware values.
 *
 * NOTE: These should be treated as read-only primitives
 * and are exported here for informational reasons
 *
 * The real hardware configuration values on which PPDK and
 * run-time clients shall work will be derived from these
 * after platform and resource offsets have been established
 */

/* Number of physical CPUs */
#define PP2_NUM_CPUS                 (4)


/* Absolute number of hardware ports */
#define PP2_NUM_PORTS                (3)
/* Absolute number of hardware register spaces */
#define PP2_NUM_REGSPACES            (9)
/* Absolute number of hardware BM Pools */
#define PP2_NUM_BMPOOLS              (16)
#define PP2_MAX_NUM_PACKPROCS        (2)

/* Number of RXQs per port */
#define PP2_HW_PORT_NUM_RXQS         (32)
/* Number of TXQs per port */
#define PP2_HW_PORT_NUM_TXQS         (8)



/******************************************************************************/
/*    Hardware Resources Offsets(reserved to OS/slow path) per Platform       */

/******************************************************************************/
/*                  Resources configured and used by PPDK                     */

/* Size of a CPU space (slot) */
#define PP2_REGSPACE_SIZE     (0x10000)
/* Address size to skip when mapping packet processor register space */
//#define PP2_SLOT_OFS_ADDR     (PP2_OFS_REGSPACES * PP2_REGSPACE_SIZE)
/* Default register space index used for initializing
 * common hardware parts
 */
#define PP2_DEFAULT_REGSPACE  (0)
/* Packet processor IDs */
#define PP2_ID0               (0)
#define PP2_ID1               (1)


/* Number of RXQs per port - configured */
#define PP2_PORT_NUM_RXQS         (1)
#if (PP2_PORT_NUM_RXQS > PP2_HW_PORT_NUM_RXQS)
#error "PPDK: number of configured RxQs cannot exceed available RxQs";
#endif
/* Number of TXQs per port - configured */
#define PP2_PORT_NUM_TXQS         (1)
#if (PP2_PORT_NUM_TXQS > PP2_HW_PORT_NUM_TXQS)
#error "PPDK: number of configured TxQs cannot exceed available TxQs";
#endif

/* 64bytes cache line - arm64 spec */
#define L1_CACHE_LINE_BYTES      (1 << 6)

/* Packet offset always used by HW at the beginng of packets.
 * This is valid for the first buffer of the packet,
 * all other buffers have a fixed offset = 32bytes
 */
#define PP2_PACKET_OFFSET         (L1_CACHE_LINE_BYTES)

/* 32bytes always used by HW, found  at the begining of buffers
 * It is also part of the packet offset(PP2_PACKET_OFFSET), first buffer
 */
#define PP2_BUFFER_OFFSET         (32)

/* The two bytes Marvell header. Either contains a special value used
 * by Marvell switches when a specific hardware mode is enabled (not
 * supported by this driver) or is filled automatically by zeroes on
 * the RX side. Those two bytes being at the front of the Ethernet
 * header, they allow to have the IP header aligned on a 4 bytes
 * boundary automatically: the hardware skips those two bytes on its
 * own.
 */
#define PP2_MH_SIZE              (2)

/* packet data is stored relative to an offset */
#define PP2_BUFF_START_ADDR(addr)   ((addr) + PP2_PACKET_OFFSET + PP2_MH_SIZE)
#define PP2_BUFF_SIZE(buff_size)    ((buff_size) + PP2_PACKET_OFFSET + PP2_MH_SIZE)

#endif /* _PP2_PLAT_H_ */
