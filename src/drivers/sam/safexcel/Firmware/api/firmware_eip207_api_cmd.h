/* firmware_eip207_api_cmd.h
 *
 * EIP-207 Firmware Packet Flow ID API:
 *
 * Defines codes for packet flows and special-case commands.
 * These codes are included in command descriptors (input tokens) supplied
 * to the Classification Engine.
 *
 * This API is defined by the EIP-207 Classification Firmware.
 *
 */

/* -------------------------------------------------------------------------- */
/*                                                                            */
/*   Module        : firmware_eip197                                          */
/*   Version       : 2.4                                                      */
/*   Configuration : FIRMWARE-GENERIC-NO-PP                                   */
/*                                                                            */
/*   Date          : 2015-Feb-25                                              */
/*                                                                            */
/* Copyright (c) 2011-2015 INSIDE Secure B.V. All Rights Reserved             */
/*                                                                            */
/* This confidential and proprietary software may be used only as authorized  */
/* by a licensing agreement from INSIDE Secure.                               */
/*                                                                            */
/* The entire notice above must be reproduced on all authorized copies that   */
/* may only be made to the extent permitted by a licensing agreement from     */
/* INSIDE Secure.                                                             */
/*                                                                            */
/* For more information or support, please go to our online support system at */
/* https://essoemsupport.insidesecure.com.                                    */
/* In case you do not have an account for this system, please send an e-mail  */
/* to ESSEmbeddedHW-Support@insidesecure.com.                                 */
/* -------------------------------------------------------------------------- */

#ifndef FIRMWARE_EIP207_API_CMD_H_
#define FIRMWARE_EIP207_API_CMD_H_


/*----------------------------------------------------------------------------
 * This module implements (provides) the following interface(s):
 */


/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */


/*----------------------------------------------------------------------------
 * Definitions and macros
 */

// Lookaside Crypto packet Flow
#define FIRMWARE_EIP207_CMD_PKT_LAC       0x04

// Lookaside IPsec packet flow
#define FIRMWARE_EIP207_CMD_PKT_LIP       0x02

// Inline IPsec packet flow
#define FIRMWARE_EIP207_CMD_PKT_IIP       0x02

// Lookaside IPsec with Token Builder offload packet flow
#define FIRMWARE_EIP207_CMD_PKT_LAIP_TB   0x18

// Lookaside DTLS (including CAPWAP)
#define FIRMWARE_EIP207_CMD_PKT_LDT       0x28

// Inline DTLS (including CAPWAP)
#define FIRMWARE_EIP207_CMD_PKT_IDT       0x28

// Inline MACsec
#define FIRMWARE_EIP207_CMD_PKT_IMA       0x38

// Invalidate Transform Record command
#define FIRMWARE_EIP207_CMD_INV_TR        0x06

// Invalidate Flow Record command
#define FIRMWARE_EIP207_CMD_INV_FR        0x05

// Bit mask that enables the flow lookup. Otherwise the transform lookup is performed.
#define FIRMWARE_EIP207_CMD_FLOW_LOOKUP   0x80

#endif /* FIRMWARE_EIP207_API_CMD_H_ */


/* end of file firmware_eip207_api_cmd.h */
