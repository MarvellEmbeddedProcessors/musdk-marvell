/* firmware_eip207_api_dwld.h
 *
 * EIP-207 Firmware Download API:
 *
 * This API is defined by the EIP-207 Classification Firmware
 *
 */

/* -------------------------------------------------------------------------- */
/*                                                                            */
/*   Module        : firmware_eip197                                          */
/*   Version       : 2.4 / 2.7                                                */
/*   Configuration : FIRMWARE-GENERIC-NO-PP                                   */
/*                                                                            */
/*   Date          : 2017-Jun-19                                              */
/*                                                                            */
/* Copyright (c) 2012-2017 INSIDE Secure B.V. All Rights Reserved             */
/*                                                                            */
/* This confidential and proprietary software may be used only as authorized  */
/* by a licensing agreement from INSIDE Secure.                               */
/*                                                                            */
/* The entire notice above must be reproduced on all authorized copies that   */
/* may only be made to the extent permitted by a licensing agreement from     */
/* INSIDE Secure.                                                             */
/*                                                                            */
/* For more information or support, please go to our online support system at */
/* https://customersupport.insidesecure.com.                                  */
/* In case you do not have an account for this system, please send an e-mail  */
/* to ESSEmbeddedHW-Support@insidesecure.com.                                 */
/* -------------------------------------------------------------------------- */

#ifndef FIRMWARE_EIP207_API_DWLD_H_
#define FIRMWARE_EIP207_API_DWLD_H_

/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */

#include "basic_defs.h"         // uint32_t


/*----------------------------------------------------------------------------
 * Definitions and macros
 */

// IPUE firmware program counter value where the engine must be started from
// in order to perform the firmware version check in the debug mode
#define FIRMWARE_EIP207b_DWLD_IPUE_VERSION_CHECK_DBG_PROG_CNTR      0x03F0
#define FIRMWARE_EIP207d_DWLD_IPUE_VERSION_CHECK_DBG_PROG_CNTR      0x05F0
// IFPP firmware program counter value where the engine must be started from
// in order to perform the firmware version check in the debug mode
#define FIRMWARE_EIP207_DWLD_IFPP_VERSION_CHECK_DBG_PROG_CNTR      0x0BF0

// OPUE firmware program counter value where the engine must be started from
// in order to perform the firmware version check in the debug mode
#define FIRMWARE_EIP207_DWLD_OPUE_VERSION_CHECK_DBG_PROG_CNTR      0x03F0

// OFPP firmware program counter value where the engine must be started from
// in order to perform the firmware version check in the debug mode
#define FIRMWARE_EIP207_DWLD_OFPP_VERSION_CHECK_DBG_PROG_CNTR      0x05F0

// Administration RAM byte offsets as opposed to PE_n_ICE_SCRATCH_RAM
// 1 KB memory area base address in Classification Engine n
#define FIRMWARE_EIP207_DWLD_WORD_OFFS                            4
#define FIRMWARE_EIP207_DWLD_VERSION_BASE                         0

// Input Pull-Up micro-Engine version word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_IPUE_VERSION_BYTE_OFFSET    \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x00 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

// Input Flow Post-Processor micro-Engine version word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_IFPP_VERSION_BYTE_OFFSET    \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x02 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

// Output Pull-Up micro-Engine version word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_OPUE_VERSION_BYTE_OFFSET    \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x00 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

// Output Flow Post-Processor micro-Engine version word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_OFPP_VERSION_BYTE_OFFSET    \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x02 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

// Input Pull-Up micro-Engine control word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_IPUE_CTRL_BYTE_OFFSET       \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x05 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

// Input Flow Post-Processor micro-engine control word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_IFPP_CTRL_BYTE_OFFSET       \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x06 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

// Output Pull-Up micro-Engine control word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_OPUE_CTRL_BYTE_OFFSET       \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x05 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

// Output Flow Post-Processor micro-engine control word byte offset
#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_OFPP_CTRL_BYTE_OFFSET       \
                    ((FIRMWARE_EIP207_DWLD_VERSION_BASE) + (0x06 * FIRMWARE_EIP207_DWLD_WORD_OFFS))

#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_INPUT_LAST_BYTE_COUNT \
                    (FIRMWARE_EIP207_DWLD_ADMIN_RAM_IFPP_CTRL_BYTE_OFFSET + 4)

#define FIRMWARE_EIP207_DWLD_ADMIN_RAM_OUTPUT_LAST_BYTE_COUNT \
                    (FIRMWARE_EIP207_DWLD_ADMIN_RAM_OFPP_CTRL_BYTE_OFFSET + 4)

typedef struct
{
    unsigned int Version_MaMiPa;
    unsigned int Major;
    unsigned int Minor;
    unsigned int PatchLevel;
    const uint32_t * Image_p;
    unsigned int WordCount;

} FIRMWARE_EIP207_DWLD_t;


/*----------------------------------------------------------------------------
 * Firmware helper functions
 */

/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_IPUE_GetReferences
 *
 * This function returns references to the IPUE firmware image
 *
 * FW_p (output)
 *     Pointer to the memory location where the IPUE firmware parameters
 *     as defined by the FIRMWARE_EIP207_DWLD_t data structure will be stored
 *
 * Return value
 *     None
 */
void
FIRMWARE_EIP207_DWLD_IPUE_GetReferences(
        FIRMWARE_EIP207_DWLD_t * const FW_p);


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_IFPP_GetReferences
 *
 * This function returns references to the IFPP firmware image
 *
 * FW_p (output)
 *     Pointer to the memory location where the IFPP firmware parameters
 *     as defined by the FIRMWARE_EIP207_DWLD_t data structure will be stored
 *
 * Return value
 *     None
 */
void
FIRMWARE_EIP207_DWLD_IFPP_GetReferences(
        FIRMWARE_EIP207_DWLD_t * const FW_p);


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_OPUE_GetReferences
 *
 * This function returns references to the OPUE firmware image
 *
 * FW_p (output)
 *     Pointer to the memory location where the OPUE firmware parameters
 *     as defined by the FIRMWARE_EIP207_DWLD_t data structure will be stored
 *
 * Return value
 *     None
 */
void
FIRMWARE_EIP207_DWLD_OPUE_GetReferences(
        FIRMWARE_EIP207_DWLD_t * const FW_p);


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_OFPP_GetReferences
 *
 * This function returns references to the OFPP firmware image
 *
 * FW_p (output)
 *     Pointer to the memory location where the IFPP firmware parameters
 *     as defined by the FIRMWARE_EIP207_DWLD_t data structure will be stored
 *
 * Return value
 *     None
 */
void
FIRMWARE_EIP207_DWLD_OFPP_GetReferences(
        FIRMWARE_EIP207_DWLD_t * const FW_p);


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_Version_Read
 *
 * This function reads the EIP-207 firmware major, minor and patch level
 * numbers from the provided 32-bit value
 *
 * Value (input)
 *     32-bit value that can be read from the Administration RAM byte offset
 *     FIRMWARE_EIP207_DWLD_ADMIN_RAM_IPUE_VERSION_BYTE_OFFSET or
 *     FIRMWARE_EIP207_DWLD_ADMIN_RAM_IFPP_VERSION_BYTE_OFFSET
 *
 * Major_p (input)
 *     Pointer to the memory where the firmware major number will be stored
 *
 * Minor_p (input)
 *     Pointer to the memory where the firmware minor number will be stored
 *
 * PatchLevel_p (input)
 *     Pointer to the memory where the firmware patch level number will
 *     be stored
 *
 * Return value
 *     None
 */
static inline void
FIRMWARE_EIP207_DWLD_Version_Read(
        const uint32_t Value,
        unsigned int * const Major_p,
        unsigned int * const Minor_p,
        unsigned int * const PatchLevel_p)
{
    *Major_p       = (unsigned int)((Value >> 8) & MASK_4_BITS);
    *Minor_p       = (unsigned int)((Value >> 4) & MASK_4_BITS);
    *PatchLevel_p  = (unsigned int)((Value)      & MASK_4_BITS);
}


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_IPUE_VersionUpdated_Read
 *
 * This function reads the IPUE firmware version updated bit
 * from the provided 32-bit value
 *
 * Value (input)
 *     32-bit value that can be read from the Administration RAM byte offset
 *     FIRMWARE_EIP207_DWLD_ADMIN_RAM_IPUE_CTRL_BYTE_OFFSET
 *
 * fVersionUpdated_p (output)
 *     Pointer to the memory where the firmware version update flag
 *     will be stored. The firmware updates this flag when started in
 *     the debug mode at the Program Counter
 *     FIRMWARE_EIP207_DWLD_IPUE_VERSION_CHECK_DBG_PROG_CNTR
 *
 * Return value
 *     None
 */
static inline void
FIRMWARE_EIP207_DWLD_IPUE_VersionUpdated_Read(
        const uint32_t Value,
        bool * const fVersionUpdated_p)
{
    *fVersionUpdated_p  = ((Value & BIT_0) != 0);
}


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_IFPP_VersionUpdated_Read
 *
 * This function reads the IFPP firmware version updated bit
 * from the provided 32-bit value
 *
 * Value (input)
 *     32-bit value that can be read from the Administration RAM byte offset
 *     FIRMWARE_EIP207_DWLD_ADMIN_RAM_IFPP_CTRL_BYTE_OFFSET
 *
 * fVersionUpdated_p (output)
 *     Pointer to the memory where the firmware version update flag
 *     will be stored. The firmware updates this flag when started in
 *     the debug mode at the Program Counter
 *     FIRMWARE_EIP207_DWLD_IFPP_VERSION_CHECK_DBG_PROG_CNTR
 *
 * Return value
 *     None
 */
static inline void
FIRMWARE_EIP207_DWLD_IFPP_VersionUpdated_Read(
        const uint32_t Value,
        bool * const fVersionUpdated_p)
{
    *fVersionUpdated_p  = ((Value & BIT_0) != 0);
}


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_OPUE_VersionUpdated_Read
 *
 * This function reads the OPUE firmware version updated bit
 * from the provided 32-bit value
 *
 * Value (input)
 *     32-bit value that can be read from the Administration RAM byte offset
 *     FIRMWARE_EIP207_DWLD_ADMIN_RAM_OPUE_CTRL_BYTE_OFFSET
 *
 * fVersionUpdated_p (output)
 *     Pointer to the memory where the firmware version update flag
 *     will be stored. The firmware updates this flag when started in
 *     the debug mode at the Program Counter
 *     FIRMWARE_EIP207_DWLD_OPUE_VERSION_CHECK_DBG_PROG_CNTR
 *
 * Return value
 *     None
 */
static inline void
FIRMWARE_EIP207_DWLD_OPUE_VersionUpdated_Read(
        const uint32_t Value,
        bool * const fVersionUpdated_p)
{
    *fVersionUpdated_p  = ((Value & BIT_0) != 0);
}


/*----------------------------------------------------------------------------
 * FIRMWARE_EIP207_DWLD_OFPP_VersionUpdated_Read
 *
 * This function reads the OFPP firmware version updated bit
 * from the provided 32-bit value
 *
 * Value (input)
 *     32-bit value that can be read from the Administration RAM byte offset
 *     FIRMWARE_EIP207_DWLD_ADMIN_RAM_OFPP_CTRL_BYTE_OFFSET
 *
 * fVersionUpdated_p (output)
 *     Pointer to the memory where the firmware version update flag
 *     will be stored. The firmware updates this flag when started in
 *     the debug mode at the Program Counter
 *     FIRMWARE_EIP207_DWLD_OFPP_VERSION_CHECK_DBG_PROG_CNTR
 *
 * Return value
 *     None
 */
static inline void
FIRMWARE_EIP207_DWLD_OFPP_VersionUpdated_Read(
        const uint32_t Value,
        bool * const fVersionUpdated_p)
{
    *fVersionUpdated_p  = ((Value & BIT_0) != 0);
}

#endif /* FIRMWARE_EIP207_API_DWLD_H_ */
/* end of file firmware_eip207_api_dwld.h */
