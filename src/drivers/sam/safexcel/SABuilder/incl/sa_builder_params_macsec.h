/* sa_builder_params_macsec.h
 *
 * MACsec specific extension to the SABuilder_Params_t type.
 */

/*****************************************************************************
Copyright (c) 2008-2016 INSIDE Secure B.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of INSIDE Secure B.V. nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/


#ifndef SA_BUILDER_PARAMS_MACSEC_H_
#define SA_BUILDER_PARAMS_MACSEC_H_


/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */

#include "sa_builder_params.h"

// Driver Framework Basic Definitions API
#include "basic_defs.h"


/*----------------------------------------------------------------------------
 * Definitions and macros
 */

/* Flag bits for the MACsecFlags field. Combine any values using a
   bitwise or.
 */
#define SAB_MACSEC_ES              BIT_0 /* End Station */
#define SAB_MACSEC_SC              BIT_1 /* Include SCI in frames */
#define SAB_MACSEC_SCB             BIT_2 /* Enable EPON Single Channel Broadcase */


/* Extension record for SAParams_t. Protocol_Extension_p must point
   to this structure when the MACsec  protocol is used.

   SABuilder_Iinit_MACsec() will fill all fields in this structure  with
   sensible defaults.
 */
typedef struct
{
    uint32_t MACsecFlags;  /* See SAB_MACSEC_* flag bits above*/
    const uint8_t *SCI_p;        /* Pointer to 8-byte SCI */
    uint8_t AN;            /* Association Number */

    uint32_t SeqNum;       /* Sequence number.*/

    uint32_t ReplayWindow; /* Size of the anti-replay window */

    uint32_t ConfOffset; /* Confidentiality Offset. Specify a number of
                                unencrypted bytes at the start of each packet.*/
    uint32_t ContextRef; /* Reference to application context */
} SABuilder_Params_MACsec_t;


#endif /* SA_BUILDER_PARAMS_MACSEC_H_ */


/* end of file sa_builder_params_macsec.h */
