/* sa_builder_params_srtp.h
 *
 * SRTP specific extension to the SABuilder_Params_t type.
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


#ifndef SA_BUILDER_PARAMS_SRTP_H_
#define SA_BUILDER_PARAMS_SRTP_H_


/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */

#include "sa_builder_params.h"

// Driver Framework Basic Definitions API
#include "basic_defs.h"



/*----------------------------------------------------------------------------
 * Definitions and macros
 */

/* Flag values for te SRTPFlags field */
#define SAB_SRTP_FLAG_SRTCP          BIT_0
#define SAB_SRTP_FLAG_INCLUDE_MKI    BIT_1


/* Extension record for SAParams_t. Protocol_Extension_p must point
   to this structure when the SRTP protocol is used.

   SABuilder_Iinit_SRTP() will fill all fields in this structure  with
   sensible defaults.
 */
typedef struct
{
    uint32_t SRTPFlags;
    uint32_t MKI;
    uint32_t ICVByteCount; /* Length of ICV in bytes. */
} SABuilder_Params_SRTP_t;


#endif /* SA_BUILDER_PARAMS_SRTP_H_ */


/* end of file sa_builder_params_srtp.h */
