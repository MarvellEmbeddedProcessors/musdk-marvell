/* sa_builder_ipsec.h
 *
 * IPsec specific functions of the SA Builder.
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


#ifndef  SA_BUILDER_IPSEC_H_
#define SA_BUILDER_IPSEC_H_


/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */

#include "sa_builder_params.h"
#include "sa_builder.h"
#include "sa_builder_params_ipsec.h"

// Driver Framework Basic Definitions API
#include "basic_defs.h"


/*----------------------------------------------------------------------------
 * SABuilder_Init_ESP
 *
 * This function initializes the SABuilder_Params_t data structure and its
 * SABuilder_Params_IPsec_t extension with sensible defaults for ESP
 * processing.
 *
 * SAParams_p (output)
 *   Pointer to SA parameter structure to be filled in.
 * SAParamsIPsec_p (output)
 *   Pointer to IPsec parameter extension to be filled in
 * spi (input)
 *   SPI of the newly created parameter structure (must not be zero).
 * TunnelTransport (input)
 *   Must be one of SAB_IPSEC_TUNNEL or SAB_IPSEC_TRANSPORT.
 * IPMode (input)
 *   Must be one of SAB_IPSEC_IPV4 or SAB_IPSEC_IPV6.
 * direction (input)
 *   Must be one of SAB_DIRECTION_INBOUND or SAB_DIRECTION_OUTBOUND.
 *
 * Both the crypto and the authentication algorithm are initialized to
 * NULL, which is illegal. At least one of the crypto algorithm or the
 * authentication algorithm and associated keys must be modified after
 * calling this function to obtain a valid parameter structure.
 *
 * Both the SAParams_p and SAParamsIPsec_p input parameters must point
 * to valid storage where variables of the appropriate type can be
 * stored. This function initializes the link from SAParams_p to
 * SAParamsIPsec_p.
 *
 * Return:
 * SAB_STATUS_OK on success
 * SAB_INVALID_PARAMETER when one of the pointer parameters is NULL
 *   or the remaining parameters have illegal values.
 */
SABuilder_Status_t
SABuilder_Init_ESP(
    SABuilder_Params_t * const SAParams_p,
    SABuilder_Params_IPsec_t * const SAParamsIPsec_p,
    const uint32_t spi,
    const uint32_t TunnelTransport,
    const uint32_t IPMode,
    const SABuilder_Direction_t direction);


#endif /* SA_BUILDER_IPSEC_H_ */


/* end of file sa_builder_ipsec.h */
