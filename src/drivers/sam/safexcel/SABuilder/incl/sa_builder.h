/* sa_builder.h
 *
 * Main API for the SA Builder.
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


#ifndef SA_BUILDER_H_
#define SA_BUILDER_H_


/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */

#include "sa_builder_params.h"

// Driver Framework Basic Definitions API
#include "basic_defs.h"


/*----------------------------------------------------------------------------
 * Definitions and macros
 */
typedef enum
{
    SAB_STATUS_OK,
    SAB_INVALID_PARAMETER,
    SAB_UNSUPPORTED_FEATURE,
    SAB_ERROR
} SABuilder_Status_t;


/*----------------------------------------------------------------------------
 * SABuilder_GetSizes();
 *
 * Compute the required sizes in 32-bit words of any of up to three memory
 * areas used by the SA.
 *
 * SAParams_p (input)
 *   Pointer to the SA parameters structure.
 * SAWord32Count_p (output)
 *   The size of the normal SA buffer,
 * SAStateWord32Count_p (output)
 *   The size of any SA state record.
 * ARC4StateWord32Count_p (output) T
 *   The size of any ARCFOUR state buffer (output).
 *
 * When the SA state record or ARCFOUR state buffer are not required by
 * the packet engine for this transform, the corresponding size outputs
 * are returned as zero.
 *
 * If any of the output parameters is a null pointer,
 * the corresponding size will not be returned.
 *
 * The SAParams_p structure must be fully filled in: it must have the
 * same contents as it would have when SABuilder_BuildSA is called.
 *
 * Return:
 * SAB_STATUS_OK on success
 * SAB_INVALID_PARAMETER when the record referenced by SAParams_p is invalid,
 * SAB_UNSUPPORTED_FEATURE when SAParams_p describes an operations that
 *    is not supported on the hardware for which this SA builder
 *    is configured.
 */
SABuilder_Status_t
SABuilder_GetSizes(
    SABuilder_Params_t *const SAParams_p,
    unsigned int *const SAWord32Count_p,
    unsigned int *const SAStateWord32Count_p,
    unsigned int *const ARC4StateWord32Count_p);


/*----------------------------------------------------------------------------
 * SABuilder_BuildSA();
 *
 * Construct the SA record for the operation described in SAParams_p in
 * up to three memory buffers. Update the field offset values in the
 * SA parameter structure.
 *
 * SAParams_p (input, output)
 *    Pointer to the SA parameters structure. Field offset values will
 *    be updated.
 * SABuffer_p (output)
 *    Pointer to the the normal SA buffer,
 * SAStateBuffer_p (output)
 *    Pointer to the SA state record buffer,
 * ARC4StateBuffer_p (output)
 *    Pointer to the ARCFOUR state buffer.
 *
 * Each of the Buffer arguments must point to a word-aligned
 * memory buffer whose size in words is at least equal to the
 * corresponding size parameter returned by SABuilder_GetSizes().
 *
 * If any of the three buffers is not required for the SA (the
 * corresponding size in SABuilder_GetSizes() is 0), the corresponding
 * Buffer arguments to this function may be a null pointer.
 *
 * Return:
 * SAB_STATUS_OK on success
 * SAB_INVALID_PARAMETER when SAParams_p is invalid, or if any of
 *    the buffer arguments  is a null pointer while the corresponding buffer
 *    would be required for the operation.
 * SAB_UNSUPPORTED_FEATURE when SAParams_p describes an operations that
 *    is not supported on the hardware for which this SA builder
 *    is configured.
 */
SABuilder_Status_t
SABuilder_BuildSA(
    SABuilder_Params_t * const SAParams_p,
    uint32_t *const SABuffer_p,
    uint32_t *const SAStateBuffer_p,
    uint32_t *const ARC4StateBuffer_p);


#endif /* SA_BUILDER_H_ */


/* end of file sa_builder.h */
