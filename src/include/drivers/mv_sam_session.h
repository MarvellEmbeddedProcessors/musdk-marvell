/*******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

	If you received this File from Marvell, you may opt to use, redistribute
	and/or modify this File under the following licensing terms.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

		* Redistributions of source code must retain the above copyright notice,
		  this list of conditions and the following disclaimer.

		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.

		* Neither the name of Marvell nor the names of its contributors may be
		  used to endorse or promote products derived from this software without
		  specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef __MV_SAM_SESSION_H__
#define __MV_SAM_SESSION_H__

#include "std.h"

struct sam_sa;

enum sam_opt {
	SAM_OPT_ENC = 1,
	SAM_OPT_DEC,
};

enum sam_alg {
	SAM_ALG_NULL = 0,
	SAM_ALG_DES,
	SAM_ALG_3DES,
	SAM_ALG_AES,
	SAM_ALG_MD5,
	SAM_ALG_SHA1
};

struct sam_session_params {
	enum sam_opt	 opt;
	enum sam_alg	 alg;
	int				 keylen;
	int				 mackeylen;
	char			 key[32];
	char			 mackey[64];
	int				 mlen;
	char			*iv;
	char			 mac_inner[64];
	char			 mac_outer[64];
	enum sam_alg	 auth_alg;
};

int sam_session_create_session(struct sam_session_params *params, struct sam_sa **sa);
int sam_session_destroy_session(struct sam_sa *sa);

#endif /* __MV_SAM_SESSION_H__ */
