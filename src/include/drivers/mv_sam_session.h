/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

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
