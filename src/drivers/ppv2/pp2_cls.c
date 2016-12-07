/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "std_internal.h"

#include "mv_pp2_cls.h"


struct pp2_cls_tbl {
	int dummy;
};

int pp2_cls_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}

void pp2_cls_tbl_deinit(struct pp2_cls_tbl *tbl)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
}

int pp2_cls_tbl_add_rule(struct pp2_cls_tbl			*tbl,
			 struct pp2_cls_tbl_rule		*rule,
			 struct pp2_cls_tbl_entry_action	*action)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}

int pp2_cls_tbl_modify_rule(struct pp2_cls_tbl			*tbl,
			    struct pp2_cls_tbl_rule		*rule,
			    struct pp2_cls_tbl_entry_action	*action)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}

int pp2_cls_tbl_remove_rule(struct pp2_cls_tbl			*tbl,
			    struct pp2_cls_tbl_rule		*rule)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}
