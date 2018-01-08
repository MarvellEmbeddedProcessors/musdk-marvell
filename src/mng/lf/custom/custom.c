/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#define log_fmt(fmt) "pf: " fmt

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "mng/db.h"
#include "mng/mv_nmp.h"
#include "mng/dispatch.h"
#include "env/trace/trc_pf.h"
#include "custom.h"


/*
 *	nmcstm_init
 *
 *	@param[in]	params - custom module parameters
 *	@param[in]	nmcstm - pointer to custom object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmcstm_init(struct nmcstm_params *params, struct nmcstm **nmcstm)
{
	params = params;
	nmcstm = nmcstm;

	return 0;
}


/*
 *	nmcstm_deinit
 *
 *	@param[in]	nmcstm - pointer to custom object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmcstm_deinit(struct nmcstm *nmcstm)
{
	nmcstm = nmcstm;

	return 0;
}


/*
 *	nmcstm_process_command
 *
 *	This function process Custom commands
 *
 *	@param[in]	nmcstm - pointer to Custom object
 *	@param[in]	cmd_code
 *	@param[in]	cmd - pointer to cmd_desc object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmcstm_process_command(void *nmcstm, u8 cmd_code, void *cmd)
{
	nmcstm = nmcstm;
	cmd_code = cmd_code;
	cmd = cmd;

	return 0;
}

