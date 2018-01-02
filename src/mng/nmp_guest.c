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

#include "std_internal.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest.h"
#include "db.h"
#include "dev_mng.h"
#include "hw_emul/gie.h"
#include "mng/dispatch.h"
#include "config.h"
#include "lib/lib_misc.h"

struct nmp_guest {
	u8	 id;
	u32	 timeout;
	char	*prb_str;
};

static int nmp_guest_wait_for_guest_file(struct nmp_guest *guest)
{
	char	file_name[SER_MAX_FILE_SIZE];
	int	timeout = guest->timeout;
	int	fd;

	/* Map serial file*/
	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, guest->id);

	/* wait for guest file to be opened by NMP */
	do {
		fd = open(file_name, O_RDWR);
		if (fd > 0) {
			close(fd);
			break;
		}

		udelay(100);
	} while (fd < 0 && --timeout);

	if (!timeout) {
		pr_err("failed to find regfile %s. timeout exceeded.\n", file_name);
		return -EFAULT;
	}

	return 0;
}

int nmp_guest_init(struct nmp_guest_params *params, struct nmp_guest **g)
{
	int	err;
	char	file_name[SER_MAX_FILE_NAME];

	*g = kcalloc(1, sizeof(struct nmp_guest), GFP_KERNEL);
	if (*g == NULL) {
		pr_err("Failed to allocate NMP handler\n");
		return -ENOMEM;
	}

	(*g)->id = params->id;
	(*g)->timeout = params->timeout;
	(*g)->prb_str = kcalloc(1, SER_MAX_FILE_SIZE, GFP_KERNEL);
	if ((*g)->prb_str == NULL) {
		err = -ENOMEM;
		goto guest_init_err1;
	}

	err = nmp_guest_wait_for_guest_file(*g);
	if (err) {
		err = -EINVAL;
		pr_err("Guest file not available\n");
		goto guest_init_err2;
	}

	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, (*g)->id);
	err = read_file_to_buf(file_name, (*g)->prb_str, SER_MAX_FILE_SIZE);
	if (err) {
		pr_err("app_read_config_file failed for %s\n", file_name);
		goto guest_init_err2;
	}

	pr_info("%s...done\n", __func__);

	return 0;

guest_init_err2:
	kfree((*g)->prb_str);
guest_init_err1:
	kfree((*g));
	return err;

}

void nmp_guest_deinit(struct nmp_guest *g)
{
	kfree(g->prb_str);
	kfree(g);
	pr_info("%s...done\n", __func__);
}

int nmp_guest_get_probe_str(struct nmp_guest *g, char **prb_str)
{
	*prb_str = g->prb_str;
	pr_debug("prb_str: %s\n", *prb_str);
	return 0;
}

