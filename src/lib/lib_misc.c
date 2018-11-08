/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include "std_internal.h"
#include "lib/lib_misc.h"

static char *mv_strtok(char *src, const char *pattern)
{
	static char *nxt_tok;
	char *ret_val = NULL;

	if (!src)
		src = nxt_tok;

	while (*src) {
		const char *pp = pattern;

		while (*pp) {
			if (*pp == *src)
				break;

			pp++;
		}
		if (!*pp) {
			if (!ret_val)
				ret_val = src;
			else if (!src[-1])
				break;
		} else
			*src = '\0';
		src++;
	}

	nxt_tok = src;

	return ret_val;
}

int mv_sys_match(const char *match, const char* obj_type, u8 hierarchy_level, u8 id[])
{
	char tmp_str[MAX_OBJ_STRING];
	char *tok;
	int rc;

	if (hierarchy_level > 2) {
		pr_err("Maximum 3 levels of hierarchy supported (given %d)!\n", hierarchy_level);
		return(-1);
	}

	memcpy(tmp_str, match, strlen(match));
	tmp_str[strlen(match)] = '\0';
	tok = mv_strtok(tmp_str, "-");
	if (!tok) {
		pr_err("Illegal match str (%s)!\n",tok);
		return -1;
	}

	if (strcmp(obj_type, tok)) {
		pr_err("String {%s} does not match obj_type {%s}\n", tok, obj_type);
		return(-1);
	}

	if (hierarchy_level == 1) {
		tok = mv_strtok(NULL, "");
		if (!tok) {
			pr_err("Illegal match str (%s)!\n", tok);
			return -1;
		}
		rc = kstrtou8(tok, 10, &id[0]);
		if (rc) {
			pr_err("String \"%s\" is not a number.\n", tok);
			return rc;
		}
	} else if (hierarchy_level == 2) {
		tok = mv_strtok(NULL, ":");
		if (!tok) {
			pr_err("Illegal match str (%s)!\n",tok);
			return -1;
		}
		rc = kstrtou8(tok, 10, &id[0]);
		if (rc) {
			pr_err("String \"%s\" is not a number.\n", tok);
			return rc;
		}
		tok = mv_strtok(NULL, "");
		if (!tok) {
			pr_err("Illegal match str (%s)!\n",tok);
			return -1;
		}
		rc = kstrtou8(tok, 10, &id[1]);
		if (rc) {
			pr_err("String \"%s\" is not a number.\n", tok);
			return rc;
		}
	}

	return 0;
}

void mem_disp(const char *_p, int len)
{
	char buf[128];
	int i, j, i0;
	const unsigned char *p = (const unsigned char *)_p;

	/* hexdump routine */
	for (i = 0; i < len; ) {
		memset(buf, sizeof(buf), ' ');
		sprintf(buf, "%5d: ", i);
		i0 = i;
		for (j=0; j < 16 && i < len; i++, j++)
			sprintf(buf+7+j*3, "%02x ", (uint8_t)(p[i]));
		i = i0;
		for (j=0; j < 16 && i < len; i++, j++)
			sprintf(buf+7+j + 48, "%c",
				isprint(p[i]) ? p[i] : '.');
		printk("%s\n", buf);
	}
	printk("\n");
}

void mv_mem_dump(const unsigned char *p, unsigned int len)
{
	unsigned int i = 0, j;

	while (i < len) {
		j = 0;
		printk("%10p: ", (p + i));
		for (j = 0; (j < 32) && (i < len); j++) {
			printk("%02x ", p[i]);
			i++;
		}
		printk("\n");
	}
}

void mv_mem_dump_words(const u32 *p, u32 words, int be)
{
	u32 i = 0, j;

	while (i < words) {
		j = 0;
		printk("%10p: ", (p + i));
		for (j = 0; (j < 8) && (i < words); j++) {
			if (be)
				printk("%08x ", be32_to_cpu(p[i]));
			else
				printk("%08x ", le32_to_cpu(p[i]));

			i++;
		}
		printk("\n");
	}
}

#ifdef __KERNEL__
static int mv_kernel_ver_get(void)
{
	int major, minor, err;
	char *kernel_version;

	kernel_version = utsname()->release;

	err = sscanf(kernel_version, "%d.%d", &major, &minor);
	if (err <= 0)
		return err;

	pr_debug("%s: ver:%s, major:%d, minor:%d\n", __func__, kernel_version, major, minor);

	return MKDEV(major, minor);
}
#else
static int mv_kernel_ver_get(void)
{
	int major, minor, parsed;
	char *kernel_version;
	struct utsname buf;
	int ret;

	ret = uname(&buf);
	if (ret < 0)
		return ret;
	kernel_version = buf.release;
	parsed = sscanf(kernel_version, "%d.%d", &major, &minor);
	if (parsed < 2) {
		pr_err("%s: Failed to Parse linux_version\n", __func__);
		return 0;
	}

	pr_debug("%s: ver:%s, major:%d, minor:%d\n", __func__, kernel_version, major, minor);

	return MKDEV(major, minor);
}
#endif

enum musdk_lnx_id lnx_id_get(void)
{
	int lk_ver;
	static enum musdk_lnx_id lnx_id = LNX_VER_INVALID;

	if (lnx_id != LNX_VER_INVALID)
		return lnx_id;

	lk_ver = mv_kernel_ver_get();

	if ((MAJOR(lk_ver) == 4) && (MINOR(lk_ver) == 4))
		lnx_id = LNX_4_4_x;
	else
		lnx_id = LNX_OTHER;

	return lnx_id;
}

int lnx_is_mainline(enum musdk_lnx_id lnx_id)
{
	return (lnx_id > LNX_4_4_x);
}
