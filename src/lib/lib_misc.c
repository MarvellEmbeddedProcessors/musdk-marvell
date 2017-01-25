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
#include "lib/lib_misc.h"

int mv_sys_match(const char *match, const char* obj_type, u8 hierarchy_level, u8 id[])
{
	char tmp_str[MAX_OBJ_STRING];
	char *tok;

	if (hierarchy_level > 2) {
		pr_err("Maximum 3 levels of hierarchy supported (given %d)!\n", hierarchy_level);
		return(-1);
	}

	memcpy(tmp_str, match, strlen(match));
	tmp_str[strlen(match)] = '\0';
	tok = strtok(tmp_str, "-");
	if (!tok) {
		pr_err("Illegal match str (%s)!\n",tok);
		return -1;
	}

	if (strcmp(obj_type, tok)) {
		pr_err("String {%s} does not match obj_type {%s}\n", tok, obj_type);
		return(-1);
	}

	if (hierarchy_level == 1) {
		tok = strtok(NULL, "");
		if (!tok) {
			pr_err("Illegal match str (%s)!\n", tok);
			return -1;
		}
		id[0] = atoi(tok);
	} else if (hierarchy_level == 2) {
		tok = strtok(NULL, ":");
		if (!tok) {
			pr_err("Illegal match str (%s)!\n",tok);
			return -1;
		}
		id[0] = atoi(tok);
		tok = strtok(NULL, "");
		if (!tok) {
			pr_err("Illegal match str (%s)!\n",tok);
			return -1;
		}
		id[1] = atoi(tok);
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
		printf("%s\n", buf);
	}
	printf("\n");
}

void mv_mem_dump(const unsigned char *p, unsigned int len)
{
	unsigned int i = 0, j;

	while (i < len) {
		j = 0;
		printf("%10p: ", (p + i));
		for (j = 0 ; j < 32 && i < len ; j++) {
			printf("%02x ", p[i]);
			i++;
		}
		printf("\n");
	}
}

