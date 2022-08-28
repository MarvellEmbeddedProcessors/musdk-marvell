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

#ifndef __LIB_MISC_H__
#define __LIB_MISC_H__

/* Serialization helpers */

#define FILE_MAX_LINE_CHARS	80
#define JSON_LEVEL_2_TAB_COMMA_ADDITION_LEN	5

/**
 * json_print_to_buffer
 *
 * Description: add a string to a buffer. This macro is used for writing a string in
 * JSON format.
 *
 * NOTE: The JSON formating is done at the calling function. This function only checks
 * there is enough space in the buffer for writing the string
 *
 * Some examples for writing to JSON format file:
 * 1) Write a new section for bpool-x:x:
 *	json_print_to_buffer(buff, size, 2, "\"pool-%d:%d\": {\n", pool->pp2_id, pool->id);
 *
 * 2) Write pool_id and pp2_id parameters (inside the above section):
 *	json_print_to_buffer(buff, size, 3, "\"pp2_id\": %u,\n", pool->pp2_id);
 *	json_print_to_buffer(buff, size, 3, "\"id\": %u,\n", pool->id);
 *
 * @param[in]	buf	A pointer to the writing buffer.
 * @param[in]	size	Total size of the buffer
 * @param[in]	tabs	Number of tabs to add to the string before
 * @param[in]		__VA_ARGS__ containing the string to write and the formating options
 */
#define json_print_to_buffer(buff, size, num_tabs, ...)				\
do {										\
	/* Check the input string can be written in the input buffer */		\
	char tmp_buf[FILE_MAX_LINE_CHARS];					\
	int tab;								\
	u32 str_len, tab_len = 0;						\
	for (tab = 0; tab < num_tabs; tab++) {					\
		str_len = snprintf(&tmp_buf[tab], FILE_MAX_LINE_CHARS, "\t");	\
		tab_len += str_len;						\
	}									\
	str_len = snprintf(&tmp_buf[tab_len], size - pos, __VA_ARGS__);		\
	if ((str_len + tab_len) <= (size - pos))				\
		strcpy(&buff[pos], &tmp_buf[0]);				\
	else									\
		pr_warn("skip write (%s): buf too short (%d)!\n",		\
			tmp_buf, size);						\
	pos += str_len + tab_len;						\
} while (0)

/**
 * json_buffer_to_input
 *
 * Description: search for a specified string and retrieve its integer value from a
 * JSON format buffer.
 *
 * NOTE: This function does not extract the values from regular JSON file. The file should
 * be according to the json_json_print_to_buffer macro above
 *
 * NOTE: Currently the macros parse only the matching string, and do not search in the input
 * buffer (meaning the calling order should be the same as the written order)
 *
 * An example for reading from JSON format file:
 *	Retireve pp2_id and pool_id:
 *		 json_buffer_to_input(sec, "pp2_id", pp2_id);
 *		 json_buffer_to_input(sec, "pool_id", pool_id);
 *
 * @param[in]	buf	A pointer to the reading buffer.
 * @param[in]	_str	A pointer to the matching string
 *
 * @param[out]	_p	The parameter to update with the value found
 */
#define json_buffer_to_input(buff, _str, _p)					\
do {										\
	int	rc, res = 0;							\
	u64	res1 = 0;							\
	int	found = 0;							\
	char	*tok, *tmp_tok;							\
										\
	/* First, check if the string exists in the buffer */			\
	if (!strstr(buff, _str)) {						\
		pr_warn("string %s not found in input buff\n", _str);		\
		break;								\
	}									\
	/*Go over the input buffer untile the match str is found. */		\
	do {									\
		tok = strsep(&buff, "\n");					\
		/*if a "{" is found, go to the next line in buffer. */		\
		tmp_tok = strstr(tok, "{");					\
		if (tmp_tok != NULL) {						\
			continue;						\
		}								\
		/*if a "}" is found, go to the next line in buffer. */		\
		tmp_tok = strstr(tok, "}");					\
		if (tmp_tok != NULL) {						\
			continue;						\
		}								\
		/*if str is not found, exit with error */			\
		if (strstr(tok, _str) == NULL) {				\
			pr_err("%s not found\n", _str);				\
			break;							\
		}								\
		/*search for the ": " string, if not found, exit with error */	\
		tmp_tok = strstr(tok, ": ");					\
		if (tmp_tok == NULL) {						\
			pr_err("Invalid string(%s)!\n", tok);			\
			break;							\
		}								\
		/* check if string contains a "," at the end and remove */	\
		if (strstr(tmp_tok, "0x") != NULL) {				\
			rc = kstrtou64(&tmp_tok[strlen(": 0x")], 16, &res1);	\
			if (rc != 0) {						\
				pr_err("Invalid string(%s)!\n", tok);		\
				break;						\
			}							\
			_p = res1;						\
			found = 1;						\
		} else {							\
			/* treat the val as integer */				\
			rc = kstrtoint(&tmp_tok[strlen(": ")], 10, &res);	\
			if (rc != 0) {						\
				pr_err("4 Invalid string(%s)!\n", tok);		\
				break;						\
			}							\
			_p = res;						\
			found = 1;						\
		}								\
	} while (!found);							\
} while (0)

/**
 * json_buffer_to_input_mac
 *
 * Description: search for a specified string and retrieve its mac address value from a
 * JSON format buffer.
 *
 * NOTE: This function does not extract the values from regular JSON file. The file should
 * be according to the json_json_print_to_buffer macro above
 *
 * NOTE: Currently the macros parse only the matching string, and do not search in the input
 * buffer (meaning the calling order should be the same as the written order)
 *
 * An example for reading from JSON format file:
 *	Retireve pp2_id and pool_id:
 *		json_buffer_to_input_mac(sec, "mac_address", port->mac_data.mac);
 *
 * @param[in]	buf	A pointer to the reading buffer.
 * @param[in]	_str	A pointer to the matching string
 *
 * @param[out]	_p	The parameter to update with the value found
 */
#define json_buffer_to_input_mac(buff, _str, _p)				\
do {										\
	int	found = 0;							\
	u8	mac[ETH_ALEN];							\
	char	*tok, *tmp_tok;							\
	/*ascii_code for quotes. This is for checkpatch compatibility. */	\
	char	ascii_code = 34;						\
										\
	/* First, check if the string exists in the buffer */			\
	if (!strstr(buff, _str)) {						\
		pr_warn("string %s not found in input buff\n", _str);		\
		break;								\
	}									\
	/*Go over the input buffer untile the match str is found. */		\
	do {									\
		tok = strsep(&buff, "\n");					\
		/*if a "{" is found, go to the next line in buffer. */		\
		tmp_tok = strstr(tok, "{");					\
		if (tmp_tok != NULL) {						\
			pr_debug("1 found %s\n", tmp_tok);			\
			continue;						\
		}								\
		/*if a "}" is found, go to the next line in buffer. */		\
		tmp_tok = strstr(tok, "}");					\
		if (tmp_tok != NULL) {						\
			pr_debug("2 found %s\n", tmp_tok);			\
			continue;						\
		}								\
		/*if str is not found, exit with error */			\
		if (strstr(tok, _str) == NULL) {				\
			pr_err("%s not found\n", _str);				\
			break;							\
		}								\
		/*search for the ": " string, if not found, exit with error */	\
		tmp_tok = strstr(tok, ": ");					\
		if (tmp_tok == NULL) {						\
			pr_err("1 Invalid string(%s)!\n", tok);			\
			break;							\
		}								\
		/* check if string contains quotes, otherwise exit with error*/	\
		if (strncmp(&tmp_tok[2], &ascii_code, 1) != 0) {		\
			pr_err("quotes not found (%s)!\n", tok);		\
			break;							\
		}								\
		/* check if string contains a "," at the end and remove */	\
		if (strstr(tmp_tok, ",") == NULL)				\
			tmp_tok[strlen(tmp_tok) - 1] = 0;			\
		else								\
			tmp_tok[strlen(tmp_tok) - 2] = 0;			\
		/* retrieve the MAC address from the string */			\
		if (sscanf(&tmp_tok[3],						\
			   "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",			\
			   &mac[0], &mac[1], &mac[2], &mac[3],			\
			   &mac[4], &mac[5]) == ETH_ALEN) {			\
			pr_debug("found mac_addr %x:%x:%x:%x:%x:%x\n",		\
				 mac[0], mac[1], mac[2],			\
				 mac[3], mac[4], mac[5]);			\
			/* Copy the mac address */				\
			memcpy(_p, mac, 6);					\
			found = 1;						\
		} else {							\
			break;							\
		}								\
	} while (!found);							\
} while (0)

/**
 * json_buffer_to_input_str
 *
 * Description: search for a specified string and retrieve its string value from a
 * JSON format buffer.
 *
 * NOTE: This function does not extract the values from regular JSON file. The file should
 * be according to the json_json_print_to_buffer macro above
 *
 * NOTE: Currently the macros parse only the matching string, and do not search in the input
 * buffer (meaning the calling order should be the same as the written order)
 *
 * An example for reading from JSON format file:
 *	Retireve pp2_id and pool_id:
 *		json_buffer_to_input_mac(sec, "mac_address", port->mac_data.mac);
 *
 * @param[in]	buf	A pointer to the reading buffer.
 * @param[in]	_str	A pointer to the matching string
 *
 * @param[out]	_p	The parameter to update with the value found
 */
#define json_buffer_to_input_str(buff, _str, _p)				\
do {										\
	int	found = 0;							\
	char	*tok, *tmp_tok;							\
	/*ascii_code for quotes. This is for checkpatch compatibility. */	\
	char	ascii_code = 34;						\
	/* First, check if the string exists in the buffer */			\
	if (!strstr(buff, _str)) {						\
		pr_warn("string %s not found in input buff\n", _str);		\
		break;								\
	}									\
	/*Go over the input buffer untile the match str is found. */		\
	do {									\
		tok = strsep(&buff, "\n");					\
		/*if a "{" is found, go to the next line in buffer. */		\
		tmp_tok = strstr(tok, "{");					\
		if (tmp_tok != NULL) {						\
			pr_debug("1 found %s\n", tmp_tok);			\
			continue;						\
		}								\
		/*if a "}" is found, go to the next line in buffer. */		\
		tmp_tok = strstr(tok, "}");					\
		if (tmp_tok != NULL) {						\
			pr_debug("2 found %s\n", tmp_tok);			\
			continue;						\
		}								\
		/*if str is not found, exit with error */			\
		if (strstr(tok, _str) == NULL) {				\
			pr_err("%s not found\n", _str);				\
			break;							\
		}								\
		/*search for the ": " string, if not found, exit with error */	\
		tmp_tok = strstr(tok, ": ");					\
		if (tmp_tok == NULL) {						\
			pr_err("1 Invalid string(%s)!\n", tok);			\
			break;							\
		}								\
		/* check if string contains quotes, otherwise exit with error*/	\
		if (strncmp(&tmp_tok[2], &ascii_code, 1) != 0) {		\
			pr_err("quotes not found (%s)!\n", tok);		\
			break;							\
		}								\
		/* check if string contains a "," at the end and remove */	\
		if (strstr(tmp_tok, ",") == NULL)				\
			tmp_tok[strlen(tmp_tok) - 1] = 0;			\
		else								\
			tmp_tok[strlen(tmp_tok) - 2] = 0;			\
		/* Copy the string */						\
		strcpy(_p, &tmp_tok[3]);					\
		found = 1;							\
	} while (!found);							\
} while (0)


/* TODO: Replace Q&D with for_loop, 20 is hardcoded in sscanf */
#define MAX_OBJ_STRING 20

int mv_sys_match(const char *match, const char *obj_type, u8 hierarchy_level, u8 id[]);

void mem_disp(const char *_p, int len);

void mv_mem_dump(const unsigned char *p, unsigned int len);
void mv_mem_dump_words(const u32 *p, u32 words, int be);

enum musdk_lnx_id {
	LNX_VER_INVALID = -1,
	LNX_4_4_x = 0,
	LNX_OTHER = 1 /* Currently LNX_4_14_x */
};

enum musdk_lnx_id lnx_id_get(void);
int lnx_is_mainline(enum musdk_lnx_id lnx_id);


#endif /* __LIB_MISC_H__ */
