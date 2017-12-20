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

#ifndef __LIB_MISC_H__
#define __LIB_MISC_H__

/* Serialization helpers */

#define FILE_MAX_LINE_CHARS	80

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
 *	json_print_to_buffer(buff, "\t\t\"pool-%d:%d\": {\n", pool->pp2_id, pool->id);
 *
 * 2) Write pool_id and pp2_id parameters (inside the above section):
 *	json_print_to_buffer(buff, "\t\t\t\"pp2_id\": %u,\n", pool->pp2_id);
 *	json_print_to_buffer(buff, "\t\t\t\"id\": %u,\n", pool->id);
 *
 * @param[in]	buf	A pointer to the writing buffer.
 * @param[in]	...	__VA_ARGS__ containing the string to write and the formating options
 */
#define json_print_to_buffer(buff, ...)					\
do {									\
	/* Check the input string can be written in the input buffer */	\
	char tmp_buf[FILE_MAX_LINE_CHARS];				\
	ans = snprintf(tmp_buf, size - pos, __VA_ARGS__);		\
	if (ans <= (size - pos))					\
		strncpy(&buff[pos], &tmp_buf[0], (size_t)ans);		\
	else								\
		pr_warn("skip write (%s): buf too short (%d)!\n",	\
			tmp_buf, size);					\
	pos += ans;							\
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
			pr_info("found mac_addr %x:%x:%x:%x:%x:%x\n",		\
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

#endif /* __LIB_MISC_H__ */
