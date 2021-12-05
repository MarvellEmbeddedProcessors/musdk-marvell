/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
		strncpy(&buff[pos], &tmp_buf[0], (size_t)(str_len + tab_len));	\
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
	LNX_4_14_x = 1,
	LNX_OTHER = 2 /* Currently LNX_5_x */
};

enum musdk_lnx_id lnx_id_get(void);
int lnx_is_mainline(enum musdk_lnx_id lnx_id);


#endif /* __LIB_MISC_H__ */
