/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
		pr_err("Illegal match str!\n");
		return -1;
	}

	if (strcmp(obj_type, tok)) {
		pr_err("String {%s} does not match obj_type {%s}\n", tok, obj_type);
		return(-1);
	}

	if (hierarchy_level == 1) {
		tok = mv_strtok(NULL, "");
		if (!tok) {
			pr_err("Illegal match str!\n");
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
			pr_err("Illegal match str!\n");
			return -1;
		}
		rc = kstrtou8(tok, 10, &id[0]);
		if (rc) {
			pr_err("String \"%s\" is not a number.\n", tok);
			return rc;
		}
		tok = mv_strtok(NULL, "");
		if (!tok) {
			pr_err("Illegal match str!\n");
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
	else if ((MAJOR(lk_ver) == 4) && (MINOR(lk_ver) == 14))
		lnx_id = LNX_4_14_x;
	else
		lnx_id = LNX_OTHER;

	return lnx_id;
}

int lnx_is_mainline(enum musdk_lnx_id lnx_id)
{
	return (lnx_id > LNX_4_4_x);
}
