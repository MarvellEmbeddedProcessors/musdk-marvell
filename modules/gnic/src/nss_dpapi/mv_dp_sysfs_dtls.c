/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/capability.h>
#include <linux/inet.h>

#include "mv_dp_defs.h"
#include "mv_dp_sysfs.h"
#include "mv_nss_dp.h"
#include "mv_dp_main.h"

#define MV_DP_DTLS_SEQ_ID_LEN		6
#define MV_DP_DTLS_KEY_128_MAX_LEN	16
#define MV_DP_DTLS_KEY_256_MAX_LEN	32
#define MV_DP_DTLS_MAC_SEC_LEN		64

#define MV_DP_DTLS_MODE_NAME_SIZE		(sizeof(dtls_mode_name) / sizeof(*dtls_mode_name))

static char *dtls_mode_name[] = {
				"MV_NSS_DP_AES_128_CBC_HMAC_SHA_1",
				"MV_NSS_DP_AES_256_CBC_HMAC_SHA_1",
				"MV_NSS_DP_AES_256_CBC_HMAC_SHA_256",
				"MV_NSS_DP_AES_128_GCM",
				"MV_NSS_DP_AES_256_GCM",
};

#define MV_DP_DTLS_MODE_LAST	MV_NSS_DP_AES_256_GCM



/* DTLS commands */
static void mv_dp_dtls_set(const char *dtls_str, size_t len);
static void mv_dp_dtls_get(int dtls_id);
static void mv_dp_dtls_show(int dtls_id);
static void mv_dp_dtls_delete(int dtls_id);

/* Utilities */
static int mv_dp_dtls_key_size_get(int mode);

/* DTLS callbacks */
static void mv_dp_dtls_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_dtls_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_dtls_show_cb(mv_nss_dp_event_t *event);
static void mv_dp_dtls_delete_cb(mv_nss_dp_event_t *event);



static ssize_t mv_dp_dtls_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  help           - Show Help\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat [FILE]         > dtls_set       - Set DTLS record from [FILE] to FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > dtls_get       - Read DTLS Record with [ID] from FW and print it in format of file\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > dtls_show      - Read DTLS Record with [ID] from FW and print it in parsed format\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > dtls_delete    - Delete DTLS Record with [ID] from FW\n");


	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         [ID]  = DTLS Record ID (0 - 65535)\n");


	return o;
}

static ssize_t mv_dp_dtls_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_dtls_sysfs_help(buf);
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_dtls_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_dtls_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b;


	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = b = 0;

	if (!strcmp(name, "dtls_set")) {
		mv_dp_dtls_set(buf, len);
	} else if (!strcmp(name, "dtls_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dtls_get(a);
	} else if (!strcmp(name, "dtls_show")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dtls_show(a);
	} else if (!strcmp(name, "dtls_delete")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dtls_delete(a);
	} else {
		goto err;
	}

	return len;

err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;


}

static DEVICE_ATTR(help,				S_IRUSR, mv_dp_dtls_sysfs_show, NULL);
static DEVICE_ATTR(dtls_set,				S_IWUSR, NULL, mv_dp_dtls_sysfs_store);
static DEVICE_ATTR(dtls_get,				S_IWUSR, NULL, mv_dp_dtls_sysfs_store);
static DEVICE_ATTR(dtls_show,				S_IWUSR, NULL, mv_dp_dtls_sysfs_store);
static DEVICE_ATTR(dtls_delete,				S_IWUSR, NULL, mv_dp_dtls_sysfs_store);


static struct attribute *mv_dp_dtls_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_dtls_set.attr,
	&dev_attr_dtls_get.attr,
	&dev_attr_dtls_show.attr,
	&dev_attr_dtls_delete.attr,

	NULL
};


static struct attribute_group mv_dp_dtls_sysfs_group = {
	.name = "dtls",
	.attrs = mv_dp_dtls_sysfs_attrs,
};



int mv_dp_dtls_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_dtls_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("DTLS sysFS group init failed %d\n", err);
		return err;
	}
	MV_DP_LOG_DBG1("DTLS sysFS INITALIZED\n");
	return err;
}

int mv_dp_dtls_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_dtls_sysfs_group);
	return 0;
}


/*

DTLS file format
----------------

Line 0:  ID: 0..65535			- record ID
Line 1:  VER: 0..255			- version
Line 2:  MODE: 0..3			- mode
Line 3:  EPOCH: 0..FFFF			- epoch
Line 4:  SEQ: XX XX XX XX XX XX		- sequence ID (6 bytes hex)
Line 5:  R_MSEC_LEN: 0..65535		- read mac secret length in bytes
Line 6:  W_MSEC_LEN: 0..65535		- write mac secret length in bytes
Line 7:  R_MSEC: XX XX ... XX XX	- read mac secret (hex)
Line 8:  W_MSEC: XX XX ... XX XX	- write mac secret (hex)
Line 9:  R_KEY: XX XX ... XX XX		- read key (hex)
Line 10: W_KEY: XX XX ... XX XX		- write key (hex)


DTLS print format
-----------------

Line 0: Record ID:			0..65535
Line 1: Version:			0..255
Line 2: Mode:				0..3
Line 3: Epoch:				0x0..0xFFFF
Line 4: Sequence ID:			XX XX XX XX XX XX
Line 5: Read mac secret  [size|data]:	65535 | XX XX ... XX XX
Line 6: Read key	 [size|data]:	65535 | XX XX ... XX XX
Line 7: Write mac secret [size|data]:	65535 | XX XX ... XX XX
Line 8: Write key	 [size|data]:	65535 | XX XX ... XX XX

*/



/* Print parsed DTLS record */

static const char *mv_dp_dtls_mode_name_get(int mode)
{
if (mode < MV_DP_DTLS_MODE_NAME_SIZE)
		return dtls_mode_name[mode];
	else
		return "INVALID";
}

static int mv_dp_dtls_key_size_get(int mode)
{
	switch (mode) {
	case MV_NSS_DP_AES_128_CBC_HMAC_SHA_1:
		return MV_DP_DTLS_KEY_128_MAX_LEN;
	case MV_NSS_DP_AES_256_CBC_HMAC_SHA_1:
		return MV_DP_DTLS_KEY_256_MAX_LEN;
	case MV_NSS_DP_AES_256_CBC_HMAC_SHA_256:
		return MV_DP_DTLS_KEY_256_MAX_LEN;
	case MV_NSS_DP_AES_128_GCM:
		return MV_DP_DTLS_KEY_128_MAX_LEN;
	case MV_NSS_DP_AES_256_GCM:
		return MV_DP_DTLS_KEY_256_MAX_LEN;
	default:
		return -1;
	}
}

static void mv_dp_dtls_print(mv_nss_dp_dtls_t *dtls)
{

	int i, key_size;


	if (!dtls) {
		MV_DP_CLI_FAIL("Null DTLS Record", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("\nRecord ID:	  %d\n", dtls->dtls_id);
	MV_DP_CLI_CONT("Version:	  %d\n", dtls->version);
	MV_DP_CLI_CONT("Mode:		  %d (%s)\n", dtls->mode, mv_dp_dtls_mode_name_get(dtls->mode));
	MV_DP_CLI_CONT("Epoch:		  0x%X\n", dtls->epoch);

	key_size = mv_dp_dtls_key_size_get(dtls->mode);
	if (-1 == key_size) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - MODE: %d\n", MV_NSS_DP_INVALID_PARAM, dtls->mode);
		return;
	}

	MV_DP_CLI_CONT("Sequence ID:	 ");
	for (i = 0; i < MV_DP_DTLS_SEQ_ID_LEN; i++)
		MV_DP_CLI_CONT(" %02X", dtls->seq_id[i]);
	MV_DP_CLI_CONT("\n");

	if (dtls->read_mac_secret_len > MV_DP_DTLS_MAC_SEC_LEN) {
		MV_DP_CLI_CONT("Read mac secret:  %d | INVALID KEY LEN\n", dtls->read_mac_secret_len);
	} else {
		MV_DP_CLI_CONT("Read mac secret:  %d |", dtls->read_mac_secret_len);
		for (i = 0; i < dtls->read_mac_secret_len; i++)
			MV_DP_CLI_CONT(" %02X", dtls->read_mac_secret[i]);
		MV_DP_CLI_CONT("\n");
	}

	MV_DP_CLI_CONT("Read key:	  %d |", key_size);
	for (i = 0; i < key_size; i++)
		MV_DP_CLI_CONT(" %02X", dtls->read_key[i]);
	MV_DP_CLI_CONT("\n");

	if (dtls->write_mac_secret_len > MV_DP_DTLS_MAC_SEC_LEN) {
		MV_DP_CLI_CONT("Write mac secret: %d | INVALID KEY LEN\n", dtls->read_mac_secret_len);
	} else {
		MV_DP_CLI_CONT("Write mac secret: %d |", dtls->write_mac_secret_len);
		for (i = 0; i < dtls->write_mac_secret_len; i++)
			MV_DP_CLI_CONT(" %02X", dtls->write_mac_secret[i]);
		MV_DP_CLI_CONT("\n");
	}

	MV_DP_CLI_CONT("Write key:	  %d |", key_size);
	for (i = 0; i < key_size; i++)
		MV_DP_CLI_CONT(" %02X", dtls->write_key[i]);
	MV_DP_CLI_CONT("\n");

	MV_DP_CLI_CONT("NONCE:	  %02X %02X %02X %02X\n", dtls->nonce[0], dtls->nonce[1],
			dtls->nonce[2], dtls->nonce[3]);
}


/* Print DTLS record in format of file*/
static void mv_dp_dtls_print_file(mv_nss_dp_dtls_t *dtls)
{

	int i, key_size;


	if (!dtls) {
		MV_DP_CLI_FAIL("Null DTLS Record", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if ((dtls->mode > MV_DP_DTLS_MODE_LAST) ||
	    (dtls->read_mac_secret_len > MV_DP_DTLS_MAC_SEC_LEN) ||
	    (dtls->write_mac_secret_len > MV_DP_DTLS_MAC_SEC_LEN)) {

		MV_DP_CLI_FAIL("Invalid DTLS Record", MV_NSS_DP_INVALID_PARAM);
		return;
	}


	MV_DP_CLI_CONT("ID: %d\n", dtls->dtls_id);
	MV_DP_CLI_CONT("VER: %d\n", dtls->version);
	MV_DP_CLI_CONT("MODE: %d\n", dtls->mode);
	MV_DP_CLI_CONT("EPOCH: %X\n", dtls->epoch);

	key_size = mv_dp_dtls_key_size_get(dtls->mode);
	if (-1 == key_size) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - MODE: %d\n", MV_NSS_DP_INVALID_PARAM, dtls->mode);
		return;
	}



	MV_DP_CLI_CONT("SEQ:");
	for (i = 0; i < MV_DP_DTLS_SEQ_ID_LEN; i++)
		MV_DP_CLI_CONT(" %02X", dtls->seq_id[i]);
	MV_DP_CLI_CONT("\n");

	MV_DP_CLI_CONT("R_MSEC_LEN: %d\n", dtls->read_mac_secret_len);
	MV_DP_CLI_CONT("W_MSEC_LEN: %d\n", dtls->write_mac_secret_len);

	MV_DP_CLI_CONT("R_MSEC:");
	for (i = 0; i < dtls->read_mac_secret_len; i++)
		MV_DP_CLI_CONT(" %02X", dtls->read_mac_secret[i]);
	MV_DP_CLI_CONT("\n");

	MV_DP_CLI_CONT("W_MSEC:");
	for (i = 0; i < dtls->write_mac_secret_len; i++)
		MV_DP_CLI_CONT(" %02X", dtls->write_mac_secret[i]);
	MV_DP_CLI_CONT("\n");

	MV_DP_CLI_CONT("R_KEY:");
	for (i = 0; i < key_size; i++)
		MV_DP_CLI_CONT(" %02X", dtls->read_key[i]);
	MV_DP_CLI_CONT("\n");

	MV_DP_CLI_CONT("W_KEY:");
	for (i = 0; i < key_size; i++)
		MV_DP_CLI_CONT(" %02X", dtls->write_key[i]);
	MV_DP_CLI_CONT("\n");
	MV_DP_CLI_CONT("NONCE: %02X %02X %02X %02X\n", dtls->nonce[0], dtls->nonce[1],
			dtls->nonce[2], dtls->nonce[3]);
}


static void mv_dp_dtls_set(const char *dtls_str, size_t len)
{
	mv_nss_dp_result_spec_t	res;
	mv_nss_dp_dtls_t	dtls;
	struct completion	*compl_ptr;
	int			rc, i, key_size;
	const char		*cur_ptr, *eof_ptr;

	if (!dtls_str) {
		MV_DP_CLI_FAIL("Null DTLS String", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	eof_ptr = dtls_str + len - 1;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	res.cb = mv_dp_dtls_set_cb;


	/* Convert string to structure */

	memset(&dtls, 0, sizeof(dtls));
	cur_ptr = dtls_str;

	rc = sscanf(cur_ptr, "ID: %hu\n", &(dtls.dtls_id));
	if (rc != 1) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - ID\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr = strchr(cur_ptr, '\n');
	if (!cur_ptr) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - Unexpexted EOF ID\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr++;



	rc = sscanf(cur_ptr, "VER: %hhu\n", &(dtls.version));
	if (rc != 1) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - VER\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr = strchr(cur_ptr, '\n');
	if (!cur_ptr) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - Unexpexted EOF VER\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr++;


	rc = sscanf(cur_ptr, "MODE: %u\n", &(dtls.mode));
	if (rc != 1) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - MODE\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr = strchr(cur_ptr, '\n');
	if (!cur_ptr) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - Unexpexted EOF MODE\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr++;


	key_size = mv_dp_dtls_key_size_get(dtls.mode);
	if (-1 == key_size) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - MODE: %d\n", MV_NSS_DP_INVALID_PARAM, dtls.mode);
		goto err;
	}

	rc = sscanf(cur_ptr, "EPOCH: %hx\n", &(dtls.epoch));
	if (rc != 1) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - EPOCH\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr = strchr(cur_ptr, '\n');
	if (!cur_ptr) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - Unexpexted EOF EPOCH\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr++;

	rc = sscanf(cur_ptr, "SEQ: %hhx %hhx %hhx %hhx %hhx %hhx\n",
		      &(dtls.seq_id[0]), &(dtls.seq_id[1]), &(dtls.seq_id[2]), &(dtls.seq_id[3]), &(dtls.seq_id[4]), &(dtls.seq_id[5]));
	if (rc != 6) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - SEQ\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr = strchr(cur_ptr, '\n');
	if (!cur_ptr) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - Unexpexted EOF SEQ\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr++;

	rc = sscanf(cur_ptr, "R_MSEC_LEN: %hu\n", &(dtls.read_mac_secret_len));
	if (rc != 1) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - R_MSEC_LEN\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	if (dtls.read_mac_secret_len > MV_DP_DTLS_MAC_SEC_LEN) {
		MV_DP_CLI_FAIL("Invalid DTLS R_MSEC_LEN %d Expected up to %d\n",
			       MV_NSS_DP_INVALID_PARAM,
			       dtls.read_mac_secret_len,
			       key_size);
		goto err;
	}
	cur_ptr = strchr(cur_ptr, '\n');
	if (!cur_ptr) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - Unexpexted EOF R_MSEC_LEN\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr++;

	rc = sscanf(cur_ptr, "W_MSEC_LEN: %hu\n", &(dtls.write_mac_secret_len));
	if (rc != 1) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - W_MSEC_LEN\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	if (dtls.write_mac_secret_len > MV_DP_DTLS_MAC_SEC_LEN) {
		MV_DP_CLI_FAIL("Invalid DTLS W_MSEC_LEN %d Expected up to %d",
			       MV_NSS_DP_INVALID_PARAM,
			       dtls.write_mac_secret_len,
			       key_size);
		goto err;
	}
	cur_ptr = strchr(cur_ptr, '\n');
	if (!cur_ptr) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - Unexpexted EOF W_MSEC_LEN\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr++;

	/*new parser**/
	/*check that there is enough space for 7 chars before eof*/
	if (eof_ptr - cur_ptr < 7) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - R_MSEC: Unexpected EOF\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	rc = strncmp(cur_ptr, "R_MSEC: ", 8);
	if (rc) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - 'R_MSEC: ' not found\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr += 7;

	for (i = 0; i < dtls.read_mac_secret_len; i++) {
		rc = sscanf(cur_ptr, " %hhx", &(dtls.read_mac_secret[i]));
		if (rc != 1) {
			MV_DP_CLI_FAIL("Invalid R_MSEC (size: %d)\n", MV_NSS_DP_INVALID_PARAM, i);
			goto err;
		}
		/*match hex*/
		cur_ptr += 3;
	}

	/*skip spaces and find \n*/
	while (*cur_ptr == ' ' && *cur_ptr != '\n' && cur_ptr != eof_ptr)
		cur_ptr++;

	if (*cur_ptr != '\n' || cur_ptr == eof_ptr) {
		MV_DP_CLI_FAIL("Invalid R_MSEC EOL expected at: %lu\n", MV_NSS_DP_INVALID_PARAM, eof_ptr - cur_ptr);
		goto err;
	}
	cur_ptr++;

	if (eof_ptr - cur_ptr < 7) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - W_MSEC: Unexpected EOF\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	rc = strncmp(cur_ptr, "W_MSEC: ", 8);
	if (rc) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - 'W_MSEC: ' not found\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr += 7;

	for (i = 0; i < dtls.read_mac_secret_len; i++) {
		rc = sscanf(cur_ptr, " %hhx", &(dtls.write_mac_secret[i]));
		if (rc != 1) {
			MV_DP_CLI_FAIL("Invalid W_MSEC (size: %d)\n", MV_NSS_DP_INVALID_PARAM, i);
			goto err;
		}
		/*match hex*/
		cur_ptr += 3;
	}

	/*skip spaces and find \n*/
	while (*cur_ptr == ' ' && *cur_ptr != '\n' && cur_ptr != eof_ptr)
		cur_ptr++;

	if (*cur_ptr != '\n' || cur_ptr == eof_ptr) {
		MV_DP_CLI_FAIL("Invalid W_MSEC EOL expected at: %lu\n", MV_NSS_DP_INVALID_PARAM, eof_ptr - cur_ptr);
		goto err;
	}
	cur_ptr++;

	if (eof_ptr - cur_ptr < 6) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - R_KEY: Unexpected EOF\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	rc = strncmp(cur_ptr, "R_KEY: ", 7);
	if (rc) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - 'R_KEY: ' not found\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr += 6;

	for (i = 0; i < key_size; i++) {
		rc = sscanf(cur_ptr, " %hhx", &(dtls.read_key[i]));
		if (rc != 1) {
			MV_DP_CLI_FAIL("Invalid R_KEY (size: %d)\n", MV_NSS_DP_INVALID_PARAM, i);
			goto err;
		}
		/*match hex*/
		cur_ptr += 3;
	}

	/*skip spaces and find \n*/
	while (*cur_ptr == ' ' && *cur_ptr != '\n' && cur_ptr != eof_ptr)
		cur_ptr++;

	if (*cur_ptr != '\n' || cur_ptr == eof_ptr) {
		MV_DP_CLI_FAIL("Invalid R_KEY EOL expected at: %lu\n", MV_NSS_DP_INVALID_PARAM, eof_ptr - cur_ptr);
		goto err;
	}
	cur_ptr++;

	if (eof_ptr - cur_ptr < 6) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - W_KEY: Unexpected EOF\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	rc = strncmp(cur_ptr, "W_KEY: ", 7);
	if (rc) {
		MV_DP_CLI_FAIL("Invalid DTLS File Format - 'W_KEY: ' not found\n", MV_NSS_DP_INVALID_PARAM);
		goto err;
	}
	cur_ptr += 6;

	for (i = 0; i < key_size; i++) {
		rc = sscanf(cur_ptr, " %hhx", &(dtls.write_key[i]));
		if (rc != 1) {
			MV_DP_CLI_FAIL("Invalid W_KEY (size: %d)\n", MV_NSS_DP_INVALID_PARAM, i);
			goto err;
		}
		/*match hex*/
		cur_ptr += 3;
	}

	if (cur_ptr < eof_ptr) {
		/*skip spaces and find \n*/
		while ((*cur_ptr == ' ' || *cur_ptr == '\n') && cur_ptr != eof_ptr)
			cur_ptr++;
		/* check for optional field nonce */
		if (!(*cur_ptr == ' ' || *cur_ptr == '\n')) {

			rc = sscanf(cur_ptr, "NONCE: %hhx %hhx %hhx %hhx\n", &(dtls.nonce[0]), &(dtls.nonce[1])
					, &(dtls.nonce[2]), &(dtls.nonce[3]));
			if (rc != 4) {
				MV_DP_CLI_FAIL("Invalid DTLS File Format - NONCE\n", MV_NSS_DP_INVALID_PARAM);
				goto err;
			}
			cur_ptr += 18;
			if (cur_ptr < eof_ptr) {
				/*skip spaces and find \n*/
				while ((*cur_ptr == ' ' || *cur_ptr == '\n') && cur_ptr != eof_ptr)
					cur_ptr++;
				if (!(*cur_ptr == ' ' || *cur_ptr == '\n')) {
					MV_DP_CLI_FAIL("Invalid NONCE EOF expected at: %lu\n",
							MV_NSS_DP_INVALID_PARAM, eof_ptr - cur_ptr);
					goto err;
				}
			}
		}
	}

	res.xid = dtls.dtls_id;

	if (MV_NSS_DP_OK != mv_nss_dp_dtls_set(&dtls, &res)) {
		MV_DP_CLI_FAIL("Set DTLS Record ID: %d\n", MV_NSS_DP_FAILED, dtls.dtls_id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Set DTLS Record ID: %d\n", dtls.dtls_id);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Set DTLS Record ID: %d - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, dtls.dtls_id);
		kfree(res.cookie);
	}

	return;

err:
	mv_dp_dtls_print_file(&dtls);
}



static void mv_dp_dtls_get(int dtls_id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_dtls_get_cb;
	res.xid = dtls_id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_dtls_get(dtls_id, &res)) {
		MV_DP_CLI_FAIL("Get DTLS Record ID: %d\n", MV_NSS_DP_FAILED, dtls_id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get DTLS Record ID: %d\n", dtls_id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get DTLS Record ID: %d - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, dtls_id);
		kfree(res.cookie);
	}
}


static void mv_dp_dtls_show(int dtls_id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_dtls_show_cb;
	res.xid = dtls_id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_dtls_get(dtls_id, &res)) {
		MV_DP_CLI_FAIL("Show DTLS Record ID: %d\n", MV_NSS_DP_FAILED, dtls_id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Show DTLS Record ID: %d\n", dtls_id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Show DTLS Record ID: %d - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, dtls_id);
		kfree(res.cookie);
	}
}

static void mv_dp_dtls_delete(int dtls_id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_dtls_delete_cb;
	res.xid = dtls_id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_dtls_delete(dtls_id, &res)) {
		MV_DP_CLI_FAIL("Delete DTLS Record ID: %d\n", MV_NSS_DP_FAILED, dtls_id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Delete DTLS Record ID: %d\n", dtls_id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Delete DTLS Record ID: %d - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, dtls_id);
		kfree(res.cookie);
	}
}



void mv_dp_dtls_set_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Set DTLS Record ID: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Set DTLS Record ID: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_dtls_get_cb(mv_nss_dp_event_t *event)
{

	/*Print response and return OK;*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get DTLS record ID: %d Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.dtls) {
		MV_DP_CLI_FAIL_CB("Get DTLS record ID: %d - Empty Config - Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get DTLS record ID: %d\n", event, event->xid);
	mv_dp_dtls_print_file(event->params.dtls);
	MV_DP_CLI_CONT("|DTLS record END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_dtls_show_cb(mv_nss_dp_event_t *event)
{

	/*Print response and return OK;*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Show DTLS record ID: %d Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.dtls) {
		MV_DP_CLI_FAIL_CB("Show DTLS record ID: %d - Empty Config - Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Show DTLS record ID: %d\n", event, event->xid);
	mv_dp_dtls_print(event->params.dtls);
	MV_DP_CLI_CONT("|DTLS record END|<\n");


err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_dtls_delete_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Delete DTLS Record ID: %d Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Delete DTLS Record ID: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

