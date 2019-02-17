/*
* ***************************************************************************
* Copyright (C) 2018 Marvell International Ltd.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/
#include <linux/platform_device.h>
#include "giu_nic_sysfs_md_cls.h"
#include "giu_nic_md.h"

#define MD_TBL_ENTRY_MAC_STR_LEN    17
#define MD_TBL_ENTRY_METADATA_LEN  (META_DATA_STR_LEN + MD_TBL_ENTRY_MAC_STR_LEN + 2)

static ssize_t metadata_classify_help(char *b)
{
	int o = 0;

	o += sprintf(b+o, "echo [dest mac addr] [md - 32 digits] > metadata_tbl\n");
	o += sprintf(b+o, "cat metadata_tbl - print the metadata classify table\n");

	return o;
}

static int print_metadata_tbl_entry(char *buf, int offset, struct md_tbl_entry *entry)
{
	int len = 0, i;

	for (i = 0 ; i < ETH_ALEN ; i++)
		len += snprintf(buf + offset + len, 1024, "%02x:", entry->eth_addr[i]);

	len -= 1;
	len += snprintf(buf + offset + len, META_DATA_STR_LEN + 1, " %016llx", entry->md.meta_data[0]);
	len += snprintf(buf + offset + len, META_DATA_STR_LEN + 1, "%016llx\n", entry->md.meta_data[1]);

	return len;
}


static ssize_t metadata_classify_info_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int len = 0, md_tbl_size, i;
	struct md_tbl_entry *entry;
	const char *name = attr->attr.name;

	if (!strcmp(name, "metadata_tbl")) {
		md_tbl_size = agnic_get_metadata_tbl_size();
		for (i = 0 ; i < md_tbl_size ; i++) {
			entry = agnic_get_metadata_tbl_entry(i);
			len += print_metadata_tbl_entry(buf, len, entry);
		}
	} else
		return metadata_classify_help(buf);

	return len;
}


static int metadata_tbl_add_entry(const char *buf, size_t len)
{
	struct md_tbl_entry entry;
	char buf2[MD_TBL_ENTRY_METADATA_LEN + 1], *p;
	int i;

	memcpy(buf2, buf, MD_TBL_ENTRY_MAC_STR_LEN);
	buf2[MD_TBL_ENTRY_MAC_STR_LEN] = '\0';
	p = buf2;
	for (i = 0 ; i < ETH_ALEN ; i++) {
		p[2] = '\0';
		if (kstrtou8(p, 16, &entry.eth_addr[i]) != 0)
			return -EINVAL;

		p += 3;
	}

	memcpy(buf2, buf + MD_TBL_ENTRY_MAC_STR_LEN + 1, META_DATA_LEN);
	buf2[META_DATA_LEN] = '\0';

	if (kstrtou64(buf2, 16, &entry.md.meta_data[0]) != 0)
		return -EINVAL;

	memcpy(buf2, buf + MD_TBL_ENTRY_MAC_STR_LEN + 1 + META_DATA_LEN, META_DATA_LEN);
	buf2[META_DATA_LEN] = '\0';

	if (kstrtou64(buf2, 16, &entry.md.meta_data[1]) != 0)
		return -EINVAL;

	if (unlikely(!agnic_add_metadata_tbl_entry(&entry)))
		return -EINVAL;

	return len;
}

static ssize_t metadata_classify_info_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t len)
{
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "metadata_tbl")) {
		if (len != MD_TBL_ENTRY_METADATA_LEN)
			return -EINVAL;

		return metadata_tbl_add_entry(buf, len);
	}

	return -ENOENT;
}

static DEVICE_ATTR(help, S_IRUSR, metadata_classify_info_show, NULL);
static DEVICE_ATTR(metadata_tbl, S_IRUSR | S_IWUSR, metadata_classify_info_show, metadata_classify_info_store);

static struct attribute *metadata_classify_info[] = {
	&dev_attr_help.attr,
	&dev_attr_metadata_tbl.attr,
	NULL
};

static struct attribute_group metadata_classify_info_group = {
	.name = "metadata_classify_info",
	.attrs = metadata_classify_info,
};

int agnic_metadata_classify_sysfs_init(struct kobject *gbe_kobj)
{
	int err;

	err = sysfs_create_group(gbe_kobj, &metadata_classify_info_group);
	if (err)
		pr_err("sysfs group %s failed %d\n", metadata_classify_info_group.name, err);

	return err;
}

int agnic_metadata_classify_sysfs_exit(struct kobject *gbe_kobj)
{
	sysfs_remove_group(gbe_kobj, &metadata_classify_info_group);

	return 0;
}



