/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "env/netdev.h"

#define FEATSTRS_MAX 64

struct netdev_featstrs {
	char *s[FEATSTRS_MAX];
};

/*
 * Feature interface
 */

static void
mv_netdev_clean_featstrs(struct netdev_featstrs *fs)
{
	int i;

	for (i = 0; i < FEATSTRS_MAX; i++) {
		if (fs->s[i] == NULL)
			continue;
		free(fs->s[i]);
		fs->s[i] = NULL;
	}
}

static int
mv_netdev_get_featstrs(int fd, struct ifreq *ifr, struct netdev_featstrs *fs)
{
	struct {
		struct ethtool_sset_info hdr;
		uint32_t buf[1];
	} sset_cmd = {0};
	struct {
		struct ethtool_gstrings gs;
		char data[0];
	} *gstrs;
	int32_t len;
	char *s;
	int i, ret;

	sset_cmd.hdr.cmd = ETHTOOL_GSSET_INFO;
	sset_cmd.hdr.sset_mask = 1 << ETH_SS_FEATURES;

	ifr->ifr_data = &sset_cmd;
	ret = ioctl(fd, SIOCETHTOOL, ifr);
	if (ret) {
		pr_err("Could not get feature count (%s)\n", strerror(errno));
		return -1;
	}

	memcpy(&len, sset_cmd.hdr.data, sizeof(int32_t));
	if (len < 0 || len > FEATSTRS_MAX) {
		pr_err("invalid feature count %d\n", len);
		return -1;
	}

	gstrs = calloc(1, sizeof(struct ethtool_gstrings) + len * ETH_GSTRING_LEN);
	gstrs->gs.cmd = ETHTOOL_GSTRINGS;
	gstrs->gs.string_set = ETH_SS_FEATURES;
	gstrs->gs.len = len;

	ifr->ifr_data = gstrs;
	ret = ioctl(fd, SIOCETHTOOL, ifr);
	if (ret) {
		pr_err("Could not get feature strings (%s)\n", strerror(errno));
		free(gstrs);
		return -1;
	}

	s = gstrs->data;
	for (i = 0; i < len; i++) {
		s[ETH_GSTRING_LEN - 1] = '\0';

		fs->s[i] = strdup(s);
		if (fs->s[i] == NULL) {
			pr_err("Failed to allocate feature strings\n");
			mv_netdev_clean_featstrs(fs);
			free(gstrs);
			return -1;
		}

		s += ETH_GSTRING_LEN;
	}
	free(gstrs);

	return 0;
}

static int
mv_netdev_set_feature_ioctl(int fd, struct ifreq *ifr,  int bit, int val)
{
	struct {
		struct ethtool_sfeatures sf;
		struct ethtool_set_features_block blk[2];
	} cmd = {0};
	int word = bit / 32;
	int sbit = bit % 32;
	int ret;

	cmd.sf.cmd = ETHTOOL_SFEATURES;
	cmd.sf.size = 2;

	ifr->ifr_data = &cmd;

	cmd.blk[word].valid |= 1 << sbit;
	cmd.blk[word].requested = val << sbit;

	ret = ioctl(fd, SIOCETHTOOL, ifr);
	if (ret) {
		if (ret < 0)
			pr_err("Error setting bit (%s)\n", strerror(errno));
		else
			pr_err("Error setting bit (%d)\n", ret);
		return -1;
	}

	return 0;
}

int mv_netdev_feature_set(const char *netdev, const char *featstr, int val)
{
	struct netdev_featstrs fs = {0};
	struct ifreq ifr = {0};
	int fbit;
	int fd;
	int ret;
	int i;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == -1) {
		pr_err("can't open socket: errno %d", errno);
		return -EFAULT;
	}

	sprintf(ifr.ifr_name, "%s", netdev);

	if (mv_netdev_get_featstrs(fd, &ifr, &fs)) {
		close(fd);
		return -EFAULT;
	}

	for (i = 0; i < FEATSTRS_MAX; i++) {
		if (fs.s[i] == NULL)
			continue;

		if (strcmp(fs.s[i], featstr))
			continue;

		fbit = i;
		break;
	}

	if (i == FEATSTRS_MAX) {
		pr_err("failed to find feature %s\n", featstr);
		close(fd);
		mv_netdev_clean_featstrs(&fs);
		return -ENOENT;
	}

	ret = mv_netdev_set_feature_ioctl(fd, &ifr, fbit, !!val);

	close(fd);
	mv_netdev_clean_featstrs(&fs);

	if (ret)
		return -EIO;

	return 0;
}

/* Send IOCTL to linux */
int mv_netdev_ioctl(u32 ctl, struct ifreq *s)
{
	int rc;
	int fd;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == -1) {
		pr_err("can't open socket: errno %d", errno);
		return -EFAULT;
	}

	rc = ioctl(fd, ctl, (char *)s);
	if (rc == -1) {
		pr_err("ioctl request failed: errno %d\n", errno);
		close(fd);
		return -EFAULT;
	}
	close(fd);
	return 0;
}
