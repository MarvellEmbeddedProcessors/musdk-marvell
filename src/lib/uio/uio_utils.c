/* Copyright (c) 2016 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "lib/uio_helper.h"

#define UIO_BASE_PATH		"/sys/class/uio"
#define MAX_FILE_NAME_LEN	64

static int get_uio_num_from_filename(char *name)
{
	enum scan_states { ss_u, ss_i, ss_o, ss_num, ss_err };
	enum scan_states state = ss_u;
	int i = 0, num = -1;
	char ch = name[0];

	while (ch && (state != ss_err)) {
		switch (ch) {
		case 'u':
			if (state == ss_u)
				state = ss_i;
			else
				state = ss_err;
			break;
		case 'i':
			if (state == ss_i)
				state = ss_o;
			else
				state = ss_err;
			break;
		case 'o':
			if (state == ss_o)
				state = ss_num;
			else
				state = ss_err;
			break;
		default:
			if ((ch >= '0') && (ch <= '9')
				&& (state == ss_num)) {
				if (num < 0)
					num = (ch - '0');
				else
					num = (num * 10) + (ch - '0');
			} else
				state = ss_err;
		}
		i++;
		ch = name[i];
	}
	if (state == ss_err)
		num = -1;

	return num;
}

static int get_uio_line_from_file(char *filename, char *linebuf)
{
	char *s;
	int i;
	FILE *file = fopen(filename, "r");

	if (!file)
		return -1;

	memset(linebuf, 0, UIO_MAX_NAME_SIZE);
	s = fgets(linebuf, UIO_MAX_NAME_SIZE, file);
	if (!s) {
		fclose(file);
		return -2;
	}
	for (i = 0; (*s) && (i < UIO_MAX_NAME_SIZE); i++) {
		if (*s == '\n')
			*s = 0;
		s++;
	}
	fclose(file);

	return 0;
}

static struct uio_info_t *get_uio_info_from_name(char *name, int filter_num)
{
	struct uio_info_t *info;
	int num = get_uio_num_from_filename(name);

	if (num < 0)
		return NULL;
	if ((filter_num >= 0) && (num != filter_num))
		return NULL;

	info = kmalloc(sizeof(struct uio_info_t), GFP_KERNEL);
	if (!info)
		return NULL;
	memset(info, 0, sizeof(struct uio_info_t));
	info->uio_num = num;

	return info;
}

static struct uio_info_t *get_uio_info_byname(char *name, const char *filter_name)
{
	struct uio_info_t *info;
	char linebuf[UIO_MAX_NAME_SIZE];
	char filename[255];

	snprintf(filename, sizeof(filename), "%s/%s/name", UIO_BASE_PATH, name);
	if (get_uio_line_from_file(filename, linebuf))
		return NULL;

	if (strncmp(linebuf, filter_name, strlen(filter_name)))
		return NULL;

	info = kmalloc(sizeof(struct uio_info_t), GFP_KERNEL);
	if (!info)
		return NULL;
	memset(info, 0, sizeof(struct uio_info_t));
	info->uio_num = get_uio_num_from_filename(name);

	return info;
}

static void uio_free_dev_attrs(struct uio_info_t *info)
{
	struct uio_dev_attr_t *p1, *p2;

	p1 = info->dev_attrs;
	while (p1) {
		p2 = p1->next;
		kfree(p1);
		p1 = p2;
	}
	info->dev_attrs = NULL;
}

static int get_uio_event_count(struct uio_info_t *info)
{
	int ret;
	char filename[MAX_FILE_NAME_LEN];
	FILE *file;

	info->event_count = 0;
	snprintf(filename, sizeof(filename), "%s/uio%d/event", UIO_BASE_PATH, info->uio_num);
	file = fopen(filename, "r");
	if (!file)
		return -1;
	ret = fscanf(file, "%d", (int *)(&info->event_count));
	fclose(file);
	if (ret < 0)
		return -2;

	return 0;
}

static int get_uio_mem_addr(struct uio_info_t *info, int map_num)
{
	int ret;
	char filename[MAX_FILE_NAME_LEN];
	FILE *file;

	if (map_num >= MAX_UIO_MAPS)
		return -1;
	info->maps[map_num].addr = UIO_INVALID_ADDR;
	snprintf(filename, sizeof(filename), "%s/uio%d/maps/map%d/addr", UIO_BASE_PATH, info->uio_num, map_num);
	file = fopen(filename, "r");
	if (!file)
		return -1;
	ret = fscanf(file, "0x%lx", &info->maps[map_num].addr);
	fclose(file);
	if (ret < 0)
		return -2;
	return 0;
}

static int get_uio_mem_name(struct uio_info_t *info, int map_num)
{
	char filename[MAX_FILE_NAME_LEN];

	if (map_num >= MAX_UIO_MAPS)
		return -1;
	snprintf(filename, sizeof(filename), "%s/uio%d/maps/map%d/name", UIO_BASE_PATH, info->uio_num, map_num);
	return get_uio_line_from_file(filename, info->maps[map_num].name);
}

static int get_uio_mem_size(struct uio_info_t *info, int map_num)
{
	int ret;
	char filename[MAX_FILE_NAME_LEN];
	FILE *file;

	if (map_num >= MAX_UIO_MAPS)
		return -1;
	info->maps[map_num].size = UIO_INVALID_SIZE;
	snprintf(filename, sizeof(filename), "%s/uio%d/maps/map%d/size", UIO_BASE_PATH, info->uio_num, map_num);
	file = fopen(filename, "r");
	if (!file)
		return -1;
	ret = fscanf(file, "0x%lx", &info->maps[map_num].size);
	fclose(file);
	if (ret < 0)
		return -2;
	return 0;
}

static int get_uio_name(struct uio_info_t *info)
{
	char filename[MAX_FILE_NAME_LEN];

	snprintf(filename, sizeof(filename), "%s/uio%d/name", UIO_BASE_PATH, info->uio_num);
	return get_uio_line_from_file(filename, info->name);
}

static int get_uio_version(struct uio_info_t *info)
{
	char filename[MAX_FILE_NAME_LEN];

	snprintf(filename, sizeof(filename), "%s/uio%d/version", UIO_BASE_PATH, info->uio_num);
	return get_uio_line_from_file(filename, info->version);
}

void uio_free_info(struct uio_info_t *info)
{
	struct uio_info_t *p1, *p2;

	p1 = info;
	while (p1) {
		uio_free_dev_attrs(p1);
		p2 = p1->next;
		kfree(p1);
		p1 = p2;
	}
}

void uio_free_mem_info(struct uio_mem_t *info)
{
	kfree(info);
	info = NULL;
}

struct uio_info_t *uio_find_devices_byname(const char *filter_name)
{
	struct dirent **namelist;
	struct uio_info_t *infolist = NULL, *infp, *last;
	int n;

	n = scandir(UIO_BASE_PATH, &namelist, 0, alphasort);
	if (n <= 0) {
		pr_err("scandir for %s failed. errno = %d (%s)\n", UIO_BASE_PATH, errno, strerror(errno));
		return NULL;
	}
	while (n--) {
		infp = get_uio_info_byname(namelist[n]->d_name, filter_name);
		kfree(namelist[n]);
		if (!infp)
			continue;

		if (!infolist)
			infolist = infp;
		else
			last->next = infp;
		last = infp;
	}
	kfree(namelist);

	return infolist;
}

struct uio_mem_t *uio_find_mem_byname(struct uio_info_t *info, const char *filter)
{
	struct uio_info_t *infp = info;
	struct uio_mem_t *uiofdp = NULL;

	if (!infp || !filter)
		return NULL;

	while (infp) {
		int i;

		for (i = 0; i < MAX_UIO_MAPS; i++) {
			if (strncmp(infp->maps[i].name, filter,
				UIO_MAX_NAME_SIZE)) {
				continue;
			} else {
				uiofdp = kcalloc(1, sizeof(struct uio_info_t), GFP_KERNEL);
				uiofdp->map_num = i;
				uiofdp->fd = UIO_INVALID_FD;
				uiofdp->info = infp;
				return uiofdp;
			}
		}
		infp = infp->next;
	}

	return uiofdp;
}

struct uio_info_t *uio_find_devices(int filter_num)
{
	struct dirent **namelist;
	struct uio_info_t *infolist = NULL, *infp, *last;
	int n = scandir(UIO_BASE_PATH, &namelist, 0, alphasort);

	if (n < 0)
		return NULL;

	while (n--) {
		infp = get_uio_info_from_name(namelist[n]->d_name, filter_num);
		kfree(namelist[n]);
		if (!infp)
			continue;
		if (!infolist)
			infolist = infp;
		else
			last->next = infp;
		last = infp;
	}
	kfree(namelist);

	return infolist;
}

int uio_get_all_info(struct uio_info_t *info)
{
	int i;

	if (!info)
		return -1;
	if ((info->uio_num < 0) || (info->uio_num > UIO_MAX_NUM))
		return -1;
	for (i = 0; i < MAX_UIO_MAPS; i++) {
		get_uio_mem_size(info, i);
		get_uio_mem_addr(info, i);
		get_uio_mem_name(info, i);
	}
	get_uio_event_count(info);
	get_uio_name(info);
	get_uio_version(info);

	return 0;
}

void *uio_single_mmap(struct uio_info_t *info, int map_num, int fd)
{
	if (!fd)
		return NULL;
	info->maps[map_num].mmap_result = UIO_MMAP_NOT_DONE;
	if (info->maps[map_num].size == UIO_INVALID_SIZE)
		return NULL;
	info->maps[map_num].mmap_result = UIO_MMAP_FAILED;
	info->maps[map_num].internal_addr =
		mmap(
			NULL,
			info->maps[map_num].size,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			fd,
			map_num * getpagesize()
		);

	if (info->maps[map_num].internal_addr != MAP_FAILED) {
		info->maps[map_num].mmap_result = UIO_MMAP_OK;
		return info->maps[map_num].internal_addr;
	}

	return NULL;
}

