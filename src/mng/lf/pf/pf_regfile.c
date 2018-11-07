/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "pf_regfile: " fmt

#include "std_internal.h"
#include "env/spinlock.h"
#include "pf_regfile.h"

struct regfile {
	void  *map_addr;
	int   size;
};

#define REGFILE_MAX_FILES_OPEN	128
struct regfile regfile_arr[REGFILE_MAX_FILES_OPEN];

/* lock to protect regfile array */
spinlock_t *regfile_lock;

/* indicator that regfile was initialized */
static int init_done;

int regfile_init(void)
{
	pr_debug("Register file utility init\n");

	if (init_done)
		return 0;

	/* Reset DB */
	memset(regfile_arr, 0x0, sizeof(regfile_arr));

	/* Init lock */
	regfile_lock = spin_lock_create();
	if (regfile_lock == NULL) {
		pr_err("Failed to initialize regfile lock\n");
		return -1;
	}

	init_done = 1;

	return 0;
}

void regfile_destroy(void)
{
	int i;

	pr_debug("Register file utility destroy\n");

	/* Release all mapping */
	for (i = 0; i < REGFILE_MAX_FILES_OPEN; i++) {
		if (regfile_arr[i].map_addr != NULL)
			/* un-map the file */
			if (munmap(regfile_arr[i].map_addr,	regfile_arr[i].size) == -1) {
				/* report error but don't exit as other resources
				 * should be released
				 */
				pr_err("Error un-mapping the file: %s", strerror(errno));
			}
	}

	spin_lock_destroy(regfile_lock);

	init_done = 0;
}

void *regfile_open(char *file_name, int file_size)
{
	int idx, fd, ret;
	void *map_addr = NULL;

	/* Look for available slot for saving the mapping address and size.
	 * It will be used during close functionality.
	 */
	spin_lock(regfile_lock);

	for (idx = 0; idx < REGFILE_MAX_FILES_OPEN; idx++) {
		if (regfile_arr[idx].map_addr == NULL) {
			/* idx has now the next available entry */
			break;
		}
	}

	spin_unlock(regfile_lock);

	if (idx == REGFILE_MAX_FILES_OPEN) {
		pr_err("Error: No available slot for mapping file %s\n", file_name);
		goto early_error;
	}

	fd = open(file_name, O_RDWR | O_CREAT);
	if (fd == -1) {
		pr_err("Failed to open file %s\n", file_name);
		goto early_error;
	}

	/* Stretch the file size to the size of the mmapp request */
	ret = lseek(fd, file_size - 1, SEEK_SET);
	if (ret == -1) {
		pr_err("Error calling lseek() to 'stretch' file %s: %s\n", file_name, strerror(errno));
		goto error;
	}

	/* Something needs to be written at the end of the file to
	 * have the file actually have the new size.
	 * Just writing an empty string at the current file position will do.
	 */
	ret = write(fd, "", 1);
	if (ret != 1) {
		pr_err("Error writing last byte of file %s: %s\n", file_name, strerror(errno));
		goto error;
	}

	/* Now the file is ready to be mapped */
	map_addr = mmap(0, file_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (map_addr == MAP_FAILED) {
		pr_err("Error mapping the file %s\n", file_name);
		goto error;
	}

	/* save mapping data */
	regfile_arr[idx].map_addr = map_addr;
	regfile_arr[idx].size = file_size;

	pr_debug("Register file %s was mapped to address %p\n", file_name, map_addr);

	return map_addr;

error:
	if (map_addr != NULL)
		munmap(map_addr, file_size);

	close(fd);
early_error:
	return NULL;
}

int regfile_close(void *addr)
{
	int i;

	/* look for map size in the map array using the address as a key */
	for (i = 0; i < REGFILE_MAX_FILES_OPEN; i++) {
		if (regfile_arr[i].map_addr == addr)
			break;
	}

	if (i == REGFILE_MAX_FILES_OPEN) {
		pr_err("Failed to close file mapped to 0x%p. No such file\n", addr);
		return -ENODEV;
	}

	/* mark the slot as available */
	regfile_arr[i].map_addr = NULL;

	/* un-map the file */
	if (munmap(addr, regfile_arr[i].size) == -1) {
		pr_err("Error un-mapping the file: %s", strerror(errno));
		return errno;
	}

	pr_debug("Register file mapped to address %p was closed\n", addr);

	return 0;
}
