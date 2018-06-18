/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include <sys/stat.h>

#include "std_internal.h"
#include "file_map.h"

int file_map(char *file_name, struct file_map_info *map_info)
{
	int fd;
	void *file_map;
	struct stat st;

	map_info->map_addr = NULL;

	/* Get file size */
	stat(file_name, &st);
	map_info->map_size = st.st_size;

	fd = open(file_name, O_RDWR);
	if (fd == -1) {
		pr_err("Failed to open file %s\n", file_name);
		return -1;
	}

	/* Now the file is ready to be mapped */
	file_map = mmap(0, map_info->map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

	/* Close the file*/
	close(fd);

	if (file_map == MAP_FAILED) {
		pr_err("Error mapping the file %s\n", file_name);
		return -1;
	}

	/* Save map address */
	map_info->map_addr = file_map;

	return 0;
}

int file_unmap(struct file_map_info *map_info)
{
	/* un-map the file */
	if (munmap(map_info->map_addr, map_info->map_size) == -1) {
		pr_err("Error un-mapping the file: %s", strerror(errno));
		return errno;
	}

	pr_info("file mapped to address %p was closed\n", map_info->map_addr);

	return 0;
}
