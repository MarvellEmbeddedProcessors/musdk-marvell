/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "lib/file_utils.h"
#include <sys/stat.h>

/**********************
 * File reading/writing utilities
 ***********************/
int write_buf_to_file(char *file_name, char *buff, u32 size)
{
	size_t	s;
	int	fd;

	fd = open(file_name, O_RDWR | O_CREAT);
	if (fd == -1) {
		pr_err("Failed to open file %s\n", file_name);
		return -EIO;
	}

	/* Position offset to end of file */
	lseek(fd, 0, SEEK_END);

	s = write(fd, buff, size);
	if (s == -1) {
		pr_err("error %d\n", errno);
		close(fd);
		return -EIO;
	}

	close(fd);

	sync();

	chmod(file_name, S_IWUSR | S_IROTH);

	return 0;
}

int read_file_to_buf(char *file_name, char *buff, u32 size)
{
	size_t	s;
	int	fd;

	/* Open file */
	fd = open(file_name, O_RDONLY);
	if (fd == -1) {
		pr_debug("Failed to open file %s\n", file_name);
		return -EINVAL;
	}

	/* Read file */
	s = read(fd, buff, size);
	if (s == -1) {
		pr_err("error %d\n", errno);
		close(fd);
		return -EINVAL;
	}
	close(fd);
	return 0;
}

