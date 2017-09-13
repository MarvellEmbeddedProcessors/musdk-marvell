/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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
