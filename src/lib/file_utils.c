/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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

#include "std_internal.h"
#include "lib/file_utils.h"

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
		pr_info("error %d\n", errno);
		close(fd);
		return -EIO;
	}

	close(fd);
	return 0;
}

int read_file_to_buf(char *file_name, char *buff, u32 size)
{
	size_t	s;
	int	fd;

	/* Open file */
	fd = open(file_name, O_RDONLY);
	if (fd == -1) {
		pr_err("Failed to open file %s\n", file_name);
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

