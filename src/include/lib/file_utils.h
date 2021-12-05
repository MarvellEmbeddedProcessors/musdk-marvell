/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __FILE_UTILS_H__
#define __FILE_UTILS_H__

/**
 * Write a buffer to file
 *
 * @param[in]	file_name	path to file.
 * @param[in]	buff		pointer to buffer to write.
 * @param[in]	size		size of buffer to write.
 */
int write_buf_to_file(char *file_name, char *buff, u32 size);

/**
 * Read a file to buffer
 *
 * @param[in]	file_name	path to file.
 * @param[in]	size		size of buffer to read.
 * @param[out]	buff		pointer to buffer to read the file
 */
int read_file_to_buf(char *file_name, char *buff, u32 size);

#endif /* __FILE_UTILS_H__ */
