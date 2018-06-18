/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _FILE_MAP_H_
#define _FILE_MAP_H_


struct file_map_info {
	void		*map_addr;
	int		map_size;
};

/**
 * Map a file to memory memory allocated for MM object.
 *
 * @param[in]	file_name	path to file.
 * @param[out]	map_info	structure which contains map information.
 */
int file_map(char *file_name, struct file_map_info *map_info);

/**
 * Un-Map the file memory.
 *
 * @param[in]	map_info	structure which contains map information.
 */
int file_unmap(struct file_map_info *map_info);

#endif /* _FILE_MAP_H__ */
