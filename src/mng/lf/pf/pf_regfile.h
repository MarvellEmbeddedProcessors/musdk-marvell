/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _REGFILE_H
#define _REGFILE_H

#include "std_internal.h"

/* Initialize register file utility
 *
 *  returns 0 on success or <0 on failure
 */
int regfile_init(void);


/* Close register file utility
 *
 */
void regfile_destroy(void);

/* Open register file
 *
 *  filename	File name
 *  size	File size
 *
 *  returns file mapping address on success or NULL on failure
 */
void *regfile_open(char *file_name, int file_size);


/* Close register file
 *
 *  addr	file mapping address
 *
 *  returns 0 on success or <0 on failure
 */
int regfile_close(void *addr);

#endif /* _REGFILE_H */
