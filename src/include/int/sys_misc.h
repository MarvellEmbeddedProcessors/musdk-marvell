/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __SYSMISC_H__
#define __SYSMISC_H__


#include <stdio.h>
#include <string.h>

/* TODO: Replace Q&D with for_loop, 20 is hardcoded in sscanf */
#define MAX_OBJ_STRING 20
static inline int mv_sys_match(const char *match, const char* obj_type, u8 hierarchy_level, u8 id[])
{
	char obj_string[MAX_OBJ_STRING];

	if (hierarchy_level > 3) {
		pr_err("Maximum %u levels of hierarchy supported\n", hierarchy_level);
		return(-1);
	}
	if ((hierarchy_level + 1) != sscanf(match, "%20[^-]%*[:]%hhu%*[:]%hhu%*[:]%hhu", obj_string, &id[0], &id[1], &id[2])) {
		pr_err("Error scanning %d levels \n", hierarchy_level);
		return(-1);
	}
	if (strcmp(obj_type, obj_string)) {
		pr_err("String {%s} does not match obj_type {%s}\n", obj_string, obj_type);
		return(-1);
	}
	return 0;
}

#endif /* __SYSMISC_H__ */


