/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __OF_SH_H__
#define __OF_SH_H__

#define OF_SCRIPT_STR	\
"#!/bin/sh\n\n"	\
"DT=/proc/device-tree\n\n"	\
"# $1 - node\n"	\
"# $2 - property\n"	\
"of_get_property()\n"	\
"{\n"	\
"    cat $DT$1/$2\n"	\
"}\n\n"	\
"# $1 - from\n"	\
"# $2 - compatible\n"	\
"# $3 - node\n"	\
"of_find_compatible_node()\n"	\
"{\n"	\
"    of_find_compatible_nodes $1 $2 | tail -n +$3 | head -1 | tr -d \\\\n\n"	\
"}\n\n"	\
"# $1 - from\n"	\
"# $2 - compatible\n"	\
"of_find_compatible_nodes()\n"	\
"{\n"	\
"    for c in $(fgrep -l $2 $(find $DT$1 -name compatible))\n"	\
"    do\n"	\
"        dirname $c\n"	\
"    done | cut -b 18-\n"	\
"}\n\n"	\
"# $1 - phandle\n"	\
"of_find_node_by_phandle()\n"	\
"{\n"	\
"    dirname $(fgrep -l $1 $(find $DT -name linux,phandle)) | cut -b 18- | tr -d \\\\n\n"	\
"}\n\n"	\
"exec <&-\n"	\
"exec 2>/dev/null\n"	\
"$@\n"

#endif /* __OF_SH_H__ */
