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
