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

#ifndef __MV_ERRNO_H__
#define __MV_ERRNO_H__

#include <errno.h>

#ifndef EPERM
#define EPERM		1
#endif /* EPERM */
#ifndef ENOENT
#define ENOENT		2
#endif /* ENOENT */
#ifndef EIO
#define EIO			5
#endif /* EIO */
#ifndef ENXIO
#define ENXIO		6
#endif /* ENXIO */
#ifndef ENOMEM
#define ENOMEM		12
#endif /* ENOMEM */
#ifndef EACCES
#define EACCES		13
#endif /* EACCES */
#ifndef EFAULT
#define EFAULT		14
#endif /* EFAULT */
#ifndef EBUSY
#define EBUSY		16
#endif /* EBUSY */
#ifndef EEXIST
#define EEXIST		17
#endif /* EEXIST */
#ifndef ENODEV
#define ENODEV		19
#endif /* ENODEV */
#ifndef EINVAL
#define EINVAL		22
#endif /* EINVAL */
#ifndef ENOMSG
#define ENOMSG		42
#endif /* ENOMSG */

#ifndef ENOBUFS
#define ENOBUFS		105
#endif /* ENOBUFS */

#ifndef ENOTSUP
#define ENOTSUP		252
#endif /* ENOTSUP */

#endif /* __MV_ERRNO_H__ */
