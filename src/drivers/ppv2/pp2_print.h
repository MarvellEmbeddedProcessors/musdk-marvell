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

#ifndef _PP2_PRINT_H_
#define _PP2_PRINT_H_

#define PP2_ERR  (1)
#define PP2_WARN (2)
#define PP2_INFO (3)
#define PP2_DBG  (4)

#define PP2_INFO_LEVEL (PP2_INFO)

#define pp2_print(_level, ...) \
do { \
	if ((_level) <= (PP2_INFO_LEVEL)) { \
		printf(__VA_ARGS__); \
	} \
} while (0)

#define dbg(_func, ...) \
do { \
	if ((PP2_DBG) <= (PP2_INFO_LEVEL)) { \
		_func(__VA_ARGS__); \
	} \
} while (0)

#define pp2_err(...) \
	pp2_print(PP2_ERR, "[ERR] " __VA_ARGS__)

#define pp2_warn(...) \
	pp2_print(PP2_WARN, "[WARN] " __VA_ARGS__)

#define pp2_info(...) \
	pp2_print(PP2_INFO, "[INFO] " __VA_ARGS__)

#define pp2_dbg(...) \
	pp2_print(PP2_DBG, "[DBG] " __VA_ARGS__)

#define pp2_info_fmt(fmt, ...) \
	pp2_info("%24s : %3d : " fmt "\n", \
		__func__, __LINE__, ##__VA_ARGS__) \

#define pp2_dbg_fmt(fmt, ...) \
	pp2_dbg("%24s : %3d : " fmt "\n", \
	       __func__, __LINE__, ##__VA_ARGS__) \

#endif /* _PP2_PRINT_H_ */
