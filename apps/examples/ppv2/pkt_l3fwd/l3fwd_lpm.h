/* Copyright (c) 2016, Linaro Limited
 * All rights reserved.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 */

#ifndef _L3FWD_LPM_H_
#define _L3FWD_LPM_H_

#ifdef __cplusplus
extern "C" {
#endif
void fib_tbl_init(void);
void fib_tbl_insert(u32 ip, int port, int depth);
int fib_tbl_lookup(u32 ip, int *port);
#ifdef __cplusplus
}
#endif

#endif
