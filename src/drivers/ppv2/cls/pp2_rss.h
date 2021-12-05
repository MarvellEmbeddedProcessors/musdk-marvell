/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_rss.h
 *
 * RSS classifier .h file
 */

#ifndef _PP2_RSS_H_
#define _PP2_RSS_H_

int pp2_rss_enable(struct pp2_port *port, int en);
int pp2_rss_hw_tbl_set(struct pp2_port *port);
int pp22_cls_rss_rxq_set(struct pp2_port *port);
int pp2_rss_musdk_map_get(struct pp2_port *port);
int pp2_cls_rss_init(struct pp2_inst *inst);
void pp2_port_set_rss(struct pp2_port *port, uint32_t en);
int pp2_cls_rss_hw_dump(struct pp2_inst *inst);
int pp2_cls_rss_hw_rxq_tbl_dump(struct pp2_inst *inst);

#endif /* _PP2_RSS_H_ */
