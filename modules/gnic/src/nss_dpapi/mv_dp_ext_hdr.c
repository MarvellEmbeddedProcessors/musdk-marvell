/************************************************************************
*  Copyright (c) 2018 Marvell.
*
*  This program is free software: you can redistribute it and/or
*  modify it under the terms of the GNU General Public License as
*  published by the Free Software Foundation, either version 2 of the
*  License, or any later version.
*
*  This program is distributed in the hope that it will be useful, but
*  WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  General Public License for more details.
*
*
*******************************************************************************/

#include <linux/types.h>
#include "mv_dp_includes.h"
#include "mv_dp_fw_if.h"
#include "mv_dp_defs.h"
#include "mv_dp_types.h"
#include "mv_dp_main.h"


/*input: input is in network order address conversion*/
u32 *mv_dp_ext_hdr_to_host(u32 *hdr)
{
	int total_ptrs;
	int w;
	void *ptr;
	u32 *ptr_buf;

	if (!hdr) {
		MV_DP_LOG_ERR("NULL HDR PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return 0;
	}

#ifdef MV_DP_USE_DRAM_PTR
	be32_to_cpus(&hdr[MV_DP_MSG_EXT_HDR_PTR_WORD]);
	ptr_buf = MV_DP_PHYS_TO_VIRT(MV_DP_MSG_EXT_HDR_PTR_GET(hdr[MV_DP_MSG_EXT_HDR_PTR_WORD]));
#else
	ptr_buf = &hdr[MV_DP_MSG_EXT_HDR_PTR_WORD];
#endif

	MV_DP_LOG_DBG2("PTR_BUF ptr:%p\n", ptr_buf);

	if (!ptr_buf) {
		MV_DP_LOG_ERR("NULL BUF PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return 0;
	}

	/*convert the header*/
	be32_to_cpus(&hdr[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0]);
	be32_to_cpus(&hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	be32_to_cpus(&hdr[MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_WORD2]);
	be32_to_cpus(&hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	total_ptrs = MV_DP_MSG_EXT_HDR_IN_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	total_ptrs += MV_DP_MSG_EXT_HDR_OUT_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	for (w = 0; w < total_ptrs; w++) {
		ptr = MV_DP_PHYS_TO_VIRT(MV_DP_MSG_EXT_HDR_PTR_GET(be32_to_cpu(ptr_buf[w])));
		ptr_buf[w] = MV_DP_MSG_EXT_HDR_PTR_SET(ptr);
	}

	return hdr;
}

/*input: input is in host order and phy addresses*/
enum mv_dp_rc mv_dp_ext_hdr_be_release(u32 *hdr)
{
	int	total_ptrs;
	u32	*ptr_buf;
	int	w;
	u32	ptr;
	MV_DP_LOG_DBG3("ENTER %s\n", __func__);

#ifdef MV_DP_DEBUG
	if (!hdr) {
		MV_DP_LOG_ERR("NULL HDR PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

#ifdef MV_DP_USE_DRAM_PTR
	ptr_buf = MV_DP_PHYS_TO_VIRT(MV_DP_MSG_EXT_HDR_PTR_GET(be32_to_cpu(hdr[MV_DP_MSG_EXT_HDR_PTR_WORD])));
#else
	ptr_buf = &hdr[MV_DP_MSG_EXT_HDR_PTR_WORD];
#endif

	MV_DP_LOG_DBG2("PTR_BUF:ptr:%p\n", ptr_buf);

	if (!ptr_buf) {
		MV_DP_LOG_ERR("NULL BUF PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	be32_to_cpus(&hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	be32_to_cpus(&hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	total_ptrs = MV_DP_MSG_EXT_HDR_IN_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	total_ptrs += MV_DP_MSG_EXT_HDR_OUT_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	MV_DP_LOG_DBG2("EXT HDR IN buffers:%d, OUT buffers:%d\n",
		       MV_DP_MSG_EXT_HDR_IN_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]),
		       MV_DP_MSG_EXT_HDR_OUT_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]));

	for (w = 0; w < total_ptrs; w++) {
		ptr = MV_DP_MSG_EXT_HDR_PTR_GET(be32_to_cpu(ptr_buf[w]));
		if (!ptr)
			MV_DP_LOG_ERR("EXT HDR Release null ptr, index:%d\n", MV_DP_RC_ERR_NULL_PTR, w);

		MV_DP_LOG_DBG2("Releasing ptr_ind:%d, ptr:0x%08X\n", w, ptr);

		kfree(MV_DP_PHYS_TO_VIRT(ptr));
	}

#ifdef MV_DP_USE_DRAM_PTR
	MV_DP_LOG_DBG2("Released: ptr:%p\n", ptr_buf);
	kfree(ptr_buf);
#endif
	MV_DP_LOG_DBG2("EXT HDR Released buffers:%d\n", total_ptrs);

	MV_DP_LOG_DBG3("EXIT %s\n", __func__);

	return MV_DP_RC_OK;
}

/*addresses are virtual !*/
enum mv_dp_rc mv_dp_ext_hdr_host_release(u32 *hdr)
{
	int	total_ptrs;
	int	w;
	void	*ptr;
	u32	*ptr_buf;
	MV_DP_LOG_DBG3("ENTER %s\n", __func__);

#ifdef MV_DP_DEBUG
	if (!hdr) {
		MV_DP_LOG_ERR("NULL HDR PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif


#ifdef MV_DP_USE_DRAM_PTR
	ptr_buf = (u32 *)MV_DP_MSG_EXT_HDR_PTR_GET(hdr[MV_DP_MSG_EXT_HDR_PTR_WORD]);
#else
	ptr_buf = &hdr[MV_DP_MSG_EXT_HDR_PTR_WORD];
#endif

	MV_DP_LOG_DBG2("PTR_BUF ptr:%p\n", ptr_buf);
	if (!ptr_buf) {
		MV_DP_LOG_ERR("NULL BUF PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	total_ptrs = MV_DP_MSG_EXT_HDR_IN_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	total_ptrs += MV_DP_MSG_EXT_HDR_OUT_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	MV_DP_LOG_DBG2("EXT HDR IN buffers:%d, OUT buffers:%d\n",
		       MV_DP_MSG_EXT_HDR_IN_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]),
		       MV_DP_MSG_EXT_HDR_OUT_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]));


	for (w = 0; w < total_ptrs; w++) {
		ptr = (void *)MV_DP_MSG_EXT_HDR_PTR_GET(ptr_buf[w]);
		if (!ptr)
			MV_DP_LOG_ERR("EXT HDR Release null ptr, index:%d\n", MV_DP_RC_ERR_NULL_PTR, w);

		MV_DP_LOG_DBG2("Releasing ptr_ind:%d, ptr:%p\n", w, ptr);

		kfree(ptr);
	}
#ifdef MV_DP_USE_DRAM_PTR
	kfree(ptr_buf);
#endif


	MV_DP_LOG_DBG2("EXT HDR Released buffers:%d\n", total_ptrs);
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);

	return MV_DP_RC_OK;
}


/*input: input is in host order*/
/*bytes of contents to print*/
void mv_dp_ext_hdr_host_show(const u32 * const hdr, int print_count)
{

	int	total_ptrs;
	u32	*ptr_buf;
	int	w;
	int	ind;

	u8 *data;

	if (!hdr) {
		MV_DP_LOG_ERR("NULL HDR PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

#ifdef MV_DP_USE_DRAM_PTR
	ptr_buf = (u32 *)MV_DP_MSG_EXT_HDR_PTR_GET(hdr[MV_DP_MSG_EXT_HDR_PTR_WORD]);
#else
	ptr_buf = &hdr[MV_DP_MSG_EXT_HDR_PTR_WORD];
#endif

	if (!ptr_buf) {
		MV_DP_LOG_ERR("NULL BUF PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	total_ptrs = MV_DP_MSG_EXT_HDR_IN_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	total_ptrs += MV_DP_MSG_EXT_HDR_OUT_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	MV_DP_LOG_MSG("|MSG EXT HDR|<\n");
	MV_DP_LOG_CONT("|IN FLAGS|IN  TOTAL|IN  PTRS|IN  SIZE|OUT FLAGS|OUT TOTAL|OUT PTRS|OUT SIZE|PTR ADDR|<\n");

	MV_DP_LOG_CONT("|%08X|%9d|%8d|%8d| %08X|%9d|%8d|%8d|%p|<\n",
		       MV_DP_MSG_EXT_HDR_IN_FLAGS_GET(hdr[MV_DP_MSG_EXT_HDR_IN_FLAGS_WORD0]),
		       MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_GET(hdr[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0]),
		       MV_DP_MSG_EXT_HDR_IN_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]),
		       MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_GET(hdr[MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_WORD1]),
		       MV_DP_MSG_EXT_HDR_OUT_FLAGS_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_FLAGS_WORD2]),
		       MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_WORD2]),
		       MV_DP_MSG_EXT_HDR_OUT_NUM_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]),
		       MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_GET(hdr[MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_WORD3]),
		       ptr_buf
		       );

	if (print_count > MV_DP_EXT_BUF_CHUNK_SIZE)
		print_count = MV_DP_EXT_BUF_CHUNK_SIZE;

	for (w = 0; w < total_ptrs; w++) {
		data = (u8 *)MV_DP_MSG_EXT_HDR_PTR_GET(ptr_buf[w]);
		MV_DP_LOG_CONT("|PTR[%3d]: %p|", w, data);
		for (ind = 0; ind < print_count; ind++) {
			if (!(ind % MV_DP_LOG_BYTE_DUMP_LINE))
				MV_DP_LOG_CONT("<\n");
			MV_DP_LOG_CONT("%02X", data[ind]);
		}
		MV_DP_LOG_CONT("<\n");
	}
	MV_DP_LOG_CONT("|MSG EXT HDR END|<\n");
}

/*print count should be at most as the chunk size*/
void mv_dp_ext_hdr_be_show(const u32 * const hdr, int print_count, bool inv)
{

	int total_ptrs;
	int w;
	int ind;
	u32 data_phy;
	u8 *data_virt;
	u32 *ptr_buf;
	u32 tmp[MV_DP_MSG_EXT_HDR_SIZE_W];

	if (!hdr) {
		MV_DP_LOG_ERR("NULL HDR PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

#ifdef MV_DP_USE_DRAM_PTR
	ptr_buf = MV_DP_PHYS_TO_VIRT(MV_DP_MSG_EXT_HDR_PTR_GET(be32_to_cpu(hdr[MV_DP_MSG_EXT_HDR_PTR_WORD])));
#else
	ptr_buf = &hdr[MV_DP_MSG_EXT_HDR_PTR_WORD];
#endif


	MV_DP_LOG_DBG2("PTR_BUF ptr:%p\n", ptr_buf);

	if (!ptr_buf) {
		MV_DP_LOG_ERR("NULL BUF PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	for (w = 0; w < MV_DP_MSG_EXT_HDR_SIZE_W; w++)
		tmp[w] = be32_to_cpu(hdr[w]);

	total_ptrs = MV_DP_MSG_EXT_HDR_IN_NUM_GET(tmp[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	total_ptrs += MV_DP_MSG_EXT_HDR_OUT_NUM_GET(tmp[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	MV_DP_LOG_MSG("|MSG EXT HDR|<\n");
	MV_DP_LOG_CONT("|IN FLAGS|IN  TOTAL|IN  PTRS|IN  SIZE|OUT FLAGS|OUT TOTAL|OUT PTRS|OUT SIZE|PTR ADDR|<\n");

	MV_DP_LOG_CONT("|%08X|%9d|%8d|%8d| %08X|%9d|%8d|%8d|%p|<\n",

		       MV_DP_MSG_EXT_HDR_IN_FLAGS_GET(tmp[MV_DP_MSG_EXT_HDR_IN_FLAGS_WORD0]),
		       MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_GET(tmp[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0]),
		       MV_DP_MSG_EXT_HDR_IN_NUM_GET(tmp[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]),
		       MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_GET(tmp[MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_WORD1]),
		       MV_DP_MSG_EXT_HDR_OUT_FLAGS_GET(tmp[MV_DP_MSG_EXT_HDR_OUT_FLAGS_WORD2]),
		       MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_GET(tmp[MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_WORD2]),
		       MV_DP_MSG_EXT_HDR_OUT_NUM_GET(tmp[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]),
		       MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_GET(tmp[MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_WORD3]),
		       ptr_buf);

	if (print_count > MV_DP_EXT_BUF_CHUNK_SIZE)
		print_count = MV_DP_EXT_BUF_CHUNK_SIZE;

	for (w = 0; w < total_ptrs; w++) {
		data_phy = MV_DP_MSG_EXT_HDR_PTR_GET(be32_to_cpu(ptr_buf[w]));
		data_virt = MV_DP_PHYS_TO_VIRT(data_phy);
		MV_DP_LOG_CONT("|PTR[%3d]:V_ADDR:%p|P_ADDR:0x%08X|", w, data_virt, data_phy);
		for (ind = 0; ind < print_count; ind++) {
			if (!(ind % MV_DP_LOG_BYTE_DUMP_LINE))
				MV_DP_LOG_CONT("<\n");
			MV_DP_LOG_CONT("%02X", data_virt[ind]);
		}
		/*invalidate only as already flushed*/
		if (inv)
			mv_dma_single_cpu_to_dev(data_virt, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_FROM_DEVICE);
		MV_DP_LOG_CONT("<\n");
	}
	MV_DP_LOG_CONT("|MSG EXT HDR END|<\n");
}

