/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "sys_dma.h"

int mv_sys_dma_mem_init(u64 size)
{
	pr_err("[%s] routine not supported yet!\n",__FUNCTION__);
	return -ENOTSUPP;
}

void *mv_sys_dma_mem_alloc(size_t align, size_t size)
{
	pr_err("[%s] routine not supported yet!\n",__FUNCTION__);
	return NULL;
}

void mv_sys_dma_mem_free(void *ptr, size_t size)
{
	pr_err("[%s] routine not supported yet!\n",__FUNCTION__);
}
