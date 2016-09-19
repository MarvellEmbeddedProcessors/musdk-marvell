/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "std.h"
#include "sys_dma.h"


int main (int argc, char *argv[])
{
    int err;

	printf("Marvell Armada US (Build: %s %s)\n", __DATE__, __TIME__);

	if ((err = mv_sys_dma_mem_init(16*1024*1024)) != 0)
		return err;

	printf("bye ...\n");
	return 0;
}
