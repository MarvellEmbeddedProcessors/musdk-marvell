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

#include <string.h>
#include <time.h>
#include "mv_std.h"
#include "lib/lib_misc.h"
#include <fcntl.h>
#include <sys/mman.h>
#include "drivers/mv_dmax2.h"
#include "perf_mon_emu.h"
#include "std_internal.h"

/* DMA test variables */
#define MB_SIZE		(u64)(1024*1024)
#ifdef MVCONF_SYS_DMA_HUGE_PAGE
#define DMA_MEM_SIZE	(120*MB_SIZE)
#else
#define DMA_MEM_SIZE	(50*MB_SIZE) /* CMA allocations */
#endif

#define MMAP_FILE_NAME	"/dev/mem"
#define DMAX2_INTERFACE_COUNT	8
#define DMAX2_MAX_DESC_SIZE	(2*MB_SIZE)
#define DMAX2_DFLT_DESC_SIZE	(0x2000) /* 8Kb */

/** Get rid of path in filename - only for unix-type paths using '/' */
#define NO_PATH(file_name) (strrchr((file_name), '/') ? \
			    strrchr((file_name), '/') + 1 : (file_name))

struct mem {
	phys_addr_t	pa;
	void		*va;
};

/* Global struct - Manual Test parameters */
struct glob_arg {
	int		data_integrity_verification;
	int		perf_measure;
	int		cycle_measure;
	int		manual_test;
	u16		engine;
	u16		desc_num;
	u32		desc_size;
	u32		align;
	enum		dmax2_mem_direction mem_attr;
	u64		overall_size;
	phys_addr_t	io_base_pa;
	u16		repeat;
};

static struct glob_arg garg = {};

static struct	dmax2 *dmax2;
static		u64 sys_dma_high_addr;
off_t		page_offset;
int		dev_mem_fd;
u32		*io_base_va;
struct mem	*src_mems = NULL, *dst_mems = NULL;
u64 actuall_overall_size;

/* performance & cycle measurement */
#define MAX_COUNT_FOR_CYCLE_CALC 1000000
static inline void start_overall_perf(void);
static inline void stop_overall_perf(void);
struct timeval t1, t2;
int usecs = 0, test_num = 0;
int pme_ev_cnt_desc_enq = -1, pme_ev_cnt_desc_deq = -1, pme_ev_cnt_hw_process = -1;

static inline void start_overall_perf(void)
{        /* start timer */
	if (unlikely(garg.perf_measure))
		gettimeofday(&t1, NULL);
}

static inline void stop_overall_perf(void)
{
	if (unlikely(garg.perf_measure)) {
		/* stop timer */
		gettimeofday(&t2, NULL);
		usecs += (t2.tv_sec - t1.tv_sec) * 1000000.0;
		usecs += (t2.tv_usec - t1.tv_usec);
	}
}

static inline void start_cycle_count(int pme_ev_cnt_id)
{
	if (unlikely(garg.cycle_measure))
		pme_ev_cnt_start(pme_ev_cnt_id);
}

static inline void stop_cycle_count(int pme_ev_cnt_id, u16 unit_count)
{
	if (unlikely(garg.cycle_measure))
		pme_ev_cnt_stop(pme_ev_cnt_id, unit_count);
}

/* Virtual mapping for physical IO memory */
static int verify_io_mem(phys_addr_t io_base_pa, int desc_size, enum dmax2_mem_direction mem_attr)
{
	off_t	page_base = 0;
	size_t	pagesize;
	int	i, temp_val;

	pagesize = sysconf(_SC_PAGE_SIZE);
	if (!pagesize) {
		pr_err("Failed to read page size\n");
		return -EFAULT;
	}

	page_base = (io_base_pa / pagesize) * pagesize;
	page_offset = io_base_pa - page_base;
	dev_mem_fd = open(MMAP_FILE_NAME, O_RDWR);
	io_base_va = mmap(NULL, page_offset + desc_size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_mem_fd, page_base);
	if (io_base_va == MAP_FAILED) {
		pr_err("Failed to map IO memory (address = 0x%lx)\n", io_base_pa);
		return -EFAULT;
	}
	/* add destination offset to page base */
	io_base_va += page_offset/sizeof(u32);

	/* verify valid read/write access to mapped IO memory */
	u32 *src_buff = (u32 *)io_base_va;

	for (i = 0; i  < desc_size / 4; i++) {
		temp_val = rand();
		src_buff[i] = (u32)temp_val;
		rmb(); /* make sure we read actual register value */
		if (src_buff[i] != temp_val) {
			pr_err("Failed writing/reading from requested IO address (0x%lx)\n", io_base_pa + i * 4);
			return -EFAULT;
		}
	}

	/* print destination IO prior to DMA copy */
	for (i = 0; i < desc_size / sizeof(u32); i++)
		pr_debug("IO mem prior to DMA: 0x%lx = 0x%x\n", io_base_pa + i * sizeof(u32), io_base_va[i]);

	return 0;
}

/* prepare & map src & dst mem DMA buffers, and map IO memory if needed */
static int prepare_mem_buffers(int desc_num, int desc_size, int align,
				phys_addr_t io_base_pa, enum dmax2_mem_direction mem_attr)
{
	int	i;

	/* Local data for source & destination */
	src_mems = (struct mem *)malloc(desc_num * sizeof(struct mem));
	if (!src_mems) {
		pr_err("No mem for dmax2-test source mem obj!\n");
		return -ENOMEM;
	}
	dst_mems = (struct mem *)malloc(desc_num * sizeof(struct mem));
	if (!dst_mems) {
		pr_err("No mem for dmax2-test destination mem obj!\n");
		return -ENOMEM;
	}

	/* Prepare DMA memory for source & destination */
	for (i = 0; i < desc_num; i++) {
		/* Source buffer */
		src_mems[i].va = mv_sys_dma_mem_alloc(desc_size, align);
		if (!src_mems[i].va)
			goto allocation_error;

		src_mems[i].pa = mv_sys_dma_mem_virt2phys(src_mems[i].va);
		pr_debug("SRC: va=0x%p, pa=0x%llx, ", src_mems[i].va, (unsigned long long int)src_mems[i].pa);

		/* Destination buffer */
		if (mem_attr == DMAX2_TRANS_MEM_ATTR_IO) {
			/* IO destination is fixed and not allocated per descriptor */
			dst_mems[i].va = io_base_va;
			dst_mems[i].pa = io_base_pa;
		} else {
			/* allocate DMA memory destination - per descriptor */
			dst_mems[i].va = mv_sys_dma_mem_alloc(desc_size, align);
			if (!dst_mems[i].va)
				goto allocation_error;

			dst_mems[i].pa = mv_sys_dma_mem_virt2phys(dst_mems[i].va);
		}

		pr_debug("DST: va=0x%p, pa=0x%llx\n", dst_mems[i].va, (unsigned long long int)dst_mems[i].pa);
	}
	return 0;

allocation_error:
	pr_err("failed to allocate source & destination memory (allocated 0x%x)!\n", 2 * i * desc_size);
	printf("\tTry using smaller descriptors, or less descriptors\n\n");
	return -EFAULT;
}

/* fill random data in source buffer, and prepare DMA descriptors with buffer information */
static void prepare_descriptors(struct mem *src_mems, struct mem *dst_mems, int desc_num,
		int desc_size, enum dmax2_mem_direction mem_attr, struct dmax2_desc descs[])
{
	u32 i, rand_io_value[desc_size / 4];

	/* Save high address part of VA */
	sys_dma_high_addr = ((u64)src_mems[0].va) & (~((1ULL << 32) - 1));

	/* Initialize random data */
	srand(time(NULL));
	for (i = 0; i < desc_num; i++) {
		/* Prepare descriptors */
		descs[i].desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
		descs[i].buff_size = desc_size;
		descs[i].src_addr = src_mems[i].pa;
		descs[i].dst_addr = dst_mems[i].pa;
		descs[i].desc_id = i;	/* desc_id/cookie = descriptor number, for easy comparison after Dequeue */

		/* if requested data integrity verification, fill buffer with random data */
		if (likely(!garg.data_integrity_verification))
			continue;

		u32 j, *src_buff = (u32 *)src_mems[i].va;

		for (j = 0; j  < desc_size / 4; j++) {
			if (mem_attr == DMAX2_TRANS_MEM_ATTR_IO) {
				/* for IO test, use same random values for all descriptors
				 * since eventually we compare data in single IO destination
				 */
				if (i == 0) {
					src_buff[j] = rand();
					rand_io_value[j] = src_buff[j]; /* save values of 1st descriptor */
					pr_debug("Random value for IO reg#%d = 0x%x\n", j, rand_io_value[j]);
				} else
					src_buff[j] = rand_io_value[j];  /* use saved values for all descriptors */
			} else
				src_buff[j] = rand();
		}
		if ((upper_32_bits((u64)src_mems[i].va)) != (sys_dma_high_addr >> 32)) {
			pr_err("va(%p) upper out of range; skipping this buff\n", src_mems[i].va);
			continue;
		}
	}
}

/* verify destination target after DMA operation completed
 * to compare destination to source buffers - this will slow entire single_test operation
 */
static inline int verify_data_integrity(struct dmax2_trans_complete_desc *res_descs, int desc_num, int desc_size,
				enum dmax2_mem_direction mem_attr)
{
	int i;

	/* if not requested, skip data integrity verification stage */
	if (likely(!garg.data_integrity_verification))
		return 0;

	/* Data integrity verification */
	for (i = 0; i < desc_num; i++) {
		 /* compare vs original descriptor number (from cookie) */
		u32 j, *dst_buff = (u32 *)dst_mems[res_descs[i].desc_id].va;
		u32 *src_buff = (u32 *)src_mems[res_descs[i].desc_id].va;

		pr_debug("Comparing desc %d with sourc desc (cookie) %d, src_buff = 0x%lx\n"
			, i, res_descs[i].desc_id, (u64)src_buff);
		/* Compare destination buffer to source buffer */
		for (j = 0; j  < desc_size / 4 ; j++) {
			if (dst_buff[j] != src_buff[j]) {
				pr_err("Desc %d mismatch (offset = 0x%x): ", res_descs[i].desc_id, j * 4);
				printf("SRC = 0x%x,  DST = 0x%x\n", src_buff[j], dst_buff[j]);
				printf("Comparing desc %d with sourc desc (desc_id) %d, src_buff = 0x%lx\n"
					, i, res_descs[i].desc_id, (u64)src_buff);
				printf("Source buffer around mismatch point:\n");
				mem_disp((void *)(src_buff + j), 8);
				printf("Destination buffer around mismatch point:\n");
				mem_disp((void *)(dst_buff + j), 8);
				return -EFAULT;
			} else if (mem_attr != DMAX2_TRANS_MEM_ATTR_IO) /* clean destination buffer */
				dst_buff[j] = 0;
		}
		/* cookie debug:
		 * char *buff = (char *)(uintptr_t)res_descs[i].desc_id;
		 * buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
		 * pr_debug("destination buff %d: desc_id = %x\n" ,i , res_descs[i].desc_id);
		 */
	}
	return 0;
}

static int dma_test(struct mem *src_mems, struct mem *dst_mems, int desc_num,
		int desc_size, enum dmax2_mem_direction mem_attr)
{
	u16	actual_desc_num,  __maybe_unused loop_count = 0;
	u64	completed_size = 0;
	int	err;
	struct	dmax2_desc			descs[desc_num];
	struct	dmax2_trans_complete_desc	res_descs[desc_num];

	actual_desc_num = desc_num;

	/* Prepare source/destination memory pools, transfer descriptors, and fill source with random data */
	memset(&descs, 0, sizeof(descs));
	prepare_descriptors(src_mems, dst_mems, desc_num, desc_size, mem_attr, descs);

	start_overall_perf();
	/* start ENQ -> DEQ -> verify until finished overall requested size */
	do {
		/* Enqueue descriptors to DMA engine */
		start_cycle_count(pme_ev_cnt_desc_enq);
		err = dmax2_enq(dmax2, &descs[0], &actual_desc_num);
		if (err < 0)
			return err;
		stop_cycle_count(pme_ev_cnt_desc_enq, actual_desc_num);

		pr_debug("ENQ %d descs, overall enq_size_completed = %ld\n"
				, actual_desc_num, completed_size + (actual_desc_num * desc_size));

		/* Wait and let HW finish process required desc_num */
		start_cycle_count(pme_ev_cnt_hw_process);
		while (dmax2_get_deq_num_available(dmax2) < actual_desc_num)
			;
		stop_cycle_count(pme_ev_cnt_hw_process, desc_num);

		/* Dequeue descriptors till operation completed  */
		start_cycle_count(pme_ev_cnt_desc_deq);
		err = dmax2_deq(dmax2, &res_descs[0], &actual_desc_num, garg.data_integrity_verification);
		stop_cycle_count(pme_ev_cnt_desc_deq, actual_desc_num);
		if (unlikely(err || actual_desc_num != desc_num)) {
			pr_err("DMA DeQ failed (err = %d, dequeued = %d, requested = %d)!\n"
				, err, actual_desc_num, desc_num);
			return -EFAULT;
		}

		completed_size += desc_num * desc_size;
		pr_debug("DEQ %d descs, overall DEQ size = %ld\n", desc_num, completed_size);

		/* if not requested, skip data integrity verification stage inside */
		if (unlikely(verify_data_integrity(res_descs, desc_num, desc_size, mem_attr) != 0))
			return -EFAULT;

		pr_debug("<-- #%d: Completed = %ld , overall_size = %ldb\n"
			, loop_count++, completed_size, garg.overall_size);
	} while (completed_size < garg.overall_size);

	stop_overall_perf();
	actuall_overall_size = completed_size;

	return 0;
}

static int single_test(int desc_num, int desc_size, int align,
		phys_addr_t io_base_pa, enum dmax2_mem_direction mem_attr)
{
	int	i, err = 0;

	actuall_overall_size = 0; /* clear overall size from previous runs */

	/* initialize performance cycle counters */
	if (garg.cycle_measure) {
		garg.perf_measure = 1;
		pme_ev_cnt_desc_enq = pme_ev_cnt_create("SW Desc Enq\t", MAX_COUNT_FOR_CYCLE_CALC, 0);
		if (pme_ev_cnt_desc_enq  < 0)
			pr_err("PME for DMA Descriptor SW Enqueue failed!\n");

		pme_ev_cnt_desc_deq = pme_ev_cnt_create("SW Desc Deq\t", MAX_COUNT_FOR_CYCLE_CALC, 0);
		if (pme_ev_cnt_desc_deq < 0)
			pr_err("PME for DMA Descriptor SW Dequeue failed!\n");

		pme_ev_cnt_hw_process = pme_ev_cnt_create("HW desc process  ", MAX_COUNT_FOR_CYCLE_CALC, 0);
		if (pme_ev_cnt_hw_process  < 0)
			pr_err("PME for DMA HW process failed!\n");
	}

	/* Configure memory access attributes, according to test destination target type */
	if (mem_attr == DMAX2_TRANS_MEM_ATTR_IO) {
		/* DMA IO test uses DMA memory as source and IO memory as destination */
		dmax2_set_mem_attributes(dmax2, DMAX2_TRANS_LOCATION_DST, mem_attr);

		/* map IO destination and verify it's accessible */
		err = verify_io_mem(io_base_pa, desc_size, mem_attr);
		if (err)
			return err;
	} else	/* DMA MEM test use DMA memory for source and destination */
		dmax2_set_mem_attributes(dmax2, DMAX2_TRANS_LOCATION_SRC_AND_DST, mem_attr);

	/* prepare & map src & dst mem DMA buffers, and map IO memory if needed */
	err = prepare_mem_buffers(desc_num, desc_size, align, io_base_pa, mem_attr);
	if (err)
		return err;

	/* run DMA test */
	err |= dma_test(src_mems, dst_mems, desc_num, desc_size, mem_attr);

	/* Release DMA memory & local struct */
	for (i = 0; i < desc_num; i++) {
		if (src_mems[i].va)
			mv_sys_dma_mem_free(src_mems[i].va);
		if (mem_attr != DMAX2_TRANS_MEM_ATTR_IO && dst_mems[i].va)
			mv_sys_dma_mem_free(dst_mems[i].va);
	}

	/* unmap IO destination */
	if (mem_attr == DMAX2_TRANS_MEM_ATTR_IO) {
		/* print destination IO After to DMA copy */
		for (i = 0; i < desc_size / sizeof(u32); i++)
			pr_debug("IO mem After DMA: 0x%lx = 0x%x\n", io_base_pa + i * sizeof(u32), io_base_va[i]);
		/* unmap virtual mapping of IO address */
		munmap(io_base_va, page_offset + desc_size);
		if (dev_mem_fd)
			close(dev_mem_fd);
	}

	if (!garg.perf_measure)
		return err;

	printf("\n<<< Test %d Summary >>>\n", test_num);

	printf("Target: %3s, desc num =%4d, desc_size(bytes) =%-7d"
		, (mem_attr == DMAX2_TRANS_MEM_ATTR_IO) ? "IO" : "MEM", desc_num, desc_size);
	printf(", total=%ld%s\n", actuall_overall_size > MB_SIZE ? actuall_overall_size/MB_SIZE : actuall_overall_size
				, actuall_overall_size > MB_SIZE ? "MB" : "b");

	if (garg.cycle_measure) {
		/* dump counters */
		pme_ev_cnt_dump(pme_ev_cnt_desc_enq, 1);
		pme_ev_cnt_dump(pme_ev_cnt_hw_process, 1);
		pme_ev_cnt_dump(pme_ev_cnt_desc_deq, 1);

		/* destroy counters */
		pme_ev_cnt_destroy(pme_ev_cnt_desc_enq);
		pme_ev_cnt_destroy(pme_ev_cnt_hw_process);
		pme_ev_cnt_destroy(pme_ev_cnt_desc_deq);
	}

	printf("Copied %.2fMB with %ld descriptors: "
		, ((double)actuall_overall_size)/MB_SIZE, actuall_overall_size/desc_size);
	printf("%.2f seconds\n", usecs/1000000.0);
	printf("Test %d Average perf:%.2f MB/s\n", test_num++, (((actuall_overall_size/MB_SIZE)*1000000.0)/usecs));
	usecs = 0;

	free(src_mems);
	free(dst_mems);

	return err;
}

static void usage(char *progname)
{
	printf("Usage: %s <OPTIONS> <TEST_PARAMS>\n"
		"  Run with no PARAMS will run various IO/MEM DMA tests:\n"

		"\nOptional OPTIONS:\n"
		"\t-i <DMA-engine-#> Interface number: min 0, max %i (default 0)\n"
		"\t-t, <total_size>  Total copy size - in MB (default 100MB).\n"
					"\t\t\t  (set 0 for a single burst)\n"
		"\t-r, <repeat_count> How many times to repeat test.\n"
		"\t--cycle           Show Cycle measurements (disabled by default)\n"
		"\t--verify          Data integrity verification - slow down DMA process\n\t\t\t\t(disabled by default)\n"
		"\nUser defined test TEST_PARAMS:\n"
		"\t-d, <destination>  Destination: mem / io (default MEM)\n"
		"\t-a, <IO address>  IO address register (default IO addr = 0x%x)\n"
		"\t-c, <desc_count>  Descriptor count (default %d, max %d).\n"
		"\t-s, <desc_size>   Descriptor size - bytes (default %db, max %ldb).\n"
		"\t?, -h, --help     Display help and exit.\n\n"
		"Examples:\n"
		"  E.g. 1: use various tests with DMA engine #1, 100MB, with cycle & data integrity:\n"
		"  \t%s -i 1 -t 100 --cycle --verify\n"
		"  E.g. 2: single DMA test to 'mem', 5000MB, 2048 descriptors, desc size = 32768:\n"
		"  \t%s -t 5000 -d mem -c 2048 -s 32768\n"
		"  E.g. 3: single DMA test to IO reg 0xf20064c0, 100MB with 2000 descriptors,\n"
		"          desc size = 4, with data integrity verification\n"
		"  \t%s -t 100 -c 2000 -s 4 -d io -a 0xf20064c0 --verify\n"
		"\n", NO_PATH(progname), DMAX2_INTERFACE_COUNT-1, 0xF2281008, DMAX2_BURST_SIZE,
			DMAX2_BURST_SIZE, DMAX2_DFLT_DESC_SIZE, DMAX2_MAX_DESC_SIZE,
			NO_PATH(progname), NO_PATH(progname), NO_PATH(progname)
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;
	/* default parameters */
	garg->data_integrity_verification = 0;
	garg->perf_measure = 1;
	garg->cycle_measure = 0;
	garg->manual_test = 0;

	garg->engine = 0;
	garg->desc_num = DMAX2_BURST_SIZE;
	garg->desc_size = DMAX2_DFLT_DESC_SIZE;
	garg->align = DMAX2_DFLT_DESC_SIZE;
	garg->mem_attr = DMAX2_TRANS_MEM_ATTR_CACHABLE;
	garg->overall_size = 100 * MB_SIZE;
	garg->io_base_pa = 0;
	garg->repeat = 1;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			return -EINVAL;
		} else if (strcmp(argv[i], "-i") == 0) { /* Engine unit # */
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			garg->engine = atoi(argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-d") == 0) { /* Destination (IO/MEM) # */
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (strcmp(argv[i+1], "mem") == 0) {
				garg->io_base_pa = -1;
				garg->mem_attr = DMAX2_TRANS_MEM_ATTR_CACHABLE;
			} else if (strcmp(argv[i+1], "io") == 0) {
				garg->mem_attr = DMAX2_TRANS_MEM_ATTR_IO;
				/* if user didn't request specific IO address, use default */
				if (!garg->io_base_pa)
					garg->io_base_pa = 0xF2281008;
			} else {
				pr_err("Invalid destination arguments!\n");
				return -EINVAL;
			}
			garg->manual_test = 1;
			i += 2;
		} else if (strcmp(argv[i], "-c") == 0) { /* descriptor count */
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			garg->desc_num = atoi(argv[i+1]);
			i += 2;
			garg->manual_test = 1;
		} else if (strcmp(argv[i], "-s") == 0) { /* descriptor size */
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			garg->desc_size = atoi(argv[i+1]);
			i += 2;
			garg->manual_test = 1;
		} else if (strcmp(argv[i], "-t") == 0) { /* Total copy size */
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			garg->overall_size = MB_SIZE * atoi(argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-r") == 0) { /* repeat count */
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			garg->repeat = atoi(argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-a") == 0) { /* IO address */
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			garg->io_base_pa = strtoul(argv[i+1], &argv[i+1], 0);
			i += 2;
		} else if (strcmp(argv[i], "--cycle") == 0) {
			garg->cycle_measure = 1;
			i += 1;
		} else if (strcmp(argv[i], "--verify") == 0) {
			garg->data_integrity_verification = 1;
			i += 1;

		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (garg->engine >= DMAX2_INTERFACE_COUNT) {
		pr_err("Invalid Engine!(supported 0..%d)\n", DMAX2_INTERFACE_COUNT - 1);
		return -EINVAL;
	}

	if (garg->desc_num > DMAX2_BURST_SIZE) {
		pr_err("Invalid Descriptor count! (supported max %d descriptors)\n", DMAX2_BURST_SIZE);
		return -EINVAL;
	}

	if (!garg->desc_size || garg->desc_size > DMAX2_MAX_DESC_SIZE) {
		pr_err("Invalid Descriptor's size: %d (max size supported= %ld)\n\n"
			, garg->desc_size, DMAX2_MAX_DESC_SIZE);
		return -EINVAL;
	}

	if (!garg->repeat) {
		pr_err("Invalid repeat count\n");
		return -EINVAL;
	}

	if (garg->mem_attr == DMAX2_TRANS_MEM_ATTR_IO && garg->desc_size % 4 != 0) {
		garg->desc_size -= garg->desc_size % 4;
		pr_err("Descriptor's size aligned to %d (32bit aligned for IO destination)\n\n", garg->desc_size);
	}

	printf("Verify Data Integrity\t= %s\nCycle Measurement\t= %s\nManual user test\t= %s\n"
		, garg->data_integrity_verification ? "Yes" : "No"
		, garg->cycle_measure ? "Yes" : "No", garg->manual_test ? "Yes" : "No");

	printf("Repeat count \t\t= %d\n", garg->repeat);

	if (garg->manual_test) {
		printf("DMA engine #%d\ndesc count \t\t= %d\ndesc size \t\t= %d\noverall size \t\t= %ldMB"
			, garg->engine, garg->desc_num, garg->desc_size, garg->overall_size / MB_SIZE);
		if (!garg->overall_size)
			printf(" (Single Burst)");
		if (garg->mem_attr == DMAX2_TRANS_MEM_ATTR_CACHABLE)
			printf("\nDestination MEM (MEM_ATTR_CACHABLE)\n");
		else if (garg->mem_attr == DMAX2_TRANS_MEM_ATTR_IO)
			printf("\nDestination IO (MEM_ATTR_IO) = 0x%lx\n", garg->io_base_pa);
	} else
		printf("\nRunning series of DMA test...\n");


	return 0;
}

/* DMA test supports predefined series of test, or user controlled parameters
 * see usage routine above for instructions
 */
int main(int argc, char *argv[])
{
	struct dmax2_params	 dmax2_params;
	int	err = 0;
	char engine_name[7];

	printf("Marvell Armada US DMA test (Build: %s %s)\n\n", __DATE__, __TIME__);

	/* parse arguments from user */
	err = parse_args(&garg, argc, argv);
	if (err) {
		usage(argv[0]);
		return err;
	}

	/* Initialize required DMA memory for DMA test and local vars */
	err = mv_sys_dma_mem_init(DMA_MEM_SIZE);
	if (err)
		return err;

	/* Initialize DMA engine */
	sprintf(engine_name, "dmax2-%d", garg.engine);
	dmax2_params.match = engine_name;
	dmax2_params.queue_size = DMAX2_BURST_SIZE;
	err = dmax2_init(&dmax2_params, &dmax2);
	if (err)
		goto free_mem;

	/* Start DMA test/s */
	for (int i = 0; i < garg.repeat && !err; i++) {

		/* If user selected manual test, run only this test as user requested */
		if (garg.manual_test) {
			err |= single_test(garg.desc_num, garg.desc_size, garg.align,
						garg.io_base_pa, garg.mem_attr);
			continue;
		}

		/* Else, run series of predefined tests */
		err |= single_test(300, 0x10000, 64, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(1000, 0x8, 0x4, 0xF2281008, DMAX2_TRANS_MEM_ATTR_IO);
		if (!err)
			err |= single_test(100, 0x8, 0x4, 0xF2281008, DMAX2_TRANS_MEM_ATTR_IO);
		if (!err)
			err |= single_test(1000, 0x8, 64, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(10, 0.5 * MB_SIZE, 4, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
#ifdef MVCONF_SYS_DMA_HUGE_PAGE
		if (!err)
			err |= single_test(29, 2*MB_SIZE, 2*MB_SIZE, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(4, 1500000, 2*MB_SIZE, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
#endif
		if (!err)
			err |= single_test(DMAX2_BURST_SIZE, 64, 4, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(100, 1000, 64, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(DMAX2_BURST_SIZE, 0x8, 0x4, 0xF2281008, DMAX2_TRANS_MEM_ATTR_IO);
		if (!err)
			err |= single_test(DMAX2_BURST_SIZE, 100, 4, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(4, 0x100000, MB_SIZE, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(2, 0x150000, 2*MB_SIZE, 0, DMAX2_TRANS_MEM_ATTR_CACHABLE);
		if (!err)
			err |= single_test(100, 0x8, 0x4, 0xF2281008, DMAX2_TRANS_MEM_ATTR_IO);
		if (!err)
			err |= single_test(100, 0x4, 0x4, 0xf20064c0, DMAX2_TRANS_MEM_ATTR_IO);

		if (garg.repeat > 1) {
			if (err)
				printf("Test %d.. Failed!", i);
			else
				printf("test %d OK!\n", i);
		}
	}

	if (err)
		printf("\nDMAX2 test Failed!\n");
	else
		printf("\nDMAX2 test passed\n");

	err = dmax2_deinit(dmax2);
	if (err)
		printf("\nDMAX2 de-initialization Failed!\n");

free_mem:
	mv_sys_dma_mem_destroy();

	return err;
}

