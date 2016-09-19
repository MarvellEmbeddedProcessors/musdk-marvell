/**************************************************************************//**
		Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include <stdlib.h>
#include <string.h>

#include "std.h"

#include "mem_mng.h"


#define MAKE_ALIGNED(addr, align)	\
	(((u64)(addr) + ((align) - 1)) & (~(((u64)align) - 1)))


/* mem_blk_t data structure defines parameters of the Memory Block */
typedef struct mem_blk {
	struct mem_blk	*next;  /* Pointer to the next memory block */

	u64		 base;  /* Base address of the memory block */
	u64		 end;   /* End address of the memory block */
	char		 name[MEM_MNG_MAX_NAME_LEN];
				/* That block of memory was allocated for something specified by the Name */
} mem_blk_t;

typedef mem_blk_t free_mem_blk_t;
typedef mem_blk_t busy_mem_blk_t;

/* mm_t data structure defines parameters of the MM object */
typedef struct mem_mng {
	spinlock_t	*lock;

	mem_blk_t	*blks;		/* List of memory blocks (Memory list) */
	mem_blk_t	*busy_blks;	/* List of busy blocks (Busy list) */
	mem_blk_t	*free_blks[MEM_MNG_MAX_ALIGNMENT + 1];
		/* Alignment lists of free blocks (Free lists) */

	u64		 free_mem_size; /* Total size of free memory (in bytes) */
} mm_t;


static busy_mem_blk_t * create_busy_blk(u64 base, u64 size, char *name)
{
	busy_mem_blk_t	*busy_blk;
	u32		 n;

	busy_blk = (busy_mem_blk_t *)malloc(sizeof(busy_mem_blk_t));
	if ( !busy_blk ) {
		pr_err("no mem for busy-block obj!\n");
		return NULL;
	}

	busy_blk->base = base;
	busy_blk->end = base + size;

	n = strlen(name);
	strncpy(busy_blk->name, name, n);
	busy_blk->name[n] = '\0';
	busy_blk->next = NULL;

	return busy_blk;
}

static mem_blk_t * create_new_blk(u64 base, u64 size)
{
	mem_blk_t	*mem_blk;

	mem_blk = (mem_blk_t *)malloc(sizeof(mem_blk_t));
	if ( !mem_blk ) {
		pr_err("no mem for block obj!\n");
		return NULL;
	}

	mem_blk->base = base;
	mem_blk->end = base+size;
	mem_blk->next = NULL;

	return mem_blk;
}

static free_mem_blk_t * create_free_blk(u64 base, u64 size)
{
	free_mem_blk_t	*free_blk;

	free_blk = (free_mem_blk_t *)malloc(sizeof(free_mem_blk_t));
	if ( !free_blk ) {
		pr_err("no mem for free-block obj!\n");
		return NULL;
	}

	free_blk->base = base;
	free_blk->end = base + size;
	free_blk->next = NULL;

	return free_blk;
}

static int add_free_blk(mm_t *mm, u64 base, u64 end)
{
	free_mem_blk_t	*prev_blk, *curr_blk, *new_blk;
	u64		 alignment, align_base;
	int		 i;

	/* Updates free lists to include  a just released block */
	for (i=0; i <= MEM_MNG_MAX_ALIGNMENT; i++) {
		prev_blk = new_blk = 0;
		curr_blk = mm->free_blks[i];

		alignment = (u64)(0x1 << i);
		align_base = MAKE_ALIGNED(base, alignment);

		/* Goes to the next free list if there is no block to free */
		if (align_base >= end)
			continue;

		/* Looks for a free block that should be updated */
		while ( curr_blk ) {
			if ( align_base <= curr_blk->end ) {
				if ( end > curr_blk->end ) {
					free_mem_blk_t *nextB;
					while ( curr_blk->next && end > curr_blk->next->end ) {
						nextB = curr_blk->next;
						curr_blk->next = curr_blk->next->next;
						free(nextB);
					}

					nextB = curr_blk->next;
					if ( !nextB || (nextB && end < nextB->base) )
						curr_blk->end = end;
					else {
						curr_blk->end = nextB->end;
						curr_blk->next = nextB->next;
						free(nextB);
					}
				} else if ( (end < curr_blk->base) && ((end-align_base) >= alignment) ) {
					if ((new_blk = create_free_blk(align_base, end-align_base)) == NULL) {
												pr_err("failed to create free block!\n");
												return -ENOMEM;
										}

					new_blk->next = curr_blk;
					if (prev_blk)
						prev_blk->next = new_blk;
					else
						mm->free_blks[i] = new_blk;
					break;
				}

				if ((align_base < curr_blk->base) && (end >= curr_blk->base))
					curr_blk->base = align_base;

				/* if size of the free block is less then alignment
				 * deletes that free block from the free list. */
				if ( (curr_blk->end - curr_blk->base) < alignment) {
					if ( prev_blk )
						prev_blk->next = curr_blk->next;
					else
						mm->free_blks[i] = curr_blk->next;
					free(curr_blk);
					curr_blk = NULL;
				}
				break;
			} else {
				prev_blk = curr_blk;
				curr_blk = curr_blk->next;
			}
		}

		/* If no free block found to be updated, insert a new free block
		 * to the end of the free list.
		 */
		if ( !curr_blk && ((((u64)(end-base)) & ((u64)(alignment-1))) == 0) ) {
			if ((new_blk = create_free_blk(align_base, end-base)) == NULL) {
								pr_err("failed to create free block!\n");
								return -ENOMEM;
						}

			if (prev_blk)
				prev_blk->next = new_blk;
			else
				mm->free_blks[i] = new_blk;
		}

		/* Update boundaries of the new free block */
		if ((alignment == 1) && !new_blk) {
			if ( curr_blk && base > curr_blk->base )
				base = curr_blk->base;
			if ( curr_blk && end < curr_blk->end )
				end = curr_blk->end;
		}
	}

	return 0;
}

static int cut_free_blk(mm_t *mm, u64 hold_base, u64 hold_end)
{
	free_mem_blk_t	*prev_blk, *curr_blk, *new_blk;
	u64		 alignment, align_base, base, end;
	int		 i;

	for (i=0; i <= MEM_MNG_MAX_ALIGNMENT; i++) {
		prev_blk = new_blk = 0;
		curr_blk = mm->free_blks[i];

		alignment = (u64)(0x1 << i);
		align_base = MAKE_ALIGNED(hold_end, alignment);

		while ( curr_blk ) {
			base = curr_blk->base;
			end = curr_blk->end;

			if ( (hold_base <= base) && (hold_end <= end) && (hold_end > base) ) {
				if ( align_base >= end ||
					 (align_base < end && ((end-align_base) < alignment)) ) {
					if (prev_blk)
						prev_blk->next = curr_blk->next;
					else
						mm->free_blks[i] = curr_blk->next;
					free(curr_blk);
				} else
					curr_blk->base = align_base;
				break;
			}
			else if ( (hold_base > base) && (hold_end <= end) ) {
				if ( (hold_base-base) >= alignment ) {
					if ( (align_base < end) && ((end-align_base) >= alignment) ) {
						if ((new_blk = create_free_blk(align_base, end-align_base)) == NULL) {
														pr_err("failed to create free block!\n");
														return -ENOMEM;
												}
						new_blk->next = curr_blk->next;
						curr_blk->next = new_blk;
					}
					curr_blk->end = hold_base;
				} else if ( (align_base < end) && ((end-align_base) >= alignment) )
					curr_blk->base = align_base;
				else {
					if (prev_blk)
						prev_blk->next = curr_blk->next;
					else
						mm->free_blks[i] = curr_blk->next;
					free(curr_blk);
				}
				break;
			} else {
				prev_blk = curr_blk;
				curr_blk = curr_blk->next;
			}
		}
	}

	return 0;
}

static void add_busy_blk(mm_t *mm, busy_mem_blk_t *new_blk)
{
	busy_mem_blk_t	*curr_blk, *prev_blk;

	/* finds a place of a new busy block in the list of busy blocks */
	prev_blk = 0;
	curr_blk = mm->busy_blks;

	while ( curr_blk && new_blk->base > curr_blk->base ) {
		prev_blk = curr_blk;
		curr_blk = curr_blk->next;
	}

	/* insert the new busy block into the list of busy blocks */
	if ( curr_blk )
		new_blk->next = curr_blk;
	if ( prev_blk )
		prev_blk->next = new_blk;
	else
		mm->busy_blks = new_blk;
}

static u64 get_greater_align(mm_t *mm, u64 size, u64 alignment, char* name)
{
	free_mem_blk_t	*free_blk;
	busy_mem_blk_t	*new_blk;
	u64		 hold_base, hold_end, align_base = 0;

	/* goes over free blocks of the 64 byte alignment list
	   and look for a block of the suitable size and
	   base address according to the alignment. */
	free_blk = mm->free_blks[MEM_MNG_MAX_ALIGNMENT];

	while ( free_blk ) {
		align_base = MAKE_ALIGNED(free_blk->base, alignment);

		/* the block is found if the aligned base inside the block
		 * and has the anough size. */
		if ( align_base >= free_blk->base &&
			 align_base < free_blk->end &&
			 size <= (free_blk->end - align_base) )
			break;
		else
			free_blk = free_blk->next;
	}

	/* If such block isn't found */
	if ( !free_blk )
		return (u64)(MEM_MNG_ILLEGAL_BASE);

	hold_base = align_base;
	hold_end = align_base + size;

	/* init a new busy block */
	if ((new_blk = create_busy_blk(hold_base, size, name)) == NULL)
		return (u64)(MEM_MNG_ILLEGAL_BASE);

	/* calls Update routine to update a lists of free blocks */
	if ( cut_free_blk ( mm, hold_base, hold_end ) != 0 ) {
		free(new_blk);
		return (u64)(MEM_MNG_ILLEGAL_BASE);
	}

	/* insert the new busy block into the list of busy blocks */
	add_busy_blk ( mm, new_blk );

	return (hold_base);
}


/**********************************************************************
 *			 MM API routines set			      *
 **********************************************************************/

int mem_mng_init(u64 base, u64 size, struct mem_mng **mm)
{
	mm_t	*mm_o;
	u64	 new_base, new_size;
	int	 i;

	if (!size) {
		pr_err("Illegal size (should be positive)!\n");
		return -EINVAL;
	}

	/* Initializes a new MM object */
	mm_o = (mm_t *)malloc(sizeof(mm_t));
	if (!mm_o) {
		pr_err("no mem for mem-mng obj!\n");
		return -ENOMEM;
	}

	mm_o->lock = spin_lock_create();
	if (!mm_o->lock) {
		free(mm_o);
		pr_err("failed to create spinlock!\n");
		return -ENOMEM;
	}

	/* Initializes counter of free memory to total size */
	mm_o->free_mem_size = size;

	/* A busy list is empty */
	mm_o->busy_blks = 0;

	/* Initializes a new memory block */
	if ((mm_o->blks = create_new_blk(base, size)) == NULL) {
		mem_mng_free(mm_o);
		pr_err("failed to create new mem block!\n");
		return -ENOMEM;
	}

	/* Initializes a new free block for each free list*/
	for (i=0; i <= MEM_MNG_MAX_ALIGNMENT; i++) {
		new_base = MAKE_ALIGNED( base, (0x1 << i) );
		new_size = size - (new_base - base);

		if ((mm_o->free_blks[i] = create_free_blk(new_base, new_size)) == NULL) {
			mem_mng_free(mm_o);
			pr_err("failed to create free mem block!\n");
			return -ENOMEM;
		}
	}

	*mm = mm_o;

	return 0;
}

void mem_mng_free(struct mem_mng *mm)
{
	mem_blk_t	*mem_blk;
	busy_mem_blk_t	*busy_blk;
	free_mem_blk_t	*free_blk;
	void		*blk;
	int		 i;

	if (!mm) {
		pr_err("Invalid handle provided!\n");
		return;
	}

	/* release memory allocated for busy blocks */
	busy_blk = mm->busy_blks;
	while ( busy_blk ) {
		blk = busy_blk;
		busy_blk = busy_blk->next;
		free(blk);
	}

	/* release memory allocated for free blocks */
	for (i=0; i <= MEM_MNG_MAX_ALIGNMENT; i++) {
		free_blk = mm->free_blks[i];
		while ( free_blk ) {
			blk = free_blk;
			free_blk = free_blk->next;
			free(blk);
		}
	}

	/* release memory allocated for memory blocks */
	mem_blk = mm->blks;
	while ( mem_blk ) {
		blk = mem_blk;
		mem_blk = mem_blk->next;
		free(blk);
	}

	if (mm->lock)
		spin_lock_destroy(mm->lock);

	/* release memory allocated for MM object itself */
	free(mm);
}

u64 mem_mng_get(struct mem_mng *mm, u64 size, u64 alignment, char *name)
{
	free_mem_blk_t	*free_blk;
	busy_mem_blk_t	*new_blk;
	u64		 hold_base, hold_end, j, i = 0;
	u32		 flags;

	if (!mm) {
		pr_err("Invalid handle provided!\n");
		return (u64)MEM_MNG_ILLEGAL_BASE;
	}

	if (!name || (strlen(name) >= MEM_MNG_MAX_NAME_LEN)) {
		pr_err("Invalid name provided!\n");
		return (u64)MEM_MNG_ILLEGAL_BASE;
	}

	/* checks that alignment value is greater then zero */
	if (alignment == 0)
		alignment = 1;

	j = alignment;

	/* checks if alignment is a power of two, if it correct and if the
	   required size is multiple of the given alignment. */
	while ((j & 0x1) == 0) {
		i++;
		j = j >> 1;
	}

	/* if the given alignment isn't power of two, returns an error */
	if (j != 1) {
		pr_err("Illegal alignment (should be power of 2)!\n");
		return (u64)MEM_MNG_ILLEGAL_BASE;
	}

	if (i > MEM_MNG_MAX_ALIGNMENT)
		return get_greater_align(mm, size, alignment, name);

	spin_lock_irqsave(mm->lock, flags);
	/* look for a block of the size greater or equal to the required size. */
	free_blk = mm->free_blks[i];
	while ( free_blk && (free_blk->end - free_blk->base) < size )
		free_blk = free_blk->next;

	/* If such block is found */
	if ( !free_blk ) {
		spin_unlock_irqrestore(mm->lock, flags);
		return (u64)(MEM_MNG_ILLEGAL_BASE);
	}

	hold_base = free_blk->base;
	hold_end = hold_base + size;

	/* init a new busy block */
	if ((new_blk = create_busy_blk(hold_base, size, name)) == NULL) {
		spin_unlock_irqrestore(mm->lock, flags);
		return (u64)(MEM_MNG_ILLEGAL_BASE);
	}

	/* calls Update routine to update a lists of free blocks */
	if ( cut_free_blk ( mm, hold_base, hold_end ) != 0 ) {
		spin_unlock_irqrestore(mm->lock, flags);
		free(new_blk);
		return (u64)(MEM_MNG_ILLEGAL_BASE);
	}

	/* Decreasing the allocated memory size from free memory size */
	mm->free_mem_size -= size;

	/* insert the new busy block into the list of busy blocks */
	add_busy_blk ( mm, new_blk );
	spin_unlock_irqrestore(mm->lock, flags);

	return (hold_base);
}

u64 mem_mng_put(struct mem_mng *mm, u64 base)
{
	busy_mem_blk_t	*busy_blk, *prev_blk;
	u64		 size;
	u32		 flags;

	if (!mm) {
		pr_err("Invalid handle provided!\n");
		return (u64)MEM_MNG_ILLEGAL_BASE;
	}

	/* Look for a busy block that have the given base value.
	 * That block will be returned back to the memory.
	 */
	prev_blk = 0;

	spin_lock_irqsave(mm->lock, flags);
	busy_blk = mm->busy_blks;
	while ( busy_blk && base != busy_blk->base ) {
		prev_blk = busy_blk;
		busy_blk = busy_blk->next;
	}

	if ( !busy_blk ) {
		spin_unlock_irqrestore(mm->lock, flags);
		return (u64)0;
	}

	if ( add_free_blk( mm, busy_blk->base, busy_blk->end ) != 0 ) {
		spin_unlock_irqrestore(mm->lock, flags);
		return (u64)0;
	}

	/* removes a busy block form the list of busy blocks */
	if ( prev_blk )
		prev_blk->next = busy_blk->next;
	else
		mm->busy_blks = busy_blk->next;

	size = busy_blk->end - busy_blk->base;

	/* Adding the deallocated memory size to free memory size */
	mm->free_mem_size += size;

	free(busy_blk);
	spin_unlock_irqrestore(mm->lock, flags);

	return (size);
}

int mem_mng_in_range(struct mem_mng *mm, u64 addr)
{
	mem_blk_t	*mem_blk;

	if (!mm) {
		pr_err("Invalid handle provided!\n");
		return 0;
	}

	mem_blk = mm->blks;

	if ((addr >= mem_blk->base) && (addr < mem_blk->end))
		return 1;
	else
		return 0;
}

u64 mem_mng_get_avail_mem(struct mem_mng *mm)
{
	if (!mm) {
		pr_err("Invalid handle provided!\n");
		return (u64)MEM_MNG_ILLEGAL_BASE;
	}

	return mm->free_mem_size;
}

void mem_mng_dump(struct mem_mng *mm)
{
	free_mem_blk_t	*free_blk;
	busy_mem_blk_t	*busy_blk;
	int		 i;

	if (!mm) {
		pr_err("Invalid handle provided!\n");
		return;
	}

	busy_blk = mm->busy_blks;
	printf("List of busy blocks:\n");
	while (busy_blk) {
		printf("\t0x%p: (%s: b=0x%llx, e=0x%llx)\n",
			busy_blk,
			busy_blk->name,
			(long long unsigned int)busy_blk->base,
			(long long unsigned int)busy_blk->end);
		busy_blk = busy_blk->next;
	}

	printf("\nLists of free blocks according to alignment:\n");
	for (i=0; i <= MEM_MNG_MAX_ALIGNMENT; i++) {
		printf("%d alignment:\n", (0x1 << i));
		free_blk = mm->free_blks[i];
		while (free_blk) {
			printf("\t0x%p: (b=0x%llx, e=0x%llx)\n",
				free_blk,
				(long long unsigned int)free_blk->base,
				(long long unsigned int)free_blk->end);
			free_blk = free_blk->next;
		}
		printf("\n");
	}
}
