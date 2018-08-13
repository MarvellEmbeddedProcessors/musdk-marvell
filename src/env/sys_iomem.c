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

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "std_internal.h"

#include "lib/list.h"
#include "lib/uio_helper.h"




struct sys_iomem_format {
	char *uio_hdr;
	char *id_format;
};


#define PP2_NETDEV_PATH		"/sys/class/net/"

#define UIO_HDR_STR		"uio_%s"
#define UIO_ID_FORMAT_STR	"_%d"
#define UIO_MAX_FORMAT_SZ	(sizeof(UIO_HDR_STR) + sizeof(UIO_ID_FORMAT_STR))



#define MMAP_FILE_NAME	"/dev/mem"
#define PAGE_SZ		0x400

#define MAX_NAME_STRING_SIZE	64
#define UIO_VER_INVALID			(-1)

struct mem_mmap_nd {
	int		 index;
	void		*va;
	phys_addr_t	 pa;
	uint64_t	 size;
	struct list	 node;
};
#define MMAP_ND_OBJ(ptr)  LIST_OBJECT(ptr, struct mem_mmap_nd, node)

struct mem_mmap {
	struct device_node	*dev_node;
	struct list		 maps_lst;
};

struct mem_uio {
	struct uio_info_t	*info;
	struct uio_mem_t	*mem;
};

struct mem_shm {
	char		dev_name[MAX_NAME_STRING_SIZE];
	void		*va;
	phys_addr_t	 pa;
	size_t		 size;
};

struct sys_iomem {
	char				*name;
	int				 index;
	enum sys_iomem_type		 type;
	union {
		struct mem_uio		 uio;
		struct mem_mmap		 mmap;
		struct mem_shm		 shmem;
	} u;
	struct list	 node;
};

/* initialized iomem devices list */
static struct list iomem_maps_lst = LIST_INIT(iomem_maps_lst);

static void iomem_uio_add_entry(struct uio_mem_t **headp, struct uio_mem_t *entry)
{
	entry->next = *headp;
	*headp = entry;
}

static struct uio_mem_t *iomem_uio_rm_entry(struct uio_mem_t **headp,
					  const char *name)
{
	struct uio_mem_t *entry = *headp;
	struct uio_mem_t *node = NULL;

	while (entry) {
		if (!strncmp(entry->info->maps[entry->map_num].name,
			     name, UIO_MAX_NAME_SIZE)) {
			*headp = entry->next;
			headp = &*headp;
			node = entry;
			return node;
		}

		headp = &entry->next;
		entry = entry->next;
	}
	return node;
}

static struct uio_info_t *iomem_find_uio_device(const char *name, int index)
{
	char	*tmp_name;
	char	format_buf[UIO_MAX_FORMAT_SZ];
	int	max_str_size = 0;
	struct uio_info_t *uio_info;

	if (name == NULL)
		return 0;

	max_str_size  = strlen(UIO_HDR_STR) + strlen(name);
	max_str_size += strlen(UIO_ID_FORMAT_STR) + INT_32_MAX_DEC_STR_SZ + 1;
	tmp_name = malloc(max_str_size);
	if (!tmp_name) {
		pr_err("no mem for IOMEM-name obj!\n");
		return NULL;
	}
	memset(tmp_name, 0, max_str_size);

	strcpy(format_buf, UIO_HDR_STR);
	if (index < 0)
		snprintf(tmp_name, max_str_size, format_buf, name);
	else {
		strcat(format_buf, UIO_ID_FORMAT_STR);
		snprintf(tmp_name, max_str_size, format_buf, name, index);
	}
	uio_info = uio_find_devices_byname(tmp_name);

	pr_debug("%s: uio_name:%s found:%d\n", __func__, tmp_name, uio_info ? 1 : 0);
	free(tmp_name);
	return uio_info;
}
static int iomem_uio_io_exists(const char *name, int index)
{
	struct uio_info_t *uio_info;

	uio_info = iomem_find_uio_device(name, index);
	return (uio_info != NULL);
}

static int iomem_uio_ioinit(struct mem_uio *uiom, const char *name, int index)
{

	uiom->info = iomem_find_uio_device(name, index);
	if (!uiom->info) {
		pr_err("%s: UIO device not found!\n", __func__);
		return -ENODEV;
	}

	struct uio_info_t *node;

	node = uiom->info;
	while (node) {
		uio_get_all_info(node);
		node = node->next;
	}

	return 0;
}

static void iomem_uio_iodestroy(struct mem_uio *uiom)
{
	uio_free_info(uiom->info);
}

static int iomem_uio_iomap(struct mem_uio	*uiom,
			   const char		*name,
			   phys_addr_t		*pa,
			   void			**va)
{
	struct uio_mem_t	*mem = NULL;

	mem = uio_find_mem_byname(uiom->info, name);
	if (!mem) {
		pr_err("uio mem region (%s) not found!\n", name);
		return -EINVAL;
	}

	if (mem->fd < 0) {
		char dev_name[16];

		snprintf(dev_name, sizeof(dev_name),
			 "/dev/uio%d", mem->info->uio_num);
		mem->fd = open(dev_name, O_RDWR);
	}

	if (mem->fd >= 0) {
		*va = uio_single_mmap(mem->info, mem->map_num, mem->fd);
		if (pa)
			*pa = (phys_addr_t)mem->info->maps[mem->map_num].addr;
		iomem_uio_add_entry(&uiom->mem, mem);
	} else
		uio_free_mem_info(mem);

	return 0;
}

static int iomem_uio_iounmap(struct mem_uio *uiom, const char *name)
{
	struct uio_mem_t *mem;

	mem = iomem_uio_rm_entry(&uiom->mem, name);
	if (!mem)
		return -ENOENT;
	uio_single_munmap(mem->info, mem->map_num);
	/**
	 * TODO
	 * Handle device closing if no map registered as mapped. Change file
	 * descriptor to -1.
	 * If no memory is mapped I don't see any reason to keep the
	 * device opened.
	 *
	 */
	uio_free_mem_info(mem);

	return 0;
}

static struct mem_mmap_nd *mmap_find_iomap_by_index(struct mem_mmap *mmapm, int index)
{
	struct mem_mmap_nd	*mmap_nd;
	struct list		*pos;

	if (!mmapm->maps_lst.next || !mmapm->maps_lst.prev)
		return NULL;

	LIST_FOR_EACH(pos, &mmapm->maps_lst) {
		mmap_nd = MMAP_ND_OBJ(pos);
		if (mmap_nd->index == index)
			return mmap_nd;
	}

	return NULL;
}

static int iomem_mmap_ioinit(struct mem_mmap *mmapm, char *name, int index)
{
	INIT_LIST(&mmapm->maps_lst);

	mmapm->dev_node = of_find_compatible_node_by_indx(NULL, index, NULL, name);
	if (!mmapm->dev_node) {
		pr_err("IO device (%s) not found!\n", name);
		return -EINVAL;
	}

	return 0;
}

static void iomem_mmap_iodestroy(struct mem_mmap *mmapm)
{
	/* TODO: free all objs */
}

static int iomem_mmap_iomap(struct mem_mmap	*mmapm,
			    const char		*name,
			    phys_addr_t		*pa,
			    void		**va)
{
	struct mem_mmap_nd	*mmap_nd;
	const uint32_t		*uint32_prop;
	int			 dev_mem_fd, index = -1;
	uint64_t		 tmp_pa, tmp_size;

	if (name) {
		/* Assuming the name is actually the memory index */
		if (strlen(name) > 2) {
			pr_err("Illegal name length (%d, max is 2)!\n", (int)strlen(name));
			return -EINVAL;
		}
		if (!((name[0] >= '0') && (name[0] <= '9'))) {
			pr_err("Illegal name (%s); must be number!\n", name);
			return -EINVAL;
		}
		if ((strlen(name) == 2) && !((name[1] >= '0') && (name[1] <= '9'))) {
			pr_err("Illegal name (%s); must be number!\n", name);
			return -EINVAL;
		}

		index = atoi(name);
		uint32_prop = of_get_address(mmapm->dev_node, index, &tmp_size, NULL);
		if (!uint32_prop) {
			pr_err("mmap region (%s) not found!\n", name);
			return -EINVAL;
		}
		tmp_pa = of_translate_address(mmapm->dev_node, uint32_prop);
	} else {
		tmp_pa = *pa;
		tmp_size = PAGE_SZ;
	}

	mmap_nd = (struct mem_mmap_nd *)malloc(sizeof(struct mem_mmap_nd));
	if (!mmap_nd) {
		pr_err("no mem for mmap mem region!\n");
		return -ENOMEM;
	}
	memset(mmap_nd, 0, sizeof(struct mem_mmap_nd));
	INIT_LIST(&mmap_nd->node);

	mmap_nd->index = index;
	mmap_nd->pa = tmp_pa;
	mmap_nd->size = tmp_size;

	dev_mem_fd = open(MMAP_FILE_NAME, O_RDWR);
	if (dev_mem_fd < 0) {
		pr_err("UIO file open (%s) = %d (%s)\n",
			MMAP_FILE_NAME, -errno, strerror(errno));
		return -EFAULT;
	}

	mmap_nd->va = mmap(NULL,
			   (size_t)mmap_nd->size,
			   PROT_READ | PROT_WRITE,
			   MAP_SHARED,
			   dev_mem_fd,
			   (off_t)mmap_nd->pa);
	close(dev_mem_fd);
	if (unlikely(mmap_nd->va == MAP_FAILED)) {
		pr_err("mmap() of 0x%016llx = %d (%s)\n",
			(unsigned long long int)mmap_nd->pa, -errno, strerror(errno));
		return -EFAULT;
	}
	list_add_to_tail(&mmap_nd->node, &mmapm->maps_lst);

	*va = mmap_nd->va;
	*pa = mmap_nd->pa;

	pr_debug("IO-remap: va=%p,pa=0x%016llx,sz=0x%llx\n",
		 mmap_nd->va,
		 (unsigned long long int)mmap_nd->pa,
		 (unsigned long long int)mmap_nd->size);

	return 0;
}

static int iomem_mmap_iounmap(struct mem_mmap *mmapm, const char *name)
{
	struct mem_mmap_nd	*mmap_nd;
	int			 err, dev_mem_fd, index = -1;

	if (name) {
		/* Assuming the name is actually the memory index */
		if (strlen(name) > 2) {
			pr_err("Illegal name length (%d, max is 2)!\n", (int)strlen(name));
			return -EINVAL;
		}
		if (!((name[0] >= '0') && (name[0] <= '9'))) {
			pr_err("Illegal name (%s); must be number!\n", name);
			return -EINVAL;
		}
		if ((strlen(name) == 2) && !((name[1] >= '0') && (name[1] <= '9'))) {
			pr_err("Illegal name (%s); must be number!\n", name);
			return -EINVAL;
		}

		index = atoi(name);
	}

	mmap_nd = mmap_find_iomap_by_index(mmapm, index);
	if (!mmap_nd) {
		pr_err("mmap mem region (%s) not found!\n", name);
		return -EINVAL;
	}

	list_del(&mmap_nd->node);

	dev_mem_fd = open(MMAP_FILE_NAME, O_RDWR);
	if (dev_mem_fd < 0) {
		pr_err("UIO file open (%s) = %d (%s)\n",
			MMAP_FILE_NAME, -errno, strerror(errno));
		return -EFAULT;
	}

	err = munmap(mmap_nd->va, mmap_nd->size);
	close(dev_mem_fd);
	if (err) {
		pr_err("munmap() of %p = %d (%s)\n",
			mmap_nd->va, -errno, strerror(errno));
		return -EFAULT;
	}

	free(mmap_nd);

	return 0;
}

static int iomem_shmem_ioinit(struct mem_shm *shm, char *name, int index, u32 size)
{
	memcpy(shm->dev_name, name, sizeof(shm->dev_name));
	if (!size)
		size = sysconf(_SC_PAGE_SIZE);
	shm->size = size;

	return 0;
}

static void iomem_shmem_iodestroy(struct mem_uio *uiom)
{
	/* Nothing to do */
}

static int iomem_shmem_iomap(struct mem_shm	*shm,
			   const char		*name,
			   phys_addr_t		*pa,
			   void			**va)
{
	int fd;
	void *ptr;

	if (!shm->va) {
		fd = open(shm->dev_name, O_RDWR);
		if (fd < 0) {
			pr_err("CMA: open() failed\n");
			return -1;
		}

		ptr = mmap(NULL, shm->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)*pa);
		if (ptr == MAP_FAILED) {
			pr_err("mmap failed\n");
			return -1;
		}

		shm->pa = *pa;
		shm->va = ptr;
		pr_info("shm->name %s, pa 0x%" PRIx64 ", va %p, size %zu\n",
			shm->dev_name, (u64)shm->pa, shm->va, shm->size);
	}

	*va = shm->va;
	return 0;
}

static int iomem_shmem_iounmap(struct mem_shm *shm, const char *name)
{
	int err;

	err = munmap(shm->va, shm->size);
	if (err != 0) {
		pr_err("munmap failed\n");
		return -1;
	}
	return 0;
}

int sys_iomem_exists(struct sys_iomem_params *params)
{
	if (params->type == SYS_IOMEM_T_UIO)
		return iomem_uio_io_exists(params->devname, params->index);
	pr_err("IOtype not supported yet!\n");
	return -ENOTSUP;
}

int sys_iomem_get_info(struct sys_iomem_params *params, struct sys_iomem_info *info)
{
	struct list		*pos;
	struct sys_iomem	*liomem_node;

	if (!list_is_empty(&iomem_maps_lst)) {
		LIST_FOR_EACH(pos, &iomem_maps_lst) {
			liomem_node = LIST_OBJECT(pos, struct sys_iomem, node);
			if (strcmp(liomem_node->name, params->devname) == 0
				&& liomem_node->index == params->index) {
				if (params->type == SYS_IOMEM_T_SHMEM) {
					info->u.shmem.va = liomem_node->u.shmem.va;
					info->u.shmem.paddr = liomem_node->u.shmem.pa;
					info->u.shmem.size = liomem_node->u.shmem.size;
				} else {
					pr_err("IOtype not supported yet!\n");
					return -ENOTSUP;
				}

				return 0;
			}
		}
	}

	return -ENXIO;
}

int sys_iomem_init(struct sys_iomem_params *params, struct sys_iomem **iomem)
{
	struct sys_iomem	*liomem;
	int			 err;
	struct list		*pos;
	struct sys_iomem	*liomem_node;

	/* Check if requested iomem device already initialized */
	if (!list_is_empty(&iomem_maps_lst)) {
		LIST_FOR_EACH(pos, &iomem_maps_lst) {
			liomem_node = LIST_OBJECT(pos, struct sys_iomem, node);
			if (strcmp(liomem_node->name, params->devname) == 0
				&& liomem_node->index == params->index) {
				pr_warn("requested iomem device already initialized\n");
			}
		}
	}

	liomem = malloc(sizeof(struct sys_iomem));
	if (!liomem) {
		pr_err("no mem for IOMEM obj!\n");
		return -ENOMEM;
	}
	memset(liomem, 0, sizeof(struct sys_iomem));

	liomem->type = params->type;

	liomem->name = malloc(strlen(params->devname)+1);
	if (!liomem->name) {
		pr_err("no mem for IOMEM-name obj!\n");
		return -ENOMEM;
	}
	memcpy(liomem->name, params->devname, strlen(params->devname));
	liomem->name[strlen(params->devname)] = '\0';
	liomem->index = params->index;

	if (liomem->type == SYS_IOMEM_T_UIO) {
		err = iomem_uio_ioinit(&liomem->u.uio, liomem->name, liomem->index);
		if (err) {
			free(liomem->name);
			free(liomem);
			return err;
		}
	} else if (liomem->type == SYS_IOMEM_T_MMAP) {
		err = iomem_mmap_ioinit(&liomem->u.mmap, liomem->name, liomem->index);
		if (err) {
			free(liomem->name);
			free(liomem);
			return err;
		}
	} else if (liomem->type == SYS_IOMEM_T_SHMEM) {
		err = iomem_shmem_ioinit(&liomem->u.shmem, liomem->name, liomem->index, params->size);
		if (err) {
			free(liomem->name);
			free(liomem);
			return err;
		}
	} else {
		pr_err("IOtype not supported yet!\n");
		return -ENOTSUP;
	}

	/* add to list: name & index */
	list_add_to_tail(&liomem->node, &iomem_maps_lst);

	*iomem = liomem;
	return 0;
}

void sys_iomem_deinit(struct sys_iomem *iomem)
{
	bool device_exists	= false;
	struct list		*pos;
	struct sys_iomem	*liomem_node;

	/* Check if requested iomem device is actually initialized */
	if (!list_is_empty(&iomem_maps_lst)) {
		LIST_FOR_EACH(pos, &iomem_maps_lst) {
			liomem_node = LIST_OBJECT(pos, struct sys_iomem, node);
			if (strcmp(liomem_node->name, iomem->name) == 0
				&& liomem_node->index == iomem->index) {
				device_exists = true;
			}
		}
	}

	if (!device_exists) {
		pr_err("requested iomem device to deinit wasn't initialized\n");
		return;
	}

	/* Remove from list of iomem devices */
	list_del(&iomem->node);

	if (iomem->type == SYS_IOMEM_T_UIO)
		iomem_uio_iodestroy(&iomem->u.uio);
	else if (iomem->type == SYS_IOMEM_T_MMAP)
		iomem_mmap_iodestroy(&iomem->u.mmap);
	else if (iomem->type == SYS_IOMEM_T_SHMEM)
		iomem_shmem_iodestroy(&iomem->u.uio);
	else {
		pr_warn("IOtype not supported yet!\n");
		return;
	}
	free(iomem->name);
	free(iomem);
}

int sys_iomem_map(struct sys_iomem *iomem, const char *name, phys_addr_t *pa, void **va)
{
	if (iomem->type == SYS_IOMEM_T_MMAP)
		return iomem_mmap_iomap(&iomem->u.mmap, name, pa, va);
	if (iomem->type == SYS_IOMEM_T_UIO)
		return iomem_uio_iomap(&iomem->u.uio, name, pa, va);
	if (iomem->type == SYS_IOMEM_T_SHMEM)
		return iomem_shmem_iomap(&iomem->u.shmem, name, pa, va);
	pr_err("IOtype not supported yet!\n");
	return -ENOTSUP;
}

int sys_iomem_unmap(struct sys_iomem *iomem, const char *name)
{
	if (iomem->type == SYS_IOMEM_T_MMAP)
		return iomem_mmap_iounmap(&iomem->u.mmap, name);
	if (iomem->type == SYS_IOMEM_T_UIO)
		return iomem_uio_iounmap(&iomem->u.uio, name);
	if (iomem->type == SYS_IOMEM_T_SHMEM)
		return iomem_shmem_iounmap(&iomem->u.shmem, name);
	pr_err("IOtype not supported yet!\n");
	return -ENOTSUP;
}
