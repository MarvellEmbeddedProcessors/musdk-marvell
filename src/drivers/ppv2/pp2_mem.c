
#include "std_internal.h"
#include "lib/uio_helper.h"

#include "pp2_types.h"
#include "pp2_mem.h"
#include "pp2.h"

/**
 * User I/O map API
 *
 */


int pp2_sys_io_exists(const char *name)
{
	struct uio_info_t * uio_info;

	if (name == NULL)
		return 0;
	uio_info = uio_find_devices_byname(name);
	if (uio_info)
		return 1;
	return 0;
}

int pp2_sys_ioinit(pp2_maps_handle_t *pp2_maps_hdl, const char *name)
{
	struct pp2_uio **pp = (struct pp2_uio **)pp2_maps_hdl;

	*pp = calloc(1, sizeof(struct pp2_uio));
	if (!*pp)
		return -ENOMEM;

	(*pp)->uio_info = uio_find_devices_byname(name);
	if (!(*pp)->uio_info)
		return -ENODEV;

	struct uio_info_t *node;

	node = (*pp)->uio_info;
	while (node) {
		uio_get_all_info(node);
		node = node->next;
	}

	return 0;
}

static void add_mem_entry(struct uio_mem_t **headp, struct uio_mem_t *entry)
{
	entry->next = *headp;
	*headp = entry;
}

static struct uio_mem_t *remove_mem_entry(struct uio_mem_t **headp,
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

uintptr_t pp2_sys_iomap(pp2_maps_handle_t pp2_maps_hdl, uint32_t *pa,
		       const char *name)
{
	struct pp2_uio *pp = (struct pp2_uio *)pp2_maps_hdl;
	struct uio_mem_t *mem = NULL;
	uintptr_t va = (uintptr_t)NULL;

	mem = uio_find_mem_byname(pp->uio_info, name);
	if (!mem)
		return (uintptr_t)NULL;

	if (mem->fd < 0) {
		char dev_name[16];

		snprintf(dev_name, sizeof(dev_name),
			 "/dev/uio%d", mem->info->uio_num);
		mem->fd = open(dev_name, O_RDWR);
	}

	if (mem->fd >= 0) {
		va = (uintptr_t)uio_single_mmap(mem->info,
						mem->map_num, mem->fd);
		if (pa)
			*pa = mem->info->maps[mem->map_num].addr;
		add_mem_entry(&pp->mem, mem);
	} else {
		uio_free_mem_info(mem);
	}

	return va;
}

int pp2_sys_iounmap(pp2_maps_handle_t pp2_maps_hdl, const char *name)
{
	struct pp2_uio *pp = (struct pp2_uio *)pp2_maps_hdl;
	struct uio_mem_t *mem;

	mem = remove_mem_entry(&pp->mem, name);
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

void pp2_sys_iodestroy(pp2_maps_handle_t pp2_maps_hdl)
{
	struct pp2_uio *pp = (struct pp2_uio *)pp2_maps_hdl;

	uio_free_info(pp->uio_info);
	free(pp);
}
