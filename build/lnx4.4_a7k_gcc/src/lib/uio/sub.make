###############################################################################
#########################  lib-uio LIBRARY   ###################################
###############################################################################
OBJ := uio_find_devices.o \
	uio_find_devices_byname.o \
	uio_free.o \
	uio_get_all_info.o \
	uio_get_device_attributes.o \
	uio_get_event_count.o \
	uio_get_mem_addr.o \
	uio_get_mem_size.o \
	uio_get_mem_name.o \
	uio_get_name.o \
	uio_get_version.o \
	uio_line_from_file.o \
	uio_num_from_filename.o \
	uio_single_mmap.o \
	uio_find_mem_byname.o

INC := $(TOPINC)
DEF := $(TOPDEF)

llibuio.o: $(OBJ)
	@(echo "(LD)  $@ <= $^")
	@($(LD) -r -o $@ -Map $*.map $^)

include $(TOPDIR)/Rules.make
