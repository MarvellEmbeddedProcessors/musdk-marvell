###############################################################################
#############################  drivers sub.make   #############################
###############################################################################
OBJ := hif.o \
	bpool.o \
	ppio.o \
	cls.o \
	pp2.o \
	pp2_bm.o \
	pp2_dm.o \
	pp2_port.o \
	pp2_mem.o \
	pp2_gop.o \
	pp2_gop_dbg.o \
	pp2_qos.o

INC := $(TOPINC)
DEF := $(TOPDEF)

lpp2.o: $(OBJ)
	@(echo "(LD)  $@ <= $^")
	@($(LD) -r -o $@ -Map $*.map $^)

include $(TOPDIR)/Rules.make
