###############################################################################
#############################  drivers sub.make   #############################
###############################################################################
OBJ := pp2.o \
	pp2_hif.o \
	pp2_bpool.o \
	pp2_ppio.o \
	pp2_cls.o \
	pp2_bm.o \
	pp2_dm.o \
	pp2_port.o \
	pp2_mem.o \
	pp2_gop.o \
	pp2_gop_dbg.o \
	pp2_hw_cls.o \
	pp2_hw_cls_dbg.o

INC := $(TOPINC)
DEF := $(TOPDEF)

lpp2.o: $(OBJ)
	@(echo "(LD)  $@ <= $^")
	@($(LD) -r -o $@ -Map $*.map $^)

include $(TOPDIR)/Rules.make
