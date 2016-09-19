###############################################################################
############################  Basic Use-Case LIBRARY   ###########################
###############################################################################
O_TARGETS += dmauc
EXOBJ :=  lmusdk.o dma_uc.o
OBJ := dma_mem.o
INC := $(TOPINC)
DEF := $(TOPDEF)

dma_uc.o: $(OBJ)
	@(echo "(LD)  $@ <= $^")
	@($(LD) -r -o $@ -Map $*.map $^)

dmauc.o: $(EXOBJ)
	@(echo "(LD)  $@ <= $^")
	@($(LD) -r -o $@ -Map $*.map $^)

include $(TOPDIR)/Rules.make
