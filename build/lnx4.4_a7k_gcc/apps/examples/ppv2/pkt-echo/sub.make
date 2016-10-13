###############################################################################
########################  Pkt-Echo Use-Case LIBRARY   #########################
###############################################################################
O_TARGETS += pkt-echo
EXOBJ :=  lmusdk.o pkt-echo_uc.o
OBJ := pkt_echo.o
INC := $(TOPINC)
DEF := $(TOPDEF)

pkt-echo_uc.o: $(OBJ)
	@(echo "(LD)  $@ <= $^")
	@($(LD) -r -o $@ -Map $*.map $^)

pkt-echo.o: $(EXOBJ)
	@(echo "(LD)  $@ <= $^")
	@($(LD) -r -o $@ -Map $*.map $^)

include $(TOPDIR)/Rules.make
