# Only in top make

include .env_params

TOPDIR          ?=
KDIR            ?=
TOPLIBGCC       ?=
ARCH            ?= arm64
CROSS_COMPILE   ?= 

TOPSRC  = $(TOPDIR)/../..
CURRDIR = $(TOPDIR)
SRCDIR  = $(TOPSRC)
TD = $(TOPSRC)
D = $(SRCDIR)

CC= $(CROSS_COMPILE)gcc
AS= $(CROSS_COMPILE)as
AR= $(CROSS_COMPILE)ar rvs
LD= $(CROSS_COMPILE)ld

CL_FLAGS ?=

M_BASIC_FLAGS = -gdwarf-2 -g2 -Wall -Wl,--stack,0x800000 \
		-Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common \
		-fomit-frame-pointer -fsigned-char -pipe -ffixed-r2 -Wno-unused-function
M_FLAGS = $(M_BASIC_FLAGS) -isystem -I$(TOPLIBGCC)/include

CFLAGS = $(M_FLAGS)

XFLAGS = $(CL_FLAGS) -include $(TOPDIR)/xflags.h -I$(TOPDIR)

TOPDEF = 
TOPDEF += $(DEFINE)
TOPINC = $(D) \
	$(TD)/src/include/int \
	$(TD)/src/include/drivers \
	$(TD)/src/include \
	$(TD)/modules/uio \
	$(KDIR)/include

O_TARGETS =

#read the root sub.make
SUBDIRS = .


.PHONY: all clean targets
all: $(O_TARGETS)

targets: 
	@(echo targets: $(O_TARGETS))

fix:

clean:
	-rm -r *.o *.a *.d *.map ~* *.mod.c .*.mod.o.cmd .*.cmd .*.ver \
	.tmp_versions $(O_TARGETS)


include $(TOPDIR)/Rules.make
