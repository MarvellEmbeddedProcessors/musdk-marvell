# Save Target Specific Variables
DEP := $(patsubst %.o, %.d,$(OBJ))
$(OBJ): T_DIR := $(SRCDIR)
$(OBJ): T_INC := $(INC)
$(OBJ): T_DEF := $(DEF)
$(DEP): T_INC := $(INC)
$(DEP): T_DEF := $(DEF)

# Common rules

%.o: $(SRCDIR)/%.c %.d
	@(set -e)
	@(echo "(CC)  $@")
	@($(CC) -Wp,-MD,.$(@).d $(CFLAGS) $(XFLAGS) $(addprefix -D, $(T_DEF)) \
	  $(addprefix -I, $(T_INC)) -c -o $(@) $<)
	@(rm -f .$@.d)

%.d: $(SRCDIR)/%.c
	@(echo \(DEP\) $@)
	@($(CC) -M -E $(CFLAGS) $(XFLAGS) $(addprefix -I, $(T_INC)) $(addprefix -D, $(T_DEF)) -o $@ -c $<)

##############################
# Sub directories itteration #
##############################

#Add the sub dir target to this dir target dependency list and declare them phony
SUBDIRTARGET := $(addsuffix /all, $(addprefix $(strip $(SRCDIR))/, $(SUBDIRS)))
.PHONY: $(SUBDIRTARGET)
$(SRCDIR)/all: $(SUBDIRTARGET)

#Add the new this target new sub dirs to the SUBDIRS_QUEUE queue
NEWSUBS := $(addprefix $(strip $(SRCDIR))/, $(SUBDIRS))
SUBDIRS_QUEUE += $(wildcard $(NEWSUBS))

#Pop the next sub dir to be proccessed
SRCDIR := $(word 1, $(SUBDIRS_QUEUE))
SUBDIRS_QUEUE := $(wordlist 2,$(words $(SUBDIRS_QUEUE)),$(SUBDIRS_QUEUE))

SUBDIRS :=
LIB :=
OBJ :=
INC := $(TOPINC)
DEF := $(TOPDEF)
ifeq ($(words $(SRCDIR)), 0)
	#do nothing
else
	include  $(patsubst $(TOPSRC)%, $(TOPDIR)%, $(SRCDIR)/sub.make)
endif
