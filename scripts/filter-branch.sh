#!/bin/sh

# Run filter-branch to remove proprietary files
git filter-branch --force --index-filter '\
git rm -rf --cached --ignore-unmatch src/hw_emul \
git rm -rf --cached --ignore-unmatch src/mng \
git rm -rf --cached --ignore-unmatch src/drivers/giu \
git rm -rf --cached --ignore-unmatch src/drivers/mqa \
git rm -rf --cached --ignore-unmatch src/drivers/agnic \
git rm -rf --cached --ignore-unmatch src/include/hw_emul \
git rm -rf --cached --ignore-unmatch src/include/mng \
git rm -rf --cached --ignore-unmatch src/include/drivers/giu_regfile_def.h \
git rm -rf --cached --ignore-unmatch src/include/drivers/mv_giu_bpool.h \
git rm -rf --cached --ignore-unmatch src/include/drivers/mv_giu_gpio.h \
git rm -rf --cached --ignore-unmatch src/include/drivers/mqa_def.h \
git rm -rf --cached --ignore-unmatch src/include/drivers/mv_mqa.h \
git rm -rf --cached --ignore-unmatch src/include/drivers/mv_mqa_queue.h \
git rm -rf --cached --ignore-unmatch src/include/drivers/mv_agnic_pfio.h \
git rm -rf --cached --ignore-unmatch src/include/drivers/mv_giu.h \
git rm -rf --cached --ignore-unmatch modules/gnic \
git rm -rf --cached --ignore-unmatch apps/common/giu_utils.c \
git rm -rf --cached --ignore-unmatch apps/common/nmp_guest_utils.c \
git rm -rf --cached --ignore-unmatch apps/include/giu_utils.h \
git rm -rf --cached --ignore-unmatch apps/include/nmp_guest_utils.h \
git rm -rf --cached --ignore-unmatch apps/examples/giu \
git rm -rf --cached --ignore-unmatch apps/examples/nmp \
git rm -rf --cached --ignore-unmatch apps/tests/giu \
git rm -rf --cached --ignore-unmatch doc/musdk_giu_user_guide.txt \
git rm -rf --cached --ignore-unmatch doc/musdk_mqa_user_guide.txt \
git rm -rf --cached --ignore-unmatch doc/musdk_nmp_user_guide.txt \
git rm -rf --cached --ignore-unmatch doc/musdk_nmp_guest_user_guide.txt \
git rm -rf --cached --ignore-unmatch scripts/filter-branch.sh \
git rm -rf --cached --ignore-unmatch scripts/clean-branch.sh \
' --prune-empty @
