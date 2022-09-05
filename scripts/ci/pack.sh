#!/bin/bash
# Copyright (c) 2018 Marvell.
#
# SPDX-License-Identifier: BSD-3-Clause
# https://spdx.org/licenses
###############################################################################
## This is the pack script for linux kernel                                  ##
## This script is called by CI automated builds                              ##
###############################################################################
## WARNING: Do NOT MODIFY the CI wrapper code segments.                      ##
## You can only modify the config and compile commands                       ##
###############################################################################
## Prerequisites:       DESTDIR is the path to the destination directory
## Usage:               pack BUILD_NAME

## =v=v=v=v=v=v=v=v=v=v=v CI WRAPPER - Do not Modify! v=v=v=v=v=v=v=v=v=v=v= ##
set -exuo pipefail
shopt -s extglob

build_name=$1
echo "running pack.sh ${build_name}"
## =^=^=^=^=^=^=^=^=^=^=^=^  End of CI WRAPPER code -=^=^=^=^=^=^=^=^=^=^=^= ##

mkdir -p $DESTDIR
mkdir -p $DESTDIR/rootfs/lib/modules/musdk

modules=(cma dmax2 sam gnic)
ko=(musdk_cma.ko mv_dmax2_uio.ko mv_sam_uio.ko mgnic_plat.ko)

for i in `seq 0 $( expr ${#modules[@]} - 1 )`; do
	kofile=modules/${modules[$i]}/${ko[$i]}
	if [[ -e $kofile ]]; then
		cp $kofile $DESTDIR/rootfs/lib/modules/musdk/
	fi
done

mkdir -p ${DESTDIR}/rootfs/usr/local
cp -R /usr/local/musdk ${DESTDIR}/rootfs/usr/local/

echo "musdk pack completed"
