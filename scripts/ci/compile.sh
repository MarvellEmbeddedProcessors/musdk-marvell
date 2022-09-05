#!/bin/bash
# Copyright (c) 2018 Marvell.
#
# SPDX-License-Identifier: BSD-3-Clause
# https://spdx.org/licenses
###############################################################################
## This is the compile script for musdk                                      ##
## This script is called by CI automated builds                              ##
## It may also be used interactively by users to compile the same way as CI  ##
###############################################################################
## WARNING: Do NOT MODIFY the CI wrapper code segments.                      ##
## You can only modify the config and compile commands                       ##
###############################################################################


## =v=v=v=v=v=v=v=v=v=v=v CI WRAPPER - Do not Modify! v=v=v=v=v=v=v=v=v=v=v= ##
set -euo pipefail
shopt -s extglob
##==================================== USAGE ================================##
function usage {
	echo """
Usage: compile [--no_configure] [--echo_only] BUILD_NAME
 or:   compile --help

Compiles musdk similar to the given CI build

 -e, --echo_only      Print out the compilation sequence but do not execute it
 -h, --help           Display this help and exit

Prerequisites:
  CROSS_COMPILE       path to cross compiler
  MUSDK_INSTALL_PATH  path to which musdk will be installed
  KDIR                path to kernel source
  CDAL_PATH           (optional) path to cdal source
"""
	exit 0
}
##============================ PARSE ARGUMENTS ==============================##
TEMP=`getopt -a -o eh --long echo_only,help \
             -n 'compile' -- "$@"`

if [ $? != 0 ] ; then
	echo "Error: Failed parsing command options" >&2
	exit 1
fi
eval set -- "$TEMP"

no_configure=
echo_only=

while true; do
	case "$1" in
		-N | --no_configure ) no_configure=true; shift ;;
		-e | --echo_only )    echo_only=true; shift ;;
		-h | --help )         usage; ;;
		-- ) shift; break ;;
		* ) break ;;
	esac
done

[[ $# -ne 1 ]] && usage
build_name=$1

echo "running compile.sh ${build_name}"
## =^=^=^=^=^=^=^=^=^=^=^=^  End of CI WRAPPER code -=^=^=^=^=^=^=^=^=^=^=^= ##


########################### MUSDK CONFIGURATION ##############################

modules=(cma dmax2)
flags=""

if [[ ! $build_name =~ linux414 ]]; then
	echo "Error: Build $build_name not supported"; exit -1;
fi
[ "${MUSDK_INSTALL_PATH}" ] \
	|| ( echo "Error: \$MUSDK_INSTALL_PATH must be defined"; exit -1 )

if [[ $build_name =~ _nosam ]]; then
	sam_flag='--enable-sam=no'
else
	modules[${#modules[@]}]="sam"
	sam_flag='--enable-sam'
fi

if [[ $build_name =~ _ks ]]; then
	echo "Error: Build $build_name is currently not supported"; exit -1;
	modules=()
	sam_flag='--enable-sam=no'
fi

if [[ $build_name =~ _prv ]]; then
	modules[${#modules[@]}]="gnic"
	flags="--enable-giu"
fi

fpic=' EXTRA_CFLAGS="-fPIC"' ## assume odp-musdk version >= 1.19

###############################################################################


## =v=v=v=v=v=v=v=v=v=v=v CI WRAPPER - Do not Modify! v=v=v=v=v=v=v=v=v=v=v= ##
cmd="""
set -x
pwd"""
## =^=^=^=^=^=^=^=^=^=^=^=^  End of CI WRAPPER code -=^=^=^=^=^=^=^=^=^=^=^= ##


#################################### COMPILE ##################################
cmd=$cmd"""
export ARCH=arm64"""

if [[ ${#modules[@]} > 0 ]]; then
for m in "${modules[@]}"; do
cmd=$cmd"""
cd modules/${m}
make
cd ../..
"""
done
fi

cmd=$cmd"""
./bootstrap
./configure --prefix=${MUSDK_INSTALL_PATH} ${sam_flag} ${flags} --host=aarch64-linux-gnu \
    CC=\${CROSS_COMPILE}gcc --enable-static --disable-shared
"""

if [[ $build_name =~ _ks ]]; then
cmd=$cmd"""
cd build/lnx4.4_a7k_gcc
make -C \$KDIR M=\$PWD modules -j4
cd ../../apps/examples/ppv2/ks
make -C \$KDIR M=\$PWD modules -j4
cd ../../../..
sudo make install"""
else
cmd=$cmd"""
make -j4 $fpic
sudo make install"""
fi
###############################################################################


## =v=v=v=v=v=v=v=v=v=v=v CI WRAPPER - Do not Modify! v=v=v=v=v=v=v=v=v=v=v= ##
if [[ $echo_only ]]; then
	echo "$cmd"
	exit 0
fi

eval "$cmd"
## =^=^=^=^=^=^=^=^=^=^=^=^  End of CI WRAPPER code -=^=^=^=^=^=^=^=^=^=^=^= ##
