#!/bin/bash

#
# usage:
#
# banner <target name>
#
banner() {
	echo
	TG=`echo $1 | sed -e "s,/.*/,,g"`
	LINE=`echo $TG |sed -e "s/./-/g"`
	echo $LINE
	echo $TG
	echo $LINE
	echo
}

banner "autoreconf"

mkdir -p m4
autoreconf --force --install -Wall || exit $?
cp aarch64-poky-config.sub config.sub

banner "Finished"
