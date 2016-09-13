#!/bin/sh

DT=/proc/device-tree

# $1 - node
# $2 - property
of_get_property()
{
    cat $DT$1/$2
}

# $1 - from
# $2 - compatible
# $3 - node
of_find_compatible_node()
{
    of_find_compatible_nodes $1 $2 | tail -n +$3 | head -1 | tr -d \\n
}

# $1 - from
# $2 - compatible
of_find_compatible_nodes()
{
    for c in $(fgrep -l $2 $(find $DT$1 -name compatible))
    do
        dirname $c
    done | cut -b 18-
}

# $1 - phandle
of_find_node_by_phandle()
{
    dirname $(fgrep -l $1 $(find $DT -name linux,phandle)) | cut -b 18- | tr -d \\n
}

exec <&-
exec 2>/dev/null

$@
