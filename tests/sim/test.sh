#!/bin/bash
set -e
unset CDPATH

DIR=$(cd -- "$(dirname -- "$0")" && pwd)
LINUXCNC_SRC=${LINUXCNC_SRC:-$DIR/../../../linuxcnc}
. "$LINUXCNC_SRC/scripts/rip-environment"
cd "$DIR"
unset DISPLAY
linuxcnc -r test.ini
if grep -q 'double home_search_dist' "$LINUXCNC_SRC/src/emc/motion/homing.h"; then
    CIA402_TEST_RETURN=1 linuxcnc -r peer.ini
else
    linuxcnc -r peer.ini
fi
linuxcnc -r flags.ini
exec linuxcnc -r absolute.ini
