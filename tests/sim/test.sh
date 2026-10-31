#!/bin/bash
set -e
unset CDPATH

DIR=$(cd -- "$(dirname -- "$0")" && pwd)
LINUXCNC_SRC=${LINUXCNC_SRC:-$DIR/../../../linuxcnc}
. "$LINUXCNC_SRC/scripts/rip-environment"
cd "$DIR"
unset DISPLAY
exec linuxcnc -r test.ini
