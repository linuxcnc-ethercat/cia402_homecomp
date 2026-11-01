#!/bin/bash
set -euo pipefail
umask 022
unset CDPATH

ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
LINUXCNC_SRC=${LINUXCNC_SRC:-$ROOT/../linuxcnc}
LINUXCNC_REF=${LINUXCNC_REF:?Set LINUXCNC_REF to the exact LinuxCNC source ref}
PACKAGE_VERSION=${PACKAGE_VERSION:-0.1.0~test1}
ARCH=${ARCH:-amd64}
MODULE="$LINUXCNC_SRC/rtlib/cia402_homecomp.so"
REF_LABEL=${LINUXCNC_REF#v}
REF_LABEL=${REF_LABEL//\//-}
PACKAGE="cia402-homecomp_${PACKAGE_VERSION}+linuxcnc${REF_LABEL}_${ARCH}"
STAGE="$ROOT/build/package/$PACKAGE"
DIST="$ROOT/dist"

if [ ! -f "$MODULE" ]; then
    echo "Build cia402_homecomp before packaging: $MODULE is missing" >&2
    exit 1
fi
if ! command -v dpkg-deb >/dev/null; then
    echo "dpkg-deb is required to build the test package" >&2
    exit 1
fi

rm -rf "$STAGE"
mkdir -p "$STAGE/DEBIAN" "$STAGE/usr/lib/linuxcnc/modules" \
    "$STAGE/usr/share/doc/cia402-homecomp" "$DIST"
install -m 0644 "$MODULE" "$STAGE/usr/lib/linuxcnc/modules/cia402_homecomp.so"
install -m 0644 "$ROOT/TESTING.md" "$STAGE/usr/share/doc/cia402-homecomp/TESTING.md"
install -m 0644 "$ROOT/LICENSE" "$STAGE/usr/share/doc/cia402-homecomp/copyright"

cat > "$STAGE/DEBIAN/control" <<EOF
Package: cia402-homecomp
Version: $PACKAGE_VERSION+linuxcnc$REF_LABEL
Architecture: $ARCH
Maintainer: linuxcnc-ethercat contributors
Section: misc
Priority: optional
Description: CiA 402 drive-controlled homing test module
 Test build of cia402_homecomp for LinuxCNC $LINUXCNC_REF.
 Use only with the matching LinuxCNC version and linuxcnc-ethercat cia402.
EOF

{
    echo "Source: https://github.com/linuxcnc-ethercat/cia402_homecomp"
    echo "Commit: $(git -C "$ROOT" rev-parse HEAD)"
    echo "LinuxCNC source ref: $LINUXCNC_REF"
    echo "Package: $PACKAGE.deb"
    echo "Hardware validation: not performed"
} > "$STAGE/usr/share/doc/cia402-homecomp/BUILD-INFO"

dpkg-deb --build --root-owner-group "$STAGE" "$DIST/$PACKAGE.deb"
echo "$DIST/$PACKAGE.deb"
