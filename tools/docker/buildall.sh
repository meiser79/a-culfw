#!/bin/bash
# Runs inside the culfw-build container (see build.sh):
#   /repo repository (read only)
#   /out  output: fw/<device>/ (*.hex|*.bin|*.dfu, flash.sh, README),
#         fw/CHANGELOG, LICENSE, README.md, all of fw/ as
#         a-culfw-<version>-firmware.zip, and build.log
# Builds every device of culfw/Devices/Makefile (target all) plus megaCUL.
# BUILD_NUMBER and BUILD_DATE (environment, optional) replace "private
# build" and "unknown" in version.h of the build copy (V command).

# The sources rely on common symbols, newer compilers default to -fno-common
mkdir -p /tmp/wr
for t in arm-none-eabi-gcc avr-gcc; do
  printf '#!/bin/sh\nexec /usr/bin/%s -fcommon "$@"\n' $t > /tmp/wr/$t
  chmod +x /tmp/wr/$t
done
export PATH=/tmp/wr:$PATH

rm -rf /tmp/w /out/fw /out/build.log /out/a-culfw-*-firmware.zip
cp -r /repo/culfw /tmp/w
[ -n "$BUILD_NUMBER" ] && sed -i "s/^#define BUILD_NUMBER .*/#define BUILD_NUMBER            \"$BUILD_NUMBER\"/" /tmp/w/version.h
[ -n "$BUILD_DATE" ] && sed -i "s/^#define BUILD_DATE .*/#define BUILD_DATE              \"$BUILD_DATE\"/" /tmp/w/version.h
grep -E 'BUILD_(NUMBER|DATE)' /tmp/w/version.h | sed 's/^#define /  /'
cd /tmp/w/Devices
find /tmp/w \( -name '*.hex' -o -name '*.bin' -o -name '*.dfu' \) -delete
touch /tmp/marker

# one make call per device, so a failing device does not stop the others
sed -n '/^all:/,/^$/p' Makefile | grep -E '^\s+make -C' | sed 's/^\s*//' |
while read -r cmd; do
  dir=$(echo "$cmd" | awk '{print $3}'); goal=$(echo "$cmd" | awk '{print $4}')
  if (cd /tmp/w/Devices && $cmd -k >> /out/build.log 2>&1); then
    echo "ok   $dir $goal"
  else
    echo "FAIL $dir $goal"
  fi
done
(cd /tmp/w/Devices/megaCUL && make -k all >> /out/build.log 2>&1) \
  && echo "ok   megaCUL all" || echo "FAIL megaCUL all"

find . -newer /tmp/marker \( -name '*.hex' -o -name '*.bin' -o -name '*.dfu' \) |
sort | while read -r f; do
  mkdir -p "/out/fw/$(dirname "$f")"
  cp "$f" "/out/fw/$f"
done
# flash.sh and README of each built device (flash.sh expects the firmware
# in its own directory)
for d in /out/fw/*/; do
  dev=$(basename "$d")
  for f in /tmp/w/Devices/$dev/flash.sh /tmp/w/Devices/$dev/README*; do
    [ -f "$f" ] && cp "$f" "$d"
  done
  [ -f "$d/flash.sh" ] && chmod 755 "$d/flash.sh"
done
cp /repo/CHANGELOG /repo/LICENSE /repo/README.md /out/fw/
echo "$(find /out/fw -type f | wc -l) files in fw/, log in build.log"

version=$(sed -n 's/^#define VERSION *"\(.*\)"/\1/p' /tmp/w/version.h)
zipname=a-culfw-$version-firmware.zip
(cd /out/fw && find . -type f | sed 's|^\./||' | LC_ALL=C sort | zip -q -X -@ "/out/$zipname")
echo "$zipname"
