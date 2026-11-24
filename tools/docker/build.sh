#!/bin/bash
# Build the firmware of all devices in a Docker container.
#   tools/docker/build.sh [output directory]   (default: tools/docker/out)
# Output: <dir>/fw/<device>/ (firmware, flash.sh, README), CHANGELOG,
#         LICENSE and README.md in <dir>/fw/, the same as
#         <dir>/a-culfw-<version>-firmware.zip, and <dir>/build.log
set -e
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
out=${1:-$here/out}
mkdir -p "$out"
out=$(cd "$out" && pwd)

# Build number and date (V command): the last commit changing culfw/ and
# its commit date, so a build of the same sources gives the same firmware.
# "-dirty" if culfw/ has uncommitted changes.
build_number=; build_date=
if git -C "$repo" rev-parse --git-dir >/dev/null 2>&1; then
  build_number=$(git -C "$repo" log -1 --format=%H -- culfw | cut -c1-7)
  build_date=$(git -C "$repo" log -1 --format=%cs -- culfw)
  git -C "$repo" diff --quiet HEAD -- culfw || build_number="$build_number-dirty"
fi

docker build -q -t culfw-build "$here" >/dev/null
docker run --rm --user "$(id -u):$(id -g)" \
  -e BUILD_NUMBER="$build_number" -e BUILD_DATE="$build_date" \
  -v "$repo:/repo:ro" -v "$here:/s:ro" -v "$out:/out" \
  culfw-build /s/buildall.sh
