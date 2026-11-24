# Alternative culfw for cul devices
___
**The orginal fimware is cloned from www.culfw.de (thanks for the work to R.K.)**

**This alternative firmware has additional send receive methods and code cleanups.**  
_The compiled firmware can be found at [MediaFire](https://www.mediafire.com/folder/iuf7lue8r578c/a-culfw)_

## Building

`tools/docker/build.sh [output directory]` builds the firmware of all devices
in a Docker container (Debian bookworm, avr-gcc and arm-none-eabi-gcc). The
firmware is written to `<output directory>/fw/<device>/`, together with the
`flash.sh` and README of the device, and packed into
`<output directory>/a-culfw-<version>-firmware.zip`; the build log is
`<output directory>/build.log` (default output directory: `tools/docker/out`).
