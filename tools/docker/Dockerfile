# Build environment for a-culfw (AVR and ARM devices)
# avr-gcc 5.4.0, arm-none-eabi-gcc 12.2.1 (Debian bookworm)
FROM debian:bookworm-slim
RUN apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends \
      gcc-avr avr-libc binutils-avr make \
      gcc-arm-none-eabi libnewlib-arm-none-eabi bc git zip \
 && rm -rf /var/lib/apt/lists/*
