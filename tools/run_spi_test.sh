#!/bin/bash
set -euo pipefail

BUILD_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$BUILD_DIR"

gcc -O2 -Wall -o spi_test spi_test.c
echo "Built spi_test"

if [ "$#" -eq 0 ]; then
  echo "Usage: sudo $0 /dev/spidev0.0 [speed_hz]"
  echo "Running one test with /dev/spidev0.0 6000000"
  sudo ./spi_test /dev/spidev0.0 6000000
else
  sudo ./spi_test "$@"
fi

