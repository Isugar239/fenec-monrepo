#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
gcc -O2 -Wall -o spi_watch spi_watch.c
echo "Built spi_watch"
if [ "$#" -lt 2 ]; then
  echo "Usage: sudo $0 /dev/spidev0.0 /dev/spidev0.1 [speed_hz]"
  echo "Running default: /dev/spidev0.0 /dev/spidev0.1 6000000"
  sudo ./spi_watch /dev/spidev0.0 /dev/spidev0.1 6000000
else
  sudo ./spi_watch "$@"
fi

