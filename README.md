# README

## Build

```bash
rm -rf build && mkdir -p build && \
g++ -std=c++17 -O2 -Wall -Wextra \
  -I lib/SPIMasterHandler \
  src/main.cpp lib/SPIMasterHandler/SPIMasterHandler.cpp \
  -o build/spi_master
```

## Sync

```bash
rsync -avz --delete pi@raspberrypi:~/Projects/SPI_COM_Master/ ~/Projects/SPI_COM_Master/
```