# SPIneV1_spi_timeout_led — timeout 50 ms + LED на PC5 всегда

Патч готового `SPIneV1.elf`, без пересборки CubeIDE.

## Что делает
- SPI timeout **50 ms** вместо вечного `HAL_MAX_DELAY`.
- При старте LED **PC5** включается и **больше не гаснет**.

## Что прошивать
`drive_spinev1/SPIneV1_spi_timeout_led.elf`  
или `.bin` с адреса `0x08000000`.

```bash
scp pi@<IP>:~/pupperv3-monorepo/drive_spinev1/SPIneV1_spi_timeout_led.elf .
```

## Как
1. STM32CubeProgrammer + ST-Link V2
2. Connect → Open file → `SPIneV1_spi_timeout_led.elf` → Download
3. Отключить ST-Link, питание control board. BOOT0 = LOW.

LED горит сразу после старта MCU. Если не горит — нет 3.3 V / BOOT0 / не та плата.

## Откат
`SPIneV1.elf` — оригинал. `SPIneV1_spi_timeout.elf` — только timeout, без LED.
