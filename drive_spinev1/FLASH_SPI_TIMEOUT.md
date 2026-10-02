# SPIneV1_spi_timeout_led — timeout 50 ms + LED PC5 при приёме SPI

Патч готового `SPIneV1.elf`, без пересборки CubeIDE.

## Что делает
- SPI timeout **50 ms** вместо вечного `HAL_MAX_DELAY`.
- При **успешном** кадре SPI светодиод **PC5** мигает ~8 Гц (по `HAL_GetTick`).
- Нет SPI / таймаут — LED **выключен**, кадр в CAN не уходит.
- Старт не тронут: `MX_GPIO_Init` как в оригинале, LED после reset выключен.

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

После boot LED молчит. Мигает, только пока доходят полные SPI-кадры. Если молчит при `spi_loopback` / роботе — нет 3.3 V / BOOT0 / не та плата / кадр не принимается.

## Откат
`SPIneV1.elf` — оригинал. `SPIneV1_spi_timeout.elf` — только timeout, без LED.
