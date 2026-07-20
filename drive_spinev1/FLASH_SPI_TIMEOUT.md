# SPIneV1_spi_timeout — прошивка без вечного SPI-hang

## Зачем
Оригинал Стэнфорда в `spi_isr()` ждёт SPI с `HAL_MAX_DELAY` (вечно).
Оборванный кадр / рассинхрон CE → STM зависает, Pi видит `all_zero`, нужен power-cycle.

Здесь в готовом `SPIneV1.elf` заменён только timeout: **50 ms** вместо infinity.
Собирать из исходников не нужно.

## Что прошивать
Файл: `drive_spinev1/SPIneV1_spi_timeout.elf`  
(или `.bin` — если bin, стартовый адрес `0x08000000`)

## Как (как обычно со сайта Стэнфорда)
1. STM32CubeProgrammer + ST-Link V2
2. Connect → Open file → `SPIneV1_spi_timeout.elf` → Download
3. Отключить ST-Link, питание control board

## Ограничения патча
- Только timeout: MCU больше не висит навсегда.
- Полный reinit SPI + «не слать мусор в CAN при bad checksum» есть в исходнике
  `Core_unpacked/Core/Src/main.c`, но для этого нужна сборка в CubeIDE.
- На хосте с одной STM: `sudo python3 tools/spine_spi_test.py --board0-only ...`
  (hold/wiggle/homing-full уже не трогают CE1).

## Откат
Прошей обратно оригинальный `SPIneV1.elf` / файл со сайта Стэнфорда.
