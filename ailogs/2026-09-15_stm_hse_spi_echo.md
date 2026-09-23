# Лог: LED/HSE/SPI echo на STM; Pi выключают

Дата: **2026-09-15** (~20:00–20:42 MSK)  
Хост: `pupper` (`pi@10.2.0.217`), uptime ~51 мин на момент записи  
Предыдущие: [2026-09-15_stm_all_zero.md](2026-09-15_stm_all_zero.md), [2026-08-31_sd_dying_boot_slow.md](2026-08-31_sd_dying_boot_slow.md)

`robot.service` на момент лога: **inactive**.

---

## Контекст дня (до вечера)

- Утром/днём Pi с **дохлой Samsung EC1S5** плохо грузился (`Card stuck`, undervoltage). Клон на меньшую «64 ГБ» **обрезан на ~1.1 ГиБ** (byte-for-byte, пустое место раздела). Не использовать этот клон как есть.
- Позже Pi снова в сети: `10.2.0.217`, hostname `pupper`, репо `~/pupperv3-monorepo`.
- Днём (см. `2026-09-15_stm_all_zero.md`): обе STM **all_zero**, homing ложный на нулях, checksum 0==0. Pi SPI жив. ST-Link не был вставлен.

---

## Что прошивали вечером (не оригинал SPIne)

Оригинал для отката:

`drive_spinev1/SPIneV1_spi_timeout_led.elf`  
LED в нём **PC5 всегда ON** после boot, SPI timeout 50 ms. Не мигает по приёму.

Тесты STM32duino, плата **Generic F446RETx**, SWD, BOOT0=LOW после заливки:

| Скетч | Такт | Что делает |
|--------|------|------------|
| `tools/stm32duino_led_blink/stm32_led_blink_pc5.ino` | HSI 16 МГц | 3 вспышки + 1 Гц. MCU жива без кварца |
| `tools/stm32duino_spi_highz/stm32_spi_pins_highz.ino` | HSI | SPI пины high-Z, LED 1 Гц |
| `tools/stm32duino_spi_slave/stm32_spi_slave.ino` | ядро Arduino | 8-bit echo, без LED |
| **`tools/stm32duino_spi_rx_led/stm32_spi_rx_led.ino`** | **HSE 8 МГц → PLL 168 МГц** | echo + LED по RX |

Рабочая копия в Arduino IDE: `C:\Users\Service\Documents\Arduino\sketch_feb24a\sketch_feb24a.ino` (синхрон с `stm32_spi_rx_led.ino`).

---

## Наблюдения вечера

1. **HSI + echo:** LED мигал, Pi видел ответ, но «чуть не то». Это **нормальный pipeline SPI slave на 1 байт**:
   - MISO[0] = preload `0xA0`
   - MISO[i] = MOSI[i-1]
   - `spi_loopback.py` это считает `OK — STM32 SPI echo`
2. **Первый HSE 180 МГц (OverDrive + PLLR):** Arduino core **не собрал** (`PLLR_Pos`, `PWR_CR_ODEN`).
3. **HSE 168 МГц без PLLR:** собралось, LED **молчал**. На F446 нули в PLLR → PLL не лочится, hang на `PLLRDY`.
4. **PLLR=2 (`2<<28`) + коды ошибок LED:** пачки вспышек. Пользователь: **«2 раза»** = код **VOS** (ложный таймаут `VOSRDY` в Arduino-заголовках).
5. **VOSRDY wait убран** (20:38). Актуальная прошивка ждёт только HSE / PLL / SWS.

Коды LED после короткой вспышки «MCU жив» (актуальный скетч):

| Паттерн | Смысл |
|---------|--------|
| 3 быстрые, потом тишина | HSE+PLL ок, ждать SPI |
| пачки по **1** | HSE не стартанул |
| пачки по **3** | PLL не lock |
| пачки по **4** | не переключились на PLL |
| ~8 Гц после `spi_loopback.py` | до STM доходят клоки |

Проверка с Pi (после стопа робота):

```bash
sudo systemctl stop robot.service
cd ~/pupperv3-monorepo/tools
sudo python3 spi_loopback.py 0 0 1000000
sudo python3 spi_loopback.py 0 1 1000000
```

Вечерний `spi_loopback` на актуальной HSE-прошивке **не записан** — выключают до прогона.

---

## Железо / не забыть

- LED control board = **PC5**, active HIGH. Другого штатного LED нет.
- Кварц SPIne: **8 МГц** на PH0/PH1. Оригинал Cube: HSE → PLL 180 МГц + overdrive. Тест упрощён до 168 МГц без overdrive.
- Если HSE мёртвый — оригинал `SPIneV1*.elf` тоже не встанет (он с HSE). HSI-мигалка это уже подтверждала: MCU жива без кварца.
- Обе STM all_zero днём: не путать с «прошит echo на одну плату». CE0 и CE1 проверять раздельно; вторая плата в high-Z или выкл, иначе MISO дерутся.
- Карта: не клонировать диск 1:1 на меньшую SD. Piclone / pishrink / карта ≥ исходной.

---

## Файлы, которые должны лежать в репо на Pi

- `tools/stm32duino_spi_rx_led/stm32_spi_rx_led.ino` — актуальный тест (залит вечером)
- `drive_spinev1/SPIneV1_spi_timeout_led.elf` — откат SPIne
- этот лог

Откат ног: прошить `SPIneV1_spi_timeout_led.elf`, BOOT0=LOW, power-cycle. На плате сейчас, скорее всего, **Arduino echo/HSE**, не SPIne.
