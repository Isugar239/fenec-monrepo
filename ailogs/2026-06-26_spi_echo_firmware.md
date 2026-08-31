# Лог: echo-прошивка STM для SPI loopback (без UART)

Дата: 2026-06-26

Продолжение: [2026-06-26_stm_original_bno_off.md](2026-06-26_stm_original_bno_off.md), [2026-06-25_spi_loopback.md](2026-06-25_spi_loopback.md)

## Задача

Один раз прошить STM тестовой прошивкой. UART смотреть нельзя. Нужна **гарантированная** проверка SPI Pi↔STM через `spi_loopback.py`.

## Почему старая тестовая прошивка не подходила

Файл `tools/stm32duino_spi_slave/stm32_spi_slave.ino` (до правки):

| Проблема | Эффект |
|----------|--------|
| PA4 (NSS) был GPIO input, не AF5 | Hardware NSS не видит CE с Pi → SPI1 slave не активируется → **MISO молчит** |
| На MISO шёл паттерн `0xA5^i`, не echo MOSI | `spi_loopback.py` всегда FAIL даже при рабочем SPI |
| Зависимость от Serial / `while(!Serial)` | Блокировка без USB; UART всё равно недоступен |

## Новая прошивка

**Файл:** `tools/stm32duino_spi_slave/stm32_spi_slave.ino`

- **Без UART** — нет `Serial`, нет блокировок.
- **PA4 = AF5** (hardware NSS) + PA5/PA6/PA7 = AF5, OSPEED high.
- **SPI1 slave, mode 0**, SSM=0 (CE с Pi на pin24/pin26).
- **Byte echo:** в `SPI1_IRQHandler` при RXNE: `DR = прочитанный байт`.
- **Preload DR = 0xA0** — совпадает с TX[0] в тесте.

### Как работает echo (важно для интерпретации)

SPI slave в full duplex отдаёт на MISO байт, **загруженный в DR до начала** текущего 8-битного кадра. Поэтому:

- RX на Pi[0] = 0xA0 (preload)
- RX на Pi[i] = TX[i−1] для i = 1..16

`spi_loopback.py` шлёт **17 байт** `0xA0..0xB0` и принимает успех если:
- `RX == TX` — проводной loopback на Pi (MOSI↔MISO замкнуты), или
- `RX[0]==TX[0]` и `RX[1:]==TX[:-1]` — **STM32 echo OK**.

## Прошивка (один раз)

1. Подключить **ту STM, которую ещё можно прошить** (правая, если у левой сломан разъём).
2. Arduino IDE / STM32Duino: плата **STM32F446** (как на control board Pupper V3).
3. Открыть `tools/stm32duino_spi_slave/stm32_spi_slave.ino` → Upload.
4. **BOOT0 = LOW**, power-cycle после прошивки.
5. Вторая STM: **отключить** от шины (или оставить — но MISO общий; лучше одна плата для чистого теста).

## Проверка с Pi (без UART)

```bash
sudo systemctl stop robot.service

# 1 МГц — надёжнее для первого теста
sudo python3 /home/pi/pupperv3-monorepo/tools/spi_loopback.py 0 0 1000000
sudo python3 /home/pi/pupperv3-monorepo/tools/spi_loopback.py 0 1 1000000

# 6 МГц — как у робота (после успеха на 1 МГц)
sudo python3 /home/pi/pupperv3-monorepo/tools/spi_loopback.py 0 0 6000000
sudo python3 /home/pi/pupperv3-monorepo/tools/spi_loopback.py 0 1 6000000
```

### Критерии

| Результат | Значение |
|-----------|----------|
| `OK — STM32 SPI echo` на CE0 | Pi↔STM (ноги 0–1) SPI **работает** |
| `OK — STM32 SPI echo` на CE1 | Pi↔STM (ноги 2–3) SPI **работает** |
| `FAIL`, все `0xFF` | MISO не драйвится — NSS/CE, питание, BOOT0, обрыв, не та плата на CE |
| `FAIL`, все `0x00` | Другой паттерн шины (см. лог 2026-06-26) |
| `OK — проводной loopback` при подключённой STM | Тест бесполезен — замкнут MOSI↔MISO на Pi, не STM |

### Какой CE смотреть

| CE | `/dev/spidev` | Ноги |
|----|---------------|------|
| CE0 | `spidev0.0` | 0, 1 |
| CE1 | `spidev0.1` | 2, 3 |

Прошитая плата должна быть на том CE, который тестируете. Если прошили «правую» STM — она обычно на CE1 (ноги 2–3); уточните по разводке кабеля.

## Что делать после теста

| Исход | Действие |
|-------|----------|
| **OK на одном CE** | SPI до этой STM исправен. Вернуть **оригинальную SPIneV1** на обе платы (если нужен робот) или оставить echo на одной для дальнейшей отладки второй. |
| **FAIL на обоих CE** | Проблема **ниже софта**: CE не доходит до PA4, питание STM, BOOT0, MISO обрыв, неправильная плата. Осциллограф: CE (pin24), PA4 (NSS), SCK PA5, MOSI PA7, MISO PA6. |
| **OK на echo, FAIL на orig SPIne** | Железо OK; баг в штатной прошивке или неверный формат пакета spine — смотреть `spine_spi_test.py`. |

## Изменённые файлы

- `tools/stm32duino_spi_slave/stm32_spi_slave.ino` — переписан: echo, PA4 AF5, без UART
- `tools/spi_loopback.py` — 17 байт, распознавание STM slave echo
- `ailogs/2026-06-26_spi_echo_firmware.md` — этот лог

## Справка: пины

| Сигнал | Pi (40-pin) | STM32 |
|--------|-------------|-------|
| CE0 | pin 24 | PA4 NSS |
| CE1 | pin 26 | PA4 NSS (вторая плата) |
| SCK | pin 23 | PA5 |
| MOSI | pin 19 | PA7 |
| MISO | pin 21 | PA6 |

SPI mode 0, 8 bit. Робот: 6 МГц, пакет spine 132 байта (`rt_spi.cpp`).

---

## Результаты прогона (конец сессии 2026-06-26)

Прошивка echo **ещё не залита на STM** (или не проверена после заливки). Прогон с Pi:

```bash
sudo python3 /home/pi/pupperv3-monorepo/tools/spi_loopback.py 0 0 1000000
sudo python3 /home/pi/pupperv3-monorepo/tools/spi_loopback.py 0 1 1000000
```

### CE0 (`/dev/spidev0.0`, 1 МГц)
```
TX: 0xa0..0xb0 (17 байт)
RX: все 0x00
FAIL — 17/17 байт не совпали (ни wire, ни slave echo)
  (все нули — MISO не подключён / slave молчит)
```

### CE1 (`/dev/spidev0.1`, 1 МГц)
```
TX: 0xa0..0xb0 (17 байт)
RX: все 0x00
FAIL — 17/17 байт не совпали (ни wire, ни slave echo)
  (все нули — MISO не подключён / slave молчит)
```

### Статус на паузу

| Что | Статус |
|-----|--------|
| Echo-прошивка в репо | готова |
| `spi_loopback.py` | обновлён |
| Прошивка на STM | **не сделана** / не подтверждена |
| SPI Pi↔STM | **FAIL** на CE0 и CE1 |

**Продолжить с:** залить `stm32_spi_slave.ino` на одну STM → `spi_loopback` на нужном CE → при OK осциллограф/вторая плата.
