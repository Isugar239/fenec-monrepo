# Лог отладки: SPI loopback на новой Raspberry Pi

Дата: 2026-06-25

Продолжение: [2026-06-24_spi_debug.md](2026-06-24_spi_debug.md)

## Контекст
Взята совершенно новая Raspberry Pi. При замкнутых MOSI↔MISO `spi_watch` по-прежнему показывает нули — возникло сомнение, работает ли SPI на малине вообще.

## Почему spi_watch вводит в заблуждение при loopback
`spi_watch` шлёт spine-пакет, где почти всё заполнено нулями (`memset(cmd_u32, 0, ...)`, только `flags=1` в словах 30–31). При замкнутых MOSI↔MISO RX **должен** повторять TX → нули в ответе — это нормально, а не признак поломки SPI.

Для loopback-теста нужен **ненулевой паттерн** (как в `spi_test.c`: 0xA0, 0xA1, …).

## Что сделали
1. Добавлен минимальный Python-скрипт `tools/spi_loopback.py`:
   - шлёт 16 байт `0xA0..0xAF` через `spidev`;
   - сравнивает TX и RX, печатает `OK` или `FAIL` с подсказкой (все нули / все 0xFF).
2. Прогнаны тесты на текущей Pi с замкнутыми MOSI↔MISO.

### Команды
```bash
sudo python3 tools/spi_loopback.py              # CE0, 1 МГц
sudo python3 tools/spi_loopback.py 0 1         # CE1
sudo python3 tools/spi_loopback.py 0 0 6000000 # CE0, 6 МГц (как у робота)

# C-утилита для сравнения:
cd ~/pupperv3-monorepo/tools && sudo ./spi_test /dev/spidev0.0 1000000
```

## Результаты тестов (2026-06-25, MOSI↔MISO замкнуты)
| Устройство       | Скорость | Результат |
|------------------|----------|-----------|
| `/dev/spidev0.0` | 1 МГц    | **OK** — RX == TX (`0xa0..0xaf`) |
| `/dev/spidev0.1` | 1 МГц    | **OK** — RX == TX (`0xa0..0xaf`) |
| `/dev/spidev0.0` | 1 МГц (C `spi_test`) | **OK** — `RX first 16 words: 0x00a0 0x00a1 ...` |

Оба chip-select (CE0 и CE1) на новой Pi работают. SPI-контроллер малины **исправен**.

## Обновлённый вывод
| Компонент            | Статус |
|----------------------|--------|
| SPI на Pi (loopback) | **OK** — CE0 и CE1 |
| `spi_watch` + loopback | нули в RX — ожидаемо (TX почти нулевой) |
| Ответ STM по MISO    | не проверялось в этой сессии (loopback без STM) |
| Joint States / моторы | без изменений (см. лог 2026-06-24) |

Гипотеза из предыдущего лога **подтверждена**: Pi шлёт и читает SPI корректно. Проблема с лапами — на стороне STM / проводки / CE-mapping, а не в драйвере Raspberry Pi.

## Следующие шаги
1. Убрать loopback, подключить STM, `sudo systemctl stop robot.service`.
2. На обоих CE по очереди: `sudo python3 tools/spi_loopback.py 0 0` и `0 1` — смотреть, отвечает ли прошитая правая STM (не нули и не 0xFF).
3. UART тестовой прошивки (`stm32_spi_slave.ino`, 115200) — растёт ли `spi_count` при обмене с Pi.
4. Осциллограф: CE (pin24/26), SCK, MOSI, MISO на PA4–PA7 правой STM.
5. Проверить BOOT0 = LOW, power-cycle STM.

## Файлы, затронутые в этой сессии
- `tools/spi_loopback.py` (новый)
- `ailogs/2026-06-25_spi_loopback.md` (этот лог)
