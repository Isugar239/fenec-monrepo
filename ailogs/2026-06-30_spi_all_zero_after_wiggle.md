# Лог: после wiggle оба SPI CE дают all-zero, robot.service снят

Дата: 2026-06-30, ~19:00 UTC+3

Продолжение: [2026-06-28_stm1_ok_stm2_cs.md](2026-06-28_stm1_ok_stm2_cs.md), [2026-06-26_spi_echo_firmware.md](2026-06-26_spi_echo_firmware.md), [2026-06-26_stm_original_bno_off.md](2026-06-26_stm_original_bno_off.md), [2026-06-25_spi_loopback.md](2026-06-25_spi_loopback.md), [2026-06-24_spi_debug.md](2026-06-24_spi_debug.md)

## Краткая суть

После предыдущего состояния, где **CE0 / STM #1 иногда отвечала `checksum_OK`** и `homing-full --board0-only` частично/иногда проходил, во время теста `wiggle-front-left` CE0 была потеряна. После power-cycle пользователь сообщил: **не помогло**.

На момент сохранения лога:

| Компонент | Статус |
|-----------|--------|
| `robot.service` | `inactive`, unit сейчас `not-found` после `disable --now` |
| ROS / robot процессы | не висят (`robot.sh`, `ros2_control`, `controller_manager`, `neural_controller` не найдены) |
| CE0 `/dev/spidev0.0` | `ALL_ZERO`, RX все `0x00` |
| CE1 `/dev/spidev0.1` | `ALL_ZERO`, RX все `0x00` |
| GPIO9 / MISO | `SPI0_MISO`, уровень `lo` |
| SPI devices | `/dev/spidev0.0`, `/dev/spidev0.1`, `/dev/spidev10.0` существуют |

Вывод: это уже не конфликт с ROS-сервисом. Сейчас MISO физически сидит в нуле или control board / кабель / STM не драйвит ответ.

## Что было в консоли перед этим

В терминале до этой сессии:

```text
python spine_spi_test.py wiggle-front-left
...
Колено: центр=-2.219 rad, амплитуда=±0.120, 0.40 Hz, 15s
  t= 0.0s  cmd=-2.219  knee=-2.219
  ...
  t= 3.4s  cmd=-2.133  knee=-2.159
  t= 3.9s  cmd=-2.253  knee=-0.000
  CE0 потерян.
```

После этого повторы `wiggle-front-left` давали:

```text
/dev/spidev0.0: CE0 не отвечает.
```

А `spine_spi_test.py read`:

```text
[/dev/spidev0.0] CE0 board0 legs[0,1]: all_zero
[/dev/spidev0.1] CE1 board1 legs[2,3]: all_zero
```

## Проверено в этой сессии

### 1. Короткий loopback-watch

Команды:

```bash
cd ~/pupperv3-monorepo/tools
timeout 3s sudo python3 spi_loopback_watch.py --ce 0 --speed 1000000 --interval 0.1
timeout 3s sudo python3 spi_loopback_watch.py --ce 1 --speed 1000000 --interval 0.1
```

Результат:

```text
/dev/spidev0.0 ALL_ZERO  rx=00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
/dev/spidev0.0: iter=30  WIRE_OK=0  SLAVE_ECHO=0

/dev/spidev0.1 ALL_ZERO  rx=00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
/dev/spidev0.1: iter=30  WIRE_OK=0  SLAVE_ECHO=0
```

### 2. Разовый `spi_loopback.py`

Команды:

```bash
cd ~/pupperv3-monorepo/tools
sudo python3 spi_loopback.py 0 0 1000000
sudo python3 spi_loopback.py 0 1 1000000
```

Результат на обоих CE:

```text
TX: ['0xa0', '0xa1', ... '0xb0']
RX: ['0x0', '0x0', ... '0x0']
FAIL — 17/17 байт не совпали (ни wire, ни slave echo)
  (все нули — MISO не подключён / slave молчит)
```

### 3. Состояние `robot.service`

Пользователь сказал, что `sudo systemctl disable --now robot.service` "не отключается". После проверки:

```text
systemctl is-active robot.service
inactive

systemctl list-units --all --no-pager "robot.service"
● robot.service not-found inactive dead robot.service

systemctl cat robot.service
No files found for robot.service.
```

Интерпретация: команда фактически остановила и сняла установленный unit. Systemd держит stale запись `not-found`, но сервис не активен и не мешает SPI.

Исходник unit в репо:

```text
/home/pi/pupperv3-monorepo/robot/utils/robot.service
```

Содержимое:

```ini
[Unit]
Description=robot

[Service]
User=pi
WorkingDirectory=/home/pi
ExecStart=/home/pi/pupperv3-monorepo/robot/utils/robot.sh
CapabilityBoundingSet=CAP_SYS_NICE
LimitRTPRIO=infinity
LimitCPU=infinity

[Install]
WantedBy=multi-user.target
```

Команды после рестарта, если нужно убрать stale state:

```bash
sudo systemctl reset-failed robot.service
sudo systemctl daemon-reload
```

Вернуть автозапуск:

```bash
sudo systemctl enable --now /home/pi/pupperv3-monorepo/robot/utils/robot.service
```

### 4. Процессы / devices / pinctrl

Процессы робота не найдены:

```text
pgrep -af "ros2_control|robot.sh|controller_manager|neural_controller"
# только сама shell-команда pgrep, реальных robot/ROS процессов нет
```

SPI devices:

```text
crw-rw---- 1 root spi ... /dev/spidev0.0
crw-rw---- 1 root spi ... /dev/spidev0.1
crw-rw---- 1 root spi ... /dev/spidev10.0
```

Пины:

```text
 7: op dh pu | hi // GPIO7 = output
 8: op dh pu | hi // GPIO8 = output
 9: a0    pn | lo // GPIO9 = SPI0_MISO
10: a0    pn | hi // GPIO10 = SPI0_MOSI
11: a0    pn | lo // GPIO11 = SPI0_SCLK
```

Важно: GPIO9 / MISO в покое `lo`, что согласуется с RX `0x00`.

## Переписка / решения этого захода

1. Пользователь: "не помогло" после power-cycle.
2. Проверили: `robot.service` был снова активен после рестарта и мог мешать SPI.
3. Остановили сервис, повторили тест: даже без сервиса оба CE дают `all_zero`.
4. Пользователь: "по короткому тесту там нули фулл".
5. Подтверждено: `spi_loopback_watch` показывает полный `ALL_ZERO` на CE0 и CE1.
6. Разобрали `disable --now`: сейчас `robot.service` уже inactive/not-found; исходный unit остался в `robot/utils/robot.service`.
7. Финальный вывод перед рестартом: проблема не в ROS-сервисе. Следующий тест должен отделить Pi/header от control boards.

## Следующий решающий тест после рестарта

### A. Проверка Pi/header без STM

1. Остановить сервис, если вдруг вернулся:

```bash
sudo systemctl stop robot.service 2>/dev/null || true
```

2. **Физически отключить control boards от SPI**.
3. На 40-pin header Raspberry Pi замкнуть:

```text
MOSI pin 19 <-> MISO pin 21
```

4. Запустить:

```bash
cd ~/pupperv3-monorepo/tools
sudo python3 spi_loopback.py 0 0 1000000
sudo python3 spi_loopback.py 0 1 1000000
```

Ожидание:

```text
OK — проводной loopback: RX == TX
```

Интерпретация:

| Результат | Значение |
|-----------|----------|
| `WIRE_OK` на Pi loopback | Pi SPI/MISO живы; MISO в ноль тянет кабель/STM/control board |
| всё равно `ALL_ZERO` | проблема на стороне Pi/header/MISO/overlay/замыкания |

### B. Если Pi loopback OK

Подключать по одной control board и проверять:

```bash
sudo python3 spi_loopback_watch.py --ce 0 --speed 1000000 --interval 0.1
sudo python3 spi_loopback_watch.py --ce 1 --speed 1000000 --interval 0.1
```

Осциллографом смотреть у конкретной STM:

| Сигнал | Pi | STM32 |
|--------|----|-------|
| CE0 | pin 24 | PA4 / NSS |
| CE1 | pin 26 | PA4 / NSS второй платы |
| SCK | pin 23 | PA5 |
| MISO | pin 21 | PA6 |
| MOSI | pin 19 | PA7 |

Ключевой вопрос: MISO (`PA6` / Pi pin21) тянется в 0 control board или уже на Pi/header.

