# Лог: CAN ID моторов перепутаны; примеры из терминала работают

Дата: **2026-09-16** (~18:20 MSK)  
Хост: `pupper` (`pi@10.2.0.217`)  
Предыдущие: [2026-09-15_stm_hse_spi_echo.md](2026-09-15_stm_hse_spi_echo.md), [HARDWARE_CAN_SILKSCREEN_SWAP.md](HARDWARE_CAN_SILKSCREEN_SWAP.md)

`robot.service` для этих проверок: **стоп**. Моторы сначала были отключены (LED/SPI), потом гоняли с терминала.

---

## Главное

**CAN ID моторов не совпадают с именами суставов в софте**, но движение с примеров `spine_spi_test.py` **есть**.

В прошивке / скрипте заложено:

| CAN id | Имя в софте |
|--------|-------------|
| 1 | abad |
| 2 | hip |
| 3 | knee |

На железе этот порядок **не тот**. При отладке смотреть, что реально качнулось, а не подпись `abad`/`hip`/`knee`.

### Левая передняя (CAN1 / CE0 leg0), 18:49

Команда `--can 1 --motor 2` (софт: **hip**). Сначала abad дёргался вокруг −6 рад; затем **+0.08 и +0.8 —abad ехал нормально**, fb −5.76 → −4.99.

| Софт (`--motor`) | CAN id | Что видно на ноге | fb в SPI |
|------------------|--------|-------------------|----------|
| abad | 1 | нет ответа | **всегда 0.000** |
| hip | 2 | **физический левый передний abad**, едет | hip-слот ≈ −5…−6 рад |
| knee | 3 | ещё не смотрели глазами | ненулевой |

Дёрганье: слали hip ±0.15 с kp=5.5 вокруг **−5.8 рад** (уже не рабочая поза) на мотор, который на самом деле abad. Слот id=1 при этом enable с fb=0 — крутящий момент без энкодера.

Не слать `--motor 1` налево. Abad двигать только `--motor 2`, маленькое смещение. Homing не запускать.

Это **отдельно** от шелкографии CAN1/CAN2 на PCB (там перепутаны шины, не id 1/2/3). Шелк: [HARDWARE_CAN_SILKSCREEN_SWAP.md](HARDWARE_CAN_SILKSCREEN_SWAP.md).

ROS/`components.xacro` с этими id для нормальной походки **не перепроверяли**. Рабочий путь сегодня — терминальные примеры ниже.

---

## Прошивка на STM (этот день)

Залит патч `drive_spinev1/SPIneV1_spi_timeout_led.elf`: timeout SPI 50 ms, LED **PC5 мигает при успешном кадре**. Не Arduino echo.

Откат: `SPIneV1.elf`. Только timeout без LED: `SPIneV1_spi_timeout.elf`.

---

## Что с терминала работало

С Pi, моторы уже в деле (не `enable`/`homing` вслепую):

```bash
sudo systemctl stop robot.service
cd ~/pupperv3-monorepo/tools

sudo python3 spine_spi_test.py probe -v
sudo python3 spine_spi_test.py watch --count 200

sudo python3 spine_spi_test.py motor-position --can 1 --motor 1 --relative --position 0.10
sudo python3 spine_spi_test.py motor-position --can 1 --motor 2 --relative --position 0.10
sudo python3 spine_spi_test.py motor-position --can 1 --motor 3 --relative --position 0.10

sudo python3 spine_spi_test.py wiggle-can1-all --seconds 3 --amplitude 0.10 --countdown
sudo python3 spine_spi_test.py motor-console
```

`--motor 1/2/3` = CAN id, не «какой сустав нарисован на ноге». Команды доходят, моторы едут; подпись сустава в логе может врать.

---

## Не путать

| Путаница | Что это |
|----------|---------|
| Шелк CAN1 ↔ CAN2 на плате | шины, см. постоянную заметку |
| CAN id 1/2/3 моторов | этот лог: id ≠ abad/hip/knee на железе |
| `spi_loopback.py` | не для SPIne: короткий кадр, таймаут, LED гаснет |
