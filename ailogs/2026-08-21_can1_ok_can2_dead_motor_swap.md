# Лог: CAN1 жив, CAN2 мёртв — перед перестановкой моторов

Дата: **2026-08-21**, ~18:08 UTC+3  
Хост: `pupper` (Raspberry Pi)  
Продолжение: [2026-06-30_spi_all_zero_after_wiggle.md](2026-06-30_spi_all_zero_after_wiggle.md), [2026-06-28_stm1_ok_stm2_cs.md](2026-06-28_stm1_ok_stm2_cs.md), [2026-06-24_spi_debug.md](2026-06-24_spi_debug.md)

---

## ⚠️ ВАЖНО: шелкография CAN на плате перепутана

**Зафиксировано пользователем (2026-08-21):** на PCB подписи CAN1/CAN2 **не совпадают** с тем, что в прошивке и софте.

| Шелкография на плате | Фактически (прошивка STM / `spine_spi_test.py`) | SPI leg | Статус **до** перестановки моторов |
|----------------------|--------------------------------------------------|---------|-------------------------------------|
| **CAN2** (надпись)   | **CAN1** (`hcan1`, PB8/PB9, `l1_state`)         | leg0    | ✅ **Жив** — данные, движение        |
| **CAN1** (надпись)   | **CAN2** (`hcan2`, PB12/PB13, `l2_state`)        | leg1    | ❌ **Мёртв** — нули, нет движения   |

При разговоре и в логах ниже:
- **«CAN1 в софте»** = шелк **CAN2** на плате = передняя левая (`leg_front_l`, `can_ch=1`)
- **«CAN2 в софте»** = шелк **CAN1** на плате = передняя правая (`leg_front_r`, `can_ch=2`)

---

## Краткая суть (до power-off / swap моторов)

| Компонент | Статус |
|-----------|--------|
| `robot.service` | **inactive** (остановлен для SPI-тестов) |
| CE0 `/dev/spidev0.0` | ✅ `checksum_OK`, STM отвечает |
| CE0 leg0 (CAN1 софт / шелк CAN2) | ✅ Реальные углы, wiggle OK |
| CE0 leg1 (CAN2 софт / шелk CAN1) | ❌ Все нули, wiggle без движения |
| CE1 `/dev/spidev0.1` | ❌ `all_zero` (вторая STM не отвечает) |

**Вывод до swap:** SPI + STM #1 (CE0) работают. Проблема локализована на **втором CAN-контуре** (CAN2 в софте = шелк CAN1): либо шина/трансивер/проводка, либо моторы/разъёмы на этом контуре.

**План пользователя:** выключить питание и **поменять моторы местами** между двумя CAN-контурами, чтобы понять — проблема в моторах или в CAN2-линии на плате.

---

## Команды тестов (все с `sudo systemctl stop robot.service`)

```bash
cd ~/pupperv3-monorepo/tools

sudo python3 spine_spi_test.py probe -v
sudo python3 spine_spi_test.py read --repeat 1
sudo python3 spine_spi_test.py wiggle-can1-all --seconds 1 --amplitude 0.10 --hz 0.4
sudo python3 spine_spi_test.py wiggle-can2-all --seconds 1 --amplitude 0.10 --hz 0.4
sudo python3 spine_spi_test.py wiggle-front-left --seconds 3 --amplitude 0.15 --hz 0.4
```

---

## probe (финальный снимок перед power-off)

```
=== probe: enable flags, CE0 и CE1 ===

[/dev/spidev0.0] CE0 board0 legs[0,1]: checksum_OK
    leg0: abad=-0.2596 hip=-0.0284 knee=-0.0544  qd=(-0.016,-0.016,-0.016) flags=0
    leg1: abad= 0.0000 hip= 0.0000 knee= 0.0000  qd=( 0.000, 0.000, 0.000) flags=0
    raw u32: 0xbe84e980 0x00000000 0xbce8d000 0x00000000 0xbd5ea900 0x00000000 0xbc821000 ...

[/dev/spidev0.1] CE1 board1 legs[2,3]: all_zero
    leg0: abad= 0.0000 hip= 0.0000 knee= 0.0000  qd=( 0.000, 0.000, 0.000) flags=0
    leg1: abad= 0.0000 hip= 0.0000 knee= 0.0000  qd=( 0.000, 0.000, 0.000) flags=0

Валидный ответ: 1/2 плат.
```

---

## wiggle-can1-all (CAN1 софт = шелк CAN2 = leg0, передняя левая)

**Все 3 сустава OK:**

```
--- abad (CAN1 id=1) center=0.260 ---
  итог abad: span=0.559 rad  OK

--- hip (CAN1 id=2) center=0.029 ---
  итог hip: span=0.139 rad  OK

--- knee (CAN1 id=3) center=0.055 ---
  итог knee: span=0.174 rad  OK
```

---

## wiggle-can2-all (CAN2 софт = шелk CAN1 = leg1, передняя правая)

**Нет движения, fb=0:**

```
--- abad (CAN2 id=1) center=-0.000 ---
  итог abad: span=0.000 rad  слабо/нет движения

--- hip (CAN2 id=2) center=-0.000 ---
  итог hip: span=0.000 rad  слабо/нет движения

--- knee (CAN2 id=3) center=-0.000 ---
  итог knee: span=0.000 rad  слабо/нет движения
```

---

## wiggle-front-left (ранний тест, колено CAN1)

Колено качалось, fb менялся (knee 0.084 → 0.218 rad).

---

## Маппинг прошивки (справочно)

Из `drive_spinev1/Core_unpacked/Core/Src/main.c`:

- `CAN1` / `hcan1` / PB8-PB9 → `l1_state` → `spi_data.q_*[0]` (leg0)
- `CAN2` / `hcan2` / PB12-PB13 → `l2_state` → `spi_data.q_*[1]` (leg1)

Из `tools/spine_spi_test.py` / `components.xacro`:

- `can_ch=1` → передняя левая (`leg_front_l_*`)
- `can_ch=2` → передняя правая (`leg_front_r_*`)

---

## Исправление в `tools/spine_spi_test.py`

В `_actuator_state()` была опечатка `acd` вместо `act` → `NameError` при `wiggle-can*-all`. Исправлено в рабочей копии (ещё не закоммичено).

---

## Что делать ПОСЛЕ перестановки моторов

1. Включить питание, **не** запускать `robot.service` сразу.
2. Повторить те же команды probe + wiggle-can1-all + wiggle-can2-all.
3. Интерпретация:

| Результат после swap | Вероятная причина |
|----------------------|-------------------|
| Мёртвый контур **остался тем же** (leg1 / CAN2 софт) | Проблема на плате: CAN2 линия, трансивер, STM pin PB12/PB13 |
| Мёртвость **переехала** на другой контур (leg0 / CAN1 софт) | Проблема в **моторах/кабеле/разъёме** того контура, который был на шелk CAN1 |
| Оба контура ожили | Плохой контакт / перепутанная распайка разъёмов |

4. После диагностики: `sudo systemctl start robot.service`

---

## Состояние системы на момент сохранения

- `robot.service`: **inactive**
- SPI devices: `/dev/spidev0.0`, `/dev/spidev0.1` доступны
- Пользователь собирается **выключить питание** для физической перестановки моторов

---

## Сырые дампы

Сохранены в `/tmp/` на Pi (до reboot):
- `/tmp/spi_probe_final.txt`
- `/tmp/spi_read_final.txt`
- `/tmp/wiggle_can1.txt`
- `/tmp/wiggle_can2.txt`
