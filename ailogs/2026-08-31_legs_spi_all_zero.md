# Лог: повторный тест лап — STM молчит (CE0 all_zero)

Дата: **2026-08-31**  
Предыдущие (23-е): [2026-08-23_can2_fixed_tx_jumper.md](2026-08-23_can2_fixed_tx_jumper.md), [2026-08-23_before_dd4_replace_poweroff.md](2026-08-23_before_dd4_replace_poweroff.md)  
Сырьё: [raw/2026-08-31/](raw/2026-08-31/)  
Параллельно: [2026-08-31_sd_dying_boot_slow.md](2026-08-31_sd_dying_boot_slow.md)

---

## Что было 23 августа (конспект)

| Шаг | CAN1 leg0 (левая) | CAN2 leg1 (правая) |
|-----|-------------------|---------------------|
| Swap DAP | жив | мёртвость не переехала → плата |
| R2 120Ω | — | не помог |
| Перепайка TX ×2 | probe+wiggle OK | probe OK, wiggle span=0 |
| Jumper PB13 → DD4 pin1 | OK | boot: лапа дёрнулась; wiggle позже снова FAIL |
| План на выключение | — | **менять DD4 (MAX3051)** + jumper оставить |

Последний живой probe 23-го:

```
leg0 CAN1: abad=0.122 hip=-0.299 knee=0.127  ✅
leg1 CAN2: abad=-0.020 hip=0.054  knee=-0.107  ✅ RX жив
```

Wiggle при свободных ногах: CAN1 span OK, CAN2 span≈0 (звук enable, fb не следует).

---

## Тест 31 августа

`robot.service` остановлен, те же команды что 23-го.

### probe / warmup (20 enable на CE0)

```
CE0 /dev/spidev0.0  all_zero  (MISO нули)
CE1 /dev/spidev0.1  all_zero  (как и раньше — вторая STM)
```

Wiggle CAN1 и CAN2 **не запускались**: `CE0 не отвечает.`

### До остановки robot.service

SPI крутился (~691000 read), но:

```
No joint state updates detected for 690000 consecutive reads
```

STM не отдавала суставы уже **до** теста.

### I2C-1

Пустая шина (нет `0x48` батареи и ничего другого).

---

## Вывод

Лапы сегодня **не проверить**. Это не откат CAN2 и не «битая ОС».

CE0 23-го отвечал `checksum_OK` с реальными углами. Сейчас MISO=0 — **STM #1 не тактируется/не питается**, либо нет питания на control board. Пока STM молчит, jumper/DD4 не отличить.

Похоже: **не включено питание платы робота** (или оно просело вместе с 5V Pi: EXT5V скачет 4.82–5.17 V).

---

## Когда плата будет под питанием

Ноги свободны, **не двигать руками** во время probe.

```bash
cd ~/pupperv3-monorepo/tools
sudo systemctl stop robot.service
sudo python3 spine_spi_test.py probe -v
sudo python3 spine_spi_test.py wiggle-can2-all --seconds 3 --amplitude 0.12 --hz 0.4
sudo python3 spine_spi_test.py wiggle-can1-all --seconds 3 --amplitude 0.12 --hz 0.4
sudo systemctl start robot.service
```

Успех (после замены DD4 / если jumper жив): CAN2 span > 0.03, fb следует cmd, нога едет сама.

`robot.service` после сегодняшнего теста снова **active**.
