# Лог: перед power-off — замена DD4 (MAX3051) на CAN2

Дата: **2026-08-23**  
Предыдущие: [2026-08-23_can2_fixed_tx_jumper.md](2026-08-23_can2_fixed_tx_jumper.md), [2026-08-23_before_tx_jumper_poweroff.md](2026-08-23_before_tx_jumper_poweroff.md)

---

## Итоговая история CAN2

| Шаг | Результат |
|-----|-----------|
| Swap DAP | мёртвость не переехала → плата |
| R2 120Ω | не помог |
| Перепайка TX ×2 | probe OK, wiggle FAIL |
| Jumper PB13→DD4 pin1 | boot: лapa дёрнулась; wiggle: звук enable, fb FAIL |
| Wiggle обе ноги (не упираются) | CAN1 ✅ / CAN2 ❌ |

---

## Последний probe

```
leg0 CAN1: abad=0.122 hip=-0.299 knee=0.127  ✅
leg1 CAN2: abad=-0.020 hip=0.054  knee=-0.107  ✅ (RX жив)
```

## Последний wiggle (обе ноги свободны)

**CAN1 левая:**
- abad span=0.121 OK
- hip span=0.211 OK
- knee span=0.138 OK
- физически: двигается (дёргано — норма для теста по одному суставу)

**CAN2 правая:**
- abad/hip/knee span≈0 FAIL
- fb застрял, cmd меняется
- **звук enable в начале** — как у рабочей лапы
- физически: **не едет** (только жжж в начале)

---

## Диагноз

- **Дорожка TX бита** (jumper частично помог)
- **DD4 TX-выход** скорее всего дохнет → enable проходит, position-команды нет
- **Jumper оставить** (дорожка бита) + **заменить DD4**

---

## Что паять

**MAX3051EKA+T** (8 pin) — **DD4** (CAN2, шелk CAN1 на spine)

| Pin | Куда |
|-----|------|
| 1 | TX ← PB13 (jumper уже есть) |
| 4 | RX → PB12 |
| 6 | CANL |
| 7 | CANH |
| 8 (RS) | GND |
| 3 | 3.3V |

**R2 120Ω** — не трогать.

---

## После включения

```bash
sudo poweroff   # перед пайкой
# ... пайка DD4 ...
# включить

cd ~/pupperv3-monorepo/tools
sudo systemctl stop robot.service
sudo python3 spine_spi_test.py probe -v
sudo python3 spine_spi_test.py wiggle-can2-all --seconds 3 --amplitude 0.12 --hz 0.4
sudo python3 spine_spi_test.py wiggle-can1-all --seconds 3 --amplitude 0.12 --hz 0.4
```

**Успех:** CAN2 span > 0.03 на всех суставах, fb следует cmd, **лаpa едет сама** (не только жжж).

Потом: `sudo systemctl start robot.service`

---

## SSH

Power-off → SSH отвалится — нормально.
