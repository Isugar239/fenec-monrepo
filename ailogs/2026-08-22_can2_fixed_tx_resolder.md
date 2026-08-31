# Лог: CAN2 ОЖИЛ после перепайки TX

Дата: **2026-08-22**, ~20:52 UTC+3  
Предыдущий: [2026-08-22_before_txrx_continuity_check_poweroff.md](2026-08-22_before_txrx_continuity_check_poweroff.md)

---

## Что сделали

- Прозвонка TX/RX: **звонится**
- Обнаружен **плохой контакт TX** — пользователь **перепаяла**
- Включили, повторили probe

---

## probe ПОСЛЕ перепайки TX ✅

```
[/dev/spidev0.0] CE0 board0 legs[0,1]: checksum_OK
    leg0: abad= 0.1219 hip=-0.2897 knee= 0.1265  ✅ CAN1
    leg1: abad=-0.0170 hip= 0.0765 knee=-0.0280  ✅ CAN2 — БЫЛО 0.000!
```

**leg1 впервые отдаёт реальные углы** — CAN2 (шелk CAN1, правая нога) **ожил**.

---

## Причина (подтверждено)

**Плохой контакт / пайка TX** (STM PB13 → MAX3051 DD4 pin 1).

Не моторы, не R2 (хотя R2 тоже нужен), не «мёртвый трансивер» — **холодная/плохая пайка TX**.

---

## Следующий шаг

Wiggle-can2 для проверки движения + fb span:

```bash
cd ~/pupperv3-monorepo/tools
sudo systemctl stop robot.service
sudo python3 spine_spi_test.py wiggle-can2-all --seconds 2 --amplitude 0.12 --hz 0.4
sudo python3 spine_spi_test.py wiggle-can1-all --seconds 2 --amplitude 0.12 --hz 0.4
```

Если обе ноги двигаются — `sudo systemctl start robot.service`

---

## CE1

`/dev/spidev0.1` по-прежнему `all_zero` — вторая STM, отдельная проблема.
