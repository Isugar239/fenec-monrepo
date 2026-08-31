# Лог: после 2-й перепайки TX — CAN2 probe OK, wiggle FAIL

Дата: **2026-08-23**  
Предыдущий: [2026-08-22_before_tx_resolder_2_poweroff.md](2026-08-22_before_tx_resolder_2_poweroff.md)

---

## Контекст

- 1-я перепайка TX: probe leg1 ожил, но wiggle не работал
- Уточнение: углы в probe были частично от **ручного** движения
- 2-я перепайка TX (PB13 → DD4 pin 1) — пользователь перепаяла снова

---

## probe после 2-й перепайки

```
leg0 CAN1: abad=0.115 hip=-0.283 knee=0.122  ✅
leg1 CAN2: abad=-0.020 hip=0.069  knee=0.084  ✅
```

---

## wiggle CAN2

| Сустав | span | fb |
|--------|------|-----|
| abad | 0.000 FAIL | застрял −0.020 |
| hip | 0.000 FAIL | застрял +0.069 |
| knee | 0.000 FAIL | застрял +0.084 |

cmd меняется, fb не следует → **TX всё ещё не работает**.

---

## Вывод

Перепайка TX **пока не помогла** (или контакт всё ещё нестабилен).

RX жив (probe читает углы — **не двигать руками** при следующем тесте!).

---

## Финальный тест (2026-08-23, прозвонка OK, 2-я перепайка TX)

Wiggle CAN2 — все FAIL (span ≈ 0), probe leg1 OK.

**Вердикт: менять DD4 (MAX3051).**

Чип: **MAX3051EKA+T** (или MAX3051ESA), 8-pin.
- Pin 1 = TX (от STM PB13)
- Pin 4 = RX (к STM PB12)
- Pin 6 = CANL, Pin 7 = CANH
- Pin 8 = RS → GND (high-speed)
- R2 = 120Ω между CANH/CANL

После замены: probe + wiggle-can2-all.
