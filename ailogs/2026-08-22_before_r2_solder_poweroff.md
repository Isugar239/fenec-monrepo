# Лог: перед power-off — пайка R2 (120 Ω) на мёртвый CAN2

Дата: **2026-08-22**  
Хост: `pupper`  
Предыдущие: [2026-08-21_can1_ok_can2_dead_motor_swap.md](2026-08-21_can1_ok_can2_dead_motor_swap.md), [2026-08-22_can_swap_result_board_fault.md](2026-08-22_can_swap_result_board_fault.md)

---

## Состояние ПЕРЕД выключением

| Параметр | Значение |
|----------|----------|
| `robot.service` | **inactive** (остановлен для тестов) |
| SPI CE0 | ✅ `checksum_OK`, STM отвечает |
| CE0 leg0 (CAN1 софт / шелк CAN2) | ✅ abad≈0.126 hip≈0.263 knee≈0.009, wiggle OK |
| CE0 leg1 (CAN2 софт / шелk CAN1) | ❌ все нули, wiggle span=0 |
| CE1 `/dev/spidev0.1` | ❌ `all_zero` (вторая STM, отдельная проблема) |
| Swap DAP | ✅ сделан 2026-08-22 — мёртвость **не переехала** → проблема на плате |

---

## Диагностика по E3.pdf

- Схема: плата распределения CAN, трансиверы **MAX3051** (DD2=CAN1, DD4=CAN2)
- **R1 (120 Ω)** — CAN1, **стоит** (рабочий контур)
- **R2 (120 Ω)** — CAN2, **НЕ ЗАПАЯН** (мёртвый контур)
- Пользователь идёт паять **120 Ω на место R2**

⚠️ Шелк на spine перепутан: мёртвый = шелк **CAN1** = CAN2 в софте = leg1 = front right.

---

## Что делать ПОСЛЕ включения и пайки R2

1. **Не** запускать `robot.service` сразу
2. Мультиметр (опционально): на мёртвом разъёме H−L ≈ **60 Ω** (power off, DAP подключены)
3. Повторить тесты:

```bash
cd ~/pupperv3-monorepo/tools
sudo systemctl stop robot.service
sudo python3 spine_spi_test.py probe -v
sudo python3 spine_spi_test.py wiggle-can1-all --seconds 1 --amplitude 0.10 --hz 0.4
sudo python3 spine_spi_test.py wiggle-can2-all --seconds 1 --amplitude 0.10 --hz 0.4
```

4. Интерпретация:

| Результат | Вывод |
|-----------|-------|
| leg1 ожил, wiggle-can2 span > 0.03 | ✅ **R2 был причиной** (нет терминации) |
| leg1 всё ещё нули | R2 не помог → трансивер DD4 / дорожки / пайка |

5. После диагностики: `sudo systemctl start robot.service`

---

## Сырые данные последнего probe (2026-08-22)

```
[/dev/spidev0.0] CE0 board0 legs[0,1]: checksum_OK
    leg0: abad= 0.1261 hip= 0.2630 knee= 0.0086
    leg1: abad= 0.0000 hip= 0.0000 knee= 0.0000
```

## Сырые данные wiggle после swap DAP

| Контур | abad | hip | knee |
|--------|------|-----|------|
| CAN1 leg0 | 0.252 OK | 0.526 OK | 0.017 слабо |
| CAN2 leg1 | 0.000 | 0.000 | 0.000 |
