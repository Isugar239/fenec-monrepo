# Лог: homing из терминала срывает моторы; boot ROS — нет

Дата: **2026-09-16** (~18:25 MSK)  
Хост: `pupper` (`pi@10.2.0.217`)  
Предыдущие: [2026-09-16_motor_ids_swapped.md](2026-09-16_motor_ids_swapped.md)

`robot.service` **стоп**. Пользователь: хоминг **при старте Pi нормальный**; `spine_spi_test.py homing` / `homing-full` — моторы сходят с ума, потом половина не отвечает, хотя «блокируются».

---

## Главное

| Что | Это |
|-----|-----|
| Homing при boot Pi | `robot.service` → `do_homing()`: `q_des` = текущая позиция, упор, `post_homing`. **Не запускать скриптом.** |
| `python spine_spi_test.py homing` | **Не тот homing.** Слал `q_des_knee=0` + `kp=5.5` + `qd=0.3` на **обе** STM. Моторы рвались к нулю. |
| `homing-full` | Копия ROS-логики, но без отпускания в конце + ложный Homed на мёртвом fb. |

После скрипта драйверы держат последнюю PD-команду → суставы **зажаты**, на `wiggle` часть не едет.

CAN id 1/2/3 всё ещё не те суставы — [2026-09-16_motor_ids_swapped.md](2026-09-16_motor_ids_swapped.md). Скрипт гонит «knee» не туда.

---

## Что было в терминале

### `homing` (3 с)

До: CE0 `hip=-4.92 knee=-1.90`, CE1 `abad=10.03` (мусор).  
Команда только на knee, но уехало:

- board0 leg0 hip Δ=0.66
- board0 leg0 knee Δ=1.90

`q_des=0` при kp=5.5 с позиции −1.9 ≈ 10 Н·м к нулю. Не ROS.

### `homing-full` (6.1 с, «Finished»)

Сразу `homed 6/12` — задние помечены готовыми (`board0_only`). Потом:

```
Homed leg_front_l_3  pos=-2.08
Homed leg_front_r_3  pos= 2.19
Homed leg_front_l_2  pos=-2.34
Homed leg_front_r_2  pos=-3.53
Homed leg_front_l_1  pos= 0.00  zero=-1.22   ← fb всегда 0, ложный упор
Homed leg_front_r_1  pos=-2.48
```

После: hip до −2.8 / −3.1, knee −2.03 / +2.17. CE1 `all_zero`.

### `wiggle-6` после этого

| Сустав | fb | Итог |
|--------|-----|------|
| Л abad | 0.000 всегда | нет движения |
| Л hip | −2.86…−3.14, span 1.85 | едет слишком широко |
| Л knee | span 0.18 | ок |
| П abad | span 0.50 | ок |
| П hip | −3.163 мёртвый | нет |
| П knee | 2.168 мёртвый | нет |

Половина молчит / зажата. Левый abad так и не ожил.

---

## Почему boot другой

ROS `do_homing` берёт `hw_command_positions_ = hw_state_positions_`, не ноль. Лимиты/флаги каналов из launch. После homing цикл `read`/`write` продолжает слать осмысленный setpoint. Скрипт выходил с `flags=1` и последним PD — STM после таймаута SPI больше не шлёт CAN, драйвер держит последнее.

---

## Что делать сейчас

1. **Силовой цикл моторов** (выкл/вкл питание ног), не только reboot Pi.
2. Не запускать `homing` / `homing-full` из скрипта, пока id не ясны.
3. Штатный homing: `sudo systemctl start robot.service`.
4. Проверка после питания: `wiggle-6` или `motor-position`, не homing.

В `tools/spine_spi_test.py` (этот день): `homing` больше не шлёт момент (exit 2 + текст). `homing-full` в `finally` отпускает `flags=0`; ложный упор без движения fb — SKIP, не Homed.

---

## 18:40 — левая нога не работает

`robot.service` уже был **active с 18:06**, пока гоняли `homing-full` / `wiggle-6`. Два хозяина SPI.

CE0 **leg0 = передняя левая (CAN1 в софте)**:

| Сустав | id | fb | |
|--------|-----|-----|---|
| abad | 1 | **всегда 0.000** | нет энкодера, wiggle span=0 |
| hip | 2 | −1.8 … −6.4 | живой, wiggle OK |
| knee | 3 | ненулевой | живой |

`SKIP leg_front_l_1` два раза: CAN id=1 на левой шине не отвечает. Без abad нога как целое не стоит.

Правая (`leg1`) abad живой (~−1.3…−1.7) — это не «обе ноги мертвы».

---

## 18:46 — ноги крутит не в ту сторону

`homing-full` ещё несколько раз. Знаки `homing_velocity` для имён abad/hip/knee, на шине другие id → сустав едет к чужому упору.

Левый hip сырой уже **−6 рад** (оборот+), это не рабочий ноль. С такого угла «правильный» знак в софте выглядит как поворот не туда.

`homing` и `homing-full` в скрипте оба **отказ** (exit 2). Направление проверять `motor-position --relative --position 0.15` по одному id.


