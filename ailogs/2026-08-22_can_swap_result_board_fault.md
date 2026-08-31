# Лог: после перестановки DAP — мёртвый контур не переехал

Дата: **2026-08-22**  
Хост: `pupper` (Raspberry Pi)  
Предыдущий тест: [2026-08-21_can1_ok_can2_dead_motor_swap.md](2026-08-21_can1_ok_can2_dead_motor_swap.md)

---

## Что сделали

Пользователь **поменял DAP местами** между двумя CAN-контурами (после power-off).  
Повторены probe + wiggle CAN1/CAN2.

⚠️ Шелкография CAN на плате перепутана — см. [HARDWARE_CAN_SILKSCREEN_SWAP.md](HARDWARE_CAN_SILKSCREEN_SWAP.md).

---

## probe (2026-08-22)

```
[/dev/spidev0.0] CE0 board0 legs[0,1]: checksum_OK
    leg0: abad= 0.1261 hip= 0.2630 knee= 0.0086  qd=(-0.016,-0.016,-0.016) flags=0
    leg1: abad= 0.0000 hip= 0.0000 knee= 0.0000  qd=( 0.000, 0.000, 0.000) flags=0

[/dev/spidev0.1] CE1 board1 legs[2,3]: all_zero

Валидный ответ: 1/2 плат.
```

**Сравнение с 2026-08-21:** leg0 жив, leg1 нули — **картина та же**.

---

## wiggle после swap

| Контур (софт) | Шелк PCB | Сустав | span (рад) | Статус |
|---------------|----------|--------|-----------|--------|
| CAN1 / leg0   | CAN2     | abad   | 0.252     | ✅ OK  |
| CAN1 / leg0   | CAN2     | hip    | 0.526     | ✅ OK  |
| CAN1 / leg0   | CAN2     | knee   | 0.017     | ⚠️ слабо |
| CAN2 / leg1   | CAN1     | abad   | 0.000     | ❌ нет движения |
| CAN2 / leg1   | CAN1     | hip    | 0.000     | ❌ нет движения |
| CAN2 / leg1   | CAN1     | knee   | 0.000     | ❌ нет движения |

---

## Вывод

**Мёртвость осталась на leg1 (CAN2 в софте = шелк CAN1 на плате).**

→ Проблема **не в DAP/моторах/кабеле**, а на **плате**:
- CAN2 линия (PB12/PB13, `hcan2`)
- трансивер CAN2
- разводка/разъём CAN2 на control board

---

## Заметки

- `robot.service`: **inactive** на время тестов
- `spine_spi_test.py wiggle-can*-all` иногда падает на «CE0 потерян» из-за чередования `all_zero`/`checksum_OK` на SPI — probe стабилен, wiggle прошёл через retry-скрипт
- CE1 (`/dev/spidev0.1`) по-прежнему `all_zero` — вторая STM не отвечает (отдельная проблема)
