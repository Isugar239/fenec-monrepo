# Лог: feedback обеих ног + SSH отваливается

Дата: **2026-08-22**, ~21:00 UTC+3

---

## Feedback после перепайки TX

### probe (статика)
```
leg0 CAN1: abad=0.122 hip=-0.292 knee=0.126  ✅
leg1 CAN2: abad=-0.020 hip=0.076  knee=-0.028  ✅ (не нули!)
```

### quick wiggle fb (cmd ±0.10 rad)
| CAN | abad | hip | knee |
|-----|------|-----|------|
| CAN1 | 0.062 OK | 0.150 OK | 0.086 OK |
| CAN2 | 0.000 FAIL | 0.000 FAIL | 0.000 FAIL |

**Вывод:** CAN2 **читает** положение (probe), но на **команды не реагирует** (span=0). TX возможно **всё ещё нестабилен** или контакт плохой под нагрузкой.

---

## Почему отваливается SSH

1. **Reboot/power-off** — сегодня несколько перезагрузок (пайка, power-off). `last` показывает `crash` ~20:32, 20:46. Uptime был ~8 мин.
2. **WiFi нестабилен** — `Tx excessive retries: 471`, signal -69 dBm, **Power Management: on**. eth0 DOWN.
3. **SD-карта** — в прошлой сессии: `mmc0: Card stuck being busy!` (может подвисать Pi).
4. **Не таймаут SSH** — ClientAlive по умолчанию выключен; disconnect = обрыв сети или reboot.

### Рекомендации
- Пайка/тесты: ожидай disconnect при power-off — это нормально
- WiFi: `sudo iwconfig wlan0 power off` или ethernet
- После пайки TX: **прогреть/проверить** контакт ещё раз — probe OK, wiggle CAN2 FAIL
