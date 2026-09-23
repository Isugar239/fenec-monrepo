# Лог: проверка STM — обе платы all_zero; скетч LED PC5

Дата: **2026-09-15** (~18:48–18:54 MSK)

Предыдущие: [2026-08-31_legs_spi_all_zero.md](2026-08-31_legs_spi_all_zero.md), [2026-06-28_stm1_ok_stm2_cs.md](2026-06-28_stm1_ok_stm2_cs.md)

Сырьё: [raw/2026-09-15/spi_probe.txt](raw/2026-09-15/spi_probe.txt)

---

## Главные выводы

| Компонент | Статус |
|-----------|--------|
| Raspberry Pi SPI | **жив** — `/dev/spidev0.0`, `/dev/spidev0.1`, `dtparam=spi=on` |
| STM #1 CE0 | **молчит** — MISO = все нули (`all_zero`) |
| STM #2 CE1 | **молчит** — то же `all_zero` |
| Checksum в ROS | 0 ошибок `BAD CHECKSUM` — нулевой пакет сам себе сходится (`0==0`) |
| Суставы | заморожены с момента activate (~395 000 consecutive reads к моменту проверки) |
| Homing | ложный успех за ~0.8 с: все 12 актуаторов «Homed» на нулях |
| I2C-1 | пусто (нет `0x48` батареи) |
| ST-Link | не подключён |

Это **не** `0xFF` (MISO high-Z / подтяжка вверх). Линия сидит в нуле: MCU не гоняет данные, либо MISO на GND, либо нет питания/такта STM.

Картина та же, что **31 августа**.

---

## Что было до probe

`robot.service` **active** с 18:24 (ROS поднялся ~18:36).

```
Homing actuators...
Homed actuator 2/5/8/11 … 1/4/7/10 … 0/3/6/9   (~0.8 с)
Finished homing!
[RT SPI] Open
SPI крутит TX на board=0 и board=1
No joint state updates detected for N consecutive reads   (росло без «resumed»)
SPI ERROR BAD CHECKSUM — ни разу
```

Интерпретация: Pi шлёт кадры, STM не отвечает. Драйвер принимает нули как валидный checksum и считает homing завершённым.

---

## Probe (robot.service остановлен)

```bash
sudo systemctl stop robot.service
cd ~/pupperv3-monorepo/tools
sudo python3 spine_spi_test.py probe -v
```

| Шина | Плата | Класс ответа |
|------|--------|--------------|
| `/dev/spidev0.0` CE0 | STM #1, ноги 0–1 | `all_zero` |
| `/dev/spidev0.1` CE1 | STM #2, ноги 2–3 | `all_zero` |

Все углы 0.000, flags=0, raw u32 — пятнадцать нулей.  
«Ни одна плата не ответила валидным spine-пакетом.»

После теста `robot.service` снова **active**.

---

## STM32duino: LED на PC5

На control board / SPIneV1 штатный LED — **PC5**, активный HIGH. Другого LED на MCU нет.

Обновлён скетч «MCU жива» (HSI 16 МГц, без HSE):

`tools/stm32duino_led_blink/stm32_led_blink_pc5.ino`

Поведение: 3 вспышки при старте, дальше мигание 1 Гц.

Arduino IDE:

- Плата: Generic STM32F4 series
- Board part number: Generic F446RETx (STM32F446RE)
- Upload: STM32CubeProgrammer (SWD) / ST-Link
- USB support: None
- После заливки: BOOT0 = LOW, снять ST-Link, power-cycle

Если после прошивки **не мигает** — нет 3.3 V / BOOT0=HIGH / не та плата / LED не на PC5.  
Если **мигает**, а SPI всё ещё `all_zero` — MCU жива, проблема в SPI (CS/MISO/кабель), не в «мертвом чипе».

---

## Что смотреть на железе

1. Горит/мигает ли LED PC5 (скейтч выше или `SPIneV1_spi_timeout_led`).
2. Питание control board (3.3 V, не только Pi).
3. Шлейф SPI, особенно MISO — не коротнул ли на GND.
4. Кварц / BOOT0, если LED не стартует.

Пока STM молчит, CAN/jumper/DD4 не отличить.
