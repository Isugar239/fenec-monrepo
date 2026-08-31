# Лог: медленный boot, сыпется microSD (не SSD)

Дата: **2026-08-31**  
Хост: `pupper` (Pi 5)  
Сырьё: [raw/2026-08-31/](raw/2026-08-31/)  
Скрипт: `tools/sd_selfcheck.sh`

---

## Суть

ОС не битая. **Сыпется microSD** Samsung `EC1S5` 64 ГБ (`mmcblk0`, не NVMe).  
Ext4 `clean`, `errors_count=0`, сломанных пакетов dpkg нет.  
Старт ~2 мин и «раз через 5» — карта зависает (`Card stuck`) + просадки 5V.

---

## Диск

| | |
|---|---|
| Root | `/dev/mmcblk0p2` ext4, 59 ГБ, 60% |
| Карта | Samsung EC1S5, manfid `0x1b`, date 12/2023 |
| NVMe | нет |
| ext4 | state=clean, last fsck 2024-11-19, lifetime writes 86 GB |

## Питание (эта загрузка)

| | |
|---|---|
| `throttled` | `0x50000` — undervoltage + throttle **были** |
| EXT5V сейчас | скачет ~4.82–5.17 V |
| `Undervoltage detected` в dmesg | 40+ раз |

Даже «небольшая» просадка для Pi 5 уже ниже порога (~4.63 V) — MMC от этого встаёт.

## I/O этой загрузки

| | |
|---|---|
| `mmc0: Card stuck being busy` | 9 |
| I/O error на `mmcblk0` (READ) | 4 (сектора в разных местах диска) |
| jbd2 blocked >120s | да |
| avg read | ~149 ms/IO (норма <15) |
| avg write | ~2.8–6 s/IO (норма <40 ms) |
| sequential 32 MiB | 25 MB/s (слабо, но не ноль) |

Сектора I/O error разбросаны (~6 / 8 / 22 / 52 ГБ на p2) — типичный дохлый NAND/контроллер, не один битый файл.

`system.journal corrupted or uncleanly shut down` — journald не дописал лог из‑за зависания карты, **не** сломанный образ ОС.

---

## Софт boot (после фикса NTP 2026-08-31)

`systemd-time-wait-sync` и `NetworkManager-wait-online` выключены.  
Эта загрузка всё равно **1 min 50 s** до graphical:

```
wayvnc.service  +1min 10.970s   (Type=notify, ждал композитор)
dbus.service    +34s            (скорее stall SD)
```

Drop-in: `/etc/systemd/system/wayvnc.service.d/no-block-boot.conf` — `TimeoutStartSec=8s`.  
С **следующего** ребута graphical не должен ждать VNC. Карту это не лечит.

---

## Вердикт

**FAIL — клонировать карту и заменить.** Переустановка на эту же SD вернёт те же зависания.

Лучше: NVMe HAT или новая A2/V30 endurance SD + БП 5.1 V / 5 A.
