# Pupper v3 – открытая платформа для роботизированного пуппи‑платформы

**Pupper v3** – это модульная робототехническая экосистема, построенная на ROS 2, предназначенная для быстрого прототипирования и исследования поведения quadruped‑роботов. Проект объединяет perception, navigation, manipulation, и high‑level behavior modules в единой, легко расширяемой структуре.

---

## Содержание
1. [Общая архитектура](#общая-архитектура)  
2. [Модули проекта](#модули-проекта)  
3. [Основные скрипты и запуск](#основные-скрипты-и-запуск)  
4. [Требования и установка](#требования-и-установка)  
5. [Запуск и тестирование](#запуск-и-тестирование)  
6. [Лицензия](#лицензия)  
7. [Контакты и вклад](#контакты-и-вклад)

---

## Общая архитектура
- **ROS 2 Workspace** (`ros2_ws/`) – центральный каталог, в котором находятся все пакеты.
- **Пакетная структура** – каждый функциональный блок (description, perception, control, simulation и т.д.) реализован как отдельный ROS 2 пакет с собственным `package.xml`, `CMakeLists.txt`/`setup.cfg` и набором узлов.
- **Скрипты-утилиты** – вспомогательные bash‑ и python‑скрипты находятся в корне и в подкаталогах `scripts/`, они упрощают сборку, запуск и диагностику.

---

## Модули проекта

| Модуль | Описание | Ключевые файлы |
|--------|----------|----------------|
| **pupper_v3_description** | URDF‑модели, RViz‑конфиги, Xacro‑шаблоны | `description/pupper_v3.urdf.xacro`, `launch/display_model.launch.py`, `launch/robot_state_publisher.launch.py` |
| **pupperv3_mujoco_sim** | Симуляция в Mujoco с кастомными аппаратными интерфейсами | `mujoco_hardware_interface.xml`, `launch/test_hw_interface.launch.py`, C++ драйверы |
| **neural_controller** | Высокоуровневый контроллер, использующий политики из JSON | `launch/launch.py`, `launch/policy_latest.json`, C++‑реализация |
| **real2sim_controller** | Мост между реальными датчиками/моторами и симуляцией | `include/real2sim_controller/real2sim_controller.hpp`, `launch/config.yaml`, `launch/launch.py` |
| **hailo** | Перцепция на базе ускорителя Hailo | `hailo_detection.py`, `hailo_inference.py`, `launch/hailo_detection_launch.py` |
| **llm_websocket_server** | Сервер для обмена данными с LLM через WebSocket | `websocket_server.py`, `launch/websocket_server_launch.py` |
| **pupper_feelings** | Модуль эмоций и выражения лица/ушей | `face_control.py`, `ear_control.py`, GUI‑компоненты |
| **cmd_vel_mux** | Множественный переключатель команд скорости | `cmd_vel_mux_node.cpp` |
| **joy_utils** | Утилиты для работы с джойстиками, экстренной остановкой | `estop_controller.cpp` |
| **bag_recorder** | Запись ROS‑bagов для последующего анализа | `bag_recorder.launch.py` |
| **real2sim_controller, imu_to_tf, etc.** | Дополнительные вспомогательные пакеты (см. дерево) | — |

---

## Основные скрипты и запуск

| Скрипт | Назначение | Как запустить |
|--------|------------|----------------|
| `robot/start_ui.sh` | Стартовый скрипт, поднимает все основные узлы в одном процессе | `./robot/start_ui.sh` (из корня репозитория) |
| `ros2_ws/build.sh` | Сборка всего workspace | `./ros2_ws/build.sh` |
| `verify_pupper_v3.sh` | Проверка наличия обязательных файлов | `./verify_pupper_v3.sh` |
| `scripts/*.py` | Разные вспомогательные скрипты (создание Mujoco‑XML, фикс URDF и т.д.) | Запуск через `python3 <script>.py` |

> **Важно:** Перед запуском убедитесь, что выполнили `colcon build` в корне workspace и source‑нули `install/setup.bash`.

---

## Требования и установка

1. **Ubuntu 22.04** (или другая поддерживаемая версия с ROS 2 Humble).  
2. **ROS 2 Humble** – `sudo apt install ros-humble-desktop`.  
3. **Mujoco 2.3** – скачайте и установите из официального репозитория.  
4. **Python 3.10+** и `pip` – для установки пакетов в каждом ROS‑пакете (`pip install -e .`).  
5. **Hailo SDK** – при необходимости установите согласно инструкциям поставщика.

```bash
# Клонирование репозитория
git clone https://github.com/your-org/pupperv3-monorepo.git
cd pupperv3-monorepo
# Сборка
colcon build
# Источник
source install/setup.bash
# Проверка
./verify_pupper_v3.sh
```

---

## Запуск и тестирование

```bash
# Запуск всех узлов через стартовый скрипт
./robot/start_ui.sh

# Запуск только конкретного пакета, например, нейронного контроллера
ros2 launch neural_controller launch.py

# Запуск симуляции Mujoco
ros2 launch pupperv3_mujoco_sim test_hw_interface.launch.py

# Запуск тестового клиента LLM‑вебсокета
ros2 launch llm_websocket_server websocket_server_launch.py
```

Для запуска тестового набора используйте `colcon test` в корне workspace.

---

## Лицензия

Этот проект распространяется под лицензией **Apache License 2.0**. Подробнее смотрите файл `LICENSE`.

---

## Контакты и вклад

- **Автор:** команда разработки Pupper (maintainer: *Your Name* – <your.email@example.com>)  
- **Issues & Pull Requests:** открывайте Issues на GitHub, PR‑ы приветствуются.  
- **Документация:** дополнительные сведения находятся в каталоге `docs/` (если есть).  
- **Вклад:** при желании поддержать проект, см. раздел *Вклад* в этом README.

---

*Пожалуйста, не удаляйте ни один из перечисленных пакетов или файлов – они составляют целостную архитектуру Pupper v3.*  