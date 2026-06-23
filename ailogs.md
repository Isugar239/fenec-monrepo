```<details>
<summary>Заметки Cursor (Pupper robot workspace)</summary>

## Gemini Live API — `audioChunks` / `mediaChunks` deprecated (2026-05)

**Симптом:** модель `gemini-3.1-flash-live-preview` (или документация/skills) пишет, что `audioChunks` / audio chunks deprecated.

**Причина (клиент → сервер):** в `@google/genai` вызов  
`session.sendRealtimeInput({ media: blob })` конвертируется в wire-поле **`mediaChunks`** (см. `liveSendRealtimeInputParametersToMldev` в SDK). Для Gemini 3.1 Live это устаревший путь.

**Правильно:**
```ts
session.sendRealtimeInput({ audio: createBlob(pcmFloat32) });
// при остановке микрофона (VAD включён по умолчанию):
session.sendRealtimeInput({ audioStreamEnd: true });
```

- `createBlob` в `ai/llm-ui/live-audio/utils.ts` — PCM 16 kHz mono, `mimeType: 'audio/pcm;rate=16000'`.
- **Не** использовать `media` в `sendRealtimeInput`; для видео — `video`, для текста — `text`.
- `sendClientContent` — только для начальной истории (`history_config`), не для живого диалога.

**Приём аудио (сервер → клиент):** читать **`serverContent.modelTurn.parts[]`** с `inlineData`, обходить **все** parts в событии (не только `[0]`). Поле `serverContent.audioChunks` относится к **Live Music**, не к обычному Live.

**Файлы UI:** `ai/llm-ui/live-audio/index.tsx`, `session-manager.ts`. Старая копия с `media`: `live-audio-original/`.

**Модели:** предпочтительно `gemini-3.1-flash-live-preview`; `gemini-2.5-flash-native-audio-preview-12-2025` в template помечена как устаревающая.

**Документация:** https://ai.google.dev/gemini-api/docs/live.md — skill `gemini-live-api-dev` (google-gemini/gemini-skills).

**TODO (не сделано):** ephemeral tokens вместо `GEMINI_API_KEY` в браузере; обход всех `modelTurn.parts` при воспроизведении; session resumption / context compression для сессий >15 мин.

</details>
## Интеграция USB-камеры Logitech в ROS 2 Jazzy (Hailo NPU) (2026-05-29)

**Симптом 1:** Нода `hailo_detection` падает с ошибкой `FileNotFoundError` или `KeyError: 'fx'/'xi'` при запуске `hailo_detection_launch.py`.
**Причина:** Код `fisheye_utils.py` в пакете жестко требует калибровочный файл `camera_params.yaml` со специфическими коэффициентами для родной широкоугольной линзы (модель Double Sphere). Без файла или при пустом файле парсер падает.

**Решение (Обход дисторсии):** Создать калибровочный файл с "идеальными" параметрами, где коэффициенты искажения выставлены в ноль. Математически это превращает Double Sphere в Pinhole модель, пропуская картинку с Logitech 1 к 1 без искажений.
*Финальная рабочая структура `camera_params.yaml` для HD:*
```yaml
camera_params:
  fx: 1000.0
  fy: 1000.0
  cx: 640.0
  cy: 360.0
  xi: 0.0
  alpha: 0.0
```
*Пути для сохранения конфига:*
- Исходники: `~/pupperv3-monorepo/ros2_ws/src/hailo/hailo/camera_params.yaml`
- Install-директория: `~/pupperv3-monorepo/ros2_ws/install/hailo/lib/python3.11/site-packages/hailo/camera_params.yaml`

---

**Симптом 2:** Нода камеры `usb_cam` падает по `std::invalid_argument` (Unsupported resolution `1400x1050`).
**Причина:** Оригинальный стек Pupper v3 залочен под родное разрешение CSI-камеры (1400x1050), которое веб-камера Logitech аппаратно не поддерживает.

**Решение:** Сборка драйвера `usb_cam` из исходников (так как бинарников `ros-jazzy-usb-cam` под ARM64 пока нет в apt) и запуск камеры в аппаратно поддерживаемом HD-разрешении (`1280x720`). Центр калибровки в YAML выше пересчитан под этот формат (`cx: 640`, `cy: 360`).

---

### 🚀 Инструкция по запуску видеопотока и Hailo NPU:

1. **Терминал 1 (Драйвер камеры Logitech):**
   Запуск захвата в формате 1280x720 на частоте 15 Hz с маппингом в системный топик детектора:
   ```bash
   ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=1280 -p image_height:=720 -p frame_rate:=15.0 -r /image_raw:=/camera/image_raw
   ```

2. **Терминал 2 (Нейросеть Hailo AI Hat):**
   Запуск аппаратного инференса в реальном режиме (модель ожидает топик `/camera/image_raw`):
   ```bash
   ros2 launch hailo hailo_detection_launch.py sim:=false
   ```

3. **Мониторинг результатов:**
   - Выходной топик детекций в формате ROS 2 `vision_msgs`: `/detections`
   - Для проверки частоты кадров детектора в терминале: `ros2 topic hz /detections`
   - Для визуализации: в Foxglove Studio перетащить топик `/detections` поверх окна `/image`.
