<details>
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