#!/usr/bin/env bash
# Быстрая проверка: WireGuard + доступность Gemini API с Pi.
set -euo pipefail

ENV_FILE="${HOME}/pupperv3-monorepo/ai/llm-ui/live-audio/.env.local"
API_KEY=""
if [[ -f "$ENV_FILE" ]]; then
  API_KEY="$(grep -E '^GEMINI_API_KEY=' "$ENV_FILE" | cut -d= -f2- || true)"
fi

echo "=== WireGuard (wg0) ==="
if sudo wg show wg0 2>/dev/null | grep -q "latest handshake"; then
  sudo wg show wg0 | grep -E "interface:|latest handshake:|transfer:"
else
  echo "wg0 не поднят или нет handshake"
fi

echo ""
echo "=== Публичный IP ==="
echo -n "через wg0:   "; curl -sS --interface wg0 --connect-timeout 8 https://ifconfig.me 2>/dev/null || echo "нет"; echo
echo -n "через wlan0: "; curl -sS --interface wlan0 --connect-timeout 8 https://ifconfig.me 2>/dev/null || echo "нет"; echo
echo -n "default:     "; curl -sS --connect-timeout 8 https://ifconfig.me 2>/dev/null || echo "нет"; echo

echo ""
echo "=== GEMINI_API_KEY ==="
if [[ -z "$API_KEY" ]]; then
  echo "НЕ ЗАДАН — создайте $ENV_FILE"
  exit 1
fi
echo "задан (${#API_KEY} символов)"

echo ""
echo "=== Gemini REST API (generateContent) ==="
RESP="$(curl -sS --connect-timeout 15 \
  -H "Content-Type: application/json" \
  -d '{"contents":[{"parts":[{"text":"reply with ok"}]}]}' \
  "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=${API_KEY}")"

if echo "$RESP" | grep -q '"text"'; then
  echo "OK — Gemini отвечает"
  echo "$RESP" | head -c 300
  echo ""
  exit 0
fi

if echo "$RESP" | grep -q "User location is not supported"; then
  echo "FAIL — Google блокирует регион для этого API-ключа/аккаунта"
  echo "  (часто не лечится VPN — нужен ключ из поддерживаемого региона)"
elif echo "$RESP" | grep -q "API key not valid"; then
  echo "FAIL — неверный API key"
else
  echo "FAIL — неожиданный ответ:"
fi
echo "$RESP" | head -c 500
echo ""
exit 1
