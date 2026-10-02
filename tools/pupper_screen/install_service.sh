#!/bin/bash
# Экран теперь в пульте Фененки (:8091). Старый сервис можно отключить.
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -sfn "${SCRIPT_DIR}/pupper-screen.service" /etc/systemd/system/pupper-screen.service
sudo systemctl daemon-reload
sudo systemctl disable --now pupper-screen.service || true
echo "Экран обслуживает pupper-dashboard.service на порту 8091."
