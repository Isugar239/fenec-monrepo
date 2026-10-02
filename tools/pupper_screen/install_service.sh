#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -sfn "${SCRIPT_DIR}/pupper-screen.service" /etc/systemd/system/pupper-screen.service
sudo systemctl daemon-reload
sudo systemctl enable pupper-screen.service
sudo systemctl restart pupper-screen.service
