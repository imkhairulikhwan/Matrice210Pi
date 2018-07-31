#!/usr/bin/env bash
# Start matrice210.service located in /etc/systemd/system
# Will launch runMatrice210.sh
# To know service status : systemctl list-units | grep matrice
sudo systemctl start matrice210 --now
