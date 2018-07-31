#!/usr/bin/env bash
# Stop matrice210.service located in /etc/systemd/system
# To know service status : systemctl list-units | grep matrice
sudo systemctl stop matrice210 --now
