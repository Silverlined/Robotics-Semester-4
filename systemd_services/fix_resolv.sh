#!/usr/bin/env bash
echo "nameserver 8.8.8.8" | sudo tee -a /etc/resolv.conf 
sudo systemctl restart systemd-resolved.service 
