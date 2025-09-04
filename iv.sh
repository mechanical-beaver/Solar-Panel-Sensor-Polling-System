#!/bin/bash
mosquitto_pub -h 192.168.252.23 -p 31883 \
  -u "telegraf" -P "hentai" \
  -t "/device/command" \
  -m "$(printf '{"device_uuid":"346C6321-2C8E-4663-9B30-55A2E05117C1","iv_char_uuid":"%s","command":"exec_iv"}' "$(uuidgen)")"
