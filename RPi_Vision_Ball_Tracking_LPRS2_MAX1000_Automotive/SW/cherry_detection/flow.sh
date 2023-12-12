#!/bin/bash

exit 1

sudo ./waf prerequisites
./waf configure
./waf build
./waf build run --app=ball_uart_chassis
sudo ./waf install
sudo ./waf postinstall
./waf services_log
sudo ./waf services_stop


