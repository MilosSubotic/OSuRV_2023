#!/bin/bash

sudo raspi-config nonint do_camera 0

sudo raspi-config
# -> 3 Interface Options -> P6 Serial Port
# -> <No> to disable serial -> <Yes> to enable serial hw
# -> <Ok> -> <Finish>
#sudo raspi-config nonint do_serial 1
#get_serial should be 1
#get_serial_hw should be 0
