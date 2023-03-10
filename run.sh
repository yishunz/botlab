#!/bin/bash
source ~/botlab-w23/system_compilation/setenv.sh
gnome-terminal --tab -t 'Tab1' -- ~/botlab-w23/build/bin/./timesync &
sleep 2
gnome-terminal --tab -t 'Tab2' -- ~/botlab-w23/build/bin/./pico_shim
sleep 2
gnome-terminal --tab -t 'Tab3' -- minicom botlab
sleep 2
