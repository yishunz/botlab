cd ~/botlab-w23/system_compilation
source setenv.sh
rm ~/botlab-w23/test_logs/cur_log
gnome-terminal --tab --title=timesync --working-directory=$HOME/botlab-w23/system_compilation/bin -- ./timesync 
gnome-terminal --tab --title=picoshim --working-directory=$HOME/botlab-w23/system_compilation/bin -- ./pico_shim
gnome-terminal --tab --title=motion_contorl --working-directory=$HOME/botlab-w23/build/bin/ -- ./motion_controller
lcm-logger ~/botlab-w23/test_logs/cur_log
