## FCU1 

sim_vehicle.py -v ArduCopter -f quad --slave 2 -I0 --auto-sysid -A "--serial2=udpclient:0.0.0.0:14552 --serial1=tcp:5764 --serial4=tcp:5765 --disable-fgview" --use-dir=FCU1 --add-param-file=$(pwd)/383.parm --debug --console --map

## FCU2

sim_vehicle.py -v ArduCopter --model json:0.0.0.0 -I1 --auto-sysid --slave 0 -A "--serial1=tcpclient:127.0.0.1:5764 --serial4=tcp:5766 --disable-fgview" --use-dir=FCU2 --add-param-file=$(pwd)/383.parm --debug --no-rebuild -m "--console --source-system 252"

## FCU3

sim_vehicle.py -v ArduCopter --model json:0.0.0.0 -I2 --auto-sysid --slave 0 -A "--serial1=tcpclient:127.0.0.1:5765 --serial4=tcpclient:127.0.0.1:5766 --disable-fgview" --use-dir=FCU3 --add-param-file=$(pwd)/383.parm --debug --no-rebuild -m "--console --source-system 253"


# Testing 
FCU1

mavproxy.py --master /dev/CUBE1 --source-system 251 --target-system 1


FCU2
mavproxy.py --master /dev/CUBE2 --source-system 252 --target-system 2
mavproxy.py --master /dev/ttyACM0 --source-system 252 --target-system 2

FCU3 

mavproxy.py --master /dev/CUBE3 --source-system 253 --target-system 3


param set FRAME_CLASS 1
param set RNGFND1_TYPE 0
param set SERIAL1_PROTOCOL 99
param set SERIAL4_PROTOCOL 99
param set SERIAL4_BAUD 921


get the CCDL routing config with 
long MAV_CMD_DO_SEND_BANNER
