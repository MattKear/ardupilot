## FCU1 

sim_vehicle.py -v ArduCopter -f quad --slave 2 -I0 --auto-sysid -A "--serial2=udpclient:0.0.0.0:14552 --serial1=tcp:5764 --serial4=tcp:5765 --disable-fgview" --use-dir=FCU1 --add-param-file=$(pwd)/383.parm --debug --console --map
sim_vehicle.py -v ArduCopter -f quad -I0 --auto-sysid -A "--serial2=udpclient:0.0.0.0:14552 --serial1=tcp:5764 --serial4=tcp:5765 --disable-fgview" --use-dir=FCU1 --add-param-file=$(pwd)/383.parm --debug --console --map

## FCU2

sim_vehicle.py -v ArduCopter --model json:0.0.0.0 -I1 --auto-sysid --slave 0 -A "--serial1=tcp:5766 --serial4=tcpclient:127.0.0.1:5764 --disable-fgview" --use-dir=FCU2 --add-param-file=$(pwd)/383.parm --debug --no-rebuild -m "--console --source-system 252"

## FCU3

sim_vehicle.py -v ArduCopter --model json:0.0.0.0 -I2 --auto-sysid --slave 0 -A "--serial1=tcpclient:127.0.0.1:5765 --serial4=tcpclient:127.0.0.1:5766 --disable-fgview" --use-dir=FCU3 --add-param-file=$(pwd)/383.parm --debug --no-rebuild -m "--console --source-system 253"


# Testing 
### FCU1

mavproxy.py --master /dev/CUBE1 --source-system 251 --target-system 1


### FCU2
mavproxy.py --master /dev/CUBE2 --source-system 252 --target-system 2

### FCU3 

mavproxy.py --master /dev/CUBE3 --source-system 253 --target-system 3

## Useful params
`param set FRAME_CLASS 1
param set RNGFND1_TYPE 0
param set SERIAL1_PROTOCOL 99
param set SERIAL4_PROTOCOL 99
param set SERIAL4_BAUD 921
param set CCDL_TOUT_ENABLE 1`


get the CCDL routing config with 
``watch CCDL*``
then
`long MAV_CMD_GET_CCDL_STATUS`

# Bootloader build 

modify ardupilot/modules/DroneCAN/libcanard/dsdl_compiler/pyuavcan/uavcan/transport.py:

`import collections.abc as collections
from collections import OrderedDict`

and 

`self.__dict__["_fields"] = OrderedDict()`


# NTRIP 

`module load ntrip
ntrip set caster caster.centipede.fr
ntrip set port 2101
ntrip set mountpoint TCY22
ntrip set username centipede
ntrip set password centipede
ntrip start`


Message mavlink
``<message id="233" name="GPS_RTCM_DATA">``



# FW upload
python3 updater.py --port /dev/CUBE1,/dev/CUBE1-BL --target-system 1 --source-system 1 arducopter.apj

python3 updater.py --port /dev/CUBE2,/dev/CUBE2-BL --target-system 2 --source-system 1 arducopter.apj

python3 updater.py --port /dev/CUBE3,/dev/CUBE3-BL --target-system 3 --source-system 1 arducopter.apj
