## FCU1 

sim_vehicle.py -v ArduCopter -f quad --slave 2 -I0 --auto-sysid -A "--serial2=udpclient:0.0.0.0:14552 --serial4=tcp:5764 --serial5=tcp:5765 --disable-fgview" --use-dir=FCU1 --add-param-file=$(pwd)/383.parm --debug --console --map

## FCU2

sim_vehicle.py -v ArduCopter --model json:0.0.0.0 -I1 --auto-sysid --slave 0 -A "--serial4=tcpclient:127.0.0.1:5764 --serial5=tcp:5766 --disable-fgview" --use-dir=FCU2 --add-param-file=$(pwd)/383.parm --debug --no-rebuild -m "--console --source-system 252"

## FCU3

sim_vehicle.py -v ArduCopter --model json:0.0.0.0 -I2 --auto-sysid --slave 0 -A "--serial4=tcpclient:127.0.0.1:5765 --serial5=tcpclient:127.0.0.1:5766 --disable-fgview" --use-dir=FCU3 --add-param-file=$(pwd)/383.parm --debug --no-rebuild -m "--console --source-system 253"


## MAVLINK Patch:

    <message id="11052" name="CCDL_TIMEOUT">
      <description>A ping message either requesting or responding to a ping. This allows to measure the system latencies.</description>
      <field type="uint64_t" name="time_usec" units="us">Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.</field>
      <field type="uint32_t" name="seq">PING sequence</field>
      <field type="uint8_t" name="target_system">0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system</field>
      <field type="uint8_t" name="target_component">0: request ping from all receiving components. If greater than 0: message is a ping response and number is the component id of the requesting component.</field>
    </message>


FCU1

mavproxy.py --master /dev/CUBE1 --source-system 251 --target-system 1


FCU2 

mavproxy.py --master /dev/ttyACM0 --source-system 252 --target-system 2

FCU3 

mavproxy.py --master /dev/CUBE3 --source-system 253 --target-system 3


param set FRAME_CLASS 1
param set RNGFND1_TYPE 0
param set SERIAL1_PROTOCOL 43
param set SERIAL4_PROTOCOL 43
param set SERIAL4_BAUD 921
