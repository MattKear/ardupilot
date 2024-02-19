#!/bin/bash
set -e
set -m

echo "Starting ArduPilot instance"

if [ -z "${SITL_PARAMETER_LIST}" ]; then
    SITL_PARAMETER_LIST="/ardupilot/copter.parm,/ardupilot/copter-octaquad.parm"
fi

if [ -z "${SITL_MODEL}" ]; then
    SITL_MODEL="octa-quad:@ROMFS/models/MNA383.json"
fi

if [ -z "${INSTANCE}" ]; then
    I_INSTANCE="-I0"
else
    I_INSTANCE="-I$(echo $INSTANCE | sed -e 's/-I//')"
fi
if [ "${INSTANCE}" -eq 0 ];then
    SITL_MODEL="$SITL_MODEL --slave 2"
    CCDL_PORTS="--serial1=tcp:5761 --serial4=tcp:5764"
    SYSID=1
fi
if [ "${INSTANCE}" -eq 1 ];then
    SITL_MODEL="json:0.0.0.0"
    CCDL_PORTS="--serial1=tcp:5771 --serial4=tcpclient:0.0.0.0:5761"
    SYSID=2
fi
if [ "${INSTANCE}" -eq 2 ];then
    SITL_MODEL="json:0.0.0.0"
    CCDL_PORTS="--serial1=tcpclient:0.0.0.0:5764 --serial4=tcpclient:0.0.0.0:5771"
    SYSID=3
fi

if [ -z "${SITL_LAT}" ]; then
    SITL_LAT="53.346441"
fi

if [ -z "${SITL_LON}" ]; then
    SITL_LON="-7.715688"
fi

if [ -z "${SITL_ALT}" ]; then
    SITL_ALT="64.8"
fi

if [ -z "${SITL_HEADING}" ]; then
    SITL_HEADING="0"
fi

if [ -z "${NOLOGS}" ]; then
    NOLOGS="0"
fi

if [ -z "${NOEKF}" ]; then
    NOEKF="0"
fi

if [ -z "${NOBATTFS}" ]; then
    NOBATTFS="0"
fi

SITL_LOCATION="$SITL_LAT,$SITL_LON,$SITL_ALT,$SITL_HEADING"

echo "INSTANCE:"
echo "$I_INSTANCE"

IDENTITY_FILE=fcu${SYSID}.parm
cp /ardupilot/fcu${SYSID}.parm ./"$IDENTITY_FILE"
sed -i '/^[[:space:]]*$/d' "$IDENTITY_FILE"
echo "${IDENTITY_FILE}:"
cat "$IDENTITY_FILE"

MASTER_FILE=master_param_fcu${I_INSTANCE}.param

if [ -z "${NO_CCDL}" ]; then
    CCDL_FILE=ccdl.parm
    cp /ardupilot/ccdl.parm ./"$CCDL_FILE"
    sed -i '/^[[:space:]]*$/d' "$CCDL_FILE"
    echo "${CCDL_FILE}:"
    cat "$CCDL_FILE"
else
    CCDL_FILE=no_ccdl.parm
    cp /ardupilot/no_ccdl.parm ./"$CCDL_FILE"
    sed -i '/^[[:space:]]*$/d' "$CCDL_FILE"
    echo "${CCDL_FILE}:"
    cat "$CCDL_FILE"
fi

cp /ardupilot/383.parm ./"$MASTER_FILE"
sed -i '/LOG_BACKEND_TYPE/d' "$MASTER_FILE"
sed -i '/COMPASS_PRIO/d' "$MASTER_FILE"
sed -i '/BATT_VOLT_PIN/d' "$MASTER_FILE"
sed -i '/BATT_CURR_PIN/d' "$MASTER_FILE"
sed -i '/COMPASS_EXTERNAL/d' "$MASTER_FILE"
sed -i '/COMPASS_SCALE/d' "$MASTER_FILE"
sed -i '/COMPASS_USE/d' "$MASTER_FILE"
sed -i '/^SERIAL/d' "$MASTER_FILE"
sed -i '/^COMPASS_PRIO/d' "$MASTER_FILE"
sed -i '/^AHRS_ORIENTATION/d' "$MASTER_FILE"
sed -i '/^GPS_POS/d' "$MASTER_FILE"

# Remove empty lines from parameters files
sed -i '/^[[:space:]]*$/d' "$MASTER_FILE"

if [ "${NOLOGS}" -eq 1 ];then
  echo "Disabling logging !"
  printf "\nLOG_BACKEND_TYPE,0" >> "$MASTER_FILE"
fi

if [ "${NOEKF}" -eq 1 ];then
  echo "Disabling EKF !"
  printf "\nAHRS_EKF_TYPE,10" >> "$MASTER_FILE"
fi

if [ "${NOBATTFS}" -eq 1 ];then
  echo "Disabling Battery failsafe !"
  sed -i 's/BATT_FS_CRT_ACT,.*/BATT_FS_CRT_ACT,0/' "$MASTER_FILE"
  sed -i 's/BATT_FS_LOW_ACT,.*/BATT_FS_LOW_ACT,0/' "$MASTER_FILE"
fi

# Remove empty lines from parameters files
sed -i '/^[[:space:]]*$/d' "$MASTER_FILE"


SUREFLITE_PARAMS="$MASTER_FILE"

args="-S $I_INSTANCE --home ${SITL_LOCATION} --model ${SITL_MODEL} --speedup 1 --serial6=sim:lightwareserial --serial2=tcp:57$(($INSTANCE+6))2 ${CCDL_PORTS} --sysid ${SYSID} --disable-fgview --defaults ${SITL_PARAMETER_LIST},${SUREFLITE_PARAMS},${IDENTITY_FILE},${CCDL_FILE}"

echo "args:"
echo "$args"

export MAVLINK20=1
# Start ArduPilot simulator
/ardupilot/arducopter ${args}
