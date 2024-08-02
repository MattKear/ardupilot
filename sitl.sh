#!/bin/bash

## Script for managing common STIL commands

if [ $# -lt 1 ]
then
    echo "sitl <action> (start|ardu|log|api|exec|rm|build|run|flightlogs)"
    exit
fi

INSTANCE_NAME="ardupilot_sitl_tri"

echo "$1"
mkdir -p "/home/$(whoami)/log"
case $1 in
    'log' )
        # tail logs
        sudo docker logs -f --since 30s $INSTANCE_NAME
        ;;
    'exec' )
        # exec
        sudo docker exec -it $INSTANCE_NAME bash
        ;;
    'rm' )
        # remove
        sudo docker rm -f /ardupilot_sitl_tri
        ;;
    'build' )
        # build
        sudo docker build --progress plain . -f ardupilot_docker/Dockerfile -t ardupilot_manna
        ;;
    'run' )
        # run
        sudo docker run -it --rm --cap-add=SYS_PTRACE -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252628' --env SITL_LON='-10.022269' --env SITL_ALT='64.8' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest bash
        ;;

    'ardu' )
        # run ardupilot kerry pad a
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p2222:22 -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252628' --env SITL_LON='-10.022269' --env SITL_ALT='13' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_moorock' )
        # run ardupilot moorock pad d
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='53.346248' --env SITL_LON='-7.715564' --env SITL_ALT='13' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_opsim_a' )
        # run ardupilot ops sim pad b
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='53.345948' --env SITL_LON='-7.715352' --env SITL_ALT='13' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_bb_a' )
        # run ardupilot bb pad a
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='53.606374' --env SITL_LON='-6.196679' --env SITL_ALT='13' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_kerry_a' )
        # run ardupilot
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252628' --env SITL_LON='-10.022269' --env SITL_ALT='13' --env SITL_HEADING='0' --env NOLOGS="1" --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_kerry_b' )
        # run ardupilot
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252643' --env SITL_LON='-10.022492' --env SITL_ALT='13' --env SITL_HEADING='0' --env NOLOGS="1" --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_kerry_c' )
        # run ardupilot
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.2529258' --env SITL_LON='-10.0228084' --env SITL_ALT='13' --env SITL_HEADING='0' --env NOLOGS="1" --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_kerry_d' )
        # run ardupilot
        sudo docker run --rm --cap-add=SYS_PTRACE --init -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252765' --env SITL_LON='-10.022252' --env SITL_ALT='13' --env SITL_HEADING='0' --env NOLOGS="1" --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;

esac
