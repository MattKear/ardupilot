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
        docker stop $(sudo docker ps -a -q)
        docker rm $(sudo docker ps -a -q)
        ;;
    'build' )
        # build
        sudo docker build . -f ardupilot_docker/Dockerfile -t ardupilot_manna
        ;;
    'run' )
        # run
        sudo docker run -it --rm -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252628' --env SITL_LON='-10.022269' --env SITL_ALT='64.8' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest bash
        ;;

    'ardu' )
        # run ardupilot kerry pad a
        sudo docker run --rm -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252628' --env SITL_LON='-10.022269' --env SITL_ALT='64.8' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_moorock' )
        # run ardupilot moorock pad d
        sudo docker run --rm -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='53.346248' --env SITL_LON='-7.715564' --env SITL_ALT='64.8' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_opsim_a' )
        # run ardupilot ops sim pad b
        sudo docker run --rm -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='53.345948' --env SITL_LON='-7.715352' --env SITL_ALT='64.8' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_bb_a' )
        # run ardupilot bb pad a
        sudo docker run --rm -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='53.606374' --env SITL_LON='-6.196679' --env SITL_ALT='64.8' --env SITL_HEADING='0' --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;
    'ardu_kerry_a' )
        # run ardupilot
        sudo docker run --rm -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 -p5501:5501 --env SITL_LAT='52.252628' --env SITL_LON='-10.022269' --env SITL_ALT='64.8' --env SITL_HEADING='0' --env NOLOGS="1" --name "${INSTANCE_NAME}" ardupilot_manna:latest
        ;;

esac
