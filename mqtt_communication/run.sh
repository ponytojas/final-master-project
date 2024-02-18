#!/bin/bash

skip_build=0
instance_count=10
network_name="mqtt-network"

show_help() {
  echo "Use: $0 [-s] [-i <number_of_instances>] [-h]"
  echo "  -s                      : Skip build the Docker image. By default, the image is built."
  echo "  -i <number_of_instances>: Number of instances to run. Default is 10."
  echo "  -h                      : Show this help message."
}

while getopts "hsi:" opt; do
  case $opt in
    s)
      skip_build=1
      ;;
    i)
      instance_count=$OPTARG
      ;;
    h)
      show_help
      exit 0
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      show_help
      exit 1
      ;;
    :)
      echo "Option -$OPTARG require one argument." >&2
      exit 1
      ;;
  esac
done

if [ $OPTIND -eq 1 ]; then show_help; exit 0; fi

if [ $skip_build -eq 0 ]; then
  docker build -t mqtt-app . || exit 1
fi


for i in $(seq 1 $instance_count); do
  docker run -d --name mqtt-app-instance-$i --env MQTT_HOST=mqtt --env MQTT_PORT=1883 --env MQTT_TOPIC=ros/data --env CARLA_HOST=host.docker.internal --env CARLA_PORT=2000 --network mqtt-network --add-host=host.docker.internal:host-gateway mqtt-app 
done
