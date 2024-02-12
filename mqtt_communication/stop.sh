#!/bin/bash

network_name="mqtt_network"

remove_network() {
  if docker network ls | grep -q $network_name; then
    echo "Removing Docker network: $network_name"
    docker network rm $network_name
  else
    echo "Docker network '$network_name' does not exist."
  fi
}

stop_and_remove_containers() {
  local containers=$(docker ps -a --filter "name=^mqtt-app-instance-[0-9]+$" --format '{{.Names}}')
  
  if [ -z "$containers" ]; then
    echo "No containers found with regex 'mqtt-app-instance-X'."
    return
  fi
  
  for container in $containers; do
    echo "Stopping and deleting container: $container"
    docker stop $container > /dev/null
    docker rm $container > /dev/null
    echo "Container $container has been stopped and removed."
  done
}

stop_and_remove_containers
# remove_network
