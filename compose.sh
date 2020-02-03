#!/bin/bash

usage="usage: compose up|stop|down

up    -   build and run
stop  -   stop containers
down  -   stop and delete containers"

if [ "$#" -lt 1 ]; then
  echo "$usage";
  exit 1;
fi;

###############################################################################

neo4j() {
  case "$1" in
    image)
      echo "neo4j:latest"
      ;;
    run)
      echo "-p 7474:7474 -p 7687:7687 \
        -v $HOME/neo4j/data:/data \
        -v $HOME/neo4j/logs:/logs"
      ;;
  esac;
}

har-cont() {
  case "$1" in
    build)
      echo "."
      ;;
    run)
      echo " --gpus all \
        -p 5000:5000 -p 8000:8000 -p 42424:42424 -p 4200:4200  -p 8765:8765 \
        --device /dev/video0:/dev/video0"
      ;;
  esac;
}

###############################################################################

services_rev=$(compgen -A function)
services=$(echo "$services_rev" | tac)
network=$(echo "$services" | paste -sd "_" -)
cmd="$1"

exists() {
  docker ps -a | grep -e "\s$1$" && echo 1
}

network_exists() {
  docker network ls | grep -e "\s$1\s" && echo 1
}

create_network() {
  docker network create --driver bridge "$network";
}

is_cache() {
  echo "$1" | grep "Using cache" 1>/dev/null
  echo "$?"
}

container_up() {
  container="$1";
  lines=()
  i=0

  from_cache=1
  if [ -n "$($container build)" ]; then
    while read -r line; do
      echo "$line";
      lines[$i]="$line";
      i=$(((i + 1) % 4));
    done < <(docker build -t "$container" $($container build));

    third=$(is_cache "${lines[$i]}")
    fourth=$(is_cache "${lines[$(((i + 1) % 4))]}")
    from_cache=$( [ "$third" == 0 ] || [ "$fourth" == 0 ] ; echo $?);
  fi;

  ex="$(exists $container)";
  if [[ ( "$from_cache" == 0 || -n "$($container image)" )  && -n "$ex" ]]; then
    echo "[S] starting $container"
    docker start "$container";
  else
    echo "[R] running $container"
    if [ "$ex" ]; then
      docker rm "$container";
    fi;

    img_name="$($container image)";
    if [ -z "$img_name" ]; then
      img_name="$container";
    fi
    docker run --network "$network" --name "$container" -d $($container run) "$img_name";
  fi;
}


case "$cmd" in
  stop)
    while read -r container; do
      echo "Stopping \"$container\""
      docker stop "$container";
    done <<< "$services_rev";
    ;;
  down)
    while read -r container; do
      echo "Shutting down \"$container\""
      ex="$(exists $container)";
      if [ -n "$ex" ]; then
        docker rm $(docker stop "$container");
      else
        echo "Container \"$container\" is already down";
      fi
    done <<< "$services_rev";
    ;;
  up)
    while read -r container; do
      running=$(docker ps -f 'status=running' | grep -e "\s$container$");
      if [ -n "$running" ]; then
        echo "container \"$container\" is running, stop it before starting again";
        exit 1;
      fi
    done <<< "$services";

    if [ ! "$(network_exists $network)" ]; then
      create_network;
    fi;

    while read -r container; do
      container_up "$container";
    done <<< "$services";
    ;;
  *)
    echo "$usage";
    echo "";
    echo "Unknow command \"$cmd\""
    ;;
esac;

exit 0
