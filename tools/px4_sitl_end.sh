#!/bin/bash
set -e

echo args: $@

model=$1

echo SITL END ARGS
echo model: $model

if [ "$model" == "" ] || [ "$model" == "none" ]
then
	echo "empty model, setting to iris as default"
	model="iris"
fi

pkill -x gazebo || true
pkill -x px4 || true
pkill -x px4_$model || true
