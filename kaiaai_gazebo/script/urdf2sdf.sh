#!/bin/bash

# Example
# urdf2sdf.sh /ros_ws/src/makerspet_loki

[[ -z "$1" ]] && { echo "$1 does not exist" ; exit 1; }

robot_model_name=$(echo "$1" | sed "s/.*\///")
echo "Converting $1/urdf/robot.urdf.xacro"

cd $1/urdf
xacro robot.urdf.xacro > robot.urdf
gz sdf -p robot.urdf > robot.sdf
rm robot.urdf
dest=$1/sdf/$robot_model_name/model.sdf
mv robot.sdf $dest
echo "Wrote $dest"
