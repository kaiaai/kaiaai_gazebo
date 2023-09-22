#!/bin/bash

# urdf2sdf.sh /full/path/to/urdf/robot.urdf.xacro
[[ -z "$1" ]] && { echo "$1 does not exist" ; exit 1; }

path_name=$(echo "$1" | sed -r "s/(.+)\/.+/\1/")
file_name_dot_urdf_dot_xacro=$(echo "$1" | sed "s/.*\///")
file_name_dot_urdf=${file_name_dot_urdf_dot_xacro%.xacro}
file_name_base=${file_name_dot_urdf%.urdf}

cd $path_name

xacro $1 > $file_name_dot_urdf
gz sdf -p $file_name_dot_urdf > $file_name_base.sdf
rm $file_name_dot_urdf
dest=$path_name/../sdf/$file_name_base/model.sdf
mv $file_name_base.sdf $dest
echo "Wrote $dest"
