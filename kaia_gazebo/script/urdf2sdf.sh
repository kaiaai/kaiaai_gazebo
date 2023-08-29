#!/bin/bash

cd $1
xacro $2.urdf.xacro > $2.urdf
gz sdf -p $2.urdf > $2.sdf
sed_arg="s/_description\/sdf\/$2//g"
sed -i $sed_arg $2.sdf
rm $2.urdf
mv $2.sdf ../sdf/$2/model.sdf


