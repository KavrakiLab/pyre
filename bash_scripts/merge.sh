#!/bin/bash
prefix="package://pyre/database"
database=("shelf_zero" "shelf_height" "shelf_height_rot")   

for d in ${database[*]};
do 
    db=$prefix/$d
    roslaunch --wait pyre merge.launch database:=$db start:=1 end:=10 & 
    roslaunch --wait pyre merge.launch database:=$db start:=1 end:=30 & 
    roslaunch --wait pyre merge.launch database:=$db start:=1 end:=50 &
    roslaunch --wait pyre merge.launch database:=$db start:=1 end:=100 &
    roslaunch --wait pyre merge.launch database:=$db start:=1 end:=300 &
    roslaunch --wait pyre merge.launch database:=$db start:=1 end:=500 &
    wait
done
