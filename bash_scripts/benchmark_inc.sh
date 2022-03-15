#!/bin/bash

prefix_dset="package://pyre/datasets/"
prefix_db="package://pyre/database/"
database=("0010" "0030" "0050" "0100" "0300" "0500")   
start=1
end=100
    
#Testing on the test dataset
dset="$prefix_dset"shelf_height_rot_test/

roslaunch --wait pyre benchmark.launch start:=$start end:=$end dataset:=$dset database:="none" algo:="UNIFORM" time:=60
for db in ${database[*]};
do 
     dbSPARK=$prefix_db/shelf_height_rot/sparkdb0001_$db
     dbFLAME=$prefix_db/shelf_height_rot/flamedb0001_$db
     roslaunch --wait pyre benchmark.launch start:=$start end:=$end dataset:=$dset database:=$dbSPARK postfix:=_$db algo:="SPARK" time:=60 &
     roslaunch --wait pyre benchmark.launch start:=$start end:=$end dataset:=$dset database:=$dbFLAME postfix:=_$db algo:="FLAME" time:=60 &
wait
done
