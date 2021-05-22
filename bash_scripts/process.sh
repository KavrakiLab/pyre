#!/bin/bash
prefix="package://pyre/datasets"
dataset=("shelf_zero" "shelf_height" "shelf_height_rot")   

for d in ${dataset[*]};
do 
    dset=$prefix/$d/
    roslaunch --wait pyre process.launch dataset:=$dset start:=1 end:=100 & 
    roslaunch --wait pyre process.launch dataset:=$dset start:=101 end:=200 & 
    roslaunch --wait pyre process.launch dataset:=$dset start:=201 end:=300 &
    roslaunch --wait pyre process.launch dataset:=$dset start:=301 end:=400 &
    roslaunch --wait pyre process.launch dataset:=$dset start:=401 end:=500 &
    wait
done
