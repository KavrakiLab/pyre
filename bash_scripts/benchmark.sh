#!/bin/bash

prefix_dset="package://pyre/datasets/"
prefix_db="package://pyre/database/"
dataset=("shelf_zero" "shelf_height" "shelf_height_rot")   
start=1
end=100

for ds in ${dataset[*]};
do 
    
    #Testing on the test dataset
    dset=$prefix_dset"/"$ds"_test/"

    dbSPARK=$prefix_db/$ds/sparkdb0001_0500
    dbFLAME=$prefix_db/$ds/flamedb0001_0500


    roslaunch --wait pyre benchmark.launch start:=$start end:=$end dataset:=$dset database:="none" algo:="UNIFORM" time:=60 &
    roslaunch --wait pyre benchmark.launch start:=$start end:=$end dataset:=$dset database:=$dbSPARK postfix:="_500" algo:="SPARK" time:=60 &
    roslaunch --wait pyre benchmark.launch start:=$start end:=$end dataset:=$dset database:=$dbFLAME postfix:="_500" algo:="FLAME" time:=60 &
    wait
done
