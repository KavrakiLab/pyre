
# Overview
An implementation of SPARK and FLAME: two learning-for-motion-planning frameworks suitable for High-DOF Robots in geometric or sensed 3D workspaces

- [Paper](https://arxiv.org/abs/2010.15335)
- [Short_Video](https://youtu.be/cH4_lIjjs58) (2 min)
- [Long_Video](https://youtu.be/DP0376NNHQo) (11 min)

Please cite our work if you use our code or compare to our approach.
```
@article{chamzas2021learning,
  author  = {Chamzas, Constantinos and Kingston, Zachary and Quintero-Peña, Carlos and Shrivastava, Anshumali and Kavraki, Lydia E.},
  title   = {Learning Sampling Distributions Using Local 3D Workspace Decompositions for Motion Planning in High Dimensions },
  booktitle={International Conference on Robotics and Automation (ICRA)},
  year={2021},
  organization={IEEE}
}
```

## 1) Installation

The following instructions have been tested on **Ubuntu 18.04**. Similar
instructions should work for other Linux distributions.
1. Install [Robowflex](https://github.com/KavrakiLab/robowflex) v1.2 and above. You can follow [these instructions](https://kavrakilab.github.io/robowflex/md__home_runner_work_robowflex_robowflex__8docs_markdown_installation.html). 

2. Clone this repository into the `src` folder of your catkin workspace:

   ```
   cd <location_of_your_workspace>/src
   git clone https://github.com/KavrakiLab/pyre.git
   ```
3. Add the Fetch robot description files (Choose only one option) 
     - Robowflex resources version. This version includes only the necessary files (URD, SRDF, meshes) 
       ```
       cd <location_of_your_workspace>/src
       git clone https://github.com/KavrakiLab/robowflex_resources.git
       ```
     - Fetch Robotics from Debian/Source (Includes all the ros-fetch software)
       ```
       # Debian
       sudo apt install ros-melodic-fetch-ros

       # Or, Source
       cd <location_of_your_workspace>/src
       git clone https://github.com/fetchrobotics/fetch_ros
       ```
4. Finally, build your catkin workspace and source the devel/setup.bash:
   ```
   cd <location_of_your_workspace>
   catkin build
   source devel/setup.bash
   ```

## 2) Training/Testing datasets

Simply unzip the datasets.zip file. They will be placed under the folder `<location_of_your_workspace>/datasets/`.
```
#Go to the pyre package
roscd pyre
unzip datasets.zip
```

The three provided datasets have motion planning problems of a Fetch robot placing its arm inside a deep shelf. 
The `scene*`  yaml files include geometric representations of the scenes used by SPARK.
The `scene_sensed*` yaml files include octomap representions  of the scenes used by FLAME.  

- `shelf_zero` : 100 test and 500 train examples of the XY dataset described in the paper. 
- `shelf_height` : 100 test and 500 train examples with X,Y,Z dataset described in the paper. 
- `shelf_heigth_rot` : 100 test and 500 train examples with X,Y,Z,Θ dataset described in the paper. 


## 3) Experience databases (Learning)

### Use precomputed experience databases
Simply unzip the databases.zip file. They will be placed under the folder `<location_of_your_workspace>/database/`.
```
roscd pyre
unzip database.zip
```

### Generate new experience databases 
1. Start a rosmaster instance. 
    ```
    roscore
    ```
2. In a new terminal run the `process.sh` script which processes each path and scene into local primitives (~60 minutes).  
    ```
     source ../../devel/setup.bash
    ./bash_scripts/process.sh
    ```
3. Afterwards run the merging script that aggregates the local primitives to complete experience databases. 
    ```
    ./bash_scripts/merge.sh
    ```

## 4) Benchmarking and Visualizing the results. 

1. Start a rosmaster instance (if you did not start one already).     
   ```
   roscore
   ```

2. Run one of the following scripts.
      - Benchmark SPARK, FLAME with full databases (500) and corresponds to Fig. 4b) for all three datasets.
         ```
         source ../../devel/setup.bash
         ./bash_scripts/benchmark.sh
         ```
      - Benchmark SPARK,FLAME with incremental databases (10, 30, 50, 100, 300, 500  corresponds to Fig. 4c) for `shelf_height_rot'.
         ```
         source ../../devel/setup.bash
         ./bash_scripts/benchmark_inc.sh 
         ```

3. To plot the results use the `unify.sh` script to aggregate the benchmarking results for each dataset. `bench_inc.sh`  the shelf_height_rot dataset
   ```
   ./benchmark/unify.sh shelf_zero_test results
   ./benchmark/unify.sh shelf_height_test results
   ./benchmark/unify.sh shelf_height_rot_test results
   ```
   A `<dataset>\_results.db` is generated for each dataset under the `benchmark/` folder. You can load these files in  [Planner Arena](http://plannerarena.org/) to plot the results.
   
   **Note:** If you are using Python2 and [`ompl_benchmark_statistics.py`](https://github.com/ompl/ompl/blob/master/scripts/ompl_benchmark_statistics.py) does not find pathlib you may have to `apt install python-pathlib2` or `pip install pathlib2`.


