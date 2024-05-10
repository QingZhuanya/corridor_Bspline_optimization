

<video src="/home/qinzhuan/opt_planner/Optimization_Corridor_Bspline/src/planner/Videos/corridor_Bspline.mp4"></video>

## 1. Setup

### 1.1 Prerequisites

```
sudo apt-get install ros-$ROS_DISTRO-costmap-*

sudo apt-get install ros-$ROS_DISTRO-server

sudo apt-get install ros-$ROS_DISTRO-tf

sudo apt-get install libeigen3-dev

sudo apt-get install libgoogle-glog-dev
```

### 1.2 Get Start

```
mkdir -p Corridor_Bspline/src/planning && cd Corridor_Bspline/src/planning

git clone git@github.com:QingZhuanya/corridor_Bspline_optimization.git

cd ../..

catkin_make

source devel/setup.zsh

roslaunch opt_planner run_opt_planner.launch
```



## 2. Acknowledgements

- https://github.com/zm0612/Hybrid_A_Star
- https://github.com/ZJU-FAST-Lab/ego-planner
- https://github.com/ApolloAuto/apollo
