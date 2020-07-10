## Steps to launch simulation 

#### Step 1. Create a catkin workspace
Skip this step if you already have a workspace
```sh
$ mkdir -p /home/workspace/catkin_ws/src
$ cd catkin_ws/
$ catkin build
```

#### Step 2. Perform a system update
```sh
$ apt-get update
```
#### Step 3. Clone the package in src
```sh
$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/HBRS-SDP/sdp_ss2020_tube_driving.git
Change the name of the package to 'tube_navigation'.
``` 

#### Step 4. Load dependable packages
```sh
Add the following dependable packages in src-
* multi_ropod_sim
* occupancy_grid
* ropod_ros_msgs
* ropod_sim_model
Also save the Goal route message file.
* dock_to_elevator_route_plan.sh
```

#### Step 4. Install Dependencies
```sh
$ sudo apt-get-install ros-kinetic-amcl
``` 
#### Step 5. Build and source your workspace
```sh
$ catkin build
For bash
$ source ~/catkin_ws/src/devel/setup.bash
For zsh
$ source ~/catkin_ws/src/devel/setup.zsh
``` 
#### Step 6. Launch the nodes
```sh
$ roslaunch ropod_gazebo multi_robot.launch 
$ roslaunch tube_navigation tube_navigation.launch
$ ./dock_to_elevator_route_plan.sh
```

The first launch command will launch the rviz with a map and robot in it. The second command will initialize the tube navigation. After running the script file the robot will start moving from start area to goal area.
