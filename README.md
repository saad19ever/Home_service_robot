# Home_service_robot


The goal of this project is to design a robot that maps its environment and autonomously navigate
to pick up virtual objects and then carries them to a drop off location. 
Both the pick up and the drop off location can be pre-set using a configuration file.

 
 ## Project structure
```
.
├── README.md
├── CMakeLists.txt
├── add_markers/
│   │
│   ├── src/add_markers.cpp
│    ├── ...
│
├── config/
│   └── add_marker_config.yaml
│
├── map/
│   ├── map.pgm
│   ├── map.yaml
│   ├── ...
│
├── pick_objects/
│   ├── src/pick_objects.cpp
│   │   └── pick_objects.cpp
│   ├── ...
│
├── scripts/
│   ├── home_service.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
│
├── slam_gmapping/
│   │
│   ├── gmapping/
│   │   ├── ...
│
├── turtlebot3/
│   ├── turtlebot3_navigation/
│   │   │
│   │   ├── launch/
│   │   │   ├── amcl.launch
│   │   │   ├── move_base.launch
│   │   │   └── turtlebot3_navigation.launch
│   │   │
│   │   ├── maps/
│   │   │   ├── map.pgm
│   │   │   └── map.yaml
│   │   │
│   │   ├── param/
│   │   |   ├── ... 
│   │   │
│   │   ├── rviz/
│   │   │   └── turtlebot3_navigation.rviz
│   │   │
│   │   ├── launch/
│   │   │   ├── turtlebot3_slam.launch
|   |   |   ├──  ...
│   │   │
│   │   └── ...
│   │
│   ├── turtlebot3_teleop/
│   │   │
│   │   ├── launch/
│   │   │   └── turtlebot3_teleop_key.launch
|   |   ├── ...
│
├── turtlebot3_simulations/
│   │
│   ├── .github/
│   │   │
│   │   └── workflows/
│   │       └── ros-ci.yml
│   │
│   │
│   ├── turtlebot3_fake/
│   │   │
│   │   ├── include/
│   │   │   │
│   │   │   └── turtlebot3_fake/
│   │   │       └── turtlebot3_fake.h
│   │   │
│   │   │
│   │   ├── launch/
│   │   │   └── turtlebot3_fake.launch
│   │   │
│   │   ├── rviz/
│   │   │   └── turtlebot3_fake.rviz
│   │   │
│   │   ├── src/
│   │   │   └── turtlebot3_fake.cpp
│   │   │
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── turtlebot3_gazebo/
│   │   │
│   │   ├── launch/
│   │   │   ├── turtlebot3_myworld.launch
│   │   │   └── ...
│   │   │
│   │   ├── worlds/
│   │   │   ├── myworld.world
│   │   │   └── ...

```

## packages used in this project
- Gmapping:  perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
- Turtlebot3_teleop:   manually control a robot using keyboard commands.
- Turtlebot3_gazebo: Turtlebot3 robot .
- dwa_local_planner: used for navigation , The dwa_local_planner package provides a controller that drives a mobile base in the plane. This controller serves to connect the path planner to the robot. Using a map, the planner creates a kinematic trajectory for the robot to get from a start to a goal location.


## Setup [ubuntu 20.04]

  ### Build
  1. Install dependencies
   ```
 $ sudo apt-get update && sudo apt-get upgrade -y
 $ sudo apt-get install ros-${ROS_DISTRO}-map-server
 $ sudo apt-get install ros-${ROS_DISTRO}-move-base
  ```

  2. Clone the project 
 ```
 $ mkdir -p ~/catkin_ws/src/ && cd ~/catkin_ws/src/
 $ git clone https://github.com/saad19ever/Home_service_robot.git
 ```
 3. build catkin workspace
 ```
 $ cd ..
 $ catkin_make
 ```

  ### Launch
  1.change permission to execute the scripts
  ```
  chmod +x src/scripts/add_marker.sh /
  chmod +x src/scripts/pick_objects.sh /
  chmod +x src/scripts/test_navigation.sh /
  chmod +x src/scripts/test_slam.sh /
  chmod +x src/scripts/home_service.sh /
  ```
  
   #### shell scripts used
   * `add_marker.sh` - script for testing add_marker node
   * `pick_objects.sh` - script for testing pick_objects 
   * `test_navigation.sh` - script for testing navigation
   * `test_slam.sh` - script for performing SLAM and drawing the map
   * `home_service.sh` - main script for the home-service-robot

   2. launch scripts
   
   To map the environment run test_slam script which will launch the world in gazebo , spawn turtlebot3 robot , launch turtlebot3_teleop_key to manually navigate the robot and gmapping node to generate the map using odometry and laser sensor.
    
 
     export TURTLEBOT3_MODEL=waffle_pi
    cd ~/catkin_ws/src/scripts
    ./test_slam.sh
     
   I already generated the map and saved in the map directory by running 
   `rosrun map_server map_saver -f <map-location-and-name>`

 To run the main project , run the Home_service script which in the turn will  simulate a home service robot capable of autonomously navigating to pick up and deliver virtual objects, communication was established by making add_markers subscribe to the AMCL_topic .
 to change the pick up and the drop off location modify the `add_marker_config.yaml` file under congif directory
 ```
export TURTLEBOT3_MODEL=waffle_pi
cd ~/catkin_ws/src/scripts
./home_service.sh
```

 
