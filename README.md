# Introduction to Robotics - Ur5 project

### Requirements

For running each sample code:

- `Ros Noetic:` http://wiki.ros.org/noetic/Installation
- `Gazebo:` https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
- `Yolov5` https://github.com/ultralytics/yolov5
- `Catkin` https://catkin-tools.readthedocs.io/en/latest/

### Installation and setup

we **suggest** to follow the locosim readme until this [point]: https://github.com/mfocchi/locosim#python, for download and setup all the catkin - Ros workspace.

For gazebo and Yolov5 , follow their own tutorial .

Once you have done the previous step :

#### Setup catkin workspace and clone the repository

```
mkdir -p ~/ros_ws/src
```

```
cd ~/ros_ws/src
```

Now you need to call the following line manually (next you will see that it will be done automatically in the .bashrc)

```
source /opt/ros/ROS_VERSION/setup.bash
```

```
cd ~/ros_ws/
```

```
 catkin_make
```

```
 cd ~/ros_ws/src/ 
```

Now you can clone the repository inside the ROS workspace you just created:


```
git clone https://github.com/MattiaRigon/progetto_robotica.git
```

Once you have cloned it 
```
cd $PWD/progetto_robotica
```
```
git submodule update --init --recursive
```

```
cd ~/ros_ws/ 
```

```
catkin_make install
```

Now you can continue to follow locosim https://github.com/mfocchi/locosim#configure-environment-variables , but remeber to change the locosim varivable in the .bashrc like this : 

```
export LOCOSIM_DIR=$HOME/ros_ws/src/progetto_robotica/locosim
```

### Run the project

Move inside this directory:

```
cd $HOME/ros_ws/src/progetto_robotica/locosim/robot_control/lab_exercises/lab_palopoli/
```
Run the ur5_generic file that will launch gazebo,rviz, and setup all the environment (pay attention to setup properly in params.py the params for run the ur5_generic.py, for example flag real_robot)

```
python3 ur5_generic.py
```

If you are in simulation:

```
cd $HOME/ros_ws/src/progetto_robotica/spawnLego_pkg/src/
```

```
python3 SpawnaLego.py
```
This script will ask you to input the number of the assigment that you want to perform :
- Assignment 1 : it will spawn only one Lego ,which is positioned with its base “naturally” in contact with the ground.
- Assignment 2 : it will spawn multiple objects on the initial stand, one for each class.
- Assignment  3 : it will spawn multiple objects on the initial stand, and there can be more than one object for each class, object could be lying on one of its lateral sides or on its top.


Run the script that make the robot move : 

```
cd $HOME/ros_ws/
```

```
rosrun publisher_pkg publisher_node
```

Run the vision script:

```
cd $HOME/ros_ws/src/progetto_robotica/vision/
```

```
python3 vision.py
```

### Contributors

| Name                 | Matricola | Github                               |
|----------------------|-----------|--------------------------------------|
|   Rigon Mattia       | 217868    | https://github.com/MattiaRigon  |
|   Simone Roman       |           | https://github.com/Sro552     |
|   Bonetto Stefano    |           | https://github.com/stefanoobonetto  |


