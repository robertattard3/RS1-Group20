# 42043 Ignition Bringup

Bringup for *42043 Robotics Studio I*. Launches a turtlebot4 in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

Worlds are build from [Gazebo Fuel](https://app.gazebosim.org/fuel/models).

## Installation

First install `turtlebot4_simulator` and its prerequisites. Instructions [here](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html). 

* Install Gazebo
  ```bash
  sudo apt-get update && sudo apt-get install wget
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update && sudo apt-get install ignition-fortress
  ```
* Install `turtlebot4_simulator`
  ```bash
  sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
  ```
* Install development tools
  ```bash
  sudo apt install ros-dev-tools
  ```
* Test simulator with default settings:
  ```bash
  ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
  ```
* Test simulator with SLAM and autonomous navigation
  ```bash
  ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
  ```

Now install this package:
* Create a new colcon workspace
  ```bash
  mkdir -p 42043_ws/src
  ```
* Copy this package to the `src` directory in this workspace
* Build package
  ```bash
  source /opt/ros/humble/setup.bash
  cd 42043_ws
  colcon build --symlink-install
  ```
* Source workspace
  ```bash
  source install/setup.bash
  ```
* Launch basic trees world
  ```bash
  ros2 launch 42043_ignition_bringup 42043_ignition.launch.py
  ```

## Parameter settings

Disable safety behaviours (eg. stopping when the real robot might topple):
```bash
ros2 param set /motion_control safety_override full
```

## Errors

### Ogre Exception

If you get an error like:

```bash
[Ogre2RenderEngine.cc:989]  Unable to create the rendering window: OGRE EXCEPTION(3:RenderingAPIException): currentGLContext was specified with no current GL context in GLXWindow::create at /build/ogre-next-UFfg83/ogre-next-2.2.5+dfsg3/RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXWindow.cpp (line 163)
```

I found [this thread](https://robotics.stackexchange.com/questions/111547/gazebo-crashes-immediately-segmentation-fault-address-not-mapped-to-object-0) which suggests to set a bash variable before launching Gazebo:

```bash
export QT_QPA_PLATFORM=xcb
```

### Empty topics

Trying to `ros2 topic echo` something and not getting a result? Could be for several reasons:

* If the robot is still on the dock, it won't publish lidar scans or camera images
* Topics might not be correctly bridging from Gazebo to ROS: check the `.sdf` file, and make sure the name of the `.sdf` is identical to the `world name` tag. For example, `simple_trees.sdf` should have that tag as `<world name="simple_trees">`

### Lidar Scan

There are issues with simulating the lidar:
* [With GPU](https://github.com/turtlebot/turtlebot4_simulator/issues/85)
* [Without GPU](https://github.com/iRobotEducation/create3_sim/issues/240)