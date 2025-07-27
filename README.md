# 41068 Ignition Bringup

Bringup for *41068 Robotics Studio I*. Launches a turtlebot4 in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

Worlds are build from [Gazebo Fuel](https://app.gazebosim.org/fuel/models).

## Installation

First install some dependencies:

* Install Gazebo
  ```bash
  sudo apt-get update && sudo apt-get install wget
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update && sudo apt-get install ignition-fortress
  ```
* Install development tools and robot localisation
  ```bash
  sudo apt install ros-dev-tools ros-humble-robot-localization
  sudo apt install ros-humble-ros-ign ros-humble-ros-ign-interfaces
  sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
  ```
  

Now install this package:
* Create a new colcon workspace
  ```bash
  mkdir -p 41068_ws/src
  ```
* Copy this package to the `src` directory in this workspace
* Build package
  ```bash
  source /opt/ros/humble/setup.bash
  cd 41068_ws
  colcon build --symlink-install
  ```
* Source workspace (if you add this to your ~/.bashrc, then you don't need to do this each time)
  ```bash
  source ~/41068_ws/install/setup.bash
  ```
* Launch basic trees world
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py
  ```
* As above with SLAM and autonomous navigation
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true
  ```
* Change world with `world` argument. Must be the name of a `.sdf` file in `worlds`, but without file extension e.g.
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=large_demo
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