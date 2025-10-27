# ROAMeR

**R**econfigurable
**O**mnidirectional
**A**rticulated
\[leg-wheel\]
**M**obil**e**
**R**obot

## Overview

This project is a tribute to the original developers of ROAMeR.

Check out the [References](#references) below for more information about the
original ROAMeR and its creators.

## Scope

The following content is planned for development in the and in other repos:

* ROS packages
  * `roamr_description`
  * TODO: `roamr_teleop`
  * TODO: `roamr_driver`
  * TODO: `roamr_sim`
  * TBD: Embedded code implementing micro-ROS
* Hardware
  * WIP: Simplified visual geometry
  * TODO: Mechanical hardware design
  * TODO: Electrical hardware design
* Firmware
  * TODO: Embedded code

## ROS Quick Start

### Versions & Installation

This project is built with Ubuntu Noble 24.04 and ROS2 Rolling Ridley. We may fall back to a stable release if development challenges are experienced with Rolling Ridley. Until then, we'll use the latest LTS from Ubuntu with the latest development distro of ROS2.

```bash
lsb_release -d
No LSB modules are available.
Description:    Ubuntu 24.04.3 LTS
```

Follow [these instructions](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html) to install Rolling Ridley from Debian packages. Choose the `ros-rolling-desktop` option for base installation.

Ensure that you set up the following export in `~/.bashrc` or similar solution to source the ROS2 environment in your shell. With ROS2 you also should define a default domain ID for the communications middleware layer. I like 42, but you can use another valid number.

```bash
source /opt/ros/rolling/setup.bash
export ROS_DOMAIN_ID=42
```

### Setup

There are two concepts to grasp with this ROS2 project. 

1. Source code repository context for development
2. ROS2 workspace for building, testing, and locally deploying the project

I like to use the following scheme to check out repos and configure workspaces:

```
/home/user/
  git/
    roamr/
      roamr_description/
      .../
      another_ros_package/
  ros/
    roamr_ws/
      src/
        roamr_description -> ~/git/roamr/roamr_description/
        another_ros_package -> ~/git/roamr/another_ros_package/
        pkg_from_another_repo -> ~/git/another_repo/external_ros_pkg/
```

I check out repos containing ROS2 packages under `git` or `svn` directories and then symlink the packages into the source directory of a ROS2 workspace in the `ros` directory with a suffix `_ws`.

Setting this up involves using the following commands:

```bash
mkdir ~/git && cd ~/git
git clone <url-for-roamr.git>
# Repeat until all repos are cloned

MY_WS="roamr" # Choose your workspace name
mkdir -p ~/ros/$MY_WS/src
cd ~/ros/$MY_WS/src
ln -s ~/git/roamr/robot_description/ robot_description
# Repeat for all ROS2 packages
# Repeat for additional workspaces if desired
```

### Build & Run

ROS2 uses the colcon build system to build and install packages. To run the package, you will need to perform the following steps:

1. Install dependencies
2. Build the packages in a workspace
3. Source the installed setup script
4. Run ROS nodes

First, install and use `rosdep` to install package dependencies for all packages in your workspace. Note that you can also perform these steps on the Git repos and individual packages. Find more info on `rosdep` usage [here](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Rosdep.html). It's worth noting that uninstalled dependencies can be manually installed when discovered through some failure during build, test, or execution of ROS2 packages.

Initial install, init, and update `rosdep`.

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

Then, install dependencies in your workspace.

```bash
MY_WS="roamr" # Choose your workspace name
cd ~/ros/$MY_WS/
rosdep install --from-paths src -y --ignore-src
```

Now you can build the workspace:

```bash
MY_WS="roamr" # Choose your workspace name
cd ~/ros/$MY_WS/
colcon build
```

After a successful build, you must source the installed setup script before running code from this local build. This line may be useful to add to your `~/.bashrc` temporarily while working with the current workspace.

```bash
MY_WS="roamr" # Choose your workspace name
cd ~/ros/$MY_WS/
source install/setup.bash
```

Or, if adding to `~/.bashrc`:

```bash
MY_WS="roamr" # Choose your workspace name
source ~/ros/$MY_WS/install/setup.bash
```

You can now launch some ROS2 nodes from one of the development packages. Let's launch the demo for ROAMeR that's part of the `roamr_description` package.

```bash
# Remember, both ROS2 and your workspace install setup scripts must be sourced here (e.g. in ~/.bashrc)
ros2 launch roamr_description demo.launch.py
```

You should see a couple windows open containing robot visualization and joint state control.

To further inspect the running system you should try the following tools.

```bash
ros2 node list # List executing nodes
ros2 topic list # List published and subscribed topics
ros2 run rqt_graph rqt_graph # View topology of nodes and topics
ros2 run tf2_tools view_frames # Create diagram of published transform frames
```

## References

1. Q. Fu, “Kinematics of Articulated Wheeled Robots: Exploiting Reconfigurability and Redundancy,”
M.S. thesis, Dept. of Mech. Eng., Univ. at Buffalo \(SUNY\), Buffalo, New York, United States, 2008.
\[Online\]. Available: <https://www.academia.edu/2835721/Kinematics_of_Articulated_Wheeled_Robots_Exploiting_Reconfigurability_and_Redundancy>
_Note: PDF full text can be downloaded with a free account_
