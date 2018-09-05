## MoveIt experimentation

### Installation

Testing on Ubuntu 18.04 with ROS melodic.  Install as many packages as possible:
```sh
 sudo apt install ros-melodic-desktop-full 
 sudo apt install ros-melodic-ros-controllers 
 sudo apt install ros-melodic-moveit*
 ```
 
### dVRK PSM
 
Install dVRK code from https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild
 
dVRK config files are located in `src/dvrk-ros/dvrk_model/model`. Testing with
`PSM1.urdf.xacro` using MoveIt Setup Assistant (see http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html).

```sh
roslaunch moveit_setup_assistant setup_assistant.launch
```

**Step 1:** Select **Create New MoveIt Configuration Package**.
In **Load a URDF...**, browse to find `PSM1.urdf.xacro` then **Load Files**.  Older
versions of MoveIt didn't support xacro but this seems to be supported now.

**Step 2:** Generate Self Collision Matrix.  Since the PSM can't really hit itself, 
there's no need to change the density, just hit **Generate Collision Matrix**.

**Step 3:** Virtual Joint.   The tutorial recommends to create a virtual joint to
"world" but there's already one in the PSM1.urdf.xacro.

**Step 4:** Planning groups.  The goal here is to separate the joints between the
arm and the gripper.  For the first group, select all joints except `jaw`, 
`jaw_mimic_1` and `jaw_mimic_2` and call it `psm1_arm`.   Then create a second 
group called `psm1_gripper` with all the `jaw` joints.

To save the configuration files, use package name `dvrk_psm1_moveit_config`.
Then remember to build the workspace and source your `setup.bash`:
```sh
catkin build
source ~/catkin_ws/devel/setup.bash
```

To test the configuration files, launch RViz:
```sh
roslaunch dvrk_psm1_moveit_config demo.launch
```

### ros control with crtk hw interface

Code is under git: 

To start testing, there is a launch file with dVRK console PSM1 in simulation mode, RViz and crtk controller:
```sh
roslaunch dvrk_psm1_moveit_config dvrk_psm1_sim_moveit_rviz.launch
```

To switch controller (not working yet!, maybe something related to controllers.yaml):
```sh
rosservice call /dvrk/PSM1/controller_manager/list_controller_types "{}" 
rosservice call /dvrk/PSM1/controller_manager/switch_controller
 ```