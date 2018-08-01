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

Then select **Create New MoveIt Configuration Package**.
In **Load a URDF...*, browse to find `PSM1.urdf.xacro` then **Load Files**.  Older
versions of MoveIt didn't support xacro but this seems to be supported now.