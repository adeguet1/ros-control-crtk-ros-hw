/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <ros-control-crtk-ros-hw/crtk_ros_hardware_interface.h>

#include <thread>

using namespace ros_control_crtk;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "crtk_ros_hw_controller");
	ros::NodeHandle node_handle;

    crtkROSHardwareInterface crtk_ros_interface(node_handle);
    std::thread * hardware_thread =
        new std::thread(boost::bind(&crtkROSHardwareInterface::loop, &crtk_ros_interface));

	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::waitForShutdown();

    // hardware_thread.halt();
	exit(0);
}
