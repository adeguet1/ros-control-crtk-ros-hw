/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <controller_manager/controller_manager.h>
#include <ros-control-crtk-ros-hw/crtk_ros_hardware_interface.h>

namespace ros_control_crtk {

    crtkROSHardwareInterface::crtkROSHardwareInterface(ros::NodeHandle & node_handle):
        m_node_handle(node_handle)
    {
        init();
        ROS_INFO_NAMED("crtk_hardware_interface", "Loaded crtk_hardware_interface.");
    }

    void crtkROSHardwareInterface::current_state_callback(const std_msgs::String & state)
    {
        if (state.data == "READY") {
            m_crtk_node_ready = true;
            // if the crtk node was found, set current servo_jp based on measured_js
            if (m_crtk_node_found) {
                std::copy(m_measured_js.position.begin(),
                          m_measured_js.position.end(),
                          m_servo_jp.position.begin());
            }
        } else {
            m_crtk_node_ready = false;
        }
    }
    
    void crtkROSHardwareInterface::measured_js_callback(const sensor_msgs::JointState & measured_js)
    {
        // check if we had already found the crtk controller
        if (!m_crtk_node_found) {
            initialize_from_crtk_node(measured_js);
        }
        // preserve local copy for controllers
        copy_measured_js_from_crtk_node(measured_js);
    }

    void crtkROSHardwareInterface::initialize_from_crtk_node(const sensor_msgs::JointState & measured_js)
    {
        // let's assume everything is going to work
        m_crtk_node_found = true;

        // check all the vectors size
        // -- name
        {
            const size_t name_size = measured_js.name.size();
            if ((name_size != m_number_of_joints)
                && (name_size != 0)) {
                ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                "measured_js.name vector has wrong size");
                m_crtk_node_found = false;
            }
            // check that the joint names are correct
            for (size_t i = 0;
                 i < name_size;
                 ++i) {
                if (measured_js.name[i] != m_measured_js.name[i]) {
                    ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                    "measured_js.name vector doesn't match with the expected one");
                    m_crtk_node_found = false;
                }
            }
        }
        // -- position
        {
            const size_t position_size = measured_js.position.size();
            if ((position_size != m_number_of_joints)
                && (position_size != 0)) {
                ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                "measured_js.position vector has wrong size");
                m_crtk_node_found = false;
            }
        }
        // -- velocity
        {
            const size_t velocity_size = measured_js.velocity.size();
            if ((velocity_size != m_number_of_joints)
                && (velocity_size != 0)) {
                ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                "measured_js.velocity vector has wrong size");
                m_crtk_node_found = false;
            }
        }
        // -- effort
        {
            const size_t effort_size = measured_js.effort.size();
            if ((effort_size != m_number_of_joints)
                && (effort_size != 0)) {
                ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                "measured_js.effort vector has wrong size");
                m_crtk_node_found = false;
            }
        }
        // copy joint state
        copy_measured_js_from_crtk_node(measured_js);

        // set initial values to send to robot based on current state
        std::copy(m_measured_js.position.begin(), m_measured_js.position.end(), m_servo_jp.position.begin());
    }

    void crtkROSHardwareInterface::copy_measured_js_from_crtk_node(const sensor_msgs::JointState & measured_js)
    {
        // -- position
        {
            const size_t position_size = measured_js.position.size();
            if (position_size != 0) {
                if (position_size == m_measured_js.position.size()) {
                    std::copy(measured_js.position.begin(), measured_js.position.end(),
                              m_measured_js.position.begin());
                } else {
                    ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                    "measured_js.position has wrong size");
                }
            }
        }
        // -- velocity
        {
            const size_t velocity_size = measured_js.velocity.size();
            if (velocity_size != 0) {
                if (velocity_size == m_measured_js.velocity.size()) {
                    std::copy(measured_js.velocity.begin(), measured_js.velocity.end(),
                              m_measured_js.velocity.begin());
                } else {
                    ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                    "measured_js.velocity vector has wrong size");
                }
            }
        }
        // -- effort
        {
            const size_t effort_size = measured_js.effort.size();
            if (effort_size != 0) {
                if (effort_size == m_measured_js.effort.size()) {
                    std::copy(measured_js.effort.begin(), measured_js.effort.end(),
                              m_measured_js.effort.begin());
                } else {
                    ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                    "measured_js.effort vector has wrong size");
                }
            }
        }
    }

    void crtkROSHardwareInterface::init(void)
    {
        m_crtk_node_found = false;
        m_crtk_node_ready = false;
        m_servo_jp_interface_running = false;

        // add subscriber to current state from crtk node - this has to be standardized
        m_current_state_subscriber = m_node_handle.subscribe("current_state", 1,
                                                             &crtkROSHardwareInterface::current_state_callback, this);

        // add subscriber to get joint state from crtk node
        m_measured_js_subscriber = m_node_handle.subscribe("measured_js", 1,
                                                           &crtkROSHardwareInterface::measured_js_callback, this);

        // add publishers to send servo commands to crtk node
        m_servo_jp_publisher = m_node_handle.advertise<sensor_msgs::JointState>("servo_jp", 1, false);

        m_number_of_joints = 6;
        m_measured_js.name.resize(m_number_of_joints);
        m_measured_js.name[0] = "outer_yaw";
        m_measured_js.name[1] = "outer_pitch";
        m_measured_js.name[2] = "outer_insertion";
        m_measured_js.name[3] = "outer_roll";
        m_measured_js.name[4] = "outer_wrist_pitch";
        m_measured_js.name[5] = "outer_wrist_yaw";
        m_measured_js.position.resize(m_number_of_joints);
        m_measured_js.velocity.resize(m_number_of_joints);
        m_measured_js.effort.resize(m_number_of_joints);

        // resize servo objects too
        m_servo_jp.name.resize(m_number_of_joints);
        m_servo_jp.position.resize(m_number_of_joints);
        m_servo_jp.velocity.resize(0);
        m_servo_jp.effort.resize(0);
        // copy names only the first time, we don't support name/size change at runtime
        std::copy(m_measured_js.name.begin(), m_measured_js.name.end(), m_servo_jp.name.begin());

        // register all interfaces for ros control
        for (size_t i = 0;
             i < m_number_of_joints;
             ++i) {
            ROS_DEBUG_STREAM_NAMED("crtk_hardware_interface",
                                   "Loading joint name: " << m_measured_js.name[i]);

            // create joint state interface
            m_joint_state_interface
                .registerHandle(hardware_interface::JointStateHandle(m_measured_js.name[i],
                                                                     &m_measured_js.position[i],
                                                                     &m_measured_js.velocity[i],
                                                                     &m_measured_js.effort[i]));

            // create position joint interface
            m_position_joint_interface
                .registerHandle(hardware_interface::JointHandle(m_joint_state_interface.getHandle(m_measured_js.name[i]),
                                                                &m_servo_jp.position[i]));
        }

        // register ros control interfaces
        registerInterface(&m_joint_state_interface);
        registerInterface(&m_position_joint_interface);
    }

    void crtkROSHardwareInterface::read(void)
    {
        // maybe we should add some code to check that we're still getting some data from the crtk node
        // estimating rate of measured_js topic might also be needed
    }

    void crtkROSHardwareInterface::write(void)
    {
        if (m_servo_jp_interface_running) {
            if (m_crtk_node_ready) {
                m_servo_jp_publisher.publish(m_servo_jp);
                std::cerr << "[" << m_measured_js.position[2] << "]";
            } else {
                ROS_ERROR_STREAM_THROTTLE_NAMED(60, "crtk_hardware_interface",
                                                "crtk node is not ready");
            }
        }
    }

    void crtkROSHardwareInterface::loop(void)
    {
        controller_manager::ControllerManager * cm
            = new controller_manager::ControllerManager(this, m_node_handle);

        ros::Time previous_time = ros::Time::now();
        while (ros::ok()) {
            this->read();
            ros::Time now = ros::Time::now();
            cm->update(now, now - previous_time);
            previous_time = now;
            this->write();

            // there should be some kind of sleep here, maybe try to find frenquency of crtk node - load from rosparam!
            ros::Duration(0.01).sleep();
        }

        delete cm;
    }

    bool crtkROSHardwareInterface::canSwitch(const std::list<hardware_interface::ControllerInfo> & start_list,
                                             const std::list<hardware_interface::ControllerInfo> & stop_list) const
    {
        for (std::list<hardware_interface::ControllerInfo>::const_iterator iter = start_list.begin();
             iter != start_list.end();
             ++iter) {
            if (iter->name == "hardware_interface::PositionJointInterface") {
                if (m_servo_jp_interface_running) {
                    ROS_ERROR("%s: an interface of that type (%s) is already running",
                              iter->name.c_str(),
                              iter->type.c_str());
                    return false;
                }
#if 0
                if (!m_crtk_node_found) {
                    ROS_ERROR("%s: can not switch to interface of that type (%s) as long as the crtk node is not found",
                              iter->name.c_str(),
                              iter->type.c_str());
                    return false;
                }
#endif
            }
        }

        // true otherwise
        return true;
    }

    void crtkROSHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> & start_list,
                                            const std::list<hardware_interface::ControllerInfo> & stop_list)
    {
        std::cerr << " doSwitch " << start_list.size() << " " << stop_list.size() << std::endl;

        for (std::list<hardware_interface::ControllerInfo>::const_iterator iter = stop_list.begin();
             iter != stop_list.end();
             ++iter) {
            std::cerr << "To stop: " << iter->name << " of type " << iter->type << std::endl;
            if (iter->name == "pos_based_pos_traj_controller") {
                m_servo_jp_interface_running = false;
                ROS_DEBUG("Stopping position interface");
            }
        }
        for (std::list<hardware_interface::ControllerInfo>::const_iterator iter = start_list.begin();
             iter != start_list.end();
             ++iter) {
            std::cerr << "To start: " << iter->name << " of type " << iter->type << std::endl;
#if 0
            if (!m_crtk_node_found) {
                ROS_ERROR("%s: can not switch to interface of that type (%s) as long as the crtk node is not found",
                          iter->name.c_str(),
                          iter->type.c_str());
            }
#endif
            if (iter->name == "pos_based_pos_traj_controller") {
                m_servo_jp_interface_running = true;
                ROS_DEBUG("Starting position interface");
            }
        }
    }

} // namespace
