#ifndef ROS_CONTROL_DIFFDRIVE_HARDWARE_INTERFACE_H
#define ROS_CONTROL_DIFFDRIVE_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <DiffDrivecpp/DiffDrive.h>
#include <DiffDrive_hardware_interface/DiffDrive_hardware.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;

namespace DiffDrive_hardware_interface
{
    static const double VELOCITY_STEP_FACTOR = 10;

    class DiffDriveHardwareInterface: public DiffDrive_hardware_interface::DiffDriveHardware
    {
        public:
            DiffDriveHardwareInterface(ros::NodeHandle& nh);
            ~DiffDriveHardwareInterface();
            void init();
            void update(const ros::TimerEvent& e);
            void read();
            void write(ros::Duration elapsed_time);

        protected:
            DiffDrivecpp::DiffDrive d_drv;
            ros::NodeHandle nh_;
            ros::Timer non_realtime_loop_;
            ros::Duration control_period_;
            ros::Duration elapsed_time_;
            double loop_hz_;
            boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
            double p_error_, v_error_, e_error_;
    };

}

#endif