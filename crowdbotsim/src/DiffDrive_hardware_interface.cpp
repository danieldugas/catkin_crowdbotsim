#include <sstream>
#include <DiffDrive_hardware_interface/DiffDrive_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <DiffDrivecpp/DiffDrive.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
// using joint_limits_interface::PositionJointSoftLimitsHandle;
// using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace DiffDrive_hardware_interface
{
    DiffDriveHardwareInterface::DiffDriveHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/DiffDrive/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &DiffDriveHardwareInterface::update, this);
    }

    DiffDriveHardwareInterface::~DiffDriveHardwareInterface() {

    }

    void DiffDriveHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/DiffDrive/hardware_interface/joints", joint_names_);

        d_drv = DiffDrivecpp::DiffDrive(nh_);

        num_joints_ = joint_names_.size();

        // Resize vectors
        // joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        // joint_effort_.resize(num_joints_);
        // joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        // joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {
            DiffDrivecpp::Joint joint = d_drv.getJoint(joint_names_[i]);

            // Create joint state interface
            JointStateHandle jointStateHandle(joint.name_, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // Create velocity joint interface
            JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
            // JointLimits limits;
            // SoftJointLimits softLimits;
            // getJointLimits(joint.name, nh_, limits)
            // PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            // positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            velocity_joint_interface_.registerHandle(jointVelocityHandle);

            // Create effort joint interface
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&effort_joint_interface_);
        // registerInterface(&positionJointSoftLimitsInterface);
    }

    void DiffDriveHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void DiffDriveHardwareInterface::read() {
        for (int i = 0; i < 2; i++) {
            joint_velocity_[i] = d_drv.read(joint_names_[i]);
        }
    }

    void DiffDriveHardwareInterface::write(ros::Duration elapsed_time) {
        // positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
        for (int i = 0; i < 2; i++) {
            d_drv.write(joint_names_[i], joint_velocity_command_[i]);
        }
    }
}