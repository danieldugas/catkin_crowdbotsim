#include <DiffDrivecpp/DiffDrive.h>

namespace DiffDrivecpp
{

    Joint::Joint()
    {
        name_ = "";
        cmd_ = 0;
    }
    Joint::Joint(std::string name)
    {
        name_ = name;
        cmd_ = 0;
    }

    void Joint::actuate(double cmd)
    {
        cmd_ = cmd;
    }

    double Joint::read()
    {
        return cmd_;
    }

    DiffDrive::DiffDrive() 
    {
        ros::NodeHandle *nh = new ros::NodeHandle();
        nh_ = *nh;

        for(int i = 0; i < 2; i++)
        { 
            joints_[i] = Joint();
        }
    }

    DiffDrive::DiffDrive(ros::NodeHandle& nh) : nh_(nh) 
    {
        std::vector<std::string> joint_names_;
        nh_.getParam("/DiffDrive/hardware_interface/joints", joint_names_);
        for(int i = 0; i < 2; i++)
        {
            joints_[i] = Joint(joint_names_[i]);
        }
    }

    Joint DiffDrive::getJoint(std::string joint_name)
    {
        for(int i = 0; i < 2; i++)
        {
            if(joint_name == joints_[i].name_)
                return joints_[i];
        }
        throw "bad joint name";
    }

    double DiffDrive::read(std::string joint_name)
    {
        for(int i = 0; i < 2; i++)
        {
            if(joint_name == joints_[i].name_)
                return joints_[i].read();
        }
        throw "bad joint name";
    }

    void DiffDrive::write(std::string joint_name, double cmd_)
    {
        for(int i = 0; i < 2; i++)
        {
            if(joint_name == joints_[i].name_)
                return joints_[i].actuate(cmd_);
        }
        throw "bad joint name";
    }

}