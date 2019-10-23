#ifndef DIFFDRIVE_CPP
#define DIFFDRIVE_CPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace DiffDrivecpp{

    class Joint
    {
        public:
            Joint();
            Joint(std::string name);
            std::string name_;
            void actuate(double cmd);
            double read();

        private:
            double cmd_;
    };

    class DiffDrive
    {
        public:
            DiffDrive();
            DiffDrive(ros::NodeHandle& nh);
            Joint getJoint(std::string joint_name);
            double read(std::string joint_name);
            void write(std::string joint_name, double cmd);

        protected:
            ros::NodeHandle nh_;

        private:
            Joint joints_[2];


    };
}


#endif