#include <DiffDrive_hardware_interface/DiffDrive_hardware_interface.h>
// #include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DiffDrive_hardware_interface");
    ros::NodeHandle nh;
    DiffDrive_hardware_interface::DiffDriveHardwareInterface rhi(nh);

    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}