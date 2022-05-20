#include "ros/ros.h"
#include "ros/console.h"

int i = 1;
void segFault()
{
    if(i > 5) // trigger a segmentation fault after 5 loop
    {
        int *p = nullptr;
        *p = 10;
    }
    i++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "one_node");
    ros::NodeHandle nh;

    ros::Rate rate(1);
    while (ros::ok())
    {
        ROS_INFO("Spin once. i = %d", i);
        segFault();
        ros::spinOnce();
        rate.sleep();
    }
}