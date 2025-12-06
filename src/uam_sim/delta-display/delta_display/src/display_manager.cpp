#include <ros/ros.h>
#include "delta_display.cpp"

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "delta_display_node");

    ros::NodeHandle n;
    delta_display::DeltaDisplay DD(n);
    ros::spin();

    return 0;
}
