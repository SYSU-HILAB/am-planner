#include <ros/ros.h>
#include <rosbag/bag.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <rosbag/view.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_republisher");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/trajectory", 10);


    std::string package_path = ros::package::getPath("so3_quadrotor_simulator");
    if (package_path.empty()) {
        ROS_ERROR("Failed to get package path");
        return 1;
    }

    std::string bag_file = package_path + "/launch/trajectory.bag"; 

    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagException& e) {
        ROS_ERROR_STREAM("Failed to open rosbag file: " << e.what());
        return 1;
    }

    quadrotor_msgs::PolynomialTrajectory::ConstPtr traj_msg;
    for (rosbag::MessageInstance const& m : rosbag::View(bag)) {
        if (m.getTopic() == "/trajectory") {
            traj_msg = m.instantiate<quadrotor_msgs::PolynomialTrajectory>();
            if (traj_msg != nullptr) {
                quadrotor_msgs::PolynomialTrajectory modified_traj_msg = *traj_msg;
                modified_traj_msg.header.stamp = ros::Time::now();
                traj_pub.publish(modified_traj_msg);
                ROS_INFO("Published modified trajectory message");
                break; // Exit loop after publishing once
            }
        }
    }

    bag.close();

    // Send the same message continuously using while loop

    ros::Rate rate(1);
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        quadrotor_msgs::PolynomialTrajectory modified_traj_msg = *traj_msg;
        modified_traj_msg.header.stamp = start_time;
        traj_pub.publish(modified_traj_msg);
        ROS_INFO("Published modified trajectory message");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
