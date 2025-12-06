#ifndef DELTA_DISPLAY_H
#define DELTA_DISPLAY_H

#include <delta_display/flatness.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

#define PI 3.1415926
#define SQRT_3 sqrt(3.0)
#define ang2rad 0.0174532925199

namespace delta_display
{
class DeltaDisplay
{
 public:
  double scale, R, r, L, l;
  double mass;
  double update_rate;
  bool debug;
  double end_effector_x, end_effector_y, end_effector_z;
  ros::Subscriber odom_sub;  // subscribe the location of plane
  ros::Subscriber sub_end;   // subscribe the end point
  ros::Subscriber position_cmd_sub;
  ros::Publisher joint_state_pub;  // publish the radian of joints
  ros::Subscriber sub_angles;      // subscribe the radian of the joints angles
  ros::Publisher bottom_publisher;
  ros::Publisher bottom_odom_publisher, quad_odom_publisher, quad_yz_publisher;
  geometry_msgs::PointStamped point_src, point_tgt;  // point from odom to world
  geometry_msgs::TransformStamped tf_quadrotor, tf11, tf12, tf21, tf22, tf31, tf32;
  nav_msgs::Odometry end_odom, quad_odom;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  Eigen::Matrix4d body_T_end;
  sensor_msgs::JointState joint;
  std::vector<geometry_msgs::TransformStamped> tf_vec;
  double x, y, z;
  Eigen::Vector3d phi;
  visualization_msgs::Marker block;  // bottom
  double bottom_x, bottom_y, bottom_z;
  flatness::FlatnessMap flatness;
  bool used_so3control;
  bool debug_endEffector;

 public:
  DeltaDisplay(ros::NodeHandle n);  // Constructor
  void tfInit();                    // initialize the tf
  void IK_kin(double y, double x, double z, Eigen::Vector3d& theta);
  void FK_kin(double theta1, double theta2, double theta3, Eigen::Vector3d& position);
  void drivenArmsQuaternion(Eigen::Vector3d vector,
                            geometry_msgs::Quaternion& quaternion);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void anglesCallback(const std_msgs::Float32MultiArray::ConstPtr& angles_msg);
  void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void endCallback(const geometry_msgs::Point::ConstPtr& point_msg);
  void visual_bottom(double x, double y, double z);
  void visual_bottom();
  void getJointPoints(double x, double y, double z, const double* theta,
                      std::vector<Eigen::Vector3d>& A, std::vector<Eigen::Vector3d>& B,
                      std::vector<Eigen::Vector3d>& C, std::vector<Eigen::Vector3d>& Cl,
                      std::vector<Eigen::Vector3d>& Cr, std::vector<Eigen::Vector3d>& Bl,
                      std::vector<Eigen::Vector3d>& Br);
};
}  // namespace delta_display

#endif  // DELTA_DISPLAY_H
