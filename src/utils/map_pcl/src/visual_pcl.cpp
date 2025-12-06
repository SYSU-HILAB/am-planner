#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

std::string pcd_file_path, frame_id;
Eigen::Vector3d object_axis, object_p = Eigen::Vector3d(0, 0, 0);
double object_theta;
ros::Publisher pcl_pub;
bool obstacle = false;
Eigen::Matrix3d current_R1 = Eigen::Matrix3d::Identity();
Eigen::Matrix3d current_R2 = Eigen::Matrix3d::Identity();
Eigen::Matrix3d current_R_gate = Eigen::Matrix3d::Identity();  // 竞速圈的旋转矩阵
Eigen::Vector3d plane1_p = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d plane2_p = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d gate_p = Eigen::Vector3d(0, 0, 0);  // 竞速圈的中心位置
std::vector<std::pair<double, double>> obstacle_points;

void plane1OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  plane1_p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                             msg->pose.pose.position.z);

  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  current_R1 = q.toRotationMatrix();
}

void plane2OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  plane2_p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                             msg->pose.pose.position.z);

  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  current_R2 = q.toRotationMatrix();
}

void gateOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gate_p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z - 0.3);

  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  current_R_gate = q.toRotationMatrix();
}

void calPose(const Eigen::Vector3d& axis, const double& theta, Eigen::Quaterniond& q,
             Eigen::Vector3d& zd)
{
  Eigen::Quaterniond land_q;
  land_q.w() = cos(theta / 2);
  land_q.x() = axis(0) * sin(theta / 2);
  land_q.y() = axis(1) * sin(theta / 2);
  land_q.z() = axis(2) * sin(theta / 2);
  q = land_q;
  Eigen::MatrixXd land_R = land_q.toRotationMatrix();
  zd = land_R.col(2).normalized();
}

void genPlaneCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  int resolution = 100;
  int length = 1;
  int width = 1;
  int depth = 10;
  double plane_z = 0.0;

  // 生成第一个平面
  for (int i = -resolution / 2; i < resolution / 2; ++i)
  {
    for (int j = -resolution / 2; j < resolution / 2; ++j)
    {
      Eigen::Vector3d vec;
      vec.x() = static_cast<float>(j) / resolution * length;
      vec.y() = static_cast<float>(i) / resolution * width;
      vec.z() = 0;

      vec = current_R1 * vec;
      vec.x() += plane1_p.x();
      vec.y() += plane1_p.y();
      vec.z() += plane1_p.z() + plane_z;

      pcl::PointXYZ point;
      point.x = vec.x();
      point.y = vec.y();
      point.z = vec.z();

      for (int k = 1; k <= resolution; ++k)
      {
        pcl::PointXYZ tmp_point = point;
        tmp_point.z = vec.z() * k / resolution;
        cloud.points.push_back(tmp_point);
      }
      cloud.points.push_back(point);
    }
  }

  // 生成第二个平面
  for (int i = -resolution / 2; i < resolution / 2; ++i)
  {
    for (int j = -resolution / 2; j < resolution / 2; ++j)
    {
      Eigen::Vector3d vec;
      vec.x() = static_cast<float>(j) / resolution * length;
      vec.y() = static_cast<float>(i) / resolution * width;
      vec.z() = 0;

      vec = current_R2 * vec;
      vec.x() += plane2_p.x();
      vec.y() += plane2_p.y();
      vec.z() += plane2_p.z() + plane_z;

      pcl::PointXYZ point;
      point.x = vec.x();
      point.y = vec.y();
      point.z = vec.z();

      for (int k = 1; k <= resolution; ++k)
      {
        pcl::PointXYZ tmp_point = point;
        tmp_point.z = vec.z() * k / resolution;
        cloud.points.push_back(tmp_point);
      }
      cloud.points.push_back(point);
    }
  }
}

void genCylindricalCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, double x, double y)
{
  const double width = 0.6;
  const double length = 0.6;
  const double height = 1.8;

  for (int i = 0; i < 10; ++i)
  {
    for (int j = 0; j < 10; ++j)
    {
      for (int k = 0; k < 30; ++k)
      {
        double px = x - width / 2 + width * i / (10 - 1);
        double py = y - length / 2 + length * j / (10 - 1);
        double pz = height * k / (30 - 1);

        pcl::PointXYZ point;
        point.x = px;
        point.y = py;
        point.z = pz;
        cloud.points.push_back(point);
      }
    }
  }
}

void genGateCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  const double gate_width = 0.70;   // 100cm宽
  const double gate_height = 0.35;  // 40cm高
  const double frame_width = 0.1;   // 10cm边框
  const double thickness = 0.05;    // 5cm厚度
  const int resolution = 100;       // 点云分辨率

  // 预分配空间

  // 生成竞速圈的点云
  for (int i = 0; i < resolution; i++)
  {
    for (int j = 0; j < resolution; j++)
    {
      double i_ratio = static_cast<double>(i) / resolution;
      double j_ratio = static_cast<double>(j) / resolution;

      // 四个边框的局部坐标
      Eigen::Vector3d point_local[4];

      // 上边框
      point_local[0] << 0,
          -gate_width / 2 - frame_width + (gate_width + 2 * frame_width) * i_ratio,
          gate_height / 2 + frame_width * j_ratio;

      // 下边框
      point_local[1] << 0,
          -gate_width / 2 - frame_width + (gate_width + 2 * frame_width) * i_ratio,
          -gate_height / 2 - frame_width * j_ratio;

      // 左边框
      point_local[2] << 0, -gate_width / 2 - frame_width * i_ratio,
          -gate_height / 2 + gate_height * j_ratio;

      // 右边框
      point_local[3] << 0, gate_width / 2 + frame_width * i_ratio,
          -gate_height / 2 + gate_height * j_ratio;

      // 添加厚度
      for (int t = 0; t < 5; t++)
      {
        double thick = thickness * t / 5 - thickness / 2;

        for (int k = 0; k < 4; k++)
        {
          Eigen::Vector3d p = point_local[k];
          p.x() = thick;

          // 转换到世界坐标系
          Eigen::Vector3d point_world = current_R_gate * p + gate_p;

          // 转换为PCL点类型并添加到点云
          pcl::PointXYZ pcl_point;
          pcl_point.x = point_world.x();
          pcl_point.y = point_world.y();
          pcl_point.z = point_world.z();
          cloud.points.push_back(pcl_point);
        }
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_pcl_node");
  ros::NodeHandle nh("~");

  nh.param("pcd_file_path", pcd_file_path, std::string("package://map_pcl/pcd/map.pcd"));
  nh.param("frame_id", frame_id, std::string("world"));

  plane1_p = Eigen::Vector3d(0, 0, 0);
  plane2_p = Eigen::Vector3d(0, 0, 0);
  gate_p = Eigen::Vector3d(0, 0, 0);
  object_axis = Eigen::Vector3d(0, 0, 1);
  object_theta = 0.0;
  current_R1 = Eigen::Matrix3d::Identity();
  current_R2 = Eigen::Matrix3d::Identity();
  current_R_gate = Eigen::Matrix3d::Identity();

  int num_obstacles;
  nh.param("num_obstacles", num_obstacles, 0);

  for (int i = 0; i < num_obstacles; i++)
  {
    double x, y;
    std::string x_param = "obstacle_" + std::to_string(i) + "_x";
    std::string y_param = "obstacle_" + std::to_string(i) + "_y";

    if (nh.getParam(x_param, x) && nh.getParam(y_param, y))
    {
      obstacle_points.push_back(std::make_pair(x, y));
    }
    else
    {
      ROS_WARN("Failed to get parameters for obstacle %d", i);
    }
  }

  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
  ros::Subscriber plane1_odom_sub = nh.subscribe("plane1_odom", 1, plane1OdomCallback);
  ros::Subscriber plane2_odom_sub = nh.subscribe("plane2_odom", 1, plane2OdomCallback);
  ros::Subscriber gate_odom_sub = nh.subscribe("gate_odom", 1, gateOdomCallback);

  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
    genPlaneCloud(plane_cloud);
    cloud = plane_cloud;

    for (const auto& point : obstacle_points)
    {
      genCylindricalCloud(cloud, point.first, point.second);
    }

    genGateCloud(cloud);

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = ros::Time::now();

    pcl_pub.publish(output);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}