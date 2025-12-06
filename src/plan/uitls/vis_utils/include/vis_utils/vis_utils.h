#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_
#include <decomp_basis/data_type.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <traj_opt/config.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Visualization
{
 private:
  Config *config;
  ros::NodeHandle nh;
  ros::Publisher trajPub;
  ros::Publisher ellipsoidPub_dilate;
  ros::Publisher quadrotorPub;
  ros::Publisher hPolyPub;
  ros::Publisher activePolyPub;
  ros::Publisher tiltRatePub;
  ros::Publisher thrMagPub;
  ros::Publisher pathPub;
  ros::Publisher ellipsoidPub;
  ros::Publisher ellipsoidPub_end;
  ros::Publisher keyPointsPub;
  ros::Publisher guidePointsPub;
  ros::Publisher spherePub;
  ros::Publisher spherePub_end;
  ros::Publisher spherePub_connect;
  ros::Publisher ellipsoidPathPub1;  // 金黄色路径
  ros::Publisher ellipsoidPathPub2;  // 珊瑚红路径
  ros::Publisher coordAxesPub;       // 坐标轴

  int id = 0;

 public:
  Visualization(Config *conf, ros::NodeHandle &nh, int id) : nh(nh)
  {
    config = conf;
    trajPub = nh.advertise<visualization_msgs::Marker>(
        "/visualization/trajectory" + std::to_string(id), 1);
    ellipsoidPub_dilate = nh.advertise<visualization_msgs::MarkerArray>(
        "/visualization/ellipsoid_dilate" + std::to_string(id), 1);
    ellipsoidPub = nh.advertise<visualization_msgs::MarkerArray>(
        "/visualization/ellipsoid" + std::to_string(id), 1);
    quadrotorPub =
        nh.advertise<visualization_msgs::MarkerArray>("/visualization/quadrotor", 1);
    hPolyPub = nh.advertise<decomp_ros_msgs::PolyhedronArray>(
        "/visualization/polyhedra" + std::to_string(id), 1);
    tiltRatePub = nh.advertise<std_msgs::Float64>("/visualization/tilt_rate", 1);
    thrMagPub = nh.advertise<std_msgs::Float64>("/visualization/thrust_magnitute", 1);
    pathPub = nh.advertise<visualization_msgs::Marker>(
        "/visualization/path" + std::to_string(id), 1);
    keyPointsPub = nh.advertise<visualization_msgs::Marker>(
        "/visualization/keypoints" + std::to_string(id), 1);
    guidePointsPub = nh.advertise<visualization_msgs::Marker>(
        "/visualization/guidepoints" + std::to_string(id), 1);
    ellipsoidPub_end = nh.advertise<visualization_msgs::MarkerArray>(
        "/visualization/ellipsoid_end" + std::to_string(id), 1);
    activePolyPub = nh.advertise<decomp_ros_msgs::PolyhedronArray>(
        "/visualization/active_polyhedron" + std::to_string(id), 1);
    spherePub = nh.advertise<visualization_msgs::MarkerArray>(
        "/visualization/sphere" + std::to_string(id), 1);
    spherePub_end = nh.advertise<visualization_msgs::MarkerArray>(
        "/visualization/sphere_end" + std::to_string(id), 1);
    spherePub_connect = nh.advertise<visualization_msgs::Marker>(
        "/visualization/sphere_connect" + std::to_string(id), 1);
    ellipsoidPathPub1 = nh.advertise<visualization_msgs::Marker>(
        "/visualization/ellipsoid_path1" + std::to_string(id), 1);
    ellipsoidPathPub2 = nh.advertise<visualization_msgs::Marker>(
        "/visualization/ellipsoid_path2" + std::to_string(id), 1);
    coordAxesPub = nh.advertise<visualization_msgs::MarkerArray>(
        "/visualization/coord_axes" + std::to_string(id), 1);
  }

  template <typename Trajectory>
  void visualize(const Trajectory &traj, const int samples = 1000)
  {
    visualization_msgs::Marker trajMarker;
    trajMarker.id = 0;
    trajMarker.header.stamp = ros::Time::now();
    trajMarker.header.frame_id = config->odomFrame;
    trajMarker.pose.orientation.w = 1.00;
    trajMarker.action = visualization_msgs::Marker::ADD;
    trajMarker.type = visualization_msgs::Marker::LINE_STRIP;
    trajMarker.header.frame_id = config->odomFrame;
    trajMarker.ns = "trajectory";
    trajMarker.color.r = config->trajVizRGB(0);
    trajMarker.color.g = config->trajVizRGB(1);
    trajMarker.color.b = config->trajVizRGB(2);
    trajMarker.color.a = 1.00;
    trajMarker.scale.x = config->trajVizWidth;

    double dt;
    dt = traj.getTotalDuration() / samples;

    geometry_msgs::Point point;
    Eigen::Vector3d pos;
    for (int i = 0; i <= samples; i++)
    {
      pos = traj.getPos(dt * i);
      point.x = pos(0);
      point.y = pos(1);
      point.z = pos(2);
      trajMarker.points.push_back(point);
    }

    trajPub.publish(trajMarker);
  }

  template <typename Trajectory>
  void visualizeSpheres(const Trajectory &traj, const Trajectory &arm_traj,
                        const int samples = 1000)
  {
    visualization_msgs::Marker sphereMarker;
    visualization_msgs::Marker sphere_end;
    visualization_msgs::Marker connectingLine;
    visualization_msgs::MarkerArray sphereMarkers, sphereMarkers_end;

    // Set up the first sphere
    sphereMarker.id = 0;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;
    sphereMarker.header.stamp = ros::Time::now();
    sphereMarker.header.frame_id = config->odomFrame;
    sphereMarker.action = visualization_msgs::Marker::ADD;
    sphereMarker.ns = "spheres";
    sphereMarker.color.r = 0.2;  // Light blue
    sphereMarker.color.g = 0.6;
    sphereMarker.color.b = 0.8;
    sphereMarker.color.a = 0.8;
    sphereMarkers.markers.clear();

    // Set up the second sphere (end effector)
    sphere_end.type = visualization_msgs::Marker::SPHERE;
    sphere_end.header.stamp = ros::Time::now();
    sphere_end.header.frame_id = config->odomFrame;
    sphere_end.action = visualization_msgs::Marker::ADD;
    sphere_end.ns = "spheres";
    sphere_end.color.r = 0.8;  // Light coral
    sphere_end.color.g = 0.3;
    sphere_end.color.b = 0.3;
    sphere_end.color.a = 0.8;
    sphereMarkers_end.markers.clear();

    // Set up the connecting line
    connectingLine.id = 0;
    connectingLine.type = visualization_msgs::Marker::LINE_STRIP;
    connectingLine.header.stamp = ros::Time::now();
    connectingLine.header.frame_id = config->odomFrame;
    connectingLine.action = visualization_msgs::Marker::ADD;
    connectingLine.ns = "connecting_line";
    connectingLine.color.r = 0.5;  // Gray
    connectingLine.color.g = 0.5;
    connectingLine.color.b = 0.5;
    connectingLine.color.a = 1.0;
    connectingLine.scale.x = 0.01;  // Line width

    double dt = traj.getTotalDuration() / samples;
    Eigen::Vector3d pos, arm_pos;
    Eigen::Matrix3d rotM;

    for (int i = 0; i <= samples; i++)
    {
      pos = traj.getPos(dt * i);
      arm_pos = arm_traj.getPos(dt * i);

      // First sphere (body)
      sphereMarker.pose.position.x = pos(0);
      sphereMarker.pose.position.y = pos(1);
      sphereMarker.pose.position.z = pos(2);
      sphereMarker.scale.x = sphereMarker.scale.y = sphereMarker.scale.z =
          config->horizHalfLen * 2.0;
      sphereMarkers.markers.push_back(sphereMarker);
      sphereMarker.id++;

      // Second sphere (end effector)
      traj.getRotation(dt * i, 0.0, config->gravAcc, rotM);
      Eigen::Vector3d E_arm_pos = rotM * (arm_pos - Eigen::Vector3d(0, 0, 0.04)) + pos;
      sphere_end.pose.position.x = E_arm_pos(0);
      sphere_end.pose.position.y = E_arm_pos(1);
      sphere_end.pose.position.z = E_arm_pos(2);
      sphere_end.scale.x = sphere_end.scale.y = sphere_end.scale.z = 0.05;
      sphereMarkers_end.markers.push_back(sphere_end);
      sphere_end.id++;

      // Connecting line
      geometry_msgs::Point p1, p2;
      p1.x = pos(0);
      p1.y = pos(1);
      p1.z = pos(2);
      p2.x = E_arm_pos(0);
      p2.y = E_arm_pos(1);
      p2.z = E_arm_pos(2);
      connectingLine.points.push_back(p1);
      connectingLine.points.push_back(p2);
    }

    spherePub.publish(sphereMarkers);
    spherePub_end.publish(sphereMarkers_end);
    spherePub_connect.publish(connectingLine);
  }

  template <typename Trajectory>
  void visualizeEllipsoid(const Trajectory &traj, const Trajectory &arm_traj,
                          const int samples = 5000, const double &grasp_time = -1.0)
  {
    visualization_msgs::Marker ellipsoidMarker;
    visualization_msgs::Marker ellipsoid_end;
    visualization_msgs::MarkerArray ellipsoidMarkers_dilate, ellipsoidMarkers,
        ellipsoidMarkers_end;

    ellipsoidMarker.id = 0;
    ellipsoidMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    ellipsoidMarker.mesh_resource = config->ellipsoidPath;
    ellipsoidMarker.header.stamp = ros::Time::now();
    ellipsoidMarker.header.frame_id = config->odomFrame;
    ellipsoidMarker.action = visualization_msgs::Marker::ADD;
    ellipsoidMarker.ns = "ellipsoids";
    ellipsoidMarker.color.r = 174.0 / 255.0;
    ellipsoidMarker.color.g = 210.0 / 255.0;
    ellipsoidMarker.color.b = 226.0 / 255.0;
    ellipsoidMarker.color.a = 1;

    ellipsoidMarkers_dilate.markers.clear();
    ellipsoidMarkers.markers.clear();
    ellipsoidMarkers_end.markers.clear();

    ellipsoid_end.type = visualization_msgs::Marker::MESH_RESOURCE;
    ellipsoid_end.mesh_resource = config->ellipsoidPath;
    ellipsoid_end.header.stamp = ros::Time::now();
    ellipsoid_end.header.frame_id = config->odomFrame;
    ellipsoid_end.pose.orientation.w = 1.00;
    ellipsoid_end.action = visualization_msgs::Marker::ADD;
    ellipsoid_end.ns = "ellipsoids_end";
    ellipsoid_end.color.r = 192.0 / 255.0;
    ellipsoid_end.color.g = 226.0 / 255.0;
    ellipsoid_end.color.b = 202.0 / 255.0;
    ellipsoid_end.color.a = 1;
    ellipsoid_end.scale.x = 0.15;
    ellipsoid_end.scale.y = 0.15;
    ellipsoid_end.scale.z = 0.03;

    double dt;
    dt = (traj.getTotalDuration() - 0.1) / samples;
    geometry_msgs::Point point;
    Eigen::Vector3d pos;
    Eigen::Matrix3d rotM;
    Eigen::Quaterniond quat;
    for (int i = 0; i <= samples; i++)
    {
      if (dt * i < 0.1) continue;
      /* whole-body */
      pos = traj.getPos(dt * i);
      ellipsoidMarker.pose.position.x = pos(0);
      ellipsoidMarker.pose.position.y = pos(1);
      ellipsoidMarker.pose.position.z = pos(2);
      traj.getRotation(dt * i, 0.0, config->gravAcc, rotM);
      quat = Eigen::Quaterniond(rotM);
      ellipsoidMarker.pose.orientation.w = quat.w();
      ellipsoidMarker.pose.orientation.x = quat.x();
      ellipsoidMarker.pose.orientation.y = quat.y();
      ellipsoidMarker.pose.orientation.z = quat.z();
      double dilate;
      Eigen::Vector3d arm_pos = arm_traj.getPos(dt * i);
      dilate = abs(arm_pos(2));
      ellipsoidMarker.scale.x = (config->horizHalfLen) * 2.0;
      ellipsoidMarker.scale.y = (config->horizHalfLen) * 2.0;
      ellipsoidMarker.scale.z = (config->vertHalfLen - 0.08 + dilate) * 2.0;
      ellipsoidMarker.color.r = 239.0 / 255.0;
      ellipsoidMarker.color.g = 118.0 / 255.0;
      ellipsoidMarker.color.b = 123.0 / 255.0;
      ellipsoidMarker.color.a = 1;

      /* dilated */
      ellipsoidMarkers_dilate.markers.push_back(ellipsoidMarker);

      /* saw */
      ellipsoidMarker.scale.x = config->horizHalfLen * 2.0;
      ellipsoidMarker.scale.y = config->horizHalfLen * 2.0;
      ellipsoidMarker.scale.z = config->vertHalfLen * 2.0 - 0.4;
      ellipsoidMarker.color.r = 174.0 / 255.0;
      ellipsoidMarker.color.g = 210.0 / 255.0;
      ellipsoidMarker.color.b = 226.0 / 255.0;
      ellipsoidMarker.color.a = 1;
      ellipsoidMarkers.markers.push_back(ellipsoidMarker);
      ellipsoidMarker.id++;

      /* end effector */
      traj.getRotation(dt * i, 0.0, config->gravAcc, rotM);
      quat = Eigen::Quaterniond(rotM);
      pos = traj.getPos(dt * i);
      Eigen::Vector3d E_arm_pos = arm_traj.getPos(dt * i);
      E_arm_pos -= Eigen::Vector3d(0, 0, 0.04);
      E_arm_pos = rotM * E_arm_pos;
      E_arm_pos = E_arm_pos + pos;
      ellipsoid_end.pose.position.x = E_arm_pos(0);
      ellipsoid_end.pose.position.y = E_arm_pos(1);
      ellipsoid_end.pose.position.z = E_arm_pos(2);
      ellipsoid_end.pose.orientation.w = quat.w();
      ellipsoid_end.pose.orientation.x = quat.x();
      ellipsoid_end.pose.orientation.y = quat.y();
      ellipsoid_end.pose.orientation.z = quat.z();
      ellipsoidMarkers_end.markers.push_back(ellipsoid_end);
      ellipsoid_end.id++;
    }

    ellipsoidPub_dilate.publish(ellipsoidMarkers_dilate);
    ellipsoidPub.publish(ellipsoidMarkers);
    ellipsoidPub_end.publish(ellipsoidMarkers_end);

    // 发布椭球体位置路径
    visualization_msgs::Marker path1_marker, path2_marker;

    // 设置金黄色路径 (主体椭球体路径)
    path1_marker.header.frame_id = config->odomFrame;
    path1_marker.header.stamp = ros::Time::now();
    path1_marker.ns = "ellipsoid_path1";
    path1_marker.id = 0;
    path1_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path1_marker.action = visualization_msgs::Marker::ADD;
    path1_marker.scale.x = 0.02;            // 线宽
    path1_marker.pose.orientation.w = 1.0;  // 保持方向不变
    path1_marker.pose.orientation.x = 0.0;
    path1_marker.pose.orientation.y = 0.0;
    path1_marker.pose.orientation.z = 0.0;
    path1_marker.color.r = 255.0 / 255.0;  // 金黄色
    path1_marker.color.g = 215.0 / 255.0;
    path1_marker.color.b = 0.0 / 255.0;
    path1_marker.color.a = 1.0;

    // 设置珊瑚红路径 (末端执行器椭球体路径)
    path2_marker.header.frame_id = config->odomFrame;
    path2_marker.header.stamp = ros::Time::now();
    path2_marker.ns = "ellipsoid_path2";
    path2_marker.id = 0;
    path2_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path2_marker.action = visualization_msgs::Marker::ADD;
    path2_marker.scale.x = 0.03;            // 线宽
    path2_marker.pose.orientation.w = 1.0;  // 保持方向不变
    path2_marker.pose.orientation.x = 0.0;  // 保持方向不变
    path2_marker.pose.orientation.y = 0.0;  // 保持方向不变
    path2_marker.pose.orientation.z = 0.0;  // 保持方向不变
    path2_marker.color.r = 255.0 / 255.0;   // 珊瑚红
    path2_marker.color.g = 127.0 / 255.0;
    path2_marker.color.b = 80.0 / 255.0;
    path2_marker.color.a = 1.0;

    // 重新遍历轨迹，收集路径点
    for (int i = 0; i <= samples; i++)
    {
      // if (dt * i < 0.1) continue;

      // 主体椭球体路径点
      Eigen::Vector3d body_pos = traj.getPos(dt * i);
      geometry_msgs::Point p1;
      p1.x = body_pos(0);
      p1.y = body_pos(1);
      p1.z = body_pos(2);
      path1_marker.points.push_back(p1);

      // 末端执行器椭球体路径点
      Eigen::Matrix3d rotM;
      traj.getRotation(dt * i, 0.0, config->gravAcc, rotM);
      Eigen::Vector3d E_arm_pos = arm_traj.getPos(dt * i);
      E_arm_pos -= Eigen::Vector3d(0, 0, 0.04);
      E_arm_pos = rotM * E_arm_pos;
      E_arm_pos = E_arm_pos + body_pos;

      geometry_msgs::Point p2;
      p2.x = E_arm_pos(0);
      p2.y = E_arm_pos(1);
      p2.z = E_arm_pos(2);
      path2_marker.points.push_back(p2);
    }
    ellipsoidPathPub1.publish(path1_marker);
    ellipsoidPathPub2.publish(path2_marker);

    // 发布稀疏的坐标轴
    visualization_msgs::MarkerArray coordAxesMarkers;
    int axes_interval = samples / config->coordinateNum;  // 每10个点显示一个坐标轴
    double drone_axis_length = 0.25;                      // 无人机坐标轴长度
    double ee_axis_length = 0.15;                         // 末端执行器坐标轴长度

    for (int i = 0; i <= samples; i += axes_interval)
    {
      if (dt * i < 0.1) continue;
      if (grasp_time > 0 && abs(dt * i - grasp_time) < 0.2) continue;

      // 获取当前位置和旋转
      Eigen::Vector3d pos = traj.getPos(dt * i);
      Eigen::Matrix3d rotM;
      traj.getRotation(dt * i, 0.0, config->gravAcc, rotM);

      // 普通无人机x轴的坐标系发布
      visualization_msgs::Marker x_axis;
      x_axis.header.frame_id = config->odomFrame;
      x_axis.header.stamp = ros::Time::now();
      x_axis.ns = "drone_coord_axes" + std::to_string(id);
      x_axis.id = i * 6;
      x_axis.type = visualization_msgs::Marker::ARROW;
      x_axis.action = visualization_msgs::Marker::ADD;
      x_axis.scale.x = 0.04;  // 箭头轴宽度
      x_axis.scale.y = 0.08;  // 箭头头部宽度
      x_axis.scale.z = 0.10;  // 箭头头部长度
      x_axis.pose.orientation.w = 1.0;
      x_axis.pose.orientation.x = 0.0;
      x_axis.pose.orientation.y = 0.0;
      x_axis.pose.orientation.z = 0.0;
      x_axis.color.r = 0.0;  // 青色
      x_axis.color.g = 0.8;
      x_axis.color.b = 0.8;
      x_axis.color.a = 1.0;
      geometry_msgs::Point start, end;
      start.x = pos(0);
      start.y = pos(1);
      start.z = pos(2);
      end.x = pos(0) + rotM.col(0)(0) * drone_axis_length;
      end.y = pos(1) + rotM.col(0)(1) * drone_axis_length;
      end.z = pos(2) + rotM.col(0)(2) * drone_axis_length;
      x_axis.points.push_back(start);
      x_axis.points.push_back(end);
      coordAxesMarkers.markers.push_back(x_axis);

      // 普通无人机y轴的坐标系发布
      visualization_msgs::Marker y_axis = x_axis;
      y_axis.id = i * 6 + 1;
      y_axis.color.r = 0.6;
      y_axis.color.g = 0.0;
      y_axis.color.b = 0.8;
      y_axis.points.clear();
      end.x = pos(0) + rotM.col(1)(0) * drone_axis_length;
      end.y = pos(1) + rotM.col(1)(1) * drone_axis_length;
      end.z = pos(2) + rotM.col(1)(2) * drone_axis_length;
      y_axis.points.push_back(start);
      y_axis.points.push_back(end);
      coordAxesMarkers.markers.push_back(y_axis);

      // 普通无人机z轴的坐标系发布
      visualization_msgs::Marker z_axis = x_axis;
      z_axis.id = i * 6 + 2;
      z_axis.color.r = 0.0;
      z_axis.color.g = 0.0;
      z_axis.color.b = 1.0;
      z_axis.points.clear();
      end.x = pos(0) + rotM.col(2)(0) * drone_axis_length;
      end.y = pos(1) + rotM.col(2)(1) * drone_axis_length;
      end.z = pos(2) + rotM.col(2)(2) * drone_axis_length;
      z_axis.points.push_back(start);
      z_axis.points.push_back(end);
      coordAxesMarkers.markers.push_back(z_axis);

      // 普通末端执行器x轴的坐标系发布
      Eigen::Vector3d E_arm_pos = arm_traj.getPos(dt * i);
      E_arm_pos -= Eigen::Vector3d(0, 0, 0.04);
      E_arm_pos = rotM * E_arm_pos;
      E_arm_pos = E_arm_pos + pos;

      // 普通末端执行器x轴的坐标系发布
      visualization_msgs::Marker ee_x_axis = x_axis;
      ee_x_axis.ns = "ee_coord_axes" + std::to_string(id);
      ee_x_axis.id = i * 6 + 3;
      ee_x_axis.color.r = 0.0;
      ee_x_axis.color.g = 0.6;
      ee_x_axis.color.b = 1.0;
      ee_x_axis.scale.x = 0.03;  // 箭头轴宽度
      ee_x_axis.scale.y = 0.05;  // 箭头头部宽度
      ee_x_axis.scale.z = 0.08;  // 箭头头部长度
      ee_x_axis.points.clear();
      start.x = E_arm_pos(0);
      start.y = E_arm_pos(1);
      start.z = E_arm_pos(2);
      end.x = E_arm_pos(0) + rotM.col(0)(0) * ee_axis_length;
      end.y = E_arm_pos(1) + rotM.col(0)(1) * ee_axis_length;
      end.z = E_arm_pos(2) + rotM.col(0)(2) * ee_axis_length;
      ee_x_axis.points.push_back(start);
      ee_x_axis.points.push_back(end);
      coordAxesMarkers.markers.push_back(ee_x_axis);

      // 普通末端执行器y轴的坐标系发布
      visualization_msgs::Marker ee_y_axis = ee_x_axis;
      ee_y_axis.id = i * 6 + 4;
      ee_y_axis.color.r = 0.0;
      ee_y_axis.color.g = 0.4;
      ee_y_axis.color.b = 0.8;
      ee_y_axis.points.clear();
      end.x = E_arm_pos(0) + rotM.col(1)(0) * ee_axis_length;
      end.y = E_arm_pos(1) + rotM.col(1)(1) * ee_axis_length;
      end.z = E_arm_pos(2) + rotM.col(1)(2) * ee_axis_length;
      ee_y_axis.points.push_back(start);
      ee_y_axis.points.push_back(end);
      coordAxesMarkers.markers.push_back(ee_y_axis);

      // 普通末端执行器z轴的坐标系发布
      visualization_msgs::Marker ee_z_axis = ee_x_axis;
      ee_z_axis.id = i * 6 + 5;
      ee_z_axis.color.r = 0.0;
      ee_z_axis.color.g = 0.7;
      ee_z_axis.color.b = 0.9;
      ee_z_axis.points.clear();
      end.x = E_arm_pos(0) + rotM.col(2)(0) * ee_axis_length;
      end.y = E_arm_pos(1) + rotM.col(2)(1) * ee_axis_length;
      end.z = E_arm_pos(2) + rotM.col(2)(2) * ee_axis_length;
      ee_z_axis.points.push_back(start);
      ee_z_axis.points.push_back(end);
      coordAxesMarkers.markers.push_back(ee_z_axis);
    }

    /* 在grasp time处添加坐标系 */
    if (grasp_time > 0.0)
    {
      // 获取grasp time时的位置和旋转
      Eigen::Vector3d grasp_pos = traj.getPos(grasp_time);
      Eigen::Matrix3d grasp_rotM;
      traj.getRotation(grasp_time, 0.0, config->gravAcc, grasp_rotM);

      /* 无人机主体在grasp time的坐标轴 */
      visualization_msgs::Marker grasp_x_axis;
      grasp_x_axis.header.frame_id = config->odomFrame;
      grasp_x_axis.header.stamp = ros::Time::now();
      grasp_x_axis.ns = "grasp_time_drone_axes" + std::to_string(id);
      grasp_x_axis.id = 0;
      grasp_x_axis.type = visualization_msgs::Marker::ARROW;
      grasp_x_axis.action = visualization_msgs::Marker::ADD;
      grasp_x_axis.scale.x = 0.04;  // 箭头轴宽度
      grasp_x_axis.scale.y = 0.08;  // 箭头头部宽度
      grasp_x_axis.scale.z = 0.10;  // 箭头头部长度
      grasp_x_axis.pose.orientation.w = 1.0;
      grasp_x_axis.pose.orientation.x = 0.0;
      grasp_x_axis.pose.orientation.y = 0.0;
      grasp_x_axis.pose.orientation.z = 0.0;
      grasp_x_axis.color.r = 0.0;  // 青色
      grasp_x_axis.color.g = 0.8;
      grasp_x_axis.color.b = 0.8;
      grasp_x_axis.color.a = 1.0;
      geometry_msgs::Point grasp_start, grasp_end;
      grasp_start.x = grasp_pos(0);
      grasp_start.y = grasp_pos(1);
      grasp_start.z = grasp_pos(2);
      grasp_end.x = grasp_pos(0) + grasp_rotM.col(0)(0) * drone_axis_length;
      grasp_end.y = grasp_pos(1) + grasp_rotM.col(0)(1) * drone_axis_length;
      grasp_end.z = grasp_pos(2) + grasp_rotM.col(0)(2) * drone_axis_length;
      grasp_x_axis.points.push_back(grasp_start);
      grasp_x_axis.points.push_back(grasp_end);
      coordAxesMarkers.markers.push_back(grasp_x_axis);

      /* 无人机主体在grasp time的y轴的坐标轴 */
      visualization_msgs::Marker grasp_y_axis = grasp_x_axis;
      grasp_y_axis.id = 1;
      grasp_y_axis.color.r = 0.6;  // 紫色
      grasp_y_axis.color.g = 0.0;
      grasp_y_axis.color.b = 0.8;
      grasp_y_axis.points.clear();
      grasp_end.x = grasp_pos(0) + grasp_rotM.col(1)(0) * drone_axis_length;
      grasp_end.y = grasp_pos(1) + grasp_rotM.col(1)(1) * drone_axis_length;
      grasp_end.z = grasp_pos(2) + grasp_rotM.col(1)(2) * drone_axis_length;
      grasp_y_axis.points.push_back(grasp_start);
      grasp_y_axis.points.push_back(grasp_end);
      coordAxesMarkers.markers.push_back(grasp_y_axis);

      /* 无人机主体在grasp time的z轴的坐标轴 */
      visualization_msgs::Marker grasp_z_axis = grasp_x_axis;
      grasp_z_axis.id = 2;
      grasp_z_axis.color.r = 0.0;  // 蓝色
      grasp_z_axis.color.g = 0.0;
      grasp_z_axis.color.b = 1.0;
      grasp_z_axis.points.clear();
      grasp_end.x = grasp_pos(0) + grasp_rotM.col(2)(0) * drone_axis_length;
      grasp_end.y = grasp_pos(1) + grasp_rotM.col(2)(1) * drone_axis_length;
      grasp_end.z = grasp_pos(2) + grasp_rotM.col(2)(2) * drone_axis_length;
      grasp_z_axis.points.push_back(grasp_start);
      grasp_z_axis.points.push_back(grasp_end);
      coordAxesMarkers.markers.push_back(grasp_z_axis);

      Eigen::Vector3d grasp_E_arm_pos = arm_traj.getPos(grasp_time);
      grasp_E_arm_pos -= Eigen::Vector3d(0, 0, 0.04);
      grasp_E_arm_pos = grasp_rotM * grasp_E_arm_pos;
      grasp_E_arm_pos = grasp_E_arm_pos + grasp_pos;

      /* 末端执行器在grasp time的x轴的坐标轴 */
      visualization_msgs::Marker grasp_ee_x_axis = grasp_x_axis;
      grasp_ee_x_axis.ns = "grasp_time_ee_axes" + std::to_string(id);
      grasp_ee_x_axis.id = 3;
      grasp_ee_x_axis.color.r = 0.0;  // 浅蓝色
      grasp_ee_x_axis.color.g = 0.6;
      grasp_ee_x_axis.color.b = 1.0;
      grasp_ee_x_axis.scale.x = 0.03;  // 箭头轴宽度
      grasp_ee_x_axis.scale.y = 0.05;  // 箭头头部宽度
      grasp_ee_x_axis.scale.z = 0.08;  // 箭头头部长度
      grasp_ee_x_axis.points.clear();
      grasp_start.x = grasp_E_arm_pos(0);
      grasp_start.y = grasp_E_arm_pos(1);
      grasp_start.z = grasp_E_arm_pos(2);
      grasp_end.x = grasp_E_arm_pos(0) + grasp_rotM.col(0)(0) * ee_axis_length;
      grasp_end.y = grasp_E_arm_pos(1) + grasp_rotM.col(0)(1) * ee_axis_length;
      grasp_end.z = grasp_E_arm_pos(2) + grasp_rotM.col(0)(2) * ee_axis_length;
      grasp_ee_x_axis.points.push_back(grasp_start);
      grasp_ee_x_axis.points.push_back(grasp_end);
      coordAxesMarkers.markers.push_back(grasp_ee_x_axis);

      /* 末端执行器在grasp time的y轴的坐标轴 */
      visualization_msgs::Marker grasp_ee_y_axis = grasp_ee_x_axis;
      grasp_ee_y_axis.id = 4;
      grasp_ee_y_axis.color.r = 0.0;  // 浅紫色
      grasp_ee_y_axis.color.g = 0.4;
      grasp_ee_y_axis.color.b = 0.8;
      grasp_ee_y_axis.points.clear();
      grasp_end.x = grasp_E_arm_pos(0) + grasp_rotM.col(1)(0) * ee_axis_length;
      grasp_end.y = grasp_E_arm_pos(1) + grasp_rotM.col(1)(1) * ee_axis_length;
      grasp_end.z = grasp_E_arm_pos(2) + grasp_rotM.col(1)(2) * ee_axis_length;
      grasp_ee_y_axis.points.push_back(grasp_start);
      grasp_ee_y_axis.points.push_back(grasp_end);
      coordAxesMarkers.markers.push_back(grasp_ee_y_axis);

      /* 末端执行器在grasp time的z轴的坐标轴 */
      visualization_msgs::Marker grasp_ee_z_axis = grasp_ee_x_axis;
      grasp_ee_z_axis.id = 5;
      grasp_ee_z_axis.color.r = 0.0;  // 浅蓝色
      grasp_ee_z_axis.color.g = 0.7;
      grasp_ee_z_axis.color.b = 0.9;
      grasp_ee_z_axis.points.clear();
      grasp_end.x = grasp_E_arm_pos(0) + grasp_rotM.col(2)(0) * ee_axis_length;
      grasp_end.y = grasp_E_arm_pos(1) + grasp_rotM.col(2)(1) * ee_axis_length;
      grasp_end.z = grasp_E_arm_pos(2) + grasp_rotM.col(2)(2) * ee_axis_length;
      grasp_ee_z_axis.points.push_back(grasp_start);
      grasp_ee_z_axis.points.push_back(grasp_end);
      coordAxesMarkers.markers.push_back(grasp_ee_z_axis);
    }
    coordAxesPub.publish(coordAxesMarkers);
  }

  template <typename Trajectory>
  void visualizeQuadrotor(const Trajectory &traj, const int samples = 1000)
  {
    visualization_msgs::Marker quadrotorMarker;
    visualization_msgs::MarkerArray quadrotorMarkers;

    quadrotorMarker.id = 0;
    quadrotorMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    quadrotorMarker.mesh_use_embedded_materials = true;
    quadrotorMarker.mesh_resource = config->quadrotorPath;
    quadrotorMarker.header.stamp = ros::Time::now();
    quadrotorMarker.header.frame_id = config->odomFrame;
    quadrotorMarker.pose.orientation.w = 1.00;
    quadrotorMarker.action = visualization_msgs::Marker::ADD;
    quadrotorMarker.ns = "quadrotor";
    quadrotorMarker.color.r = 0.0;
    quadrotorMarker.color.g = 0.0;
    quadrotorMarker.color.b = 0.0;
    quadrotorMarker.color.a = 0.0;
    quadrotorMarker.scale.x =
        (config->horizHalfLen + config->safeMargin * 3.3 + 0.2) * sqrt(2.0);
    quadrotorMarker.scale.y =
        (config->horizHalfLen + config->safeMargin * 3.3 + 0.2) * sqrt(2.0);
    quadrotorMarker.scale.z = config->vertHalfLen * 8.0;

    quadrotorMarker.action = visualization_msgs::Marker::DELETEALL;
    quadrotorMarkers.markers.push_back(quadrotorMarker);
    quadrotorPub.publish(quadrotorMarkers);
    quadrotorMarker.action = visualization_msgs::Marker::ADD;
    quadrotorMarkers.markers.clear();

    double dt = 5.0 / samples;  // hzc30
    geometry_msgs::Point point;
    Eigen::Vector3d pos;
    Eigen::Matrix3d rotM;
    Eigen::Quaterniond quat;
    for (int i = 0; i <= samples; i++)
    {
      pos = traj.getPos(6 + dt * i);  // 4.5
      quadrotorMarker.pose.position.x = pos(0);
      quadrotorMarker.pose.position.y = pos(1);
      quadrotorMarker.pose.position.z = pos(2);
      // traj.getRotation(6+dt * i, M_PI_4, config->gravAcc, rotM);hzc
      traj.getRotation(6 + dt * i, M_PI_2, config->gravAcc, rotM);
      quat = Eigen::Quaterniond(rotM);
      quadrotorMarker.pose.orientation.w = quat.w();
      quadrotorMarker.pose.orientation.x = quat.x();
      quadrotorMarker.pose.orientation.y = quat.y();
      quadrotorMarker.pose.orientation.z = quat.z();
      quadrotorMarkers.markers.push_back(quadrotorMarker);
      quadrotorMarker.id++;
    }

    quadrotorPub.publish(quadrotorMarkers);
  }

  void visualizePolyH(const vec_E<Polyhedron3D> &polyhedra)
  {
    decomp_ros_msgs::PolyhedronArray poly_msg =
        DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = config->odomFrame;
    poly_msg.header.stamp = ros::Time::now();
    hPolyPub.publish(poly_msg);
  }

  void visualizeActivePolyH(const vec_E<Polyhedron3D> &polyhedra)
  {
    decomp_ros_msgs::PolyhedronArray poly_msg =
        DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = config->odomFrame;
    poly_msg.header.stamp = ros::Time::now();
    activePolyPub.publish(poly_msg);
  }

  template <typename Trajectory>
  void visualizeProfile(const Trajectory &traj, const double &t)
  {
    Eigen::Vector3d tiltRate = traj.getTiltRate(t, config->gravAcc);
    Eigen::Vector3d thrAcc = traj.getAcc(t);
    thrAcc(2) += config->gravAcc;

    std_msgs::Float64 tilt, thr;
    tilt.data = tiltRate.norm();
    thr.data = thrAcc.norm();

    tiltRatePub.publish(tilt);
    thrMagPub.publish(thr);
  }

  void visualizePath(const vec_Vec3f &path)
  {
    // Publish path to RViz
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.04;  // Line width
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;

    for (const auto &point : path)
    {
      geometry_msgs::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      path_marker.points.push_back(p);
    }

    pathPub.publish(path_marker);
    ROS_INFO("Path published!");
  }

  void visualizeKeyPoints(const MatDf &path)
  {
    // Publish path to RViz
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "key_points";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::POINTS;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.4;  // Line width
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;

    for (int i = 0; i < path.cols(); ++i)
    {
      geometry_msgs::Point p;
      p.x = path(0, i);
      p.y = path(1, i);
      p.z = path(2, i);
      path_marker.points.push_back(p);
    }

    keyPointsPub.publish(path_marker);
    ROS_INFO("Key points published!");
  }

  void visualizeGuidePoints(const MatDf &guide)
  {
    // Publish path to RViz
    visualization_msgs::Marker guide_marker;
    guide_marker.header.frame_id = "world";
    guide_marker.header.stamp = ros::Time::now();
    guide_marker.ns = "guide_points";
    guide_marker.id = 0;
    guide_marker.type = visualization_msgs::Marker::POINTS;
    guide_marker.action = visualization_msgs::Marker::ADD;
    guide_marker.scale.x = 0.1;  // Line width
    guide_marker.scale.y = 0.0;
    guide_marker.scale.z = 0.0;
    guide_marker.color.r = 220.0 / 255.0;
    guide_marker.color.g = 20.0 / 255.0;
    guide_marker.color.b = 60.0 / 255.0;
    guide_marker.color.a = 1.0;

    for (int i = 0; i < guide.cols(); ++i)
    {
      geometry_msgs::Point p;
      p.x = guide(0, i);
      p.y = guide(1, i);
      p.z = guide(2, i);
      guide_marker.points.push_back(p);
    }

    guidePointsPub.publish(guide_marker);
    ROS_INFO("Guide points published!");
  }
};

#endif  // VISUALIZATION_H