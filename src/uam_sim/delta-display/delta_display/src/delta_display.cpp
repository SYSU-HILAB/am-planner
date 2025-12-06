#include "delta_display/delta_display.h"

/*
-------- top view ---------
           /|\ y axis
  \         |         /
     \      |      / 1
    2   \   |60°/
           \|/ _______> x axis
            |
            |
            |
            | 3
*/

namespace delta_display
{
DeltaDisplay::DeltaDisplay(ros::NodeHandle n)
{
  n.param("scale", scale, 1.000);
  n.param("staticR", R, 0.067);
  n.param("dynar", r, 0.024);
  n.param("upper_arm", L, 0.100);
  n.param("lower_arm", l, 0.160);
  n.param("rate", update_rate, 100.0);
  n.param("debug", debug, true);
  n.param("mass", mass, 1.00);
  n.param("debug_endEffector", debug_endEffector, false);

  R *= scale;
  r *= scale;
  L *= scale;
  l *= scale;

  used_so3control = false;
  phi = Eigen::Vector3d(1.0 / 6.0 * PI, 5.0 / 6.0 * PI, 3.0 / 2.0 * PI);
  joint_state_pub = n.advertise<sensor_msgs::JointState>(
      "/joint_states", 1, false);  // publish the joints to robot_description
  bottom_publisher = n.advertise<visualization_msgs::Marker>("/world/bottom", 1, true);
  bottom_odom_publisher =
      n.advertise<nav_msgs::Odometry>("delta_display/odom/bottom", 1, true);
  quad_odom_publisher =
      n.advertise<nav_msgs::Odometry>("delta_display/odom/quadrotor", 1, true);

  quad_yz_publisher =
      n.advertise<std_msgs::Float64>("delta_display/odom/quadrotor_yz", 1, true);

  odom_sub = n.subscribe<nav_msgs::Odometry>("/world/quadrotor", 1,
                                             &DeltaDisplay::odomCallback, this);

  position_cmd_sub = n.subscribe<quadrotor_msgs::PositionCommand>(
      "position_des", 1, &DeltaDisplay::posCmdCallback, this);
  sub_end = n.subscribe<geometry_msgs::Point>("body/end_effector", 100,
                                              &DeltaDisplay::endCallback, this);
  sub_angles = n.subscribe<std_msgs::Float32MultiArray>(
      "angles", 100, &DeltaDisplay::anglesCallback, this);

  flatness.reset(mass, 9.81, 0.05, 0.05, 0.05, 0.05);

  tfInit();
  ros::Rate rate = update_rate;
  int interval = 0;

  while (ros::ok())
  {
    tf_quadrotor.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(tf_quadrotor);  // quadrotor tf
    joint.header.stamp = ros::Time::now();
    joint_state_pub.publish(joint);
    visual_bottom();
    if (debug_endEffector)
    {
      interval = 0;
      Eigen::Vector4d end_p(end_effector_x, end_effector_y, end_effector_z, 1);
      Eigen::Vector4d body_p = body_T_end * end_p;
      point_src.point.x = body_p(0);
      point_src.point.y = body_p(1);
      point_src.point.z = body_p(2);
      tf2::doTransform(point_src, point_tgt, tf_quadrotor);
      ROS_INFO("The end effector position is %.2lf, %.2lf, %.2lf.",
               point_tgt.point.x / scale, point_tgt.point.y / scale,
               point_tgt.point.z / scale);
    }
    else
    {
      interval++;
    }
    for (int i = 0; i < 6; i++)
    {
      tf_vec.at(i).header.stamp = ros::Time::now();
      tf_broadcaster.sendTransform(tf_vec.at(i));
    }
    ros::spinOnce();
    rate.sleep();
  }
}

void DeltaDisplay::tfInit()
{
  /*
      TA means trailing arms
          --------
              \
               \  mi_1
                \
                 \
                / /
         TA_il / /  TA_ir
              / /
             / /
            ====
  */

  joint.header.frame_id = "body";
  joint.name = {"m1_1", "m2_1", "m3_1"};
  joint.position = {0, 0, 0};

  /*
      trailing arms
  */
  tf11.header.frame_id = "end_effector";
  tf11.child_frame_id = "TA_1l";
  tf12.header.frame_id = "end_effector";
  tf12.child_frame_id = "TA_1r";
  tf21.header.frame_id = "end_effector";
  tf21.child_frame_id = "TA_2l";
  tf22.header.frame_id = "end_effector";
  tf22.child_frame_id = "TA_2r";
  tf31.header.frame_id = "end_effector";
  tf31.child_frame_id = "TA_3l";
  tf32.header.frame_id = "end_effector";
  tf32.child_frame_id = "TA_3r";
  tf_vec.push_back(tf11);
  tf_vec.push_back(tf12);
  tf_vec.push_back(tf21);
  tf_vec.push_back(tf22);
  tf_vec.push_back(tf31);
  tf_vec.push_back(tf32);
  for (int i = 0; i < 6; i++)
  {
    tf_vec.at(i).transform.translation.x = 0;
    tf_vec.at(i).transform.translation.y = 0;
    tf_vec.at(i).transform.translation.z = 0;
    tf_vec.at(i).transform.rotation.x = 0;
    tf_vec.at(i).transform.rotation.y = 0;
    tf_vec.at(i).transform.rotation.z = 0;
    tf_vec.at(i).transform.rotation.w = 1;
  }

  /*
      quadrotor base
  */
  tf_quadrotor.header.frame_id = "world";
  tf_quadrotor.child_frame_id = "body";
  tf_quadrotor.transform.translation.x = 0;
  tf_quadrotor.transform.translation.y = 0;
  tf_quadrotor.transform.translation.z = 0;
  tf_quadrotor.transform.rotation.x = 0;
  tf_quadrotor.transform.rotation.y = 0;
  tf_quadrotor.transform.rotation.z = 0;
  tf_quadrotor.transform.rotation.w = 1;

  /*
      end to odom
  */
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d end_R_odom = q.toRotationMatrix().transpose();
  body_T_end << end_R_odom(0, 0), end_R_odom(0, 1), end_R_odom(0, 2), 0, end_R_odom(1, 0),
      end_R_odom(1, 1), end_R_odom(1, 2), 0, end_R_odom(2, 0), end_R_odom(2, 1),
      end_R_odom(2, 2), -0.04 * scale, 0, 0, 0, 1;
  /*
      odom to world
  */
  point_src.header.frame_id = "body";
  point_tgt.header.frame_id = "world";

  /*
      bottom
  */
  block.type = visualization_msgs::Marker::MESH_RESOURCE;
  block.mesh_resource = "package://delta_display/meshes/bottom.STL";
  block.action = visualization_msgs::Marker::ADD;
  block.id = 0;
  block.header.frame_id = "end_effector";
  block.ns = "block";
  // set scale
  block.color.r = 0.5f;
  block.color.g = 0.5f;
  block.color.b = 0.5f;
  block.color.a = 1.0f;
  block.scale.x = 0.0014 * scale;
  block.scale.y = 0.0014 * scale;
  block.scale.z = 0.0014 * scale;

  // set the initial position of the end effector
  geometry_msgs::Point init_position;
  init_position.z = -l / 2.0 / scale;
  endCallback(boost::make_shared<geometry_msgs::Point>(init_position));

  end_odom.header.frame_id = "world";
  quad_odom.header.frame_id = "world";
}

void DeltaDisplay::posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  if (used_so3control)
  {
    return;
  }

  Eigen::Vector3d v(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  Eigen::Vector3d a(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  Eigen::Vector3d j(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);
  double yaw = cmd->yaw;
  double yaw_dot = cmd->yaw_dot;
  double thr;
  Eigen::Vector4d quat;
  Eigen::Vector3d omg;
  flatness.forward(v, a, j, yaw, yaw_dot, thr, quat, omg);
  tf_quadrotor.transform.translation.x = cmd->position.x;
  tf_quadrotor.transform.translation.y = cmd->position.y;
  tf_quadrotor.transform.translation.z = cmd->position.z;
  tf_quadrotor.transform.rotation.x = quat(1);
  tf_quadrotor.transform.rotation.y = quat(2);
  tf_quadrotor.transform.rotation.z = quat(3);
  tf_quadrotor.transform.rotation.w = quat(0);

  quad_odom.pose.pose.position.x = cmd->position.x;
  quad_odom.pose.pose.position.y = cmd->position.y;
  quad_odom.pose.pose.position.z = cmd->position.z;
  quad_odom.pose.pose.orientation.x = quat(1);
  quad_odom.pose.pose.orientation.y = quat(2);
  quad_odom.pose.pose.orientation.z = quat(3);
  quad_odom.pose.pose.orientation.w = quat(0);
  quad_odom.twist.twist.linear.x = v.x();
  quad_odom.twist.twist.linear.y = v.y();
  quad_odom.twist.twist.linear.z = v.z();
  quad_odom.twist.twist.angular.x = omg.x();
  quad_odom.twist.twist.angular.y = omg.y();
  quad_odom.twist.twist.angular.z = omg.z();
  quad_odom.header.stamp = ros::Time::now();
  quad_odom_publisher.publish(quad_odom);

  Eigen::Quaterniond q(quat(0), quat(1), quat(2), quat(3));
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d zb_axis = R.col(2);
  Eigen::Vector2d z_proj(zb_axis.y(), zb_axis.z());
  double angle = std::atan2(z_proj.x(), z_proj.y());
  std_msgs::Float64 angle_msg;
  angle_msg.data = angle;
  quad_yz_publisher.publish(angle_msg);
}

void DeltaDisplay::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  if (debug) ROS_INFO("\033[34mThe quadrotor got new pose.\033[0m");
  tf_quadrotor.transform.translation.x = odom->pose.pose.position.x;
  tf_quadrotor.transform.translation.y = odom->pose.pose.position.y;
  tf_quadrotor.transform.translation.z = odom->pose.pose.position.z;
  tf_quadrotor.transform.rotation.x = odom->pose.pose.orientation.x;
  tf_quadrotor.transform.rotation.y = odom->pose.pose.orientation.y;
  tf_quadrotor.transform.rotation.z = odom->pose.pose.orientation.z;
  tf_quadrotor.transform.rotation.w = odom->pose.pose.orientation.w;

  visual_bottom();
}

/*
    given angles from topic /angles
    theta \in [0, 90]

    =====base=====------------
                  theta = 90 degree


    =====base======
                  |  theta = 0 degree
                  |
                  |
                  |
*/

void DeltaDisplay::anglesCallback(const std_msgs::Float32MultiArray::ConstPtr &angles_msg)
{
  double theta1, theta2, theta3;
  theta1 = angles_msg->data[0] * ang2rad;
  theta2 = angles_msg->data[1] * ang2rad;
  theta3 = angles_msg->data[2] * ang2rad;

  if (debug) ROS_INFO("Received angles are %.2lf, %.2lf, %.2lf.", theta1, theta2, theta3);
  Eigen::Vector3d position;
  FK_kin(theta1, theta2, theta3, position);
  double x, y, z;
  x = position.x();
  y = position.y();
  z = position.z();

  if (std::isnan(theta1) || std::isnan(theta2) || std::isnan(theta3) || theta1 < 0 ||
      theta2 < 0 || theta3 < 0 || theta1 > PI / 2 || theta2 > PI / 2 || theta3 > PI / 2)
  {
    ROS_ERROR("Out of the workspace");
    return;
  }

  double theta[] = {PI / 2 - theta1, PI / 2 - theta2, PI / 2 - theta3};
  for (int i = 0; i < 3; i++)
  {
    joint.position.at(i) = theta[i];
  }

  // more detil in the getJointPoints function
  std::vector<Eigen::Vector3d> A;
  std::vector<Eigen::Vector3d> B;
  std::vector<Eigen::Vector3d> C;
  std::vector<Eigen::Vector3d> B_left_array;
  std::vector<Eigen::Vector3d> B_right_array;
  std::vector<Eigen::Vector3d> C_left_array;
  std::vector<Eigen::Vector3d> C_right_array;

  getJointPoints(x, y, z, theta, A, B, C, C_left_array, C_right_array, B_left_array,
                 B_right_array);

  double zbias = 0.015;
  for (int i = 0; i < 3; i++)
  {
    tf_vec.at(2 * i).transform.translation.x = B_left_array[i].x();
    tf_vec.at(2 * i).transform.translation.y = B_left_array[i].y();
    tf_vec.at(2 * i).transform.translation.z = B_left_array[i].z() + zbias;
    geometry_msgs::Quaternion q1;
    drivenArmsQuaternion(C_left_array[i] - B_left_array[i], q1);
    tf_vec.at(2 * i).transform.rotation.w = q1.w;
    tf_vec.at(2 * i).transform.rotation.x = q1.x;
    tf_vec.at(2 * i).transform.rotation.y = q1.y;
    tf_vec.at(2 * i).transform.rotation.z = q1.z;

    tf_vec.at(2 * i + 1).transform.translation.x = B_right_array[i].x();
    tf_vec.at(2 * i + 1).transform.translation.y = B_right_array[i].y();
    tf_vec.at(2 * i + 1).transform.translation.z = B_right_array[i].z() + zbias;
    geometry_msgs::Quaternion q2;
    drivenArmsQuaternion(C_right_array[i] - B_right_array[i], q2);
    tf_vec.at(2 * i + 1).transform.rotation.w = q2.w;
    tf_vec.at(2 * i + 1).transform.rotation.x = q2.x;
    tf_vec.at(2 * i + 1).transform.rotation.y = q2.y;
    tf_vec.at(2 * i + 1).transform.rotation.z = q2.z;
  }
  end_effector_x = x;
  end_effector_y = y;
  end_effector_z = z;
  visual_bottom(x, y, z);
}

void DeltaDisplay::endCallback(const geometry_msgs::Point::ConstPtr &point_msg)
{
  double x = point_msg->x * scale;
  double y = point_msg->y * scale;
  double z = point_msg->z * scale;

  if (debug)
    ROS_INFO("Received end_effecot is %.4lf, %.4lf, %.4lf.", x / scale, y / scale,
             z / scale);
  if (z > 0)
  {
    ROS_ERROR("Point ERROR! The z should be negative!");
    return;
  }

  double theta1, theta2, theta3;
  Eigen::Vector3d theta_vector;
  IK_kin(x, y, z, theta_vector);

  theta1 = theta_vector(0);
  theta2 = theta_vector(1);
  theta3 = theta_vector(2);

  if (std::isnan(theta1) || std::isnan(theta2) || std::isnan(theta3) || theta1 < 0 ||
      theta2 < 0 || theta3 < 0 || theta1 > PI / 2 || theta2 > PI / 2 || theta3 > PI / 2)
  {
    // ROS_ERROR("x:%lf, y:%lf, z:%lf, Out of the workspace!", x, y, z);
    return;
  }

  double theta[] = {theta1, theta2, theta3};

  for (int i = 0; i < 3; i++)
  {
    joint.position.at(i) = theta[i];
  }

  std::vector<Eigen::Vector3d> A;
  std::vector<Eigen::Vector3d> B;
  std::vector<Eigen::Vector3d> C;
  std::vector<Eigen::Vector3d> B_left_array;
  std::vector<Eigen::Vector3d> B_right_array;
  std::vector<Eigen::Vector3d> C_left_array;
  std::vector<Eigen::Vector3d> C_right_array;
  getJointPoints(x, y, z, theta, A, B, C, C_left_array, C_right_array, B_left_array,
                 B_right_array);

  double zbias = 0.015 * scale;
  for (int i = 0; i < 3; i++)
  {
    tf_vec.at(2 * i).transform.translation.x = B_left_array[i].x();
    tf_vec.at(2 * i).transform.translation.y = B_left_array[i].y();
    tf_vec.at(2 * i).transform.translation.z = B_left_array[i].z() + zbias;
    geometry_msgs::Quaternion q1;
    drivenArmsQuaternion(C_left_array[i] - B_left_array[i], q1);
    tf_vec.at(2 * i).transform.rotation.w = q1.w;
    tf_vec.at(2 * i).transform.rotation.x = q1.x;
    tf_vec.at(2 * i).transform.rotation.y = q1.y;
    tf_vec.at(2 * i).transform.rotation.z = q1.z;

    tf_vec.at(2 * i + 1).transform.translation.x = B_right_array[i].x();
    tf_vec.at(2 * i + 1).transform.translation.y = B_right_array[i].y();
    tf_vec.at(2 * i + 1).transform.translation.z = B_right_array[i].z() + zbias;
    geometry_msgs::Quaternion q2;
    drivenArmsQuaternion(C_right_array[i] - B_right_array[i], q2);
    tf_vec.at(2 * i + 1).transform.rotation.w = q2.w;
    tf_vec.at(2 * i + 1).transform.rotation.x = q2.x;
    tf_vec.at(2 * i + 1).transform.rotation.y = q2.y;
    tf_vec.at(2 * i + 1).transform.rotation.z = q2.z;
  }
  end_effector_x = x;
  end_effector_y = y;
  end_effector_z = z;
  visual_bottom(x, y, z);
}

void DeltaDisplay::visual_bottom(double x, double y, double z)
{
  bottom_x = x;
  bottom_y = y;
  bottom_z = z;
  visual_bottom();
}

void DeltaDisplay::visual_bottom()
{
  block.pose.position.x = bottom_x;
  block.pose.position.y = bottom_y;
  block.pose.position.z = bottom_z + 0.018 * scale;

  block.pose.orientation.x = 0.0;
  block.pose.orientation.y = 0.0;
  block.pose.orientation.z = 0.2588;
  block.pose.orientation.w = 0.9659;

  bottom_publisher.publish(block);

  /*calculate the bottom position in the world frame*/

  Eigen::Vector4d bottom_p(bottom_x, bottom_y, bottom_z, 1);
  Eigen::Vector4d body_p_homo = body_T_end * bottom_p;
  geometry_msgs::PointStamped body_p, world_p;
  body_p.header.frame_id = "body";
  world_p.header.frame_id = "world";
  body_p.point.x = body_p_homo(0);
  body_p.point.y = body_p_homo(1);
  body_p.point.z = body_p_homo(2);

  tf2::doTransform(body_p, world_p, tf_quadrotor);
  end_odom.pose.pose.position.x = world_p.point.x;
  end_odom.pose.pose.position.y = world_p.point.y;
  end_odom.pose.pose.position.z = world_p.point.z;
  bottom_odom_publisher.publish(end_odom);
}

void DeltaDisplay::IK_kin(double x1, double y1, double z1, Eigen::Vector3d &theta_vector)
{
  double K1, U1, V1;
  double K2, U2, V2;
  double K3, U3, V3;

  double t1, t2, t3;
  double middlevalue;
  double lower_arm = this->l;
  double staticR = this->R;
  double dynar = this->r;
  double upper_arm = this->L;
  Eigen::Vector3d position(x1, y1, z1);

  middlevalue = pow(lower_arm, 2) - pow(upper_arm, 2) - position.dot(position) -
                pow((staticR - dynar), 2);
  K1 = 2 * position(2) +
       (middlevalue + (staticR - dynar) * (SQRT_3 * position(0) + position(1))) /
           upper_arm;
  U1 = -2 * (2 * (staticR - dynar) - SQRT_3 * position(0) - position(1));
  V1 = -4 * position(2) + K1;

  K2 = 2 * position(2) +
       (middlevalue - (staticR - dynar) * (SQRT_3 * position(0) - position(1))) /
           upper_arm;
  U2 = -2 * (2 * (staticR - dynar) + SQRT_3 * position(0) - position(1));
  V2 = -4 * position(2) + K2;

  K3 =
      position(2) + (middlevalue - 2 * position(1) * (staticR - dynar)) / (2 * upper_arm);
  U3 = -2 * (staticR - dynar + position(1));
  V3 = -2 * position(2) + K3;

  t1 = (-U1 - sqrt(pow(U1, 2) - 4 * K1 * V1)) / (2 * K1);
  t2 = (-U2 - sqrt(pow(U2, 2) - 4 * K2 * V2)) / (2 * K2);
  t3 = (-U3 - sqrt(pow(U3, 2) - 4 * K3 * V3)) / (2 * K3);

  double theta1 = PI / 2 - 2 * atan(t1);
  double theta2 = PI / 2 - 2 * atan(t2);
  double theta3 = PI / 2 - 2 * atan(t3);
  theta_vector(0) = theta1;
  theta_vector(1) = theta2;
  theta_vector(2) = theta3;
  if (debug)
    ROS_INFO("\033[1;32mThe calculated angles are %.2lf, %.2lf, %.2lf.\033[0m", theta1,
             theta2, theta3);
}

void DeltaDisplay::FK_kin(double theta1, double theta2, double theta3,
                          Eigen::Vector3d &position)
{
  double A1, A2;
  double B1, B13;
  double B2;
  double B23;
  double B3;
  double C1, C2, C23, C3, C13;
  double D1;
  double D2;
  double E2, E1, F1, F2;
  double a, b, c;
  double lower_arm = this->l;
  double staticR = this->R;
  double dynar = this->r;
  double upper_arm = this->L;
  A1 = sqrt(3) / 2.0 * (staticR + upper_arm * sin(theta1) - dynar);
  B1 = (staticR + upper_arm * sin(theta1) - dynar) / 2.0;
  C1 = upper_arm * (cos(theta1));

  A2 = -sqrt(3) / 2.0 * (staticR + upper_arm * sin(theta2) - dynar);
  B2 = (staticR + upper_arm * sin(theta2) - dynar) / 2.0;
  C2 = upper_arm * cos(theta2);

  B3 = staticR + upper_arm * sin(theta3) - dynar;
  C3 = upper_arm * cos(theta3);

  D1 = (A1 * A1 + B1 * B1 + C1 * C1 - B3 * B3 - C3 * C3) / 2.0;
  D2 = (A2 * A2 + B2 * B2 + C2 * C2 - B3 * B3 - C3 * C3) / 2.0;
  B13 = B1 + B3;
  C13 = C1 - C3;
  B23 = B2 + B3;
  C23 = C2 - C3;
  E2 = (A2 * C13 - A1 * C23) / (A2 * B13 - A1 * B23);
  F2 = (A2 * D1 - A1 * D2) / (A2 * B13 - A1 * B23);
  E1 = (B13 * C23 - B23 * C13) / (A2 * B13 - A1 * B23);
  F1 = (B13 * D2 - B23 * D1) / (A2 * B13 - A1 * B23);
  a = E1 * E1 + E2 * E2 + 1;
  b = 2 * E2 * F2 + 2 * B3 * E2 + 2 * E1 * F1 + 2 * C3;
  c = F2 * F2 + B3 * B3 + 2 * B3 * F2 + F1 * F1 + C3 * C3 - lower_arm * lower_arm;
  position(2) = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
  position(0) = E1 * position(2) + F1;
  position(1) = E2 * position(2) + F2;

  if (debug)
    ROS_INFO("\033[1;32mThe calculated position is %.2lf, %.2lf, %.2lf.\033[0m",
             position(0) / scale, position(1) / scale, position(2) / scale);
}

/*
 the x axis concides with the robotic arm

   /|\ y
    |
    |
  z X------->----------------------------------O
            x
*/

void DeltaDisplay::drivenArmsQuaternion(Eigen::Vector3d v, geometry_msgs::Quaternion &q)
{
  // get the representation of 'axis-angle' from the x_a to the v
  Eigen::Vector3d x_a(1, 0, 0);  // the init x axis is 1,0,0
  Eigen::Vector3d cross1 = x_a.cross(v).normalized();
  // cal the included angle
  double rad = std::acos(x_a.dot(v) / (x_a.norm() * v.norm()));
  Eigen::AngleAxisd AngleAxisd1(rad, cross1);
  Eigen::Matrix3d R1 = AngleAxisd1.matrix();

  // we expect that the z axis of the arms is parallel to the x-y plane and
  // perpendicular to x axis so calculate the orientation of the z axis of the
  // arms
  Eigen::Vector3d z_a(0, 0, 1);
  z_a = R1 * z_a;
  Eigen::Vector3d xyplane(-v.y(), v.x(), 0);
  xyplane = xyplane.normalized();

  // the second transformation is to make the z axis parallel to the x-y plane
  double rad2 = std::acos(z_a.dot(xyplane));
  Eigen::Vector3d cross2 = z_a.cross(xyplane).normalized();  // rotate along the x axis
  Eigen::AngleAxisd angleAxisd2(rad2, cross2);
  Eigen::Matrix3d R2 = angleAxisd2.matrix();
  Eigen::Matrix3d R = R2 * R1;

  Eigen::Quaterniond eigenQuaternion(R);
  eigenQuaternion.normalize();
  q.w = eigenQuaternion.w();
  q.x = eigenQuaternion.x();
  q.y = eigenQuaternion.y();
  q.z = eigenQuaternion.z();
}

/*
    the active arm is like this:
    ------ A -----
            \
             \
              \
               \B
            Bl/ / Br
             / /
            / /
           / /
       Cl ====  Cr
            C

    The bottom is like this:
         ______
        /      \
       /        \
       \        /
        \___|__/
      Cl   C   Cr
*/

void DeltaDisplay::getJointPoints(double x, double y, double z, const double *theta,
                                  std::vector<Eigen::Vector3d> &A,
                                  std::vector<Eigen::Vector3d> &B,
                                  std::vector<Eigen::Vector3d> &C,
                                  std::vector<Eigen::Vector3d> &C_left_array,
                                  std::vector<Eigen::Vector3d> &C_right_array,
                                  std::vector<Eigen::Vector3d> &B_left_array,
                                  std::vector<Eigen::Vector3d> &B_right_array)
{
  // -------- Point A ----------
  for (int i = 0; i < 3; i++)
  {
    Eigen::Vector3d p;
    p.x() = R * cos(phi(i));
    p.y() = R * sin(phi(i));
    p.z() = 0;
    A.push_back(p);
  }

  // -------- Point B ----------
  for (int i = 0; i < 3; i++)
  {
    Eigen::Vector3d p;
    p.x() = R * cos(phi(i)) + L * cos(theta[i]) * cos(phi(i));
    p.y() = R * sin(phi(i)) + L * cos(theta[i]) * sin(phi(i));
    p.z() = 0 - L * sin(theta[i]);
    B.push_back(p);
  }

  // -------- Point C ----------
  for (int i = 0; i < 3; i++)
  {
    Eigen::Vector3d p;
    p.x() = r * cos(phi(i)) + x;
    p.y() = r * sin(phi(i)) + y;
    p.z() = 0 + z;
    C.push_back(p);
  }

  // -------- Point Cl and Cr----------
  double radian_COCr, len_ClCr;
  radian_COCr = 60 / 2 * ang2rad;
  len_ClCr = 2 * r * sin(radian_COCr);
  Eigen::Matrix3d R;
  double rot_rad[3] = {phi(0) - phi(2), phi(1) - phi(2), 0};

  for (int i = 0; i < 3; i++)
  {
    Eigen::Vector3d Cleft;
    Eigen::Vector3d Cright;
    R << cos(rot_rad[i]), -sin(rot_rad[i]), 0, sin(rot_rad[i]), cos(rot_rad[i]), 0, 0, 0,
        1;
    Cleft.x() = r * cos(phi(2) - radian_COCr);
    Cleft.y() = r * sin(phi(2) - radian_COCr);
    Cleft.z() = z;
    Cright.x() = r * cos(phi(2) + radian_COCr);
    Cright.y() = r * sin(phi(2) + radian_COCr);
    Cright.z() = z;
    Eigen::Vector3d tmpCl = R * Cleft;
    tmpCl.x() = tmpCl.x() + x;
    tmpCl.y() = tmpCl.y() + y;
    C_left_array.push_back(tmpCl);
    Eigen::Vector3d tmpCr = R * Cright;
    tmpCr.x() = tmpCr.x() + x;
    tmpCr.y() = tmpCr.y() + y;
    C_right_array.push_back(tmpCr);
  }
  // -------- Point Bl and Br----------

  for (int i = 0; i < 3; i++)
  {
    R << cos(rot_rad[i]), -sin(rot_rad[i]), 0, sin(rot_rad[i]), cos(rot_rad[i]), 0, 0, 0,
        1;
    Eigen::Vector3d tmpl(-len_ClCr / 2, 0, 0);
    Eigen::Vector3d tmpr(len_ClCr / 2, 0, 0);
    Eigen::Vector3d tmpBl = B[i] + R * tmpl;
    Eigen::Vector3d tmpBr = B[i] + R * tmpr;

    B_left_array.push_back(tmpBl);
    B_right_array.push_back(tmpBr);
  }
}
}  // namespace delta_display
