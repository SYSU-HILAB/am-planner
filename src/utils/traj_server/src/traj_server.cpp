#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
using namespace std;
#define PI acos(-1)
#define hover_yaw 0
const int _DIM_x = 0;
const int _DIM_y = 1;
const int _DIM_z = 2;

double start_x;
double start_y;
double start_z;
vector<Eigen::Vector3d> history_poslist;
Eigen::Vector4d target_detected_pos;
int vis_id = 0;
double initArmZ;
double pos_all_mse = 0, vel_all_mse = 0;
double pos_all_mse_x = 0, pos_all_mse_y = 0, pos_all_mse_z = 0;
int count_mse = 0;

bool rcv_arm_flag = false;
bool rcv_traj_flag = false;

bool debug;
bool is_sim = false;
bool for_arm = false;  /* if consider delta arm */
bool for_base = false; /* if consider base */

// 添加轨迹记录相关变量
const double PATH_RECORD_THRESHOLD = 0.01;       // 记录轨迹点的距离阈值
visualization_msgs::Marker history_path_marker;  // 实际轨迹
visualization_msgs::Marker desired_path_marker;  // 期望轨迹
Eigen::Vector3d last_recorded_pos;
Eigen::Vector3d last_recorded_des_pos;

class TrajectoryServer
{
 private:
  // Subscribers
  ros::Subscriber _odom_sub;
  ros::Subscriber _traj_sub;
  ros::Subscriber _traj_arm_sub;
  ros::Subscriber _predict_sub;

  // publishers
  ros::Publisher _cmd_pub;
  ros::Publisher _cmd_arm_pub;
  ros::Publisher _vis_cmd_pub;
  ros::Publisher _vis_cmd_arm_pub;
  ros::Publisher _vis_vel_pub;
  ros::Publisher _vis_acc_pub;
  ros::Publisher _vis_traj_pub;
  ros::Publisher _vis_pos_pub;
  ros::Publisher _path_pub;      // 实际轨迹发布器
  ros::Publisher _des_path_pub;  // 期望轨迹发布器

  // configuration for trajectory
  int _n_segment = 0;
  int _traj_id = 0;
  uint32_t _traj_flag = 0;

  int _n_segment_arm = 0;
  int _traj_id_arm = 0;
  uint32_t _traj_flag_arm = 0;

  Eigen::VectorXd _time;
  vector<Eigen::MatrixXd> _normalizedcoeflist;
  vector<Eigen::MatrixXd> _normalizedcoeflist_arm;
  vector<int> _order;
  vector<int> _order_arm;

  double _vis_traj_width = 0.05;
  double _mag_coeff = 1;
  ros::Time _final_time = ros::TIME_MIN;
  ros::Time _start_time = ros::TIME_MAX;
  bool _rcv_odom = false;

  double _start_yaw = 0.0, _final_yaw = 0.0;
  enum ServerState
  {
    INIT = 0,
    TRAJ,
    HOVER
  } state = INIT;
  enum ArmState
  {
    ARMINIT = 0,
    ARMTRAJ,
  } arm_state = ARMINIT;

  nav_msgs::Odometry _odom;
  quadrotor_msgs::PositionCommand _cmd;
  quadrotor_msgs::PositionCommand _last_cmd;
  quadrotor_msgs::PositionCommand _arm_cmd;

  quadrotor_msgs::PositionCommand _vis_cmd;
  visualization_msgs::Marker _vis_vel, _vis_acc, _vis_pos, _vis_traj;

  sensor_msgs::PointCloud2 traj_pts;
  pcl::PointCloud<pcl::PointXYZ> traj_pts_pcd;

 public:
  vector<Eigen::VectorXd> CList;   // Position coefficients vector, used to record all the
                                   // pre-compute 'n choose k' combinatorial for the
                                   // bernstein coefficients .
  vector<Eigen::VectorXd> CvList;  // Velocity coefficients vector.
  vector<Eigen::VectorXd> CaList;  // Acceleration coefficients vector.

  TrajectoryServer(ros::NodeHandle &handle)
  {
    /*
      Odometry subscriber
    */
    _odom_sub = handle.subscribe("odometry", 50, &TrajectoryServer::rcvOdometryCallback,
                                 this, ros::TransportHints().tcpNoDelay());

    /*
      Trajectory subscriber
    */
    _traj_sub =
        handle.subscribe("trajectory", 2, &TrajectoryServer::rcvTrajectoryCallabck, this);
    _traj_arm_sub = handle.subscribe("trajectory_arm", 2,
                                     &TrajectoryServer::rcvArmTrajectoryCallabck, this);
    _predict_sub = handle.subscribe("front_pos_forpredict", 1,
                                    &TrajectoryServer::frontPosPredictCallback, this);
    /*
      Real-world publisher
    */
    _cmd_pub = handle.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);
    _cmd_arm_pub =
        handle.advertise<quadrotor_msgs::PositionCommand>("position_cmd_delta", 50);

    /*
      Visualization body
    */
    _vis_cmd_pub = handle.advertise<quadrotor_msgs::PositionCommand>("position_des", 50);
    _vis_cmd_arm_pub = handle.advertise<geometry_msgs::Point>("body/end_effector", 50);

    /*
      Visualization marker
    */
    _vis_pos_pub = handle.advertise<visualization_msgs::Marker>("desired_pos", 1);
    _vis_vel_pub = handle.advertise<visualization_msgs::Marker>("desired_vel", 50);
    _vis_acc_pub = handle.advertise<visualization_msgs::Marker>("desired_acc", 50);
    _vis_traj_pub = handle.advertise<visualization_msgs::Marker>("desired_traj", 1);
    _path_pub = handle.advertise<visualization_msgs::Marker>("history_path", 1);
    _des_path_pub = handle.advertise<visualization_msgs::Marker>("desired_path", 1);

    double pos_gain[3] = {5.7, 5.7, 6.2};
    double vel_gain[3] = {3.4, 3.4, 4.0};
    setGains(pos_gain, vel_gain);

    last_recorded_pos = Eigen::Vector3d::Zero();
    last_recorded_des_pos = Eigen::Vector3d::Zero();

    visInit();
  }

  void visInit()
  {
    _vis_traj.header.stamp = ros::Time::now();
    _vis_traj.header.frame_id = "world";
    _vis_traj.ns = "traj_sever/trajectory";
    _vis_traj.id = vis_id;
    _vis_traj.type = visualization_msgs::Marker::LINE_STRIP;
    _vis_traj.action = visualization_msgs::Marker::ADD;
    _vis_traj.scale.x = _vis_traj_width / 1.3;
    _vis_traj.scale.y = _vis_traj_width / 1.3;
    _vis_traj.scale.z = _vis_traj_width / 1.3;
    _vis_traj.pose.orientation.x = 0.0;
    _vis_traj.pose.orientation.y = 0.0;
    _vis_traj.pose.orientation.z = 0.0;
    _vis_traj.pose.orientation.w = 1.0;
    _vis_traj.color.r = 1.0;
    _vis_traj.color.g = 1.0;
    _vis_traj.color.b = 0.0;
    _vis_traj.color.a = 1.0;
    _vis_traj.points.clear();

    _vis_pos.header.stamp = ros::Time::now();
    _vis_pos.header.frame_id = "world";
    _vis_pos.ns = "traj_server/desired_pos";
    _vis_pos.id = 0;
    _vis_pos.type = visualization_msgs::Marker::SPHERE_LIST;
    _vis_pos.action = visualization_msgs::Marker::ADD;
    _vis_pos.scale.x = _vis_traj_width;
    _vis_pos.scale.y = _vis_traj_width;
    _vis_pos.scale.z = _vis_traj_width;
    _vis_pos.pose.orientation.x = 0.0;
    _vis_pos.pose.orientation.y = 0.0;
    _vis_pos.pose.orientation.z = 0.0;
    _vis_pos.pose.orientation.w = 1.0;
    _vis_pos.color.r = 1.0;
    _vis_pos.color.g = 0.0;
    _vis_pos.color.b = 1.0;
    _vis_pos.color.a = 1.0;
    _vis_pos.points.clear();

    _vis_vel.header.stamp = ros::Time::now();
    _vis_vel.header.frame_id = "world";
    _vis_vel.ns = "traj_server/desired_vel";
    _vis_vel.id = 0;
    _vis_vel.type = visualization_msgs::Marker::ARROW;
    _vis_vel.action = visualization_msgs::Marker::ADD;
    _vis_vel.color.a = 1.0;
    _vis_vel.color.r = 0.0;
    _vis_vel.color.g = 1.0;
    _vis_vel.color.b = 0.0;
    _vis_vel.scale.x = 0.2;
    _vis_vel.scale.y = 0.4;
    _vis_vel.scale.z = 0.4;

    _vis_acc.header.stamp = ros::Time::now();
    _vis_acc.header.frame_id = "world";
    _vis_acc.ns = "traj_server/desired_acc";
    _vis_acc.id = 0;
    _vis_acc.type = visualization_msgs::Marker::ARROW;
    _vis_acc.action = visualization_msgs::Marker::ADD;
    _vis_acc.color.a = 1.0;
    _vis_acc.color.r = 1.0;
    _vis_acc.color.g = 1.0;
    _vis_acc.color.b = 0.0;
    _vis_acc.scale.x = 0.2;
    _vis_acc.scale.y = 0.4;
    _vis_acc.scale.z = 0.4;

    // Initialize history path marker
    history_path_marker.header.frame_id = "world";
    history_path_marker.ns = "history_path";
    history_path_marker.id = 0;
    history_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    history_path_marker.action = visualization_msgs::Marker::ADD;
    history_path_marker.scale.x = 0.05;  // 线宽
    history_path_marker.color.r = 58.0 / 255.0;
    history_path_marker.color.g = 181.0 / 255.0;
    history_path_marker.color.b = 179.0 / 255.0;
    history_path_marker.color.a = 1.0;
    history_path_marker.pose.orientation.w = 1.0;

    // Initialize desired path marker
    desired_path_marker = history_path_marker;
    desired_path_marker.ns = "desired_path";
    desired_path_marker.id = 1;
    desired_path_marker.color.r = 149.0 / 255.0;
    desired_path_marker.color.g = 79.0 / 255.0;
    desired_path_marker.color.b = 151.0 / 255.0;
  }

  void setGains(double pos_gain[3], double vel_gain[3])
  {
    _cmd.kx[_DIM_x] = pos_gain[_DIM_x];
    _cmd.kx[_DIM_y] = pos_gain[_DIM_y];
    _cmd.kx[_DIM_z] = pos_gain[_DIM_z];

    _cmd.kv[_DIM_x] = vel_gain[_DIM_x];
    _cmd.kv[_DIM_y] = vel_gain[_DIM_y];
    _cmd.kv[_DIM_z] = vel_gain[_DIM_z];
  }

  void frontPosPredictCallback(const nav_msgs::Path &front_pos)
  {
    target_detected_pos << front_pos.poses[0].pose.position.x,
        front_pos.poses[0].pose.position.y, front_pos.poses[0].pose.position.z,
        front_pos.poses[0].pose.orientation.x;
  }

  void rcvOdometryCallback(const nav_msgs::Odometry &odom)
  {
    // #1. store the odometry
    _odom = odom;
    _rcv_odom = true;

    // 获取当前位置
    Eigen::Vector3d current_pos(_odom.pose.pose.position.x, _odom.pose.pose.position.y,
                                _odom.pose.pose.position.z);

    // 计算与上一个记录点的距离
    double dist = (current_pos - last_recorded_pos).norm();

    // 如果距离超过阈值，记录新的点
    if (dist > PATH_RECORD_THRESHOLD)
    {
      geometry_msgs::Point pt;
      pt.x = current_pos(0);
      pt.y = current_pos(1);
      pt.z = current_pos(2);
      history_path_marker.points.push_back(pt);
      last_recorded_pos = current_pos;

      // 发布实际轨迹
      history_path_marker.header.stamp = ros::Time::now();
      _path_pub.publish(history_path_marker);
    }
  }

  void recordDesiredPath(const Eigen::Vector3d &des_pos)
  {
    // 计算与上一个记录的期望点的距离
    double dist = (des_pos - last_recorded_des_pos).norm();

    // 如果距离超过阈值，记录新的点
    if (dist > PATH_RECORD_THRESHOLD)
    {
      geometry_msgs::Point pt;
      pt.x = des_pos(0);
      pt.y = des_pos(1);
      pt.z = des_pos(2);
      desired_path_marker.points.push_back(pt);
      last_recorded_des_pos = des_pos;

      // 发布期望轨迹
      desired_path_marker.header.stamp = ros::Time::now();
      _des_path_pub.publish(desired_path_marker);
    }
  }

  void timer_callback(const ros::TimerEvent &event)
  {
    /* #1. INIT */
    if (state == INIT)
    {
      _cmd.header.stamp = ros::Time::now();
      _cmd.header.frame_id = "world";
      _cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

      // init vel
      _cmd.velocity.x = 0.0;
      _cmd.velocity.y = 0.0;
      _cmd.velocity.z = 0.0;

      // init acc
      _cmd.acceleration.x = 0.0;
      _cmd.acceleration.y = 0.0;
      _cmd.acceleration.z = 0.0;

      // init jer
      _cmd.jerk.x = 0.0;
      _cmd.jerk.y = 0.0;
      _cmd.jerk.z = 0.0;

      // init yaw
      _cmd.yaw = hover_yaw;

      // init pos
      if (!is_sim)
      {
        _cmd.position.x = start_x;
        _cmd.position.y = start_y;
        _cmd.position.z = start_z;
      }
      else
      {
        if (_rcv_odom)
        {
          _cmd.position.x = _odom.pose.pose.position.x;
          _cmd.position.y = _odom.pose.pose.position.y;
          _cmd.position.z = _odom.pose.pose.position.z;
          _cmd_pub.publish(_cmd);
        }
        else
        {
          // ROS_WARN("[TrajServer] Not sim. Waiting for the odometry message.");
        }
      }
      _vis_cmd = _cmd;
      _vis_cmd_pub.publish(_vis_cmd);
    }

    if (arm_state == ARMINIT)
    {
      _arm_cmd.header.stamp = ros::Time::now();
      _arm_cmd.header.frame_id = "body";
      _arm_cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
      _arm_cmd.position.x = 0.0;
      _arm_cmd.position.y = 0.0;
      _arm_cmd.position.z = -initArmZ;
      _cmd_arm_pub.publish(_arm_cmd);

      /* visual for Delta-Display */
      geometry_msgs::Point arm_init;
      arm_init.x = 0.0;
      arm_init.y = 0.0;
      arm_init.z = -initArmZ;
      _vis_cmd_arm_pub.publish(arm_init);
    }

    // #2. if complete the trajectory, hover
    if (state == TRAJ && ((ros::Time::now() - _start_time).toSec() / _mag_coeff >
                          (_final_time - _start_time).toSec()))
    {
      ROS_INFO("[TrajServer]: State change from TRAJ to HOVER");
      state = HOVER;
      // output mse
      double mse1 = pos_all_mse / count_mse;
      double mse2 = vel_all_mse / count_mse;
      double mse1x = pos_all_mse_x / count_mse;
      double mse1y = pos_all_mse_y / count_mse;
      double mse1z = pos_all_mse_z / count_mse;
      if (!is_sim)
      {
        ROS_INFO("[TrajServer]: MSE: pos_x = %.3lf, pos_y = %.3lf, pos_z = %.3lf.", mse1x,
                 mse1y, mse1z);
        ROS_INFO("[TrajServer]: MSE: pos = %.3lf, vel = %.3lf", mse1, mse2);
      }
      _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
    }

    if (arm_state == ARMTRAJ && ((ros::Time::now() - _start_time).toSec() / _mag_coeff >
                                 (_final_time - _start_time).toSec()))
    {
      ROS_INFO("[TrajServer]: Arm State change from ARMTRAJ to ARMINIT");
      arm_state = ARMINIT;
      _traj_flag_arm = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
    }

    // #3. try to publish command
    if (for_base) pubPositionCommand();
    if (for_arm) pubArmPositionCommand();
  }

  void pubArmPositionCommand()
  {
    if (arm_state == ARMINIT)
    {
      if (debug) ROS_INFO("[TrajServer] Arm State = ARMINIT.");
      return;
    }
    if (arm_state == TRAJ)
    {
      if (debug) ROS_INFO("[TrajServer] Arm State = ARMTRAJ.");
      _arm_cmd.header.stamp = ros::Time::now();
      _arm_cmd.header.frame_id = "body";

      _arm_cmd.trajectory_flag = _traj_flag_arm;
      _arm_cmd.trajectory_id = _traj_id_arm;

      double t = (ros::Time::now() - _start_time).toSec() / _mag_coeff;
      if (t < 0)
      {
        ROS_WARN("[TrajServer] Have not started the ARM trajectory.");
        return;
      }

      int seg_idx;  // locate the segment
      double dur;   // duration of the segment
      for (seg_idx = 0; seg_idx < _n_segment_arm && t > (dur = _time[seg_idx]); seg_idx++)
      {
        t -= dur;
      }
      if (seg_idx == _n_segment_arm)
      {
        seg_idx--;
        t += _time[seg_idx];
      }
      t /= _time[seg_idx];
      int cur_order = _order_arm[seg_idx];
      int cur_poly_num = cur_order + 1;
      Eigen::Vector3d pos, vel, acc, jer;
      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
      double t1, t2, t3, t4, t5;
      t1 = t;
      t2 = t1 * t1;
      t3 = t2 * t1;
      t4 = t3 * t1;
      t5 = t4 * t1;
      beta0 << t5, t4, t3, t2, t1, 1;
      beta1 << 5 * t4, 4 * t3, 3 * t2, 2 * t1, 1, 0;
      beta2 << 20 * t3, 12 * t2, 6 * t1, 2, 0, 0;
      beta3 << 60 * t2, 24 * t1, 6, 0, 0, 0;

      pos = _normalizedcoeflist_arm[seg_idx] * beta0;
      vel = _normalizedcoeflist_arm[seg_idx] * beta1 / _time[seg_idx];
      acc = _normalizedcoeflist_arm[seg_idx] * beta2 / (_time[seg_idx] * _time[seg_idx]);
      jer = _normalizedcoeflist_arm[seg_idx] * beta3 /
            (_time[seg_idx] * _time[seg_idx] * _time[seg_idx]);

      _arm_cmd.position.x = pos[0];
      _arm_cmd.position.y = pos[1];
      double ub = -0.06, lb = -0.23;
      if (pos[2] < lb || pos[2] > ub)
      {
        ROS_WARN("[TrajServer] Arm command out of bound: z = %.3lf", pos[2]);
      }
      _arm_cmd.position.z = min(max(pos[2], -0.23), -0.06);
      _arm_cmd.velocity.x = vel[0];
      _arm_cmd.velocity.y = vel[1];
      _arm_cmd.velocity.z = vel[2];
      _arm_cmd.acceleration.x = acc[0];
      _arm_cmd.acceleration.y = acc[1];
      _arm_cmd.acceleration.z = acc[2];
      _arm_cmd.jerk.x = jer[0];
      _arm_cmd.jerk.y = jer[1];
      _arm_cmd.jerk.z = jer[2];
    }
    _cmd_arm_pub.publish(_arm_cmd);

    geometry_msgs::Point arm_pos;
    arm_pos.x = _arm_cmd.position.x;
    arm_pos.y = _arm_cmd.position.y;
    arm_pos.z = _arm_cmd.position.z;
    _vis_cmd_arm_pub.publish(arm_pos);
  }
  void pubPositionCommand()
  {
    if (state == INIT)
    {
      if (debug) ROS_INFO("[TrajServer] State = INIT.");
      return;
    }
    if (state == HOVER)
    {
      if (debug) ROS_INFO("[TrajServer] State = HOVER.");
      // not to publish the command
      return;

      _cmd.header.stamp = _odom.header.stamp;
      _cmd.header.frame_id = "world";
      _cmd.trajectory_flag = _traj_flag;

      // _cmd.yaw = _odom.pose.pose.orientation.;
      _cmd.yaw = hover_yaw;
      _cmd.velocity.x = 0.0;
      _cmd.velocity.y = 0.0;
      _cmd.velocity.z = 0.0;

      _cmd.acceleration.x = 0.0;
      _cmd.acceleration.y = 0.0;
      _cmd.acceleration.z = 0.0;

      _cmd.jerk.x = 0.0;
      _cmd.jerk.y = 0.0;
      _cmd.jerk.z = 0.0;
    }
    if (state == TRAJ)
    {
      if (debug) ROS_INFO("[TrajServer] State = TRAJ.");

      _cmd.header.stamp = _odom.header.stamp;
      _cmd.header.frame_id = "world";
      _cmd.trajectory_flag = _traj_flag;
      _cmd.trajectory_id = _traj_id;
      double t = (ros::Time::now() - _start_time).toSec() / _mag_coeff;
      if (t < 0)
      {
        ROS_WARN("[TrajServer] Have not started the trajectory.");
        return;
      }

      int seg_idx;  // locate the segment
      double dur;   // duration of the segment
      for (seg_idx = 0; seg_idx < _n_segment && t > (dur = _time[seg_idx]); seg_idx++)
      {
        t -= dur;
      }
      if (seg_idx == _n_segment)
      {
        seg_idx--;
        t += _time[seg_idx];
      }
      t /= _time[seg_idx];
      int cur_order = _order[seg_idx];
      int cur_poly_num = cur_order + 1;
      Eigen::Vector3d pos, vel, acc, jer;
      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
      double t1, t2, t3, t4, t5;
      t1 = t;
      t2 = t1 * t1;
      t3 = t2 * t1;
      t4 = t3 * t1;
      t5 = t4 * t1;
      beta0 << t5, t4, t3, t2, t1, 1;
      beta1 << 5 * t4, 4 * t3, 3 * t2, 2 * t1, 1, 0;
      beta2 << 20 * t3, 12 * t2, 6 * t1, 2, 0, 0;
      beta3 << 60 * t2, 24 * t1, 6, 0, 0, 0;

      pos = _normalizedcoeflist[seg_idx] * beta0;
      vel = _normalizedcoeflist[seg_idx] * beta1 / _time[seg_idx];
      acc = _normalizedcoeflist[seg_idx] * beta2 / (_time[seg_idx] * _time[seg_idx]);
      jer = _normalizedcoeflist[seg_idx] * beta3 /
            (_time[seg_idx] * _time[seg_idx] * _time[seg_idx]);

      _cmd.position.x = pos[0];
      _cmd.position.y = pos[1];
      _cmd.position.z = pos[2];
      _cmd.velocity.x = vel[0];
      _cmd.velocity.y = vel[1];
      _cmd.velocity.z = vel[2];
      _cmd.acceleration.x = acc[0];
      _cmd.acceleration.y = acc[1];
      _cmd.acceleration.z = acc[2];
      _cmd.jerk.x = jer[0];
      _cmd.jerk.y = jer[1];
      _cmd.jerk.z = jer[2];
      _cmd.yaw = hover_yaw;
      _cmd.yaw_dot = 0.01;

      // 记录期望轨迹点
      recordDesiredPath(pos);
    }
    _last_cmd = _cmd;
    _cmd_pub.publish(_cmd);

    /* calculate MSE */
    if (_rcv_odom)
    {
      Eigen::Vector3d cmd_position(_cmd.position.x, _cmd.position.y, _cmd.position.z);
      Eigen::Vector3d cmd_velocity(_cmd.velocity.x, _cmd.velocity.y, _cmd.velocity.z);
      double pos_mse = (cmd_position - Eigen::Vector3d(_odom.pose.pose.position.x,
                                                       _odom.pose.pose.position.y,
                                                       _odom.pose.pose.position.z))
                           .norm();
      double pos_mse_x = abs(cmd_position(0) - _odom.pose.pose.position.x);
      double pos_mse_y = abs(cmd_position(1) - _odom.pose.pose.position.y);
      double pos_mse_z = abs(cmd_position(2) - _odom.pose.pose.position.z);

      double vel_mse = (cmd_velocity - Eigen::Vector3d(_odom.twist.twist.linear.x,
                                                       _odom.twist.twist.linear.y,
                                                       _odom.twist.twist.linear.z))
                           .norm();
      if (true)
      {
        // ROS_INFO("[TrajServer] MSE: pos = %.3lf, vel = %.3lf", pos_mse, vel_mse);
        pos_all_mse += pos_mse;
        vel_all_mse += vel_mse;
        pos_all_mse_x += pos_mse_x;
        pos_all_mse_y += pos_mse_y;
        pos_all_mse_z += pos_mse_z;

        count_mse++;
      }
    }
    else
    {
      if (!is_sim) ROS_WARN("[TrajServer] Not receive the odometry message.");
    }

    /* visual the model in Delta-Display */
    _vis_cmd = _cmd;
    _vis_cmd_pub.publish(_vis_cmd);

    geometry_msgs::Point pt;
    pt.x = _cmd.position.x;
    pt.y = _cmd.position.y;
    pt.z = _cmd.position.z;
    /* visual desired position */
    _vis_pos.points.clear();
    _vis_pos.points.push_back(pt);
    _vis_pos_pub.publish(_vis_pos);

    /* visual desired velocity */
    _vis_vel.header.stamp = ros::Time::now();
    _vis_vel.points.clear();
    _vis_vel.points.push_back(pt);
    geometry_msgs::Point pt_vel;
    pt_vel.x = _cmd.position.x + _cmd.velocity.x;
    pt_vel.y = _cmd.position.y + _cmd.velocity.y;
    pt_vel.z = _cmd.position.z + _cmd.velocity.z;
    _vis_vel.points.push_back(pt_vel);
    _vis_vel_pub.publish(_vis_vel);

    /* visual desired trajectory */
    if (_vis_traj.points.size() > 100000)
      _vis_traj.points.erase(_vis_traj.points.begin());
    _vis_traj.points.push_back(pt);
    _vis_traj_pub.publish(_vis_traj);

    /* visual desired acceleration */
    _vis_acc.header.stamp = ros::Time::now();
    _vis_acc.points.clear();
    _vis_acc.points.push_back(pt);
    geometry_msgs::Point pt_acc;
    pt_acc.x = _cmd.position.x + _cmd.acceleration.x;
    pt_acc.y = _cmd.position.y + _cmd.acceleration.y;
    pt_acc.z = _cmd.position.z + _cmd.acceleration.z;
    _vis_acc.points.push_back(pt);
    _vis_acc_pub.publish(_vis_acc);
  }

  void rcvTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory &traj)
  {
    if (rcv_traj_flag || !for_base)  // receive the trajectory once
    {
      return;
    }
    rcv_traj_flag = true;
    ROS_INFO("[TrajServer] Received a new trajectory with %d segments.",
             traj.num_segment);
    if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
    {
      state = TRAJ;
      ROS_INFO("[TrajServer] State change from INIT to TRAJ.");
      _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
      _traj_id = traj.trajectory_id;
      _n_segment = traj.num_segment;

      _final_time = _start_time = ros::Time::now();  // reset time
      _time.resize(_n_segment);
      _order.resize(_n_segment);

      _time.setZero();
      ;
      _order.clear();
      _normalizedcoeflist.clear();

      // store the time, order and final_time
      for (int idx = 0; idx < _n_segment; ++idx)
      {
        _final_time += ros::Duration(traj.time[idx]);
        _time(idx) = traj.time[idx];
        _order.push_back(traj.order[idx]);
      }

      _start_yaw = traj.start_yaw;
      _final_yaw = traj.final_yaw;
      _mag_coeff = traj.mag_coeff;

      int shift = 0;
      for (int idx = 0; idx < _n_segment; idx++)
      {
        int order = traj.order[idx];
        Eigen::MatrixXd coefmat;
        coefmat = Eigen::MatrixXd::Zero(3, order + 1);

        for (int j = 0; j < order + 1; j++)
        {
          coefmat(0, j) = traj.coef_x[shift + j];
          coefmat(1, j) = traj.coef_y[shift + j];
          coefmat(2, j) = traj.coef_z[shift + j];
        }
        _normalizedcoeflist.push_back(coefmat);
        shift += (order + 1);
      }
    }
    else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT)
    {
      ROS_WARN("[TrajServer] Aborting the trajectories!");
      state = HOVER;
      ROS_INFO("[TrajServer] State change to HOVER.");
      _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
    }
    else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
    {
      ROS_WARN("[TrajServer] Impossible to execute the trajectories!");
      state = HOVER;
      ROS_INFO("[TrajServer] State change to HOVER.");
      _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
    }
    else
    {
      ROS_WARN("[TrajServer] Unknown action!");
    }
  }

  void rcvArmTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory &traj)
  {
    if (rcv_arm_flag || !for_arm)
    {
      return;
    }
    rcv_arm_flag = true;
    ROS_INFO("[TrajServer] Received a new arm trajectory with %d segments.",
             traj.num_segment);

    if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
    {
      arm_state = ARMTRAJ;
      ROS_INFO("[TrajServer] Arm State change from INIT to TRAJ.");
      _traj_flag_arm = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
      _traj_id_arm = traj.trajectory_id;
      _n_segment_arm = traj.num_segment;
      _mag_coeff = traj.mag_coeff;
      _final_time = _start_time = ros::Time::now();  // reset time

      _time.resize(_n_segment_arm);
      _order.resize(_n_segment_arm);
      _time.setZero();
      _order_arm.clear();
      _normalizedcoeflist_arm.clear();

      // store the time, order and final_time
      for (int idx = 0; idx < _n_segment_arm; ++idx)
      {
        _final_time += ros::Duration(traj.time[idx]);
        _time(idx) = traj.time[idx];
        _order_arm.push_back(traj.order[idx]);
      }

      int shift = 0;
      for (int idx = 0; idx < traj.num_segment; idx++)
      {
        int order = traj.order[idx];
        Eigen::MatrixXd coefmat;
        coefmat = Eigen::MatrixXd::Zero(3, order + 1);

        for (int j = 0; j <= order; j++)
        {
          coefmat(0, j) = traj.coef_x[shift + j];
          coefmat(1, j) = traj.coef_y[shift + j];
          coefmat(2, j) = traj.coef_z[shift + j];
        }
        _normalizedcoeflist_arm.push_back(coefmat);
        shift += (order + 1);
      }
    }
    else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT)
    {
      ROS_WARN("[TrajServer] Aborting the ARM trajectories!");
      _traj_flag_arm = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
    }
    else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
    {
      ROS_WARN("[TrajServer] Impossible to execute the ARM trajectories!");
      _traj_flag_arm = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
    }
    else
    {
      ROS_WARN("[TrajServer] Unknown ARM action!");
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server_node");
  ros::NodeHandle handle("~");

  handle.param("/start_x", start_x, 0.0);
  handle.param("/start_y", start_y, 0.0);
  handle.param("/start_z", start_z, 0.0);
  handle.param("debug", debug, false);
  handle.param("is_sim", is_sim, false);
  handle.param("for_arm", for_arm, false);
  handle.param("for_base", for_base, false);
  handle.param("InitArmZ", initArmZ, 0.07);

  if (!for_arm && !for_base)
  {
    ROS_ERROR("[TrajServer] Please specify the trajectory type: for_arm or for_base.");
    return -1;
  }
  if (for_arm)
  {
    ROS_INFO("[TrajServer] Trajectory for arm is enabled.");
  }
  if (for_base)
  {
    ROS_INFO("[TrajServer] Trajectory for base is enabled.");
  }

  sleep(2);
  TrajectoryServer server(handle);
  // creat timers per 0.2s
  ros::Timer timer =
      handle.createTimer(ros::Duration(0.01), &TrajectoryServer::timer_callback, &server);

  ros::spin();

  return 0;
}
