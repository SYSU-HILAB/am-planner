#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>  // Changed from std_srvs/Empty.h to std_msgs/Float64.h
#include <std_msgs/Float64MultiArray.h>
#include <traj_opt/config.h>
#include <traj_opt/se3_planner.h>

#define PI 3.1415926

#define USE_TRIGGER_START false
#define USE_TRIGGER_PUBLISH false

/* plan fsm state */
class PlannerManage
{
 public:
  PlannerManage(ros::NodeHandle &nh) : nh_(nh)
  {
    /* init the start and end */
    initModeAndStartAndEnd();

    /* init the ros */
    initRos();

    /* init the ros utils */
    config_.loadParameters(nh);

    /* init the se3 planner */
    planner_ = new GlobalPlanner(config_, nh, 1);

    /* cf. config.h
      MODE_GRASP = 0
      MODE_STRIKE = 1
      MODE_OTHER = 2
    */
    if (mode_ == MODE_GRASP)
    {
      graspModeInit();
    }
    if (mode_ == MODE_STRIKE)
    {
      strikeModeInit();
    }
    if (mode_ == MODE_OTHER)
    {
      otherModeInit();
    }

    /* load the pre and post point, i.e. guide points mainly */
    initPreAndPost();

    if (!checkInterInfo())
    {
      ROS_ERROR("[MANAGE]: Inter info size error!");
      exit(1);
    }

    ROS_INFO("[MANAGE]: PlannerManage init done!");
  }

  /* the most important function, which is called by the timer by frequency of 1hz */
  void timer(const ros::TimerEvent &event)
  {
    if (use_trigger_start_ && !trigger_start_)
    {
      /* we don't use trigger start in simulation */
      return;
    }
    if (traj_ready_)
    {
      /* plan once only */
      return;
    }
    if (!has_quadrotor_odom_)
    {
      /* we don't use quadrotor odom in simulation */
      /* and the start odom loaded in launch file */
      ROS_ERROR("[MANAGE]: Cannot plan without quadrotor odom data!");
      return;
    }
    if (!has_object_odom_ && (mode_ == MODE_GRASP || mode_ == MODE_STRIKE))
    {
      /* we don't use object odom in simulation */
      /* and the object odom loaded in launch file */
      ROS_WARN("[MANAGE]: Cannot plan without object odom data!");
      return;
    }
    double cost;
    std::vector<double> logs;
    if (plan(cost, logs)) /* plan the trajectory */
    {
      TrajToMsg(traj_arm_, traj_arm_msg_);
      TrajToMsg(traj_, traj_msg_);

      { /* publish the cost and the points (optional) */
        std::cout << "[MANAGE]: Published cost " << std::fixed << std::setprecision(3)
                  << cost << " to /finish_simulation topic." << std::endl;
        std_msgs::Float64MultiArray logger_msg;
        if (logs.size() == 0)
        {
          ROS_ERROR("[MANAGE]: No logs to publish!");
        }
        else
        {
          for (int i = 0; i < logs.size(); i++)
          {
            logger_msg.data.push_back(logs[i]);
          }
          logger_pub_.publish(logger_msg);
        }
      }

      std_msgs::Float64 msg;
      msg.data = cost;
      finish_simulation_pub_.publish(msg);

      traj_ready_ = true;
      if (!use_traj_pub_trigger_)
      {
        publishTriggerCallback(nullptr);
      }
      else
      {
        ROS_INFO(
            "\033[1;32m[MANAGE]: Trajectory planning completed, waiting for publish "
            "trigger!\033[0m");
      }
    }
    else
    {
      std::cout << "[MANAGE]: Waiting for PCL data!" << std::endl;
      return;
    }
  }

  bool plan(double &cost, std::vector<double> &logs)
  {
    /*
     cost is the return of final coat
     logs is the output information of optimization
    */

    /* global planning setup*/
    /* init_state_ is the start state of quadrotor */
    /* fin_state_ is the end state of quadrotor */
    /* mode_ is the mode of the planner */
    /* inter_info_ consist of all the waypoint constriants */
    planner_->Setup(init_state_, fin_state_, mode_, inter_info_); /* cf. se3_planner.h */
    /* visualize the obeject plane */
    /* plan the trajectory */
    if (!planner_->Plan(cost, logs))
    {
      return false;
    }
    /* derive the trajectory from global planner */
    planner_->OutputTraj(traj_, traj_arm_);
    /* visualize the trajectory */
    planner_->VisualTraj();
    return true;
  }

  /************ Odom Callback *************/

  void objectOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (use_object_odom_)
    {
      Eigen::Quaterniond obj_q;
      Eigen::Vector3d obj_p;
      Eigen::Matrix3d obj_R;
      Eigen::VectorXd info_vec(17);

      obj_q =
          Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      obj_p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                              msg->pose.pose.position.z);
      obj_R = obj_q.toRotationMatrix();
      info_vec(0) = 2.0;
      info_vec.segment(1, 3) = obj_p;
      info_vec.segment(4, 3) = obj_R.col(2);
      info_vec[7] = 1.0;
      info_vec.segment(14, 3) = Eigen::Vector3d(0, 0, 0);

      if (mode_ == MODE_GRASP)
      {
        info_vec.segment(8, 3) = Eigen::Vector3d(0, 0, -config_.EEDownVel);
        info_vec.segment(11, 3) = Eigen::Vector3d(1, 1, 1);
        info_vec.segment(14, 3) = Eigen::Vector3d(0, 0, 0);
      }
      else if (mode_ == MODE_STRIKE)
      {
        info_vec.segment(8, 3) = Eigen::Vector3d(0.1, 0, 0);
        info_vec.segment(11, 3) = Eigen::Vector3d(2, 0, 0);
        info_vec.segment(14, 3) = Eigen::Vector3d(0, 0, 0);
      }
      else
      {
        ROS_INFO("Warning: Object odom callback is not used for mode %d", mode_);
        return;
      }
      inter_info_[0] = info_vec;
      has_object_odom_ = true;
      visualObjectPosition(obj_p);
    }
  }

  void amOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
  {
    if (use_external_odom_)
    {
      init_state_.col(0) << odom->pose.pose.position.x, odom->pose.pose.position.y,
          odom->pose.pose.position.z;
      init_state_.col(1) << odom->twist.twist.linear.x, odom->twist.twist.linear.y,
          odom->twist.twist.linear.z;
      init_state_.col(2) << 0, 0, 0;
      has_quadrotor_odom_ = true;
    }
  }

  /************ Trigger Callback *************/

  void startTriggerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (plan_once_)
    {
      ROS_ERROR("[MANAGE]: The planner has already run once. Please restart the node.");
      return;
    }
    if (!has_quadrotor_odom_)
    {
      ROS_ERROR("[MANAGE]: Waiting for odom data before planning!");
      return;
    }
    if (!has_object_odom_)
    {
      ROS_ERROR("[MANAGE]: Waiting for object odom data before planning!");
      return;
    }
    plan_once_ = true;
    trigger_start_ = true;
  }

  void publishTriggerCallback(const std_msgs::Empty::ConstPtr &msg)
  {
    if (traj_ready_)
    {
      traj_arm_pub_.publish(traj_arm_msg_);
      sleep(0.5);
      traj_pub_.publish(traj_msg_);
      ROS_INFO("\033[1;32m[MANAGE]: The trajectory is published!\033[0m");
    }
    else
    {
      ROS_INFO("\033[1;32m[MANAGE]: Waiting for trajectory to be ready!\033[0m");
    }
  }

  /************ Utils *************/

  template <int D>
  void TrajToMsg(const Trajectory<D> &traj,
                 quadrotor_msgs::PolynomialTrajectory &traj_msg)
  {
    /* merge the two trajectories */
    static int count = 0;
    traj_msg.header.seq = count;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = config_.odomFrame;
    traj_msg.trajectory_id = count;
    traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj_msg.num_segment = traj.getPieceNum();
    traj_msg.start_yaw = 0;
    traj_msg.final_yaw = 0;

    for (int i = 0; i < traj.getPieceNum(); i++)
    {
      for (int j = 0; j <= traj[i].getOrder(); j++)
      {
        CoefficientMat<D> coemat = traj[i].normalizePosCoeffMat();
        traj_msg.coef_x.push_back(coemat(0, j));
        traj_msg.coef_y.push_back(coemat(1, j));
        traj_msg.coef_z.push_back(coemat(2, j));
      }
      traj_msg.time.push_back(traj[i].getDuration());
      traj_msg.order.push_back(traj[i].getOrder());
    }
    traj_msg.mag_coeff = 1;
    count++;
  }

  void visualObjectPosition(Eigen::Vector3d obj_p)
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "object";
    marker.id = 0;  // Use different id for each marker
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.pose.position.x = obj_p.x();
    marker.pose.position.y = obj_p.y();
    marker.pose.position.z = obj_p.z();
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker_array.markers.push_back(marker);
    visual_obj_pub_.publish(marker_array);
  }

  /* init function for different modes */

  void graspModeInit()
  {
    Eigen::Vector3d obj_p;
    Eigen::Matrix3d obj_R;
    Eigen::Quaterniond q;

    obj_p.setZero();
    obj_R.setIdentity();
    nh_.param("/object_height", object_height_, 0.1); /* set in desk.py */
    nh_.param("use_object_odom", use_object_odom_, false);
    if (!use_object_odom_)
    {
      nh_.getParam("/object_px", obj_p.x());
      nh_.getParam("/object_py", obj_p.y());
      nh_.getParam("/object_pz", obj_p.z());
      nh_.getParam("/object_qw", q.w());
      nh_.getParam("/object_qx", q.x());
      nh_.getParam("/object_qy", q.y());
      nh_.getParam("/object_qz", q.z());
      obj_R = q.toRotationMatrix();
      Eigen::VectorXd info_vec(17);
      info_vec(0) = 2.0;
      info_vec.segment(1, 3) = obj_p;                                      // position
      info_vec.segment(4, 3) = obj_R.col(2);                               // orientation
      info_vec[7] = 1.0;                                                   // if cons ori.
      info_vec.segment(8, 3) = Eigen::Vector3d(0, 0, -config_.EEDownVel);  // velocity
      info_vec.segment(11, 3) = Eigen::Vector3d(1, 1, 1);                  // if cons vel.
      info_vec.segment(14, 3) = Eigen::Vector3d(0, 0, 0);  // if fix any axis
      inter_info_.push_back(info_vec);
      has_object_odom_ = true;
      visualObjectPosition(obj_p);
    }
    else
    {
      /* Add a placeholder intermediate point, will be updated later */
      Eigen::VectorXd info_vec = Eigen::VectorXd::Zero(17);
      info_vec(0) = 2.0;
      inter_info_.push_back(info_vec);
    }
    ROS_INFO("[MANAGE]: Grasp mode init!");
  }

  void strikeModeInit()
  {
    Eigen::Vector3d obj_p;
    Eigen::Matrix3d obj_R;
    Eigen::Quaterniond q;
    obj_p.setZero();
    obj_R.setIdentity();
    nh_.param("/object_height", object_height_, 0.1); /* set in desk.py */
    nh_.param("use_object_odom", use_object_odom_, false);
    if (!use_object_odom_)
    {
      has_object_odom_ = true;

      nh_.getParam("/object_px", obj_p.x());
      nh_.getParam("/object_py", obj_p.y());
      nh_.getParam("/object_pz", obj_p.z());
      nh_.getParam("/object_qw", q.w());
      nh_.getParam("/object_qx", q.x());
      nh_.getParam("/object_qy", q.y());
      nh_.getParam("/object_qz", q.z());
      obj_R = q.toRotationMatrix();
      Eigen::VectorXd info_vec(17);
      info_vec(0) = 2.0;
      info_vec.segment(1, 3) = obj_p;                       // position
      info_vec.segment(4, 3) = obj_R.col(2);                // orientation
      info_vec[7] = 1.0;                                    // if cons ori
      info_vec.segment(8, 3) = Eigen::Vector3d(0.1, 0, 0);  // velocity
      info_vec.segment(11, 3) = Eigen::Vector3d(2, 0, 0);   // if cons vel.
      info_vec.segment(14, 3) = Eigen::Vector3d(0, 0, 0);   // if fix any axis
      inter_info_.push_back(info_vec);
      visualObjectPosition(obj_p);
    }
    else
    {
      inter_info_.push_back(Eigen::VectorXd::Zero(17));
    }
    ROS_INFO("[MANAGE]: Strike mode init!");
  }

  void otherModeInit()
  {
    XmlRpc::XmlRpcValue inter_points_param;
    if (!nh_.getParam("/" + task_ + "/inter_points", inter_points_param))
    {
      ROS_ERROR("[MANAGE]: Failed to get /%s/inter_points from parameter server!",
                task_.c_str());
      return;
    }
    for (int i = 0; i < inter_points_param.size(); ++i)
    {
      XmlRpc::XmlRpcValue &point_array = inter_points_param[i];
      Eigen::VectorXd info_vec(point_array.size());

      for (int j = 0; j < point_array.size(); ++j)
      {
        if (point_array[j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
          info_vec(j) = static_cast<double>(point_array[j]);
        }
        else if (point_array[j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
          info_vec(j) = static_cast<double>(static_cast<int>(point_array[j]));
        }
        else
        {
          ROS_WARN("[MANAGE]: Unexpected type for inter_points[%d][%d]", i, j);
          info_vec(j) = 0.0;
        }
      }
      inter_info_.push_back(info_vec);
    }

    ROS_INFO("[MANAGE]: Other mode init! Loaded %zu inter points.", inter_info_.size());
  }

  /************ Mode Init *************/

  void initModeAndStartAndEnd()
  {
    int mode = 0;
    nh_.getParam("/mode", mode);
    if (mode == 0)
    {
      mode_ = MODE_GRASP;
    }
    else if (mode == 1)
    {
      mode_ = MODE_STRIKE;
    }
    else
    {
      mode_ = MODE_OTHER;
    }

    init_state_.setZero(3, 3);
    fin_state_.setZero(3, 3);
    nh_.param("use_quadrotor_odom", use_external_odom_, false);
    if (mode_ == MODE_GRASP || mode_ == MODE_STRIKE)
    {
      /* start and end initialization */
      if (!use_external_odom_)
      {
        nh_.param("/start_x", init_state_.col(0)(0), -3.0);
        nh_.param("/start_y", init_state_.col(0)(1), 0.0);
        nh_.param("/start_z", init_state_.col(0)(2), 1.2);
        has_quadrotor_odom_ = true;
      }
      nh_.param("/end_x", fin_state_.col(0)(0), 3.0);
      nh_.param("/end_y", fin_state_.col(0)(1), 0.0);
      nh_.param("/end_z", fin_state_.col(0)(2), 1.2);
    }
    else /* mode == MODE_OTHER */
    {
      nh_.getParam("/task", task_);
      std::vector<double> start_pt, end_pt;
      if (!use_external_odom_)
      {
        nh_.getParam("/" + task_ + "/start_pt", start_pt);
        init_state_.col(0) << start_pt[0], start_pt[1], start_pt[2];
        has_quadrotor_odom_ = true;
      }
      nh_.getParam("/" + task_ + "/end_pt", end_pt);
      fin_state_.col(0) << end_pt[0], end_pt[1], end_pt[2];
    }
    ROS_INFO("[MANAGE]: Initilized start and end!");
  }

  void initPreAndPost()
  {
    int num_pre_point, num_post_point;
    nh_.param("/num_pre_point", num_pre_point, 0);
    std::vector<Eigen::VectorXd> tmp_inter_info = inter_info_;
    inter_info_.clear();
    /* pre point */
    if (num_pre_point > 0)
    {
      for (int i = 0; i < num_pre_point; i++)
      {
        Eigen::VectorXd info_vec(4);
        info_vec(0) = 0;
        nh_.getParam("/pre_x_" + std::to_string(i), info_vec(1));
        nh_.getParam("/pre_y_" + std::to_string(i), info_vec(2));
        nh_.getParam("/pre_z_" + std::to_string(i), info_vec(3));
        inter_info_.push_back(info_vec);
        if (info_vec(1) == 0 && info_vec(2) == 0 && info_vec(3) == 0)
        {
          ROS_ERROR("[MANAGE]: Pre point %d cannot be all zeros!", i);
          return;
        }
      }
    }

    inter_info_.insert(inter_info_.end(), tmp_inter_info.begin(), tmp_inter_info.end());

    nh_.param("/num_post_point", num_post_point, 0);
    if (num_post_point > 0)
    {
      for (int i = 0; i < num_post_point; i++)
      {
        Eigen::VectorXd info_vec(4);
        info_vec(0) = 0;
        nh_.getParam("/post_x_" + std::to_string(i), info_vec(1));
        nh_.getParam("/post_y_" + std::to_string(i), info_vec(2));
        nh_.getParam("/post_z_" + std::to_string(i), info_vec(3));
        inter_info_.push_back(info_vec);
        if (info_vec(1) == 0 && info_vec(2) == 0 && info_vec(3) == 0)
        {
          ROS_ERROR("[MANAGE]: Post point %d cannot be all zeros!", i);
          return;
        }
      }
    }
    ROS_INFO("[MANAGE]: Initilized pre and post!");
  }

  void initRos()
  {
    /* odom and trajectory */
    if (use_external_odom_)
    {
      am_odom_sub_ =
          nh_.subscribe("quadrotor_odom", 1, &PlannerManage::amOdomCallback, this);
    }
    object_odom_sub_ =
        nh_.subscribe("object_odom", 1, &PlannerManage::objectOdomCallback, this);
    traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 1);
    traj_arm_pub_ =
        nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory_arm", 1);
    timer_ = nh_.createTimer(ros::Duration(1), &PlannerManage::timer, this);

    /* functional subscribers and publishers */
    start_sub_ =
        nh_.subscribe("/start_trigger", 1, &PlannerManage::startTriggerCallback, this);
    publish_trigger_sub_ = nh_.subscribe("/publish_trigger", 1,
                                         &PlannerManage::publishTriggerCallback, this);
    visual_obj_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/visualization/target", 1);
    finish_simulation_pub_ = nh_.advertise<std_msgs::Float64>("/finish_simulation", 1);
    logger_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/logger", 1);
    ROS_INFO("[MANAGE]: Initilized ros!");
  }

  bool checkInterInfo()
  {
    const int expected_sizes[] = {4, 7, 17, 20};
    for (const auto &info : inter_info_)
    {
      int mode = static_cast<int>(info(0));
      if (info.size() != expected_sizes[mode])
      {
        ROS_ERROR("[MANAGE]: Inter info size error: got %zu, expect %d for mode %d",
                  info.size(), expected_sizes[mode], mode);
        return false;
      }
    }
    return true;
  }

 private:
  ros::NodeHandle nh_;
  Config config_;
  ros::Subscriber start_sub_;
  ros::Subscriber am_odom_sub_;
  ros::Subscriber object_odom_sub_;
  ros::Publisher traj_pub_;
  ros::Publisher traj_arm_pub_;
  ros::Publisher visual_obj_pub_;
  ros::Subscriber publish_trigger_sub_;
  ros::Publisher logger_pub_;
  ros::Timer timer_;
  GlobalPlanner *planner_;
  bool plan_once_ = false;
  bool use_trigger_start_ = USE_TRIGGER_START;
  bool trigger_start_ = false;
  bool use_traj_pub_trigger_ = USE_TRIGGER_PUBLISH;
  bool use_external_odom_ = false;
  bool use_object_odom_ = false;

  bool has_quadrotor_odom_ = false;
  bool has_object_odom_ = false;
  bool traj_ready_ = false;

  Mode mode_;
  std::string task_;

  quadrotor_msgs::PolynomialTrajectory traj_msg_;
  quadrotor_msgs::PolynomialTrajectory traj_arm_msg_;

  Trajectory<5> traj_;
  Trajectory<5> traj_arm_;

  std::vector<Eigen::MatrixXd> state_;
  Eigen::MatrixXd init_state_, fin_state_;
  std::vector<Eigen::VectorXd> inter_info_;

  double object_height_;

  ros::Publisher finish_simulation_pub_;  // Changed from ServiceClient to Publisher
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "se3_node");
  ros::NodeHandle nh("~");
  PlannerManage planner_node(nh);
  ros::spin();
  return 0;
}
