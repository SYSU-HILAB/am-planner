#ifndef SE3_PLANNER_H
#define SE3_PLANNER_H
#include <decomp_basis/data_type.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <geometry_msgs/PoseStamped.h>
#include <jps_planner/jps_planner.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <ros/ros.h>
#include <se3gcopter/se3gcopter.h>
#include <se3gcopter/trajectory.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <traj_opt/config.h>
#include <vis_utils/vis_utils.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <memory>

using namespace std;

class GlobalPlanner
{
 public:
  GlobalPlanner(Config &conf, ros::NodeHandle &nh_, const int id);
  ~GlobalPlanner();

  /* plan configuration */
  Config &config_;
  MatDf init_state_, fin_state_;

  /* ROS utils */
  ros::NodeHandle nh_;
  ros::Publisher traj_pub_;
  ros::Subscriber pcl_sub_;
  ros::Publisher path_pub_;
  vec_Vec3f *obs_pointer_;
  std::shared_ptr<MapUtil<3>> map_util_;
  std::shared_ptr<JPSPlanner3D> jps3d_;

  Visualization *visualization_;
  Visualization *visualization_temp_;
  Trajectory<5> *traj_;
  Trajectory<5> *traj_arm_;

  Trajectory<5> *traj_temp_;
  Trajectory<5> *traj_arm_temp_;

  /* grasp parameters */
  std::vector<Eigen::VectorXd> inter_info_;
  double grasp_time_ = -1.0;

  Mode mode_;
  /* planning */
  bool HasMap() { return has_map_; };

  void Setup(const MatDf &init_state, const MatDf &fin_state, const Mode &mode,
             const std::vector<Eigen::VectorXd> &inter_info);
  bool Plan(double &cost, std::vector<double> &logs);

  void OutputTraj(Trajectory<5> &traj, Trajectory<5> &traj_arm);
  void VisualTraj();

 private:
  /* planning */

  void PathSearching(const Vec3f &start_pt,
                     const std::vector<Eigen::VectorXd> &inter_info, const Vec3f &end_pt,
                     vec_Vec3f &path, std::vector<int> &inter_ids);
  void SfcGenerate(const Mode mode, const vec_Vec3f &path, const vector<int> &inter_ids,
                   const std::vector<Eigen::VectorXd> &inter_info, vector<MatDf> &hPolys,
                   vector<int> &inter_polys_ids);
  double MincoOptimizer(const Mode &mode, const MatDf &init_state,
                        const std::vector<Eigen::VectorXd> &inter_info,
                        const MatDf &fin_state, const vector<int> &inter_polys_ids,
                        const vector<MatDf> &hPolys, SE3GCOPTER &nonlinOpt,
                        Trajectory<5> &traj, Trajectory<5> &traj_arm,
                        Trajectory<5> &traj_temp, Trajectory<5> &traj_arm_temp,
                        std::vector<double> &logs);

 private:
  bool has_map_ = false;
  void PclCallback(const sensor_msgs::PointCloud2 &pointcloud_map);
  template <typename Trajectory>
  void Traj2Msg(Trajectory traj, quadrotor_msgs::PolynomialTrajectory &traj_msg);
  bool JpsSearchPath(const Vec3f &start, const Vec3f &end, const bool &need_inter,
                     const Vec3f &inter, bool use_start_inter, const Vec3f &start_inter,
                     bool use_end_inter, const Vec3f &end_inter, vec_Vec3f &path,
                     int &inter_id);
};

#endif  // SE3_PLANNER_H_
