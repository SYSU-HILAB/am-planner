/**
 * @file jps_planner.h
 * @brief JPSPlanner
 */
#ifndef JPS_PLANNER_BASE_H
#define JPS_PLANNER_BASE_H

#include <data_type/data_type.h>
#include <geometry_msgs/PoseStamped.h>
#include <jps_planner/graph_search.h>
#include <map_util/map_util.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap_server/OctomapServer.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <boost/heap/d_ary_heap.hpp>  // boost::heap::d_ary_heap
#include <iostream>
#include <limits>  // std::numeric_limits
#include <map>
#include <memory>  // std::shared_ptr
#include <string>
#include <unordered_map>  // std::unordered_map
#include <vector>         // std::vector
using namespace std;
class GraphSearch;
/**
 * @brief Abstract base for planning
 */
template <int Dim>
class JPSPlanner
{
 public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug mode
   */
  JPSPlanner(bool verbose = false);

  /// Set map util for collistion checking
  void setMapUtil(const std::shared_ptr<MapUtil<Dim>> &map_util);
  /**
   * @brief Status of the planner
   *
   * 0 --- exit normally;
   * -1 --- no path found;
   * 1, 2 --- start or goal is not free.
   */
  bool hasMap() { return has_map; }
  int status();
  /// Get the modified path
  vec_Vecf<Dim> getPath();
  /// Get the raw path
  vec_Vecf<Dim> getRawPath();
  /// based on the modified path, sample it
  vec_Vecf<Dim> getSamplePath();
  /// remove redundant points on the same line
  vec_Vecf<Dim> removeLinePts(const vec_Vecf<Dim> &path);
  /// Remove some corner waypoints
  vec_Vecf<Dim> removeCornerPts(const vec_Vecf<Dim> &path);
  /// Must be called before run the planning thread
  vec_Vecf<Dim> samplePath(const vec_Vecf<Dim> &path);
  /// discretize the path into grids
  void updateMap();
  /// Planning function
  bool plan(const Vecf<Dim> &start, const Vecf<Dim> &goal, decimal_t eps = 1,
            bool use_jps = false);
  /// Get the nodes in open set
  vec_Vecf<Dim> getOpenSet() const;
  /// Get the nodes in close set
  vec_Vecf<Dim> getCloseSet() const;
  /// Get all the nodes
  vec_Vecf<Dim> getAllSet() const;
  // ros
  void setParam(ros::NodeHandle &nh);
  // map update timer
  ros::Timer map_update_timer;
  void map_update_timer_cb(const ros::TimerEvent &event);
  // vis publisher
  ros::Publisher rawpath_jps_pub_, optipath_jps_pub_, key_grid_jps_pub_,
      sample_path_jps_pub_;
  // visiualization
  void publish_rawpath();
  void publish_optipath();
  void publish_keygrid();
  void publish_samplepath();
  void publishAll();

  // convert from vec_Vec3f to std::vector<Eigen::Vector3d>
  static bool convert_path(const vec_Vec3f &path, std::vector<Eigen::Vector3d> &result);

 protected:
  /// Assume using 3D voxel map for all 2d and 3d planning
  std::shared_ptr<MapUtil<Dim>> map_util_;
  /// The planner
  std::shared_ptr<JPS::GraphSearch> graph_search_;
  /// Raw path from planner
  vec_Vecf<Dim> raw_path_;
  /// Modified path for future usage
  vec_Vecf<Dim> path_;
  /// Sample path
  vec_Vecf<Dim> sample_path_;
  /// Flag indicating the success of planning
  int status_ = 0;
  /// Enabled for printing info
  bool planner_verbose_;
  /// 1-D map array
  std::vector<char> cmap_;
  bool map_util_set = false;
  bool has_map = false;
  string world_frame_id;
};

/// Planner for 2D OccMap
typedef JPSPlanner<2> JPSPlanner2D;

/// Planner for 3D VoxelMap
typedef JPSPlanner<3> JPSPlanner3D;

#endif
