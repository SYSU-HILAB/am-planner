#include <traj_opt/se3_planner.h>

GlobalPlanner::GlobalPlanner(Config &conf, ros::NodeHandle &nh, const int id)
    : config_(conf), nh_(nh)
{
  visualization_ = new Visualization(&conf, nh, id);
  visualization_temp_ = new Visualization(&conf, nh, id + 1);
  pcl_sub_ = nh.subscribe("/global_map", 10, &GlobalPlanner::PclCallback, this,
                          ros::TransportHints().tcpNoDelay());
  traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  path_pub_ = nh_.advertise<visualization_msgs::Marker>("/path", 10);
  map_util_ = std::make_shared<MapUtil<3>>();
  map_util_->setParam(nh_);

  jps3d_ = std::make_shared<JPSPlanner3D>(true);
  jps3d_->setParam(nh_);
  jps3d_->setMapUtil(map_util_);

  traj_ = new Trajectory<5>;
  traj_arm_ = new Trajectory<5>;
  traj_temp_ = new Trajectory<5>;
  traj_arm_temp_ = new Trajectory<5>;

  init_state_.setZero(3, 3);
  fin_state_.setZero(3, 3);
}

bool GlobalPlanner::Plan(double &cost, std::vector<double> &logs)
{
  Vec3f start_pt, end_pt;
  std::vector<int> inter_ids;
  vec_Vec3f path; /* the path from jps */

  if (!has_map_) return false;
  if (!jps3d_->hasMap()) return false;
  start_pt = init_state_.col(0);
  end_pt = fin_state_.col(0);

  /* path searching */
  PathSearching(start_pt, inter_info_, end_pt, path, inter_ids);
  ROS_INFO("[SE3_PLANNER]: inter_ids size: %d", inter_ids.size());

  /* generate the SFC */
  vector<MatDf> hPolys;
  vector<int> inter_polys_ids;
  SfcGenerate(mode_, path, inter_ids, inter_info_, hPolys, inter_polys_ids);

  /* MINCO optimizer */
  SE3GCOPTER gcopter;
  cost =
      MincoOptimizer(mode_, init_state_, inter_info_, fin_state_, inter_polys_ids, hPolys,
                     gcopter, *traj_, *traj_arm_, *traj_temp_, *traj_arm_temp_, logs);
  return true;
}

void GlobalPlanner::PathSearching(const Vec3f &start_pt,
                                  const std::vector<Eigen::VectorXd> &inter_info,
                                  const Vec3f &end_pt, vec_Vec3f &path,
                                  std::vector<int> &inter_ids)
{
  ROS_INFO("[SE3_PLANNER]: Begin path searching!");
  int inter_id;
  bool success;
  vec_Vec3f tmp_path;
  double t1 = ros::Time::now().toSec(), len;
  Vec3f pt1, pt2;
  path.clear();
  inter_ids.clear();

  pt1 = start_pt;
  path.push_back(pt1);
  if (inter_info.size() > 0)
  {
    for (int i = 0; i < inter_info.size(); i++)
    {
      /*  important!!!
        Because the waypoint constraint is set for the quadrotor or end effector.
        However, the end effector is to close to the obstacle like desk.
        So we extend the point along the z axis of body frame to 0.15m
        to avoid the point too close to the obstacle.
      */
      pt2 = inter_info[i].segment(1, 3);
      if (inter_info[i](0) == 2.0)
      {
        if (inter_info[i][7] == 0.0)
          pt2 = pt2 + 0.15 * Vec3f(0, 0, 1);
        else
          pt2 = pt2 + 0.15 * inter_info[i].segment(4, 3);
      }
      if (!JpsSearchPath(pt1, pt2, false, Vec3f::Zero(), false, Vec3f::Zero(), false,
                         Vec3f::Zero(), tmp_path, inter_id))
      {
        ROS_ERROR("\033[31m[SE3_PLANNER]: Path search failed!\033[0m");
        assert(0);
      }
      path.insert(path.end(), tmp_path.begin() + 1, tmp_path.end());
      len = path.size();
      inter_ids.push_back(len - 1);
      pt1 = pt2;
    }
  }

  pt2 = end_pt;
  if (!JpsSearchPath(pt1, pt2, false, Vec3f::Zero(), false, Vec3f::Zero(), false,
                     Vec3f::Zero(), tmp_path, inter_id))
  {
    ROS_ERROR("\033[31m[SE3_PLANNER]: Path search failed!\033[0m");
    assert(0);
  }
  path.insert(path.end(), tmp_path.begin() + 1, tmp_path.end());
  ROS_INFO("[SE3_PLANNER]: JPS search path!");
  double t2 = ros::Time::now().toSec();
  ROS_INFO("\033[32m[SE3_PLANNER]: JPS path searching time: %.3f s\033[0m", t2 - t1);

  // for (auto &pt : path)
  // {
  //   ROS_INFO("Waypoint: x=%.3f, y=%.3f, z=%.3f", pt(0), pt(1), pt(2));
  // }
  // Visualize the path
  visualization_->visualizePath(path);
}

void GlobalPlanner::SfcGenerate(const Mode mode, const vec_Vec3f &path,
                                const vector<int> &inter_ids,
                                const std::vector<Eigen::VectorXd> &inter_info,
                                vector<MatDf> &hPolys, vector<int> &inter_polys_ids)
{
  /* active generation */
  EllipsoidDecomp3D decomp_util;
  Vec3f local_box(config_.polyhedronBox(0), config_.polyhedronBox(1),
                  config_.polyhedronBox(2));
  vec_E<Polyhedron3D> decompPolys;
  vec_E<Ellipsoid3D> ellips;
  vec_E<Polyhedron3D> pre_polys, lat_polys;
  vec_E<Ellipsoid3D> pre_ellips, lat_ellips;
  Vec3f inter_pt;
  int overlap, path_size = path.size();
  Vec3f pre_pt, lat_pt, previous_pt_avg, later_pt_avg, pre_dir, inter_dir, later_dir,
      pre_plane_normal, later_plane_normal;
  int poly_count;
  int inter_id;
  bool reach, once;
  vec_Vec3f tmp_line;
  decomp_util.set_local_bbox(local_box);
  decomp_util.set_obs(*obs_pointer_);
  inter_polys_ids.clear();
  pre_polys.clear();
  lat_polys.clear();
  pre_ellips.clear();
  lat_ellips.clear();
  nh_.getParam("overlap", overlap);

  for (int i = 0; i < inter_ids.size(); i++)
  {
    /* generate the inter polyhedron */
    inter_id = inter_ids[i];
    inter_pt = path[inter_id];
    previous_pt_avg = Vec3f::Zero();
    later_pt_avg = Vec3f::Zero();
    /* get the average of the overlap points */
    if (inter_id - overlap < 0 || inter_id + overlap >= path_size)
    {
      ROS_ERROR("\033[31m[SE3_PLANNER]: The overlap is too large!\033[0m");
      assert(0);
      return;
    }
    else
    {
      std::cout << "overlap = " << overlap << std::endl;
    }
    for (int q = 0; q < overlap; q++)
    {
      previous_pt_avg += path[inter_id - q];
      later_pt_avg += path[inter_id + q];
    }
    previous_pt_avg /= overlap;
    later_pt_avg /= overlap;
    if (inter_info[i](0) == 2.0)
    {
      Eigen::Vector3d zd;
      if (inter_info[i][7] == 0.0)
      {
        zd = Vec3f(0, 0, 1);
      }
      else
      {
        zd = inter_info[i].segment(4, 3);
      }
      inter_dir = (previous_pt_avg - inter_pt).normalized();
      pre_plane_normal = zd.cross(inter_dir);
      pre_dir = pre_plane_normal.cross(zd);
      later_plane_normal = zd.cross(later_pt_avg - inter_pt);
      later_dir = later_plane_normal.cross(zd);
      if (pre_dir.norm() == 0) pre_dir = zd;
      if (later_dir.norm() == 0) later_dir = zd;
      pre_pt = inter_pt + pre_dir * config_.platformR;
      lat_pt = inter_pt + later_dir * config_.platformR;
    }
    else
    {
      pre_pt = previous_pt_avg;
      lat_pt = later_pt_avg;
    }

    if (config_.platformR > 4)
    {
      ROS_ERROR("[SE3_PLANNER]: The platform radius is too large!");
      return;
    }

    tmp_line.clear();
    tmp_line.push_back(pre_pt);
    tmp_line.push_back(inter_pt);
    decomp_util.dilate(tmp_line);

    pre_polys.push_back(decomp_util.get_polyhedrons()[0]);
    pre_ellips.push_back(decomp_util.get_ellipsoids()[0]);

    tmp_line.clear();
    tmp_line.push_back(lat_pt);
    tmp_line.push_back(inter_pt);
    decomp_util.dilate(tmp_line);
    lat_polys.push_back(decomp_util.get_polyhedrons()[0]);
    lat_ellips.push_back(decomp_util.get_ellipsoids()[0]);
  }

  reach = false; /* reach the active generated polyhedron */
  bool finished = false;
  if (inter_ids.size() == 0)
  {
    finished = true;
  }
  poly_count = 0; /* count the number of the polyhedron */
  int reach_i_th_inter_pt = 0;

  for (int i = 0; i < path_size - 1;)
  {
    /* find the farest unblocked point */
    int k;
    for (k = i + 1; k < path_size; k++)
    {
      if (map_util_->isBlocked(path[i], path[k]) ||
          ((path[i] - path[k]).norm() >= config_.maxLengthLine))
      {
        k--;
        break;
      }

      if (!finished)
      {
        if (pre_polys[reach_i_th_inter_pt].inside(path[k]))
        {
          reach = true;
          break;
        }
      }
    }
    if (k < i + 1)
    {
      k = i + 1;
    }
    if (k >= path_size)
    {
      k = path_size - 1;
    }
    vec_Vec3f line;
    line.push_back(path[i]);
    line.push_back(path[k]);
    decomp_util.dilate(line);
    Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
    Ellipsoid3D ellip = decomp_util.get_ellipsoids()[0];
    poly_count++;
    decompPolys.push_back(poly);
    ellips.push_back(ellip);
    if (reach)
    {
      decompPolys.push_back(pre_polys[reach_i_th_inter_pt]);
      decompPolys.push_back(lat_polys[reach_i_th_inter_pt]);
      ellips.push_back(pre_ellips[reach_i_th_inter_pt]);
      ellips.push_back(lat_ellips[reach_i_th_inter_pt]);
      poly_count += 2;
      ROS_INFO("[SE3_PLANNER]: The inter point is in the polyhedron %d and %d !",
               poly_count - 2, poly_count - 1);
      inter_polys_ids.push_back(poly_count - 2);
      k = inter_ids[reach_i_th_inter_pt] + overlap;
      if (k >= path_size)
      {
        k = path_size - 1;
      }
      poly = lat_polys[reach_i_th_inter_pt];
      reach = false;
      reach_i_th_inter_pt++;
      if (reach_i_th_inter_pt == inter_ids.size())
      {
        finished = true;
      }
    }

    // find the nearest one to the boundry of poly.
    int j;
    for (j = k; j < path_size; j++)
    {
      Vec3f pt;
      pt[0] = path[j][0];
      pt[1] = path[j][1];
      pt[2] = path[j][2];
      if (!poly.inside(pt)) break;
      if (!finished && pre_polys[reach_i_th_inter_pt].inside(pt)) break;
    }
    j--;
    if (j >= path_size - 1)
    {
      break;
    }
    int wp;
    wp = std::max(i + 1, static_cast<int>(round((1 * i + 4 * j) / 5)));
    i = wp;
  }
  for (size_t i = 0; i < decompPolys.size(); i++)
  {
    decompPolys[i].add(
        Hyperplane3D(Vec3f(0.0, 0.0, config_.mapHeight), Vec3f(0.0, 0.0, 1.0)));
    decompPolys[i].add(Hyperplane3D(Vec3f(0.0, 0.0, 0.5), Vec3f(0.0, 0.0, -1.0)));
  }
  visualization_->visualizePolyH(decompPolys);

  MatDf current_poly;
  for (uint i = 0; i < decompPolys.size(); i++)
  {
    vec_E<Hyperplane3D> current_hyperplanes = decompPolys[i].hyperplanes();
    current_poly.resize(6, current_hyperplanes.size());
    for (uint j = 0; j < current_hyperplanes.size(); j++)
    {
      current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
    }
    hPolys.push_back(current_poly);
  }
}

double GlobalPlanner::MincoOptimizer(
    const Mode &mode, const MatDf &init_state,
    const std::vector<Eigen::VectorXd> &inter_info, const MatDf &fin_state,
    const vector<int> &inter_polys_ids, const vector<MatDf> &hPolys,
    SE3GCOPTER &nonlinOpt, Trajectory<5> &traj, Trajectory<5> &traj_arm,
    Trajectory<5> &traj_temp, Trajectory<5> &traj_arm_temp, std::vector<double> &logs)
{
  std::cout << "\033[33mBegin to optimize the trajectory with MINCO...\033[0m"
            << std::endl;
  std::chrono::high_resolution_clock::time_point tic, toc;
  tic = std::chrono::high_resolution_clock::now();

  /* setup the MINCO optimizer */
  /* cf. se3gcopter.h */
  if (!nonlinOpt.setup(mode, init_state, inter_info, fin_state, inter_polys_ids, hPolys,
                       INFINITY, config_, nh_))
  {
    std::cout << "Failed to setup the MINCO!" << std::endl;
    return 1e20;
  }
  else
  {
    std::cout << "\033[1;32m[SE3_PLANNER]: MINCO setup successfully!\033[0m" << std::endl;
  }
  double finalObj, compTime;
  MatDf iP, recPosMat;

  /* optimize the trajectory */
  finalObj =
      nonlinOpt.optimize(traj, traj_arm, traj_temp, traj_arm_temp, iP, grasp_time_, logs);
  toc = std::chrono::high_resolution_clock::now();
  /* visualization the recorded points */
  compTime =
      std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;
  std::cout << "\033[32mFinish optimization!\033[0m" << std::endl;
  std::cout << "Optimization time usage: " << compTime << " ms" << std::endl;
  std::cout << "Final cost: " << finalObj << std::endl;
  std::cout << "Total Duration: " << traj.getTotalDuration() << std::endl;
  std::cout << "Maximum Vel: " << traj.getMaxVelRate() << std::endl;
  std::cout << "Maximum Acc: " << traj.getMaxAccRate() << std::endl;
  std::cout << "Arm Maximum Vel: " << traj_arm.getMaxVelRate() << std::endl;
  // get max position and min position
  {
    double total_time = traj_arm.getTotalDuration();
    double max_pos_z = -1000, min_pos_z = 1000;
    double max_pos_x = -1000, min_pos_x = 1000;
    double max_pos_y = -1000, min_pos_y = 1000;
    double grasp_T = 0;
    for (double t = 0; t < total_time; t += 0.01)
    {
      Vec3f pos = traj_arm.getPos(t);
      if (pos(2) > max_pos_z) max_pos_z = pos(2);
      if (pos(2) < min_pos_z) min_pos_z = pos(2);
      if (pos(0) > max_pos_x) max_pos_x = pos(0);
      if (pos(0) < min_pos_x) min_pos_x = pos(0);
      if (pos(1) > max_pos_y) max_pos_y = pos(1);
      if (pos(1) < min_pos_y) min_pos_y = pos(1);
    }

    std::cout << "Max Z: " << std::fixed << std::setprecision(3) << max_pos_z
              << ", Min Z: " << min_pos_z << std::endl;
    std::cout << "Max X: " << std::fixed << std::setprecision(3) << max_pos_x
              << ", Min X: " << min_pos_x << std::endl;
    std::cout << "Max Y: " << std::fixed << std::setprecision(3) << max_pos_y
              << ", Min Y: " << min_pos_y << std::endl;
  }
  return finalObj;
}

void GlobalPlanner::Setup(const MatDf &init_state, const MatDf &fin_state,
                          const Mode &mode,
                          const std::vector<Eigen::VectorXd> &inter_info)
{
  /* start and end */
  init_state_ = init_state;
  fin_state_ = fin_state;
  mode_ = mode;
  inter_info_ = inter_info;

  /* visualization guide points */
  if (mode_ == MODE_GRASP || mode_ == MODE_STRIKE)
  {
    std::vector<Vec3f> guide_pts;
    for (int i = 0; i < inter_info_.size(); i++)
    {
      if (inter_info_[i](0) == 0)
      {
        guide_pts.push_back(inter_info_[i].segment(1, 3));
      }
    }
    MatDf guide_pts_mat(3, guide_pts.size());
    for (int i = 0; i < guide_pts.size(); i++)
    {
      guide_pts_mat.col(i) = guide_pts[i];
    }
    if (guide_pts_mat.cols() > 0)
      visualization_->visualizeGuidePoints(guide_pts_mat);
    else
      ROS_WARN("[SE3_PLANNER]: No setting guide points !");
  }
}

void GlobalPlanner::OutputTraj(Trajectory<5> &traj, Trajectory<5> &traj_arm)
{
  traj = *traj_;
  traj_arm = *traj_arm_;
};

void GlobalPlanner::VisualTraj()
{
  /* visual the first N segments */
  visualization_->visualizeEllipsoid(*traj_, *traj_arm_, 2000, grasp_time_);
  visualization_temp_->visualizeEllipsoid(*traj_temp_, *traj_arm_temp_, 2000, -1);
  visualization_->visualizeSpheres(*traj_, *traj_arm_, 11);
}

bool GlobalPlanner::JpsSearchPath(const Vec3f &start, const Vec3f &end,
                                  const bool &need_inter, const Vec3f &inter,
                                  bool use_start_inter, const Vec3f &start_inter,
                                  bool use_end_inter, const Vec3f &end_inter,
                                  vec_Vec3f &path, int &inter_id)
{
  vec_Vec3f path_segment;
  int status;
  path.clear();

  auto search_segment = [&](const Vec3f &s_pos, const Vec3f &e_pos) -> bool
  {
    ROS_INFO(
        "[SE3_PLANNER]: Searching path from (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)",
        s_pos(0), s_pos(1), s_pos(2), e_pos(0), e_pos(1), e_pos(2));
    bool success = jps3d_->plan(s_pos, e_pos);
    if (success)
    {
      vec_Vec3f temp_path;
      temp_path = jps3d_->getSamplePath();
      path.insert(path.end(), temp_path.begin() + (path.empty() ? 0 : 1),
                  temp_path.end());
      return true;
    }
    return false;
  };

  // Start to inter
  if (need_inter)
  {
    if (use_start_inter)
    {
      if (!search_segment(start, start_inter) || !search_segment(start_inter, inter))
      {
        return false;
      }
    }
    else
    {
      if (!search_segment(start, inter))
      {
        return false;
      }
    }

    inter_id = path.size() - 1;  // Save the index of inter point

    // inter to end
    if (use_end_inter)
    {
      if (!search_segment(inter, end_inter) || !search_segment(end_inter, end))
      {
        return false;
      }
    }
    else
    {
      if (!search_segment(inter, end))
      {
        return false;
      }
    }
  }
  else
  {
    inter_id = -1;
    if (use_start_inter)
    {
      if (!search_segment(start, start_inter) || !search_segment(start_inter, end))
      {
        return false;
      }
    }
    else
    {
      if (!search_segment(start, end))
      {
        return false;
      }
    }
  }
  return true;
}

void GlobalPlanner::PclCallback(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  if (has_map_) return;
  ROS_INFO("[SE3_PLANNER]: please wait~");
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(pointcloud_map, cloud);
  cloud.header.frame_id = config_.odomFrame;
  obs_pointer_ = new vec_Vec3f();
  *obs_pointer_ = DecompROS::cloud_to_vec(cloud);
  has_map_ = true;
  ROS_WARN("POINT BE BUILD!");
}
template <typename Trajectory>
void GlobalPlanner::Traj2Msg(Trajectory traj,
                             quadrotor_msgs::PolynomialTrajectory &traj_msg)
{
  static int count = 0;
  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = config_.odomFrame;
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
  traj_msg.num_segment = traj.getPieceNum();
  traj_msg.start_yaw = 0;
  traj_msg.final_yaw = 0;
  for (unsigned int i = 0; i < traj_msg.num_segment; i++)
  {
    for (unsigned int j = 0; j <= static_cast<int>(traj[i].getOrder()); j++)
    {
      CoefficientMat<5> coemat = traj[i].normalizePosCoeffMat();
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
