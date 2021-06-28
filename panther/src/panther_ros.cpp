/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "panther_ros.hpp"

#include <nav_msgs/Path.h>

#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>  //For DecompROS::polyhedron_array_to_ros
#include <decomp_geometry/polyhedron.h>       //For hyperplane
#include <Eigen/Geometry>

#include <panther_msgs/Log.h>

// #include <assert.h> /* assert */

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

typedef PANTHER_timers::Timer MyTimer;

// this object is created in the panther_ros_node
PantherRos::PantherRos(ros::NodeHandle nh1, ros::NodeHandle nh2, ros::NodeHandle nh3) : nh1_(nh1), nh2_(nh2), nh3_(nh3)
{
  name_drone_ = ros::this_node::getNamespace();  // This returns also the slashes (2 in Kinetic, 1 in Melodic)
  name_drone_.erase(std::remove(name_drone_.begin(), name_drone_.end(), '/'), name_drone_.end());  // Remove the slashes

  std::string id = name_drone_;
  id.erase(0, 2);  // Erase SQ or HX i.e. SQ12 --> 12  HX8621 --> 8621 # TODO Hard-coded for this this convention
  id_ = std::stoi(id);

  // wait for body transform to be published before initializing.  This has to be done before creating a new PANTHER
  // object
  name_camera_depth_optical_frame_tf_ = name_drone_ + "/camera_depth_optical_frame";

  while (true)
  {
    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener{ new tf2_ros::TransformListener(tf_buffer) };
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer.lookupTransform(name_drone_, name_camera_depth_optical_frame_tf_, ros::Time(0),
                                                    ros::Duration(0.5));  // Note that ros::Time(0) will just get us the
                                                                          // latest available transform.
      par_.b_T_c = tf2::transformToEigen(transform_stamped);
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "Trying to find transform %s --> %s", name_drone_.c_str(),
                        name_camera_depth_optical_frame_tf_.c_str());
    }
  }
  ROS_INFO("Found transform %s --> %s", name_drone_.c_str(), name_camera_depth_optical_frame_tf_.c_str());
  // std::cout << "par_.b_T_c.matrix()= " << par_.b_T_c.matrix() << std::endl;

  safeGetParam(nh1_, "use_ff", par_.use_ff);
  safeGetParam(nh1_, "visual", par_.visual);
  safeGetParam(nh1_, "color_type", par_.color_type);
  safeGetParam(nh1_, "n_agents", par_.n_agents);

  safeGetParam(nh1_, "dc", par_.dc);
  safeGetParam(nh1_, "goal_radius", par_.goal_radius);
  safeGetParam(nh1_, "drone_radius", par_.drone_radius);

  safeGetParam(nh1_, "Ra", par_.Ra);

  safeGetParam(nh1_, "ydot_max", par_.ydot_max);

  safeGetParam(nh1_, "x_min", par_.x_min);
  safeGetParam(nh1_, "x_max", par_.x_max);

  safeGetParam(nh1_, "y_min", par_.y_min);
  safeGetParam(nh1_, "y_max", par_.y_max);

  safeGetParam(nh1_, "z_min", par_.z_min);
  safeGetParam(nh1_, "z_max", par_.z_max);

  std::vector<double> v_max_tmp;
  std::vector<double> a_max_tmp;
  std::vector<double> j_max_tmp;

  safeGetParam(nh1_, "v_max", v_max_tmp);
  safeGetParam(nh1_, "a_max", a_max_tmp);
  safeGetParam(nh1_, "j_max", j_max_tmp);

  par_.v_max << v_max_tmp[0], v_max_tmp[1], v_max_tmp[2];
  par_.a_max << a_max_tmp[0], a_max_tmp[1], a_max_tmp[2];
  par_.j_max << j_max_tmp[0], j_max_tmp[1], j_max_tmp[2];

  safeGetParam(nh1_, "factor_alloc", par_.factor_alloc);

  safeGetParam(nh1_, "factor_alpha", par_.factor_alpha);

  safeGetParam(nh1_, "num_seg", par_.num_seg);
  safeGetParam(nh1_, "deg_pos", par_.deg_pos);
  safeGetParam(nh1_, "deg_yaw", par_.deg_yaw);
  safeGetParam(nh1_, "num_max_of_obst", par_.num_max_of_obst);
  safeGetParam(nh1_, "num_samples_simpson", par_.num_samples_simpson);

  safeGetParam(nh1_, "upper_bound_runtime_snlopt", par_.upper_bound_runtime_snlopt);
  safeGetParam(nh1_, "lower_bound_runtime_snlopt", par_.lower_bound_runtime_snlopt);
  safeGetParam(nh1_, "kappa", par_.kappa);
  safeGetParam(nh1_, "mu", par_.mu);

  safeGetParam(nh1_, "max_seconds_keeping_traj", par_.max_seconds_keeping_traj);

  safeGetParam(nh1_, "a_star_samp_x", par_.a_star_samp_x);
  safeGetParam(nh1_, "a_star_samp_y", par_.a_star_samp_y);
  safeGetParam(nh1_, "a_star_samp_z", par_.a_star_samp_z);
  safeGetParam(nh1_, "a_star_fraction_voxel_size", par_.a_star_fraction_voxel_size);
  safeGetParam(nh1_, "allow_infeasible_guess", par_.allow_infeasible_guess);

  safeGetParam(nh1_, "a_star_bias", par_.a_star_bias);

  safeGetParam(nh1_, "basis", par_.basis);

  safeGetParam(nh1_, "mode", par_.mode);

  safeGetParam(nh1_, "angle_deg_focus_front", par_.angle_deg_focus_front);

  safeGetParam(nh1_, "res_plot_traj", par_.res_plot_traj);

  safeGetParam(nh1_, "norminv_prob", par_.norminv_prob);
  safeGetParam(nh1_, "gamma", par_.gamma);

  safeGetParam(nh1_, "alpha_shrink", par_.alpha_shrink);

  safeGetParam(nh1_, "fov_x_deg", par_.fov_x_deg);
  safeGetParam(nh1_, "fov_y_deg", par_.fov_y_deg);
  safeGetParam(nh1_, "fov_depth", par_.fov_depth);

  safeGetParam(nh1_, "c_smooth_yaw_search", par_.c_smooth_yaw_search);
  safeGetParam(nh1_, "c_visibility_yaw_search", par_.c_visibility_yaw_search);
  // safeGetParam(nh1_, "num_of_layers", par_.num_of_layers); //This one is the same as num_samples_simpson
  safeGetParam(nh1_, "num_of_yaw_per_layer", par_.num_of_yaw_per_layer);

  safeGetParam(nh1_, "c_pos_smooth", par_.c_pos_smooth);
  safeGetParam(nh1_, "c_yaw_smooth", par_.c_yaw_smooth);
  safeGetParam(nh1_, "c_fov", par_.c_fov);
  safeGetParam(nh1_, "c_final_pos", par_.c_final_pos);
  safeGetParam(nh1_, "c_final_yaw", par_.c_final_yaw);
  safeGetParam(nh1_, "print_graph_yaw_info", par_.print_graph_yaw_info);

  bool perfect_prediction;  // use_ground_truth_prediction
  safeGetParam(nh1_, "perfect_prediction", perfect_prediction);

  // safeGetParam(nh1_, "distance_to_force_final_pos", par_.distance_to_force_final_pos);
  // safeGetParam(nh1_, "factor_alloc_when_forcing_final_pos", par_.factor_alloc_when_forcing_final_pos);
  // par_.force_final_pos = false;

  if ((par_.basis != "B_SPLINE" || par_.basis != "BEZIER" || par_.basis != "MINVO") == false)
  {
    std::cout << red << bold << "Basis " << par_.basis << " not implemented yet, aborting" << reset << std::endl;
    abort();
  }
  else
  {
    std::cout << bold << green << "Basis chosen: " << par_.basis << reset << std::endl;
  }

  // CHECK parameters
  std::cout << bold << "Parameters obtained, checking them..." << reset << std::endl;

  verify((par_.c_smooth_yaw_search >= 0), "par_.c_smooth_yaw_search>=0 must hold");
  verify((par_.c_visibility_yaw_search >= 0), "par_.c_visibility_yaw_search>=0 must hold");
  verify((par_.num_of_yaw_per_layer >= 1), "par_.num_of_yaw_per_layer>=1 must hold");

  verify((par_.c_pos_smooth >= 0), "par_.c_pos_smooth>=0 must hold");
  verify((par_.c_yaw_smooth >= 0), "par_.c_yaw_smooth>=0 must hold");
  verify((par_.c_fov >= 0), "par_.c_fov>=0 must hold");
  verify((par_.c_final_pos >= 0), "par_.c_final_pos>=0 must hold");
  verify((par_.c_final_yaw >= 0), "par_.c_final_yaw>=0 must hold");

  verify((par_.ydot_max >= 0), "ydot_max>=0 must hold");
  verify((par_.gamma >= 0), "par_.gamma >= 0 must hold");
  // verify((par_.beta < 0 || par_.alpha < 0), " ");
  // verify((par_.a_max.z() <= 9.81), "par_.a_max.z() >= 9.81, the drone will flip");
  verify((par_.factor_alloc >= 1.0), "Needed: factor_alloc>=1");
  verify((par_.kappa >= 0 && par_.mu >= 0), "Needed: kappa and mu >= 0");
  verify(((par_.kappa + par_.mu) <= 1), "Needed: (par_.kappa + par_.mu) <= 1");
  verify((par_.a_star_fraction_voxel_size >= 0.0 && par_.a_star_fraction_voxel_size <= 1.0), "a_star_fraction_voxel_"
                                                                                             "size is not in [0,1] ");
  verify((par_.deg_pos == 3), "PANTHER needs deg_pos==3");
  verify((par_.deg_yaw == 2), "PANTHER needs deg_yaw==2");
  verify((par_.num_max_of_obst >= 0), "num_max_of_obst>=0 must hold");
  verify((par_.num_seg >= 1), "num_seg>=1 must hold");

  verify((par_.fov_x_deg >= 0), "fov_x_deg>=0 must hold");
  verify((par_.fov_y_deg >= 0), "fov_y_deg>=0 must hold");

  verify((par_.fov_y_deg == par_.fov_x_deg), "par_.fov_y_deg == par_.fov_x_deg must hold");

  std::cout << bold << "Parameters checked" << reset << std::endl;

  panther_ptr_ = std::unique_ptr<Panther>(new Panther(par_));

  // Publishers
  pub_goal_ = nh1_.advertise<snapstack_msgs::Goal>("goal", 1);
  pub_setpoint_ = nh1_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_point_G_ = nh1_.advertise<geometry_msgs::PointStamped>("point_G", 1);
  pub_point_G_term_ = nh1_.advertise<geometry_msgs::PointStamped>("point_G_term", 1);
  pub_point_A_ = nh1_.advertise<visualization_msgs::Marker>("point_A", 1);
  pub_actual_traj_ = nh1_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  poly_safe_pub_ = nh1_.advertise<decomp_ros_msgs::PolyhedronArray>("polys", 1, true);
  pub_traj_safe_colored_ = nh1_.advertise<visualization_msgs::MarkerArray>("traj_obtained", 1);
  pub_traj_ = nh1_.advertise<panther_msgs::DynTraj>("/trajs", 1, true);  // The last boolean is latched or not
  pub_fov_ = nh1_.advertise<visualization_msgs::Marker>("fov", 1);
  pub_obstacles_ = nh1_.advertise<visualization_msgs::Marker>("obstacles", 1);
  pub_log_ = nh1_.advertise<panther_msgs::Log>("log", 1);

  // Subscribers
  sub_term_goal_ = nh1_.subscribe("term_goal", 1, &PantherRos::terminalGoalCB, this);
  sub_whoplans_ = nh1_.subscribe("who_plans", 1, &PantherRos::whoPlansCB, this);
  sub_state_ = nh1_.subscribe("state", 1, &PantherRos::stateCB, this);

  ///
  if (perfect_prediction == false)
  {
    ROS_INFO("NOT using ground truth trajectories (subscribed to trajs_predicted)");
    sub_traj_ = nh1_.subscribe("trajs_predicted", 20, &PantherRos::trajCB, this);  // number is queue size

    // obstacles --> topic trajs_predicted
    // agents -->  //Not implemented yet  (solve the common frame problem)
  }
  else
  {
    ROS_INFO("Using ground truth trajectories (subscribed to /trajs)");

    sub_traj_ = nh1_.subscribe("/trajs", 20, &PantherRos::trajCB, this);  // The number is the queue size
    // obstacles --> topic /trajs
    // agents --> topic /trajs
    // Everything in the same world frame
  }
  //

  // Timers
  pubCBTimer_ = nh2_.createTimer(ros::Duration(par_.dc), &PantherRos::pubCB, this);
  replanCBTimer_ = nh3_.createTimer(ros::Duration(par_.dc), &PantherRos::replanCB, this);

  // For now stop all these subscribers/timers until we receive GO
  sub_state_.shutdown();
  sub_term_goal_.shutdown();
  pubCBTimer_.stop();
  replanCBTimer_.stop();

  // Rviz_Visual_Tools
  visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
  visual_tools_->loadMarkerPub();  // create publisher before waiting
  ros::Duration(0.5).sleep();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // Markers
  setpoint_ = getMarkerSphere(0.35, ORANGE_TRANS);
  E_ = getMarkerSphere(0.35, RED_NORMAL);
  A_ = getMarkerSphere(0.35, RED_NORMAL);

  timer_stop_.reset();

  clearMarkerActualTraj();

  bool gui_mission;
  safeGetParam(nh1_, "gui_mission", gui_mission);

  std::cout << yellow << bold << "gui_mission= " << gui_mission << reset << std::endl;
  std::cout << yellow << bold << "perfect_prediction= " << perfect_prediction << reset << std::endl;
  std::cout << yellow << bold << "mode= " << par_.mode << reset << std::endl;
  // To avoid having to click on the GUI
  if (gui_mission == false)
  {
    panther_msgs::WhoPlans tmp;
    tmp.value = panther_msgs::WhoPlans::PANTHER;
    whoPlansCB(tmp);
  }
  constructFOVMarker();  // only needed once

  ROS_INFO("Planner initialized");
}

PantherRos::~PantherRos()
{
  sub_state_.shutdown();
  sub_term_goal_.shutdown();
  pubCBTimer_.stop();
  replanCBTimer_.stop();
}

void PantherRos::pubObstacles(mt::Edges edges_obstacles)
{
  if (edges_obstacles.size() > 0)
  {
    pub_obstacles_.publish(edges2Marker(edges_obstacles, color(RED_NORMAL)));
  }

  return;
}

void PantherRos::trajCB(const panther_msgs::DynTraj& msg)
{
  if (msg.id == id_)
  {  // This is my own trajectory
    return;
  }

  Eigen::Vector3d W_pos(msg.pos.x, msg.pos.y, msg.pos.z);  // position in world frame
  double dist = (state_.pos - W_pos).norm();

  bool can_use_its_info = (dist <= 4 * par_.Ra);  // See explanation of 4*Ra in Panther::updateTrajObstacles

  // if (can_use_its_info == false)
  // {
  //   return;
  // } //TODO: Commented (4-Feb-2021)

  mt::dynTraj tmp;
  tmp.use_pwp_field = msg.use_pwp_field;

  if (msg.use_pwp_field)
  {
    tmp.pwp_mean = pwpMsg2Pwp(msg.pwp_mean);
    tmp.pwp_var = pwpMsg2Pwp(msg.pwp_var);
  }
  else
  {
    tmp.s_mean.push_back(msg.s_mean[0]);
    tmp.s_mean.push_back(msg.s_mean[1]);
    tmp.s_mean.push_back(msg.s_mean[2]);

    tmp.s_var.push_back(msg.s_var[0]);
    tmp.s_var.push_back(msg.s_var[1]);
    tmp.s_var.push_back(msg.s_var[2]);
  }

  tmp.bbox << msg.bbox[0], msg.bbox[1], msg.bbox[2];
  tmp.id = msg.id;
  tmp.is_agent = msg.is_agent;
  tmp.time_received = ros::Time::now().toSec();

  panther_ptr_->updateTrajObstacles(tmp);
}

// This trajectory contains all the future trajectory (current_pos --> A --> final_point_of_traj), because it's the
// composition of pwp
void PantherRos::publishOwnTraj(const mt::PieceWisePol& pwp)
{
  panther_msgs::DynTraj msg;
  msg.use_pwp_field = true;
  msg.pwp_mean = pwp2PwpMsg(pwp);

  mt::state tmp;
  tmp.setZero();
  msg.pwp_var = pwp2PwpMsg(createPwpFromStaticPosition(tmp));  // zero variance

  // msg.function = s;
  msg.bbox.push_back(2 * par_.drone_radius);
  msg.bbox.push_back(2 * par_.drone_radius);
  msg.bbox.push_back(2 * par_.drone_radius);
  msg.pos.x = state_.pos.x();
  msg.pos.y = state_.pos.y();
  msg.pos.z = state_.pos.z();
  msg.id = id_;

  msg.is_agent = true;
  pub_traj_.publish(msg);
}

void PantherRos::replanCB(const ros::TimerEvent& e)
{
  if (ros::ok() && published_initial_position_ == true)
  {
    mt::Edges edges_obstacles;
    std::vector<mt::state> X_safe;

    std::vector<Hyperplane3D> planes;
    mt::PieceWisePol pwp;
    mt::log log;

    bool replanned =
        panther_ptr_->replan(edges_obstacles, X_safe, planes, num_of_LPs_run_, num_of_QCQPs_run_, pwp, log);

    if (log.drone_status != DroneStatus::GOAL_REACHED)  // log.replanning_was_needed
    {
      pub_log_.publish(log2LogMsg(log));
    }

    if (par_.visual)
    {
      // Delete markers to publish stuff
      visual_tools_->deleteAllMarkers();
      visual_tools_->enableBatchPublishing();

      pubObstacles(edges_obstacles);
      pubTraj(X_safe);
      publishPlanes(planes);
    }

    if (replanned)
    {
      publishOwnTraj(pwp);
      pwp_last_ = pwp;
    }
    else
    {
      int time_ms = int(ros::Time::now().toSec() * 1000);

      if (timer_stop_.elapsedSoFarMs() > 500.0)  // publish every half a second. TODO set as param
      {
        publishOwnTraj(pwp_last_);  // This is needed because is drone DRONE1 stops, it needs to keep publishing his
                                    // last planned trajectory, so that other drones can avoid it (even if DRONE1 was
                                    // very far from the other drones with it last successfully planned a trajectory).
                                    // Note that these trajectories are time-indexed, and the last position is taken if
                                    // t>times.back(). See eval() function in the pwp struct
        timer_stop_.reset();
      }
    }

    // std::cout << "[Callback] Leaving replanCB" << std::endl;
  }
}

void PantherRos::publishPlanes(std::vector<Hyperplane3D>& planes)
{
  auto color = visual_tools_->getRandColor();

  int i = 0;
  for (auto plane_i : planes)
  {
    if ((i % par_.num_seg) == 0)  // planes for a new obstacle --> new color
    {
      color = visual_tools_->getRandColor();  // rviz_visual_tools::TRANSLUCENT_LIGHT;
    }
    Eigen::Isometry3d pose;
    pose.translation() = plane_i.p_;

    // Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
    Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, plane_i.n_);
    pose.linear() = q.toRotationMatrix();

    double height = 0.001;  // very thin
    double x_width = 2;     // very thin
    double y_width = 2;     // very thin
    visual_tools_->publishCuboid(pose, x_width, y_width, height, color);
    i++;

    /*    double d_i = -plane_i.n_.dot(plane_i.p_);
        std::cout << bold << "Publishing plane, d_i= " << d_i << reset << std::endl;
        visual_tools_->publishABCDPlane(plane_i.n_.x(), plane_i.n_.y(), plane_i.n_.z(), d_i, rvt::MAGENTA, 2, 2);*/
  }
  visual_tools_->trigger();
}

void PantherRos::publishPoly(const vec_E<Polyhedron<3>>& poly)
{
  // std::cout << "Going to publish= " << (poly[0].hyperplanes())[0].n_ << std::endl;
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly);
  poly_msg.header.frame_id = world_name_;

  poly_safe_pub_.publish(poly_msg);
}

void PantherRos::stateCB(const snapstack_msgs::State& msg)
{
  mt::state state_tmp;
  state_tmp.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_tmp.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_tmp.setAccel(0.0, 0.0, 0.0);
  state_tmp.setYaw(0.0);  // This field is not used

  // double roll, pitch, yaw;
  // quaternion2Euler(msg.quat, roll, pitch, yaw);
  // state_tmp.setYaw(yaw); TODO: use here (for yaw) the convention PANTHER is using (psi)

  state_ = state_tmp;

  // std::cout << bold << yellow << "PANTHER_ROS state= " << reset;
  // state_tmp.print();
  panther_ptr_->updateState(state_tmp);

  // W_T_B_ = Eigen::Translation3d(msg.pos.x, msg.pos.y, msg.pos.z) *
  //          Eigen::Quaterniond(msg.quat.w, msg.quat.x, msg.quat.y, msg.quat.z);

  if (published_initial_position_ == false)
  {
    pwp_last_ = createPwpFromStaticPosition(state_);
    publishOwnTraj(pwp_last_);
    published_initial_position_ = true;
  }
  if (panther_ptr_->IsTranslating() == true && par_.visual)
  {
    pubActualTraj();
  }
}

void PantherRos::whoPlansCB(const panther_msgs::WhoPlans& msg)
{
  if (msg.value != msg.PANTHER)
  {  // PANTHER does nothing
    sub_state_.shutdown();
    sub_term_goal_.shutdown();
    pubCBTimer_.stop();
    replanCBTimer_.stop();
    panther_ptr_->resetInitialization();
    std::cout << on_blue << "**************PANTHER STOPPED" << reset << std::endl;
  }
  else
  {  // PANTHER is the one who plans now (this happens when the take-off is finished)
    sub_term_goal_ = nh1_.subscribe("term_goal", 1, &PantherRos::terminalGoalCB, this);  // TODO: duplicated from above
    sub_state_ = nh1_.subscribe("state", 1, &PantherRos::stateCB, this);                 // TODO: duplicated from above
    pubCBTimer_.start();
    replanCBTimer_.start();
    std::cout << on_blue << "**************PANTHER STARTED" << reset << std::endl;
  }
}

void PantherRos::pubCB(const ros::TimerEvent& e)
{
  mt::state next_goal;
  if (panther_ptr_->getNextGoal(next_goal))
  {
    snapstack_msgs::Goal goal;

    goal.p = eigen2rosvector(next_goal.pos);
    goal.v = eigen2rosvector(next_goal.vel);
    goal.a = eigen2rosvector((par_.use_ff) * next_goal.accel);
    goal.j = eigen2rosvector((par_.use_ff) * next_goal.jerk);
    goal.dpsi = next_goal.dyaw;
    goal.psi = next_goal.yaw;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = world_name_;
    goal.power = true;  // allow the outer loop to send low-level autopilot commands

    pub_goal_.publish(goal);

    setpoint_.header.stamp = ros::Time::now();
    setpoint_.pose.position.x = goal.p.x;
    setpoint_.pose.position.y = goal.p.y;
    setpoint_.pose.position.z = goal.p.z;

    pub_setpoint_.publish(setpoint_);
  }

  publishFOV();
}

void PantherRos::clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher)
{
  if ((*tmp).markers.size() == 0)
  {
    return;
  }
  int id_begin = (*tmp).markers[0].id;

  for (int i = 0; i < (*tmp).markers.size(); i++)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::DELETE;
    m.id = i + id_begin;
    m.scale.x = 0.15;
    m.scale.y = 0;
    m.scale.z = 0;
    (*tmp).markers[i] = m;
  }

  (*publisher).publish(*tmp);
  (*tmp).markers.clear();
}

void PantherRos::pubTraj(const std::vector<mt::state>& data)
{
  // Trajectory
  nav_msgs::Path traj;
  traj.poses.clear();
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = world_name_;

  geometry_msgs::PoseStamped temp_path;

  int increm = (int)std::max(data.size() / par_.res_plot_traj, 1.0);  // this is to speed up rviz

  for (int i = 0; i < data.size(); i = i + increm)
  {
    temp_path.pose.position.x = data[i].pos(0);
    temp_path.pose.position.y = data[i].pos(1);
    temp_path.pose.position.z = data[i].pos(2);
    temp_path.pose.orientation.w = 1;
    temp_path.pose.orientation.x = 0;
    temp_path.pose.orientation.y = 0;
    temp_path.pose.orientation.z = 0;
    traj.poses.push_back(temp_path);
  }

  pub_traj_safe_.publish(traj);
  clearMarkerArray(&traj_safe_colored_, &pub_traj_safe_colored_);

  double scale = 0.15;

  traj_safe_colored_ = trajectory2ColoredMarkerArray(data, par_.v_max.maxCoeff(), increm, name_drone_, scale,
                                                     par_.color_type, id_, par_.n_agents);
  pub_traj_safe_colored_.publish(traj_safe_colored_);
}

void PantherRos::pubActualTraj()
{
  static geometry_msgs::Point p_last = pointOrigin();

  mt::state current_state;
  panther_ptr_->getState(current_state);
  Eigen::Vector3d act_pos = current_state.pos;

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.id = actual_trajID_;  // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
  m.ns = "ActualTraj_" + name_drone_;
  actual_trajID_++;
  // m.color = getColorJet(current_state.vel.norm(), 0, par_.v_max.maxCoeff());  // color(RED_NORMAL);

  if (par_.color_type == "vel")
  {
    m.color = getColorJet(current_state.vel.norm(), 0, par_.v_max.maxCoeff());  // note that par_.v_max is per axis!
  }
  else
  {
    m.color = getColorJet(id_, 0, par_.n_agents);  // note that par_.v_max is per axis!
  }

  m.scale.x = 0.15;
  m.scale.y = 0.0001;
  m.scale.z = 0.0001;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = world_name_;

  // pose is actually not used in the marker, but if not RVIZ complains about the quaternion
  m.pose.position = pointOrigin();
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  geometry_msgs::Point p;
  p = eigen2point(act_pos);
  m.points.push_back(p_last);
  m.points.push_back(p);
  p_last = p;

  if (m.id == 0)
  {
    return;
  }

  pub_actual_traj_.publish(m);
}

void PantherRos::clearMarkerActualTraj()
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  pub_actual_traj_.publish(m);
  actual_trajID_ = 0;
}

void PantherRos::clearMarkerColoredTraj()
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 1;
  m.scale.y = 1;
  m.scale.z = 1;
  pub_actual_traj_.publish(m);
}

void PantherRos::pubState(const mt::state& data, const ros::Publisher pub)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = world_name_;
  p.point = eigen2point(data.pos);
  pub.publish(p);
}

void PantherRos::terminalGoalCB(const geometry_msgs::PoseStamped& msg)
{
  mt::state G_term;
  double z;
  if (fabs(msg.pose.position.z) < 1e-5)  // This happens when you click in RVIZ (msg.z is 0.0)
  {
    z = 2.0;
  }
  else  // This happens when you publish by yourself the goal (should always be above the ground)
  {
    z = msg.pose.position.z;
  }
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, z);
  panther_ptr_->setTerminalGoal(G_term);

  mt::state G;  // projected goal
  panther_ptr_->getG(G);

  pubState(G_term, pub_point_G_term_);
  pubState(G, pub_point_G_);

  clearMarkerActualTraj();
}

void PantherRos::constructFOVMarker()
{
  marker_fov_.header.frame_id = name_camera_depth_optical_frame_tf_;  // name_drone_;
  marker_fov_.header.stamp = ros::Time::now();
  marker_fov_.ns = "marker_fov";
  marker_fov_.id = 0;
  marker_fov_.frame_locked = true;
  marker_fov_.type = marker_fov_.LINE_LIST;
  marker_fov_.action = marker_fov_.ADD;
  marker_fov_.pose = identityGeometryMsgsPose();

  double delta_y = par_.fov_depth * fabs(tan((par_.fov_x_deg * M_PI / 180) / 2.0));
  double delta_z = par_.fov_depth * fabs(tan((par_.fov_y_deg * M_PI / 180) / 2.0));

  geometry_msgs::Point v0 = eigen2point(Eigen::Vector3d(0.0, 0.0, 0.0));
  geometry_msgs::Point v1 = eigen2point(Eigen::Vector3d(-delta_y, delta_z, par_.fov_depth));
  geometry_msgs::Point v2 = eigen2point(Eigen::Vector3d(delta_y, delta_z, par_.fov_depth));
  geometry_msgs::Point v3 = eigen2point(Eigen::Vector3d(delta_y, -delta_z, par_.fov_depth));
  geometry_msgs::Point v4 = eigen2point(Eigen::Vector3d(-delta_y, -delta_z, par_.fov_depth));

  // geometry_msgs::Point v1 = eigen2point(Eigen::Vector3d(par_.fov_depth, delta_y, -delta_z));
  // geometry_msgs::Point v2 = eigen2point(Eigen::Vector3d(par_.fov_depth, -delta_y, -delta_z));
  // geometry_msgs::Point v3 = eigen2point(Eigen::Vector3d(par_.fov_depth, -delta_y, delta_z));
  // geometry_msgs::Point v4 = eigen2point(Eigen::Vector3d(par_.fov_depth, delta_y, delta_z));

  marker_fov_.points.clear();

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v1);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v2);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v3);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v4);

  // Line
  marker_fov_.points.push_back(v1);
  marker_fov_.points.push_back(v2);

  // Line
  marker_fov_.points.push_back(v2);
  marker_fov_.points.push_back(v3);

  // Line
  marker_fov_.points.push_back(v3);
  marker_fov_.points.push_back(v4);

  // Line
  marker_fov_.points.push_back(v4);
  marker_fov_.points.push_back(v1);

  marker_fov_.scale.x = 0.03;
  marker_fov_.scale.y = 0.00001;
  marker_fov_.scale.z = 0.00001;
  marker_fov_.color.a = 1.0;
  marker_fov_.color.r = 0.0;
  marker_fov_.color.g = 1.0;
  marker_fov_.color.b = 0.0;
}

void PantherRos::publishFOV()
{
  marker_fov_.header.stamp = ros::Time::now();
  pub_fov_.publish(marker_fov_);
  return;
}

void PantherRos::verify(bool cond, std::string info_if_false)
{
  if (cond == false)
  {
    std::cout << termcolor::bold << termcolor::red << info_if_false << termcolor::reset << std::endl;
    std::cout << termcolor::red << "Aborting" << termcolor::reset << std::endl;
    abort();
  }
}
