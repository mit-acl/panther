/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <casadi/casadi.hpp>

#include "solver_ipopt.hpp"
#include "termcolor.hpp"
#include "bspline_utils.hpp"
#include "ros/ros.h"

#include <unsupported/Eigen/Splines>
#include <iostream>
#include <list>
#include <random>
#include <iostream>
#include <vector>
#include <fstream>

#include <ros/package.h>

using namespace termcolor;

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

SolverIpopt::SolverIpopt(mt::parameters &par, std::shared_ptr<mt::log> log_ptr)
{
  par_ = par;
  log_ptr_ = log_ptr;

  // All these values are for the position spline
  p_ = par_.deg_pos;
  M_ = par_.num_seg + 2 * p_;
  N_ = M_ - p_ - 1;

  Ny_ = (par_.num_seg + par_.deg_yaw - 1);
  ///////////////////////////////////////

  mt::basisConverter basis_converter;
  // basis used for collision
  if (par_.basis == "MINVO")
  {
    basis_ = MINVO;
    M_pos_bs2basis_ = basis_converter.getMinvoDeg3Converters(par_.num_seg);
    M_vel_bs2basis_ = basis_converter.getMinvoDeg2Converters(par_.num_seg);
  }
  else if (par_.basis == "BEZIER")
  {
    basis_ = BEZIER;
    M_pos_bs2basis_ = basis_converter.getBezierDeg3Converters(par_.num_seg);
    M_vel_bs2basis_ = basis_converter.getBezierDeg2Converters(par_.num_seg);
  }
  else if (par_.basis == "B_SPLINE")
  {
    basis_ = B_SPLINE;
    M_pos_bs2basis_ = basis_converter.getBSplineDeg3Converters(par_.num_seg);
    M_vel_bs2basis_ = basis_converter.getBSplineDeg2Converters(par_.num_seg);
  }
  else
  {
    std::cout << red << "Basis " << par_.basis << " not implemented yet" << reset << std::endl;
    std::cout << red << "============================================" << reset << std::endl;
    abort();
  }

  ///////////////////////////////////////
  ///////////////////////////////////////

  // // TODO: if C++14, use std::make_unique instead
  separator_solver_ptr_ = std::unique_ptr<separator::Separator>(new separator::Separator());
  octopusSolver_ptr_ =
      std::unique_ptr<OctopusSearch>(new OctopusSearch(par_.basis, par_.num_seg, par_.deg_pos, par_.alpha_shrink));

  std::string folder = ros::package::getPath("panther") + "/matlab/casadi_generated_files/";
  std::fstream myfile(folder + "index_instruction.txt", std::ios_base::in);
  myfile >> index_instruction_;
  cf_op_ = casadi::Function::load(folder + "op.casadi");
  // cf_op_force_final_pos_ = casadi::Function::load(folder + "op_force_final_pos.casadi");
  cf_fixed_pos_op_ = casadi::Function::load(folder + "op_fixed_pos.casadi");
  cf_fit_yaw_ = casadi::Function::load(folder + "fit_yaw.casadi");
  cf_visibility_ = casadi::Function::load(folder + "visibility.casadi");

  // OTHER OPTION:    std::cout << bold << red << getPathName(__FILE__) << reset << std::endl;
  // getPathName() is defined above in this file

  all_w_fe_ = casadi::DM(3, par_.num_samples_simpson);
  all_w_velfewrtworld_ = casadi::DM(3, par_.num_samples_simpson);

  Eigen::Matrix<double, 4, 4> b_Tmatrix_c = par_.b_T_c.matrix();
  b_Tmatrixcasadi_c_ = casadi::DM(4, 4);

  // std::cout << "par_.b_T_c.matrix()= " << par_.b_T_c.matrix() << std::endl;

  for (int i = 0; i < b_Tmatrix_c.rows(); i++)
  {
    for (int j = 0; j < b_Tmatrix_c.cols(); j++)
    {
      b_Tmatrixcasadi_c_(i, j) = b_Tmatrix_c(i, j);
    }
  }

  //////////////////////////////////////// CONSTRUCT THE GRAPH FOR THE YAW SEARCH
  ////////////////////////////////////////////////////////////////////////////////

  num_of_yaw_per_layer_ = par_.num_of_yaw_per_layer;
  num_of_layers_ = par_.num_samples_simpson;

  vector_yaw_samples_ = casadi::DM::zeros(1, num_of_yaw_per_layer_);
  for (int j = 0; j < num_of_yaw_per_layer_; j++)
  {
    vector_yaw_samples_(j) = -M_PI + j * 2 * M_PI / num_of_yaw_per_layer_;  // \in [-pi, pi]
  }

  // mygraph_t mygraph_(0);  // start a graph with 0 vertices
  mygraph_.clear();

  // create all the vertexes and add them to the graph
  std::vector<std::vector<vd>> all_vertexes_tmp(num_of_layers_ - 1, std::vector<vd>(num_of_yaw_per_layer_));  // TODO
  all_vertexes_ = all_vertexes_tmp;
  std::vector<vd> tmp(1);  // first layer only one element
  all_vertexes_.insert(all_vertexes_.begin(), tmp);

  // https://stackoverflow.com/questions/47904550/should-i-keep-track-of-vertex-descriptors-in-boost-graph-library

  double y0_tmp = 0.0;  // this value will be updated at the start of each iteration

  // add rest of the vertexes
  for (size_t i = 0; i < num_of_layers_; i++)  // i is the index of each layer
  {
    size_t num_of_circles_layer_i = (i == 0) ? 1 : num_of_yaw_per_layer_;
    for (size_t j = 0; j < num_of_circles_layer_i; j++)  // j is the index of each  circle in the layer i
    {
      vd vertex1 = boost::add_vertex(mygraph_);
      all_vertexes_[i][j] = vertex1;
      mygraph_[vertex1].yaw = (i == 0) ? y0_tmp : double(vector_yaw_samples_(j));
      mygraph_[vertex1].layer = i;
      mygraph_[vertex1].circle = j;
      // mygraph_[vertex1].print();
      // std::cout << "So far, the graph has " << num_vertices(mygraph_) << "vertices" << std::endl;
    }
  }

  for (size_t i = 0; i < (num_of_layers_ - 1); i++)  // i is the number of layers
  {
    size_t num_of_circles_layer_i = (i == 0) ? 1 : num_of_yaw_per_layer_;

    for (size_t j = 0; j < num_of_circles_layer_i; j++)  // j is the circle index of layer i
    {
      for (size_t j_next = 0; j_next < num_of_yaw_per_layer_; j_next++)
      {
        vd index_vertex1 = all_vertexes_[i][j];
        vd index_vertex2 = all_vertexes_[i + 1][j_next];

        // std::cout <<  mygraph_[index_vertex2].layer << ", " << mygraph_[index_vertex2].circle
        //           << std::endl;
        // std::cout <<  i + 1 << ", " << j_next << std::endl;

        edge_descriptor e;
        bool inserted;
        boost::tie(e, inserted) = add_edge(index_vertex1, index_vertex2, mygraph_);
      }
    }
  }

  ////////////////////////////////////////
  ////////////////////////////////////////
}

SolverIpopt::~SolverIpopt()
{
}

void SolverIpopt::getPlanes(std::vector<Hyperplane3D> &planes)
{
  planes = planes_;
}

int SolverIpopt::getNumOfLPsRun()
{
  return octopusSolver_ptr_->getNumOfLPsRun();
}

int SolverIpopt::getNumOfQCQPsRun()
{
  return num_of_QCQPs_run_;
}

void SolverIpopt::setMaxRuntimeKappaAndMu(double max_runtime, double kappa, double mu)
{
  kappa_ = kappa;
  mu_ = mu;
  max_runtime_ = max_runtime;
}

void SolverIpopt::setHulls(ConvexHullsOfCurves_Std &hulls)
{
  hulls_.clear();
  hulls_ = hulls;
  num_of_obst_ = hulls_.size();
  num_of_normals_ = par_.num_seg * num_of_obst_;
}

//////////////////////////////////////////////////////////

void SolverIpopt::setSimpsonFeatureSamples(const std::vector<Eigen::Vector3d> &samples,
                                           const std::vector<Eigen::Vector3d> &w_velsampleswrtworld)
{
  assert(samples.size() == par_.num_samples_simpson);
  assert(w_velsampleswrtworld.size() == par_.num_samples_simpson);
  for (int i = 0; i < samples.size(); i++)
  {
    all_w_fe_(0, i) = samples[i].x();
    all_w_fe_(1, i) = samples[i].y();
    all_w_fe_(2, i) = samples[i].z();

    all_w_velfewrtworld_(0, i) = w_velsampleswrtworld[i].x();
    all_w_velfewrtworld_(1, i) = w_velsampleswrtworld[i].y();
    all_w_velfewrtworld_(2, i) = w_velsampleswrtworld[i].z();
  }
}

casadi::DM SolverIpopt::eigen2casadi(const Eigen::Vector3d &a)
{
  casadi::DM b = casadi::DM::zeros(3, 1);
  b(0, 0) = a(0);
  b(1, 0) = a(1);
  b(2, 0) = a(2);
  return b;
};

// Note that t_final will be updated in case the saturation in deltaT_ has had effect
bool SolverIpopt::setInitStateFinalStateInitTFinalT(mt::state initial_state, mt::state final_state, double t_init,
                                                    double &t_final)
{
  ///////////////////////////
  Eigen::Vector3d p0 = initial_state.pos;
  Eigen::Vector3d v0 = initial_state.vel;
  Eigen::Vector3d a0 = initial_state.accel;

  Eigen::Vector3d pf = final_state.pos;
  Eigen::Vector3d vf = final_state.vel;
  Eigen::Vector3d af = final_state.accel;

  // here we saturate the value to ensure it is within the limits
  // the reason for this is the epsilon_tol_constraints (in the previous iteration, it may be slightly unfeasible)
  saturate(v0, -par_.v_max, par_.v_max);  // TODO: should this change be also reflected in initial_state_?
  saturate(a0, -par_.a_max, par_.a_max);
  saturate(vf, -par_.v_max, par_.v_max);
  saturate(af, -par_.a_max, par_.a_max);

  initial_state_ = initial_state;
  final_state_ = final_state;

  initial_state_.yaw = wrapFromMPitoPi(initial_state_.yaw);
  final_state_.yaw = wrapFromMPitoPi(final_state_.yaw);

  /// Now shift final_state_.yaw  so that the difference wrt initial_state_.yaw is <=pi

  double previous_phi = initial_state_.yaw;
  double phi_i = final_state_.yaw;
  double difference = previous_phi - phi_i;

  double phi_i_f = phi_i + floor(difference / (2 * M_PI)) * 2 * M_PI;
  double phi_i_c = phi_i + ceil(difference / (2 * M_PI)) * 2 * M_PI;

  final_state_.yaw = (fabs(previous_phi - phi_i_f) < fabs(previous_phi - phi_i_c)) ? phi_i_f : phi_i_c;

  /// Just for debugging
  if (fabs(initial_state_.yaw - final_state_.yaw) > M_PI)
  {
    std::cout << red << bold << "This diff must be <= pi" << reset << std::endl;
    abort();
  }
  ///

  std::cout << "initial_state= " << std::endl;
  initial_state.printHorizontal();

  std::cout << "final_state= " << std::endl;
  final_state.printHorizontal();

  //////////////////////////////

  deltaT_ = (t_final - t_init) / (1.0 * (M_ - 2 * p_ - 1 + 1));

  double old_deltaT = deltaT_;

  //////////////////////////////
  // Now make sure deltaT in knots_ is such that -v_max<=v1<=v_max is satisfied:
  for (int axis = 0; axis < 3; axis++)
  {
    double upper_bound, lower_bound;
    if (fabs(a0(axis)) > 1e-7)
    {
      upper_bound = ((p_ - 1) * (sgn(a0(axis)) * par_.v_max(axis) - v0(axis)) / (a0(axis)));
      lower_bound = ((p_ - 1) * (-sgn(a0(axis)) * par_.v_max(axis) - v0(axis)) / (a0(axis)));

      ////////////////// Debugging
      // if (upper_bound < lower_bound)
      // {
      //   std::cout << red << bold << "This should never happen, aborting" << std::endl;
      //   abort();
      // }
      //////////////////

      if (upper_bound <= 0)
      {
        std::cout << red << bold << "There is no way to satisfy v1" << std::endl;  //(deltat will be zero)
        return false;
      }

      saturate(deltaT_, std::max(0.0, lower_bound), upper_bound);
    }
    else
    {
      // do nothing: a0 ==0 for that axis, so that means that v1==v0, and therefore v1 satisfies constraints for that
      // axis
    }
  }

  if (old_deltaT != deltaT_)
  {
    std::cout << red << bold << "old_deltaT= " << old_deltaT << reset << std::endl;
    std::cout << red << bold << "deltaT_= " << deltaT_ << reset << std::endl;
  }

  // Eigen::Vector3d bound1 = ((p_ - 1) * (par_.v_max - v0).array() / (a0.array()));
  // Eigen::Vector3d bound2 = ((p_ - 1) * (-par_.v_max - v0).array() / (a0.array()));

  // // note that if any element of a0 is ==0.0, then its corresponding element in bound1 (or bound2) is +-infinity,
  // but
  // // valid  for the saturation below

  // saturate(deltaT_, std::min(bound1.x(), bound2.x()), std::max(bound1.x(), bound2.x()));
  // saturate(deltaT_, std::min(bound1.y(), bound2.y()), std::max(bound1.y(), bound2.y()));
  // saturate(deltaT_, std::min(bound1.z(), bound2.z()), std::max(bound1.z(), bound2.z()));

  // std::cout << "std::min(bound1.x(), bound2.x()= " << std::min(bound1.x(), bound2.x()) << std::endl;
  // std::cout << "std::max(bound1.x(), bound2.x()= " << std::max(bound1.x(), bound2.x()) << std::endl;

  // std::cout << "std::min(bound1.y(), bound2.y()= " << std::min(bound1.y(), bound2.y()) << std::endl;
  // std::cout << "std::max(bound1.y(), bound2.y()= " << std::max(bound1.y(), bound2.y()) << std::endl;

  // std::cout << "std::min(bound1.z(), bound2.z()= " << std::min(bound1.z(), bound2.z()) << std::endl;
  // std::cout << "std::max(bound1.z(), bound2.z()= " << std::max(bound1.z(), bound2.z()) << std::endl;

  // std::cout << bold << "deltaT_ after= " << deltaT_ << reset << std::endl;

  t_final = t_init + (1.0 * (M_ - 2 * p_ - 1 + 1)) * deltaT_;

  t_init_ = t_init;
  t_final_ = t_final;

  /////////////////////////

  /////////////////////////

  Eigen::RowVectorXd knots(M_ + 1);
  for (int i = 0; i <= p_; i++)
  {
    knots[i] = t_init_;
  }

  for (int i = (p_ + 1); i <= M_ - p_ - 1; i++)
  {
    knots[i] = knots[i - 1] + deltaT_;  // Uniform b-spline (internal knots are equally spaced)
  }

  for (int i = (M_ - p_); i <= M_; i++)
  {
    knots[i] = t_final_;
  }

  knots_ = knots;
  //////////////////

  //////////////////

  // See https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html

  double t1 = knots_(1);
  double t2 = knots_(2);
  double tpP1 = knots_(p_ + 1);
  double t1PpP1 = knots_(1 + p_ + 1);

  double tN = knots_(N_);
  double tNm1 = knots_(N_ - 1);
  double tNPp = knots_(N_ + p_);
  double tNm1Pp = knots_(N_ - 1 + p_);

  // See Mathematica Notebook
  q0_ = p0;
  q1_ = p0 + (-t1 + tpP1) * v0 / p_;
  q2_ = (p_ * p_ * q1_ - (t1PpP1 - t2) * (a0 * (t2 - tpP1) + v0) - p_ * (q1_ + (-t1PpP1 + t2) * v0)) / ((-1 + p_) * p_);

  qN_ = pf;
  qNm1_ = pf + ((tN - tNPp) * vf) / p_;
  qNm2_ = (p_ * p_ * qNm1_ - (tNm1 - tNm1Pp) * (af * (-tN + tNm1Pp) + vf) - p_ * (qNm1_ + (-tNm1 + tNm1Pp) * vf)) /
          ((-1 + p_) * p_);

  return true;
}

bool SolverIpopt::optimize()
{
  std::cout << "in SolverIpopt::optimize" << std::endl;

  // reset some stuff
  traj_solution_.clear();
  bool guess_found = generateAStarGuess();  // I obtain q_quess_, n_guess_, d_guess_
  if (guess_found == false)
  {
    std::cout << bold << red << "Planes haven't been found" << reset << std::endl;
    return false;
  }
  n_ = n_guess_;
  d_ = d_guess_;

  int max_num_of_planes = par_.num_max_of_obst * par_.num_seg;
  if ((n_guess_.size() > max_num_of_planes))
  {
    std::cout << red << bold << "the casadi function does not support so many planes" << reset << std::endl;
    std::cout << red << bold << "you have " << num_of_obst_ << "*" << par_.num_seg << "=" << n_guess_.size()
              << " planes" << std::endl;
    std::cout << red << bold << "and max is  " << par_.num_max_of_obst << "*" << par_.num_seg << "="
              << max_num_of_planes << " planes" << std::endl;
    return false;
  }

  ////////////////////////////////////
  //////////////////////////////////// CASADI

  // Conversion DM <--> Eigen:  https://github.com/casadi/casadi/issues/2563
  auto eigen2std = [](Eigen::Vector3d &v) { return std::vector<double>{ v.x(), v.y(), v.z() }; };

  std::map<std::string, casadi::DM> map_arguments;
  map_arguments["thetax_FOV_deg"] = par_.fov_x_deg;
  map_arguments["thetay_FOV_deg"] = par_.fov_y_deg;
  map_arguments["b_T_c"] = b_Tmatrixcasadi_c_;
  map_arguments["Ra"] = par_.Ra;
  map_arguments["p0"] = eigen2std(initial_state_.pos);
  map_arguments["v0"] = eigen2std(initial_state_.vel);
  map_arguments["a0"] = eigen2std(initial_state_.accel);
  map_arguments["pf"] = eigen2std(final_state_.pos);
  map_arguments["vf"] = eigen2std(final_state_.vel);
  map_arguments["af"] = eigen2std(final_state_.accel);
  map_arguments["y0"] = initial_state_.yaw;
  map_arguments["yf"] = final_state_.yaw;

  // if (fabs(final_state_.yaw) > 1e-5 || par_.c_final_yaw > 0.0)
  // {
  //   std::cout << red << bold << "Implement this!" << std::endl;
  //   abort();
  // }

  map_arguments["ydot0"] = initial_state_.dyaw;
  map_arguments["ydotf"] =
      final_state_.dyaw;  // Needed: if not (and if you are minimizing ddyaw), ddyaw=cte --> yaw will explode
  map_arguments["v_max"] = eigen2std(par_.v_max);
  map_arguments["a_max"] = eigen2std(par_.a_max);
  map_arguments["j_max"] = eigen2std(par_.j_max);
  map_arguments["ydot_max"] = par_.ydot_max;
  map_arguments["x_lim"] = std::vector<double>{ par_.x_min, par_.x_max };
  map_arguments["y_lim"] = std::vector<double>{ par_.y_min, par_.y_max };
  map_arguments["z_lim"] = std::vector<double>{ par_.z_min, par_.z_max };
  map_arguments["total_time"] = (t_final_ - t_init_);
  // all_w_fe is a matrix whose columns are the positions of the feature (in world frame) in the times [t0,t0+XX,
  // ...,tf-XX, tf] (i.e. uniformly distributed and including t0 and tf)
  map_arguments["all_w_fe"] = all_w_fe_;
  map_arguments["all_w_velfewrtworld"] = all_w_velfewrtworld_;

  map_arguments["c_pos_smooth"] = par_.c_pos_smooth;
  map_arguments["c_final_pos"] = par_.c_final_pos;  // / pow((final_state_.pos - initial_state_.pos).norm(), 4);
  map_arguments["c_final_yaw"] = par_.c_final_yaw;

  static casadi::DM all_nd(4, max_num_of_planes);
  all_nd = casadi::DM::zeros(4, max_num_of_planes);
  for (int i = 0; i < n_guess_.size(); i++)
  {
    // Casadi needs the plane equation as n_casadi'x+d_casadi<=0
    // The free space is on the side n'x+d <= -1 (and also on the side n'x+d <= 1)
    // Hence, n_casadi=n, and d_casadi=d-1
    all_nd(0, i) = n_guess_[i].x();
    all_nd(1, i) = n_guess_[i].y();
    all_nd(2, i) = n_guess_[i].z();
    all_nd(3, i) = d_guess_[i] - 1;
  }

  map_arguments["all_nd"] = all_nd;

  ///////////////// GUESS FOR POSITION CONTROL POINTS
  casadi::DM matrix_qp_guess(3, (N_ + 1));  // TODO: do this just once?
  for (int i = 0; i < matrix_qp_guess.columns(); i++)
  {
    matrix_qp_guess(0, i) = qp_guess_[i].x();
    matrix_qp_guess(1, i) = qp_guess_[i].y();
    matrix_qp_guess(2, i) = qp_guess_[i].z();
  }
  map_arguments["pCPs"] = matrix_qp_guess;

  ////////////////////////////////Generate Yaw Guess
  casadi::DM matrix_qy_guess(1, N_);  // TODO: do this just once?

  // if (use_straight_yaw_guess_ == false)
  // {
  matrix_qy_guess = generateYawGuess(matrix_qp_guess, all_w_fe_, initial_state_.yaw, initial_state_.dyaw,
                                     final_state_.dyaw, t_init_, t_final_);
  // }
  // else
  // {
  //   std::cout << "Using straight line guess" << std::endl;
  //   for (int i = 0; i < matrix_qy_guess.columns(); i++)
  //   {
  //     matrix_qy_guess(0, i) = initial_state_.yaw + i * (final_state_.yaw - initial_state_.yaw) / (1.0 * N_);
  //   }
  // }
  // std::cout << bold << blue << "Guess for yaw\n" << matrix_qy_guess << reset << std::endl;

  map_arguments["yCPs"] = matrix_qy_guess;

  // for(std::map<std::string, casadi::DM>::const_iterator it = map_arguments.begin();
  //     it != map_arguments.end(); ++it)
  // {
  //     std::cout << it->first << " " << it->second<< "\n";
  // }
  ////////////////////////// CALL THE SOLVER
  std::map<std::string, casadi::DM> result;
  log_ptr_->tim_opt.tic();
  // if (par_.force_final_pos == true)
  // {
  //   result = cf_op_force_final_pos_(map_arguments);
  // }
  // else
  // {
  // result = cf_op_(map_arguments);
  // }

  /////////////////////////////////

  if (par_.mode == "panther" && focus_on_obstacle_ == true)
  {
    map_arguments["c_yaw_smooth"] = par_.c_yaw_smooth;
    map_arguments["c_fov"] = par_.c_fov;
    std::cout << bold << green << "Optimizing for YAW and POSITION!" << reset << std::endl;
    result = cf_op_(map_arguments);
  }
  else if (par_.mode == "py" && focus_on_obstacle_ == true)
  {
    // first solve for the position spline
    map_arguments["c_yaw_smooth"] = 0.0;
    map_arguments["c_fov"] = 0.0;
    std::cout << bold << green << "Optimizing first for POSITION!" << reset << std::endl;
    result = cf_op_(map_arguments);

    // Use the position control points obtained for solve for yaw. Note that here the pos spline is FIXED
    map_arguments["c_yaw_smooth"] = par_.c_yaw_smooth;
    map_arguments["c_fov"] = par_.c_fov;
    map_arguments["pCPs"] = result["pCPs"];

    std::cout << bold << green << "and then for YAW!" << reset << std::endl;

    std::map<std::string, casadi::DM> result_for_yaw = cf_fixed_pos_op_(map_arguments);

    //////////// Debugging
    if (result["yCPs"].columns() != result_for_yaw["yCPs"].columns())
    {
      std::cout << "Sizes do not match. This is likely because you did not run main.m with both pos_is_fixed=true and "
                   "pos_is_fixed=false"
                << std::endl;
      abort();
    }
    ///////////////////

    result["yCPs"] = result_for_yaw["yCPs"];

    // The costs logged will not be the right ones, so don't use them in this mode
  }
  else if (par_.mode == "noPA" || par_.mode == "ysweep" || focus_on_obstacle_ == false)
  {
    map_arguments["c_yaw_smooth"] = 0.0;
    map_arguments["c_fov"] = 0.0;
    std::cout << bold << green << "Optimizing for POSITION!" << reset << std::endl;
    result = cf_op_(map_arguments);
  }
  else
  {
    std::cout << "Mode not implemented yet. Aborting" << std::endl;
    abort();
  }
  ////////////////////////////

  log_ptr_->tim_opt.toc();

  ///////////////// GET STATUS FROM THE SOLVER
  // Very hacky solution, see discussion at https://groups.google.com/g/casadi-users/c/1061E0eVAXM/m/dFHpw1CQBgAJ
  // Inspired from https://gist.github.com/jgillis/9d12df1994b6fea08eddd0a3f0b0737f
  // auto optimstatus = cf_op_.instruction_MX(index_instruction_).which_function().stats(1)["return_status"];

  std::string optimstatus;
  // if (par_.force_final_pos == true)
  // {
  //   optimstatus = std::string(
  //       cf_op_force_final_pos_.instruction_MX(index_instruction_).which_function().stats(1)["return_status"]);
  // }
  // else
  // {
  optimstatus = std::string(cf_op_.instruction_MX(index_instruction_).which_function().stats(1)["return_status"]);
  // }

  ////// Example of how to obtain inf_pr and inf_du
  // std::vector<double> inf_pr_all = std::map<std::string, casadi::GenericType>(
  //     cf_op_.instruction_MX(index_instruction_).which_function().stats(1)["iterations"])["inf_pr"];
  // std::vector<double> inf_du_all = std::map<std::string, casadi::GenericType>(
  //     cf_op_.instruction_MX(index_instruction_).which_function().stats(1)["iterations"])["inf_du"];
  // double inf_pr = inf_pr_all.back();
  // double inf_du = inf_du_all.back();
  // std::cout << "inf_pr= " << inf_pr << std::endl;
  // std::cout << "inf_du= " << inf_du << std::endl;

  //////////////// LOG COSTS OBTAINED
  log_ptr_->pos_smooth_cost = double(result["pos_smooth_cost"]);
  log_ptr_->yaw_smooth_cost = double(result["yaw_smooth_cost"]);
  log_ptr_->fov_cost = double(result["fov_cost"]);
  log_ptr_->final_pos_cost = double(result["final_pos_cost"]);
  log_ptr_->final_yaw_cost = double(result["final_yaw_cost"]);

  ///////////////// DECIDE ACCORDING TO STATUS OF THE SOLVER
  std::vector<Eigen::Vector3d> qp;  // Solution found (Control points for position)
  std::vector<double> qy;           // Solution found (Control points for yaw)
  std::cout << "optimstatus= " << optimstatus << std::endl;
  // See names here:
  // https://github.com/casadi/casadi/blob/fadc86444f3c7ab824dc3f2d91d4c0cfe7f9dad5/casadi/interfaces/ipopt/ipopt_interface.cpp
  if (optimstatus == "Solve_Succeeded" || optimstatus == "Solved_To_Acceptable_Level")
  {
    std::cout << green << "IPOPT found a solution" << reset << std::endl;
    log_ptr_->success_opt = true;
    // copy the solution
    auto qp_casadi = result["pCPs"];
    for (int i = 0; i < qp_casadi.columns(); i++)
    {
      qp.push_back(Eigen::Vector3d(double(qp_casadi(0, i)), double(qp_casadi(1, i)), double(qp_casadi(2, i))));
    }

    // std::cout<<"SOLUTION OPTIMIZATION: "<<result["yCps"]<<std::endl;
    // std::cout<<"all_w_fe="<<map_arguments["all_w_fe"]<<std::endl;
    // std::cout<<"all_w_velfewrtworld="<<map_arguments["all_w_velfewrtworld"]<<std::endl;

    ///////////////////////////////////
    if (par_.mode == "panther" || par_.mode == "py")
    {
      if (focus_on_obstacle_ == true)
      {
        qy = static_cast<std::vector<double>>(result["yCPs"]);
      }
      else
      {  // find the yaw spline that goes to final_state_.yaw as fast as possible

        double y0 = initial_state_.yaw;
        double yf = final_state_.yaw;
        double ydot0 = initial_state_.dyaw;
        int p = par_.deg_yaw;

        qy.clear();
        qy.push_back(y0);
        qy.push_back(y0 + deltaT_ * ydot0 / (double(p)));  // y0 and ydot0 fix the second control point

        int num_cps_yaw = par_.num_seg + p;

        for (int i = 0; i < (num_cps_yaw - 3); i++)
        {
          double v_needed = p * (yf - qy.back()) / (p * deltaT_);

          saturate(v_needed, -par_.ydot_max, par_.ydot_max);  // Make sure it's within the limits

          double next_qy = qy.back() + (p * deltaT_) * v_needed / (double(p));

          qy.push_back(next_qy);
        }

        qy.push_back(qy.back());  // TODO: HERE I'M ASSUMMING FINAL YAW VELOCITY=0 (i.e., final_state_.dyaw==0)

        // std::cout << "HERE 4!" << std::endl;
        // std::cout << "qy= " << std::endl;
        // for (auto qi : qy)
        // {
        //   std::cout << qi << " ";
        // }

        // std::cout << std::endl;
        // std::cout << "deltaT_= " << deltaT_ << std::endl;

        // std::cout << "yf= " << yf << std::endl;
        // std::cout << "y0= " << y0 << std::endl;
      }
    }
    else if (par_.mode == "noPA" || par_.mode == "ysweep")
    {  // constant yaw
      // Note that in ysweep, the yaw will be a sinusoidal function, see Panther::getNextGoal
      qy.clear();
      for (int i = 0; i < result["yCPs"].columns(); i++)
      {
        qy.push_back(initial_state_.yaw);
      }
    }
    else
    {
      std::cout << "Mode not implemented yet. Aborting" << std::endl;
      abort();
    }

    ///////////////////////////
  }
  else
  {
    std::cout << red << "IPOPT failed to find a solution" << reset << std::endl;
    log_ptr_->success_opt = false;
    // qp = qp_guess_;
    // qy = qy_guess_;
    // TODO: If I want to commit to the guesses, they need to be feasible (right now they aren't
    // because of j_max and yaw_dot_max) For now, let's not commit to them and return false
    return false;
  }

  ///////////////// PRINT SOLUTION
  // std::cout << "Position control Points obtained= " << std::endl;
  // printStd(qp);

  // std::cout << "Yaw control Points obtained= " << std::endl;
  // printStd(qy);

  ///////////////// Fill  traj_solution_ and pwp_solution_
  int param_pp = 3;
  int param_py = 2;
  // std::cout << "qy.size()= " << qy.size() << std::endl;
  CPs2TrajAndPwp(qp, qy, traj_solution_, pwp_solution_, param_pp, param_py, knots_, par_.dc);

  // Force last vel and jerk =final_state_ (which it's not guaranteed because of the discretization with par_.dc)
  traj_solution_.back().vel = final_state_.vel;
  traj_solution_.back().accel = final_state_.accel;
  traj_solution_.back().jerk = Eigen::Vector3d::Zero();
  traj_solution_.back().ddyaw = final_state_.ddyaw;

  // Uncomment the following line if you wanna visualize the planes
  // fillPlanesFromNDQ(n_, d_, qp);

  return true;
}

void SolverIpopt::getSolution(mt::PieceWisePol &solution)
{
  solution = pwp_solution_;
}

void SolverIpopt::fillPlanesFromNDQ(const std::vector<Eigen::Vector3d> &n, const std::vector<double> &d,
                                    const std::vector<Eigen::Vector3d> &q)
{
  planes_.clear();

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < par_.num_seg; i++)
    {
      int ip = obst_index * par_.num_seg + i;  // index plane
      Eigen::Vector3d centroid_hull;
      findCentroidHull(hulls_[obst_index][i], centroid_hull);

      Eigen::Vector3d point_in_plane;

      Eigen::Matrix<double, 3, 4> Qmv, Qbs;  // minvo. each column contains a MINVO control point
      Qbs.col(0) = q[i];
      Qbs.col(1) = q[i + 1];
      Qbs.col(2) = q[i + 2];
      Qbs.col(3) = q[i + 3];

      transformPosBSpline2otherBasis(Qbs, Qmv, i);

      Eigen::Vector3d centroid_cps = Qmv.rowwise().mean();

      // the colors refer to the second figure of
      // https://github.com/mit-acl/separator/tree/06c0ddc6e2f11dbfc5b6083c2ea31b23fd4fa9d1

      // Equation of the red planes is n'x+d == 1
      // Convert here to equation [A B C]'x+D ==0
      double A = n[ip].x();
      double B = n[ip].y();
      double C = n[ip].z();
      double D = d[ip] - 1;

      /////////////////// OPTION 1: point_in_plane = intersection between line  centroid_cps --> centroid_hull
      // bool intersects = getIntersectionWithPlane(centroid_cps, centroid_hull, Eigen::Vector4d(A, B, C, D),
      //                                            point_in_plane);  // result saved in point_in_plane

      //////////////////////////

      /////////////////// OPTION 2: point_in_plane = intersection between line  centroid_cps --> closest_vertex
      double dist_min = std::numeric_limits<double>::max();  // delta_min will contain the minimum distance between
                                                             // the centroid_cps and the vertexes of the obstacle
      int index_closest_vertex = 0;
      for (int j = 0; j < hulls_[obst_index][i].cols(); j++)
      {
        Eigen::Vector3d vertex = hulls_[obst_index][i].col(j);

        double distance_to_vertex = (centroid_cps - vertex).norm();
        if (distance_to_vertex < dist_min)
        {
          dist_min = distance_to_vertex;
          index_closest_vertex = j;
        }
      }

      Eigen::Vector3d closest_vertex = hulls_[obst_index][i].col(index_closest_vertex);

      bool intersects = getIntersectionWithPlane(centroid_cps, closest_vertex, Eigen::Vector4d(A, B, C, D),
                                                 point_in_plane);  // result saved in point_in_plane

      //////////////////////////

      if (intersects == false)
      {
        // TODO: this msg is printed sometimes in Multi-Agent simulations. Find out why
        std::cout << red << "There is no intersection, this should never happen (TODO)" << reset << std::endl;
        continue;  // abort();
      }

      Hyperplane3D plane(point_in_plane, n[i]);
      planes_.push_back(plane);
    }
  }
}

// returns 1 if there is an intersection between the segment P1-P2 and the plane given by coeff=[A B C D]
// (Ax+By+Cz+D==0)  returns 0 if there is no intersection.
// The intersection point is saved in "intersection"
bool SolverIpopt::getIntersectionWithPlane(const Eigen::Vector3d &P1, const Eigen::Vector3d &P2,
                                           const Eigen::Vector4d &coeff, Eigen::Vector3d &intersection)
{
  double A = coeff[0];
  double B = coeff[1];
  double C = coeff[2];
  double D = coeff[3];
  // http://www.ambrsoft.com/TrigoCalc/Plan3D/PlaneLineIntersection_.htm
  double x1 = P1[0];
  double a = (P2[0] - P1[0]);
  double y1 = P1[1];
  double b = (P2[1] - P1[1]);
  double z1 = P1[2];
  double c = (P2[2] - P1[2]);
  double t = -(A * x1 + B * y1 + C * z1 + D) / (A * a + B * b + C * c);

  (intersection)[0] = x1 + a * t;
  (intersection)[1] = y1 + b * t;
  (intersection)[2] = z1 + c * t;

  bool result = (t < 0 || t > 1) ? false : true;  // False if the intersection is with the line P1-P2, not with the
                                                  // segment P1 - P2

  return result;
}

//  casadi::DM all_nd(casadi::Sparsity::dense(4, max_num_of_planes));
// casadi::DM::rand(4, 0);

// std::string getPathName(const std::string &s)
// {
//   char sep = '/';

// #ifdef _WIN32
//   sep = '\\';
// #endif

//   size_t i = s.rfind(sep, s.length());
//   if (i != std::string::npos)
//   {
//     return (s.substr(0, i));
//   }

//   return ("");
// }