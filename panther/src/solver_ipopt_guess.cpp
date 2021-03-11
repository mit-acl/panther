/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "solver_ipopt.hpp"
#include "termcolor.hpp"
#include "bspline_utils.hpp"
#include "ros/ros.h"

#include <decomp_util/ellipsoid_decomp.h>  //For Polyhedron definition
#include <unsupported/Eigen/Splines>
#include <iostream>
#include <list>
#include <random>
#include <iostream>
#include <vector>

using namespace termcolor;

bool SolverIpopt::generateAStarGuess()
{
  std::cout << "[NL] Running A* from" << q0_.transpose() << " to " << final_state_.pos.transpose()
            << ", allowing time = " << kappa_ * max_runtime_ * 1000 << " ms" << std::endl;

  n_guess_.clear();
  qp_guess_.clear();
  d_guess_.clear();
  planes_.clear();

  // generateStraightLineGuess();
  /*  generateRandomN(n_guess_);
    generateRandomD(d_guess_);
    generateRandomQ(qp_guess_);*/

  // std::cout << "The StraightLineGuess is" << std::endl;
  // printStd(qp_guess_);

  octopusSolver_ptr_->setUp(t_init_, t_final_, hulls_);

  octopusSolver_ptr_->setq0q1q2(q0_, q1_, q2_);
  octopusSolver_ptr_->setGoal(final_state_.pos);

  double goal_size = 0.05;  //[meters]

  octopusSolver_ptr_->setXYZMinMaxAndRa(par_.x_min, par_.x_max, par_.y_min, par_.y_max, par_.z_min, par_.z_max,
                                        par_.Ra);             // limits for the search, in world frame
  octopusSolver_ptr_->setBBoxSearch(2000.0, 2000.0, 2000.0);  // limits for the search, centered on q2
  octopusSolver_ptr_->setMaxValuesAndSamples(par_.v_max, par_.a_max, par_.a_star_samp_x, par_.a_star_samp_y,
                                             par_.a_star_samp_z, par_.a_star_fraction_voxel_size);

  octopusSolver_ptr_->setRunTime(kappa_ * max_runtime_);  // hack, should be kappa_ * max_runtime_
  octopusSolver_ptr_->setGoalSize(goal_size);
  octopusSolver_ptr_->setBias(par_.a_star_bias);
  octopusSolver_ptr_->setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  log_ptr_->tim_guess_pos.tic();
  bool success = octopusSolver_ptr_->run(q, n, d);
  log_ptr_->tim_guess_pos.toc();

  // num_of_LPs_run_ = octopusSolver_ptr_->getNumOfLPsRun();

  // fillPlanesFromNDQ(n_guess_, d_guess_, qp_guess_);

  if (success)
  {
    log_ptr_->success_guess_pos = true;
    ROS_INFO_STREAM("[NL] A* found a feasible solution!");
    qp_guess_ = q;
    n_guess_ = n;
    d_guess_ = d;
    // For a 3d plot of the plane found, see figures at
    // https://github.com/mit-acl/separator/tree/06c0ddc6e2f11dbfc5b6083c2ea31b23fd4fa9d1
    // At this point the blue planes have the the equation n'x+d == -1
    // The free space is on the side n'x+d <= -1 (and also on the side n'x+d <= 1)
    // The obstacle is on the side n'x+d >= 1 (and also on the side n'x+d >= -1)

    return true;
  }
  else
  {
    log_ptr_->success_guess_pos = false;
    return false;
  }
}

void SolverIpopt::generateRandomD(std::vector<double>& d)
{
  d.clear();

  for (int j = 0; j < num_of_obst_ * num_of_segments_; j++)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    d.push_back(r1);
  }
}

void SolverIpopt::generateRandomN(std::vector<Eigen::Vector3d>& n)
{
  n.clear();
  for (int j = 0; j < num_of_obst_ * num_of_segments_; j++)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    double r2 = ((double)rand() / (RAND_MAX));
    double r3 = ((double)rand() / (RAND_MAX));
    n.push_back(Eigen::Vector3d(r1, r2, r3));
  }

  // std::cout << "After Generating RandomN, n has size= " << n.size() << std::endl;
}

void SolverIpopt::generateRandomQ(std::vector<Eigen::Vector3d>& q)
{
  q.clear();

  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> dist_x(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_y(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_z(par_.z_min, par_.z_max);

  for (int i = 0; i <= N_; i++)
  {
    q.push_back(Eigen::Vector3d(dist_x(generator), dist_y(generator), dist_z(generator)));
  }

  saturateQ(q);  // make sure is inside the bounds specified
}

void SolverIpopt::generateRandomGuess()
{
  n_guess_.clear();
  qp_guess_.clear();
  d_guess_.clear();

  generateRandomN(n_guess_);
  generateRandomD(d_guess_);
  generateRandomQ(qp_guess_);
}

void SolverIpopt::generateStraightLineGuess()
{
  // std::cout << "Using StraightLineGuess" << std::endl;
  qp_guess_.clear();
  n_guess_.clear();
  d_guess_.clear();

  qp_guess_.push_back(q0_);  // Not a decision variable
  qp_guess_.push_back(q1_);  // Not a decision variable
  qp_guess_.push_back(q2_);  // Not a decision variable

  for (int i = 1; i < (N_ - 2 - 2); i++)
  {
    Eigen::Vector3d q_i = q2_ + i * (final_state_.pos - q2_) / (N_ - 2 - 2);
    qp_guess_.push_back(q_i);
  }

  qp_guess_.push_back(qNm2_);  // three last cps are the same because of the vel/accel final conditions
  qp_guess_.push_back(qNm1_);
  qp_guess_.push_back(qN_);
  // Now qp_guess_ should have (N_+1) elements
  saturateQ(qp_guess_);  // make sure is inside the bounds specified

  //////////////////////

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      std::vector<Eigen::Vector3d> last4Cps(4);

      Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
      Eigen::Matrix<double, 3, 4> Qmv;  // minvo. each column contains a MINVO control point
      Qbs.col(0) = qp_guess_[i];
      Qbs.col(1) = qp_guess_[i + 1];
      Qbs.col(2) = qp_guess_[i + 2];
      Qbs.col(3) = qp_guess_[i + 3];

      transformPosBSpline2otherBasis(Qbs, Qmv, i);

      Eigen::Vector3d n_i;
      double d_i;

      bool satisfies_LP = separator_solver_ptr_->solveModel(n_i, d_i, hulls_[obst_index][i], Qmv);

      n_guess_.push_back(n_i);
      d_guess_.push_back(d_i);
    }
  }
}