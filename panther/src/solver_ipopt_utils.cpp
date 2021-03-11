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

void SolverIpopt::printQND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  std::cout << std::setprecision(5) << std::endl;
  std::cout << "   control points:" << std::endl;
  for (Eigen::Vector3d q_i : q)
  {
    std::cout << q_i.transpose() << std::endl;
  }
  std::cout << "   normals:" << std::endl;
  for (Eigen::Vector3d n_i : n)
  {
    std::cout << n_i.transpose() << std::endl;
  }
  std::cout << "   d coeffs:" << std::endl;
  for (double d_i : d)
  {
    std::cout << d_i << std::endl;
  }
  std::cout << reset << std::endl;
}

void SolverIpopt::transformPosBSpline2otherBasis(const Eigen::Matrix<double, 3, 4> &Qbs,
                                                 Eigen::Matrix<double, 3, 4> &Qmv, int interval)
{
  Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverIpopt::transformVelBSpline2otherBasis(const Eigen::Matrix<double, 3, 3> &Qbs,
                                                 Eigen::Matrix<double, 3, 3> &Qmv, int interval)
{
  Qmv = Qbs * M_vel_bs2basis_[interval];
}

void SolverIpopt::saturateQ(std::vector<Eigen::Vector3d> &q)
{
  for (int i = 0; i < q.size(); i++)
  {
    q[i].z() = std::max(q[i].z(), par_.z_min);  // Make sure it's within the limits
    q[i].z() = std::min(q[i].z(), par_.z_max);  // Make sure it's within the limits
  }
}

void SolverIpopt::findCentroidHull(const Polyhedron_Std &hull, Eigen::Vector3d &centroid)
{
  centroid = Eigen::Vector3d::Zero();

  for (int i = 0; i < hull.cols(); i++)
  {
    centroid += hull.col(i);
  }
  if (hull.cols() > 0)
  {
    centroid = centroid / hull.cols();
  }
}

void SolverIpopt::printStd(const std::vector<Eigen::Vector3d> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i.transpose() << std::endl;
  }
}
void SolverIpopt::printStd(const std::vector<double> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i << std::endl;
  }
}