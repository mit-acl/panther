/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include "panther_types.hpp"

void CPs2TrajAndPwp_cleaner(std::vector<Eigen::Vector3d> &qp, std::vector<double> &qy, std::vector<mt::state> &traj,
                            mt::PieceWisePol &pwp, int param_pp, int param_py, Eigen::RowVectorXd &knots_p, double dc);

void CPs2TrajAndPwp(std::vector<Eigen::Vector3d> &q, std::vector<mt::state> &traj, mt::PieceWisePol &solution_, int N,
                    int p, int num_seg, Eigen::RowVectorXd &knots, double dc);

Eigen::Spline3d findInterpolatingBsplineNormalized(const std::vector<double> &times,
                                                   const std::vector<Eigen::Vector3d> &positions);