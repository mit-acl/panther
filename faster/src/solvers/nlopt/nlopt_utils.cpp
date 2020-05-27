// Jesus Tordesillas Torres, jtorde@mit.edu, May 2020

// This file simply creates a solverNlopt object, sets its params and some random obstacles, and then checks all the
// gradients numerically

#include <iostream>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#include <Eigen/Dense>
#include <random>
#include "./../../timer.hpp"

#include "solverNlopt.hpp"
#include "nlopt_utils.hpp"

bool nlopt_utils::checkGradientsNlopt()
{
  double z_ground = -2.0;
  double z_max = 2.0;
  double dc = 0.01;
  double Ra = 4.0;
  int deg_pol = 3;
  int samples_per_interval = 1;
  double weight = 10000.0;  // Note that internally, this weight will be changed to other value for the check (to get
                            // rid of numerical issues)
  double epsilon_tol_constraints = 0.1;
  double xtol_rel = 1e-07;
  double ftol_rel = 1e-07;
  std::string solver = "LD_MMA";
  double kappa = 0.0;
  double mu = 1.0;
  int a_star_samp_x = 7;
  int a_star_samp_y = 7;
  int a_star_samp_z = 7;
  double increment = 0.3;  // grid used to prune nodes that are on the same cell
  double runtime = 1.0;    //(not use this criterion)  //[seconds]
  double v_max = 10;
  double a_max = 60;
  state initial;
  initial.pos = Eigen::Vector3d(-4.0, 0.0, 0.0);
  state final;
  final.pos = Eigen::Vector3d(4.0, 0.0, 0.0);
  double dist_to_use_straight_guess = 1.0;
  double a_star_fraction_voxel_size = 0.0;
  int num_pol = 8;

  par_snlopt param;
  param.z_min = z_ground;
  param.z_max = z_max;
  param.v_max = v_max;
  param.a_max = a_max;
  param.dc = dc;
  param.dist_to_use_straight_guess = dist_to_use_straight_guess;
  param.a_star_samp_x = a_star_samp_x;
  param.a_star_samp_y = a_star_samp_y;
  param.a_star_samp_z = a_star_samp_z;
  param.a_star_fraction_voxel_size = a_star_fraction_voxel_size;
  param.num_pol = num_pol;
  param.deg_pol = deg_pol;
  param.weight = weight;
  param.epsilon_tol_constraints = epsilon_tol_constraints;
  param.xtol_rel = xtol_rel;
  param.ftol_rel = ftol_rel;
  param.solver = solver;
  param.basis = "B_SPLINE";

  double t_min = 0.0;
  double t_max = t_min + (final.pos - initial.pos).norm() / (0.3 * v_max);

  Polyhedron_Std hull;

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, -70.0));

  hull.push_back(Eigen::Vector3d(-0.5, 0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, 0.5, -70.0));

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, -70.0));
  hull.push_back(Eigen::Vector3d(-0.5, 0.5, -70.0));

  hull.push_back(Eigen::Vector3d(0.5, 0.5, 70.0));

  ConvexHullsOfCurves_Std hulls_curves;
  ConvexHullsOfCurve_Std hulls_curve;
  // Assummes static obstacle
  for (int i = 0; i < num_pol; i++)
  {
    hulls_curve.push_back(hull);
  }

  hulls_curves.push_back(hulls_curve);

  SolverNlopt snlopt(param);  // snlopt(a,g) a polynomials of degree 3
  snlopt.setHulls(hulls_curves);
  snlopt.setMaxRuntimeKappaAndMu(runtime, kappa, mu);

  snlopt.setInitStateFinalStateInitTFinalT(initial, final, t_min, t_max);

  return snlopt.checkGradientsUsingFiniteDiff();
}