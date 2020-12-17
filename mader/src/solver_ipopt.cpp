#include <casadi/casadi.hpp>

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
#include <fstream>

#include <ros/package.h>  //TODO: remove this ros dependency

using namespace termcolor;

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

struct SolverIpopt::PImpl
{
  casadi::Function casadi_function_;
  casadi::DM all_w_fe_;
};

SolverIpopt::SolverIpopt(mt::parameters &par)
{
  par_ = par;

  // All these values are for the position spline
  p_ = par_.deg_pos;
  M_ = par_.num_seg + 2 * p_;
  N_ = M_ - p_ - 1;

  Ny_ = (par_.num_seg + par_.deg_yaw - 1);
  // num_seg = (M_ - 2 * p_);  // this is the same as par_.num_seg
  ///////////////////////////////////////

  mt::basisConverter basis_converter;
  // basis used for collision
  if (par_.basis == "MINVO")
  {
    basis_ = MINVO;
    M_pos_bs2basis_ = basis_converter.getMinvoPosConverters(par_.num_seg);
    M_vel_bs2basis_ = basis_converter.getMinvoVelConverters(par_.num_seg);
  }
  else if (par_.basis == "BEZIER")
  {
    basis_ = BEZIER;
    M_pos_bs2basis_ = basis_converter.getBezierPosConverters(par_.num_seg);
    M_vel_bs2basis_ = basis_converter.getBezierVelConverters(par_.num_seg);
  }
  else if (par_.basis == "B_SPLINE")
  {
    basis_ = B_SPLINE;
    M_pos_bs2basis_ = basis_converter.getBSplinePosConverters(par_.num_seg);
    M_vel_bs2basis_ = basis_converter.getBSplineVelConverters(par_.num_seg);
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

  // hack

  std::fstream myfile(ros::package::getPath("mader") + "/matlab/index_instruction.txt",
                      std::ios_base::in);  // TODO: remove this ROS dependency
  // OTHER OPTION:    std::cout << bold << red << getPathName(__FILE__) << reset << std::endl;
  // getPathName() is defined above in this file
  myfile >> index_instruction_;
  std::cout << "index_instruction_= " << index_instruction_ << std::endl;
  /// end of hack

  m_casadi_ptr_ = std::unique_ptr<PImpl>(new PImpl());

  m_casadi_ptr_->casadi_function_ = casadi::Function::load(ros::package::getPath("mader") + "/matlab/"
                                                                                            "my_function.casadi");

  m_casadi_ptr_->all_w_fe_ = casadi::DM::rand(3, par_.num_samples_simpson);
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

void SolverIpopt::setSimpsonFeatureSamples(const std::vector<Eigen::Vector3d> &samples)
{
  assert(samples.size() == par_.num_samples_simpson);
  for (int i = 0; i < samples.size(); i++)
  {
    m_casadi_ptr_->all_w_fe_(0, i) = samples[i].x();
    m_casadi_ptr_->all_w_fe_(1, i) = samples[i].y();
    m_casadi_ptr_->all_w_fe_(2, i) = samples[i].z();
  }
}

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
  saturate(v0, -par_.v_max, par_.v_max);  // TODO: but this changes are not reflected in initial_state_
  saturate(a0, -par_.a_max, par_.a_max);
  saturate(vf, -par_.v_max, par_.v_max);
  saturate(af, -par_.a_max, par_.a_max);

  initial_state_ = initial_state;
  final_state_ = final_state;

  std::cout << "initial_state= " << std::endl;
  initial_state.printHorizontal();

  std::cout << "final_state= " << std::endl;
  final_state.printHorizontal();

  //////////////////////////////

  deltaT_ = (t_final - t_init) / (1.0 * (M_ - 2 * p_ - 1 + 1));

  //////////////////////////////
  // Now make sure deltaT in knots_ is such that -v_max<=v1<=v_max is satisfied:
  // std::cout << bold << "deltaT_ before= " << deltaT_ << reset << std::endl;
  for (int axis = 0; axis < 3; axis++)
  {
    double upper_bound, lower_bound;
    if (fabs(a0(axis)) > 1e-7)
    {
      upper_bound = ((p_ - 1) * (sgn(a0(axis)) * par_.v_max(axis) - v0(axis)) / (a0(axis)));
      lower_bound = ((p_ - 1) * (-sgn(a0(axis)) * par_.v_max(axis) - v0(axis)) / (a0(axis)));

      // std::cout << "axis= " << axis << std::endl;
      // std::cout << "lower_bound= " << lower_bound << std::endl;
      // std::cout << "upper_bound= " << upper_bound << std::endl;

      ////////////////// Just for debugging
      if (upper_bound < lower_bound)
      {
        std::cout << red << bold << "This should never happen, aborting" << std::endl;
        abort();
      }
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
    knots[i] = knots[i - 1] + deltaT_;  // Assumming a uniform b-spline (internal knots are equally spaced)
  }

  for (int i = (M_ - p_); i <= M_; i++)
  {
    knots[i] = t_final_;
  }

  knots_ = knots;
  //////////////////

  //////////////////

  // See https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
  // See also eq. 15 of the paper "Robust and Efficent quadrotor..."

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
  // note that, for a v0 and a0 given, q2_ is not guaranteed to lie within the bounds. If that's the case --> keep
  // executing previous trajectory
  if (q2_.x() > par_.x_max || q2_.x() < par_.x_min ||  //////////////
      q2_.y() > par_.y_max || q2_.y() < par_.y_min ||  /////////////////
      q2_.z() > par_.z_max || q2_.z() < par_.z_min)
  {
    std::cout << bold << red << "q2_ is not in [min, max]" << reset << std::endl;
    std::cout << "q2_= " << q2_.transpose() << std::endl;
    std::cout << "par_.x_min= " << par_.x_min << ", par_.x_max=" << par_.x_max << std::endl;
    std::cout << "par_.y_min= " << par_.y_min << ", par_.y_max=" << par_.y_max << std::endl;
    std::cout << "par_.z_min= " << par_.z_min << ", par_.z_max=" << par_.z_max << std::endl;
    return false;
  }
  bool guess_is_feasible = generateAStarGuess();  // I obtain q_quess_, n_guess_, d_guess_
  if (guess_is_feasible == false)
  {
    std::cout << "Planes haven't been found" << std::endl;
    return false;
  }
  n_ = n_guess_;
  d_ = d_guess_;

  ////////////////////////////////////
  ////////////////////////////////////
  //////////////////////////////////// CASADI!

  std::cout << "Casadi!" << std::endl;
  std::cout << "par_.v_max= " << par_.v_max.transpose() << std::endl;
  std::cout << "par_.a_max= " << par_.a_max.transpose() << std::endl;

  // Conversion DM <--> Eigen:  https://github.com/casadi/casadi/issues/2563
  auto eigen2std = [](Eigen::Vector3d &v) { return std::vector<double>{ v.x(), v.y(), v.z() }; };

  std::map<std::string, casadi::DM> map_arguments;
  map_arguments["thetax_FOV_deg"] = par_.fov_x_deg;
  map_arguments["thetay_FOV_deg"] = par_.fov_y_deg;
  map_arguments["p0"] = eigen2std(initial_state_.pos);
  map_arguments["v0"] = eigen2std(initial_state_.vel);
  map_arguments["a0"] = eigen2std(initial_state_.accel);
  map_arguments["pf"] = eigen2std(final_state_.pos);
  map_arguments["vf"] = eigen2std(final_state_.vel);
  map_arguments["af"] = eigen2std(final_state_.accel);
  map_arguments["y0"] = initial_state_.yaw;
  map_arguments["ydot0"] = initial_state_.dyaw;
  map_arguments["ydotf"] =
      final_state_.dyaw;  // Needed: if not (and if you are minimizing ddyaw), ddyaw=cte --> yaw will explode
  std::cout << bold << blue << "y0= " << initial_state_.yaw << reset << std::endl;
  std::cout << bold << blue << "ydot0= " << initial_state_.dyaw << reset << std::endl;
  map_arguments["v_max"] = eigen2std(par_.v_max);
  map_arguments["a_max"] = eigen2std(par_.a_max);
  map_arguments["ydot_max"] = par_.ydot_max;
  map_arguments["total_time"] = (t_final_ - t_init_);
  // all_w_fe is a matrix whose columns are the positions of the feature (in world frame) in the times [t0,t0+XX,
  // ...,tf-XX, tf] (i.e. uniformly distributed and including t0 and tf)
  map_arguments["all_w_fe"] = m_casadi_ptr_->all_w_fe_;
  map_arguments["c_jerk"] = par_.c_jerk;
  map_arguments["c_yaw"] = par_.c_yaw;
  map_arguments["c_vel_isInFOV"] = par_.c_vel_isInFOV;
  map_arguments["c_final_pos"] = par_.c_final_pos;

  std::cout << "Total time= " << (t_final_ - t_init_) << std::endl;

  int max_num_of_planes = par_.num_max_of_obst * par_.num_seg;

  assert((n_guess_.size() <= max_num_of_planes) && "the casadi function does not support so many planes");

  //  casadi::DM all_nd(casadi::Sparsity::dense(4, max_num_of_planes));  // TODO: do this just once

  casadi::DM all_nd(casadi::DM::zeros(4, max_num_of_planes));  // TODO: do this just once
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

  map_arguments["all_nd"] = all_nd;  // casadi::DM::rand(4, 0);

  ///////////////// GUESS FOR POSITION CONTROL POINTS
  casadi::DM matrix_qp_guess(casadi::Sparsity::dense(3, (N_ + 1)));  // TODO: do this just once
  for (int i = 0; i < matrix_qp_guess.columns(); i++)
  {
    matrix_qp_guess(0, i) = qp_guess_[i].x();
    matrix_qp_guess(1, i) = qp_guess_[i].y();
    matrix_qp_guess(2, i) = qp_guess_[i].z();
  }
  map_arguments["guess_CPs_Pos"] = matrix_qp_guess;

  ///////////////// GUESS FOR YAW CONTROL POINTS
  qy_guess_.clear();  // THIS SHOULD GO IN THE OCTOPUS SEARCH?
  for (int i = 0; i < (Ny_ + 1); i++)
  {
    qy_guess_.push_back(initial_state_.yaw);  // rand()
  }                                           // THIS SHOULD GO IN THE OCTOPUS SEARCH?

  std::cout << bold << red << "par_.num_seg= " << par_.num_seg << reset << std::endl;

  casadi::DM matrix_qy_guess(casadi::Sparsity::dense(1, (Ny_ + 1)));  // TODO: do this just once
  for (int i = 0; i < matrix_qy_guess.columns(); i++)
  {
    matrix_qy_guess(0, i) = qy_guess_[i];
  }
  map_arguments["guess_CPs_Yaw"] = matrix_qy_guess;

  ///////////////// CALL THE SOLVER
  std::map<std::string, casadi::DM> result = m_casadi_ptr_->casadi_function_(map_arguments);

  ///////////////// GET STATUS FROM THE SOLVER
  // Very hacky solution, see discussion at https://groups.google.com/g/casadi-users/c/1061E0eVAXM/m/dFHpw1CQBgAJ
  // Inspired from https://gist.github.com/jgillis/9d12df1994b6fea08eddd0a3f0b0737f
  auto optimstatus =
      m_casadi_ptr_->casadi_function_.instruction_MX(index_instruction_).which_function().stats(1)["return_status"];

  ///////////////// DECIDE ACCORDING TO STATUS OF THE SOLVER
  std::vector<Eigen::Vector3d> qp;  // Solution found (Control points for position)
  std::vector<double> qy;           // Solution found (Control points for yaw)
  std::cout << "optimstatus= " << optimstatus << std::endl;
  // See names here:
  // https://github.com/casadi/casadi/blob/fadc86444f3c7ab824dc3f2d91d4c0cfe7f9dad5/casadi/interfaces/ipopt/ipopt_interface.cpp
  if (optimstatus == "Solve_Succeeded" || optimstatus == "Solved_To_Acceptable_Level")
  {
    std::cout << green << "IPOPT found a solution!" << reset << std::endl;

    // copy the solution
    auto qp_casadi = result["all_pCPs"];
    for (int i = 0; i < qp_casadi.columns(); i++)
    {
      qp.push_back(Eigen::Vector3d(double(qp_casadi(0, i)), double(qp_casadi(1, i)), double(qp_casadi(2, i))));
    }

    qy = static_cast<std::vector<double>>(result["all_yCPs"]);
  }
  else
  {
    std::cout << red << "IPOPT failed to find a solution" << reset
              << std::endl;  //, using initial guess (which is feasible)
    qp = qp_guess_;
    qy = qy_guess_;  // TODO: I need to make sure that this is feasible as well!! (both in position, in velocity and in
                     // initial/final conditions). For now, let's return false until I fix this.
    return false;
  }

  ///////////////// PRINT SOLUTION
  std::cout << "Position control Points obtained= " << std::endl;
  printStd(qp);

  std::cout << "Yaw control Points obtained= " << std::endl;
  printStd(qy);

  ///////////////// Fill  traj_solution_ and pwp_solution_
  int param_pp = 3;
  int param_py = 2;
  std::cout << "qy.size()= " << qy.size() << std::endl;
  CPs2TrajAndPwp_cleaner(qp, qy, traj_solution_, pwp_solution_, param_pp, param_py, knots_, par_.dc);

  std::cout << "Called CPs2TrajAndPwp!" << std::endl;

  // Force last position =final_state_ (which it's not guaranteed because of the discretization with par_.dc)
  traj_solution_.back().vel = final_state_.vel;
  traj_solution_.back().accel = final_state_.accel;
  traj_solution_.back().jerk = Eigen::Vector3d::Zero();

  // std::cout << bold << red << "traj_solution_.[-2].accel= " << traj_solution_.end()[-2].accel.transpose() << reset
  //           << std::endl;

  // std::cout << bold << red << "traj_solution_.back().accel= " << traj_solution_.back().accel.transpose() << reset
  //           << std::endl;

  // Uncomment the following line if you wanna visualize the planes
  // fillPlanesFromNDQ(n_, d_, q);  // TODO: move this outside the SolverIpopt class

  return true;
}

void SolverIpopt::getSolution(mt::PieceWisePol &solution)
{
  solution = pwp_solution_;
}

//////////////////////////////////////////
//////////////////////////////////////////
////////////////////////////////////////// THIS SHOULD GO TO A DIFFERENT PLACE
//////////////////////////////////////////
//////////////////////////////////////////
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

      // Equation of the blue planes is n'x+d == -1
      // Convert here to equation [A B C]'x+D ==0
      double A = n[ip].x();
      double B = n[ip].y();
      double C = n[ip].z();
      double D = d[ip] + 1;

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