/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>

#include "mader.hpp"
#include "timer.hpp"
#include "termcolor.hpp"

// #include "nlopt_utils.hpp"

using namespace termcolor;

// Uncomment the type of timer you want:
// typedef ROSTimer MyTimer;
// typedef ROSWallTimer MyTimer;
typedef MADER_timers::Timer MyTimer;

Mader::Mader(mt::parameters par) : par_(par)
{
  drone_status_ == DroneStatus::YAWING;
  G_.pos << 0, 0, 0;
  G_term_.pos << 0, 0, 0;

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

  changeDroneStatus(DroneStatus::GOAL_REACHED);
  resetInitialization();

  mt::basisConverter basis_converter;

  if (par.basis == "MINVO")
  {
    A_basis_deg2_rest_ = basis_converter.getArestMinvoDeg2();
    A_basis_deg3_rest_ = basis_converter.getArestMinvoDeg3();
  }
  else if (par.basis == "BEZIER")
  {
    A_basis_deg2_rest_ = basis_converter.getArestBezierDeg2();
    A_basis_deg3_rest_ = basis_converter.getArestBezierDeg3();
  }
  else if (par.basis == "B_SPLINE")
  {
    A_basis_deg2_rest_ = basis_converter.getArestBSplineDeg2();
    A_basis_deg3_rest_ = basis_converter.getArestBSplineDeg3();
  }
  else
  {
    std::cout << red << "Basis " << par.basis << " not implemented yet" << reset << std::endl;
    std::cout << red << "============================================" << reset << std::endl;
    abort();
  }

  A_basis_deg2_rest_inverse_ = A_basis_deg2_rest_.inverse();
  A_basis_deg3_rest_inverse_ = A_basis_deg3_rest_.inverse();

  // solver_ = new SolverNlopt(par_for_solver);
  // solver_ = new SolverGurobi(par_for_solver);

  log_ptr_ = std::shared_ptr<mt::log>(new mt::log);

  solver_ = new SolverIpopt(par_, log_ptr_);

  separator_solver_ = new separator::Separator();
}

void Mader::dynTraj2dynTrajCompiled(const mt::dynTraj& traj, mt::dynTrajCompiled& traj_compiled)
{
  if (traj.use_pwp_field == true)
  {
    traj_compiled.pwp_mean = traj.pwp_mean;
    traj_compiled.pwp_var = traj.pwp_var;
    traj_compiled.is_static =
        ((traj.pwp_mean.eval(0.0) - traj.pwp_mean.eval(1e30)).norm() < 1e-5);  // TODO: Improve this
  }
  else
  {
    mtx_t_.lock();

    typedef exprtk::symbol_table<double> symbol_table_t;
    typedef exprtk::expression<double> expression_t;
    typedef exprtk::parser<double> parser_t;

    // Compile the mean
    for (auto function_i : traj.s_mean)
    {
      symbol_table_t symbol_table;
      symbol_table.add_variable("t", t_);
      symbol_table.add_constants();
      expression_t expression;
      expression.register_symbol_table(symbol_table);
      parser_t parser;
      parser.compile(function_i, expression);
      traj_compiled.s_mean.push_back(expression);
    }

    // Compile the variance
    for (auto function_i : traj.s_var)
    {
      symbol_table_t symbol_table;
      symbol_table.add_variable("t", t_);
      symbol_table.add_constants();
      expression_t expression;
      expression.register_symbol_table(symbol_table);
      parser_t parser;
      parser.compile(function_i, expression);
      traj_compiled.s_var.push_back(expression);
    }

    mtx_t_.unlock();

    traj_compiled.is_static =
        (traj.s_mean[0].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate x
        (traj.s_mean[1].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate y
        (traj.s_mean[2].find("t") == std::string::npos);    // there is no dependence on t in the coordinate z
  }

  traj_compiled.use_pwp_field = traj.use_pwp_field;
  traj_compiled.is_agent = traj.is_agent;
  traj_compiled.bbox = traj.bbox;
  traj_compiled.id = traj.id;
  traj_compiled.time_received = traj.time_received;  // ros::Time::now().toSec();
}

// Note that this function is here because I need t_ for this evaluation
Eigen::Vector3d Mader::evalMeanDynTrajCompiled(const mt::dynTrajCompiled& traj, double t)
{
  Eigen::Vector3d tmp;

  if (traj.use_pwp_field == true)
  {
    tmp = traj.pwp_mean.eval(t);
  }
  else
  {
    mtx_t_.lock();
    t_ = t;
    tmp << traj.s_mean[0].value(),  ////////////////////
        traj.s_mean[1].value(),     ////////////////
        traj.s_mean[2].value();     /////////////////

    mtx_t_.unlock();
  }
  return tmp;
}

// Note that this function is here because I need t_ for this evaluation
Eigen::Vector3d Mader::evalVarDynTrajCompiled(const mt::dynTrajCompiled& traj, double t)
{
  Eigen::Vector3d tmp;

  if (traj.use_pwp_field == true)
  {
    tmp = traj.pwp_var.eval(t);
  }
  else
  {
    mtx_t_.lock();
    t_ = t;
    tmp << traj.s_var[0].value(),  ////////////////////
        traj.s_var[1].value(),     ////////////////
        traj.s_var[2].value();     /////////////////

    mtx_t_.unlock();
  }
  return tmp;
}

void Mader::removeOldTrajectories()
{
  double time_now = ros::Time::now().toSec();
  std::vector<int> ids_to_remove;

  mtx_trajs_.lock();

  for (int index_traj = 0; index_traj < trajs_.size(); index_traj++)
  {
    if ((time_now - trajs_[index_traj].time_received) > par_.max_seconds_keeping_traj)
    {
      ids_to_remove.push_back(trajs_[index_traj].id);
    }
  }

  for (auto id : ids_to_remove)
  {
    // ROS_WARN_STREAM("Removing " << id);
    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](mt::dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  mtx_trajs_.unlock();
}

// Note that we need to compile the trajectories inside mader.cpp because t_ is in mader.hpp
void Mader::updateTrajObstacles(mt::dynTraj traj)
{
  MyTimer tmp_t(true);

  if (started_check_ == true && traj.is_agent == true)
  {
    have_received_trajectories_while_checking_ = true;
  }

  // std::cout << on_blue << bold << "in  updateTrajObstacles(), waiting to lock mtx_trajs_" << reset << std::endl;
  mtx_trajs_.lock();

  std::vector<mt::dynTrajCompiled>::iterator obs_ptr =
      std::find_if(trajs_.begin(), trajs_.end(),
                   [=](const mt::dynTrajCompiled& traj_compiled) { return traj_compiled.id == traj.id; });

  bool exists_in_local_map = (obs_ptr != std::end(trajs_));

  mt::dynTrajCompiled traj_compiled;
  dynTraj2dynTrajCompiled(traj, traj_compiled);

  if (exists_in_local_map)
  {  // if that object already exists, substitute its trajectory
    *obs_ptr = traj_compiled;
  }
  else
  {  // if it doesn't exist, add it to the local map
    trajs_.push_back(traj_compiled);
    // ROS_WARN_STREAM("Adding " << traj_compiled.id);
  }

  // and now let's delete those trajectories of the obs/agents whose current positions are outside the local map
  // Note that these positions are obtained with the trajectory stored in the past in the local map
  std::vector<int> ids_to_remove;

  double time_now = ros::Time::now().toSec();

  for (int index_traj = 0; index_traj < trajs_.size(); index_traj++)
  {
    bool traj_affects_me = false;

    Eigen::Vector3d center_obs = evalMeanDynTrajCompiled(trajs_[index_traj], time_now);

    // mtx_t_.unlock();
    if (((traj_compiled.is_static == true) && (center_obs - state_.pos).norm() > 2 * par_.Ra) ||  ////
        ((traj_compiled.is_static == false) && (center_obs - state_.pos).norm() > 4 * par_.Ra))
    // #### Static Obstacle: 2*Ra because: traj_{k-1} is inside a sphere of Ra.
    // Then, in iteration k the point A (which I don't
    // know yet)  is taken along that trajectory, and
    // another trajectory of radius Ra will be obtained.
    // Therefore, I need to take 2*Ra to make sure the
    // extreme case (A taken at the end of traj_{k-1} is
    // covered).

    // #### Dynamic Agent: 4*Ra. Same reasoning as above, but with two agets
    // #### Dynamic Obstacle: 4*Ra, it's a heuristics.

    // ######REMEMBER######
    // Note that removeTrajsThatWillNotAffectMe will later
    // on take care of deleting the ones I don't need once
    // I know A
    {
      ids_to_remove.push_back(trajs_[index_traj].id);
    }
  }

  for (auto id : ids_to_remove)
  {
    // ROS_WARN_STREAM("Removing " << id);
    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](mt::dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  mtx_trajs_.unlock();
  // std::cout << red << bold << "in updateTrajObstacles(), mtx_trajs_ unlocked" << reset << std::endl;

  have_received_trajectories_while_checking_ = false;
  // std::cout << bold << blue << "updateTrajObstacles took " << tmp_t << reset << std::endl;
}

std::vector<Eigen::Vector3d> Mader::vertexesOfInterval(mt::PieceWisePol& pwp, double t_start, double t_end,
                                                       const Eigen::Vector3d& delta)
{
  std::vector<Eigen::Vector3d> points;

  std::vector<double>::iterator low = std::lower_bound(pwp.times.begin(), pwp.times.end(), t_start);
  std::vector<double>::iterator up = std::upper_bound(pwp.times.begin(), pwp.times.end(), t_end);

  // Example: times=[1 2 3 4 5 6 7]
  // t_start=1.5;
  // t_end=5.5
  // then low points to "2" (low - pwp.times.begin() is 1)
  // and up points to "6" (up - pwp.times.begin() is 5)

  int index_first_interval = low - pwp.times.begin() - 1;  // index of the interval [1,2]
  int index_last_interval = up - pwp.times.begin() - 1;    // index of the interval [5,6]

  saturate(index_first_interval, 0, (int)(pwp.all_coeff_x.size() - 1));
  saturate(index_last_interval, 0, (int)(pwp.all_coeff_x.size() - 1));

  // push all the complete intervals
  for (int i = index_first_interval; i <= index_last_interval; i++)
  {
    Eigen::VectorXd coeff_x_scaled;
    Eigen::VectorXd coeff_y_scaled;
    Eigen::VectorXd coeff_z_scaled;

    if (i == index_first_interval && i != index_last_interval)
    {
      double u_t_start = pwp.t2u(t_start);

      changeDomPoly(pwp.all_coeff_x[i], u_t_start, 1.0, coeff_x_scaled, 0.0, 1.0);
      changeDomPoly(pwp.all_coeff_y[i], u_t_start, 1.0, coeff_y_scaled, 0.0, 1.0);
      changeDomPoly(pwp.all_coeff_z[i], u_t_start, 1.0, coeff_z_scaled, 0.0, 1.0);
      std::cout << "=====================================================" << std::endl;
      pwp.print();

      // std::cout << red << bold << "pwp.all_coeff_x[i]= " << pwp.all_coeff_x[i].transpose() << reset << std::endl;
      std::cout << red << bold << "t_start= " << t_start << reset << std::endl;
      std::cout << red << bold << "coeff_x_scaled= " << coeff_x_scaled.transpose() << reset << std::endl;
      std::cout << red << bold << "u_t_start= " << u_t_start << reset << std::endl;
    }
    else if (i == index_last_interval && i != index_first_interval)
    {
      double u_t_end = pwp.t2u(t_end);
      changeDomPoly(pwp.all_coeff_x[i], 0.0, u_t_end, coeff_x_scaled, 0.0, 1.0);
      changeDomPoly(pwp.all_coeff_y[i], 0.0, u_t_end, coeff_y_scaled, 0.0, 1.0);
      changeDomPoly(pwp.all_coeff_z[i], 0.0, u_t_end, coeff_z_scaled, 0.0, 1.0);
    }
    else if (i == index_first_interval && i == index_last_interval)  // happens where there is only one interval
    {
      double u_t_start = pwp.t2u(t_start);
      double u_t_end = pwp.t2u(t_end);
      changeDomPoly(pwp.all_coeff_x[i], u_t_start, u_t_end, coeff_x_scaled, 0.0, 1.0);
      changeDomPoly(pwp.all_coeff_y[i], u_t_start, u_t_end, coeff_y_scaled, 0.0, 1.0);
      changeDomPoly(pwp.all_coeff_z[i], u_t_start, u_t_end, coeff_z_scaled, 0.0, 1.0);
    }
    else
    {
      coeff_x_scaled = pwp.all_coeff_x[i];
      coeff_y_scaled = pwp.all_coeff_y[i];
      coeff_z_scaled = pwp.all_coeff_z[i];
    }

    int deg = pwp.getDeg();
    /// TODO
    if (deg > 3)
    {
      std::cout << bold << red << "The part below is assumming that degree<=3. You have deg=" << pwp.getDeg()
                << " Aborting" << reset << std::endl;
      abort();
    }
    ////////

    int tmp = coeff_x_scaled.size();

    Eigen::Matrix<double, 3, -1> P(3, deg + 1);
    Eigen::Matrix<double, 3, -1> V(3, deg + 1);  // 3 x number of vertexes of the simplex

    P.row(0) = coeff_x_scaled;
    P.row(1) = coeff_y_scaled;
    P.row(2) = coeff_z_scaled;

    // P = Eigen::Matrix<double, 3, 4>::Zero();
    // (P.row(0)).tail(tmp) = coeff_x_scaled;
    // (P.row(1)).tail(tmp) = coeff_y_scaled;
    // (P.row(2)).tail(tmp) = coeff_z_scaled;

    if (deg == 3)
    {
      V = P * A_basis_deg3_rest_inverse_;
    }
    else if (deg == 2)
    {
      // std::cout << on_blue << "Using Deg=2!!!" << reset << std::endl;
      V = P * A_basis_deg2_rest_inverse_;
    }
    else
    {
      // TODO
      std::cout << "Not implemented yet. Aborting" << std::endl;
      abort();
    }

    // std::cout << "P= \n" << P << std::endl;
    // std::cout << "V= \n" << V << std::endl;

    for (int j = 0; j < V.cols(); j++)
    {
      double x = V(0, j);
      double y = V(1, j);
      double z = V(2, j);  //[x,y,z] is the point

      if (delta.norm() < 1e-6)
      {  // no inflation
        std::cout << "No inflation" << std::endl;
        points.push_back(Eigen::Vector3d(x, y, z));
      }
      else
      {
        // points.push_back(Eigen::Vector3d(V(1, j), V(2, j), V(3, j)));  // x,y,z
        points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z + delta.z()));
        points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z + delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z + delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z + delta.z()));
      }
    }
  }

  return points;
}

// // return a vector that contains all the vertexes of the polyhedral approx of an interval.
std::vector<Eigen::Vector3d> Mader::vertexesOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  // every side of the box will be increased by 2*delta (+delta on one end, -delta on the other)
  // note that we use the variance at t_end (which is going to be worse that the one at t_start)
  Eigen::Vector3d delta = traj.bbox / 2.0 + (par_.drone_radius) * Eigen::Vector3d::Ones() +  //////
                          par_.norminv_prob * (evalVarDynTrajCompiled(traj, t_end)).cwiseSqrt();

  if (traj.use_pwp_field == false)
  {
    std::vector<Eigen::Vector3d> points;

    // Will always have a sample at the beginning of the interval, and another at the end.
    for (double t = t_start;                           /////////////
         (t < t_end) ||                                /////////////
         ((t > t_end) && ((t - t_end) < par_.gamma));  /////// This is to ensure we have a sample a the end
         t = t + par_.gamma)
    {
      Eigen::Vector3d tmp =
          evalMeanDynTrajCompiled(traj, std::min(t, t_end));  // this min only has effect on the last sample

      //"Minkowski sum along the trajectory: box centered on the trajectory"
      points.push_back(Eigen::Vector3d(tmp.x() + delta.x(), tmp.y() + delta.y(), tmp.z() + delta.z()));
      points.push_back(Eigen::Vector3d(tmp.x() + delta.x(), tmp.y() - delta.y(), tmp.z() - delta.z()));
      points.push_back(Eigen::Vector3d(tmp.x() + delta.x(), tmp.y() + delta.y(), tmp.z() - delta.z()));
      points.push_back(Eigen::Vector3d(tmp.x() + delta.x(), tmp.y() - delta.y(), tmp.z() + delta.z()));
      points.push_back(Eigen::Vector3d(tmp.x() - delta.x(), tmp.y() - delta.y(), tmp.z() - delta.z()));
      points.push_back(Eigen::Vector3d(tmp.x() - delta.x(), tmp.y() + delta.y(), tmp.z() + delta.z()));
      points.push_back(Eigen::Vector3d(tmp.x() - delta.x(), tmp.y() + delta.y(), tmp.z() - delta.z()));
      points.push_back(Eigen::Vector3d(tmp.x() - delta.x(), tmp.y() - delta.y(), tmp.z() + delta.z()));
    }

    return points;
  }
  else
  {
    return vertexesOfInterval(traj.pwp_mean, t_start, t_end, delta);
  }
}

// See https://doc.cgal.org/Manual/3.7/examples/Convex_hull_3/quickhull_3.cpp
CGAL_Polyhedron_3 Mader::convexHullOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  std::vector<Eigen::Vector3d> points = vertexesOfInterval(traj, t_start, t_end);

  std::vector<Point_3> points_cgal;
  for (auto point_i : points)
  {
    points_cgal.push_back(Point_3(point_i.x(), point_i.y(), point_i.z()));
  }

  return convexHullOfPoints(points_cgal);
}

// trajs_ is already locked when calling this function
void Mader::removeTrajsThatWillNotAffectMe(const mt::state& A, double t_start, double t_end)
{
  std::vector<int> ids_to_remove;

  for (auto traj : trajs_)
  {
    bool traj_affects_me = false;

    // STATIC OBSTACLES/AGENTS
    if (traj.is_static == true)
    {
      Eigen::Vector3d center_obs =
          evalMeanDynTrajCompiled(traj, t_start);  // Note that t_start is constant along the trajectory

      Eigen::Vector3d positive_half_diagonal;
      positive_half_diagonal << traj.bbox[0] / 2.0, traj.bbox[1] / 2.0, traj.bbox[2] / 2.0;

      Eigen::Vector3d c1 = center_obs - positive_half_diagonal;
      Eigen::Vector3d c2 = center_obs + positive_half_diagonal;
      traj_affects_me = boxIntersectsSphere(A.pos, par_.Ra, c1, c2);
    }
    else
    {                                                            // DYNAMIC OBSTACLES/AGENTS
      double deltaT = (t_end - t_start) / (1.0 * par_.num_seg);  // num_seg is the number of intervals
      for (int i = 0; i < par_.num_seg; i++)                     // for each interval
      {
        std::vector<Eigen::Vector3d> points =
            vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);

        for (auto point_i : points)  // for every vertex of each interval
        {
          if ((point_i - A.pos).norm() <= par_.Ra)
          {
            traj_affects_me = true;
            goto exit;
          }
        }
      }
    }

  exit:
    if (traj_affects_me == false)
    {
      // std::cout << red << bold << "Going to  delete traj " << trajs_[index_traj].id << reset << std::endl;
      ids_to_remove.push_back(traj.id);
    }
  }

  for (auto id : ids_to_remove)
  {
    // ROS_INFO_STREAM("traj " << id << " doesn't affect me");
    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](mt::dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  /*  std::cout << "After deleting the trajectory, we have these ids= " << std::endl;

    for (auto traj : trajs_)
    {
      std::cout << traj.id << std::endl;
    }*/
}

bool Mader::IsTranslating()
{
  return (drone_status_ == DroneStatus::GOAL_SEEN || drone_status_ == DroneStatus::TRAVELING);
}

ConvexHullsOfCurve Mader::convexHullsOfCurve(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  ConvexHullsOfCurve convexHulls;
  double deltaT = (t_end - t_start) / (1.0 * par_.num_seg);  // num_seg is the number of intervals

  for (int i = 0; i < par_.num_seg; i++)
  {
    convexHulls.push_back(convexHullOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT));
  }

  return convexHulls;
}

ConvexHullsOfCurves Mader::convexHullsOfCurves(double t_start, double t_end)
{
  ConvexHullsOfCurves result;

  for (auto traj : trajs_)
  {
    result.push_back(convexHullsOfCurve(traj, t_start, t_end));
  }

  return result;
}

// argmax_prob_collision is the index of trajectory I should focus on
// a negative value means that there are no trajectories to track
void Mader::sampleFeaturePosVel(int argmax_prob_collision, double t_start, double t_end,
                                std::vector<Eigen::Vector3d>& pos, std::vector<Eigen::Vector3d>& vel)
{
  pos.clear();
  vel.clear();

  // std::cout << red << bold << "in sampleFeaturePositions, waiting to lock mtx_trajs_" << reset << std::endl;
  // mtx_trajs_.lock(); //Already locked when this function is called
  // std::cout << red << bold << "in sampleFeaturePositions, waiting to lock t_" << reset << std::endl;
  // mtx_t_.lock();

  double delta = (t_end - t_start) / par_.num_samples_simpson;

  bool used_last_state_tracked = false;

  for (int i = 0; i < par_.num_samples_simpson; i++)
  {
    if (argmax_prob_collision >= 0)
    {
      double ti = t_start + i * delta;  // which is constant along the trajectory
      Eigen::Vector3d pos_i = evalMeanDynTrajCompiled(trajs_[argmax_prob_collision], ti);

      pos.push_back(pos_i);

      // MyTimer timer(true);
      // This commented part always returns 0.0. why??
      // See also
      // https://github.com/ArashPartow/exprtk/blob/66bed77369557fe1872df4c999c9d9ccb3adc3f6/readme.txt#LC4010:~:text=This%20free%20function%20will%20attempt%20to%20perform%20a%20numerical%20differentiation
      // Eigen::Vector3d vel_i = Eigen::Vector3d(exprtk::derivative(trajs_[wt].function[0], "t"),  ////////////
      //                                         exprtk::derivative(trajs_[wt].function[1], "t"),  ////////////
      //                                         exprtk::derivative(trajs_[wt].function[2], t_));
      // std::cout << "time to take derivatives= " << timer << std::endl;
      // std::cout << on_green << bold << "vel= " << vel_i.transpose() << reset << std::endl;

      // std::cout << on_green << bold << "pos= " << pos[i].transpose() << reset << std::endl;

      // Use finite differences to obtain the derivative
      double epsilon = 1e-6;

      Eigen::Vector3d pos_i_epsilon = evalMeanDynTrajCompiled(trajs_[argmax_prob_collision], ti + epsilon);

      vel.push_back((pos_i_epsilon - pos_i) / epsilon);

      // std::cout << bold << "Velocity= " << vel[i].transpose() << reset << std::endl;
      //////////////////////////////
    }
    else
    {
      pos.push_back(last_state_tracked_.pos);
      vel.push_back(last_state_tracked_.vel);
      used_last_state_tracked = true;
    }
  }

  if (used_last_state_tracked == true)
  {
    std::cout << bold << "There is no dynamic obstacle to track, using last_pos_tracked_" << reset << std::endl;
  }

  last_state_tracked_.pos = pos.back();
  last_state_tracked_.vel = vel.back();
  // mtx_t_.unlock();
  // std::cout << red << bold << "in sampleFeaturePositions, mtx_t_ unlocked" << reset << std::endl;
  // mtx_trajs_.unlock();
  // std::cout << red << bold << "in sampleFeaturePositions, mtx_trajs_ unlocked" << reset << std::endl;
}

void Mader::setTerminalGoal(mt::state& term_goal)
{
  if (state_initialized_ == false)  // because I need plan_size()>=1
  {
    std::cout << "[Mader::setTerminalGoal] State not initized yet, doing nothing" << std::endl;
  }

  std::cout << "Setting Terminal Goal" << std::endl;
  mtx_G_term.lock();
  mtx_state.lock();
  mtx_planner_status_.lock();

  G_term_.pos = term_goal.pos;
  Eigen::Vector3d temp = state_.pos;
  G_.pos = G_term_.pos;
  if (drone_status_ == DroneStatus::GOAL_REACHED)
  {
    /////////////////////////////////
    /////////////////////////////////
    /////////////////////////////////
    mtx_plan_.lock();  // must be before changeDroneStatus

    changeDroneStatus(DroneStatus::YAWING);
    mt::state last_state = plan_.back();

    double desired_yaw = atan2(G_term_.pos[1] - last_state.pos[1], G_term_.pos[0] - last_state.pos[0]);
    double diff = desired_yaw - last_state.yaw;
    angle_wrap(diff);

    double dyaw = copysign(1, diff) * par_.ydot_max;

    int num_of_el = (int)fabs(diff / (par_.dc * dyaw));

    assert((plan_.size() >= 1) && "plan_.size() must be >=1");

    // std::cout << "num_of_el= " << num_of_el << std::endl;
    // std::cout << "diff= " << diff << std::endl;
    // std::cout << "par_.ydot_max= " << par_.ydot_max << std::endl;
    // std::cout << "par_.dc= " << par_.dc << std::endl;

    for (int i = 1; i < (num_of_el + 1); i++)
    {
      // std::cout << "Introducing Element " << i << " out of " << num_of_el << std::endl;
      mt::state state_i = plan_.get(i - 1);
      state_i.yaw = state_i.yaw + dyaw * par_.dc;
      if (i == num_of_el)
      {
        state_i.dyaw = 0;  // 0 final yaw velocity
      }
      else
      {
        state_i.dyaw = dyaw;
      }
      plan_.push_back(state_i);
    }
    mtx_plan_.unlock();
    // abort();
    /////////////////////////////////
    /////////////////////////////////
    /////////////////////////////////
  }
  if (drone_status_ == DroneStatus::GOAL_SEEN)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }
  terminal_goal_initialized_ = true;

  // std::cout << bold << red << "[FA] Received Term Goal=" << G_term_.pos.transpose() << reset << std::endl;
  // std::cout << bold << red << "[FA] Received Proj Goal=" << G_.pos.transpose() << reset << std::endl;

  mtx_state.unlock();
  mtx_G_term.unlock();
  mtx_planner_status_.unlock();
}

void Mader::getG(mt::state& G)
{
  G = G_;
}

void Mader::getState(mt::state& data)
{
  mtx_state.lock();
  data = state_;
  mtx_state.unlock();
}

void Mader::updateState(mt::state data)
{
  state_ = data;

  if (state_initialized_ == false)
  {
    plan_.clear();  // just in case, (actually not needed because done in resetInitialization()
    mt::state tmp;
    tmp.pos = data.pos;
    tmp.yaw = data.yaw;
    plan_.push_back(tmp);
  }

  state_initialized_ = true;
}

bool Mader::initializedAllExceptPlanner()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

bool Mader::initializedStateAndTermGoal()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    return false;
  }
  return true;
}

bool Mader::initialized()
{
  if (!state_initialized_ || !terminal_goal_initialized_ || !planner_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;
        std::cout << "planner_initialized_= " << planner_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

// check wheter a mt::dynTrajCompiled and a pwp_optimized are in collision in the interval [t_start, t_end]
bool Mader::trajsAndPwpAreInCollision(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized, double t_start,
                                      double t_end)
{
  Eigen::Vector3d n_i;
  double d_i;

  double deltaT = (t_end - t_start) / (1.0 * par_.num_seg);  // num_seg is the number of intervals
  for (int i = 0; i < par_.num_seg; i++)                     // for each interval
  {
    // This is my trajectory (no inflation)
    std::vector<Eigen::Vector3d> pointsA =
        vertexesOfInterval(pwp_optimized, t_start + i * deltaT, t_start + (i + 1) * deltaT, Eigen::Vector3d::Zero());

    // This is the trajectory of the other agent/obstacle
    std::vector<Eigen::Vector3d> pointsB = vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);

    // std::cout << "Going to solve model with pointsA.size()= " << pointsA.size() << std::endl;
    // for (auto point_i : pointsA)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    // std::cout << "Going to solve model with pointsB.size()= " << pointsB.size() << std::endl;
    // for (auto point_i : pointsB)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    if (separator_solver_->solveModel(n_i, d_i, pointsA, pointsB) == false)
    {
      return true;  // There is not a solution --> they collide
    }
  }

  // if reached this point, they don't collide
  return false;
}
// Checks that I have not received new trajectories that affect me while doing the optimization
bool Mader::safetyCheckAfterOpt(mt::PieceWisePol pwp_optimized)
{
  started_check_ = true;

  bool result = true;
  for (auto traj : trajs_)
  {
    if (traj.time_received > time_init_opt_ && traj.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj, pwp_optimized, pwp_optimized.times.front(), pwp_optimized.times.back()))
      {
        ROS_ERROR_STREAM("Traj collides with " << traj.id);
        result = false;  // will have to redo the optimization
        break;
      }
    }
  }

  // and now do another check in case I've received anything while I was checking. Note that mtx_trajs_ is locked!
  if (have_received_trajectories_while_checking_ == true)
  {
    ROS_ERROR_STREAM("Recvd traj while checking ");
    result = false;
  }
  started_check_ = false;

  return result;
}

bool Mader::isReplanningNeeded()
{
  if (initializedStateAndTermGoal() == false)
  {
    // std::cout << "Not Replanning" << std::endl;
    return false;  // Note that log is not modified --> will keep its default values
  }

  //////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  // mtx_state.lock();
  mtx_G_term.lock();

  // mt::state state_local = state_;
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal

  mtx_G_term.unlock();
  // mtx_state.unlock();

  // Check if we have reached the goal
  double dist_to_goal = (G_term.pos - plan_.front().pos).norm();
  // std::cout << "dist_to_goal= " << dist_to_goal << std::endl;
  if (dist_to_goal < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
    exists_previous_pwp_ = false;
  }

  // Check if we have seen the goal in the last replan
  mtx_plan_.lock();
  double dist_last_plan_to_goal = (G_term.pos - plan_.back().pos).norm();
  // std::cout << "dist_last_plan_to_goal= " << dist_last_plan_to_goal << std::endl;
  mtx_plan_.unlock();
  if (dist_last_plan_to_goal < par_.goal_radius && drone_status_ == DroneStatus::TRAVELING)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
    std::cout << "Status changed to GOAL_SEEN!" << std::endl;
    exists_previous_pwp_ = false;
  }

  // Don't plan if drone is not traveling
  if (drone_status_ == DroneStatus::GOAL_REACHED || (drone_status_ == DroneStatus::YAWING) ||
      (drone_status_ == DroneStatus::GOAL_SEEN))
  {
    // std::cout << "No replanning needed because" << std::endl;
    // printDroneStatus();
    return false;
  }
}

bool Mader::replan(mt::Edges& edges_obstacles_out, std::vector<mt::state>& X_safe_out,
                   std::vector<Hyperplane3D>& planes, int& num_of_LPs_run, int& num_of_QCQPs_run,
                   mt::PieceWisePol& pwp_out, mt::log& log)
{
  (*log_ptr_) = {};  // Reset the struct with the default values
  if (isReplanningNeeded() == false)
  {
    log_ptr_->replanning_was_needed = false;
    log = (*log_ptr_);
    return false;
  }

  std::cout << bold << on_white << "**********************IN REPLAN CB*******************" << reset << std::endl;

  log_ptr_->replanning_was_needed = true;
  log_ptr_->tim_total_replan.tic();

  removeOldTrajectories();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select mt::state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_G_term.lock();
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  mt::state A;
  int k_index_end, k_index;

  // If k_index_end=0, then A = plan_.back() = plan_[plan_.size() - 1]

  mtx_plan_.lock();

  saturate(deltaT_, par_.lower_bound_runtime_snlopt / par_.dc, par_.upper_bound_runtime_snlopt / par_.dc);

  k_index_end = std::max((int)(plan_.size() - deltaT_), 0);

  if (plan_.size() < 5)
  {
    k_index_end = 0;
  }

  k_index = plan_.size() - 1 - k_index_end;
  A = plan_.get(k_index);

  mtx_plan_.unlock();

  // std::cout << blue << "k_index:" << k_index << reset << std::endl;
  // std::cout << blue << "k_index_end:" << k_index_end << reset << std::endl;
  // std::cout << blue << "plan_.size():" << plan_.size() << reset << std::endl;

  double runtime_snlopt;

  if (k_index_end != 0)
  {
    runtime_snlopt = k_index * par_.dc;  // std::min(, par_.upper_bound_runtime_snlopt);
  }
  else
  {
    runtime_snlopt = par_.upper_bound_runtime_snlopt;  // I'm stopped at the end of the trajectory --> take my
                                                       // time to replan
  }
  saturate(runtime_snlopt, par_.lower_bound_runtime_snlopt, par_.upper_bound_runtime_snlopt);

  // std::cout << green << "Runtime snlopt= " << runtime_snlopt << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get point G ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  double distA2TermGoal = (G_term.pos - A.pos).norm();
  double ra = std::min((distA2TermGoal - 0.001), par_.Ra);  // radius of the sphere S
  mt::state G;
  G.pos = A.pos + ra * (G_term.pos - A.pos).normalized();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Set Times in optimization! ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  solver_->setMaxRuntimeKappaAndMu(runtime_snlopt, par_.kappa, par_.mu);

  //////////////////////
  double time_now = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  double t_start = k_index * par_.dc + time_now;

  double factor_alloc_tmp = par_.factor_alloc;

  // std::cout << "distA2TermGoal= " << distA2TermGoal << std::endl;

  //// when it's near the terminal goal --> force the final condition (if not it may oscillate)
  // if (distA2TermGoal < par_.distance_to_force_final_pos)
  // {
  //   std::cout << "distA2TermGoal= " << distA2TermGoal << std::endl;
  //   std::cout << bold << blue << "Forcing final Pos" << reset << std::endl;

  //   factor_alloc_tmp = par_.factor_alloc_when_forcing_final_pos;
  //   solver_->par_.c_final_pos = 0.0;  // Is a constraint --> don't care about this cost
  //   solver_->par_.c_fov = 5.0;
  //   solver_->par_.force_final_pos = true;
  // }
  // else
  // {
  solver_->par_.c_final_pos = par_.c_final_pos;
  solver_->par_.c_fov = par_.c_fov;
  solver_->par_.force_final_pos = false;  // par_.force_final_pos;
  // }

  double time_allocated = getMinTimeDoubleIntegrator3D(A.pos, A.vel, G.pos, G.vel, par_.v_max, par_.a_max);

  // double t_final = t_start + (A.pos - G.pos).array().abs().maxCoeff() /
  //                                (factor_alloc_tmp * par_.v_max.x());  // time to execute the optimized path

  // std::cout << red << bold << "TIME ALLOCATED= " << time_allocated << reset << std::endl;

  double t_final = t_start + factor_alloc_tmp * time_allocated;

  bool correctInitialCond =
      solver_->setInitStateFinalStateInitTFinalT(A, G, t_start,
                                                 t_final);  // note that here t_final may have been updated

  if (correctInitialCond == false)
  {
    logAndTimeReplan("Solver cannot guarantee feasibility for v1", false, log);
    return false;
  }
  /////////////////////////////////////////////////////////////////////////
  ////////////////////////Compute trajectory to focus on //////////////////
  /////////////////////////////////////////////////////////////////////////

  double max_prob_collision = -std::numeric_limits<double>::max();  // it's actually a heuristics of the probability (we
                                                                    // are summing below --> can be >1)
  int argmax_prob_collision = -1;  // will contain the index of the trajectory to focus on

  int num_samplesp1 = 20;
  double delta = 1.0 / num_samplesp1;
  Eigen::Vector3d R = par_.drone_radius * Eigen::Vector3d::Ones();

  mtx_trajs_.lock();
  for (int i = 0; i < trajs_.size(); i++)
  {
    double prob_i = 0.0;
    for (int j = 0; j <= num_samplesp1; j++)
    {
      double t = t_start + j * delta * (t_final - t_start);

      Eigen::Vector3d pos_drone = A.pos + j * delta * (G_term_.pos - A.pos);  // not a random variable
      Eigen::Vector3d pos_obs_mean = evalMeanDynTrajCompiled(trajs_[i], t);
      Eigen::Vector3d pos_obs_std = (evalVarDynTrajCompiled(trajs_[i], t)).cwiseSqrt();
      // std::cout << "pos_obs_std= " << pos_obs_std << std::endl;
      prob_i += probMultivariateNormalDist(-R, R, pos_obs_mean - pos_drone, pos_obs_std);
    }

    std::cout << "Trajectory " << i << " has P(collision)= " << prob_i * pow(10, 15) << "e-15" << std::endl;

    if (prob_i > max_prob_collision)
    {
      max_prob_collision = prob_i;
      argmax_prob_collision = i;
    }
  }

  std::vector<Eigen::Vector3d> w_posfeature;      // velocity of the feature expressed in w
  std::vector<Eigen::Vector3d> w_velfeaturewrtw;  // velocity of the feature wrt w, expressed in w
  sampleFeaturePosVel(argmax_prob_collision, t_start, t_final, w_posfeature,
                      w_velfeaturewrtw);  // need to do it here so that argmax_prob_collision does not become invalid
                                          // with new updates
  std::cout << bold << "Chosen Trajectory " << argmax_prob_collision << reset << std::endl;

  mtx_trajs_.unlock();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve optimization! ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  // std::cout << red << bold << "in replan(), waiting to lock mtx_trajs_" << reset << std::endl;
  mtx_trajs_.lock();

  time_init_opt_ = ros::Time::now().toSec();
  // removeTrajsThatWillNotAffectMe(A, t_start, t_final); //TODO: Commented (4-Feb-2021)
  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_start, t_final);
  mtx_trajs_.unlock();
  // std::cout << red << bold << "in replan(), mtx_trajs_ unlocked" << reset << std::endl;

  ConvexHullsOfCurves_Std hulls_std = vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = vectorGCALPol2vectorJPSPol(hulls);
  edges_obstacles_out = vectorGCALPol2edges(hulls);

  solver_->setHulls(hulls_std);

  solver_->setSimpsonFeatureSamples(w_posfeature, w_velfeaturewrtw);

  //////////////////////
  std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
            << std::endl;

  std::cout << "[FA] Calling NL" << std::endl;

  bool result = solver_->optimize();

  num_of_LPs_run = solver_->getNumOfLPsRun();
  num_of_QCQPs_run = solver_->getNumOfQCQPsRun();

  total_replannings_++;
  if (result == false)
  {
    logAndTimeReplan("Solver failed", false, log);
    return false;
  }

  solver_->getPlanes(planes);

  solutions_found_++;

  mt::PieceWisePol pwp_now;
  solver_->getSolution(pwp_now);

  MyTimer check_t(true);

  mtx_trajs_.lock();
  bool is_safe_after_opt = safetyCheckAfterOpt(pwp_now);
  mtx_trajs_.unlock();

  if (is_safe_after_opt == false)
  {
    logAndTimeReplan("SafetyCheckAfterOpt not satisfied", false, log);
    return false;
  }

  M_ = G_term;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Append to plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  mtx_plan_.lock();

  int plan_size = plan_.size();

  if ((plan_size - 1 - k_index_end) < 0)
  {
    // std::cout << "plan_size= " << plan_size << std::endl;
    // std::cout << "k_index_end= " << k_index_end << std::endl;
    mtx_plan_.unlock();
    log_ptr_->info_replan = "Point A already published";
    logAndTimeReplan("Point A already published", false, log);
    return false;
  }
  else
  {
    // std::cout << "Appending" << std::endl;
    // std::cout << "before, plan_size=" << plan_.size() << std::endl;
    plan_.erase(plan_.end() - k_index_end - 1, plan_.end());  // this deletes also the initial condition...
    // std::cout << "middle, plan_size=" << plan_.size() << " sol.size()=" << (solver_->traj_solution_).size()
    // << std::endl;
    for (int i = 0; i < (solver_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      plan_.push_back(solver_->traj_solution_[i]);
    }
    // std::cout << "after, plan_size=" << plan_.size() << std::endl;
  }

  mtx_plan_.unlock();

  ////////////////////
  ////////////////////

  if (exists_previous_pwp_ == true)
  {
    pwp_out = composePieceWisePol(time_now, par_.dc, pwp_prev_, pwp_now);
    pwp_prev_ = pwp_out;
  }
  else
  {  //
    pwp_out = pwp_now;
    pwp_prev_ = pwp_now;
    exists_previous_pwp_ = true;
  }

  X_safe_out = plan_.toStdVector();

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  // Check if we have planned until G_term
  // mt::state F = plan_.back();  // Final point of the safe path (\equiv final point of the comitted path)
  double dist = (G_term_.pos - plan_.back().pos).norm();

  if (dist < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
  }

  planner_initialized_ = true;

  logAndTimeReplan("Success", true, log);
  return true;
}

void Mader::logAndTimeReplan(const std::string& info, const bool& success, mt::log& log)
{
  log_ptr_->info_replan = info;
  log_ptr_->tim_total_replan.toc();
  log_ptr_->success_replanning = success;

  double total_time_ms = log_ptr_->tim_total_replan.getMsSaved();

  mtx_offsets.lock();
  if (success == false)
  {
    std::cout << bold << red << log_ptr_->info_replan << reset << std::endl;
    int states_last_replan = ceil(total_time_ms / (par_.dc * 1000));  // Number of states that
                                                                      // would have been needed for
                                                                      // the last replan
    deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
    deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
  }
  else
  {
    int states_last_replan = ceil(total_time_ms / (par_.dc * 1000));  // Number of states that
                                                                      // would have been needed for
                                                                      // the last replan
    deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
  }
  mtx_offsets.unlock();

  log = (*log_ptr_);
}

void Mader::resetInitialization()
{
  planner_initialized_ = false;
  state_initialized_ = false;

  terminal_goal_initialized_ = false;
  plan_.clear();
}

bool Mader::getNextGoal(mt::state& next_goal)
{
  if (initializedStateAndTermGoal() == false)  // || (drone_status_ == DroneStatus::GOAL_REACHED && plan_.size() == 1))
                                               // TODO: if included this part commented out, the last state (which is
                                               // the one that has zero accel) will never get published
  {
    // std::cout << "Not publishing new goal!!" << std::endl;
    // std::cout << "plan_.size() ==" << plan_.size() << std::endl;
    // std::cout << "plan_.content[0] ==" << std::endl;
    // plan_.content[0].print();
    return false;
  }

  mtx_goals.lock();
  mtx_plan_.lock();

  next_goal.setZero();
  next_goal = plan_.front();

  if (plan_.size() > 1)
  {
    plan_.pop_front();
  }

  if (plan_.size() == 1 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }

  mtx_goals.unlock();
  mtx_plan_.unlock();
  return true;
}

// Debugging functions
void Mader::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
  {
    return;
  }

  std::cout << "Changing DroneStatus from ";
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "GOAL_REACHED" << reset;
      break;
  }
  std::cout << " to ";

  switch (new_status)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "GOAL_REACHED" << reset;
      break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

void Mader::printDroneStatus()
{
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset << std::endl;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset << std::endl;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset << std::endl;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset << std::endl;
      break;
  }
}
