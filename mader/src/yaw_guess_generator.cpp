#include <casadi/casadi.hpp>
#include "solver_ipopt.hpp"
#include "termcolor.hpp"

//
//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//

// Difference astar_search vs astar_search_tree:
// https://stackoverflow.com/questions/16201095/boost-graph-library-a-for-consistent-heuristic

// Directed vs undirected: https://www.boost.org/doc/libs/1_54_0/libs/graph/doc/adjacency_list.html

// When there are several
// goals:http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#multiple-goals:~:text=If%20you%20want%20to%20search%20for%20any%20of%20several%20goals%2C%20construct

// #include <boost/graph/graphviz.hpp>
#include <ctime>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>  // for sqrt

#include <ros/package.h>  //TODO: remove this ros dependency

using namespace boost;
using namespace std;
using namespace termcolor;

////////////////////////////////////////
////////////////////////////////////////
// euclidean distance heuristic
template <class Graph, class CostType>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor vd;

  distance_heuristic()
  {
  }
  CostType operator()(vd u)
  {
    // CostType dx = locations_[goal_].x - locations_[u].x;
    // CostType dy = locations_[goal_].y - locations_[u].y;
    CostType heuristic = 0;
    return heuristic;
  }

private:
};
////////////////////////////////////////
////////////////////////////////////////
template <class vd>
class found_goal
{
public:
  found_goal(vd g) : goal_found_(g)
  {
  }

  vd get_goal_found()
  {
    return goal_found_;
  }

private:
  vd goal_found_;
};  // exception for termination

////////////////////////////////////////
////////////////////////////////////////
// visitor that terminates when we find the goal
template <class vd>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(size_t index_goal_layer) : index_goal_layer_(index_goal_layer)
  {
  }
  template <class Graph>
  void examine_vertex(vd u, Graph& g)
  {
    if (g[u].layer == index_goal_layer_)
    {
      throw found_goal<vd>(u);
    }
  }

private:
  size_t index_goal_layer_;
};
////////////////////////////////////////
////////////////////////////////////////

// https://stackoverflow.com/a/11498248/6057617
double wrapFromMPitoPi(double x)
{
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0)
    x += 2 * M_PI;
  return x - M_PI;
}

casadi::DM SolverIpopt::generateYawGuess(casadi::DM matrix_qp_guess, casadi::DM all_w_fe, double y0, double ydot0,
                                         double ydotf, double t0, double tf)
{
  WeightMap weightmap = get(boost::edge_weight, mygraph_);

  std::map<std::string, casadi::DM> map_arg;
  map_arg["thetax_FOV_deg"] = par_.fov_x_deg;
  map_arg["thetay_FOV_deg"] = par_.fov_y_deg;
  map_arg["b_T_c"] = b_Tmatrixcasadi_c_;
  map_arg["all_w_fe"] = all_w_fe;
  map_arg["guess_CPs_Pos"] = matrix_qp_guess;
  map_arg["yaw_samples"] = vector_yaw_samples_;

  std::map<std::string, casadi::DM> result = casadi_visibility_function_(map_arg);
  casadi::DM vis_matrix_casadi = result["result"];  // Its values are in [0.1]
                                                    // It's a matrix of size (num_of_layers_)x(num_of_yaw_per_layer_)
                                                    // we won't use its 1st col  (since y0 is given)

  double y0wrapped = wrapFromMPitoPi(y0);
  std::cout << bold << yellow << "y0wrapped= " << y0wrapped << reset << std::endl;
  // Set the value of the first node (initial yaw)
  mygraph_[all_vertexes_[0][0]].yaw = y0wrapped;

  double deltaT = (tf - t0) / (double(num_of_layers_));

  //////////////////////// Iterate through all the edges of the graph and add the cost
  auto es = boost::edges(mygraph_);
  for (auto ed_ptr = es.first; ed_ptr != es.second; ++ed_ptr)  // ed_ptr is edge descriptor pointer
  {
    vd index_vertex1 = boost::source(*ed_ptr, mygraph_);
    vd index_vertex2 = boost::target(*ed_ptr, mygraph_);

    // std::cout << boost::source(*ed_ptr, mygraph_) << ' ' << boost::target(*ed_ptr, mygraph_) << std::endl;

    double visibility = double(vis_matrix_casadi(
        mygraph_[index_vertex2].layer, mygraph_[index_vertex2].circle));  // \in [0,1]
                                                                          // Note that vis_matrix_casadi has in the rows
                                                                          // the circles, in the columns the layers.

    // TODO: the distance cost is fixed (don't change in each iteration --> add it only once at the beginning?)
    double distance = abs(wrapFromMPitoPi(mygraph_[index_vertex1].yaw - mygraph_[index_vertex2].yaw));
    double distance_squared = pow(distance, 2.0);

    weightmap[*ed_ptr] = par_.c_smooth_yaw_search * distance_squared - par_.c_visibility_yaw_search * visibility + 1.0 +
                         ((distance / deltaT) > par_.ydot_max) * 1e6;  //+1.0 to ensure it's >=0

    // if it doesn't satisfy the  ydot_maxconstraint --> very expensive edge. Note that with this option (instead of the
    // option of NOT creating an edge) there will always be a solution in the graph

    // std::cout << "edge between [" << j << ", " << i << "] and [" << j_next << ", " << i + 1
    //           << "] with cost= " << weightmap[e] << std::endl;
  }
  ////////////////////////

  // std::cout << bold << yellow << "num_edges(mygraph_)= " << num_edges(mygraph_) << reset << std::endl;

  ////DEBUGGING
  // if (num_edges(mygraph_) < (num_of_layers_ - 1))
  // {
  //   std::cout << red << bold << "The layers are disconnected for sure, no solution will be found" << reset <<
  //   std::endl; std::cout << red << bold << "Maybe ydot_max is too small?" << std::endl; abort();
  // }
  ////END OF DEBUGGING

  vd start = all_vertexes_[0][0];

  vector<vd> p(num_vertices(mygraph_));
  vector<cost_graph> d(num_vertices(mygraph_));
  try
  {
    // call astar named parameter interface
    astar_search_tree(mygraph_, start, distance_heuristic<mygraph_t, cost_graph>(),
                      predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, mygraph_)))
                          .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, mygraph_)))
                          .visitor(astar_goal_visitor<vd>(num_of_layers_ - 1)));
  }

  catch (found_goal<vd> fg)
  {  // found a path to the goal
    std::list<vd> shortest_path_vd;
    for (vd v = fg.get_goal_found();; v = p[v])
    {
      shortest_path_vd.push_front(v);
      if (p[v] == v)
        break;
    }
    std::list<vd>::iterator spi = shortest_path_vd.begin();
    // std::cout << mygraph_[all_vertexes_[0][0]].yaw;

    casadi::DM vector_shortest_path(1, shortest_path_vd.size());  // TODO: do this just once?

    vector_shortest_path(0) = mygraph_[*spi].yaw;

    int i = 1;

    for (++spi; spi != shortest_path_vd.end(); ++spi)
    {
      // double yaw_tmp = mygraph_[*spi].yaw;
      // std::cout << " -> " << yaw_tmp;
      vector_shortest_path(i) = mygraph_[*spi].yaw;
      i = i + 1;
    }

    // std::cout << "vector_yaw_samples_=\n" << vector_yaw_samples_ << std::endl;
    // std::cout << "vis_matrix_casadi=\n" << vis_matrix_casadi << std::endl;

    for (int j = 0; j < vector_yaw_samples_.numel(); j++)
    {
      std::cout << right << std::fixed << std::setw(8) << std::setfill(' ') << "[" << j << "]" << reset;
    }
    std::cout << std::endl;
    for (int j = 0; j < vector_yaw_samples_.numel(); j++)
    {
      std::cout << right << std::fixed << std::setw(8) << std::setfill(' ') << blue << vector_yaw_samples_(j) << reset;
    }
    std::cout << std::endl;

    for (int i = 0; i < vis_matrix_casadi.rows(); i++)
    {
      std::cout << "[" << i << "] ";
      for (int j = 0; j < vis_matrix_casadi.columns(); j++)
      {
        {
          std::cout << right << std::fixed << std::setw(8) << std::setfill(' ');
          if (abs(double(vector_yaw_samples_(j)) - double(vector_shortest_path(i))) < 1e-5)
          {
            std::cout << "\033[0;31m";
          }
          else
          {
            // std::cout << left << std::fixed << std::setw(8) << std::setfill(' ') << vis_matrix_casadi(i, j) << reset;
          }

          std::cout << vis_matrix_casadi(i, j) << reset;
        }
      }
      std::cout << std::endl;
    }

    std::cout << "Shortest path: ";
    for (int i = 0; i < vector_shortest_path.numel(); i++)
    {
      std::cout << yellow << vector_shortest_path(i);
      if (i != (vector_shortest_path.numel() - 1))
      {
        std::cout << " --> ";
      }
      std::cout << reset;
    }
    cout << endl << "\nTotal cost: " << d[fg.get_goal_found()] << endl;

    ////////////////////////////////////////
    // Now fit a spline to the yaws found
    ////////////////////////////////////////

    // First correct the angles so that the max absolute difference between two adjacent elements is <=pi

    // See "fit_to_angular_data.m"
    casadi::DM vsp_corrected = vector_shortest_path;
    // vector_shortest_path(casadi::Sparsity::dense(1, vector_shortest_path.columns()));  // TODO: do this just once

    vsp_corrected(0) = vector_shortest_path(0);
    for (size_t i = 1; i < vsp_corrected.columns(); i++)  // starts in 1, not in 0
    {
      double previous_phi = double(vsp_corrected(i - 1));
      double phi_i = double(vsp_corrected(i));
      double difference = previous_phi - phi_i;

      double phi_i_f = phi_i + floor(difference / (2 * M_PI)) * 2 * M_PI;
      double phi_i_c = phi_i + ceil(difference / (2 * M_PI)) * 2 * M_PI;

      if (fabs(previous_phi - phi_i_f) < fabs(previous_phi - phi_i_c))
      {
        vsp_corrected(i) = phi_i_f;
      }
      else
      {
        vsp_corrected(i) = phi_i_c;
      }
    }

    ////////////////ONLY FOR DEBUGGING
    for (size_t i = 1; i < vsp_corrected.columns(); i++)
    {
      double phi_mi = double(vsp_corrected(i - 1));
      double phi_i = double(vsp_corrected(i));

      if (fabs(phi_i - phi_mi) > M_PI)
      {
        std::cout << red << bold << "This diff must be <= pi" << reset << std::endl;
        abort();
      }

      // assert(fabs(phi_i - phi_mi) <= M_PI && "This diff must be <= pi");
    }
    ////////////////END OF ONLY FOR DEBUGGING

    std::map<std::string, casadi::DM> map_arg2;
    map_arg2["all_yaw"] = vsp_corrected;
    map_arg2["y0"] = y0;
    map_arg2["ydot0"] = ydot0;
    map_arg2["ydotf"] = ydotf;
    std::map<std::string, casadi::DM> result2 = casadi_fit_yaw_function_(map_arg2);
    casadi::DM yaw_qps_matrix_casadi = result2["result"];

    //////////////////////////////////////
    //////////////////////////////////////

    return yaw_qps_matrix_casadi;

    // return 0;
  }
  std::cout << red << bold << "Boost A* Didn't find a path!! " << std::endl;
  // This should never happen

  // For debugging
  abort();
  // End of for debugging

  casadi::DM constant_yaw_matrix_casadi = y0 * casadi::DM::ones(1, Ny_ + 1);

  return constant_yaw_matrix_casadi;
}