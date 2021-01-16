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

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
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
// auxiliary types
struct location
{
  float y, x;  // lat, long
};

struct data
{
  float yaw;
  size_t layer;

  void print()
  {
    std::cout << "yaw= " << yaw << ", layer= " << layer << std::endl;
  }
};

typedef float cost;

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
  std::map<std::string, casadi::DM> map_arg;
  map_arg["thetax_FOV_deg"] = par_.fov_x_deg;
  map_arg["thetay_FOV_deg"] = par_.fov_y_deg;
  map_arg["thetay_FOV_deg"] = par_.fov_y_deg;
  map_arg["b_T_c"] = b_Tmatrixcasadi_c_;
  map_arg["all_w_fe"] = all_w_fe;
  map_arg["guess_CPs_Pos"] = matrix_qp_guess;

  casadi::DM vector_yaw_samples(casadi::DM::zeros(1, par_.num_of_yaw_per_layer));
  for (int j = 0; j < par_.num_of_yaw_per_layer; j++)
  {
    vector_yaw_samples(0, j) = j * 2 * M_PI / par_.num_of_yaw_per_layer;  // TODO: handle wrapping angle
  }

  map_arg["yaw_samples"] = vector_yaw_samples;

  std::map<std::string, casadi::DM> result = casadi_visibility_function_(map_arg);
  casadi::DM vis_matrix_casadi = result["result"];  // Its values are in [0.1]
                                                    // we won't use its 1st col  (since y0 is given)

  // specify some types
  // typedef adjacency_list<listS, vecS, undirectedS, no_property, property<edge_weight_t, cost>> mygraph_t;
  typedef adjacency_list<listS, vecS, undirectedS, data, property<edge_weight_t, cost>> mygraph_t;
  typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vd;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef std::pair<int, int> edge;

  mygraph_t mygraph(0);  // start a graph with 0 vertices

  WeightMap weightmap = get(edge_weight, mygraph);

  // create all the vertexes and add them to the graph
  std::vector<std::vector<vd>> all_vertexes(par_.num_of_layers - 1, std::vector<vd>(par_.num_of_yaw_per_layer));
  std::vector<vd> tmp(1);  // first layer only one element
  all_vertexes.insert(all_vertexes.begin(), tmp);

  // https://stackoverflow.com/questions/47904550/should-i-keep-track-of-vertex-descriptors-in-boost-graph-library

  // add rest of the vertexes
  for (size_t i = 0; i < par_.num_of_layers; i++)  // i is the index of each layer
  {
    size_t num_of_circles_layer_i = (i == 0) ? 1 : par_.num_of_yaw_per_layer;
    for (size_t j = 0; j < num_of_circles_layer_i; j++)  // j is the index of each  circle in the layer i
    {
      vd vertex1 = boost::add_vertex(mygraph);
      all_vertexes[i][j] = vertex1;
      mygraph[vertex1].yaw = (i == 0) ? y0 : double(vector_yaw_samples(0, j));
      mygraph[vertex1].layer = i;
      // mygraph[vertex1].print();
      // std::cout << "So far, the graph has " << num_vertices(mygraph) << "vertices" << std::endl;
    }
  }

  double deltaT = (tf - t0) / (double(par_.num_of_layers));

  for (size_t i = 0; i < (par_.num_of_layers - 1); i++)  // i is the number of layers
  {
    size_t num_of_circles_layer_i = (i == 0) ? 1 : par_.num_of_yaw_per_layer;

    for (size_t j = 0; j < num_of_circles_layer_i; j++)  // j is the circle index of layer i
    {
      for (size_t j_next = 0; j_next < par_.num_of_yaw_per_layer; j_next++)
      {
        vd index_vertex1 = all_vertexes[i][j];
        vd index_vertex2 = all_vertexes[i + 1][j_next];  // TODO: [j_next, i + 1] would be more natural

        double distance = abs(wrapFromMPitoPi(mygraph[index_vertex1].yaw - mygraph[index_vertex2].yaw));

        // See if I should create and edge between all_vertexes[i][j] and all_vertexes[i+1][j]:
        if ((distance / deltaT) <= par_.ydot_max)  // This is a necessary condition (but not sufficient, since it's an
                                                   // average of yaw_dot between two yaws)
        {
          edge_descriptor e;
          bool inserted;
          boost::tie(e, inserted) = add_edge(index_vertex1, index_vertex2, mygraph);
          double distance_squared = pow(distance, 2.0);
          double visibility = double(vis_matrix_casadi(j_next, i + 1));  // \in [0,1]
                                                                         // Note that vis_matrix_casadi has in the rows
                                                                         // the circles, in the columns the layers.

          // std::cout << "edge between [" << i << ", " << j << "] and [" << i + 1 << ", " << j_next
          //           << "] with cost= " << edge_weight << std::endl;
          weightmap[e] = par_.c_smooth_yaw_search * distance_squared - par_.c_visibility_yaw_search * visibility +
                         1.5;  //+1.5 to ensure it's >=0
        }
      }
    }
  }

  std::cout << bold << yellow << "num_edges(mygraph)= " << num_edges(mygraph) << reset << std::endl;

  ////DEBUGGING
  if (num_edges(mygraph) < (par_.num_of_layers - 1))
  {
    std::cout << red << bold << "The layers are disconnected for sure, no solution will be found" << reset << std::endl;
    std::cout << red << bold << "Maybe ydot_max is too small?" << std::endl;
    abort();
  }
  ////END OF DEBUGGING

  vd start = all_vertexes[0][0];

  vector<vd> p(num_vertices(mygraph));
  vector<cost> d(num_vertices(mygraph));
  try
  {
    // call astar named parameter interface
    astar_search_tree(mygraph, start, distance_heuristic<mygraph_t, cost>(),
                      predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, mygraph)))
                          .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, mygraph)))
                          .visitor(astar_goal_visitor<vd>(par_.num_of_layers - 1)));
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
    // std::cout << mygraph[all_vertexes[0][0]].yaw;

    casadi::DM vector_shortest_path(casadi::Sparsity::dense(1, shortest_path_vd.size()));  // TODO: do this just once

    vector_shortest_path(0, 0) = mygraph[*spi].yaw;

    int i = 1;

    for (++spi; spi != shortest_path_vd.end(); ++spi)
    {
      // double yaw_tmp = mygraph[*spi].yaw;
      // std::cout << " -> " << yaw_tmp;
      vector_shortest_path(0, i) = mygraph[*spi].yaw;
      i = i + 1;
    }
    std::cout << "Shortest path: ";
    for (int i = 0; i < vector_shortest_path.columns(); i++)
    {
      std::cout << vector_shortest_path(0, i) << " --> ";
    }
    cout << endl << "\nTotal cost: " << d[fg.get_goal_found()] << endl;

    ////////////////////////////////////////
    // Now fit a spline to the yaws found
    ////////////////////////////////////////

    // First correct the angles so that the max absolute difference between two adjacent elements is <=pi

    // See "fit_to_angular_data.m"
    casadi::DM vsp_corrected =
        vector_shortest_path(casadi::Sparsity::dense(1, vector_shortest_path.columns()));  // TODO: do this just once

    vsp_corrected(0, 0) = vector_shortest_path(0, 0);
    for (size_t i = 1; i < vsp_corrected.columns(); i++)  // starts in 1, not in 0
    {
      double previous_phi = double(vsp_corrected(0, i - 1));
      double phi_i = double(vsp_corrected(0, i));
      double difference = previous_phi - phi_i;

      double phi_i_f = phi_i + floor(difference / (2 * M_PI)) * 2 * M_PI;
      double phi_i_c = phi_i + ceil(difference / (2 * M_PI)) * 2 * M_PI;

      if (fabs(previous_phi - phi_i_f) < fabs(previous_phi - phi_i_c))
      {
        vsp_corrected(0, i) = phi_i_f;
      }
      else
      {
        vsp_corrected(0, i) = phi_i_c;
      }
    }

    ////////////////ONLY FOR DEBUGGING
    for (size_t i = 1; i < vsp_corrected.columns(); i++)
    {
      double phi_mi = double(vsp_corrected(0, i - 1));
      double phi_i = double(vsp_corrected(0, i));

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

  casadi::DM constant_yaw_matrix_casadi = y0 * casadi::DM::ones(1, Ny_ + 1);

  return constant_yaw_matrix_casadi;
}