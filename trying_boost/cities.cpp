
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

using namespace boost;
using namespace std;

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

int main(int argc, char** argv)
{
  // specify some types
  typedef adjacency_list<listS, vecS, undirectedS, no_property, property<edge_weight_t, cost>> mygraph_t;
  typedef adjacency_list<listS, vecS, undirectedS, data, property<edge_weight_t, cost>> mygraph_t_bueno;
  typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vd;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef std::pair<int, int> edge;

  size_t num_of_layers = 5;
  size_t num_of_circles_per_layer = 5;  // except in the initial layer, that has only one value

  mygraph_t_bueno mygraph(0);  // start a graph with 0 vertices

  WeightMap weightmap = get(edge_weight, mygraph);

  // create all the vertexes and add them to the graph
  std::vector<std::vector<vd>> all_vertexes(num_of_layers - 1, std::vector<vd>(num_of_circles_per_layer));
  std::vector<vd> tmp(1);  // first layer only one element
  all_vertexes.insert(all_vertexes.begin(), tmp);

  // https://stackoverflow.com/questions/47904550/should-i-keep-track-of-vertex-descriptors-in-boost-graph-library

  // add rest of the vertexes
  for (size_t i = 0; i < num_of_layers; i++)  // i is the number of layers
  {
    size_t num_of_circles_layer_i = (i == 0) ? 1 : num_of_circles_per_layer;
    for (size_t j = 0; j < num_of_circles_layer_i; j++)  // j is the number of circles per layer
    {
      vd vertex1 = boost::add_vertex(mygraph);
      all_vertexes[i][j] = vertex1;
      mygraph[vertex1].yaw = 2 * M_PI / (j + 1);  // TODO: Check this to do it uniformly (wrapping)
      mygraph[vertex1].layer = i;
      // mygraph[vertex1].print();
      // std::cout << "So far, the graph has " << num_vertices(mygraph) << "vertices" << std::endl;
    }
  }

  for (size_t i = 0; i < (num_of_layers - 1); i++)  // i is the number of layers
  {
    size_t num_of_circles_layer_i = (i == 0) ? 1 : num_of_circles_per_layer;

    for (size_t j = 0; j < num_of_circles_layer_i; j++)  // j is the circle index of layer i
    {
      for (size_t j_next = 0; j_next < num_of_circles_per_layer; j_next++)
      {
        edge_descriptor e;
        bool inserted;

        vd index_vertex1 = all_vertexes[i][j];
        vd index_vertex2 = all_vertexes[i + 1][j_next];

        boost::tie(e, inserted) = add_edge(index_vertex1, index_vertex2, mygraph);

        double tmp = wrapFromMPitoPi(mygraph[index_vertex1].yaw - mygraph[index_vertex2].yaw);

        double distance_squared = tmp * tmp;
        double visibility = 0;

        // std::cout << "edge between [" << i << ", " << j << "] and [" << i + 1 << ", " << j_next
        //           << "] with cost= " << edge_weight << std::endl;

        weightmap[e] = distance_squared - visibility;
      }
    }
  }

  vd start = all_vertexes[0][0];

  vector<vd> p(num_vertices(mygraph));
  vector<cost> d(num_vertices(mygraph));
  try
  {
    // call astar named parameter interface
    astar_search_tree(mygraph, start, distance_heuristic<mygraph_t, cost>(),
                      predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, mygraph)))
                          .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, mygraph)))
                          .visitor(astar_goal_visitor<vd>(num_of_layers - 1)));
  }

  catch (found_goal<vd> fg)
  {  // found a path to the goal
    list<vd> shortest_path;
    for (vd v = fg.get_goal_found();; v = p[v])
    {
      shortest_path.push_front(v);
      if (p[v] == v)
        break;
    }
    cout << "Shortest path: ";
    list<vd>::iterator spi = shortest_path.begin();
    std::cout << mygraph[all_vertexes[0][0]].yaw;
    for (++spi; spi != shortest_path.end(); ++spi)
    {
      std::cout << " -> " << mygraph[*spi].yaw;
    }
    std::cout << std::endl;
    cout << endl << "Total cost: " << d[fg.get_goal_found()] << endl;
    return 0;
  }
  cout << "Didn't find a path " << endl;
  return 0;
}