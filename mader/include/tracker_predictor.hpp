/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <sensor_msgs/PointCloud2.h>
#include "termcolor.hpp"

#ifndef TRACKER_PREDICTOR_HPP
#define TRACKER_PREDICTOR_HPP

// typedef MADER_timers::Timer MyTimer;

struct cluster  // one observation
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d bbox;  // Total side x, total side y, total side z;
};

class track
{
public:
  track(int size, const cluster& c)
  {
    ssw = size;
    history = std::deque<cluster>(ssw, c);  // Constant initialization
  }

  void addToHistory(const cluster& c)
  {
    history.push_back(c);
    if (history.size() > ssw)  // TODO (size of the sliding window)
    {
      history.pop_front();  // Delete the oldest element
    }
  }

  unsigned int num_frames_skipped = 0;

  mt::PieceWisePol pwp;

private:
  unsigned int ssw;  // size of the sliding window
  unsigned int id;

  // This deque will ALWAYS have ssw elements
  std::deque<cluster> history;  //[t-N], [t-N+1],...,[t] (i.e. index of the oldest element is 0)

  // void print()
  // {
  //   std::cout << termcolor::bold << "Centroid= " << termcolor::reset << centroid.transpose() << ", " <<
  //   termcolor::bold
  //             << "bbox= " << termcolor::reset << bbox.transpose() << std::endl;
  // }
};

class TrackerPredictor
{
public:
  TrackerPredictor();

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

protected:
private:
  double getCostRowColum(cluster& a, track& b, double time);
  void addNewTrack(const cluster& c, std::vector<track>& all_tracks, double time);
};

#endif