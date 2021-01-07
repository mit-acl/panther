/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <sensor_msgs/PointCloud2.h>

#ifndef TRACKER_PREDICTOR_HPP
#define TRACKER_PREDICTOR_HPP

// typedef MADER_timers::Timer MyTimer;

struct cluster
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d bbox;  // Total side x, total side y, total side z;
};

struct track
{
  unsigned int id;
  Eigen::Vector3d centroid;
  Eigen::Vector3d bbox;  // Total side x, total side y, total side z;
  mt::PieceWisePol pwp;
  unsigned int num_frames_skipped = 0;
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