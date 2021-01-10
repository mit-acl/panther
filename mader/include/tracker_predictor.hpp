/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <sensor_msgs/PointCloud2.h>
#include "termcolor.hpp"
#include <casadi/casadi.hpp>
#include <tf2_ros/transform_listener.h>

#ifndef TRACKER_PREDICTOR_HPP
#define TRACKER_PREDICTOR_HPP

// typedef MADER_timers::Timer MyTimer;

struct cluster  // one observation
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d bbox;  // Total side x, total side y, total side z;
  double time;           // in seconds

  void print()
  {
    std::cout << termcolor::bold << "Centroid= " << termcolor::reset << centroid.transpose() << ", " << termcolor::bold
              << "bbox= " << termcolor::reset << bbox.transpose() << " time= " << time << std::endl;
  }
};

class track
{
public:
  track(int size, const cluster& c)
  {
    ssw = size;

    history = std::deque<cluster>(ssw, c);  // Constant initialization

    // We have only one observation --> we assume the obstacle has always been there
    // std::cout << termcolor::magenta << "c.time= " << c.time << termcolor::reset << std::endl;
    for (int i = 0; i < size; i++)
    {
      history[i].time = c.time - (size - i - 1);
      //   c.time - (size - i - 1) * c.time / size;  // I need to have different times, if not A will become singular
      // std::cout << termcolor::magenta << "i= " << i << "history[i].time= " << history[i].time << termcolor::reset
      //           << std::endl;
    }
  }

  void addToHistory(const cluster& c)
  {
    history.push_back(c);
    if (history.size() > ssw)  // TODO (size of the sliding window)
    {
      history.pop_front();  // Delete the oldest element
    }
  }

  unsigned int getSizeSW()
  {
    return ssw;
  }

  Eigen::Vector3d getCentroidHistory(int i)
  {
    return history[i].centroid;
  }

  double getTimeHistory(int i)
  {
    return history[i].time;
  }

  double getTotalTimeSW()  // Total time of the sliding window
  {
    return (history.front().time - history.back().time);
  }

  double getOldestTimeSW()
  {
    return (history.front().time);
  }

  double getEarliestTimeSW()
  {
    return (history.back().time);
  }

  mt::PieceWisePol pwp;

  unsigned int num_frames_skipped = 0;

private:
  unsigned int ssw;  // size of the sliding window
  unsigned int id;

  // This deque will ALWAYS have ssw elements
  std::deque<cluster> history;  //[t-N], [t-N+1],...,[t] (i.e. index of the oldest element is 0)
};

class TrackerPredictor
{
public:
  TrackerPredictor();

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
  void printAllTracks();

protected:
private:
  double getCostRowColum(cluster& a, track& b, double time);
  void addNewTrack(const cluster& c);

  void generatePredictedPwpForTrack(track& track_j);

  std::vector<track> all_tracks_;

  std::vector<casadi::Function> cfs_kkt_Ab_;
  casadi::Function cf_coeff_predicted_;

  int num_seg_prediction_ = 4;    // TODO (as a param)
  int size_sliding_window_ = 10;  // TODO (as a param)

  double meters_to_create_new_track_ = 2.0;  // TODO (as a param)

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  tf2_ros::Buffer tf_buffer_;
};

#endif