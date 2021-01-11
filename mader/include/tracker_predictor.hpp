/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */
#include <Eigen/Core>
#include "mader_types.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "termcolor.hpp"
#include <casadi/casadi.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

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
    std::streamsize ss = std::cout.precision();  // original precision
    std::cout << termcolor::bold << "Centroid= " << termcolor::reset << centroid.transpose() << ", " << termcolor::bold
              << "bbox= " << termcolor::reset << bbox.transpose() << " time= " << std::setprecision(20) << time
              << std::setprecision(ss) << std::endl;
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
    for (int i = 0; i < ssw; i++)
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
    return (history.back().time - history.front().time);
  }

  double getOldestTimeSW()
  {
    return (history.front().time);
  }

  double getRelativeTimeHistory(int i)
  {
    return (history[i].time - history.front().time);
  }

  double getEarliestTimeSW()
  {
    return (history.back().time);
  }

  double getRelativeOldestTimeSW()
  {
    return 0.0;
  }

  double getRelativeEarliestTimeSW()
  {
    return (history.back().time - history.front().time);
  }

  Eigen::Vector3d getEarliestCentroid()
  {
    return history.back().centroid;
  }

  Eigen::Vector3d getEarliestBbox()
  {
    return history.back().bbox;
  }

  void printPrediction(double seconds, int samples)
  {
    double last_time = getEarliestTimeSW();
    double delta = seconds / samples;

    std::cout << "Predictions: " << std::endl;
    for (double t = last_time; t < (last_time + seconds); t = t + delta)
    {
      std::cout << "    t_to_the_future=" << t - last_time << " = " << pwp.eval(t).transpose() << std::endl;
    }
  }

  void printHistory()
  {
    std::cout << "Track History= " << std::endl;

    for (auto& c : history)
    {
      c.print();
    }
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
  TrackerPredictor(ros::NodeHandle nh);

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
  void printAllTracks();

  void generatePredictedPwpForTrack(track& track_j);

protected:
private:
  double getCostRowColum(cluster& a, track& b, double time);
  void addNewTrack(const cluster& c);

  visualization_msgs::MarkerArray getBBoxesAsMarkerArray();

  std::vector<track> all_tracks_;

  std::vector<casadi::Function> cfs_kkt_Ab_;
  casadi::Function cf_coeff_predicted_;

  int num_seg_prediction_;  // Comes from Matlab

  int size_sliding_window_;            // TODO (as a param)
  double meters_to_create_new_track_;  // TODO (as a param)
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  tf2_ros::Buffer tf_buffer_;

  ros::Publisher pub_marker_predicted_traj_;
  ros::Publisher pub_marker_bbox_obstacles_;

  ros::NodeHandle nh_;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
};

#endif