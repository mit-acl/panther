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
#include <string>  // std::string, std::stoi
#include <mader_msgs/Logtp.h>

#ifndef TRACKER_PREDICTOR_HPP
#define TRACKER_PREDICTOR_HPP

// typedef MADER_timers::Timer MyTimer;

namespace tp  // Tracker and predictor
{
struct logtp
{
  MADER_timers::Timer tim_total_tp;        //
  MADER_timers::Timer tim_conversion_pcl;  //
  MADER_timers::Timer tim_tf_transform;    //
  MADER_timers::Timer tim_remove_nans;     //
  MADER_timers::Timer tim_passthrough;     //
  MADER_timers::Timer tim_voxel_grid;      //
  MADER_timers::Timer tim_pub_filtered;    //
  MADER_timers::Timer tim_tree;            //
  MADER_timers::Timer tim_clustering;      //
  MADER_timers::Timer tim_bbox;            //
  MADER_timers::Timer tim_hungarian;       //
  MADER_timers::Timer tim_fitting;         //
  MADER_timers::Timer tim_pub;             //
};

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
  track(const tp::cluster& c, const int& min_ssw_tmp, const int& max_ssw_tmp)
  {
    max_ssw = max_ssw_tmp;
    min_ssw = min_ssw_tmp;

    // history = std::deque<tp::cluster>(min_ssw, c);  // Constant initialization
    // // We have only one observation --> we assume the obstacle has always been there
    // for (int i = 0; i < min_ssw; i++)
    // {
    //   history[i].time = c.time - (min_ssw - i - 1);
    //   //   c.time - (size - i - 1) * c.time / size;  // I need to have different times, if not A will become singular
    //   // std::cout << termcolor::magenta << "i= " << i << "history[i].time= " << history[i].time << termcolor::reset
    //   //           << std::endl;
    // }

    history.clear();
    history.push_back(c);

    color = Eigen::Vector3d(((double)rand() / (RAND_MAX)),   ////// r
                            ((double)rand() / (RAND_MAX)),   ////// g
                            ((double)rand() / (RAND_MAX)));  ////// b

    // use its hex value as the id
    // https://www.codespeedy.com/convert-rgb-to-hex-color-code-in-cpp/
    int r = color.x() * 255;
    int g = color.y() * 255;
    int b = color.z() * 255;

    std::stringstream ss;
    ss << "#";
    ss << std::hex << (r << 16 | g << 8 | b);
    id_string = ss.str();

    id_int = stoi(std::to_string(r) + std::to_string(g) + std::to_string(b));  // concatenate r, g, b

    // TODO: The previous approach will **almost** always generate different ids, but not always
  }

  void addToHistory(const tp::cluster& c)
  {
    history.push_back(c);
    if (history.size() > max_ssw)
    {
      history.pop_front();  // Delete the oldest element
    }
  }

  bool hasEnoughSamples()
  {
    return (getSizeSW() >= min_ssw);
  }

  unsigned int getSizeSW()
  {
    return history.size();
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

  double getLatestTimeSW()
  {
    return (history.back().time);
  }

  double getRelativeOldestTimeSW()
  {
    return 0.0;
  }

  double getRelativeLatestTimeSW()
  {
    return (history.back().time - history.front().time);
  }

  Eigen::Vector3d getLatestCentroid()
  {
    return history.back().centroid;
  }

  Eigen::Vector3d getLatestBbox()
  {
    return history.back().bbox;
  }

  void printPrediction(double seconds, int samples)
  {
    double last_time = getLatestTimeSW();
    double delta = seconds / samples;

    std::cout << "Predictions: " << std::endl;
    for (double t = last_time; t < (last_time + seconds); t = t + delta)
    {
      std::cout << "    t_to_the_future=" << t - last_time << " = " << pwp_mean.eval(t).transpose() << std::endl;
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

  mt::PieceWisePol pwp_mean;
  mt::PieceWisePol pwp_var;

  unsigned int num_frames_skipped = 0;
  Eigen::Vector3d color;
  std::string id_string;
  int id_int;

private:
  unsigned int max_ssw;  // max size of the sliding window
  unsigned int min_ssw;  // min size of the sliding window
  unsigned int id;

  // This deque will ALWAYS have ssw elements
  std::deque<tp::cluster> history;  //[t-N], [t-N+1],...,[t] (i.e. index of the oldest element is 0)
};                                  // namespace tp

};  // namespace tp
class TrackerPredictor
{
public:
  TrackerPredictor(ros::NodeHandle nh);

  // void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
  void cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud1);

  void printAllTracks();

  void generatePredictedPwpForTrack(tp::track& track_j);

protected:
private:
  double getCostRowColum(tp::cluster& a, tp::track& b, double time);
  void addNewTrack(const tp::cluster& c);
  void deleteMarkers();

  mader_msgs::Logtp logtp2LogtpMsg(tp::logtp log);

  visualization_msgs::MarkerArray getBBoxesAsMarkerArray();

  std::vector<tp::track> all_tracks_;

  // casadi::Function cf_get_mean_variance_pred_;

  std::map<int, casadi::Function> cf_get_mean_variance_pred_;

  int num_seg_prediction_;  // Comes from Matlab

  double z_ground_;
  int min_size_sliding_window_;
  int max_size_sliding_window_;
  double meters_to_create_new_track_;
  int max_frames_skipped_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  int min_dim_cluster_size_;
  double leaf_size_filter_;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  tf2_ros::Buffer tf_buffer_;

  ros::Publisher pub_marker_predicted_traj_;
  ros::Publisher pub_marker_bbox_obstacles_;
  ros::Publisher pub_traj_;
  ros::Publisher pub_pcloud_filtered_;
  ros::Publisher pub_log_;

  ros::NodeHandle nh_;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

  std::string namespace_markers = "predictor";

  std::vector<int> ids_markers_published_;

  tp::logtp log_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud2_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud3_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud4_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
};

#endif
