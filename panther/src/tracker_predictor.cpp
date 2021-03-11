/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
// #include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include "panther_types.hpp"

#include "Hungarian.h"
#include "tracker_predictor.hpp"

#include <ros/package.h>  //TODO: remove this ros dependency

#include <tf2_eigen/tf2_eigen.h>

#include "utils.hpp"

#include "visualization_msgs/MarkerArray.h"

#include <panther_msgs/DynTraj.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

using namespace termcolor;

TrackerPredictor::TrackerPredictor(ros::NodeHandle nh) : nh_(nh)
{
  // safeGetParam(nh_, "z_ground", z_ground_);
  safeGetParam(nh_, "x_min", x_min_);
  safeGetParam(nh_, "x_max", x_max_);

  safeGetParam(nh_, "y_min", y_min_);
  safeGetParam(nh_, "y_max", y_max_);

  safeGetParam(nh_, "z_min", z_min_);
  safeGetParam(nh_, "z_max", z_max_);

  safeGetParam(nh_, "min_size_sliding_window", min_size_sliding_window_);
  safeGetParam(nh_, "max_size_sliding_window", max_size_sliding_window_);
  safeGetParam(nh_, "meters_to_create_new_track", meters_to_create_new_track_);
  safeGetParam(nh_, "max_frames_skipped", max_frames_skipped_);
  safeGetParam(nh_, "cluster_tolerance", cluster_tolerance_);
  safeGetParam(nh_, "min_cluster_size", min_cluster_size_);
  safeGetParam(nh_, "max_cluster_size", max_cluster_size_);
  safeGetParam(nh_, "min_dim_cluster_size", min_dim_cluster_size_);
  safeGetParam(nh_, "leaf_size_filter", leaf_size_filter_);

  for (int i = min_size_sliding_window_; i <= max_size_sliding_window_; i++)
  {
    std::string folder = ros::package::getPath("panther") + "/matlab/casadi_generated_files/";

    cf_get_mean_variance_pred_[i] =
        casadi::Function::load(folder + "get_mean_variance_pred_" + std::to_string(i) + ".casadi");
  }

  tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));  // needed (although tf_listener_ptr_ is not used explicitly)

  pub_marker_predicted_traj_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_predicted_traj", 1);
  pub_marker_bbox_obstacles_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_bbox_obstacles", 1);
  pub_traj_ = nh_.advertise<panther_msgs::DynTraj>("trajs_predicted", 1, true);  // The last boolean is latched or not
  // pub_pcloud_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>("pcloud_filtered", 1);
  pub_pcloud_filtered_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("pcloud_filtered", 1);
  pub_log_ = nh_.advertise<panther_msgs::Logtp>("logtp", 1);

  int i;

  // sub_ = nh_.subscribe("cloud", 1, boost::bind(TrackerPredictor::cloud_cb, _1, i));
  sub_ = nh_.subscribe("cloud", 1, &TrackerPredictor::cloud_cb, this);
  // sub_state_ = nh1_.subscribe("state", 1, &PantherRos::stateCB, this);

  tree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);

  input_cloud2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloud3_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloud4_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloud1 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

double TrackerPredictor::getCostRowColum(tp::cluster& a, tp::track& b, double time)
{
  double result = (a.centroid - b.pwp_mean.eval(time)).norm();
  // std::cout << "getCostRowColum= " << result << std::endl;
  return result;
}

void TrackerPredictor::addNewTrack(const tp::cluster& c)
{
  tp::track tmp(c, min_size_sliding_window_, max_size_sliding_window_);
  // mt::PieceWisePol pwp;  // will have only one interval
  // pwp.times.push_back(time);
  // pwp.times.push_back(std::numeric_limits<double>::max());  // infty

  // pwp.coeff_x.push_back(Eigen::Vector4d(0.0, 0.0, 0.0, c.centroid.x()));  // [a b c d]' of Int0 , [a b c d]' of
  // Int1,... pwp.coeff_y.push_back(Eigen::Vector4d(0.0, 0.0, 0.0, c.centroid.y()));  // [a b c d]' of Int0 , [a b c d]'
  // of Int1,... pwp.coeff_z.push_back(Eigen::Vector4d(0.0, 0.0, 0.0, c.centroid.z()));  // [a b c d]' of Int0 , [a b c
  // d]' of Int1,...

  // tmp.pwp = pwp;

  // unsigned int last_id = (all_tracks_.size() == 0) ? 0 : (all_tracks_[all_tracks_.rbegin()->.id] + 1);

  // std::cout << red << "Calling generatePredictedPwpForTrack()" << reset << std::endl;

  generatePredictedPwpForTrack(tmp);

  all_tracks_.push_back(tmp);

  // std::cout << red << "End of addNewTrack" << reset << std::endl;

  // all_tracks_[last_id] = tmp;
}

void TrackerPredictor::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
// void TrackerPredictor::cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud1)
{
  log_.tim_total_tp.tic();
  ///////////////////////////
  ///////////////////////////
  ///////////////////////////I need to do it here because if input_cloud is empty (after filtering), I return

  // Increase by one the frames skipped on all the tracks
  std::for_each(all_tracks_.begin(), all_tracks_.end(), [](tp::track& x) { x.num_frames_skipped++; });

  // Erase the tracks that haven't been detected in many frames
  int tracks_removed = 0;
  all_tracks_.erase(std::remove_if(all_tracks_.begin(), all_tracks_.end(),
                                   [this, tracks_removed](const tp::track& x) mutable {
                                     tracks_removed++;
                                     return x.num_frames_skipped > max_frames_skipped_;
                                   }),
                    all_tracks_.end());

  // std::cout << "Removed " << tracks_removed << " tracks because too many frames skipped" << std::endl;

  ///////////////////////////
  ///////////////////////////
  ///////////////////////////

  // Convert to PCL
  log_.tim_conversion_pcl.tic();
  pcl::fromROSMsg(*pcl2ptr_msg, *input_cloud1);
  log_.tim_conversion_pcl.toc();

  // std::cout << "Input point cloud has " << input_cloud1->points.size() << " points" << std::endl;

  std_msgs::Header header_pcloud;
  pcl_conversions::fromPCL(input_cloud1->header, header_pcloud);

  log_.tim_tf_transform.tic();
  // Transform w_T_b
  Eigen::Affine3d w_T_b;
  geometry_msgs::TransformStamped transform_stamped;

  try
  {
    transform_stamped = tf_buffer_.lookupTransform("world", header_pcloud.frame_id, header_pcloud.stamp,
                                                   ros::Duration(0.02));  // TODO: change this duration time?

    // transform_stamped = tf_buffer_.lookupTransform("world", header_pcloud.frame_id,
    // header_pcloud.stamp,ros::Duration(0.02));

    w_T_b = tf2::transformToEigen(transform_stamped);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_DEBUG("[world_database_master_ros] OnGetTransform failed with %s", ex.what());
    return;
  }
  pcl::transformPointCloud(*input_cloud1, *input_cloud2_, w_T_b);
  log_.tim_tf_transform.toc();

  // Remove nans
  log_.tim_remove_nans.tic();
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud2_, *input_cloud3_, indices);
  log_.tim_remove_nans.toc();

  log_.tim_passthrough.tic();

  // // PassThrough Filter (remove the ground)
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(input_cloud3_);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(z_ground_, 1e6);
  // // pass.filter(*input_cloud4_);
  // pass.filter(*input_cloud_);

  // Box filter https://stackoverflow.com/questions/45790828/remove-points-outside-defined-3d-box-inside-pcl-visualizer
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0));
  boxFilter.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0));
  boxFilter.setInputCloud(input_cloud3_);
  boxFilter.filter(*input_cloud4_);

  log_.tim_passthrough.toc();

  // Voxel grid filter
  log_.tim_voxel_grid.tic();
  // // pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor; //this one crashes randomly on the NUC and Jetson
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud4_);
  sor.setLeafSize(leaf_size_filter_, leaf_size_filter_, leaf_size_filter_);
  sor.filter(*input_cloud_);
  log_.tim_voxel_grid.toc();

  log_.tim_pub_filtered.tic();

  // Publish filtered point cloud

  // Option 1
  sensor_msgs::PointCloud2 filtered_pcl2_msg;
  pcl::toROSMsg(*input_cloud_, filtered_pcl2_msg);
  filtered_pcl2_msg.header.frame_id = "world";
  filtered_pcl2_msg.header.stamp = ros::Time::now();
  pub_pcloud_filtered_.publish(filtered_pcl2_msg);

  // Option 2, crashes randomly on the NUC and Jetson
  // input_cloud_->header.frame_id = "world";
  // //
  // https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_conversions/include/pcl_conversions/pcl_conversions.h#L88
  // input_cloud_->header.stamp = ros::Time::now().toNSec() / 1000ull;
  // pub_pcloud_filtered_.publish(*input_cloud_);

  log_.tim_pub_filtered.toc();

  /////Listen to the transform

  // Eigen::Vector3d transform;

  // Eigen::Affine3d w_T_b;

  // sensor_msgs::PointCloud2Ptr pcl2ptr_msg_transformed(new sensor_msgs::PointCloud2());

  // tf2::doTransform(*pcl2ptr_msg, *pcl2ptr_msg_transformed, transform_stamped);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_with_nans(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // pcl::fromROSMsg(*pcl2ptr_msg_transformed, *input_cloud_with_nans);

  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*input_cloud_with_nans, *input_cloud, indices);

  // // Difference VoxelGrid vs UniformSampling:
  // //
  // http://www.pcl-users.org/Voxelgrid-without-the-weighted-centroid-td3551390.html#message3564880:~:text=One%20is%20part%20of%20the%20filtering,be%20used%20with%20the%20features%20library.

  // // Voxel grid filter
  // pcl::VoxelGrid<pcl::PointXYZ> sor;
  // sor.setInputCloud(input_cloud);
  // sor.setLeafSize(0.1f, 0.1f, 0.1f);
  // sor.filter(*input_cloud);

  if (input_cloud_->points.size() == 0)
  {
    // std::cout << "Point cloud is empty, doing nothing" << std::endl;
    return;
  }

  // std::cout << "Filtering:" << input_cloud1->points.size() << " -->" << input_cloud_->points.size() << " points";

  log_.tim_tree.tic();
  tree_->setInputCloud(input_cloud_);
  log_.tim_tree.toc();

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree_);
  ec.setInputCloud(input_cloud_);

  /* Extract the clusters out of pc and save indices in cluster_indices.*/

  // std::cout << "Doing the clustering..." << std::endl;
  log_.tim_clustering.tic();
  ec.extract(cluster_indices);
  log_.tim_clustering.toc();
  // std::cout << "Clustering done!" << std::endl;

  std::vector<tp::cluster> clusters;

  double time_pcloud = header_pcloud.stamp.toSec();

  log_.tim_bbox.tic();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    // std::cout << "--- New cluster" << std::endl;
    // std::cout << " " << std::endl;

    ///////////////////////
    // Compute bounding box
    ///////////////////////

    // First option (slow):

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    // for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    // {
    //   cloud_cluster->points.push_back(input_cloud->points[*pit]);  // TODO: I think I can avoid doing this
    // }

    // cloud_cluster->width = cloud_cluster->points.size();
    // cloud_cluster->height = 1;
    // cloud_cluster->is_dense = true;

    // pcl::PointXYZ minPt, maxPt;

    // pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

    ////////////////////////
    ////////////////////////

    // Second option, it's faster (taken from
    // https://stackoverflow.com/questions/35669182/this-predefined-function-slowing-down-my-programs-performance   )
    // But note that I've deleted the else() (I think they are wrong)
    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      // std::cout << "input_cloud->points[*pit]= " << input_cloud->points[*pit] << std::endl;
      if (input_cloud_->points[*pit].x <= min_x)
      {
        min_x = input_cloud_->points[*pit].x;
      }
      if (input_cloud_->points[*pit].y <= min_y)
      {
        min_y = input_cloud_->points[*pit].y;
      }
      if (input_cloud_->points[*pit].z <= min_z)
      {
        min_z = input_cloud_->points[*pit].z;
      }
      if (input_cloud_->points[*pit].x >= max_x)
      {
        // std::cout << "assigning max_x" << std::endl;
        max_x = input_cloud_->points[*pit].x;
      }
      if (input_cloud_->points[*pit].y >= max_y)
      {
        // std::cout << "assigning max_y" << std::endl;
        max_y = input_cloud_->points[*pit].y;
      }
      if (input_cloud_->points[*pit].z >= max_z)
      {
        // std::cout << "assigning max_z" << std::endl;
        max_z = input_cloud_->points[*pit].z;
      }
    }

    // std::cout << "min_x= " << min_x << std::endl;
    // std::cout << "min_y= " << min_y << std::endl;
    // std::cout << "min_z= " << min_z << std::endl;
    // std::cout << "max_x= " << max_x << std::endl;
    // std::cout << "max_y= " << max_y << std::endl;
    // std::cout << "max_z= " << max_z << std::endl;

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      sum_x += input_cloud_->points[*pit].x;
      sum_y += input_cloud_->points[*pit].y;
      sum_z += input_cloud_->points[*pit].z;
    }

    double mean_x = sum_x / (it->indices).size();
    double mean_y = sum_y / (it->indices).size();
    double mean_z = sum_z / (it->indices).size();

    tp::cluster tmp;
    tmp.centroid = Eigen::Vector3d(mean_x, mean_y, mean_z);

    tmp.bbox = Eigen::Vector3d(2 * (std::max(fabs(max_x - mean_x), fabs(min_x - mean_x))),
                               2 * (std::max(fabs(max_y - mean_y), fabs(min_y - mean_y))),
                               2 * (std::max(fabs(max_z - mean_z), fabs(min_z - mean_z))));

    tmp.time = time_pcloud;
    clusters.push_back(tmp);

    // ////////////////////////
    // ////////////////////////

    // std::vector<Eigen::Vector4d> vertexes_bbox(8);
    // vertexes_bbox[0] = Eigen::Vector4d(max_x, max_y, max_z, 1.0);
    // vertexes_bbox[1] = Eigen::Vector4d(max_x, max_y, min_z, 1.0);
    // vertexes_bbox[2] = Eigen::Vector4d(max_x, min_y, max_z, 1.0);
    // vertexes_bbox[3] = Eigen::Vector4d(max_x, min_y, min_z, 1.0);
    // vertexes_bbox[4] = Eigen::Vector4d(min_x, max_y, max_z, 1.0);
    // vertexes_bbox[5] = Eigen::Vector4d(min_x, max_y, min_z, 1.0);
    // vertexes_bbox[6] = Eigen::Vector4d(min_x, min_y, max_z, 1.0);
    // vertexes_bbox[7] = Eigen::Vector4d(min_x, min_y, min_z, 1.0);

    // // for (size_t i = 0; i < vertexes_bbox.size(); i++)
    // // {
    // //   // std::cout << "vertex " << i << " = " << vertexes_bbox[i].transpose() << std::endl;
    // // }

    // // std::cout << "w_T_b.translation()= " << w_T_b.translation() << std::endl;
    // // std::cout << "w_T_b.rotation()= " << w_T_b.rotation() << std::endl;

    // // apply r_T_w to the vertexes of the bbox (this is much faster than transforming the whole point cloud, although
    // a
    // // little bit conservative (because it's the AABB of a AABB))

    // // for (size_t i = 0; i < vertexes_bbox.size(); i++)
    // // {
    // //   vertexes_bbox[i] = w_T_b * vertexes_bbox[i];
    // // }
    // // https://stackoverflow.com/questions/9070752/getting-the-bounding-box-of-a-vector-of-points
    // auto xExtremes =
    //     std::minmax_element(vertexes_bbox.begin(), vertexes_bbox.end(),
    //                         [](const Eigen::Vector4d& lhs, const Eigen::Vector4d& rhs) { return lhs.x() < rhs.x();
    //                         });

    // auto yExtremes =
    //     std::minmax_element(vertexes_bbox.begin(), vertexes_bbox.end(),
    //                         [](const Eigen::Vector4d& lhs, const Eigen::Vector4d& rhs) { return lhs.y() < rhs.y();
    //                         });

    // auto zExtremes =
    //     std::minmax_element(vertexes_bbox.begin(), vertexes_bbox.end(),
    //                         [](const Eigen::Vector4d& lhs, const Eigen::Vector4d& rhs) { return lhs.z() < rhs.z();
    //                         });

    // max_x = xExtremes.second->x();
    // max_y = yExtremes.second->y();
    // max_z = zExtremes.second->z();

    // min_x = xExtremes.first->x();
    // min_y = yExtremes.first->y();
    // min_z = zExtremes.first->z();

    // // std::cout << std::endl;

    // // std::cout << "min_x= " << min_x << std::endl;
    // // std::cout << "min_y= " << min_y << std::endl;
    // // std::cout << "min_z= " << min_z << std::endl;
    // // std::cout << "max_x= " << max_x << std::endl;
    // // std::cout << "max_y= " << max_y << std::endl;
    // // std::cout << "max_z= " << max_z << std::endl;

    // // min_x = std::numeric_limits<double>::max();
    // // max_x = -std::numeric_limits<double>::max();
    // // min_y = std::numeric_limits<double>::max();
    // // max_y = -std::numeric_limits<double>::max();
    // // min_z = std::numeric_limits<double>::max();
    // // max_z = -std::numeric_limits<double>::max();

    // // for (auto& vertex : vertexes_bbox)
    // // {
    // //   std::cout << "vertex before= " << vertex.transpose() << std::endl;

    // //   if (vertex.x() <= min_x)
    // //     min_x = vertex.x();
    // //   if (vertex.y() <= min_y)
    // //     min_y = vertex.y();
    // //   if (vertex.z() <= min_z)
    // //     min_z = vertex.z();
    // //   if (vertex.x() >= max_x)
    // //     max_x = vertex.x();
    // //   if (vertex.y() >= max_y)
    // //     max_y = vertex.y();
    // //   if (vertex.z() >= max_z)
    // //     max_z = vertex.z();
    // // }

    // tp::cluster tmp;
    // tmp.bbox = Eigen::Vector3d(max_x - min_x, max_y - min_y, max_z - min_z);

    // assert(tmp.bbox.x() >= 0 && "Must hold: tmp.bbox.x() >= 0");
    // assert(tmp.bbox.y() >= 0 && "Must hold: tmp.bbox.y() >= 0");
    // assert(tmp.bbox.z() >= 0 && "Must hold: tmp.bbox.z() >= 0");

    // if (tmp.bbox.x() < min_dim_cluster_size_ && tmp.bbox.y() < min_dim_cluster_size_ &&
    //     tmp.bbox.z() < min_dim_cluster_size_)
    // {
    //   continue;  // cluster too small --> noise
    // }

    // // std::cout << bold << magenta << "bbox= " << tmp.bbox << reset << std::endl;

    // tmp.centroid = Eigen::Vector3d((max_x + min_x) / 2.0, (max_y + min_y) / 2.0,
    //                                (max_z + min_z) / 2.0);  // This is the centroid of the bbox, not the
    //                                                         // centroid of the point cloud

    // // std::cout << red << "tmp.centroid= " << tmp.centroid.transpose() << reset << std::endl;

    // tmp.time = time_pcloud;

    // // std::cout << red << tmp.centroid.transpose() << reset << std::endl;
    // clusters.push_back(tmp);
  }
  log_.tim_bbox.toc();

  // rows = clusters
  // colums = tracks

  std::vector<unsigned int> indexes_costs_too_big;

  // Compute costs for each of the clusters detected
  // std::cout << "Computing costs for each of the clusters detected" << std::endl;
  // std::cout << "Num of clusters detected= " << clusters.size() << std::endl;
  // std::cout << "Num of current tracks   = " << all_tracks_.size() << std::endl;
  for (unsigned int i = 0; i < clusters.size(); i++)
  {
    double min_cost = std::numeric_limits<double>::max();
    for (auto& track_j : all_tracks_)
    {
      min_cost = std::min(min_cost, getCostRowColum(clusters[i], track_j, time_pcloud));
    }

    // std::cout << "min_cost= " << min_cost << std::endl;

    if (min_cost > meters_to_create_new_track_)
    {
      indexes_costs_too_big.push_back(i);
    }
  }

  // std::cout << "Creating= " << indexes_costs_too_big.size() << " new clusters because cost row is too big" <<
  // std::endl;

  // If the minimum of each of the rows is too large --> create new track
  for (auto i : indexes_costs_too_big)
  {
    // clusters[i].print();
    addNewTrack(clusters[i]);
  }

  //////////////////////////
  ///////////////////////////

  //////////////////////////
  // Run Hungarian Algorithm
  //////////////////////////

  if (clusters.size() > 0)
  {
    // std::cout << "Running Hungarian Algorithm" << std::endl;

    // std::cout << "Creating the cost matrix!" << std::endl;
    // Create the cost matrix
    std::vector<std::vector<double>> costMatrix;

    for (auto cluster_i : clusters)  // for each of the rows
    {
      std::vector<double> costs_cluster_i;
      for (auto& track_j : all_tracks_)  // for each of the columns
      {
        costs_cluster_i.push_back(getCostRowColum(cluster_i, track_j, time_pcloud));
      }

      costMatrix.push_back(costs_cluster_i);  // Add row to the matrix
    }

    // Run the Hungarian Algorithm;
    HungarianAlgorithm HungAlgo;
    std::vector<int> track_assigned_to_cluster;
    // std::cout << "Calling Hungarian Algorithm now!!" << std::endl;
    log_.tim_hungarian.tic();
    double cost = HungAlgo.Solve(costMatrix, track_assigned_to_cluster);
    log_.tim_hungarian.toc();
    // std::cout << "Called  Hungarian Algorithm!" << std::endl;

    for (unsigned int i = 0; i < costMatrix.size(); i++)  // for each of the rows
    {
      // std::cout << i << "," << track_assigned_to_cluster[i] << "\t";

      all_tracks_[track_assigned_to_cluster[i]].num_frames_skipped--;

      // If a cluster has been unassigned (can happen if rows>columns), then create a new track for it
      if (track_assigned_to_cluster[i] == -1)
      {
        // std::cout << "cluster " << i << " unassigned, creating new track for it" << std::endl;
        std::cout << clusters[i].centroid.transpose() << std::endl;
        addNewTrack(clusters[i]);
      }
      else
      {  // add an element to the history of the track
        if (all_tracks_[track_assigned_to_cluster[i]].is_new == true)
        {
          all_tracks_[track_assigned_to_cluster[i]].is_new = false;
        }
        else
        {
          all_tracks_[track_assigned_to_cluster[i]].addToHistory(clusters[i]);
        }
      }
    }
    // std::cout << "\n";
  }
  else
  {
    // std::cout << "No clusters detected" << std::endl;
  }
  // printAllTracks();

  ////////////////////////////////////
  // Now fit a spline to past history
  ////////////////////////////////////

  log_.tim_fitting.tic();
  for (auto& track_j : all_tracks_)
  {
    generatePredictedPwpForTrack(track_j);
  }
  log_.tim_fitting.toc();

  ////////////////////////////////////
  // publish the stuff
  ////////////////////////////////////

  int samples = 20;

  log_.tim_pub.tic();
  int j = 0;
  for (auto& track_j : all_tracks_)
  {
    // std::cout << "----" << std::endl;
    // std::cout<<"==================TIME PCLOUD: "<<std::endl;
    // std::streamsize ss = std::cout.precision();  // original precision
    // std::cout << " time_pcloud= " << std::setprecision(20) << time_pcloud  << std::setprecision(ss) << std::endl;
    // std::cout<<"==================HISTORY: "<<std::endl;
    // track_j.printHistory();
    // std::cout<<"==================PREDICTION: "<<std::endl;
    // track_j.printPrediction(3.0, 5);

    if (track_j.shouldPublish() == false)
    {
      continue;
    }

    // track_j.pwp.print();

    std::string ns = "predicted_traj_" + std::to_string(j);
    pub_marker_predicted_traj_.publish(
        pwp2ColoredMarkerArray(track_j.pwp_mean, time_pcloud, time_pcloud + 2.0, samples, ns, track_j.color));

    /////////////////// construct a DynTraj msg. //TODO: use the pwp instead (this will require modifications in the
    /// panther code, for when it's not an agent)

    panther_msgs::DynTraj dynTraj_msg;
    dynTraj_msg.header.frame_id = "world";
    dynTraj_msg.header.stamp = ros::Time::now();
    dynTraj_msg.use_pwp_field = true;
    dynTraj_msg.pwp_mean = pwp2PwpMsg(track_j.pwp_mean);
    dynTraj_msg.pwp_var = pwp2PwpMsg(track_j.pwp_var);
    // dynTraj_msg.s_mean = pieceWisePol2String(track_j.pwp_mean);
    // dynTraj_msg.s_var = pieceWisePol2String(track_j.pwp_var);

    std::vector<double> tmp = eigen2std(track_j.getLatestBbox());
    // std::vector<double> tmp = eigen2std(track_j.getMaxBbox());

    dynTraj_msg.bbox = std::vector<float>(tmp.begin(), tmp.end());  // TODO: Here I'm using the latest Bbox. Should I
                                                                    // use the biggest one of the whole history?
    dynTraj_msg.pos = eigen2rosvector(track_j.pwp_mean.eval(ros::Time::now().toSec()));
    dynTraj_msg.id = track_j.id_int;
    dynTraj_msg.is_agent = false;

    pub_traj_.publish(dynTraj_msg);
    //////////////////

    j++;
  }
  deleteMarkers();
  pub_marker_bbox_obstacles_.publish(getBBoxesAsMarkerArray());

  log_.tim_pub.toc();

  // std::cout << "End of cloud_cb" << std::endl;
  log_.tim_total_tp.toc();

  pub_log_.publish(logtp2LogtpMsg(log_));
}

panther_msgs::Logtp TrackerPredictor::logtp2LogtpMsg(tp::logtp log)
{
  panther_msgs::Logtp log_msg;

  log_msg.ms_total_tp = log_.tim_total_tp.getMsSaved();
  log_msg.ms_conversion_pcl = log_.tim_conversion_pcl.getMsSaved();
  log_msg.ms_tf_transform = log_.tim_tf_transform.getMsSaved();
  log_msg.ms_remove_nans = log_.tim_remove_nans.getMsSaved();
  log_msg.ms_passthrough = log_.tim_passthrough.getMsSaved();
  log_msg.ms_voxel_grid = log_.tim_voxel_grid.getMsSaved();
  log_msg.ms_pub_filtered = log_.tim_pub_filtered.getMsSaved();
  log_msg.ms_tree = log_.tim_tree.getMsSaved();
  log_msg.ms_clustering = log_.tim_clustering.getMsSaved();
  log_msg.ms_bbox = log_.tim_bbox.getMsSaved();
  log_msg.ms_hungarian = log_.tim_hungarian.getMsSaved();
  log_msg.ms_fitting = log_.tim_fitting.getMsSaved();
  log_msg.ms_pub = log_.tim_pub.getMsSaved();

  log_msg.header.stamp = ros::Time::now();

  return log_msg;
}

void TrackerPredictor::printAllTracks()
{
  std::cout << green << "All tracks: " << reset;
  for (int i = 0; i < all_tracks_.size(); i++)
  {
    std::cout << "Track " << i << " (" << all_tracks_[i].num_frames_skipped << "), ";
  }
  std::cout << std::endl;
}

void TrackerPredictor::generatePredictedPwpForTrack(tp::track& track_j)
{
  // std::cout << "Creating the matrices" << std::endl;
  casadi::DM all_pos = casadi::DM::zeros(3, track_j.getSizeSW());  //(casadi::Sparsity::dense(3, track_j.getSizeSW()));
  casadi::DM all_t = casadi::DM::zeros(1, track_j.getSizeSW());    //(casadi::Sparsity::dense(1, track_j.getSizeSW()));

  // std::cout << "Matrices created" << std::endl;

  int current_ssw = track_j.getSizeSW();

  // std::cout<<"getTotalTimeSW= "<<track_j.getTotalTimeSW()<<std::endl;

  for (int i = 0; i < current_ssw; i++)
  {
    // Conversion DM <--> Eigen:  https://github.com/casadi/casadi/issues/2563
    Eigen::Vector3d centroid_i_filtered;

    Eigen::Vector3d centroid_i = track_j.getCentroidHistory(i);

    // if(i>=1){ //low-pass filter
    //     Eigen::Vector3d centroid_im1 = track_j.getCentroidHistory(i-1);
    //     double alpha=0.95;
    //     centroid_i_filtered = (1-alpha)*centroid_i + alpha*centroid_im1;
    //   }
    // else
    // {
    centroid_i_filtered = centroid_i;
    // }
    all_pos(0, i) = centroid_i_filtered.x();
    all_pos(1, i) = centroid_i_filtered.y();
    all_pos(2, i) = centroid_i_filtered.z();
    // std::cout << "track_j.getTimeHistory(i)= " << track_j.getTimeHistory(i) << std::endl;
    // std::cout << "Going to add, i=" << i << " cols= " << track_j.getSizeSW() << std::endl;
    all_t(0, i) = track_j.getTimeHistory(i);
    // std::cout << "Added" << std::endl;
  }

  // std::cout << "Matrices assigned" << std::endl;

  // track_j.printHistory();

  std::map<std::string, casadi::DM> map_arguments;
  map_arguments["all_t"] = all_t;
  map_arguments["all_pos"] = all_pos;

  // std::cout<<"current_ssw= "<<current_ssw<<std::endl;

  // std::cout << "all_pos.size()=\n " << all_pos.rows() << ", " << all_pos.columns() << std::endl;
  // std::cout << "all_pos:\n " << all_pos << std::endl;
  // std::cout << "all_t:\n " << all_t << std::endl;
  // std::cout << "all_t.size()=\n " << all_t.rows() << ", " << all_t.columns() << std::endl;

  // std::cout << "Calling casadi!" << std::endl;
  // Note that Casadi may crash (with the error "Evaluation failed") if secs_prediction (in prediction_one_segment.m) is
  // very big (like 100)
  std::map<std::string, casadi::DM> result = cf_get_mean_variance_pred_[current_ssw](map_arguments);
  // std::cout << "Called casadi " << std::endl;

  // std::cout << "RESULT Before: " << std::endl;

  // std::cout << "coeffs_mean:\n " << result["coeff_mean"] << std::endl;
  // std::cout << "coeffs_var:\n " << result["coeff_var"] << std::endl;
  // std::cout << "secs_prediction:\n " << result["secs_prediction"] << std::endl;

  casadi::DM coeffs_mean = result["coeff_mean"];
  casadi::DM coeffs_var = result["coeff_var"];
  double secs_prediction = double(result["secs_prediction"]);

  // std::cout << "RESULT AFTER: " << std::endl;

  // std::cout << "coeffs_mean:\n " << coeffs_mean << std::endl;
  // std::cout << "coeffs_var:\n " << coeffs_var << std::endl;
  // std::cout << "secs_prediction:\n " << secs_prediction << std::endl;

  // std::cout << "Coeffs: " << std::endl;
  // std::cout << coeffs << std::endl;

  //////////////////////////

  ///////////////////////////////////////////////////// Fill the mean
  Eigen::VectorXd mean_coeff_x(coeffs_mean.columns());
  Eigen::VectorXd mean_coeff_y(coeffs_mean.columns());
  Eigen::VectorXd mean_coeff_z(coeffs_mean.columns());

  for (int i = 0; i < mean_coeff_x.size(); i++)
  {
    mean_coeff_x(i) = double(coeffs_mean(0, i));
    mean_coeff_y(i) = double(coeffs_mean(1, i));
    mean_coeff_z(i) = double(coeffs_mean(2, i));
  }

  mt::PieceWisePol pwp_mean;  // will have only one interval
  pwp_mean.times.push_back(track_j.getLatestTimeSW());
  pwp_mean.times.push_back(track_j.getLatestTimeSW() + secs_prediction);

  pwp_mean.all_coeff_x.push_back(mean_coeff_x);
  pwp_mean.all_coeff_y.push_back(mean_coeff_y);
  pwp_mean.all_coeff_z.push_back(mean_coeff_z);

  track_j.pwp_mean = pwp_mean;

  // std::cout << "mean_coeff_x= " << mean_coeff_x.transpose() << std::endl;

  // std::cout << " -------- PWP " << std::endl;
  // pwp_mean.print();
  // std::cout << "Evaluation at t=" << track_j.getLatestTimeSW() << " = "
  //           << pwp_mean.eval(track_j.getLatestTimeSW()).transpose() << std::endl;
  // std::cout << magenta << "real= " << track_j.getLatestCentroid().transpose() << reset << std::endl;

  ///////////////////////////////////////////////////// Fill the variance
  Eigen::VectorXd var_coeff_x(coeffs_var.columns());
  Eigen::VectorXd var_coeff_y(coeffs_var.columns());
  Eigen::VectorXd var_coeff_z(coeffs_var.columns());

  for (int i = 0; i < var_coeff_x.size(); i++)
  {
    var_coeff_x(i) = double(coeffs_var(0, i));
    var_coeff_y(i) = double(coeffs_var(1, i));
    var_coeff_z(i) = double(coeffs_var(2, i));
  }

  mt::PieceWisePol pwp_var;  // will have only one interval
  pwp_var.times = pwp_mean.times;

  pwp_var.all_coeff_x.push_back(var_coeff_x);
  pwp_var.all_coeff_y.push_back(var_coeff_y);
  pwp_var.all_coeff_z.push_back(var_coeff_z);

  track_j.pwp_var = pwp_var;

  // double time_pcloud = track_j.getLatestTimeSW() - track_j.getOldestTimeSW();
  // Eigen::Vector4d T =
  //     Eigen::Vector4d(pow(time_pcloud, 2), pow(time_pcloud, 2), pow(time_pcloud, 1), pow(time_pcloud, 0));
  // std::cout << magenta << "predicted before= " << coeff_old_x.transpose() * T <<  //////
  //     ", " << coeff_old_y.transpose() * T <<                                      /////
  //     ", " << coeff_old_z.transpose() * T << reset << std::endl;

  // std::cout << magenta << "predicted after= " << track_j.pwp.eval(time_pcloud +
  // track_j.getOldestTimeSW()).transpose()
  //           << reset << std::endl;
  // std::cout << magenta << "real= " << track_j.getLatestCentroid().transpose() << reset << std::endl;
  // std::cout << std::endl;
}

// See issue https://answers.ros.org/question/263031/delete-all-rviz-markers-in-a-specific-namespace/
void TrackerPredictor::deleteMarkers()
{
  visualization_msgs::MarkerArray marker_array;
  for (auto& j : ids_markers_published_)
  {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;
    m.id = j;
    m.ns = namespace_markers;
    marker_array.markers.push_back(m);
  }

  pub_marker_bbox_obstacles_.publish(marker_array);

  ids_markers_published_.clear();
}

visualization_msgs::MarkerArray TrackerPredictor::getBBoxesAsMarkerArray()
{
  visualization_msgs::MarkerArray marker_array;

  int j = 0;
  for (auto& track_j : all_tracks_)
  {
    if (track_j.shouldPublish() == false)
    {
      continue;
    }
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::CUBE;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.ns = namespace_markers;
    m.action = visualization_msgs::Marker::ADD;
    m.id = j;
    m.lifetime = ros::Duration(0.0);  // 0.0 means forever

    ids_markers_published_.push_back(m.id);

    std_msgs::ColorRGBA color;
    color.r = track_j.color.x();
    color.g = track_j.color.y();
    color.b = track_j.color.z();
    color.a = 0.6;

    m.color = color;  // color(BLUE_TRANS_TRANS);

    Eigen::Vector3d centroid = track_j.getLatestCentroid();

    m.pose.position.x = centroid.x();
    m.pose.position.y = centroid.y();
    m.pose.position.z = centroid.z();

    // Eigen::Vector3d bbox = track_j.getLatestBbox();
    Eigen::Vector3d bbox = track_j.getMaxBbox();

    m.scale.x = bbox.x();
    m.scale.y = bbox.y();
    m.scale.z = bbox.z();

    m.pose.orientation.w = 1.0;

    marker_array.markers.push_back(m);

    j = j + 1;

    // add its text
    m.id = j;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.ns = namespace_markers;
    m.text = track_j.id_string;
    m.color.a = 1.0;

    m.pose.position.z = m.pose.position.z + bbox.z();

    ids_markers_published_.push_back(m.id);

    marker_array.markers.push_back(m);

    j = j + 1;
  }

  return marker_array;
}
