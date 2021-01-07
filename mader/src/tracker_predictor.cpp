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

#include "mader_types.hpp"

#include "Hungarian.h"
#include "tracker_predictor.hpp"

TrackerPredictor::TrackerPredictor()
{
}

double TrackerPredictor::getCostRowColum(cluster& a, track& b, double time)
{
  return (a.centroid - b.pwp.eval(time)).norm();
}

void TrackerPredictor::addNewTrack(const cluster& c, std::vector<track>& all_tracks, double time)
{
  track tmp;
  tmp.centroid = c.centroid;
  tmp.bbox = c.bbox;
  mt::PieceWisePol pwp;  // will have only one interval
  pwp.times.push_back(time);
  pwp.times.push_back(std::numeric_limits<double>::max());  // infty

  pwp.coeff_x.push_back(Eigen::Vector4d(0.0, 0.0, 0.0, c.centroid.x()));  // [a b c d]' of Int0 , [a b c d]' of Int1,...
  pwp.coeff_y.push_back(Eigen::Vector4d(0.0, 0.0, 0.0, c.centroid.y()));  // [a b c d]' of Int0 , [a b c d]' of Int1,...
  pwp.coeff_z.push_back(Eigen::Vector4d(0.0, 0.0, 0.0, c.centroid.z()));  // [a b c d]' of Int0 , [a b c d]' of Int1,...

  tmp.pwp = pwp;

  // unsigned int last_id = (all_tracks.size() == 0) ? 0 : (all_tracks[all_tracks.rbegin()->.id] + 1);

  all_tracks.push_back(tmp);

  // all_tracks[last_id] = tmp;
}

void TrackerPredictor::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *input_cloud);

  tree->setInputCloud(input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.3);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(600);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);

  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract(cluster_indices);

  std::vector<cluster> clusters;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    // First option:

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

    // Second option, it's faster (see
    // https://stackoverflow.com/questions/35669182/this-predefined-function-slowing-down-my-programs-performance)

    auto pit0 = it->indices.begin();

    double min_x = input_cloud->points[*pit0].x, min_y = input_cloud->points[*pit0].y,
           min_z = input_cloud->points[*pit0].z, max_x = input_cloud->points[*pit0].x,
           max_y = input_cloud->points[*pit0].y, max_z = input_cloud->points[*pit0].z;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      if (input_cloud->points[*pit].x <= min_x)
        min_x = input_cloud->points[*pit].x;
      else if (input_cloud->points[*pit].y <= min_y)
        min_y = input_cloud->points[*pit].y;
      else if (input_cloud->points[*pit].z <= min_z)
        min_z = input_cloud->points[*pit].z;
      else if (input_cloud->points[*pit].x >= max_x)
        max_x = input_cloud->points[*pit].x;
      else if (input_cloud->points[*pit].y >= max_y)
        max_y = input_cloud->points[*pit].y;
      else if (input_cloud->points[*pit].z >= max_z)
        max_z = input_cloud->points[*pit].z;
    }

    ////////////////////////
    ////////////////////////

    cluster tmp;
    tmp.bbox = Eigen::Vector3d(max_x - min_x, max_y - min_y, max_z - min_z);
    tmp.centroid = Eigen::Vector3d((max_x - min_x) / 2.0, (max_y - min_y) / 2.0,
                                   (max_z - min_z) / 2.0);  // This is the centroid of the bbox, not the centroid of the
                                                            // point cloud
    clusters.push_back(tmp);
  }

  std::cout << "Done with cloud_cb" << std::endl;

  std::vector<track> all_tracks;

  // rows = clusters
  // colums = tracks

  double meters_to_create_new_track = 2.0;
  int max_frames_skipped = 10;

  std::vector<unsigned int> indexes_costs_too_big;

  double time = input->header.stamp.toSec();

  // Compute costs for each of the clusters detected
  for (unsigned int i = 0; i < clusters.size(); i++)
  {
    double min_cost = std::numeric_limits<double>::max();
    for (auto& all_tracks_j : all_tracks)
    {
      min_cost = std::min(min_cost, getCostRowColum(clusters[i], all_tracks_j, time));
    }

    if (min_cost > meters_to_create_new_track)
    {
      indexes_costs_too_big.push_back(i);
    }
  }

  // If the minimum of each of the rows is too large --> create new track
  for (auto i : indexes_costs_too_big)
  {
    addNewTrack(clusters[i], all_tracks, time);
  }

  //////////////////////////
  // Run Hungarian Algorithm
  //////////////////////////

  // Create the cost matrix
  std::vector<std::vector<double>> costMatrix;

  for (auto cluster_i : clusters)  // for each of the rows
  {
    std::vector<double> costs_cluster_i;
    for (auto& all_tracks_j : all_tracks)  // for each of the columns
    {
      costs_cluster_i.push_back(getCostRowColum(cluster_i, all_tracks_j, time));
    }

    costMatrix.push_back(costs_cluster_i);  // Add row to the matrix
  }

  // Run the Hungarian Algorithm;
  HungarianAlgorithm HungAlgo;
  std::vector<int> track_assigned_to_cluster;
  double cost = HungAlgo.Solve(costMatrix, track_assigned_to_cluster);

  //////////////////////////
  //////////////////////////

  ///////////////////////////

  // Increase by one the frames skipped on all the tracks
  std::for_each(all_tracks.begin(), all_tracks.end(), [](track& x) { x.num_frames_skipped++; });

  for (unsigned int i = 0; i < costMatrix.size(); i++)  // for each of the rows
  {
    std::cout << i << "," << track_assigned_to_cluster[i] << "\t";

    all_tracks[track_assigned_to_cluster[i]].num_frames_skipped--;

    // If a cluster has been unassigned (can happen if rows>columns), then create a new track for it
    if (track_assigned_to_cluster[i] == -1)
    {
      addNewTrack(clusters[i], all_tracks, time);
    }
  }

  //////////////////////////
  //////////////////////////

  // Erase the tracks that haven't been detected in many frames
  all_tracks.erase(
      std::remove_if(all_tracks.begin(), all_tracks.end(),
                     [max_frames_skipped](const track& x) { return x.num_frames_skipped > max_frames_skipped; }),
      all_tracks.end());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cluster_node");

  ros::NodeHandle nh("~");

  std::cout << "About to setup callback\n";

  TrackerPredictor tmp;

  ros::Subscriber sub = nh.subscribe("cloud", 1, &TrackerPredictor::cloud_cb, &tmp);

  // pub_cluster0 = nh.advertise<sensor_msgs::PointCloud2>("cluster_0", 1);
  // objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);

  // markerPub = nh.advertise<visualization_msgs::MarkerArray>("viz", 1);

  ros::spin();
}