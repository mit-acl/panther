#include "tracker_predictor.hpp"
#include <Eigen/Core>
#include <ros/package.h>
#include "termcolor.hpp"

using namespace termcolor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cluster_node");

  ros::NodeHandle nh("~");

  TrackerPredictor tracker_predictor(nh);

  tp::cluster c;
  c.centroid = Eigen::Vector3d::Ones();
  c.bbox = Eigen::Vector3d(1.0, 1.0, 1.0);
  c.time = ros::Time::now().toSec();

  tp::track my_track(c, 10, 15);

  tracker_predictor.generatePredictedPwpForTrack(my_track);

  double time = 0.0;  // ros::Time::now().toSec();

  std::cout << " ============================= " << std::endl;

  for (int i = 0; i < 10; i++)
  {
    tp::cluster c;
    c.centroid = pow(i, 2) * Eigen::Vector3d::Ones();
    c.bbox = Eigen::Vector3d(1.0, 1.0, 1.0);
    c.time = time + i;
    std::cout << std::setprecision(20) << "t= " << c.time << ". c.centroid= " << c.centroid.transpose() << std::endl;

    my_track.addToHistory(c);
  }

  tracker_predictor.generatePredictedPwpForTrack(my_track);

  double time_test = my_track.getLatestTimeSW();

  my_track.printPrediction(3.0, 5);

  // ros::spin();
  return 0;
}