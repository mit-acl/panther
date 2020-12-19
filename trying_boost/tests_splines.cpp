#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <iostream>

typedef Eigen::Spline<float, 3> Spline3d;

int main()
{
  std::vector<Eigen::VectorXf> waypoints;
  Eigen::Vector3f po1(2, 3, 4);
  Eigen::Vector3f po2(2, 5, 4);
  Eigen::Vector3f po3(2, 8, 9);
  Eigen::Vector3f po4(2, 8, 23);
  waypoints.push_back(po1);
  waypoints.push_back(po2);
  waypoints.push_back(po3);
  waypoints.push_back(po4);

  // The degree of the interpolating spline needs to be one less than the number of points
  // that are fitted to the spline.
  Eigen::MatrixXf points(3, waypoints.size());
  int row_index = 0;
  for (auto const way_point : waypoints)
  {
    points.col(row_index) << way_point[0], way_point[1], way_point[2];
    row_index++;
  }
  Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2);
  float time_ = 0;
  for (int i = 0; i < 20; i++)
  {
    time_ += 1.0 / (20 * 1.0);
    Eigen::VectorXf values = spline(time_);
    std::cout << values << std::endl;
  }
  return 0;
}