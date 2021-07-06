/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "panther_types.hpp"
#include <deque>

#include <panther_msgs/PieceWisePolTraj.h>
#include <panther_msgs/CoeffPoly.h>
#include <panther_msgs/Log.h>

#include "ros/ros.h"

#define RED_NORMAL 1
#define RED_TRANS 2
#define RED_TRANS_TRANS 3
#define GREEN_NORMAL 4
#define BLUE_NORMAL 5
#define BLUE_TRANS 6
#define BLUE_TRANS_TRANS 7
#define BLUE_LIGHT 8
#define YELLOW_NORMAL 9
#define ORANGE_TRANS 10
#define BLACK_TRANS 11
#define TEAL_NORMAL 12

// #define STATE 0
// #define INPUT 1

// #define POS 0
// #define VEL 1
// #define ACCEL 2
// #define JERK 3

// #define WHOLE_TRAJ 0
// #define RESCUE_PATH 1

// #define OCCUPIED_SPACE 1

// #define UNKOWN_AND_OCCUPIED_SPACE 2

// TODO: put this in a namespace

template <typename T>
bool safeGetParam(ros::NodeHandle& nh, std::string const& param_name, T& param_value)
{
  if (!nh.getParam(param_name, param_value))
  {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(param_name, true).c_str());
    exit(1);
  }
  return true;
}

void verify(bool cond, std::string info_if_false);

double getMinTimeDoubleIntegrator1D(const double& p0, const double& v0, const double& pf, const double& vf,
                                    const double& v_max, const double& a_max);

double getMinTimeDoubleIntegrator3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pf,
                                    const Eigen::Vector3d& vf, const Eigen::Vector3d& v_max,
                                    const Eigen::Vector3d& a_max);

panther_msgs::Log log2LogMsg(mt::log log);
visualization_msgs::MarkerArray pwp2ColoredMarkerArray(mt::PieceWisePol& pwp, double t_init, double t_final,
                                                       int samples, std::string ns, Eigen::Vector3d& color);

mt::PieceWisePol createPwpFromStaticPosition(const mt::state& current_state);

mt::PieceWisePol pwpMsg2Pwp(const panther_msgs::PieceWisePolTraj& pwp_msg);
panther_msgs::PieceWisePolTraj pwp2PwpMsg(const mt::PieceWisePol& pwp);

visualization_msgs::Marker edges2Marker(const mt::Edges& edges, std_msgs::ColorRGBA color_marker);

geometry_msgs::Pose identityGeometryMsgsPose();

mt::PieceWisePol composePieceWisePol(const double t, const double dc, mt::PieceWisePol& p1, mt::PieceWisePol& p2);

bool boxIntersectsSphere(Eigen::Vector3d center, double r, Eigen::Vector3d c1, Eigen::Vector3d c2);

void printStateDeque(std::deque<mt::state>& data);

std::vector<std::string> pieceWisePol2String(const mt::PieceWisePol& piecewisepol);

void printStateVector(std::vector<mt::state>& data);

std_msgs::ColorRGBA getColorJet(double v, double vmin, double vmax);

std_msgs::ColorRGBA color(int id);

void quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw);

void quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

void quaternion2Euler(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw);

void saturate(int& var, const int min, const int max);

void saturate(double& var, const double min, const double max);

void saturate(Eigen::Vector3d& tmp, const Eigen::Vector3d& min, const Eigen::Vector3d& max);

visualization_msgs::Marker getMarkerSphere(double scale, int my_color);

double angleBetVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

void angle_wrap(double& diff);

geometry_msgs::Point pointOrigin();

Eigen::Vector3d vec2eigen(geometry_msgs::Vector3 vector);

geometry_msgs::Vector3 eigen2rosvector(Eigen::Vector3d vector);

geometry_msgs::Point eigen2point(Eigen::Vector3d vector);

std::vector<double> eigen2std(const Eigen::Vector3d& v);

geometry_msgs::Vector3 vectorNull();

geometry_msgs::Vector3 vectorUniform(double a);

int nChoosek(int n, int k);
void linearTransformPoly(const Eigen::VectorXd& coeff_old, Eigen::VectorXd& coeff_new, double a, double b);
void changeDomPoly(const Eigen::VectorXd& coeff_p, double tp1, double tp2, Eigen::VectorXd& coeff_q, double tq1,
                   double tq2);
// sign function
template <typename T>
int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

double cdfUnivariateNormalDist(double x, double mu, double std_deviation);
double probUnivariateNormalDistAB(double a, double b, double mu, double std_deviation);
double probMultivariateNormalDist(const Eigen::VectorXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& mu,
                                  const Eigen::VectorXd& std_deviation);

// Overload to be able to print a std::vector
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
{
  if (!v.empty())
  {
    out << '[';
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

visualization_msgs::MarkerArray trajectory2ColoredMarkerArray(const mt::trajectory& data, double max_value, int increm,
                                                              std::string ns, double scale, std::string color_type,
                                                              int id_agent, int n_agents);

#endif
