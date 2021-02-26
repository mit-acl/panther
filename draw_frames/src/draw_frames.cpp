// Taken from http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <fstream>

class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids) : it_(nh_), frame_ids_(frame_ids)
  {
    std::string image_topic = nh_.resolveName("image");

    std::string param_name = "/SQ01s/panther/mode";

    if (!nh_.getParam("/SQ01s/panther/mode", mode_))
    {
      ROS_ERROR("Failed to find parameter: %s", nh_.resolveName(param_name, true).c_str());
      exit(1);
    }

    name_file_pos_ = "/home/jtorde/Desktop/ws/src/panther/" + mode_ + "_projection_positions.txt";
    name_file_vel_ = "/home/jtorde/Desktop/ws/src/panther/" + mode_ + "_projection_velocities.txt";

    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);
    // image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1 & FrameDrawer::imageCb, this);
    // image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

    // Delete content of the files
    // https://stackoverflow.com/questions/17032970/clear-data-inside-text-file-in-c
    std::ofstream ofs;
    ofs.open(name_file_pos_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();

    ofs.open(name_file_vel_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
  }

  std::string mode_;
  double last_time_ = 0.0;
  bool last_initialized_ = false;
  cv::Point2d last_uv_;

  std::string name_file_pos_;
  std::string name_file_vel_;

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try
    {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    std::string frame_id = frame_ids_.front();  // Taking only the first one for now. TODO

    // BOOST_FOREACH (const std::string& frame_id, frame_ids_)
    // {
    tf::StampedTransform transform;
    try
    {
      ros::Time acquisition_time = info_msg->header.stamp;
      ros::Duration timeout(1.0 / 30);
      tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id, acquisition_time, timeout);
      tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id, acquisition_time, transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return;
    }

    tf::Point pt = transform.getOrigin();

    cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
    cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

    // Create and open a text file

    // NOte that the (x,y) coordinates are as follows:

    //  ------> x
    //  |
    //  |   IMAGE
    //  |
    //  V y

    // (i.e. origin is on the top left corner of the image).
    // For a point to be in the image, we need to have x \in [0, image.cols] and y \in [0, image.rows]

    bool point_in_front_of_camera = (pt.z() > 0);
    bool point_behind_camera = (!point_in_front_of_camera);
    std::fstream myfile;

    // if (point_behind_camera == false)
    // {

    myfile.open(name_file_pos_, std::ios_base::app);
    myfile << uv.x << " " << uv.y << " " << image.cols << " " << image.rows << " " << int(point_in_front_of_camera)
           << "\n";
    myfile.close();
    // }
    // else
    // {
    //   std::cout << "Point is behind the camera!!!" << std::endl;
    // }

    std::cout << "Image of width= " << image.cols << ", height= " << image.rows << std::endl;
    if (uv.x > image.cols || uv.x < 0 || uv.y > image.rows || uv.y < 0 || point_behind_camera)
    {
      std::cout << "Not in FOV!" << std::endl;
      last_initialized_ = false;
    }
    else
    {
      all_projections_.push_back(uv);
      plotAllProjections(image);
      // cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);

      // CvSize text_size;
      // int baseline;
      // cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
      //   CvPoint origin = cvPoint(uv.x - text_size.width / 2, uv.y - RADIUS - baseline - 3);
      // cv:
      //   putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 12, CV_RGB(255, 0, 0));

      if (last_initialized_ == true)
      {
        double distance = sqrt(pow(uv.x - last_uv_.x, 2) + pow(uv.y - last_uv_.y, 2));
        double delta_t = ((info_msg->header.stamp).toSec() - last_time_);
        double vel = distance / delta_t;

        std::cout << "Last point was " << last_uv_.x << ", " << last_uv_.y << std::endl;
        std::cout << "Now is " << uv.x << ", " << uv.y << std::endl;
        std::cout << "delta_t= " << std::setprecision(4) << delta_t << " ss" << std::endl;
        std::cout << "Vel= " << vel << " px/s" << std::endl;

        myfile.open(name_file_vel_, std::ios_base::app);
        myfile << vel << "\n";
        myfile.close();
      }
      last_time_ = (info_msg->header.stamp).toSec();
      last_uv_ = uv;
      last_initialized_ = true;
    }

    //}

    pub_.publish(input_bridge->toImageMsg());
  }

  void plotAllProjections(cv::Mat& image)
  {
    static const int RADIUS = 3;

    for (int i = 0; i < all_projections_.size(); i++)
    {
      if (i == (all_projections_.size() - 1))
      {
        cv::circle(image, all_projections_[i], 2 * RADIUS, CV_RGB(0, 255, 0), -1);
      }
      else
      {
        cv::circle(image, all_projections_[i], RADIUS, CV_RGB(255, 0, 0), -1);
      }
    }
  }

private:
  std::vector<cv::Point2d> all_projections_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
  ros::spin();
}