// Taken from http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

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

    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);
    // image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1 & FrameDrawer::imageCb, this);
    // image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
  }

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

    BOOST_FOREACH (const std::string& frame_id, frame_ids_)
    {
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

      all_projections_.push_back(uv);

      plotAllProjections(image);
      // cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);

      // CvSize text_size;
      // int baseline;
      // cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
      //   CvPoint origin = cvPoint(uv.x - text_size.width / 2, uv.y - RADIUS - baseline - 3);
      // cv:
      //   putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 12, CV_RGB(255, 0, 0));

      std::cout << "Drawing at " << uv.x << ", " << uv.y << std::endl;
    }

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