#include "faster_ros.hpp"

int main(int argc, char **argv)
{
  std::cout << "Going to initialize Faster node" << std::endl;
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "faster");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_replan_CB("~");
  ros::NodeHandle nh_pub_CB("~");
  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  nh_replan_CB.setCallbackQueue(&custom_queue1);
  nh_pub_CB.setCallbackQueue(&custom_queue2);

  std::cout << "Creating faster_ros object" << std::endl;
  FasterRos FasterRos(nh, nh_replan_CB, nh_pub_CB);

  std::cout << "faster_ros object created" << std::endl;

  // NOW THE CALLBACK replanCB is IN A DIFFERENT thread than the other callbacks.

  // TODO: I think one thread in the line below will be enough (and will have the same effect, becacuse there is only
  // one callback)
  ros::AsyncSpinner spinner1(0, &custom_queue1);
  ros::AsyncSpinner spinner2(0, &custom_queue2);
  spinner1.start();  // start spinner of the custom queue 1
  spinner2.start();  // start spinner of the custom queue 2

  // ros::spin();  // spin the normal queue

  while (ros::ok())
  {
    ros::spinOnce();  // spin the normal queue
  }

  // with this the callbacks can be executed in parallel. If not, the replanning and pub callbacks will be executed in
  // series, and if the replanning callback takes a lot of time, the pub callback will not be executed
  // ros::AsyncSpinner spinner(0);  // 0 means # of threads= # of CPU cores
  // spinner.start();
  ros::waitForShutdown();
  return 0;
}