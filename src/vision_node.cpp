
#include <ros_kortex_vision/vision.h>

#include <rclcpp/rclcpp.hpp>
#include <signal.h>

Vision* g_vision = NULL;

void sigintHandler(int signal)
{
  if (g_vision)
  {
    g_vision->quit();
  }

  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_vision", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh, nh_private("~");

  // Override the default ros sigint handler.
  signal(SIGINT, sigintHandler);

  g_vision = new Vision(nh, nh_private);
  g_vision->run();

  delete g_vision;

  return 0;
}
