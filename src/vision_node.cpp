
#include <ros_kortex_vision/vision.h>

#include <rclcpp/rclcpp.hpp>
#include <signal.h>

std::unique_ptr<ros_kortex_vision::Vision> node;

void sigintHandler(int signal)
{
  if (node)
  {
    node->quit();
  }

  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  // Override the default sigint handler.
  signal(SIGINT, sigintHandler);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;

  node = std::make_unique<ros_kortex_vision::Vision>(opt);
  rclcpp::spin(node->getNodeBaseInterface());
  return 0;
}
