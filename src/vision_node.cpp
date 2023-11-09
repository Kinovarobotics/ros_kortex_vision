
#include <ros_kortex_vision/vision.h>

#include <rclcpp/rclcpp.hpp>
#include <signal.h>

std::shared_ptr<ros_kortex_vision::Vision> node;

void sigintHandler(int signal)
{
  if (node)
  {
    node->quit();
  }

  rclcpp::shutdown();
  exit(signal);
}

int main(int argc, char** argv)
{
  // Override the default sigint handler.
  signal(SIGINT, sigintHandler);

  rclcpp::init(argc, argv);

  auto opt = rclcpp::NodeOptions().use_intra_process_comms(true);
  node = std::make_shared<ros_kortex_vision::Vision>(opt);
  node->run();

  return 0;
}
