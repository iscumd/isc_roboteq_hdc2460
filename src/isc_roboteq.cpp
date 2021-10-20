#include <memory>
#include <functional>

#include "isc_roboteq/isc_roboteq.hpp"
#include "std_msgs/msg/string.hpp"

namespace Roboteq
{
Roboteq::Roboteq(rclcpp::NodeOptions options)
: Node("roboteq", options)
{


}
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world isc_roboteq package\n");
  return 0;
}
