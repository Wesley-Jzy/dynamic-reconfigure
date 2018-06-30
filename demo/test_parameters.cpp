#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "TestRclConfig.h"
#include "dynamic_reconfigure_server_cpp.hpp"
#include <vector>
#include <string>

void cb(std::map<std::string, boost::any> all_value) {
  printf("user callback\n");
  /*
  std::string str_param = boost::any_cast<std::string>(all_value["str_param"]);
  int64_t int_param = boost::any_cast<int64_t>(all_value["int_param"]);
  int64_t size = boost::any_cast<int64_t>(all_value["size"]);
  double double_param = boost::any_cast<double>(all_value["double_param"]);
  bool bool_param = boost::any_cast<bool>(all_value["bool_param"]);

  std::cout << str_param.c_str() << int_param << size << double_param;
  if (bool_param)
    printf("True\n");
  else
    printf("False\n");
  */
}

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("get_parameters3");

  auto ser_ptr = std::make_shared<rqt_reconfigure::Server<ConfigureVec>>(node, cb);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
