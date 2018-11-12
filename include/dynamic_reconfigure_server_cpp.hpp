#ifndef __rqt__reconfigure__SERVER_H__
#define __rqt__reconfigure__SERVER_H__


#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <map>
#include <string>
#include <boost/any.hpp>

#include <thread>

namespace rqt_reconfigure {

template <class ConfigType> 
class Server_cpp {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Server_cpp)
  
  RCLCPP_PUBLIC
  explicit Server_cpp(const std::string node_name,
      std::function<void(std::map<std::string, boost::any>)> user_callback) :
      callback_function(user_callback) {
    printf("Server init____________\n");
    //parameter_service = std::make_shared<rclcpp::ParameterService>(node);
    //auto parameters_init_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    node = rclcpp::Node::make_shared(node_name);
    node->set_parameters(ConfigType().getParameterVariantVec());
    auto listdata = node->list_parameters({"name"}, 0);
    for(auto name : listdata.names) {
      auto param_val = node->get_parameters({
        "value." + name.substr(5)
      });
      values.insert(values.end(), param_val.begin(), param_val.end());
    }
    auto callback =
      [this](const std::vector<rclcpp::Parameter> & parameter_changed) 
        -> rcl_interfaces::msg::SetParametersResult
      { 
        this->param_change(parameter_changed);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = 1;
        return result;
      };
    node->register_param_change_callback(callback);
    rclcpp::spin(node);
  }
  void param_change(const std::vector<rclcpp::Parameter> &parameter_changed) {
    for (auto iter = values.begin(); iter != values.end();) {
      if(iter->get_name() == parameter_changed[0].get_name()) {
        iter = values.erase(iter);
      }
      else
        iter ++;
    }
    values.insert(values.end(), parameter_changed.begin(), parameter_changed.end());

    std::map<std::string, boost::any> value_map;
    
    for (auto parameter : values) {
      switch(parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          value_map[parameter.get_name().substr(6)] = parameter.as_bool();
          continue;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          value_map[parameter.get_name().substr(6)] = parameter.as_int();
          continue;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          value_map[parameter.get_name().substr(6)] = parameter.as_double();
          continue;
        case rclcpp::ParameterType::PARAMETER_STRING:
          value_map[parameter.get_name().substr(6)] = parameter.as_string();
          continue;
        default:
          continue;
      }
    }
    callback_function(value_map);
  }
  /*
  void set_callback(std::function<void(std::vector<rclcpp::Parameter>)> callback_f) {
    callback_function = callback_f;
  }
  */
private:
  //std::shared_ptr<rclcpp::ParameterService> parameter_service;
  rclcpp::Node::SharedPtr node;
  std::vector<rclcpp::Parameter> values;
  std::function<void(std::map<std::string, boost::any>)> callback_function;
};

static void workThread(const std::string node_name,
                      std::function<void(std::map<std::string, boost::any>)> user_callback) {
        rqt_reconfigure::Server_cpp<ConfigureVec>(node_name, user_callback);
      }


template <class ConfigType>
class Server {
  public:
      explicit Server(const std::string node_name,
                      std::function<void(std::map<std::string, boost::any>)> user_callback)
                      :callback_function(user_callback) {
        std::string new_node_name = "DynamicReconfigure_" + node_name;
        std::thread t(workThread, new_node_name, callback_function);
        t.detach();     
      }
  private:
    std::function<void(std::map<std::string, boost::any>)> callback_function;
};

}//rqt_reconfigure

#endif 
