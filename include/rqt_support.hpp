#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <vector>
#include <map>




namespace rqt_reconfigure{


class Client {
public:
  Client(const rclcpp::Node::SharedPtr node, const std::string remote_name) : parameters_client(node, remote_name) {
    auto listdata = parameters_client.list_parameters({"name"}, 0);
    for(auto name : listdata.names) {
      name_vec.push_back(name.substr(5));
    }
  }
  std::vector<rclcpp::Parameter> get_description(){
    std::vector<rclcpp::Parameter> description;
    for (std::string name : name_vec) {
      auto param_des = parameters_client.get_parameters({
        "name." + name,
        "type." + name, 
        "default." + name, 
        "max." + name, 
        "min." + name, 
        "value." + name, 
        "des." + name,
        "lev." + name,
        "edit." + name,
      });
      description.insert(description.end(), param_des.begin(), param_des.end());
    }
    return description;
  }
  std::vector<rclcpp::Parameter> get_values(){
    
    std::vector<rclcpp::Parameter> values;
    for (std::string name : name_vec) {
      auto param_des = parameters_client.get_parameters({
        "value." + name,
        "type." + name,
      });
      values.insert(values.end(), param_des.begin(), param_des.end());
    }
    return values;
  }
  void update_params(std::vector<rclcpp::Parameter> para) {
    auto set_parameters_results = parameters_client.set_parameters(para);
  }
private:
  //static std::map<std::string, std::shared_ptr<Client>> check_map;
  rclcpp::SyncParametersClient parameters_client;
  std::vector<std::string> name_vec;
};

class Client_map {
public:
  std::map<std::string, std::shared_ptr<Client>> check_map;
  std::shared_ptr<Client> get_client(const rclcpp::Node::SharedPtr node, std::string remote_name) {
    //auto iter = check_map.find(remote_name);
    
    if (check_map.count(remote_name) != 0) {
      //check_map;
      return check_map.find(remote_name)->second;
      //return check_map[remote_name];
    }
    std::shared_ptr<Client> get = std::make_shared<Client>(node, remote_name);
    check_map[remote_name] = get;
    return get;
  }
};
static Client_map client_map;

} // rqt_reconfigure
