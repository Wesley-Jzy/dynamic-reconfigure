#ifndef RQT_SUPPORT_H
#define RQT_SUPPORT_H

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <vector>
#include <map>

namespace rqt_reconfigure {

class Client {
    public:
        Client(const rclcpp::Node::SharedPtr node, const std::string remote_name);

        std::vector<rclcpp::Parameter> get_description();

        std::vector<rclcpp::Parameter> get_values();

        void update_params(std::vector<rclcpp::Parameter> para);

    private:
        //static std::map<std::string, std::shared_ptr<Client>> check_map;
        rclcpp::SyncParametersClient parameters_client;
        std::vector<std::string> name_vec;
};

class Client_map {
    public:
        std::map<std::string, std::shared_ptr<Client>> check_map;

        std::shared_ptr<Client> get_client(std::string remote_name);
};

static Client_map client_map;

} // rqt_reconfigure namespace end

#endif
