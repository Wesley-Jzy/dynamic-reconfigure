#include "rqt_support.hpp"
#include "uuid/uuid.h"

namespace rqt_reconfigure {

Client::Client(const rclcpp::Node::SharedPtr node, const std::string remote_name)
    : parameters_client(node, remote_name) {
        try {
            std::chrono::duration<int64_t, std::milli> timeout = std::chrono::duration<int64_t, std::milli>(500);
            parameters_client.wait_for_service(timeout);
            auto listdata = parameters_client.list_parameters({"name"}, 0);
            for(auto name : listdata.names) {
                name_vec.push_back(name.substr(5));
            }
        } catch (...) {
            printf("error");
        }
    }

std::vector<rclcpp::Parameter> Client::get_description() {
    std::vector<rclcpp::Parameter> description;
    try {
        for (std::string name : name_vec) {
            std::chrono::duration<int64_t, std::milli> timeout = std::chrono::duration<int64_t, std::milli>(500);
            parameters_client.wait_for_service(timeout);
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
    } catch(...) {
        printf("error");
    }

    return description;
}

std::vector<rclcpp::Parameter> Client::get_values(){
    std::vector<rclcpp::Parameter> values;
    try {
        for (std::string name : name_vec) {
            std::chrono::duration<int64_t, std::milli> timeout = std::chrono::duration<int64_t, std::milli>(500);
            parameters_client.wait_for_service(timeout);
            auto param_des = parameters_client.get_parameters({
                    "value." + name,
                    "type." + name,
                    });
            values.insert(values.end(), param_des.begin(), param_des.end());
        }
    } catch(...) {
        printf("error");
    }
    return values;
}

void Client::update_params(std::vector<rclcpp::Parameter> para) {
    try {
        std::chrono::duration<int64_t, std::milli> timeout = std::chrono::duration<int64_t, std::milli>(500);
        parameters_client.wait_for_service(timeout);
        auto set_parameters_results = parameters_client.set_parameters(para);
    } catch(...) {
        printf("error");
    }
}

std::shared_ptr<Client> Client_map::get_client(std::string remote_name) {
    if (check_map.count(remote_name) != 0) {
        return check_map.find(remote_name)->second;
    }

    uuid_t uuid;
    uuid_generate(uuid);
    char _str[36];
    uuid_unparse(uuid, _str);
    std::string uuid_name(_str);
    uuid_name.erase(std::remove(uuid_name.begin(), uuid_name.end(), '-'), uuid_name.end());

    auto node = rclcpp::Node::make_shared("get_parameters_try_client" + uuid_name);
    std::shared_ptr<Client> get = std::make_shared<Client>(node, remote_name);
    check_map[remote_name] = get;
    return get;
}

} //rqt_reconfigure namespace end
