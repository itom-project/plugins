// rosapi_node.h

#ifndef ROSAPI_NODE_H
#define ROSAPI_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>

#include <rosidl_runtime_cpp/traits.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

// Contains all the required services and message types
#include <rosapi_msgs/msg/type_def.hpp>
#include <rosapi_msgs/srv/delete_param.hpp>
#include <rosapi_msgs/srv/get_action_servers.hpp>
#include <rosapi_msgs/srv/get_param.hpp>
#include <rosapi_msgs/srv/get_param_names.hpp>
#include <rosapi_msgs/srv/get_ros_version.hpp>
#include <rosapi_msgs/srv/get_time.hpp>
#include <rosapi_msgs/srv/has_param.hpp>
#include <rosapi_msgs/srv/message_details.hpp>
#include <rosapi_msgs/srv/node_details.hpp>
#include <rosapi_msgs/srv/nodes.hpp>
#include <rosapi_msgs/srv/publishers.hpp>
#include <rosapi_msgs/srv/service_node.hpp>
#include <rosapi_msgs/srv/service_providers.hpp>
#include <rosapi_msgs/srv/service_request_details.hpp>
#include <rosapi_msgs/srv/service_response_details.hpp>
#include <rosapi_msgs/srv/services.hpp>
#include <rosapi_msgs/srv/services_for_type.hpp>
#include <rosapi_msgs/srv/service_type.hpp>
#include <rosapi_msgs/srv/set_param.hpp>
#include <rosapi_msgs/srv/subscribers.hpp>
#include <rosapi_msgs/srv/topics.hpp>
#include <rosapi_msgs/srv/topics_and_raw_types.hpp>
#include <rosapi_msgs/srv/topics_for_type.hpp>
#include <rosapi_msgs/srv/topic_type.hpp>

class RosapiNode : public rclcpp::Node
{
public:
    RosapiNode();

private:
    void register_services();

    // Service callback function
    void handle_get_topics(const std::shared_ptr<rosapi_msgs::srv::Topics::Request> request,
                           std::shared_ptr<rosapi_msgs::srv::Topics::Response> response);

    void handle_get_topics_for_type(const std::shared_ptr<rosapi_msgs::srv::TopicsForType::Request> request,
                                    std::shared_ptr<rosapi_msgs::srv::TopicsForType::Response> response);

    void handle_get_topics_and_raw_types(const std::shared_ptr<rosapi_msgs::srv::TopicsAndRawTypes::Request> request,
                                         std::shared_ptr<rosapi_msgs::srv::TopicsAndRawTypes::Response> response);

    void handle_get_services(const std::shared_ptr<rosapi_msgs::srv::Services::Request> request,
                             std::shared_ptr<rosapi_msgs::srv::Services::Response> response);

    void handle_get_services_for_type(const std::shared_ptr<rosapi_msgs::srv::ServicesForType::Request> request,
                                      std::shared_ptr<rosapi_msgs::srv::ServicesForType::Response> response);

    void handle_get_nodes(const std::shared_ptr<rosapi_msgs::srv::Nodes::Request> request,
                          std::shared_ptr<rosapi_msgs::srv::Nodes::Response> response);

    void handle_get_node_details(const std::shared_ptr<rosapi_msgs::srv::NodeDetails::Request> request,
                                 std::shared_ptr<rosapi_msgs::srv::NodeDetails::Response> response);

    void handle_get_action_servers(const std::shared_ptr<rosapi_msgs::srv::GetActionServers::Request> request,
                                   std::shared_ptr<rosapi_msgs::srv::GetActionServers::Response> response);

    void handle_get_topic_type(const std::shared_ptr<rosapi_msgs::srv::TopicType::Request> request,
                               std::shared_ptr<rosapi_msgs::srv::TopicType::Response> response);

    void handle_get_service_type(const std::shared_ptr<rosapi_msgs::srv::ServiceType::Request> request,
                                 std::shared_ptr<rosapi_msgs::srv::ServiceType::Response> response);

    void handle_get_publishers(const std::shared_ptr<rosapi_msgs::srv::Publishers::Request> request,
                               std::shared_ptr<rosapi_msgs::srv::Publishers::Response> response);

    void handle_get_subscribers(const std::shared_ptr<rosapi_msgs::srv::Subscribers::Request> request,
                                std::shared_ptr<rosapi_msgs::srv::Subscribers::Response> response);

    void handle_get_service_providers(const std::shared_ptr<rosapi_msgs::srv::ServiceProviders::Request> request,
                                      std::shared_ptr<rosapi_msgs::srv::ServiceProviders::Response> response);

    void handle_get_service_node(const std::shared_ptr<rosapi_msgs::srv::ServiceNode::Request> request,
                                 std::shared_ptr<rosapi_msgs::srv::ServiceNode::Response> response);

    void handle_get_message_details(const std::shared_ptr<rosapi_msgs::srv::MessageDetails::Request> request,
                                    std::shared_ptr<rosapi_msgs::srv::MessageDetails::Response> response);

    void handle_get_service_request_details(const std::shared_ptr<rosapi_msgs::srv::ServiceRequestDetails::Request> request,
                                            std::shared_ptr<rosapi_msgs::srv::ServiceRequestDetails::Response> response);

    void handle_get_service_response_details(const std::shared_ptr<rosapi_msgs::srv::ServiceResponseDetails::Request> request,
                                             std::shared_ptr<rosapi_msgs::srv::ServiceResponseDetails::Response> response);

    void handle_set_param(const std::shared_ptr<rosapi_msgs::srv::SetParam::Request> request,
                          std::shared_ptr<rosapi_msgs::srv::SetParam::Response> response);

    void handle_get_param(const std::shared_ptr<rosapi_msgs::srv::GetParam::Request> request,
                          std::shared_ptr<rosapi_msgs::srv::GetParam::Response> response);

    void handle_has_param(const std::shared_ptr<rosapi_msgs::srv::HasParam::Request> request,
                          std::shared_ptr<rosapi_msgs::srv::HasParam::Response> response);

    void handle_delete_param(const std::shared_ptr<rosapi_msgs::srv::DeleteParam::Request> request,
                             std::shared_ptr<rosapi_msgs::srv::DeleteParam::Response> response);

    void handle_get_param_names(const std::shared_ptr<rosapi_msgs::srv::GetParamNames::Request> request,
                                std::shared_ptr<rosapi_msgs::srv::GetParamNames::Response> response);

    void handle_get_time(const std::shared_ptr<rosapi_msgs::srv::GetTime::Request> request,
                         std::shared_ptr<rosapi_msgs::srv::GetTime::Response> response);

    void handle_get_ros_version(const std::shared_ptr<rosapi_msgs::srv::GetROSVersion::Request> request,
                                std::shared_ptr<rosapi_msgs::srv::GetROSVersion::Response> response);

    // Ancillary methods
    std::pair<std::string, std::string> get_node_and_param_name(const std::string &param);

    void print_malformed_param_name_warning(const std::string &param_name);

    // Service member variables
    rclcpp::Service<rosapi_msgs::srv::Topics>::SharedPtr get_topics_service_;
    rclcpp::Service<rosapi_msgs::srv::TopicsForType>::SharedPtr get_topics_for_type_service_;
    rclcpp::Service<rosapi_msgs::srv::TopicsAndRawTypes>::SharedPtr get_topics_and_raw_types_service_;
    rclcpp::Service<rosapi_msgs::srv::Services>::SharedPtr get_services_service_;
    rclcpp::Service<rosapi_msgs::srv::ServicesForType>::SharedPtr get_services_for_type_service_;
    rclcpp::Service<rosapi_msgs::srv::Nodes>::SharedPtr get_nodes_service_;
    rclcpp::Service<rosapi_msgs::srv::NodeDetails>::SharedPtr get_node_details_service_;
    rclcpp::Service<rosapi_msgs::srv::GetActionServers>::SharedPtr get_action_servers_service_;
    rclcpp::Service<rosapi_msgs::srv::TopicType>::SharedPtr get_topic_type_service_;
    rclcpp::Service<rosapi_msgs::srv::ServiceType>::SharedPtr get_service_type_service_;
    rclcpp::Service<rosapi_msgs::srv::Publishers>::SharedPtr get_publishers_service_;
    rclcpp::Service<rosapi_msgs::srv::Subscribers>::SharedPtr get_subscribers_service_;
    rclcpp::Service<rosapi_msgs::srv::ServiceProviders>::SharedPtr get_service_providers_service_;
    rclcpp::Service<rosapi_msgs::srv::ServiceNode>::SharedPtr get_service_node_service_;
    rclcpp::Service<rosapi_msgs::srv::MessageDetails>::SharedPtr get_message_details_service_;
    rclcpp::Service<rosapi_msgs::srv::ServiceRequestDetails>::SharedPtr get_service_request_details_service_;
    rclcpp::Service<rosapi_msgs::srv::ServiceResponseDetails>::SharedPtr get_service_response_details_service_;
    rclcpp::Service<rosapi_msgs::srv::SetParam>::SharedPtr set_param_service_;
    rclcpp::Service<rosapi_msgs::srv::GetParam>::SharedPtr get_param_service_;
    rclcpp::Service<rosapi_msgs::srv::HasParam>::SharedPtr has_param_service_;
    rclcpp::Service<rosapi_msgs::srv::DeleteParam>::SharedPtr delete_param_service_;
    rclcpp::Service<rosapi_msgs::srv::GetParamNames>::SharedPtr get_param_names_service_;
    rclcpp::Service<rosapi_msgs::srv::GetTime>::SharedPtr get_time_service_;
    rclcpp::Service<rosapi_msgs::srv::GetROSVersion>::SharedPtr get_ros_version_service_;

};

#endif // ROSAPI_NODE_H
