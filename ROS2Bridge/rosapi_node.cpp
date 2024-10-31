// rosapi_node.cpp

#include "rosapi_node/rosapi_node.h"
#include <cstdlib>
#include <regex>

RosapiNode::RosapiNode() : Node("rosapi")
{
    this->declare_parameter<std::vector<std::string>>("topics_glob", {"*"});
    this->declare_parameter<std::vector<std::string>>("services_glob", {"*"});
    this->declare_parameter<std::vector<std::string>>("params_glob", {"*"});

    // 如果需要，初始化 globs、params 和 proxy 等价物

    register_services();
}

void RosapiNode::register_services()
{
    // 如果需要，初始化 proxy 和 params 等价物

    get_topics_service_ = this->create_service<rosapi_msgs::srv::Topics>(
        "/rosapi/topics",
        std::bind(&RosapiNode::handle_get_topics, this, std::placeholders::_1, std::placeholders::_2));

    get_topics_for_type_service_ = this->create_service<rosapi_msgs::srv::TopicsForType>(
        "/rosapi/topics_for_type",
        std::bind(&RosapiNode::handle_get_topics_for_type, this, std::placeholders::_1, std::placeholders::_2));

    get_topics_and_raw_types_service_ = this->create_service<rosapi_msgs::srv::TopicsAndRawTypes>(
        "/rosapi/topics_and_raw_types",
        std::bind(&RosapiNode::handle_get_topics_and_raw_types, this, std::placeholders::_1, std::placeholders::_2));

    get_services_service_ = this->create_service<rosapi_msgs::srv::Services>(
        "/rosapi/services",
        std::bind(&RosapiNode::handle_get_services, this, std::placeholders::_1, std::placeholders::_2));

    // 在 RosapiNode::register_services() 方法中

    get_services_for_type_service_ = this->create_service<rosapi_msgs::srv::ServicesForType>(
        "/rosapi/services_for_type",
        std::bind(&RosapiNode::handle_get_services_for_type, this, std::placeholders::_1, std::placeholders::_2));

    get_nodes_service_ = this->create_service<rosapi_msgs::srv::Nodes>(
        "/rosapi/nodes",
        std::bind(&RosapiNode::handle_get_nodes, this, std::placeholders::_1, std::placeholders::_2));

    get_node_details_service_ = this->create_service<rosapi_msgs::srv::NodeDetails>(
        "/rosapi/node_details",
        std::bind(&RosapiNode::handle_get_node_details, this, std::placeholders::_1, std::placeholders::_2));

    get_action_servers_service_ = this->create_service<rosapi_msgs::srv::GetActionServers>(
        "/rosapi/action_servers",
        std::bind(&RosapiNode::handle_get_action_servers, this, std::placeholders::_1, std::placeholders::_2));

    get_topic_type_service_ = this->create_service<rosapi_msgs::srv::TopicType>(
        "/rosapi/topic_type",
        std::bind(&RosapiNode::handle_get_topic_type, this, std::placeholders::_1, std::placeholders::_2));

    get_service_type_service_ = this->create_service<rosapi_msgs::srv::ServiceType>(
        "/rosapi/service_type",
        std::bind(&RosapiNode::handle_get_service_type, this, std::placeholders::_1, std::placeholders::_2));

    get_publishers_service_ = this->create_service<rosapi_msgs::srv::Publishers>(
        "/rosapi/publishers",
        std::bind(&RosapiNode::handle_get_publishers, this, std::placeholders::_1, std::placeholders::_2));

    get_subscribers_service_ = this->create_service<rosapi_msgs::srv::Subscribers>(
        "/rosapi/subscribers",
        std::bind(&RosapiNode::handle_get_subscribers, this, std::placeholders::_1, std::placeholders::_2));

    get_service_providers_service_ = this->create_service<rosapi_msgs::srv::ServiceProviders>(
        "/rosapi/service_providers",
        std::bind(&RosapiNode::handle_get_service_providers, this, std::placeholders::_1, std::placeholders::_2));

    get_service_node_service_ = this->create_service<rosapi_msgs::srv::ServiceNode>(
        "/rosapi/service_node",
        std::bind(&RosapiNode::handle_get_service_node, this, std::placeholders::_1, std::placeholders::_2));

    get_message_details_service_ = this->create_service<rosapi_msgs::srv::MessageDetails>(
        "/rosapi/message_details",
        std::bind(&RosapiNode::handle_get_message_details, this, std::placeholders::_1, std::placeholders::_2));

    get_service_request_details_service_ = this->create_service<rosapi_msgs::srv::ServiceRequestDetails>(
        "/rosapi/service_request_details",
        std::bind(&RosapiNode::handle_get_service_request_details, this, std::placeholders::_1, std::placeholders::_2));

    get_service_response_details_service_ = this->create_service<rosapi_msgs::srv::ServiceResponseDetails>(
        "/rosapi/service_response_details",
        std::bind(&RosapiNode::handle_get_service_response_details, this, std::placeholders::_1, std::placeholders::_2));


    set_param_service_ = this->create_service<rosapi_msgs::srv::SetParam>(
        "/rosapi/set_param",
        std::bind(&RosapiNode::handle_set_param, this, std::placeholders::_1, std::placeholders::_2));

    get_param_service_ = this->create_service<rosapi_msgs::srv::GetParam>(
        "/rosapi/get_param",
        std::bind(&RosapiNode::handle_get_param, this, std::placeholders::_1, std::placeholders::_2));

    has_param_service_ = this->create_service<rosapi_msgs::srv::HasParam>(
        "/rosapi/has_param",
        std::bind(&RosapiNode::handle_has_param, this, std::placeholders::_1, std::placeholders::_2));

    delete_param_service_ = this->create_service<rosapi_msgs::srv::DeleteParam>(
        "/rosapi/delete_param",
        std::bind(&RosapiNode::handle_delete_param, this, std::placeholders::_1, std::placeholders::_2));

    get_param_names_service_ = this->create_service<rosapi_msgs::srv::GetParamNames>(
        "/rosapi/get_param_names",
        std::bind(&RosapiNode::handle_get_param_names, this, std::placeholders::_1, std::placeholders::_2));

    get_time_service_ = this->create_service<rosapi_msgs::srv::GetTime>(
        "/rosapi/get_time",
        std::bind(&RosapiNode::handle_get_time, this, std::placeholders::_1, std::placeholders::_2));

    get_ros_version_service_ = this->create_service<rosapi_msgs::srv::GetROSVersion>(
        "/rosapi/get_ros_version",
        std::bind(&RosapiNode::handle_get_ros_version, this, std::placeholders::_1, std::placeholders::_2));
}

// 以下是服务回调函数的实现
void RosapiNode::handle_get_topics(const std::shared_ptr<rosapi_msgs::srv::Topics::Request> /*request*/,
                                   std::shared_ptr<rosapi_msgs::srv::Topics::Response> response)
{
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &topic : topics_and_types)
    {
        response->topics.push_back(topic.first);
        response->types.push_back(topic.second[0]);
    }
}

void RosapiNode::handle_get_topics_for_type(const std::shared_ptr<rosapi_msgs::srv::TopicsForType::Request> request,
                                            std::shared_ptr<rosapi_msgs::srv::TopicsForType::Response> response)
{
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &topic : topics_and_types)
    {
        if (std::find(topic.second.begin(), topic.second.end(), request->type) != topic.second.end())
        {
            response->topics.push_back(topic.first);
        }
    }
}

void RosapiNode::handle_get_topics_and_raw_types(const std::shared_ptr<rosapi_msgs::srv::TopicsAndRawTypes::Request> /*request*/,
                                                 std::shared_ptr<rosapi_msgs::srv::TopicsAndRawTypes::Response> response)
{
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &topic : topics_and_types)
    {
        response->topics.push_back(topic.first);
        response->types.push_back(topic.second[0]);
        // 这里假设 typedefs_full_text 可以从类型中获取，这可能需要额外的实现
        response->typedefs_full_text.push_back("typedef for " + topic.second[0]);
    }
}


// 获取所有服务
void RosapiNode::handle_get_services(const std::shared_ptr<rosapi_msgs::srv::Services::Request> /*request*/,
                                     std::shared_ptr<rosapi_msgs::srv::Services::Response> response)
{
    auto services_and_types = this->get_service_names_and_types();
    for (const auto &service : services_and_types)
    {
        response->services.push_back(service.first);
    }
}

// 根据类型获取服务
void RosapiNode::handle_get_services_for_type(const std::shared_ptr<rosapi_msgs::srv::ServicesForType::Request> request,
                                              std::shared_ptr<rosapi_msgs::srv::ServicesForType::Response> response)
{
    auto services_and_types = this->get_service_names_and_types();
    for (const auto &service : services_and_types)
    {
        if (std::find(service.second.begin(), service.second.end(), request->type) != service.second.end())
        {
            response->services.push_back(service.first);
        }
    }
}

// 获取所有节点
void RosapiNode::handle_get_nodes(const std::shared_ptr<rosapi_msgs::srv::Nodes::Request> /*request*/,
                                  std::shared_ptr<rosapi_msgs::srv::Nodes::Response> response)
{
    auto node_names = this->get_node_names();
    response->nodes = node_names;
}

// 返回指定节点的详细信息，包括发布的主题、订阅的主题和提供的服务
void RosapiNode::handle_get_node_details(const std::shared_ptr<rosapi_msgs::srv::NodeDetails::Request> request,
                                         std::shared_ptr<rosapi_msgs::srv::NodeDetails::Response> response)
{
    const std::string target_node_name = request->node;

    // 获取系统中的所有主题并过滤出该节点的发布者和订阅者
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &topic : topics_and_types)
    {
        // 获取主题的发布者信息
        auto publishers_info = this->get_publishers_info_by_topic(topic.first);
        for (const auto &publisher : publishers_info)
        {
            if (publisher.node_name() == target_node_name)
            {
                response->publishing.push_back(topic.first); // 添加发布的主题
            }
        }

        // 获取主题的订阅者信息
        auto subscribers_info = this->get_subscriptions_info_by_topic(topic.first);
        for (const auto &subscriber : subscribers_info)
        {
            if (subscriber.node_name() == target_node_name)
            {
                response->subscribing.push_back(topic.first); // 添加订阅的主题
            }
        }
    }

    // 获取系统中的所有服务并过滤出该节点提供的服务
    auto services_and_types = this->get_service_names_and_types();
    for (const auto &service : services_and_types)
    {
        // 使用 get_service_names_and_types_by_node 检查是否该节点提供此服务
        auto node_services = this->get_service_names_and_types_by_node(target_node_name, this->get_namespace());
        if (node_services.find(service.first) != node_services.end())
        {
            response->services.push_back(service.first); // 添加提供的服务
        }
    }
}

// 获取所有 action 服务器
void RosapiNode::handle_get_action_servers(const std::shared_ptr<rosapi_msgs::srv::GetActionServers::Request> /*request*/,
                                           std::shared_ptr<rosapi_msgs::srv::GetActionServers::Response> response)
{
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &topic : topics_and_types)
    {
        // 假设 action 服务器主题包含 "/action/" 以便进行筛选
        if (topic.first.find("/action/") != std::string::npos)
        {
            response->action_servers.push_back(topic.first);
        }
    }
}

// 获取主题的类型
void RosapiNode::handle_get_topic_type(const std::shared_ptr<rosapi_msgs::srv::TopicType::Request> request,
                                       std::shared_ptr<rosapi_msgs::srv::TopicType::Response> response)
{
    auto topics_and_types = this->get_topic_names_and_types();
    auto it = topics_and_types.find(request->topic);
    if (it != topics_and_types.end())
    {
        response->type = it->second[0];
    }
    else
    {
        response->type = "";  // 主题未找到时返回空字符串
    }
}

// 获取服务的类型
void RosapiNode::handle_get_service_type(const std::shared_ptr<rosapi_msgs::srv::ServiceType::Request> request,
                                         std::shared_ptr<rosapi_msgs::srv::ServiceType::Response> response)
{
    auto services_and_types = this->get_service_names_and_types();
    auto it = services_and_types.find(request->service);
    if (it != services_and_types.end())
    {
        response->type = it->second[0];
    }
    else
    {
        response->type = "";  // 服务未找到时返回空字符串
    }
}

// 获取指定主题的发布者列表
void RosapiNode::handle_get_publishers(const std::shared_ptr<rosapi_msgs::srv::Publishers::Request> request,
                                       std::shared_ptr<rosapi_msgs::srv::Publishers::Response> response)
{
    auto publishers_info = this->get_publishers_info_by_topic(request->topic);
    for (const auto &pub_info : publishers_info)
    {
        // 提取发布者的节点名称并添加到响应中
        response->publishers.push_back(pub_info.node_name());
    }
}

// 获取指定主题的订阅者列表
void RosapiNode::handle_get_subscribers(const std::shared_ptr<rosapi_msgs::srv::Subscribers::Request> request,
                                        std::shared_ptr<rosapi_msgs::srv::Subscribers::Response> response)
{
    try
    {
        auto subscriptions_info = this->get_subscriptions_info_by_topic(request->topic);

        for (const auto &info : subscriptions_info)
        {
            response->subscribers.push_back(info.node_name());  // 只获取订阅者的节点名称
        }
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get subscribers for topic '%s': %s",
                     request->topic.c_str(), e.what());
    }
}


void RosapiNode::handle_get_service_providers(const std::shared_ptr<rosapi_msgs::srv::ServiceProviders::Request> request,
                                              std::shared_ptr<rosapi_msgs::srv::ServiceProviders::Response> response)
{
    try
    {
        // 获取所有服务名称和其广告的节点名称
        auto services_and_types = this->get_service_names_and_types();

        // 遍历所有服务，并检查是否包含请求的服务名称
        for (const auto &service_entry : services_and_types)
        {
            const std::string &service_name = service_entry.first;

            // 如果找到与请求服务名称相同的服务
            if (service_name == request->service)
            {
                // 使用 get_service_names_and_types_by_node 获取提供该服务的节点
                auto nodes_for_service = this->get_service_names_and_types_by_node(service_name, this->get_namespace());
                for (const auto &node : nodes_for_service)
                {
                    response->providers.push_back(node.first);  // 将节点名称添加到响应
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get service providers for '%s': %s",
                     request->service.c_str(), e.what());
    }
}

// 根据服务获取节点名称
void RosapiNode::handle_get_service_node(const std::shared_ptr<rosapi_msgs::srv::ServiceNode::Request> request,
                                         std::shared_ptr<rosapi_msgs::srv::ServiceNode::Response> response)
{
    try
    {
        // 获取所有已知节点的名称和命名空间
        auto all_nodes = this->get_node_names();

        for (const auto &node : all_nodes)
        {
            // 查询该节点提供的所有服务
            auto services = this->get_service_names_and_types_by_node(node, this->get_namespace());

            // 检查该节点是否提供了请求的服务
            auto it = services.find(request->service);
            if (it != services.end())
            {
                // 如果找到该服务，返回提供该服务的节点名称
                response->node = node;
                return;
            }
        }
        // 如果没有找到服务提供者，返回空字符串
        response->node = "";
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to find node for service '%s': %s",
                     request->service.c_str(), e.what());
        response->node = "";
    }
}

// 获取消息类型的详细信息
void RosapiNode::handle_get_message_details(const std::shared_ptr<rosapi_msgs::srv::MessageDetails::Request> request,
                                            std::shared_ptr<rosapi_msgs::srv::MessageDetails::Response> response)
{
    // 此处应获取消息类型定义并填充 response->typedefs
    // 由于ROS 2 C++缺少动态消息类型支持，您需要手动填充此信息
    // ToDo
    rosapi_msgs::msg::TypeDef type_def;
    type_def.type = request->type;
    type_def.fieldnames = {"example_field1", "example_field2"};
    type_def.fieldtypes = {"string", "int32"};
    response->typedefs.push_back(type_def);
}

// 获取服务请求的详细信息
void RosapiNode::handle_get_service_request_details(const std::shared_ptr<rosapi_msgs::srv::ServiceRequestDetails::Request> request,
                                                    std::shared_ptr<rosapi_msgs::srv::ServiceRequestDetails::Response> response)
{
    // 您可以通过编写自定义方法获取请求的TypeDef信息
    // ToDo
    rosapi_msgs::msg::TypeDef request_type_def;
    request_type_def.type = request->type + "Request";
    request_type_def.fieldnames = {"request_field1", "request_field2"};
    request_type_def.fieldtypes = {"string", "float64"};
    response->typedefs.push_back(request_type_def);
}

// 获取服务响应的详细信息
void RosapiNode::handle_get_service_response_details(const std::shared_ptr<rosapi_msgs::srv::ServiceResponseDetails::Request> request,
                                                     std::shared_ptr<rosapi_msgs::srv::ServiceResponseDetails::Response> response)
{
    // 同样，您可以通过编写自定义方法获取响应的TypeDef信息

    // ToDo
    rosapi_msgs::msg::TypeDef response_type_def;
    response_type_def.type = request->type + "Response";
    response_type_def.fieldnames = {"response_field1", "response_field2"};
    response_type_def.fieldtypes = {"bool", "float64"};
    response->typedefs.push_back(response_type_def);
}

void RosapiNode::handle_set_param(const std::shared_ptr<rosapi_msgs::srv::SetParam::Request> request,
                                  std::shared_ptr<rosapi_msgs::srv::SetParam::Response> /*response*/)
{
    // 设置参数
    this->set_parameter(rclcpp::Parameter(request->name, request->value));
    // 不需要设置 response，因为响应为空
}


// 获取参数
void RosapiNode::handle_get_param(const std::shared_ptr<rosapi_msgs::srv::GetParam::Request> request,
                                  std::shared_ptr<rosapi_msgs::srv::GetParam::Response> response)
{
    if (this->has_parameter(request->name))
    {
        response->value = this->get_parameter(request->name).get_value<std::string>();
    }
    else
    {
        response->value = request->default_value;
        RCLCPP_WARN(this->get_logger(), "Parameter not found: %s. Returning default value.", request->name.c_str());
    }
}

// 检查参数是否存在
void RosapiNode::handle_has_param(const std::shared_ptr<rosapi_msgs::srv::HasParam::Request> request,
                                  std::shared_ptr<rosapi_msgs::srv::HasParam::Response> response)
{
    response->exists = this->has_parameter(request->name);
}

void RosapiNode::handle_delete_param(const std::shared_ptr<rosapi_msgs::srv::DeleteParam::Request> request,
                                     std::shared_ptr<rosapi_msgs::srv::DeleteParam::Response> /*response*/)
{
    if (this->has_parameter(request->name))
    {
        this->undeclare_parameter(request->name);
        // 不需要设置 response，因为响应为空
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Parameter not found for deletion: %s", request->name.c_str());
        // 不需要设置 response，因为响应为空
    }
}

// 获取所有参数名称
void RosapiNode::handle_get_param_names(const std::shared_ptr<rosapi_msgs::srv::GetParamNames::Request> /*request*/,
                                        std::shared_ptr<rosapi_msgs::srv::GetParamNames::Response> response)
{
    auto param_list = this->list_parameters({}, 10);
    response->names = param_list.names;
}

// 获取当前时间
void RosapiNode::handle_get_time(const std::shared_ptr<rosapi_msgs::srv::GetTime::Request> /*request*/,
                                 std::shared_ptr<rosapi_msgs::srv::GetTime::Response> response)
{
    response->time = this->now();
}

// 获取 ROS 版本
void RosapiNode::handle_get_ros_version(const std::shared_ptr<rosapi_msgs::srv::GetROSVersion::Request> /*request*/,
                                        std::shared_ptr<rosapi_msgs::srv::GetROSVersion::Response> response)
{
    response->version = 2;
    response->distro = std::string(getenv("ROS_DISTRO"));
}


// 辅助方法
std::pair<std::string, std::string> RosapiNode::get_node_and_param_name(const std::string &param)
{
    auto pos = param.find(":");
    if (pos == std::string::npos)
    {
        throw std::invalid_argument("Malformed parameter name");
    }
    return {param.substr(0, pos), param.substr(pos + 1)};
}

void RosapiNode::print_malformed_param_name_warning(const std::string &param_name)
{
    RCLCPP_WARN(this->get_logger(), "Malformed parameter name: %s; expecting <node_name>:<param_name>", param_name.c_str());
}

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosapiNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
