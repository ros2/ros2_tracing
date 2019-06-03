#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

class ServiceNode : public rclcpp::Node
{
public:
    ServiceNode(rclcpp::NodeOptions options) : Node("service_node", options)
    {
        srv_ = this->create_service<std_srvs::srv::Empty>(
            "service",
            std::bind(&ServiceNode::service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));
    }

private:
    void service_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        const std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        // Nothing
        (void)request_header;
        (void)request;
        (void)response;
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto service_node = std::make_shared<ServiceNode>(rclcpp::NodeOptions());
    exec.add_node(service_node);

    printf("spinning once\n");
    exec.spin_once();

    rclcpp::shutdown();
    return 0;
}
