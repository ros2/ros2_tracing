#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"


class PongNode : public rclcpp::Node
{
public:
    PongNode(rclcpp::NodeOptions options) : Node("pong_node", options)
    {
        srv_ = this->create_service<std_srvs::srv::Empty>(
            "ping",
            std::bind(&PongNode::service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));
        client_ = this->create_client<std_srvs::srv::Empty>(
            "pong");
    }


private:
    void service_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        const std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request_header;
        (void)request;
        (void)response;
        RCLCPP_INFO(this->get_logger(), "got request");
        auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        client_->async_send_request(req);
        rclcpp::shutdown();
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto pong_node = std::make_shared<PongNode>(rclcpp::NodeOptions());
    exec.add_node(pong_node);

    printf("spinning\n");
    exec.spin();

    // Will actually be called inside the node's service callback
    rclcpp::shutdown();
    return 0;
}
