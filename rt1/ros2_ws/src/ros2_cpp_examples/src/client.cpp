#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient()
        : Node("add_two_ints_client")
    {
        client_ = this->create_client<AddTwoInts>("add_two_ints");

        // Wait for the service to be available
        wait_for_service();

        // Create and send a request
        send_request(41, 1);
    }

private:
    void wait_for_service()
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
                throw std::runtime_error("Service not available");
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
    }

    void send_request(int64_t a, int64_t b)
    {
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto result_future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed :(");
            client_->remove_pending_request(result_future);
            return;
        }

        auto result = result_future.get();
        RCLCPP_INFO(
            this->get_logger(),
            "Result of %" PRId64 " + %" PRId64 " = %" PRId64,
            request->a, request->b, result->sum);
    }

    rclcpp::Client<AddTwoInts>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<AddTwoIntsClient>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
