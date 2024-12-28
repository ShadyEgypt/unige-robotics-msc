#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsService : public rclcpp::Node
{
public:
    AddTwoIntsService()
        : Node("add_two_ints_service")
    {
        // Create the service
        service_ = this->create_service<AddTwoInts>(
            "add_two_ints",
            std::bind(
                &AddTwoIntsService::handle_service,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "Service 'add_two_ints' is ready.");
    }

private:
    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<AddTwoInts::Request> request,
        const std::shared_ptr<AddTwoInts::Response> response)
    {
        (void)request_header; // Unused in this example
        RCLCPP_INFO(
            this->get_logger(),
            "Request: %" PRId64 " + %" PRId64,
            request->a, request->b);

        response->sum = request->a + request->b;
    }

    rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
