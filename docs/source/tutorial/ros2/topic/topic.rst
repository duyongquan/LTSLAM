.. highlight:: c++

.. default-domain:: cpp

====================
ROS2 Topic
====================


1 Publisher
====================


**not_composable**

.. code-block:: c++

    #include <chrono>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

    /* We do not recommend this style anymore, because composition of multiple
    * nodes in the same executable is not possible. Please see one of the subclass
    * examples for the "new" recommended styles. This example is only included
    * for completeness because it is similar to "classic" standalone ROS nodes. 
    */

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("minimal_publisher");
        auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
        std_msgs::msg::String message;
        auto publish_count = 0;
        rclcpp::WallRate loop_rate(500ms);

        while (rclcpp::ok()) 
        {
            message.data = "Hello, world! " + std::to_string(publish_count++);
            RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
            try {
            publisher->publish(message);
            rclcpp::spin_some(node);
            } catch (const rclcpp::exceptions::RCLError & e) {
            RCLCPP_ERROR(
                node->get_logger(),
                "unexpectedly failed with %s",
                e.what());
            }
            loop_rate.sleep();
        }
        rclcpp::shutdown();
        return 0;
    }

**lambda**

**member_function**

**member_function_with_type_adapter**

**member_function_with_unique_network_flow_endpoints**

**member_function_with_wait_for_all_acked**

2 Subscriber
====================