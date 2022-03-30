#include "xslam_ros/vins_mono/node.h"

namespace xslam_ros {
namespace vins_mono {

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(const std::string&,
                          const typename MessageType::ConstPtr&),
                          const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) 
{
    return node_handle->subscribe<MessageType>(
        topic, kInfiniteSubscriberQueueSize,
        boost::function<void(const typename MessageType::ConstPtr&)>(
            [node, handler,
            topic](const typename MessageType::ConstPtr& msg) {
                (node->*handler)(topic, msg);
            }));
}
} // namespace

Node::Node(const NodeOptions& node_options,
        //  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
        tf2_ros::Buffer* tf_buffer)
{

}

void Node::RunFinalOptimization()
{

}

void Node::StartDefaultTopics()
{

}

void Node::HandleImuMessage(const std::string& sensor_id, const sensor_msgs::Imu::ConstPtr& msg)
{

}

::ros::NodeHandle* Node::node_handle()
{
    return &node_handle_;
}


} // namespace vins_mono 
} // namespace xslam_ros
