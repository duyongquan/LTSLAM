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
        std::shared_ptr<xslam::vins::VinsBuilderInterface> vins_builder,
        tf2_ros::Buffer* tf_buffer)
        : options_(node_options),
          vins_builder_bridge_(node_options, vins_builder, tf_buffer)
{

    // IMU trajectory
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(1),
        &Node::PublishIMUTrajectory, this));

    // Image trajectory
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(1),
        &Node::PublishImageTrajectory, this));

    // Ground Truth trajectory
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(1),
        &Node::PublishGroundTruthTrajectory, this));
}

Node::~Node()
{
    RunFinalOptimization();
}

void Node::RunFinalOptimization()
{

}

void Node::StartDefaultTopics()
{
    // IMU sensor message
    subscribers_.push_back({
        SubscribeWithHandler<sensor_msgs::Imu>(
             &Node::HandleImuMessage, kImuTopic,
             &node_handle_, this), kImuTopic
    });

    // Image sensor message
    subscribers_.push_back({
        SubscribeWithHandler<sensor_msgs::Image>(
             &Node::HandleImageMessage, kImageTopic,
             &node_handle_, this), kImageTopic
    });
}

void Node::HandleImuMessage(
    const std::string& sensor_id, 
    const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto sensor_bridge_ptr = vins_builder_bridge_.sensor_bridge();
    auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
    sensor_bridge_ptr->HandleImuMessage(sensor_id, std::move(imu_data_ptr));
}

void Node::HandleImageMessage(
    const std::string& sensor_id, 
    const sensor_msgs::Image::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto sensor_bridge_ptr = vins_builder_bridge_.sensor_bridge();
    auto image_data_ptr = sensor_bridge_ptr->ToImageData(msg);
    sensor_bridge_ptr->HandleImageMessage(sensor_id, std::move(image_data_ptr));
}

::ros::NodeHandle* Node::node_handle()
{
    return &node_handle_;
}

void Node::PublishIMUTrajectory(
    const ::ros::WallTimerEvent& unused_timer_event)
{

}

void Node::PublishImageTrajectory(
    const ::ros::WallTimerEvent& unused_timer_event)
{

}

void Node::PublishGroundTruthTrajectory(
    const ::ros::WallTimerEvent& unused_timer_event)
{

}

} // namespace vins_mono 
} // namespace xslam_ros
