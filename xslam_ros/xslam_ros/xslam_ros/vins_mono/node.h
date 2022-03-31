#ifndef XSLAM_ROS_VINS_MONO_NODE_H
#define XSLAM_ROS_VINS_MONO_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "xslam_ros/vins_mono/node_constants.h"
#include "xslam_ros/vins_mono/node_options.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace xslam_ros {
namespace vins_mono {

class Node 
{
public:
    Node(const NodeOptions& node_options,
        //  std::unique_ptr<cartographer::mapping::VinsBuilderInterface> map_builder,
        tf2_ros::Buffer* tf_buffer);
    ~Node(){};

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    // Runs final optimization.
    void RunFinalOptimization();

    // Starts the first trajectory with the default topics.
    void StartDefaultTopics();

    void HandleImuMessage(const std::string& sensor_id, const sensor_msgs::Imu::ConstPtr& msg);
    void HandleImageMessage(const std::string& sensor_id, const sensor_msgs::Image::ConstPtr& msg);

    ::ros::NodeHandle* node_handle();


private:
    struct Subscriber 
    {
        ::ros::Subscriber subscriber;

        // ::ros::Subscriber::getTopic() does not necessarily return the same
        // std::string
        // it was given in its constructor. Since we rely on the topic name as the
        // unique identifier of a subscriber, we remember it ourselves.
        std::string topic;
    };


    ::ros::NodeHandle node_handle_;
    std::unordered_map<int, std::vector<Subscriber>> subscribers_;
    std::unordered_set<std::string> subscribed_topics_;

    // We have to keep the timer handles of ::ros::WallTimers around, otherwise
    // they do not fire.
    std::vector<::ros::WallTimer> wall_timers_;

};

} // namespace vins_mono 
} // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_NODE_H