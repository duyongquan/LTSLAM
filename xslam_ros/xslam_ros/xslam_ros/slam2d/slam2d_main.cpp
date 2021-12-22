#include "xslam_ros/slam2d/slam2d.h"

#include "ros/ros.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include <cmath>
#include <Eigen/Eigen>


namespace xslam_ros {
namespace {

slam2d::Slam2D slam;

ros::Publisher pub_pose, pub_path;
ros::Publisher pub_laserscan;
ros::Publisher pub_map2d;

void PublishMap2D(slam2d::Slam2D &slam);
void PublishPose(slam2d::Slam2D &slam);

void MultiEchoToLaserscan(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    //publish laserscan
    sensor_msgs::LaserScan laserscan;
    laserscan.header.stamp    = msg->header.stamp;
    laserscan.header.frame_id = "base_link";
    laserscan.angle_min       = msg->angle_min;
    laserscan.angle_max       = msg->angle_max;
    laserscan.angle_increment = msg->angle_increment;
    laserscan.time_increment  = msg->time_increment;
    laserscan.scan_time = msg->scan_time;
    laserscan.range_min = msg->range_min;
    laserscan.range_max = msg->range_max;
    laserscan.ranges.resize(msg->ranges.size());
    laserscan.intensities.resize(msg->ranges.size());
    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        laserscan.ranges[i] = msg->ranges[i].echoes[0];
        laserscan.intensities[i] = msg->intensities[i].echoes[0];
    }
    pub_laserscan.publish(laserscan);
}

void HandleLaserscanMessage(const sensor_msgs::LaserScanConstPtr &msg)
{
    slam.HandleLaserScanMessages(msg);
    slam.Update();
    PublishPose(slam);
    PublishMap2D(slam);
}

void HandleMultiEchoLaserscanMessage(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    slam.HandleLaserScanMessages(msg);
    slam.Update();
    PublishPose(slam);
    PublishMap2D(slam);
    MultiEchoToLaserscan(msg);
}


void PublishMap2D(slam2d::Slam2D &slam)
{
    slam.set_map2d_timestamp(ros::Time(slam.timestamp()));
    pub_map2d.publish(slam.GetOccupancyGridMap());
}

void PublishPose(slam2d::Slam2D &slam)
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(slam.timestamp());
    pose.header.frame_id = "odom";
    double theta = slam.state().theta;
    pose.pose.orientation.w = cos(0.5 * theta);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin(0.5 * theta);
    pose.pose.position.x = slam.state().t(0);
    pose.pose.position.y = slam.state().t(1);
    pose.pose.position.z = 0;
    pub_pose.publish(pose);

    path.header.frame_id = "odom";
    path.poses.push_back(pose);
    pub_path.publish(path);

    //send transfrom
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr;
    tr.header.stamp = ros::Time(slam.timestamp());
    tr.header.frame_id = "odom";
    tr.child_frame_id = "base_link";
    tr.transform.translation.x = slam.state().t(0);
    tr.transform.translation.y = slam.state().t(1);
    tr.transform.translation.z = 0;
    tr.transform.rotation.x = pose.pose.orientation.x;
    tr.transform.rotation.y = pose.pose.orientation.y;
    tr.transform.rotation.z = pose.pose.orientation.z;
    tr.transform.rotation.w = pose.pose.orientation.w;
    br.sendTransform(tr);
}

void Run() 
{
    ros::NodeHandle nh;

    ros::Subscriber sub_multiecho_laserscan = nh.subscribe<sensor_msgs::MultiEchoLaserScan>("/multiecho_scan", 100, HandleMultiEchoLaserscanMessage);
    ros::Subscriber sub_laserscan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, HandleLaserscanMessage);

    pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/laserscan", 100);
    pub_pose      = nh.advertise<geometry_msgs::PoseStamped>("/est_pose", 100);
    pub_path      = nh.advertise<nav_msgs::Path>("/path", 100);
    pub_map2d     = nh.advertise<nav_msgs::OccupancyGrid>("/map", 100);
    ros::spin();
}

}  // namespace
}  // namespace xslam_node

int main(int argc, char** argv) 
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ::ros::init(argc, argv, "xslam_node");
    ::ros::start();

    xslam_ros::Run();
    ::ros::shutdown();
    return 0;
}
