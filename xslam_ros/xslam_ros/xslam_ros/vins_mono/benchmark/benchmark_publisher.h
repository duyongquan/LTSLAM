#ifndef XSLAM_ROS_VINS_MONO_BENCHMARK_BENCHMARK_PUBLISHER_H
#define XSLAM_ROS_VINS_MONO_BENCHMARK_BENCHMARK_PUBLISHER_H

#include <string>
#include <vector>
#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace xslam_ros {
namespace vins_mono {
namespace benchmark {

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) != EOF)
        {
            t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};

class Benchmark
{
public:
    Benchmark(ros::NodeHandle& node);
    ~Benchmark();

    Benchmark(const Benchmark&) = delete;
    Benchmark& operator=(const Benchmark&) = delete;

    void PushlishGroundTruthPath();

private:

    void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);

    ros::NodeHandle node_;

    int idx = 1;
    vector<Data> benchmark;

    ros::Publisher pub_odom_;
    ros::Publisher pub_path_;
    nav_msgs::Path path;

    int init = 0;
    Quaterniond baseRgt;
    Vector3d baseTgt;
    tf::Transform trans;
};

} // namespace benchmark 
} // namespace vins_mono
} // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_BENCHMARK_BENCHMARK_PUBLISHER_H