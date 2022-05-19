#include "xslam_ros/vins_mono/benchmark/benchmark_publisher.h"

namespace xslam_ros {
namespace vins_mono {
namespace benchmark {

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}


Benchmark::Benchmark(ros::NodeHandle& node)
{
    // string csv_file = readParam<string>(node, "data_name");

    string csv_file = "/home/quan/workspace/slam/source/LTSLAM/xslam_ros/xslam_ros/configuration_files/benchmark/MH_01_easy/data.csv";

    std::cout << "load ground truth " << csv_file << std::endl;
    FILE *f = fopen(csv_file.c_str(), "r");
    if (f==NULL)
    {
      ROS_WARN("can't load ground truth; wrong path");
      //std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
      return;
    }
    char tmp[10000];
    if (fgets(tmp, 10000, f) == NULL)
    {
        ROS_WARN("can't load ground truth; no data available");
    }
    while (!feof(f))
        benchmark.emplace_back(f);
    fclose(f);
    benchmark.pop_back();
    ROS_INFO("Data loaded: %d", (int)benchmark.size());

    pub_odom_ = node.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_path_ = node.advertise<nav_msgs::Path>("path", 1000);

    // ros::Subscriber sub_odom = n.subscribe("estimated_odometry", 1000, odom_callback);

}

Benchmark::~Benchmark()
{}

void Benchmark::PushlishGroundTruthPath()
{
    // if (odom_msg->header.stamp.toSec() > benchmark.back().t)
    //   return;
  
    // for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= odom_msg->header.stamp.toSec(); idx++)
    //     ;


    // if (init++ < SKIP)
    // {
    //     baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
    //                           odom_msg->pose.pose.orientation.x,
    //                           odom_msg->pose.pose.orientation.y,
    //                           odom_msg->pose.pose.orientation.z) *
    //               Quaterniond(benchmark[idx - 1].qw,
    //                           benchmark[idx - 1].qx,
    //                           benchmark[idx - 1].qy,
    //                           benchmark[idx - 1].qz).inverse();
    //     baseTgt = Vector3d{odom_msg->pose.pose.position.x,
    //                        odom_msg->pose.pose.position.y,
    //                        odom_msg->pose.pose.position.z} -
    //               baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    //     return;
    // }

    idx++;
    if (idx >= static_cast<int>(benchmark.size())) {
        return;
    }

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(benchmark[idx - 1].t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Vector3d tmp_V = baseRgt * Vector3d{benchmark[idx - 1].vx,
                                        benchmark[idx - 1].vy,
                                        benchmark[idx - 1].vz};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    pub_odom_.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.poses.push_back(pose_stamped);
    pub_path_.publish(path);
}

void Benchmark::odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    //ROS_INFO("odom callback!");
    if (odom_msg->header.stamp.toSec() > benchmark.back().t)
      return;
  
    for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= odom_msg->header.stamp.toSec(); idx++)
        ;


    if (init++ < SKIP)
    {
        baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
                              odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z) *
                  Quaterniond(benchmark[idx - 1].qw,
                              benchmark[idx - 1].qx,
                              benchmark[idx - 1].qy,
                              benchmark[idx - 1].qz).inverse();
        baseTgt = Vector3d{odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z} -
                  baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        return;
    }

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(benchmark[idx - 1].t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Vector3d tmp_V = baseRgt * Vector3d{benchmark[idx - 1].vx,
                                        benchmark[idx - 1].vy,
                                        benchmark[idx - 1].vz};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    pub_odom_.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.poses.push_back(pose_stamped);
    pub_path_.publish(path);
}

} // namespace benchmark 
} // namespace vins_mono
} // namespace xslam_ros