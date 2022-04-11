#ifndef XSLAM_ROS_VINS_MONO_ROS_VISUAILIZATION__H
#define XSLAM_ROS_VINS_MONO_ROS_VISUAILIZATION__H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace xslam_ros {
namespace vins_mono {

class CameraPoseVisualization 
{
public:
	std::string marker_ns;

	CameraPoseVisualization(float red, float green, float blue, float alpha);
	
	void SetImageBoundaryColor(float red, float green, float blue, float alpha = 1.0);
	void SetOpticalCenterConnectorColor(float red, float green, float blue, float alpha = 1.0);
	void SetScale(double scale);
	void SetLineWidth(double width);

	void AddPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
	void Reset();

	void Publish(ros::Publisher& pub, const std_msgs::Header& header);
	void AddEdge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
	void AddLoopEdge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
	void PublishImage( ros::Publisher &pub, const std_msgs::Header &header);

private:
	std::vector<visualization_msgs::Marker> markers_;
	std_msgs::ColorRGBA image_boundary_color_;
	std_msgs::ColorRGBA optical_center_connector_color_;
	double scale_;
	double line_width_;
	visualization_msgs::Marker image_;
	int LOOP_EDGE_NUM;
	int tmp_loop_edge_num_;

	static const Eigen::Vector3d imlt_;
	static const Eigen::Vector3d imlb_;
	static const Eigen::Vector3d imrt_;
	static const Eigen::Vector3d imrb_;
	static const Eigen::Vector3d oc_  ;
	static const Eigen::Vector3d lt0_ ;
	static const Eigen::Vector3d lt1_ ;
	static const Eigen::Vector3d lt2_ ;
};

}  // namespace vins_mono
}  // namespace xslam_ros

#endif  // XSLAM_ROS_VINS_MONO_ROS_VISUAILIZATION__H
