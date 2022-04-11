#include "xslam_ros/vins_mono/ros_visualization.h"

namespace xslam_ros {
namespace vins_mono {

const Eigen::Vector3d CameraPoseVisualization::imlt_ = Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrt_ = Eigen::Vector3d( 1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imlb_ = Eigen::Vector3d(-1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrb_ = Eigen::Vector3d( 1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt0_ = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt1_ = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt2_ = Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::oc_  = Eigen::Vector3d(0.0, 0.0, 0.0);

void Eigen2Point(const Eigen::Vector3d& v, geometry_msgs::Point& p) 
{
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

CameraPoseVisualization::CameraPoseVisualization(float red, float green, float blue, float alpha)
    : marker_ns("CameraPoseVisualization"), scale_(0.2), line_width_(0.01) 
{
    image_boundary_color_.r = red;
    image_boundary_color_.g = green;
    image_boundary_color_.b = blue;
    image_boundary_color_.a = alpha;
    optical_center_connector_color_.r = red;
    optical_center_connector_color_.g = green;
    optical_center_connector_color_.b = blue;
    optical_center_connector_color_.a = alpha;
    LOOP_EDGE_NUM = 20;
    tmp_loop_edge_num_ = 1;
}

void CameraPoseVisualization::SetImageBoundaryColor(float red, float green, float blue, float alpha) 
{
    image_boundary_color_.r = red;
    image_boundary_color_.g = green;
    image_boundary_color_.b = blue;
    image_boundary_color_.a = alpha;
}

void CameraPoseVisualization::SetOpticalCenterConnectorColor(float red, float green, float blue, float alpha) 
{
    optical_center_connector_color_.r = red;
    optical_center_connector_color_.g = green;
    optical_center_connector_color_.b = blue;
    optical_center_connector_color_.a = alpha;
}

void CameraPoseVisualization::SetScale(double scale) 
{
    scale_ = scale;
}

void CameraPoseVisualization::SetLineWidth(double width) 
{
    line_width_ = width;
}

void CameraPoseVisualization::AddEdge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1)
{
    visualization_msgs::Marker marker;

    marker.ns = marker_ns;
    marker.id = markers_.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;

    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    markers_.push_back(marker);
}

void CameraPoseVisualization::AddLoopEdge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1)
{
    visualization_msgs::Marker marker;

    marker.ns = marker_ns;
    marker.id = markers_.size() + 1;
  
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.02;
    marker.color.r = 1.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);
    markers_.push_back(marker);
}

void CameraPoseVisualization::AddPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) 
{
    visualization_msgs::Marker marker;

    marker.ns = marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = line_width_;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;


    geometry_msgs::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;

    Eigen2Point(q * (scale_ *imlt_) + p, pt_lt);
    Eigen2Point(q * (scale_ *imlb_) + p, pt_lb);
    Eigen2Point(q * (scale_ *imrt_) + p, pt_rt);
    Eigen2Point(q * (scale_ *imrb_) + p, pt_rb);
    Eigen2Point(q * (scale_ *lt0_ ) + p, pt_lt0);
    Eigen2Point(q * (scale_ *lt1_ ) + p, pt_lt1);
    Eigen2Point(q * (scale_ *lt2_ ) + p, pt_lt2);
    Eigen2Point(q * (scale_ *oc_  ) + p, pt_oc);

    // image boundaries
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_lb);
    marker.colors.push_back(image_boundary_color_);
    marker.colors.push_back(image_boundary_color_);

    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_rb);
    marker.colors.push_back(image_boundary_color_);
    marker.colors.push_back(image_boundary_color_);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_rt);
    marker.colors.push_back(image_boundary_color_);
    marker.colors.push_back(image_boundary_color_);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_lt);
    marker.colors.push_back(image_boundary_color_);
    marker.colors.push_back(image_boundary_color_);

    // top-left indicator
    marker.points.push_back(pt_lt0);
    marker.points.push_back(pt_lt1);
    marker.colors.push_back(image_boundary_color_);
    marker.colors.push_back(image_boundary_color_);

    marker.points.push_back(pt_lt1);
    marker.points.push_back(pt_lt2);
    marker.colors.push_back(image_boundary_color_);
    marker.colors.push_back(image_boundary_color_);

    // optical center connector
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(optical_center_connector_color_);
    marker.colors.push_back(optical_center_connector_color_);


    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(optical_center_connector_color_);
    marker.colors.push_back(optical_center_connector_color_);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(optical_center_connector_color_);
    marker.colors.push_back(optical_center_connector_color_);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(optical_center_connector_color_);
    marker.colors.push_back(optical_center_connector_color_);

    markers_.push_back(marker);
}

void CameraPoseVisualization::Reset() 
{
	  markers_.clear();
}

void CameraPoseVisualization::Publish(ros::Publisher &pub, const std_msgs::Header &header) 
{
    visualization_msgs::MarkerArray markerArray_msg;

    for(auto& marker : markers_) {
      marker.header = header;
      markerArray_msg.markers.push_back(marker);
    }
    
    pub.publish(markerArray_msg);
}

void CameraPoseVisualization::PublishImage(ros::Publisher &pub, const std_msgs::Header &header) 
{
    image_.header = header;
    pub.publish(image_);
}

}  // namespace vins_mono
}  // namespace xslam_ros