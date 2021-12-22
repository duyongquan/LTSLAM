#include "xslam_ros/slam2d/slam2d.h"
#include "xslam_ros/slam2d/lidar_cost_function.h"


namespace xslam_ros {
namespace slam2d {

Slam2D::Slam2D()
{
    state_.t = Eigen::Vector2d::Zero();
    state_.theta = 0;
    map2d_.header.frame_id = "odom";
    map2d_.info.width = 2000;
    map2d_.info.height = 2000;
    map2d_.info.resolution = 0.15;
    map2d_.info.origin.orientation.w = 1;
    map2d_.info.origin.orientation.x = 0;
    map2d_.info.origin.orientation.y = 0;
    map2d_.info.origin.orientation.z = 0;
    map2d_.info.origin.position.x = -0.5 * map2d_.info.width * map2d_.info.resolution;
    map2d_.info.origin.position.y = -0.5 * map2d_.info.height * map2d_.info.resolution;
    map2d_.info.origin.position.z = 0;
    map2d_.data.resize(map2d_.info.width * map2d_.info.height);
    cvmap2d_ = cv::Mat(map2d_.info.width, map2d_.info.height, CV_8SC1, -1);
    CVMapToMap();
}

void Slam2D::HandleLaserScanMessages(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    timestamp_ = msg->header.stamp.toSec();
    scan_.points.resize(msg->ranges.size());
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        float dist = msg->ranges[i].echoes[0]; //only first echo used for slam2d
        float theta = msg->angle_min + i * msg->angle_increment;
        scan_.points[i].x = dist * cos(theta);
        scan_.points[i].y = dist * sin(theta);
    }
    scan_.width = scan_.points.size();
    scan_.height = 1;
    scan_.is_dense = true;
}

void Slam2D::HandleLaserScanMessages(const sensor_msgs::LaserScanConstPtr &msg)
{
    timestamp_ = msg->header.stamp.toSec();
    scan_.points.resize(msg->ranges.size());
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        float dist = msg->ranges[i]; //only first echo used for slam2d
        float theta = msg->angle_min + i * msg->angle_increment;
        scan_.points[i].x = dist * cos(theta);
        scan_.points[i].y = dist * sin(theta);
    }
    scan_.width = scan_.points.size();
    scan_.height = 1;
    scan_.is_dense = true;
}


void Slam2D::ScanMatch()
{
    double pose[3] = {0};

    if (scan_.points.size() < 0 || scan_prev_.points.size() < 0) 
    {
        return;
    }

  
    ceres::Problem problem;
    //solve delta with ceres constraints
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(scan_.makeShared());
    int K = 2; // K nearest neighbor search
    std::vector<int> index(K);
    std::vector<float> distance(K);

    //1. project scan_prev to scan

    Eigen::Matrix2d R;
    R(0, 0) = cos(delta_.theta); R(0, 1) = -sin(delta_.theta);
    R(1, 0) = sin(delta_.theta); R(1, 1) = cos(delta_.theta);
    Eigen::Vector2d dt = delta_.t;
    //find nearest neighur
    for (int i = 0; i < scan_prev_.points.size(); i++)
    {
        PointType search_point = scan_prev_.points[i];
        //project search_point to current frame
        PointType search_point_predict = ToPoint(R * ToEigen(search_point) + dt);
        if (kdtree.nearestKSearch(search_point_predict, K, index, distance) == K)
        {
            //add constraints
            Eigen::Vector2d p =  ToEigen(search_point);
            Eigen::Vector2d p1 = ToEigen(scan_.points[index[0]]);
            Eigen::Vector2d p2 = ToEigen(scan_.points[index[1]]);
            ceres::CostFunction *cost_function = LidarEdgeError::Create(p, p1, p2);
            problem.AddResidualBlock(cost_function,
                                        new ceres::CauchyLoss(0.5),
                                        pose);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    LOG(INFO) << "result: [" <<  pose[0] << ", " << pose[1] << ", " << pose[2] << "]";
    delta_.theta = pose[0];
    delta_.t(0) = pose[1];
    delta_.t(1) = pose[2];
}

void Slam2D::MatchRandom()
{
    Eigen::Vector3d pose(state_.theta, state_.t(0), state_.t(1));
    double eps = 1e-5;
    //search best mattch
    int N = 200;

    for (int i = 0; i < N; i++)
    {
        //random direction
        Eigen::Vector3d d = Eigen::Vector3d::Random();
        d(0) /= 10.0;
        d.normalize();
        double min_len = 0;
        double max_len = 0.2;
        //search best len
        while((max_len - min_len) > eps)
        {
            int score1 = MatchScore(pose + d * min_len);
            int score2 = MatchScore(pose + d * max_len);
            if (score1 >= score2)
            {
                max_len = (min_len + max_len) / 2.0;
            } else 
            {
                min_len = (min_len + max_len) / 2.0;
            }
        }
        pose += d * min_len;
        Eigen::Vector3d dx = d * min_len;
        int score = MatchScore(pose);
        printf("score: %d, min_len: %lf\n", score, min_len);
        std::cout << "dx: " << dx.transpose() << endl;
    }
    //update to state
    state_.theta = pose(0);
    state_.t = pose.bottomRows(2);
}

int Slam2D::MatchScore(Eigen::Vector3d pose)
{
    int score = 0;
    Eigen::Matrix2d R;
    Eigen::Vector2d t(pose(1), pose(2));
    double theta = pose(0);
    R(0, 0) = cos(theta); R(0, 1) = -sin(theta);
    R(1, 0) = sin(theta); R(1, 1) =  cos(theta);
    //printf("cols: %d, rows: %d\n", cvmap2d.cols, cvmap2d.rows);
    for (int i = 0; i < scan_.points.size(); i++)
    {
        Eigen::Vector2d p = ToEigen(scan_.points[i]);
        Eigen::Vector2d pp = WorldToMap(R * p + t);

        //cout << "pp: " << pp.transpose() << endl;
        if ((pp(0) <= 1) || (pp(0) >= cvmap2d_.cols) || (pp(1) <= 1) || (pp(1) >= cvmap2d_.rows))
        {
            continue;
        } 
        else 
        {
            //get value from map
            int x = round(pp(0));
            int y = round(pp(1));
            if (cvmap2d_.at<int8_t>(y * cvmap2d_.cols + x) == 100)
            {
                score++;
            }
        }
    }
    //generate local map and compute local optimal?
    return score;
}

void Slam2D::Update()
{
    static int cnt = 0;
    if (scan_.points.size() && scan_.points.size())
    {
        ScanMatch();
        UpdateTransform();
        MatchRandom();
        UpdateMap();
    }

    if (scan_.points.size())
    {
        scan_prev_ = scan_;
    }
    cnt++;
}

void Slam2D::UpdateTransform()
{
    Eigen::Matrix2d dR;
    dR(0, 0) = cos(delta_.theta); dR(0, 1) = -sin(delta_.theta);
    dR(1, 0) = sin(delta_.theta); dR(1, 1) =  cos(delta_.theta);


    Eigen::Vector2d dt_inv = -dR.transpose() * delta_.t;
    Eigen::Matrix2d dR_inv = dR.transpose();
    

    Eigen::Matrix2d R;
    R(0, 0) = cos(state_.theta); R(0, 1) = -sin(state_.theta);
    R(1, 0) = sin(state_.theta); R(1, 1) =  cos(state_.theta);
    state_.theta += (-delta_.theta);
    state_.t     += R * dt_inv;

}

void Slam2D::Bresenham(cv::Point2i p1, cv::Point2i p2)
{
    //drawing a line from p1 to p2
    int dx = std::abs(p2.x - p1.x);
    int sx = (p2.x > p1.x) ? 1 : -1;
    int dy = std::abs(p2.y - p1.y);
    int sy = (p2.y > p1.y) ? 1 : -1;
    int err = (dx > dy ? dx : dy) / 2;
    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;

    while (x1 != x2 && y1 != y2)
    {
        if (cvmap2d_.at<int8_t>(y1 * cvmap2d_.cols + x1) == 100)
        {
            break;
        }
        else if (cvmap2d_.at<int8_t>(y1 * cvmap2d_.cols + x1) == -1)
        {
            cvmap2d_.at<int8_t>(y1 * cvmap2d_.cols + x1) = 0;
        }
        int e2 = err;
        if (e2 > -dx) 
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y1 += sy;
        }
    }
}

void Slam2D::UpdateMap()
{
    //update map with scan and state
    cv::Point2f tt;
    tt.x = state_.t(0);
    tt.y = state_.t(1);
    cv::Point2i origin = WorldToMap(tt);
    if (origin.x < 0 || origin.x >= cvmap2d_.cols || 
        origin.y < 0 || origin.y >= cvmap2d_.rows) {
        return;
    }

    Eigen::Matrix2d R;
    R(0, 0) = cos(state_.theta); R(0, 1) = -sin(state_.theta);
    R(1, 0) = sin(state_.theta); R(1, 1) =  cos(state_.theta);

    for (int i = 0; i < scan_.points.size(); i++)
    {
        PointType p = scan_.points[i];
        float dist = sqrtf(p.x * p.x + p.y * p.y);
        if (dist > 20) continue;
        Eigen::Vector2d pp = R * ToEigen(p) + state_.t;
        cv::Point2f ppp(pp(0), pp(1));

        cv::Point2i pt = WorldToMap(ppp);

        if (pt.x < 0 || pt.x >= cvmap2d_.cols || pt.y < 0 || pt.y >= cvmap2d_.rows)
            return;

        Bresenham(origin, pt);
        cvmap2d_.at<int8_t>(pt.y * cvmap2d_.cols + pt.x) = 100; //100->occupancied
    }
    CVMapToMap();
}

void Slam2D::CVMapToMap()
{
    for (int i = 0; i < cvmap2d_.rows; i++)
    {
        for(int j = 0; j < cvmap2d_.cols; j++)
        {
            map2d_.data[i * map2d_.info.width + j] = cvmap2d_.at<int8_t>(i, j);
        }
    }
    if (cvmap_vis_enable_)
    {
        cv::imshow("cvmap2d", cvmap2d_);
        cv::waitKey(2);
    }
}

Eigen::Vector2d Slam2D::WorldToMap(Eigen::Vector2d p)
{
    Eigen::Vector2d m;
    m = p / map2d_.info.resolution;
    m(0) += map2d_.info.width * 0.5;
    m(1) += map2d_.info.height * 0.5;
    return m;
}

cv::Point2i Slam2D::WorldToMap(cv::Point2f p)
{
    return 
    {
        roundf(p.x / map2d_.info.resolution + map2d_.info.width  * 0.5),
        roundf(p.y / map2d_.info.resolution + map2d_.info.height * 0.5)
    };
}

double Slam2D::timestamp() const
{
    return timestamp_;
}

State2D Slam2D::state() const
{
    return state_;
}

void  Slam2D::set_map2d_timestamp(ros::Time timestamp)
{
    map2d_.header.stamp = timestamp;
}

nav_msgs::OccupancyGrid& Slam2D::GetOccupancyGridMap()
{
    return map2d_;
}

} // namespace slam2d
} // namespace xslam_ros
