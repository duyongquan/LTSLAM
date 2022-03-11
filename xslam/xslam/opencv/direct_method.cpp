#include "xslam/opencv/direct_method.h"

#include <mutex>
#include <vector>


#if 0
namespace xslam {
namespace opencv {

Camera camera_config;

void DirectMetod::RunDemo(const std::string& filename)
{

    std::string left_file = filename + "left.png";
    std::string disparity_file = filename + "disparity.png";

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 2000;
    int boarder = 20;
    VecVector2d pixels_ref;
    std::vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) 
    {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = camera_config.fx * camera_config.baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref;

    for (int i = 1; i < 6; i++) 
    {  // 1~10

        std::string img_file = filename + "00000" + std::to_string(i) + ".png";
        cv::Mat img = cv::imread(img_file, 0);
        // try single layer by uncomment this line
        // DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    }
}

void DirectMetod::JacobianAccumulator::accumulate_jacobian(const cv::Range &range) 
{

    // parameters
    const int half_patch_size = 1;
    int cnt_good = 0;
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d bias = Vector6d::Zero();
    double cost_tmp = 0;

    for (size_t i = range.start; i < range.end; i++) 
    {
        // compute the projection in the second image
        Eigen::Vector3d point_ref = depth_ref[i] * Eigen::Vector3d((px_ref[i][0] - camera_config.cx) / camera_config.fx, 
            (px_ref[i][1] - camera_config.cy) / camera_config.fy, 1);

        Eigen::Vector3d point_cur = T21 * point_ref;
        if (point_cur[2] < 0)   // depth invalid
            continue;

        float u = camera_config.fx * point_cur[0] / point_cur[2] + camera_config.cx;
        float v = camera_config.fy * point_cur[1] / point_cur[2] + camera_config.cy;
        if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size ||
            v > img2.rows - half_patch_size)
            continue;

        projection[i] = Eigen::Vector2d(u, v);
        double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
            Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
        cnt_good++;

        // and compute error and jacobian
        for (int x = -half_patch_size; x <= half_patch_size; x++) 
        {
            for (int y = -half_patch_size; y <= half_patch_size; y++) 
            {

                double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) -
                               GetPixelValue(img2, u + x, v + y);
                Matrix26d J_pixel_xi;
                Eigen::Vector2d J_img_pixel;

                J_pixel_xi(0, 0) = camera_config.fx * Z_inv;
                J_pixel_xi(0, 1) = 0;
                J_pixel_xi(0, 2) = -camera_config.fx * X * Z2_inv;
                J_pixel_xi(0, 3) = -camera_config.fx * X * Y * Z2_inv;
                J_pixel_xi(0, 4) =  camera_config.fx + camera_config.fx * X * X * Z2_inv;
                J_pixel_xi(0, 5) = -camera_config.fx * Y * Z_inv;

                J_pixel_xi(1, 0) = 0;
                J_pixel_xi(1, 1) =  camera_config.fy * Z_inv;
                J_pixel_xi(1, 2) = -camera_config.fy * Y * Z2_inv;
                J_pixel_xi(1, 3) = -camera_config.fy - camera_config.fy * Y * Y * Z2_inv;
                J_pixel_xi(1, 4) =  camera_config.fy * X * Y * Z2_inv;
                J_pixel_xi(1, 5) =  camera_config.fy * X * Z_inv;

                J_img_pixel = Eigen::Vector2d(
                    0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
                    0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y))
                );

                // total jacobian
                Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

                hessian += J * J.transpose();
                bias += -error * J;
                cost_tmp += error * error;
            }
        }
    }

    if (cnt_good) 
    {
        // set hessian, bias and cost
        std::unique_lock<std::mutex> lock(hessian_mutex);
        H += hessian;
        b += bias;
        cost += cost_tmp / cnt_good;
    }
}

// bilinear interpolation
float GetPixelValue(const cv::Mat &img, float x, float y)
{
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);

    return float(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step] +
        xx * yy * data[img.step + 1]
    );
}

void DirectMetod::DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const std::vector<double> depth_ref,
    Sophus::SE3d &T21) 
{
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    auto t1 = std::chrono::steady_clock::now();
    JacobianAccumulator jaco_accu(img1, img2, px_ref, depth_ref, T21);

    for (int iter = 0; iter < iterations; iter++) 
    {
        jaco_accu.reset();
        cv::parallel_for_(cv::Range(0, px_ref.size()),
                          std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1));
        Matrix6d H = jaco_accu.hessian();
        Vector6d b = jaco_accu.bias();

        // solve update and put it into estimation
        Vector6d update = H.ldlt().solve(b);;
        T21 = Sophus::SE3d::exp(update) * T21;
        cost = jaco_accu.cost_func();

        if (std::isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            std::cout << "update is nan" << std::endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            std::cout << "cost increased: " << cost << ", " << lastCost << std::endl;
            break;
        }
        if (update.norm() < 1e-3) {
            // converge
            break;
        }

        lastCost = cost;
        std::cout << "iteration: " << iter << ", cost: " << cost << std::endl;
    }

    std::cout << "T21 = \n" << T21.matrix() << std::endl;
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "direct method for single layer: " << time_used.count() << std::endl;

    // plot the projected pixels here
    cv::Mat img2_show;
    cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
    VecVector2d projection = jaco_accu.projected_points();
    for (size_t i = 0; i < px_ref.size(); ++i) 
    {
        auto p_ref = px_ref[i];
        auto p_cur = projection[i];

        if (p_cur[0] > 0 && p_cur[1] > 0) 
        {
            cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_show, cv::Point2f(p_ref[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]),
                     cv::Scalar(0, 250, 0));
        }
    }
    cv::imshow("current", img2_show);
    cv::waitKey();
}

void DirectMetod::DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const std::vector<double> depth_ref,
    Sophus::SE3d &T21) 
{

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    std::vector<cv::Mat> pyr1, pyr2; // image pyramids
    for (int i = 0; i < pyramids; i++) 
    {
        if (i == 0) {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        } else {
            cv::Mat img1_pyr, img2_pyr;
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }

    // backup the old values
    double fxG = camera_config.fx, fyG = camera_config.fy;
    double cxG = camera_config.cx, cyG = camera_config.cy;  
    
    for (int level = pyramids - 1; level >= 0; level--) 
    {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) 
        {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // scale fx, fy, cx, cy in different pyramid levels
        camera_config.fx = fxG * scales[level];
        camera_config.fy = fyG * scales[level];
        camera_config.cx = cxG * scales[level];
        camera_config.cy = cyG * scales[level];
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }
}

} // namespace opencv
} // namespace xslam

#endif 