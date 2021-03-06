#include "xslam/opencv/geometry_transform/image_formation.h"

#include <fstream>

namespace xslam {
namespace opencv {
namespace geometry_transform {

void ImageFormation::RunDemo(const std::string& filename)
{
    // The given camera configuration: focal length, principal point, image resolution, position, and orientation
    double f = 1000, cx = 320, cy = 240, noise_std = 1;
    cv::Size img_res(640, 480);
    std::vector<cv::Point3d> cam_pos = { cv::Point3d(0, 0, 0), cv::Point3d(-2, -2, 0), cv::Point3d(2, 2, 0), cv::Point3d(-2, 2, 0), cv::Point3d(2, -2, 0) };
    std::vector<cv::Point3d> cam_ori = { cv::Point3d(0, 0, 0), cv::Point3d(-CV_PI / 12, CV_PI / 12, 0), cv::Point3d(CV_PI / 12, -CV_PI / 12, 0), cv::Point3d(CV_PI / 12, CV_PI / 12, 0), cv::Point3d(-CV_PI / 12, -CV_PI / 12, 0) };

    // Load a point cloud in the homogeneous coordinate
    FILE* fin = fopen(filename.c_str(), "rt");
    if (fin == NULL) return;
    cv::Mat X;
    while (!feof(fin))
    {
        double x, y, z;
        if (fscanf(fin, "%lf %lf %lf", &x, &y, &z) == 3) X.push_back(cv::Vec4d(x, y, z, 1));
    }
    fclose(fin);
    X = X.reshape(1).t(); // Convert to a 4 x N matrix

    // Generate images for each camera pose
    cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    for (size_t i = 0; i < cam_pos.size(); i++)
    {
        // Derive a projection matrix
        cv::Mat Rc = Rz(cam_ori[i].z) * Ry(cam_ori[i].y) * Rx(cam_ori[i].x);
        cv::Mat tc(cam_pos[i]);
        cv::Mat Rt;
        cv::hconcat(Rc.t(), -Rc.t() * tc, Rt);
        cv::Mat P = K * Rt;

        // Project the points (c.f. OpenCV provides 'cv::projectPoints()' with consideration of distortion.)
        cv::Mat x = P * X;
        x.row(0) = x.row(0) / x.row(2);
        x.row(1) = x.row(1) / x.row(2);
        x.row(2) = 1;

        // Add Gaussian noise
        cv::Mat noise(2, x.cols, x.type());
        cv::randn(noise, cv::Scalar(0), cv::Scalar(noise_std));
        x.rowRange(0, 2) = x.rowRange(0, 2) + noise;

        // Show and store the points
        cv::Mat image = cv::Mat::zeros(img_res, CV_8UC1);
        for (int c = 0; c < x.cols; c++)
        {
            cv::Point p(x.col(c).rowRange(0, 2));
            if (p.x >= 0 && p.x < img_res.width && p.y >= 0 && p.y < img_res.height)
                cv::circle(image, p, 2, 255, -1);
        }
        cv::imshow(cv::format("3DV_Tutorial: Image Formation %d", i), image);

        FILE* fout = fopen(cv::format("image_formation%d.xyz", i).c_str(), "wt");
        if (fout == NULL) return;
        for (int c = 0; c < x.cols; c++)
            fprintf(fout, "%f %f 1\n", x.at<double>(0, c), x.at<double>(1, c));
        fclose(fout);
    }

    cv::waitKey(0);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam