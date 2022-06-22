#include "xslam/opencv/geometry_transform/line_fitting_ransac.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>
#include <math.h>

namespace xslam {
namespace opencv {
namespace geometry_transform {

void LineFittingRANSAC::RunDemo(const std::string& iamge)
{
     cv::Vec3d truth(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), -240.0); // The line model: a*x + b*y + c = 0 (a^2 + b^2 = 1)
    int ransac_trial = 50, ransac_n_sample = 2;
    double ransac_thresh = 3.0; // 3 x 'data_inlier_noise'
    int data_num = 1000;
    double data_inlier_ratio = 0.5, data_inlier_noise = 1.0;

    // Generate data
    std::vector<cv::Point2d> data;
    cv::RNG rng;
    for (int i = 0; i < data_num; i++)
    {
        if (rng.uniform(0.0, 1.0) < data_inlier_ratio)
        {
            double x = rng.uniform(0.0, 480.0);
            double y = (truth[0] * x + truth[2]) / -truth[1];
            x += rng.gaussian(data_inlier_noise);
            y += rng.gaussian(data_inlier_noise);
            data.push_back(cv::Point2d(x, y)); // Inlier
        }
        else data.push_back(cv::Point2d(rng.uniform(0.0, 640.0), rng.uniform(0.0, 480.0))); // Outlier
    }

    // Estimate a line using RANSAC
    int best_score = -1;
    cv::Vec3d best_line;
    for (int i = 0; i < ransac_trial; i++)
    {
        // Step 1: Hypothesis generation
        std::vector<cv::Point2d> sample;
        for (int j = 1; j < ransac_n_sample; j++)
        {
            int index = rng.uniform(0, int(data.size()));
            sample.push_back(data[index]);
        }
        cv::Vec4d nnxy;
        cv::fitLine(sample, nnxy, cv::DIST_L2, 0, 0.01, 0.01);
        cv::Vec3d line = CONVERT_LINE(nnxy);

        // Step 2: Hypothesis evaluation
        int score = 0;
        for (size_t j = 0; j < data.size(); j++)
        {
            double error = fabs(line(0) * data[j].x + line(1) * data[j].y + line(2));
            if (error < ransac_thresh) score++;
        }

        if (score > best_score)
        {
            best_score = score;
            best_line = line;
        }
    }

    // Estimate a line using least squares method (for reference)
    cv::Vec4d nnxy;
    cv::fitLine(data, nnxy, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Vec3d lsm_line = CONVERT_LINE(nnxy);

    // Display estimates
    printf("* The Truth: %.3f, %.3f, %.3f\n", truth[0], truth[1], truth[2]);
    printf("* Estimate (RANSAC): %.3f, %.3f, %.3f (Score: %d)\n", best_line[0], best_line[1], best_line[2], best_score);
    printf("* Estimate (LSM): %.3f, %.3f, %.3f\n", lsm_line[0], lsm_line[1], lsm_line[2]);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam
