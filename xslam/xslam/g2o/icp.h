
#ifndef XSLAM_G2O_ICP_H
#define XSLAM_G2O_ICP_H

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Eigen>
namespace xslam {
namespace g2o {


#define  points_nums_  30
class ICP
{
public:
    void RunDemo();

private:

    struct Result
    {
        Eigen::Matrix4d trans;
        std::vector<float> distances;
        int iter;
    };

    struct Neighbor 
    {
        std::vector<float> distances;
        std::vector<int> indices;
    };


    Eigen::Matrix4d BestFitTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    Result RunICP(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations=20, int tolerance = 0.001);

    // throughout method
    Neighbor FindNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
    float ComputeDistance(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb);

    float GenerateRandom(void);

    Eigen::Matrix3d RotationMatrix(Eigen::Vector3d axis, float theta);

    const int iterations_nums_ = 100;
    const double noise_sigma_ = 0.01; // standard deviation error to be added
    const double translation_ = 0.1;  // max translation of the test set
    const double rotation = 0.1;      // max rotation (radians) of the test set
};

} // namespace g2o
} // namespace xslam


#endif // XSLAM_G2O_ICP_H
