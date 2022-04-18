#ifndef XSLAM_G2O_LEVENBERG_MARQUARDT_H
#define XSLAM_G2O_LEVENBERG_MARQUARDT_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Cholesky>
#include <chrono>


// You can reference web cite
// http://users.ics.forth.gr/~lourakis/sparseLM/index.html

namespace xslam {
namespace g2o {

// 计时类 
class Runtimer
{
public:
    inline void Start()
    {
        start_  = std::chrono::steady_clock::now();
    }
    
    inline void Stop()
    {
        end_ = std::chrono::steady_clock::now();
    }
    
    inline double Duration()
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(start_ - end_).count() * 1000.0;
    }
    
private:
    std::chrono::steady_clock::time_point start_; // start time ponit
    std::chrono::steady_clock::time_point end_; //stop time point
};


// y = exp(ax^2 + bx + c)
class LevenbergMarquardt
{
public:
    LevenbergMarquardt(double a, double b, double c) 
        : a_{a}, b_{b}, c_{a} {}

    ~LevenbergMarquardt() {}
   
    void AddObservation(const double& x, const double& y);
    void SetParameters(const double& epsilon_1, const double& epsilon_2, int max_iter, bool verbose);

    void Solve();
    double GetCost();
    void ComputeJcobianFx();
    void ComputeHg();

    double F(double a, double b, double c);
    double L0_L( Eigen::Vector3d& h);

private:

    Eigen::MatrixXd objective_function_; 
    Eigen::MatrixXd J_; // 雅克比矩阵
    Eigen::Matrix3d H_; // H矩阵
    Eigen::Vector3d g_;
    
    std::vector< Eigen::Vector2d> obs_; // 观测

    // 要求的三个参数 
    double a_;
    double b_;
    double c_;

    // parameters
    double epsilon_1_;
    double epsilon_2_;
    int max_iter_;
    bool verbose_;
};

class LMCurveFitting
{
public:
     void RunDemo(LevenbergMarquardt& lm);
};

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_LEVENBERG_MARQUARDT_H
