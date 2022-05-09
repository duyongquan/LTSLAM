#ifndef XSLAM_G2O_DESCENT_GRADIENT_METHOD_H
#define XSLAM_G2O_DESCENT_GRADIENT_METHOD_H

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <chrono>

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


class DescentGradient
{
public:
    void RunDemo();

private:
    void setParameters(int max_iter, double min_gradient, double alpha, bool is_out);
    void addObservation(const double& x, const double& y);
    void calcGradient();
    double getCost();
    void solve();

    /* 要求解的四个参数 */
    double a_;
    double b_;
    double c_;
    double d_;
    
    /* parameters */
    int max_iter_;
    double min_gradient_;
    double alpha_;
    
    /* 观测 */
    std::vector<Eigen::Vector2d> obs_;
    
    /* Gradient */
    Eigen::Vector4d Gradient_;
        
    /* 是否输出中间结果 */
    bool is_out_;
};


} // namespace g2o
} // namespace xslam


#endif // XSLAM_G2O_DESCENT_GRADIENT_METHOD_H