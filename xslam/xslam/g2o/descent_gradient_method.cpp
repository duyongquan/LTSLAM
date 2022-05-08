#include "xslam/g2o/descent_gradient_method.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <glog/logging.h>

namespace xslam {
namespace g2o {

void DescentGradient::RunDemo()
{
    const double aa = 4, bb = 3, cc = 2, dd = 1;// 实际方程的参数
    a_ = 0.0;
    b_ = 0.0;
    c_ = 0.0;
    d_ = 0.0; // 初值
    
    /* 构造问题 */
    setParameters(80000, 8e-5, 8e-7, true); //步长的选择非常重要，如果太大就会导致发散, 太小收敛速度极慢。
    
    /* 制造数据 */
    const size_t N = 1000; //数据个数
    cv::RNG rng(cv::getTickCount());
    for( size_t i = 0; i < N; i ++)
    {
        /* 生产带有高斯噪声的数据　*/
        double x = rng.uniform(0.0, 4.0) ;
        double y = aa*x*x*x + bb*x*x + cc*x + dd + rng.gaussian(1);
        
        /* 添加到观测中　*/
        addObservation(x, y);
    }
    /* 用梯度下降法求解 */
    solve();
}

void DescentGradient::setParameters(int max_iter, double min_gradient, double alpha, bool is_out)
{
    max_iter_ = max_iter;
    min_gradient_ = min_gradient;
    alpha_ = alpha;
    is_out_ = is_out;
}

void DescentGradient::addObservation(const double& x, const double& y)
{
    obs_.push_back( Eigen::Vector2d(x, y));
}

void DescentGradient::calcGradient() //计算梯度
{
    double g_a = 0.0, g_b = 0.0, g_c = 0.0, g_d = 0.0;
    
    /* 计算梯度 */
    for( size_t i = 0; i < obs_.size(); i ++)
    {
        Eigen::Vector2d& ob = obs_.at(i);
        const double& x= ob(0);
        const double& y = ob(1);
        
        g_a += - x*x*x * ( y - a_*x*x*x - b_*x*x - c_*x - d_ );
        g_b += - x*x * ( y - a_*x*x*x - b_*x*x - c_*x - d_ );
        g_c += - x*( y - a_*x*x*x - b_*x*x - c_*x - d_ );
        g_d += -  ( y - a_*x*x*x - b_*x*x - c_*x - d_ );
    }
    
    Gradient_(0) = g_a;
    Gradient_(1) = g_b;
    Gradient_(2) = g_c;
    Gradient_(3) = g_d;
}

double DescentGradient::getCost()
{
    double sum_cost = 0;
    for( size_t i = 0; i < obs_.size(); i ++)
    {
        Eigen::Vector2d& ob = obs_.at(i);
        const double& x= ob(0);
        const double& y = ob(1);
        double r = y - a_*x*x*x - b_*x*x - c_*x - d_;
        sum_cost += r*r;
    }
    return sum_cost;
}

void DescentGradient::solve()
{
    double sumt =0;
    bool is_conv = false;
    
    for( int i = 0; i < max_iter_; i ++)
    {
        Runtimer t;
        t.Start();
        calcGradient();
        double dg = sqrt(Gradient_.transpose() * Gradient_);
        if( dg*alpha_  < min_gradient_ ) 
        {
            is_conv = true;
            break;
        }

        /* update */
        a_ += -alpha_ * Gradient_(0);
        b_ += -alpha_ * Gradient_(1);
        c_ += -alpha_ * Gradient_(2);
        d_ += -alpha_ * Gradient_(3);
                
        t.Stop();
        if( is_out_ )
        {
            std::cout << "Iter: " << std::left <<std::setw(3) << i << " Result: "<< std::left <<std::setw(10)  << a_ << " " << std::left <<std::setw(10)  << b_ << " " << std::left <<std::setw(10) << c_ <<
            " " << std::left <<std::setw(10) << d_ <<
            " Gradient: " << std::left <<std::setw(14) << dg << " cost: "<< std::left <<std::setw(14)  << getCost() << " time: " << std::left <<std::setw(14) << t.Duration()  <<
            " total_time: "<< std::left <<std::setw(14) << (sumt += t.Duration()) << std::endl;
        }
    }
    if( is_conv  == true)
        std::cout << "\nConverged\n\n";
    else
        std::cout << "\nDiverged\n\n";

    const double aa = 4, bb = 3, cc = 2, dd = 1;// 实际方程的参数
    std::cout << std::endl;
 
    LOG(INFO) << "方程: y = ax^3 + bx^2 + cx + d ";
    LOG(INFO) << "实际方程的参数:";
    LOG(INFO) << "a = " << aa;
    LOG(INFO) << "b = " << bb;
    LOG(INFO) << "c = " << cc;
    LOG(INFO) << "d = " << dd;

    std::cout << std::endl;
    LOG(INFO) << "最速梯度下降算法求解方程的参数:";
    LOG(INFO) << "a = " << a_;
    LOG(INFO) << "b = " << b_;
    LOG(INFO) << "c = " << c_;
    LOG(INFO) << "c = " << d_;

}

} // namespace g2o
} // namespace xslam
