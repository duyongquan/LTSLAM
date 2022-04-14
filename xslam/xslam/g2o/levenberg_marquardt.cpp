#include "xslam/g2o/levenberg_marquardt.h"

#include "glog/logging.h"

namespace xslam {
namespace g2o {

void LMCurveFitting::RunDemo(LevenbergMarquardt& lm)
{
    const double aa = 0.1, bb = 0.5, cc = 2; // 实际方程的参数
    lm.SetParameters(1e-10, 1e-10, 100, true);

    //  制造数据 
    const size_t N = 100; //数据个数
    cv::RNG rng(cv::getTickCount());

    for( size_t i = 0; i < N; i ++)
    {
        // 生产带有高斯噪声的数据
        double x = rng.uniform(0.0, 1.0) ;
        double y = exp(aa*x*x + bb*x + cc) + rng.gaussian(0.05);
        
        // 添加到观测中
        lm.AddObservation(x, y);
    }

    // 用LM法求解
    lm.Solve();
}

void LevenbergMarquardt::AddObservation(const double& x, const double& y)
{
    obs_.push_back(Eigen::Vector2d(x, y));
}

void LevenbergMarquardt::SetParameters(
    const double& epsilon_1, const double& epsilon_2, int max_iter, bool verbose)
{
    epsilon_1_ = epsilon_1;
    epsilon_2_ = epsilon_2;
    max_iter_ = max_iter;
    verbose_ = verbose;
}

void LevenbergMarquardt::Solve()
{
    int k = 0;
    double nu = 2.0;
    ComputeJcobianFx();
    ComputeHg();
    bool found = ( g_.lpNorm<Eigen::Infinity>() < epsilon_1_ );
    
    std::vector<double> A;
    A.push_back(H_(0, 0));
    A.push_back(H_(1, 1));
    A.push_back(H_(2, 2));

    auto max_p = std::max_element(A.begin(), A.end());
    double mu = *max_p;
    double sumt =0;

    while (!found && k < max_iter_)
    {        
        Runtimer t;
        t.Start();

        k = k +1;
        Eigen::Matrix3d G = H_ + mu * Eigen::Matrix3d::Identity();
        Eigen::Vector3d h = G.ldlt().solve(g_); // h is step
        
        if( h.norm() <= epsilon_2_ * (sqrt(a_* a_ +  b_ * b_ + c_* c_ ) + epsilon_2_ )) {
            found = true;
        }
        else
        {
            double na = a_ + h(0);
            double nb = b_ + h(1);
            double nc = c_ + h(2);
            
            double rho =( F(a_, b_, c_) - F(na, nb, nc) )  / L0_L(h);

            if( rho > 0)
            {
                a_ = na;
                b_ = nb;
                c_ = nc;
                ComputeJcobianFx();
                ComputeHg();
                                  
                found = ( g_.lpNorm<Eigen::Infinity>() < epsilon_1_ );
                mu = mu * std::max<double>(0.33, 1 - std::pow(2*rho -1, 3));
                nu = 2.0;
            }
            else
            {
                mu = mu * nu; 
                nu = 2*nu;
            }
        }
        
        t.Stop();
        if( verbose_ )
        {
            std::cout << "Iter: " << std::left <<std::setw(3) << k << " Result: "<< std::left <<std::setw(10)  << a_ << " " << std::left <<std::setw(10)  << b_ << " " << std::left <<std::setw(10) << c_ << 
            " step: " << std::left <<std::setw(14) << h.norm() << " cost: "<< std::left <<std::setw(14)  << GetCost() << " time: " << std::left <<std::setw(14) << t.Duration()  <<
            " total_time: "<< std::left <<std::setw(14) << (sumt += t.Duration()) << std::endl;
        }   
    } 
    
    if( found  == true) {
        LOG(INFO) << "Converged";
    } else {
      LOG(INFO) << "Diverged";
    }

    const double aa = 0.1, bb = 0.5, cc = 2; // 实际方程的参数
    std::cout << std::endl;
    LOG(INFO) << "实际方程的参数:";
    LOG(INFO) << "a = " << aa;
    LOG(INFO) << "b = " << bb;
    LOG(INFO) << "c = " << cc;

    std::cout << std::endl;
    LOG(INFO) << "LM算法求解方程的参数:";
    LOG(INFO) << "a = " << a_;
    LOG(INFO) << "b = " << b_;
    LOG(INFO) << "c = " << c_;

}

double LevenbergMarquardt::GetCost()
{
    Eigen::MatrixXd cost= objective_function_.transpose() * objective_function_;
    return cost(0, 0);
}

void LevenbergMarquardt::ComputeJcobianFx()
{
    J_.resize(obs_.size(), 3);
    objective_function_.resize(obs_.size(), 1);
    
    for ( size_t i = 0; i < obs_.size(); i ++)
    {
        const Eigen::Vector2d& ob = obs_.at(i);
        const double& x = ob(0);
        const double& y = ob(1);
        double j1 = -x * x * exp(a_ * x * x + b_* x + c_);
        double j2 = -x * exp(a_ * x * x + b_*x + c_);
        double j3 = -exp(a_ * x*x + b_*x + c_);
        J_(i, 0 ) = j1;
        J_(i, 1) = j2;
        J_(i, 2) = j3;
        objective_function_(i, 0) = y - exp( a_ *x*x + b_*x +c_);
    }
}

void LevenbergMarquardt::ComputeHg()
{
    H_ = J_.transpose() * J_;
    g_ = -J_.transpose() * objective_function_;
}

double LevenbergMarquardt::F(double a, double b, double c)
{
    Eigen::MatrixXd fx;
    fx.resize(obs_.size(), 1);
    
    for ( size_t i = 0; i < obs_.size(); i ++)
    {
        const Eigen::Vector2d& ob = obs_.at(i);
        const double& x = ob(0);
        const double& y = ob(1);
        fx(i, 0) = y - exp( a *x*x + b*x +c);
    }
    Eigen::MatrixXd F = 0.5 * fx.transpose() * fx;
    return F(0,0);
}

double LevenbergMarquardt::L0_L( Eigen::Vector3d& h)
{
    Eigen::MatrixXd L = -h.transpose() * J_.transpose() * objective_function_ - 0.5 * h.transpose() * J_.transpose() * J_ * h;
    return L(0,0);
}


} // namespace g2o
} // namespace xslam