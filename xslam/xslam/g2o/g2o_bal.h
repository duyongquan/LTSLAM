#ifndef XSLAM_G2O_G2O_BAL_H
#define XSLAM_G2O_G2O_BAL_H

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
 
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "sophus/se3.hpp"

namespace xslam {
namespace g2o {

// 数据集可以在这里下载: http://grail.cs.washington.edu/projects/bal/
// https://blog.csdn.net/JohnnyYeh/article/details/82315543

//定义求解器 pose：九维 路标点：三维
using  BalBlockSolver = ::g2o::BlockSolver<::g2o::BlockSolverTraits<9, 3>>;
using Vector9d = Eigen::Matrix< double, 9, 1 > ;

// BAL相机顶点
class VertexCameraBAL : public ::g2o::BaseVertex<9, Vector9d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {}
    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update);
    virtual bool read (std::istream& in) { return false;}
    virtual bool write (std::ostream& out) const {return false;}
};


// BAL 空间点顶点
class VertexPointBAL : public ::g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() {}
    virtual void setToOriginImpl() {}

    virtual void oplusImpl (const double* update);

    virtual bool read (std::istream& in ) { return false;}
    virtual bool write (std::ostream& out) const {return false;}
};

// BAL 边
class EdgeObservationBAL : public ::g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    virtual bool read ( std::istream& in ) {return false;}
    virtual bool write ( std::ostream& out ) const {return false;}

    // 使用G20数值导,不像书上调用ceres的自动求导功能.
    virtual void computeError() override;

    /***
     * 自己写雅克比矩阵
     */
    virtual void linearizeOplus() override;
};

//加载BAL的文本文件
class LoadBALProblem
{
public:
    LoadBALProblem(std::string filename ) : filename_(filename) {}
    ~LoadBALProblem()
    {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] observations_cameras;
        delete[] observations_points;
    }

    //返回相机数量,空间点数量，观测点数量
    int num_cameras()        const{ return num_cameras_;     }
    int num_points()         const{ return num_points_;      }
    int num_observations()   const{ return num_observations_;}

    double num_observations_cameras(int index)   { return observations_cameras[index];}
    double num_observations_points(int index)   { return observations_points[index];}
    double num_observations_uv(int index)   { return observations_[index];}

    int point_index(int index)   { return point_index_[index];}
    int camera_index(int index)   { return camera_index_[index];}

    //读取数据
    void ReadData();

    //将pose和point生成点云.
    void WriteToPLYFile(const std::string& filename);

    //当优化完成,重新将数据写入数组,覆盖原来未被优化的数据
    double* ReWritePoses(){return observations_cameras;}
    double* ReWritePoints(){return observations_points;}

    /***
     * 坐标系操作,相机坐标转换到世界坐标
     * 实现将pose转换到世界坐标系
     * @param angleAxis 旋转向量
     * @param P_c pose的t
     * @param P_w 世界坐标系pose的t
     */
    void Camera2World(const Eigen::Vector3d angleAxis, const Eigen::Vector3d P_c, Eigen::Vector3d& P_w);

private:
    int num_cameras_;
    int num_points_;
    int num_observations_;
    int observations_cameras_;
    int observations_points_;
    std::string filename_;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* observations_cameras;
    double* observations_points;
};

class LargeBA
{
public:
    void RunDemo(const std::string& problem_filename);

private:

    /***
     * 解决BAL问题
     * @param filename 将.txt文件传进来,最后将优化的后点云输出
     */
    void SolveBALProblem(const std::string filename);
};

/***
 * 实现将空间点坐标转换成相机坐标系所看到的点,即 Pc = Tcw * Pw
 * @param camera 相机的9的数据，实际上只用到前面6个数据
 * @param P_w 世界坐标系下的空间点
 * @param P_c 相机坐标系下的空间点
 */
void World2Camera(const Vector9d camera, const Eigen::Vector3d P_w, Eigen::Vector3d& P_c);

/***
 * 去畸变，世界坐标下的空间点转换成相机坐标系下的空间,最后变成像素坐标的像素点,基本跟书上的一致
 * 思想如下：
 * P  =  R * X + t       (conversion from world to camera coordinates)
 * p  = -P / P.z         (perspective division) 这里有个负号需要注意
 * p' =  f * r(p) * p    (conversion to pixel coordinates)
 * r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
 * @param camera 相机的9的数据，实际上只用到前面6个数据
 * @param point 世界坐标系下的空间点
 * @param u 像素坐标的像素点
 */
inline void CamProjectionWithDistortion(const Vector9d camera, const Eigen::Vector3d point, Eigen::Vector2d& u);

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_G2O_BAL_H
