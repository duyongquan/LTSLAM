#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_EPNP_EIGEN_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_EPNP_EIGEN_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace geometry_transform {

struct DistPattern 
{
  int a;
  int b;
};

class EPnPEigen 
{
 public:
  EPnPEigen(Eigen::MatrixXd& points3d, Eigen::MatrixXd& points2d, Eigen::Matrix3d& K);
  ~EPnPEigen(){}

  void computePose();

  void RunDemo(const std::string& filename);

 private:
  void chooseControlPoints(void);
  void computeBaryCentricCoordinates(void);
  void calculateM(Eigen::MatrixXd& M);
  void computeL6x10(const Eigen::MatrixXd& U, Eigen::MatrixXd& L6x10); // Jesse: U is the matrix of eigen vectors.
  void computeRho(Eigen::VectorXd& rho);
  void findBetasApprox1(const Eigen::MatrixXd& L6x10, const Eigen::VectorXd& rho, double* betas);
  void computeGaussNewtonJacobian(const Eigen::MatrixXd& L6x10, double betas[4], Eigen::MatrixXd& jacobian);
  void computeResiduals(const Eigen::MatrixXd& U, double betas[4], Eigen::VectorXd& residuals);
  void doGaussNewtonOptimization(const Eigen::MatrixXd& U, const Eigen::MatrixXd& L6x10, double betas[4]);
  void computeControlPointsUnderCameraCoord(const Eigen::MatrixXd& U, double betas[4]);
  void computeReferencePointsUnderCameraCoord(void);
  void solveForSign(void);
  void estimateRt(Eigen::Matrix3d& R, Eigen::Vector3d& t);
  void computeRt(const Eigen::MatrixXd&U, double betas[4], Eigen::Matrix3d& R, Eigen::Vector3d& t);

  void randperm(int num, double max_range, Eigen::MatrixXd& reference_3d_points);

  Eigen::MatrixXd reference_3d_points_;
  Eigen::MatrixXd reference_2d_points_;
  Eigen::MatrixXd reference_3d_points_camera_coord_;
  Eigen::MatrixXd control_3d_points_;
  Eigen::MatrixXd control_3d_points_camera_coord_;
  Eigen::MatrixXd bary_centric_coord_;
  int reference_points_count_;
  double fu_, fv_, uc_, vc_;
};

class EPnPEigenDebugTool 
{
 public:
  EPnPEigenDebugTool(){}
  ~EPnPEigenDebugTool(){} 	

  void writeToCSVFile(const std::string& fileName, Eigen::MatrixXd& mat);
  void readFromCSVFile(const std::string& fileName, Eigen::MatrixXd& mat);
  void printForMatlab(const std::string& matName, Eigen::MatrixXd& mat);
  void printForMatlab(const std::string& vecName, Eigen::VectorXd& vec);
  void printForMatlab(const std::string& matName, Eigen::Matrix3d& mat);
  void printForMatlab(const std::string& vecName, Eigen::Vector3d& vec);
}; 

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_EPNP_EIGEN_H

