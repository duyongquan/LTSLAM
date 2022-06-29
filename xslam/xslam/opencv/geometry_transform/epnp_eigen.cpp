#include "xslam/opencv/geometry_transform/epnp_eigen.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>

namespace xslam {
namespace opencv {
namespace geometry_transform {

void EPnPEigen::RunDemo(const std::string& filename)
{
  EPnPEigenDebugTool eigen_debug_tool;
  Eigen::MatrixXd Pw(100, 3);
  Eigen::MatrixXd pc(100, 2);
  double R_pnp[3][3] = { 0.0 };
  double T_pnp[3] = { 0.0 }; 

  Eigen::Matrix3d K;

  K << 120.0, 0, 321.0,
       0, 122.0, 238.0,
       0,     0,     1;

  eigen_debug_tool.readFromCSVFile("reference_3d_points.csv", Pw);
  eigen_debug_tool.readFromCSVFile("reference_2d_points.csv", pc);
  //eigen_debug_tool.printForMatlab("Pc = ", pc);

  // EPnPEigen pnpEigenSolver(Pw, pc, K);

  std::cout << "I: solve pose by EPnPEigen." << std::endl;
  computePose();
}

EPnPEigen::EPnPEigen(Eigen::MatrixXd& points3d, Eigen::MatrixXd& points2d, Eigen::Matrix3d& K) 
{
  reference_3d_points_ = points3d;
  reference_2d_points_ = points2d;  
  reference_points_count_ = reference_3d_points_.rows();

  control_3d_points_ = Eigen::MatrixXd::Zero(4, 3);
  control_3d_points_camera_coord_ = Eigen::MatrixXd::Zero(4, 3);
  bary_centric_coord_ = Eigen::MatrixXd::Zero(reference_points_count_, 4);
  reference_3d_points_camera_coord_ = Eigen::MatrixXd::Zero(reference_points_count_, 3);

  fu_ = K(0, 0);
  fv_ = K(1, 1);
  uc_ = K(0, 2);
  vc_ = K(1, 2);
}

void EPnPEigen::chooseControlPoints(void){
  double lambda;
  Eigen::VectorXd eigvec;
  Eigen::MatrixXd pointsSum = reference_3d_points_.colwise().sum();
  pointsSum = pointsSum/reference_points_count_;
  control_3d_points_.row(0) = pointsSum;

  Eigen::MatrixXd centroidMat = pointsSum.replicate(reference_points_count_, 1);
  Eigen::MatrixXd PW0 = reference_3d_points_ - centroidMat;
  Eigen::MatrixXd PW0t = PW0;
  PW0t.transposeInPlace();
  Eigen::MatrixXd PW0tPW0 = PW0t * PW0;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(PW0tPW0);
  Eigen::VectorXd eigenval = es.eigenvalues();
  Eigen::VectorXd k = (eigenval/reference_points_count_).cwiseSqrt();

  // Jesse: will not be included in the release code.
  int sign_value[3] = {1, -1, -1};

  for (int i = 2; i >= 0; i--){
    lambda = k(i);
    eigvec = es.eigenvectors().col(i);
    control_3d_points_.row(3-i) = control_3d_points_.row(0) + sign_value[i] * lambda * eigvec.transpose();
  }
}

void EPnPEigen::computeBaryCentricCoordinates()
{
  Eigen::MatrixXd CC(3,3);

  for (int i = 0; i < 3; i++){
    CC.row(i) = control_3d_points_.row(i+1) - control_3d_points_.row(0);
  }	
  CC.transposeInPlace();

  Eigen::MatrixXd CC_inv = CC.inverse();

  Eigen::MatrixXd pt_3d_diff_mat(1,3);
  Eigen::Vector3d pt_3d_diff_vec;
  double a;
  for (int i = 0; i < reference_points_count_; i++){
    pt_3d_diff_mat = reference_3d_points_.row(i) - control_3d_points_.row(0);
    pt_3d_diff_vec = pt_3d_diff_mat.transpose();
    pt_3d_diff_vec = CC_inv * pt_3d_diff_vec;
    a = 1.0 - pt_3d_diff_vec.sum();
    bary_centric_coord_(i,0) = a;
    bary_centric_coord_.block(i, 1, 1, 3) = pt_3d_diff_vec.transpose();
  }
}


void EPnPEigen::calculateM(Eigen::MatrixXd& M)
{
  double uci, vci, barycentric;
  for (int i = 0; i < reference_points_count_; i++){
  	uci = uc_ - reference_2d_points_(i, 0);
    vci = vc_ - reference_2d_points_(i, 1);
    for (int j = 0; j < 4; j++){
      barycentric = bary_centric_coord_(i,j);
      M(2*i, 3*j    ) = barycentric*fu_;
      M(2*i, 3*j + 1) = 0;
      M(2*i, 3*j + 2) = barycentric*uci;

      M(2*i + 1, 3*j    ) = 0;
      M(2*i + 1, 3*j + 1) = barycentric*fv_;
      M(2*i + 1, 3*j + 2) = barycentric*vci;
    }  	
  }
}


void EPnPEigen::computeL6x10(const Eigen::MatrixXd& U, Eigen::MatrixXd& L6x10)
{
  Eigen::MatrixXd V = U.block(0, 0, 12, 4);
  Eigen::MatrixXd DiffMat = Eigen::MatrixXd::Zero(18, 4);
  DistPattern diff_pattern[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

  for (int i = 0; i < 6; i++){
    DiffMat.block(3*i, 0, 3, 4) = V.block(3 * diff_pattern[i].a, 0, 3, 4) - V.block(3 * diff_pattern[i].b, 0, 3, 4);  	
  } 	

  Eigen::Vector3d v1, v2, v3, v4;
  for (int i = 0; i < 6; i++){
  	v1 = DiffMat.block(3*i, 0, 3, 1);
  	v2 = DiffMat.block(3*i, 1, 3, 1);
  	v3 = DiffMat.block(3*i, 2, 3, 1);
  	v4 = DiffMat.block(3*i, 3, 3, 1);

  	L6x10.block(i, 0, 1, 10) << v1.dot(v1), 2*v1.dot(v2), v2.dot(v2), 2*v1.dot(v3), 2*v2.dot(v3), 
  	                            v3.dot(v3), 2*v1.dot(v4), 2*v2.dot(v4), 2*v3.dot(v4), v4.dot(v4);
  }
}


void EPnPEigen::computeRho(Eigen::VectorXd& rho){
  Eigen::Vector3d control_point_a, control_point_b, control_point_diff;
  DistPattern diff_pattern[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

  for (int i = 0; i < 6; i++){
    control_point_a << control_3d_points_(diff_pattern[i].a, 0), control_3d_points_(diff_pattern[i].a, 1), control_3d_points_(diff_pattern[i].a, 2);
    control_point_b << control_3d_points_(diff_pattern[i].b, 0), control_3d_points_(diff_pattern[i].b, 1), control_3d_points_(diff_pattern[i].b, 2);
    control_point_diff = control_point_a - control_point_b;

    rho(i) = control_point_diff.dot(control_point_diff);
  }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]
// for N = 4
void EPnPEigen::findBetasApprox1(
  const Eigen::MatrixXd& L6x10, const Eigen::VectorXd& rho, double* betas)
{
  Eigen::MatrixXd L6x4(6, 4);

  L6x4.block(0, 0, 6, 2) = L6x10.block(0, 0, 6, 2);
  L6x4.block(0, 2, 6, 1) = L6x10.block(0, 3, 6, 1);
  L6x4.block(0, 3, 6, 1) = L6x10.block(0, 6, 6, 1);

  Eigen::VectorXd B = L6x4.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rho);

  if (B(0) < 0) {
    betas[0] = sqrt(-B(0));
    betas[1] = -B(1) / betas[0];
    betas[2] = -B(2) / betas[0];
    betas[3] = -B(3) / betas[0];
  } else {
    betas[0] = sqrt(B(0));
    betas[1] = B(1) / betas[0];
    betas[2] = B(2) / betas[0];
    betas[3] = B(3) / betas[0];
  }
}

void EPnPEigen::computeResiduals(
  const Eigen::MatrixXd& U, double betas[4], Eigen::VectorXd& residuals)
{
  Eigen::MatrixXd V = U.block(0, 0, 12, 4);
  DistPattern diff_pattern[6] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
  Eigen::VectorXd CC(12, 1);
  Eigen::Vector3d Ca, Cb;
  Eigen::MatrixXd Wa, Wb;
  Eigen::Vector3d Vwa, Vwb;

  CC = betas[0] * V.block(0, 0, 12, 1) + betas[1] * V.block(0, 1, 12, 1) + 
    betas[2] * V.block(0, 2, 12, 1) + betas[3] * V.block(0, 3, 12, 1);

  for (int i = 0; i < 6; i++) {
    Ca = CC.block(3*diff_pattern[i].a, 0, 3, 1);
    Cb = CC.block(3*diff_pattern[i].b, 0, 3, 1);
    Wa = control_3d_points_.block(diff_pattern[i].a, 0, 1, 3);
    Wb = control_3d_points_.block(diff_pattern[i].b, 0, 1, 3);

    Ca = Ca - Cb;
    Cb = Ca;
    double d1 = Ca.dot(Cb);

    Wa = Wa - Wb;
    Wa.transposeInPlace();
    Vwa = Wa;
    Vwb = Vwa;
    double d2 = Vwa.dot(Vwb);

    residuals(i) = d1 - d2;  	
  }
}


void EPnPEigen::computeGaussNewtonJacobian(
  const Eigen::MatrixXd& L6x10, double betas[4], Eigen::MatrixXd& jacobian)
{
  Eigen::MatrixXd L2J(10, 4);

  L2J << 2*betas[0],          0,          0,          0,  // 0
           betas[1],   betas[0],          0,          0,  // 1
                  0, 2*betas[1],          0,          0,  // 2
           betas[2],          0,   betas[0],          0,  // 3
                  0,   betas[2],   betas[1],          0,  // 4
                  0,          0, 2*betas[2],          0,  // 5
           betas[3],          0,          0,   betas[0],  // 6
                  0,   betas[3],          0,   betas[1],  // 7
                  0,          0,   betas[3],   betas[2],  // 8
                  0,          0,          0, 2*betas[3];  // 9

  jacobian = L6x10 * L2J;
}


void EPnPEigen::doGaussNewtonOptimization(
  const Eigen::MatrixXd& U, const Eigen::MatrixXd& L6x10, double betas[4])
{
  const int iterations_number = 5;
  Eigen::MatrixXd jacobian(6, 4);
  Eigen::VectorXd residuals(6, 1);
  Eigen::MatrixXd JtJ(4, 4), JtJ_inv(4, 4);
  //Eigen::Map<Eigen::Vector4d> Vb(betas);
  Eigen::Vector4d Vb;
  Eigen::Vector4d jacobian_res;

  Vb << betas[0], betas[1], betas[2], betas[3];
  for (int i = 0; i < iterations_number; i++){
    computeGaussNewtonJacobian(L6x10, betas, jacobian);
    computeResiduals(U, betas, residuals);

    JtJ = jacobian.transpose() * jacobian;
    JtJ_inv = JtJ.inverse(); 
    jacobian_res = jacobian.transpose() * residuals;

    Vb = Vb - JtJ_inv * jacobian_res;

    betas[0] = Vb(0);
    betas[1] = Vb(1);
    betas[2] = Vb(2);
    betas[3] = Vb(3);
  }
  
}


void EPnPEigen::computeControlPointsUnderCameraCoord(const Eigen::MatrixXd& U, double betas[4])
{
  Eigen::MatrixXd V = U.block(0, 0, 12, 4);
  Eigen::MatrixXd control_3d_points_camera_coord_vector(12, 1);

  control_3d_points_camera_coord_vector = betas[0] * V.block(0, 0, 12, 1) + betas[1] * V.block(0, 1, 12, 1) +
                                          betas[2] * V.block(0, 2, 12, 1) + betas[3] * V.block(0, 3, 12, 1);
  
  for (int i = 0; i < 4; i++){
    control_3d_points_camera_coord_.block(i, 0, 1, 3) << control_3d_points_camera_coord_vector(3*i), 
                                                         control_3d_points_camera_coord_vector(3*i + 1),
                                                         control_3d_points_camera_coord_vector(3*i + 2);
  }
}


void EPnPEigen::computeReferencePointsUnderCameraCoord(void)
{
  reference_3d_points_camera_coord_ = bary_centric_coord_ * control_3d_points_camera_coord_;
}


void EPnPEigen::solveForSign(void)
{
  if (reference_3d_points_camera_coord_(0,2) < 0){
    control_3d_points_camera_coord_ = -1 * control_3d_points_camera_coord_;
    reference_3d_points_camera_coord_ = -1 * reference_3d_points_camera_coord_;
  }
}


void EPnPEigen::estimateRt(Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
  Eigen::MatrixXd pointsSum = reference_3d_points_.colwise().sum();
  pointsSum = pointsSum/reference_points_count_;
  Eigen::Vector3d P0w = pointsSum.transpose();

  Eigen::MatrixXd centroidMat = pointsSum.replicate(reference_points_count_, 1);
  Eigen::MatrixXd Piw = reference_3d_points_ - centroidMat;
  
  pointsSum = reference_3d_points_camera_coord_.colwise().sum();
  pointsSum = pointsSum/reference_points_count_;
  Eigen::Vector3d P0c = pointsSum.transpose();
  centroidMat = pointsSum.replicate(reference_points_count_, 1);
  Eigen::MatrixXd Pic = reference_3d_points_camera_coord_ - centroidMat;

  Eigen::Matrix3d M = Pic.transpose() * Piw;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();

  R = U*V.transpose();
  double detR = R.determinant();

  if (detR < 0){
    R(2,0) = -R(2,0);
    R(2,1) = -R(2,1);
    R(2,2) = -R(2,2);    
  }

  t = P0c - R*P0w;  

  std::cout << "detR = " << detR << std::endl;

  EPnPEigenDebugTool tool;
  tool.printForMatlab("M = ", M);
  tool.printForMatlab("U = ", U);
  tool.printForMatlab("V = ", V);
  tool.printForMatlab("R = ", R);
  tool.printForMatlab("t = ", t);
}


void EPnPEigen::computeRt(
  const Eigen::MatrixXd&U, 
  double betas[4], 
  Eigen::Matrix3d& R, 
  Eigen::Vector3d& t)
{
  computeControlPointsUnderCameraCoord(U, betas);
  computeReferencePointsUnderCameraCoord();
  solveForSign();

  estimateRt(R, t);
}


void EPnPEigen::randperm(int num, double max_range, Eigen::MatrixXd& reference_3d_points)
{
  std::vector<int> temp;

  std::srand((unsigned)time(NULL));

  for (int i = 0; i < 3*num; i++){
    temp.push_back(rand());  	
  }


  std::random_shuffle(temp.begin(), temp.end());

  for (int i = 0; i < num; i++){
    //cout << "index = " << i << ": " << temp[3*i]/300.0 << ", " << temp[3*i+1]/300.0 << ", " << temp[3*i+2]/300.0 << endl;
    reference_3d_points(i, 0) = temp[3*i] * max_range/RAND_MAX;
    reference_3d_points(i, 1) = temp[3*i + 1] * max_range/RAND_MAX;
    reference_3d_points(i, 2) = temp[3*i + 2] * max_range/RAND_MAX; 
  }
}

void EPnPEigen::computePose()
{
  chooseControlPoints();
  computeBaryCentricCoordinates();	

  Eigen::MatrixXd M(2*reference_points_count_, 12);
  M = Eigen::MatrixXd::Zero(2*reference_points_count_, 12);
  calculateM(M);
  
  Eigen::MatrixXd MtM = M.transpose() * M;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(MtM);
  Eigen::VectorXd eigenval = es.eigenvalues();
  Eigen::MatrixXd eigvector = es.eigenvectors();

  eigvector.block(0, 2, 12, 1) = -eigvector.block(0, 2, 12, 1);  // Jesse: only for debug
  eigvector.block(0, 3, 12, 1) = -eigvector.block(0, 3, 12, 1);  // Jesse: only for debug

  Eigen::MatrixXd L6x10 = Eigen::MatrixXd::Zero(6, 10);
  Eigen::VectorXd rho(6, 1);

  computeL6x10(eigvector, L6x10);
  computeRho(rho);

  double betas[4][4];
  findBetasApprox1(L6x10, rho, betas[1]);
  printf("betas[1] = %f, %f, %f, %f\n", betas[1][0], betas[1][1], betas[1][2], betas[1][3]);
  doGaussNewtonOptimization(eigvector, L6x10, betas[1]);
  printf("betas[1] = %f, %f, %f, %f\n", betas[1][0], betas[1][1], betas[1][2], betas[1][3]);  
  Eigen::Matrix3d R1 = Eigen::Matrix3d::Zero(3,3);
  Eigen::Vector3d t1 = Eigen::Vector3d::Zero(3,1);
  computeRt(eigvector, betas[1], R1, t1); 
}

const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
const static Eigen::IOFormat MatlabFormat(Eigen::FullPrecision, 0, ", ", "\n", "", "", "[", "]");

void EPnPEigenDebugTool::writeToCSVFile(const std::string& fileName, Eigen::MatrixXd& mat)
{
  std::ofstream csvfile(fileName.c_str());
  csvfile << mat.format(CSVFormat);    
}

void EPnPEigenDebugTool::readFromCSVFile(const std::string& fileName, Eigen::MatrixXd& mat)
{
  std::ifstream indata;
  indata.open(fileName);
  std::string line;
  unsigned int rows = 0, cols;

  while(getline(indata, line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    cols = 0;
    while (getline(lineStream, cell, ',')){
      mat(rows, cols) = stod(cell);
      cols++;
    }  	
    rows++;
    std::cout << std::endl;
  }	

  indata.close();
}

void EPnPEigenDebugTool::printForMatlab(const std::string& matName, Eigen::MatrixXd& mat)
{
  std::cout << matName << mat.format(MatlabFormat);
  std::cout << ";" << std::endl;	
}

void EPnPEigenDebugTool::printForMatlab(const std::string& vecName, Eigen::VectorXd& vec)
{	
  std::cout << vecName << vec.format(MatlabFormat);	
  std::cout << ";" << std::endl;
}

void EPnPEigenDebugTool::printForMatlab(const std::string& matName, Eigen::Matrix3d& mat)
{
  std::cout << matName << mat.format(MatlabFormat);
  std::cout << ";" << std::endl;    
}

void EPnPEigenDebugTool::printForMatlab(const std::string& vecName, Eigen::Vector3d& vec)
{
  std::cout << vecName << vec.format(MatlabFormat);  
  std::cout << ";" << std::endl;  
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam