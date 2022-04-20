#include "xslam/g2o/g2o_bal.h"

#include "glog/logging.h"

namespace xslam {
namespace g2o {

/*==========================================================================================
 * 
 *                                       VertexCameraBAL  
 * 
 *==========================================================================================*/

void VertexCameraBAL::oplusImpl(const double* update)
{
   Vector9d::ConstMapType v ( update );

    Sophus::Vector6d SE3_Rt;
    //se3是平移在前，旋转在后，另外这里的平移不是我们平时使用的平移
    SE3_Rt << _estimate[3],_estimate[4],_estimate[5] ,_estimate[0],_estimate[1], _estimate[2];

    Sophus::Vector6d update_se3;
    update_se3 << update[3],update[4],update[5],update[0],update[1],update[2];

    // Sophus::SE3 Update_SE3 = Sophus::SE3::exp(update_se3)*Sophus::SE3::exp(SE3_Rt);
    // Vector9d u;
    // u << Update_SE3.log()[3],Update_SE3.log()[4],Update_SE3.log()[5],
    //         Update_SE3.log()[0],Update_SE3.log()[1],Update_SE3.log()[2],
    //         _estimate[6] + v[6],_estimate[7] +v[7],_estimate[8] +v[8];
    // _estimate = u;
}

/*==========================================================================================
 * 
 *                                       VertexPointBAL  
 * 
 *==========================================================================================*/

void VertexPointBAL::oplusImpl(const double* update)
{
    Eigen::Vector3d::ConstMapType v ( update );
    _estimate += v;
}


/*==========================================================================================
 * 
 *                                       EdgeObservationBAL  
 * 
 *==========================================================================================*/

void EdgeObservationBAL::computeError()
{
    //这里将第0个顶点，相机位姿取出来。
    const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));

    //这里将第1个顶点，空间点位置取出来。
    const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));

    Eigen::Vector2d u;
    CamProjectionWithDistortion(cam->estimate(), point->estimate(), u );
    _error[0] = _measurement[0] - u[0];
    _error[1] = _measurement[1] - u[1];
}

void EdgeObservationBAL::linearizeOplus()
{
    //这里将第0个顶点，相机位姿取出来。
    const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
    //这里将第1个顶点，空间点位置取出来。
    const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );

    //_jacobianOplusXi 为 e 对 T , f , k1, k2 求导 , 其中T为6维.故 Xi 为 2*9
    //_jacobianOplusXi = Eigen::Matrix<double,2,9>::Zero();
    //_jacobianOplusXj 为 e 对 空间点 求导 ,故 Xj 为 2*3
    //_jacobianOplusXj = Eigen::Matrix<double,2,3>::Zero();

    Eigen::Vector3d P_w;
    Eigen::Vector3d P_c;
    Vector9d camera_param;

    P_w = point->estimate();
    camera_param = cam->estimate();
    World2Camera(camera_param,P_w,P_c);

    double f = camera_param[6];
    double k1 = camera_param[7];
    double k2 = camera_param[8];

    double x_z =  P_c[0]/ P_c[2];
    double y_z =  P_c[1]/ P_c[2];

    double x =  P_c[0];
    double y =  P_c[1];
    double z =  P_c[2];

    double p2 = x_z*x_z + y_z*y_z;
    double r =  1.0 + k1 * p2 + k2 * p2 * p2;
    double fr = f * r;

    Eigen::Matrix<double,2,3> de_dp;
    Eigen::Matrix<double,2,3> dp_dE;
    Eigen::Vector2d de_df;
    Eigen::Vector2d de_dk1;
    Eigen::Vector2d de_dk2;

    double dr2_dx = 2 * x / (z * z);
    double dr2_dy = 2 * y / (z * z);
    //double dr2_dz = -2 * ( x*x + y*y)/ (z*z*z);

    //    | u |   | f * (-Xc/Zc) *(1 + k1 * ||p_c|| ^ 2 + k2 * ||p_c|| ^ 4 ) |
    //e = |   | - |                                                          |
    //    | v |   | f * (-Yc/Zc) *(1 + k1 * ||p_c|| ^ 2 + k2 * ||p_c|| ^ 4 ) |
    //
    // ||p_c|| = sqrt(x_c * x_c + y_c * y_c) , 代码中用p2表示

    // e[0] = u + f * (Xc/Zc) *(1 + k1 * ||p_c|| ^ 2 + k2 * ||p_c|| ^ 4 )
    // e[1] = v + f * (Yc/Zc) *(1 + k1 * ||p_c|| ^ 2 + k2 * ||p_c|| ^ 4 )

    // de      de       dP'
    //---- = ------ * ------
    // dE      dP'      dE
    // 请把这里的E看成为左扰动,这里矩阵大小为 2 * 6

    //e对相机坐标系下的空间点p求导, 这里矩阵大小为 2 * 3
    //具体形式如下：
    // du         f        f * Xc           2*Xc                             2*Xc
    //------ = ------*r + --------*( k1 * ------- + 2 * k2 * ||p_c|| ^ 2 * -------  )
    // dXc       Zc          Zc             Zc^2                             Zc^2

    // du       f * Xc           2*Yc                             2*Yc
    //----- = -------- * ( k1 * ------- + 2 * k2 * ||p_c|| ^ 2 * ------- )
    // dYc       Zc              Zc^2                             Zc^2

    // du       -f*r*Xc       - 2*f*Xc*p2
    //----- = ----------- + --------------- * ( k1 + 2 * k2 * p2 )
    // dZc        Zc^2            Zc^2
    double k = k1 + 2*k2*p2;

    de_dp(0,0) = fr/z + f* x_z * dr2_dx * k;
    de_dp(0,1) = f*x_z * dr2_dy * k;
    de_dp(0,2) = -fr*x/(z*z)- 2*f*x*p2 * k/(z*z);

    // dv/dXc dv/dYc dv/dZc 推导同上 ,如果对这个矩阵了解的话,其中规律也很容易看出来
    de_dp(1,0) = f*x_z * dr2_dy * k;
    de_dp(1,1) = fr/z + f*y_z * dr2_dy * k;
    de_dp(1,2) = -fr*y/(z*z) - 2*f*y*p2* k/(z*z);

    //p对左扰动求导,具体可以看书上的推导,dP'/dE 只取前三行 ,形式为 [I, -P'^],这里矩阵大小为 3 * 6
    //| 1 0 0   0    Zc -YC|
    //| 0 1 0  -Zc   0   Xc|
    //| 0 0 1  Yc   -Xc   0|
    dp_dE(0,0) = de_dp(0,1) * (-z) + de_dp(0,2) * y;
    dp_dE(0,1) = de_dp(0,0) * z    + de_dp(0,2) * (-x);
    dp_dE(0,2) = de_dp(0,0) * (-y) + de_dp(0,1) * x;

    dp_dE(1,0) = de_dp(1,1) * (-z) + de_dp(1,2) * y;
    dp_dE(1,1) = de_dp(1,0) * z    + de_dp(1,2) * (-x);
    dp_dE(1,2) = de_dp(1,0) * (-y) + de_dp(1,1) * x;

    //e 对 f 进行求导 ,这里矩阵大小为 2 * 1
    //       Xc
    // r * ------
    //       Zc
    de_df[0] = x/z*r;
    de_df[1] = y/z*r;

    //e 对 k1 进行求导 ,这里矩阵大小为 2 * 1
    //       Xc
    // f * ------  * ||p_c||^2
    //       Zc
    de_dk1[0] = f*x/z*p2;
    de_dk1[1] = f*y/z*p2;

    //e 对 k2 进行求导 ,这里矩阵大小为 2 * 1
    //       Xc
    // f * ------  * ||p_c||^4
    //       Zc
    de_dk2[0] = f*x/z*p2*p2;
    de_dk2[1] = f*y/z*p2*p2;

    _jacobianOplusXi.block<2,3>(0,0) = dp_dE;
    _jacobianOplusXi.block<2,3>(0,3) = de_dp;
    _jacobianOplusXi.block<2,1>(0,6) = de_df;
    _jacobianOplusXi.block<2,1>(0,7) = de_dk1;
    _jacobianOplusXi.block<2,1>(0,8) = de_dk2;

      Eigen::Vector3d angleAxis(camera_param[0],camera_param[1], camera_param[2]);
    // _jacobianOplusXj = de_dp * Sophus::SO3::exp(angleAxis).matrix();
}

/*==========================================================================================
 * 
 *                                       LoadBALProblem  
 * 
 *==========================================================================================*/
void LoadBALProblem::ReadData()
{
    std::ifstream fin(filename_);
    if(!fin)
    {
        LOG(INFO) << "Error opening .txt file";
        return;
    }
    fin >> num_cameras_;
    fin >> num_points_;
    fin >> num_observations_;

    //输出第一行数据,这里包括，相机的姿态数量,空间点的数量,观测数据数量
    LOG(INFO) << "pose number: " << num_cameras_ ;
    LOG(INFO)  << "point number: " << num_points_  ;
    LOG(INFO)  << "observations number: " << num_observations_ ;

    //根据读入的数据来初始化数组
    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    //开始读取观测数据
    for (int i = 0; i < num_observations_; ++i)
    {
        fin>>camera_index_[i];
        fin>>point_index_[i];
        fin>>observations_[2*i];
        fin>>observations_[2*i+1];
    }

    //这里读取本文文件里一行一个数字的内容,相机的参数有9个，分别为R,t,f,k1,k2,这个R是旋转向量,空间为三个一组
    observations_cameras_ = 9*num_cameras_;
    observations_points_ = 3*num_points_;
    observations_cameras = new double[observations_cameras_];
    observations_points = new double[observations_points_];

    //读取相机初始数据
    for (int i = 0; i < observations_cameras_; ++i)
    {
        fin>>observations_cameras[i];
    }

    //读取空间初始数据
    for (int i = 0; i < observations_points_; ++i)
    {
        fin>>observations_points[i];
    }
}

void LoadBALProblem::WriteToPLYFile(const std::string& filename)
{
    std::ofstream of(filename.c_str());

    of<< "ply"
      << '\n' << "format ascii 1.0"
      << '\n' << "element vertex " << num_cameras_ + num_points_
      << '\n' << "property float x"
      << '\n' << "property float y"
      << '\n' << "property float z"
      << '\n' << "property uchar red"
      << '\n' << "property uchar green"
      << '\n' << "property uchar blue"
      << '\n' << "end_header" << std::endl;

    for(int i = 0; i < num_cameras(); ++i)
    {


        Eigen::Vector3d P_c;
        Eigen::Vector3d P_w;

        Eigen::Vector3d angleAxis(observations_cameras[9*i+0],
                                  observations_cameras[9*i+1],
                                  observations_cameras[9*i+2]);

        P_c[0] = observations_cameras[9*i+3];
        P_c[1] = observations_cameras[9*i+4];
        P_c[2] = observations_cameras[9*i+5];

        Camera2World(angleAxis,P_c, P_w);
        of << P_w[0] << ' ' << P_w[1] << ' ' << P_w[2]<< ' '
           << "0 255 0" << '\n';
    }

    // Export the structure (i.e. 3D Points) as white points.
    for(int i = 0; i < num_points(); ++i){
        for(int j = 0; j < 3; ++j)
        {
            of << observations_points[3*i+j] << ' ';
        }
        //加上颜色
        of << "255 255 255\n";
    }

    of.close();
}

void LoadBALProblem::Camera2World(const Eigen::Vector3d angleAxis, const Eigen::Vector3d P_c, Eigen::Vector3d& P_w)
{
    // cv::Mat Rcw;
    // Sophus::Vector6d Tcw;
    // Tcw <<P_c[0], P_c[1], P_c[2], angleAxis[0],angleAxis[1],angleAxis[2];
    // Eigen::Matrix4d Twc;
    // //Twc = Tcw^-1
    // Twc = Sophus::SE3::exp(Tcw).matrix().inverse();
    // P_w[0] = Twc(0,3);
    // P_w[1] = Twc(1,3);
    // P_w[2] = Twc(2,3);
}

/*==========================================================================================
 * 
 *                                       LargeBA  
 * 
 *==========================================================================================*/
void LargeBA::RunDemo(const std::string& problem_filename)
{
  // const string filename = "../problem-16-22106-pre.txt";
  //const string filename = "../problem-52-64053-pre.txt";
  SolveBALProblem(problem_filename);
}

void LargeBA::SolveBALProblem(const std::string filename)
{
    //生成初始数据
    LoadBALProblem loadBALProblem(filename);
    loadBALProblem.ReadData();

    //生成初始为未被优化的ply点云文件
    loadBALProblem.WriteToPLYFile("../old.ply");

    //创建一个稀疏优化器对象
    ::g2o::SparseOptimizer optimizer;

    //使用稀疏求解器
    //    g2o::LinearSolver<BalBlockSolver::PoseMatrixType>* linearSolver =
    //            new g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>();
    //    dynamic_cast<g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);

    ::g2o::LinearSolver<BalBlockSolver::PoseMatrixType>* linearSolver =
            new ::g2o::LinearSolverCSparse<BalBlockSolver::PoseMatrixType>();
    dynamic_cast<::g2o::LinearSolverCSparse<BalBlockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);

    // //矩阵块求解器
    // BalBlockSolver* solver_ptr = new BalBlockSolver(linearSolver);

    // //使用LM算法
    // ::g2o::OptimizationAlgorithmLevenberg* solver = new ::g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    // //优化器构建完成
    // optimizer.setAlgorithm(solver);

    // //打开调试输出
    // optimizer.setVerbose(true);

    // //增加pose顶点
    // const int num_cam = loadBALProblem.num_cameras();
    // for(int i = 0; i < num_cam; i++)
    // {
    //     Vector9d temVecCamera;
    //     for (int j = 0; j < 9; j++)
    //     {
    //         temVecCamera[j] = loadBALProblem.num_observations_cameras(9*i+j);
    //     }
    //     VertexCameraBAL* pCamera = new VertexCameraBAL();
    //     pCamera->setEstimate(temVecCamera);
    //     pCamera->setId(i);
    //     optimizer.addVertex(pCamera);

    // }

    // //增加路标顶点
    // const int point_num = loadBALProblem.num_points();
    // for(int i = 0; i < point_num; i++)
    // {
    //     Eigen::Vector3d temVecPoint;
    //     for (int j = 0; j < 3; j++)
    //     {
    //         temVecPoint[j] = loadBALProblem.num_observations_points(3*i+j);
    //     }
    //     VertexPointBAL* pPoint = new VertexPointBAL();
    //     pPoint->setEstimate(temVecPoint);

    //     //这里加上pose的数量,避免跟pose的ID重复
    //     pPoint->setId(i + num_cam);
    //     pPoint->setMarginalized(true);
    //     optimizer.addVertex(pPoint);
    // }
    // //增加边
    // const int  num_observations =loadBALProblem.num_observations();
    // for(int i = 0; i < num_observations; ++i)
    // {
    //     EdgeObservationBAL* bal_edge = new EdgeObservationBAL();

    //     //得到相机ID和空间点ID
    //     const int camera_id = loadBALProblem.camera_index(i);
    //     const int point_id = loadBALProblem.point_index(i) + num_cam;

    //     //使用了鲁棒核函数
    //     ::g2o::RobustKernelHuber* rk = new ::g2o::RobustKernelHuber;
    //     rk->setDelta(1.0);
    //     bal_edge->setRobustKernel(rk);


    //     bal_edge->setVertex(0,dynamic_cast<VertexCameraBAL*>(optimizer.vertex(camera_id)));
    //     bal_edge->setVertex(1,dynamic_cast<VertexPointBAL*>(optimizer.vertex(point_id)));
    //     bal_edge->setInformation(Eigen::Matrix2d::Identity());
    //     bal_edge->setMeasurement(Eigen::Vector2d(loadBALProblem.num_observations_uv(2*i + 0),
    //                                              loadBALProblem.num_observations_uv(2*i + 1)));

    //     optimizer.addEdge(bal_edge);
    // }

    // optimizer.initializeOptimization();
    // optimizer.setVerbose(true);
    // optimizer.optimize(20);

    // double* cameras = loadBALProblem.ReWritePoses();
    // for(int i = 0; i < num_cam; i++)
    // {
    //     VertexCameraBAL* pCamera = dynamic_cast<VertexCameraBAL*>(optimizer.vertex(i));
    //     Vector9d NewCameraVec = pCamera->estimate();
    //     memcpy(cameras + i * 9, NewCameraVec.data(), sizeof(double) * 9);
    // }

    // double* points = loadBALProblem.ReWritePoints();
    // for(int j = 0; j < point_num; j++)
    // {
    //     VertexPointBAL* pPoint = dynamic_cast<VertexPointBAL*>(optimizer.vertex(j + num_cam));
    //     Eigen::Vector3d NewPointVec = pPoint->estimate();
    //     memcpy(points + j * 3, NewPointVec.data(), sizeof(double) * 3);
    // }

    loadBALProblem.WriteToPLYFile("../new.ply");
    LOG(INFO)<<"finished..." ;
}

void World2Camera(const Vector9d camera, const Eigen::Vector3d P_w, Eigen::Vector3d& P_c)
{
    //这里的非齐次坐标的变换要注意
    Eigen::Vector4d Pw(P_w[0],P_w[1],P_w[2],1.0);
    Sophus::Vector6d se3_RT;
    se3_RT << camera[3],camera[4], camera[5],camera[0],camera[1], camera[2];

    // Eigen::Vector4d P = Sophus::SE3::exp(se3_RT).matrix() * Pw;
    // P_c[0] = P[0];
    // P_c[1] = P[1];
    // P_c[2] = P[2];
}

inline void CamProjectionWithDistortion(const Vector9d camera, const Eigen::Vector3d point, Eigen::Vector2d& u)
{
    Eigen::Vector3d p;
    World2Camera(camera, point, p);

    // Compute the center fo distortion
    double xp = -p[0]/p[2];
    double yp = -p[1]/p[2];

    // Apply second and fourth order radial distortion
    const double k1 = camera[7];
    const double k2 = camera[8];

    double r2 = xp*xp + yp*yp;
    double distortion = 1.0 + k1*r2 + k2*r2*r2 ;

    const double f = camera[6];
    u[0] = f * distortion * xp;
    u[1] = f * distortion * yp;
}

} // namespace g2o
} // namespace xslam