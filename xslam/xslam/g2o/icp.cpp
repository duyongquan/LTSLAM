#include "xslam/g2o/icp.h"

#include "glog/logging.h"

#include <algorithm>
#include <numeric>
#include <iostream>

namespace xslam {
namespace g2o {

void ICP::RunDemo()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(points_nums_, 3);
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    Eigen::Matrix4d T;
    Eigen::Vector3d t1;
    Eigen::Matrix3d R1;

    float total_time = 0;
    unsigned start, end;
    float interval;

    for (int i=0; i < iterations_nums_; i++) 
    {
        B = A;
        t = Eigen::Vector3d::Random()* translation_;

        for( int jj =0; jj< points_nums_; jj++){
            B.block<1,3>(jj,0) = B.block<1,3>(jj,0) + t.transpose();
        }

        R = RotationMatrix(Eigen::Vector3d::Random(), GenerateRandom()* rotation);
        B = (R * B.transpose()).transpose();

        B += Eigen::MatrixXd::Random(points_nums_,3) * noise_sigma_;

        // start = GetTickCount();
        T = BestFitTransform(B,A);
        // end = GetTickCount();
        interval = float((end - start))/1000;
        total_time += interval;

        C = Eigen::MatrixXd::Ones(points_nums_, 4);
        C.block<points_nums_, 3>(0,0) = B;

        C = (T * C.transpose()).transpose();
        t1 = T.block<3,1>(0,3);
        R1 = T.block<3,3>(0,0);

        if(i == 3){
            std::cout << "position error \n" << C.block<points_nums_, 3>(0,0) - A << std::endl << std::endl;
            std::cout << "trans error" << std::endl << -t1 - t << std::endl << std::endl;
            std::cout << "R error" << std::endl << R1.inverse() - R << std::endl << std::endl;
        }
    }
}

Eigen::Matrix4d ICP::BestFitTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
{
   /*
    Notice:
    1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
    2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
    */
    Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4,4);
    Eigen::Vector3d centroid_A(0,0,0);
    Eigen::Vector3d centroid_B(0,0,0);
    Eigen::MatrixXd AA = A;
    Eigen::MatrixXd BB = B;
    int row = A.rows();

    for(int i=0; i<row; i++){
        centroid_A += A.block<1,3>(i,0).transpose();
        centroid_B += B.block<1,3>(i,0).transpose();
    }

    centroid_A /= row;
    centroid_B /= row;

    for(int i=0; i<row; i++){
        AA.block<1,3>(i,0) = A.block<1,3>(i,0) - centroid_A.transpose();
        BB.block<1,3>(i,0) = B.block<1,3>(i,0) - centroid_B.transpose();
    }

    Eigen::MatrixXd H = AA.transpose()*BB;
    Eigen::MatrixXd U;
    Eigen::VectorXd S;
    Eigen::MatrixXd V;
    Eigen::MatrixXd Vt;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    Vt = V.transpose();

    R = Vt.transpose()*U.transpose();

    if (R.determinant() < 0 ){
        Vt.block<1,3>(2,0) *= -1;
        R = Vt.transpose()*U.transpose();
    }

    t = centroid_B - R*centroid_A;

    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return T;
}

ICP::Result ICP::RunICP(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance)
{
     int row = A.rows();
    Eigen::MatrixXd src = Eigen::MatrixXd::Ones(3+1,row);
    Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(3,row);
    Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(3+1,row);
    ICP::Neighbor neighbor;
    Eigen::Matrix4d T;
    Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(3,row);
    ICP::Result result;

    int iter = 0;

    for (int i = 0; i<row; i++){
        src.block<3,1>(0,i) = A.block<1,3>(i,0).transpose();
        src3d.block<3,1>(0,i) = A.block<1,3>(i,0).transpose();
        dst.block<3,1>(0,i) = B.block<1,3>(i,0).transpose();

    }

    double prev_error = 0;
    double mean_error = 0;
    for (int i=0; i<max_iterations; i++){
        neighbor = FindNearest(src3d.transpose(),B);

        for(int j=0; j<row; j++){
            dst_chorder.block<3,1>(0,j) = dst.block<3,1>(0,neighbor.indices[j]);
        }

        T = BestFitTransform(src3d.transpose(),dst_chorder.transpose());

        src = T*src;
        for(int j=0; j<row; j++){
            src3d.block<3,1>(0,j) = src.block<3,1>(0,j);
        }

        mean_error = std::accumulate(neighbor.distances.begin(),neighbor.distances.end(),0.0)/neighbor.distances.size();
        if (abs(prev_error - mean_error) < tolerance){
            break;
        }
        prev_error = mean_error;
        iter = i+2;
    }

    T = BestFitTransform(A,src3d.transpose());
    result.trans = T;
    result.distances = neighbor.distances;
    result.iter = iter;

    return result;
}

// throughout method
ICP::Neighbor ICP::FindNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst)
{
    int row_src = src.rows();
    int row_dst = dst.rows();
    Eigen::Vector3d vec_src;
    Eigen::Vector3d vec_dst;

    ICP::Neighbor neigh;
    float min = 100;
    int index = 0;
    float dist_temp = 0;

    for(int ii=0; ii < row_src; ii++){
        vec_src = src.block<1,3>(ii,0).transpose();
        min = 100;
        index = 0;
        dist_temp = 0;
        for(int jj=0; jj < row_dst; jj++){
            vec_dst = dst.block<1,3>(jj,0).transpose();
            dist_temp = ComputeDistance(vec_src,vec_dst);
            if (dist_temp < min){
                min = dist_temp;
                index = jj;
            }
        }
        // cout << min << " " << index << endl;
        // neigh.distances[ii] = min;
        // neigh.indices[ii] = index;
        neigh.distances.push_back(min);
        neigh.indices.push_back(index);
    }

    return neigh;
}

float ICP::ComputeDistance(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb)
{ 
    return (pta - ptb).norm();

    // return sqrt((pta[0] - ptb[0]) * (pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]) + (pta[2]-ptb[2])*(pta[2]-ptb[2]));
}


// 0-1 float variables
float ICP::GenerateRandom(void)
{
    return (rand() % 100) / 1000;
}

Eigen::Matrix3d ICP::RotationMatrix(Eigen::Vector3d axis, float theta)
{
    axis = axis / sqrt(axis.transpose()*axis);
    float a = cos(theta/2);
    Eigen::Vector3d temp = -axis*sin(theta/2);

    float b,c,d;
    b = temp(0);
    c = temp(1);
    d = temp(2);
    Eigen::Matrix3d R;

    R << a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c),
        2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b),
        2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c;

    return R;
}

} // namespace g2o
} // namespace xslam
