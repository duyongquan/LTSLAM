#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <cstdlib>

#include "glog/logging.h"
#include "gtest/gtest.h"



namespace xslam {
namespace eigen {

const int N = 1005;
// typedef double Type;
using Type = double;
 
Type A[N][N], L[N][N];
 
// 分解A得到A = L * L^T 
void Cholesky(Type A[][N], Type L[][N], int n)
{
    for(int k = 0; k < n; k++)
    {
        Type sum = 0;
        for(int i = 0; i < k; i++)
            sum += L[k][i] * L[k][i];

        sum = A[k][k] - sum;
        L[k][k] = sqrt(sum > 0 ? sum : 0);
        for(int i = k + 1; i < n; i++) {
            sum = 0;
            for(int j = 0; j < k; j++)
                sum += L[i][j] * L[k][j];
            L[i][k] = (A[i][k] - sum) / L[k][k];
        }

        for(int j = 0; j < k; j++)
            L[j][k] = 0;
    }
}
 
// 回带过程
std::vector<Type> Solve(Type L[][N], std::vector<Type> X, int n)
{
    /** LY = B  => Y */
    for(int k = 0; k < n; k++) {
        for(int i = 0; i < k; i++)
            X[k] -= X[i] * L[k][i];
        X[k] /= L[k][k];
    }

    /** L^TX = Y => X */
    for(int k = n - 1; k >= 0; k--) {
        for(int i = k + 1; i < n; i++)
            X[k] -= X[i] * L[i][k];
        X[k] /= L[k][k];
    }
    return X;
}
 
void Print(Type L[][N], const std::vector<Type> B, int n)
{
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < n; j++) {
           std::cout << L[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    std::vector<Type> X = Solve(L, B, n);
    std::vector<Type>::iterator it;
    for(it = X.begin(); it != X.end(); it++) {
         std::cout << *it << " ";
    }
    std::cout << std::endl;
}

TEST(Cholesky, myself)
{
    const int n = 4;
    memset(L, 0, sizeof(L));

    // 4 -2 4 2
    // -2 10 -2 -7
    // 4 -2 8 4
    // 2 -7 4 7
    A[0][0] = 4;
    A[0][1] = -2;
    A[0][2] = 4;
    A[0][3] = 2;

    A[1][0] = -2;
    A[1][1] = 10;
    A[1][2] = -2;
    A[1][3] = -7;

    A[2][0] = 4;
    A[2][1] = -2;
    A[2][2] = 8;
    A[2][3] = 4;

    A[3][0] = 2;
    A[3][1] = -7;
    A[3][2] = 4;
    A[3][3] = 7;

    A[4][0] = 8;
    A[4][1] = 2;
    A[4][2] = 16;
    A[4][3] = 6;

     // 8 2 16 6
    std::vector<Type> B;
    B.push_back(8);
    B.push_back(2);
    B.push_back(16);
    B.push_back(6);

    // Solve
    Cholesky(A, L, n);
    Print(L, B, n);
} 


TEST(Cholesky, eigen)
{
    Eigen::Matrix4d A;
    A << 4, -2, 4, 2,
        -2, 10, -2, -7,
         4, -2, 8, 4,
         2, -7, 4, 7;

    Eigen::Vector4d b;
    b << 8, 2, 16, 6;

    // Cholesky Solve
    auto x = A.llt().solve(b);
    std::cout << "solver x : " << x;
}

} // namespace eigen
} // namespace xslam  