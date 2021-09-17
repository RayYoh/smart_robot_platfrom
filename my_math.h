#ifndef MY_MATH_H
#define MY_MATH_H
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
using namespace Eigen;
using namespace std;
typedef Matrix<double, 7, 1> Vector7d;  //使用typedef为现有类型创建别名，定义易于记忆的类型名
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 4, 4> Matrix4d;
typedef Matrix<double, 3, 3> Matrix3d;
class MyMath {
public:
    static vector<vector<double>> transpose(vector<vector<double>>& A);
    static double ** Matrix_multip(vector<vector<double>>& A,vector<vector<double>>& B);
    static vector<double> relative_pose_deviation(vector<double> vecOne,vector<double> vecTwo);
    static Matrix4d trans_to_matrix(vector<double> vec);
    static Matrix4d x_rotation_matrix(double angle);
    static Matrix4d y_rotation_matrix(double angle);
    static Matrix4d z_rotation_matrix(double angle);
    static vector<double> rpy_to_rotvec(vector<double> vec);
    static vector<double> compute_rotvec_offset(vector<double> vecOne,vector<double> vecTwo);
    static vector<double> rotvec_to_rpy(vector<double> vec);
    static vector<double> Vector6d_to_stdvector(Vector6d vec6d);
    static vector<double> trans_pose(vector<double> base,vector<double> offset);
};
#endif // MY_MATH_H
