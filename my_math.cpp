#include "my_math.h"
vector<vector<double>> MyMath::transpose(vector<vector<double>>& A)
{
    if(A.empty())
        return A;
    double rows=A.size(),cols=A[0].size();
    vector<double> temp(rows,0);
    vector<vector<double>> res(cols,temp);
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {

            res[j][i]=A[i][j];
        }
    }
    return res;
}

double ** MyMath::Matrix_multip(vector<vector<double>>& A,vector<vector<double>>& B)
{
    int rows = A.size();
    int cols = B[0].size();
    double** sum;
    sum = new double *[rows];
    for(int i = 0; i < rows; i ++)
        sum[i] = new double[cols];



    if(A[0].size() != B.size())
    {
        return sum;
    }

    for(int i = 0; i<rows; i++)
    {
        for(int j = 0; j<A[0].size(); j++)
        {

            for(int k = 0; k<B[0].size(); k++)
            {
                sum[i][k] += A[i][j]*B[j][k];
            }
        }
    }

    return sum;
}
vector<double> MyMath::relative_pose_deviation(vector<double> vecOne, vector<double> vecTwo){
    vector<double> vecRes(6,0);
    Matrix4d a_t1_trans=trans_to_matrix(vecOne);
    Matrix4d a_t2_trans=trans_to_matrix(vecTwo);
    Matrix4d matRes = (a_t1_trans.inverse()) * a_t2_trans;
    vecRes[0]=matRes(0,3);
    vecRes[1]=matRes(1,3);
    vecRes[2]=matRes(2,3);
    Matrix3d matRes33=matRes.block(0,0,3,3);
    AngleAxisd rotation_vector_offset(matRes33);
    vecRes[3]=rotation_vector_offset.axis()(0)*rotation_vector_offset.angle();
    vecRes[4]=rotation_vector_offset.axis()(1)*rotation_vector_offset.angle();
    vecRes[5]=rotation_vector_offset.axis()(2)*rotation_vector_offset.angle();
    return vecRes;
}
vector<double> MyMath::trans_pose(vector<double> base, vector<double> offset){
    vector<double> vecRes(6,0);
    Matrix4d a_t_trans=trans_to_matrix(base);
    Vector4d t_pos;
    t_pos<<offset[0],offset[1],offset[2],1;
    Vector4d a_pos=a_t_trans*t_pos;
    vecRes[0]=a_pos(0);
    vecRes[1]=a_pos(1);
    vecRes[2]=a_pos(2);
    return vecRes;
}
Matrix4d MyMath::trans_to_matrix(vector<double> vec){
    Matrix4d matRes;
    double x,y,z;
    x=vec[0];y=vec[1];z=vec[2];
    Vector3d rotation_vector(vec[3],vec[4],vec[5]);
    Vector3d unit_rotation_vector = rotation_vector/rotation_vector.norm();
    Matrix3d rotation_matrix=AngleAxisd(rotation_vector.norm(),unit_rotation_vector).toRotationMatrix();
    Matrix<double, 3, 4> mat34;
    Matrix<double, 3, 1> mat31;
    Matrix<double, 1, 4> mat14;
    mat31<<x,y,z;
    mat34<<rotation_matrix,mat31;
    mat14<<0,0,0,1;
    matRes<<mat34,mat14;
    return matRes;
}
Matrix4d MyMath::x_rotation_matrix(double angle){
    Matrix4d matRes;
    matRes << 1,0,0,0,
            0,cos(angle),-sin(angle),0,
            0,sin(angle),cos(angle),0,
            0,0,0,1;
    return matRes;
}
Matrix4d MyMath::y_rotation_matrix(double angle){
    Matrix4d matRes;
    matRes << cos(angle),0,sin(angle),0,
            0,1,0,0,
            -sin(angle),0,cos(angle),0,
            0,0,0,1;
    return matRes;
}
Matrix4d MyMath::z_rotation_matrix(double angle){
    Matrix4d matRes;
    matRes << cos(angle),-sin(angle),0,0,
            sin(angle),cos(angle),0,0,
            0,0,1,0,
            0,0,0,1;
    return matRes;
}
vector<double> MyMath::rpy_to_rotvec(vector<double> vec){
    vector<double> vecRes(3,0);
    AngleAxisd rotation_vector;
    rotation_vector = Eigen::AngleAxisd(vec[2], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(vec[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(vec[0], Eigen::Vector3d::UnitX());
    double angle=rotation_vector.angle();
    vecRes[0]=rotation_vector.axis()(0)*angle;
    vecRes[1]=rotation_vector.axis()(1)*angle;
    vecRes[2]=rotation_vector.axis()(2)*angle;
    return vecRes;
}
vector<double> MyMath::compute_rotvec_offset(vector<double> vecOne, vector<double> vecTwo){
    Vector3d one(rotvec_to_rpy(vecOne).data());
    Vector3d two(rotvec_to_rpy(vecTwo).data());
    one = one - two;
    return vector<double>{one(0),one(1),one(2)};
}
vector<double> MyMath::rotvec_to_rpy(vector<double> vec){
    Vector3d rotation_vector(vec[0],vec[1],vec[2]);
    double rotation_vector_norm=rotation_vector.norm();
    Vector3d unit_rotation_vector = rotation_vector/rotation_vector_norm;
    Matrix3d rotation_matrix=AngleAxisd(rotation_vector_norm,unit_rotation_vector).toRotationMatrix();
    Vector3d eulerAngle = rotation_matrix.eulerAngles(2,1,0);
    vector<double> vecRes{eulerAngle(0),eulerAngle(1),eulerAngle(2)};
    return vecRes;
}
vector<double> MyMath::Vector6d_to_stdvector(Vector6d vec6d){
    vector<double> vecRes(6,0);
    for(int i=0;i<6;++i){
        vecRes[i]=vec6d(i);
    }
    return vecRes;
}
