#include "my_force_sensor_two.h"
#include <vector>
#include "my_math.h"
using namespace std;
MyForceSensorTwo::MyForceSensorTwo(quint16 port):
    MyForceSensor (port),
    my_force_data(vector<double>(12,0)),
    my_force_zero(vector<double>(12,0)),
    my_change_data(vector<double>(12,0)){

}
void MyForceSensorTwo::handleData(char *str){
    if(str[52]!=-112||str[53]!=0)
        return;
    unsigned char *buf=reinterpret_cast<unsigned char*>(str);
    for (int i = 0; i < 12; ++i)
    {
        double temp = buf[4 * i] + 256 * buf[4 * i + 1] + 65536 * buf[4 * i + 2] + 16777216 * buf[4 * i + 3];
        if(temp<0||temp>4098)
            break;
        my_force_data[i] = temp;
    }
    if(isNeedChange)
        changeData();
    Q_EMIT sensorHaveData();
    return;
}
void MyForceSensorTwo::setZero(){
    my_force_zero = my_force_data;
    isCanChange=true;
}
void MyForceSensorTwo::onSocketReadyRead(){
    if(pTcpSocket->bytesAvailable()<=0)
        return;
    QByteArray pData=pTcpSocket->read(1);
    if(char(pData[0])==-128)
    {
        if(pTcpSocket->bytesAvailable()<=0)
            return;
        pData=pTcpSocket->read(1);
        if(char(pData[0])==0)
        {
            if(pTcpSocket->bytesAvailable()<=53)
                return;
            pData=pTcpSocket->read(54);
            handleData(pData.data());
            QTime now=QTime::currentTime();
            int elapsed = time_for_compute.msecsTo(now);
            time_for_compute = now;
            cout<<" sensor two data period: "<<elapsed<<" ms "<<endl;
            pData=pTcpSocket->readAll();
        }
    }
}
void MyForceSensorTwo::changeData(){
    //channel 0-5
    vector<vector<double>> A;
    vector<double> vTemp;
    for(int i=0;i<6;++i)
        vTemp.push_back(my_force_data[i]-my_force_zero[i]);
    A.push_back(vTemp);
    vector<vector<double>> B= {{ -1.06757643e-01,  5.20550999e-03, -6.22006907e-03, -2.26032213e-02,   5.22933560e-01,  1.36975131e-01 },
                               { -2.45234650e-03,  5.43822626e-02, -1.79622663e-03,  2.20193398e-01,  -1.74248531e-01, -4.14177489e-02 },
                               { -6.06251806e-04,  3.12473233e-04, -9.07207803e-02, -3.49536451e-02,   5.60471178e-02,  1.60876609e-02 },
                               { -3.67237616e-03,  5.91225813e-03, -1.03404021e-02,  2.61955616e+00,   1.42475863e-01, -4.04999434e-02 },
                               { -5.63763009e-03,  3.64921045e-03,  2.87568275e-04, -5.48347003e-02,   2.59400255e+00,  8.91082840e-03 },
                               { -6.65255892e-04, -7.34467602e-04,  2.89176676e-03,  6.98912637e-02,  -9.17903129e-02, -2.11648267e+00 }};
    double ** res = MyMath::Matrix_multip(A,B);
    vector<double> vecRes(6,0);
    for(int i=0;i<6;++i)
        vecRes[i]=res[0][i];
    double K_FY_MY_P = 0.3084875168920069, K_FY_MY_M = -0.2938884533792768, K_FY_MX_P = -33.49203950066071, K_FY_MX_M = -33.490723600600504,
           K_FX_MY_P = 33.392538783763094, K_FX_MY_M = 33.568888194970796, K_FX_MX_P = 0.31892260297354963, K_FX_MX_M = -0.32496705056979464;
    if (vecRes[1] >= 0)
    {
        vecRes[3] = vecRes[3] - K_FY_MX_P * vecRes[1];
        vecRes[4] = vecRes[4] - K_FY_MY_P * vecRes[1];
    }
    else
    {
        vecRes[3] = vecRes[3] - K_FY_MX_M * vecRes[1];
        vecRes[4] = vecRes[4] - K_FY_MY_M * vecRes[1];
    }

    if (vecRes[0] >= 0)
    {
        vecRes[3] = vecRes[3] - K_FX_MX_P * vecRes[0];
        vecRes[4] = vecRes[4] - K_FX_MY_P * vecRes[0];
    }
    else
    {
        vecRes[3] = vecRes[3] - K_FX_MX_M * vecRes[0];
        vecRes[4] = vecRes[4] - K_FX_MY_M * vecRes[0];
    }
    for (int i = 0; i < 3; ++i)
    {
        vecRes[i + 3] /= 10;
    }
    my_change_data[0]=vecRes[0];
    my_change_data[1]=-vecRes[1];
    my_change_data[2]=-vecRes[2];
    my_change_data[3]=vecRes[3];
    my_change_data[4]=-vecRes[4];
    my_change_data[5]=-vecRes[5];
    //channel 6-11
    for(int i=0;i<6;++i)
        A[0][i]=(my_force_data[i+6]-my_force_zero[i+6]);
    B={{ -0.508355458 ,-0.010221245 ,0.001899991,0.017937002,-0.89303229,-0.078759786 },
         { 0.01629597,-0.509286966,0.003531495,0.890626257,0.034062921,-0.107769097 },
         { -0.003018283,0.002707507,0.167344678,0.013296802,5.05802E-05,0.003378879 },
         { -0.00482948,0.670421448,-0.009244749,-2.482150596,-0.020502753,0.124676258 },
         { -0.678125117,-0.022025622,0.004429708,0.080445601,-2.492087999,-0.123587686 },
         { 0.000117831,0.000621971,-0.000255144,-0.000579705,0.002064402,0.777330755 }};
    res = MyMath::Matrix_multip(A,B);
    for(int i=0;i<6;++i)
        my_change_data[i+6]=res[0][i];
}
vector<double> MyForceSensorTwo::getForceDdata(){
    return my_force_data;
}
vector<double> MyForceSensorTwo::getChangeData(){
    return my_change_data;
}

