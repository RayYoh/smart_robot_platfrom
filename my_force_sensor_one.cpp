#include "my_force_sensor_one.h"
#include <vector>
#include "my_math.h"
using namespace std;
MyForceSensorOne::MyForceSensorOne(quint16 port):
    MyForceSensor (port),
    my_force_data(vector<double>(6,0)),
    my_force_zero(vector<double>(6,0)),
    my_change_data(vector<double>(6,0)){

}
void MyForceSensorOne::onNewConnection(){
  pTcpSocket = pTcpServer->nextPendingConnection();//get socket
  connect(pTcpSocket,SIGNAL(disconnected()),this,SLOT(onClientDisconnected()));
  connect(pTcpSocket,SIGNAL(stateChanged(QAbstractSocket::SocketState)),this,SLOT(onSocketStateChange(QAbstractSocket::SocketState)));
  //connect(pTcpSocket,SIGNAL(readyRead()),this,SLOT(onSocketReadyRead()));
  start();
  isCanSend=true;
  isCanSet=true;
  clientConnected();
  return;
}
void MyForceSensorOne::run() {
    while(1){
        if(pTcpSocket->bytesAvailable()<=0)
            continue;
        QByteArray pData=pTcpSocket->read(1);
        if(char(pData[0])==8)
        {
            if(pTcpSocket->bytesAvailable()<=2)
                continue;
            pData=pTcpSocket->read(3);
            if(char(pData[0])==0&&char(pData[1])==0&&char(pData[2])==0)
            {
                if(pTcpSocket->bytesAvailable()<=19)
                    continue;
                pData=pTcpSocket->read(20);
                handleData(pData.data());
                QTime now=QTime::currentTime();
                int elapsed = time_for_compute.msecsTo(now);
                time_for_compute = now;
                cout<<" sensor one data period: "<<elapsed<<" ms "<<endl;
            }
        }
    }
}
void MyForceSensorOne::handleData(char *str){
    if(str[16]!=9||str[17]!=0||str[18]!=0||str[19]!=0)
        return;
    static int count=0;
    ++count;
    static vector<deque<double>> vDqBuf(6,deque<double>(SENSOR_COUNT));
    static vector<double> vDataAvg(6,0);
    vector<double> vDataNow(6,0);
    short data1, data2, data, num;
    data = 0x00FF;

    data1 = data2 = 0;
    data1 = str[4] * 256;
    data2 = str[5] & data;
    num = data1 + data2;
    vDataNow[0] += num / 3.2768;

    data1 = data2 = 0;
    data1 = str[10] * 256;
    data2 = str[11] & data;
    num = data1 + data2;
    vDataNow[1] += num / 3.2768;

    data1 = data2 = 0;
    data1 = str[6] * 256;
    data2 = str[7] & data;
    num = data1 + data2;
    vDataNow[2] += num / 3.2768;

    data1 = data2 = 0;
    data1 = str[12] * 256;
    data2 = str[13] & data;
    num = data1 + data2;
    vDataNow[3] += num / 3.2768;

    data1 = data2 = 0;
    data1 = str[8] * 256;
    data2 = str[9] & data;
    num = data1 + data2;
    vDataNow[4] += num / 3.2768;

    data1 = data2 = 0;
    data1 = str[14] * 256;
    data2 = str[15] & data;
    num = data1 + data2;
    vDataNow[5] += num / 3.2768;

    for(int i=0;i<6;++i)
    {
        double sum=vDataAvg[i]*SENSOR_COUNT-vDqBuf[i].front()+vDataNow[i];
        vDataAvg[i]=sum/SENSOR_COUNT;
        my_force_data[i]=vDataAvg[i];
        vDqBuf[i].pop_front();
        vDqBuf[i].push_back(vDataNow[i]);
    }
    if(isNeedChange)
        changeData();
    Q_EMIT sensorHaveData();
    return;
}
void MyForceSensorOne::setZero(){
    my_force_zero = my_force_data;
    isCanChange=true;
}
void MyForceSensorOne::changeData(){
    vector<vector<double>> A;
    vector<double> vTemp;
    for(int i=0;i<6;++i)
        vTemp.push_back(my_force_data[i]-my_force_zero[i]);
    A.push_back(vTemp);
    vector<vector<double>> B= {{ -0.508355458 ,-0.010221245 ,0.001899991,0.017937002,-0.89303229,-0.078759786 },
                               { 0.01629597,-0.509286966,0.003531495,0.890626257,0.034062921,-0.107769097 },
                               { -0.003018283,0.002707507,0.167344678,0.013296802,5.05802E-05,0.003378879 },
                               { -0.00482948,0.670421448,-0.009244749,-2.482150596,-0.020502753,0.124676258 },
                               { -0.678125117,-0.022025622,0.004429708,0.080445601,-2.492087999,-0.123587686 },
                               { 0.000117831,0.000621971,-0.000255144,-0.000579705,0.002064402,0.777330755 } };
    double ** res = MyMath::Matrix_multip(A,B);
    for(int i=0;i<6;++i)
        my_change_data[i]=res[0][i];
}
vector<double> MyForceSensorOne::getForceDdata(){
    return my_force_data;
}
vector<double> MyForceSensorOne::getChangeData(){
    return my_change_data;
}
