#include "my_robot_control.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "dlib/svm.h"
#include <chrono>
#include <thread>
#include <QMessageBox>

using namespace ur_rtde;
using namespace std::chrono;

MyRobotControl::MyRobotControl():
    pRtdeControl(nullptr),
    pRtdeReceive(nullptr){

}
bool MyRobotControl::connect(string strIp){
    try {
        pRtdeControl = new RTDEControlInterface(strIp);
        pRtdeReceive = new RTDEReceiveInterface(strIp);
    } catch (boost::wrapexcept<boost::system::system_error> &e) {
        showMessage("connect failed:"+QString(e.what()));
        return false;
    }
    return true;
}
bool MyRobotControl::disconnect(){
    if(checkRobotConnect())
    {
        pRtdeControl->disconnect();
        pRtdeControl=nullptr;
        return true;
    }
    return true;
}
void MyRobotControl::showMessage(QString text){
    QMessageBox::information(nullptr,"connect information",text,QMessageBox::Ok);
}
bool MyRobotControl::moveRobot(vector<double> &q,double speed,bool isSpeed,bool isIcrem,bool isTCP){
    //q[cm,cm,cm,deg,deg,deg] isTCP=True or q[deg,deg,deg,deg,deg,deg] isTCP=False
    //isSpeed-Speed Contropl
    //isIcrem-Incremental or not
    //isTCP-isTCPPos or QPos
    vector<double> vDouTemp(6,0);
    if(!checkRobotConnect()||!checkRobotStatus())
        return false;
    if(isSpeed){
        for(int i=0;i<3;++i)
            q[i]/=1000.0;
        return velocityControl(q);
    }
    for(int i=0;i<3;++i)
        q[i+3]=q[i+3]/180.0*3.1415926535898;
    if(isTCP){
        vDouTemp=pRtdeReceive->getActualTCPPose();
        for(int i=0;i<3;++i)
            q[i]/=1000.0;
    }
    else{
        vDouTemp=pRtdeReceive->getActualQ();
        for(int i=0;i<3;++i)
            q[i]=q[i]/180.0*3.1415926535898;
    }
    if(isIcrem){
        for(int i=0;i<6;++i)
            vDouTemp[i]+=q[i];
    }
    if(isTCP)
    {
        if(isIcrem)
            return pRtdeControl->moveL(vDouTemp,speed);
        else {
            return pRtdeControl->moveL(q,speed);
        }
    }
    else {
        if(isIcrem)
            return pRtdeControl->moveJ(vDouTemp,speed);
        else {
            return pRtdeControl->moveJ(q,speed);
        }
    }
    return false;
}
vector<double> MyRobotControl::getTCPPos(){
    vector<double> vecRes;
    if(!checkRobotConnect())
    {
        return vecRes;
    }
    vector<double> vRes = pRtdeReceive->getActualTCPPose();   // m
    return vRes;
}
vector<double> MyRobotControl::getJointPos(){
    vector<double> vecRes;
    if(!checkRobotConnect())
    {
        return vecRes;
    }
    vector<double> vRes = pRtdeReceive->getActualQ(); //rad
    for(int i = 0;i < 6; ++i){
        vRes[i] = vRes[i] / 3.1415926535898 * 180;
    }
    return vRes; //deg
}
bool MyRobotControl::startTeachMode(){
    if(!checkRobotConnect()||!checkRobotStatus())
        return false;
    pRtdeControl->teachMode();
    return true;
}
bool MyRobotControl::endTeachMode(){
    pRtdeControl->endTeachMode();
    return true;
}
bool MyRobotControl::checkRobotConnect(){
    if(!pRtdeControl)
    {
        showMessage("operate failed:\nno legal entity,entity was nullptr,try reconnect robot");
        return false;
    }
    if(!pRtdeReceive)
    {
        showMessage("operate failed:\nno legal entity,entity was nullptr,try reconnect robot");
        return false;
    }
    if(!pRtdeReceive->isConnected())
    {
        showMessage("operate failed:\nrobot is disconnected,try reconnect robot");
        return false;
    }
    return true;
}
bool MyRobotControl::checkRobotStatus(){
    if(!checkRobotConnect()){
        return false;
    }
    if(3!=pRtdeReceive->getRobotStatus())
    {
        showMessage("operate failed:\nrobot not in power button pressed status(3),try reconnect robot");
        return false;
    }
    return true;
}
int32_t MyRobotControl::getRobotStatus(){
    if(!checkRobotConnect()){
        return -1;
    }
    return pRtdeReceive->getRobotStatus();
}
bool MyRobotControl::stopRobot(){
    if(!checkRobotConnect()||!checkRobotStatus())
        return false;
    return pRtdeControl->speedStop();
}
bool MyRobotControl::isCanControl(){
    return checkRobotStatus()&&checkRobotConnect();
}
bool MyRobotControl::velocityControl(vector<double> vel){
    if(!checkRobotConnect()||!checkRobotStatus())
        return false;
    return pRtdeControl->speedL(vel,2);
}
bool MyRobotControl::servoControl(vector<double> pos,double time){
    if(!checkRobotConnect()||!checkRobotStatus())
        return false;
    return pRtdeControl->servoL(pos,1,1,time,0.03,500);
}
bool MyRobotControl::servoStop(){
    if(!checkRobotConnect())
        return false;
    return pRtdeControl->servoStop();
}
bool MyRobotControl::posControl(vector<double> pos){
    if(!checkRobotConnect()||!checkRobotStatus())
        return false;
    return pRtdeControl->moveL(pos);
}
vector<double> MyRobotControl::getTCPForce(){
    vector<double> vecRes;
    if(!checkRobotConnect()||!checkRobotStatus())
        return vecRes;
    vecRes = pRtdeReceive->getActualTCPForce();
//    ostringstream oss;//创建一个流
//    for(int i=0;i<6;++i){
//        oss<<" torques "<<i<<" : "<<vecRes[i];
//    }
//    showMessage(QString::fromStdString(oss.str()));
    return vecRes;

}
double MyRobotControl::getRobotControlStepTime(){
    if(!checkRobotConnect())
        return 0;
    double res = pRtdeControl->getStepTime();
    showMessage(QString::number(res));
    return res;
}
vector<double> MyRobotControl::getRobotJointTorques(){
    vector<double> vecRes;
    if(!checkRobotConnect())
        return vecRes;
    vecRes = pRtdeControl->getJointTorques();
//    ostringstream oss;//创建一个流
//    for(int i=0;i<6;++i){
//        oss<<" torques "<<i<<" : "<<vecRes[i];
//    }
//    showMessage(QString::fromStdString(oss.str()));
    return vecRes;
}
vector<double> MyRobotControl::poseTrans(vector<double> base, vector<double> offset){
    vector<double> vecRes(6,0);
    if(!checkRobotConnect()||!checkRobotStatus())
        return vecRes;
    return pRtdeControl->poseTrans(base,offset);
}
MyRobotControl::~MyRobotControl(){
    pRtdeControl->speedStop();
    delete pRtdeControl;
    delete pRtdeReceive;
}
