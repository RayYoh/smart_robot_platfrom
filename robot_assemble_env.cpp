#include "robot_assemble_env.h"

RobotAssembleEnv::RobotAssembleEnv()
{
    depth=0.1;
}
void RobotAssembleEnv::set_param(MyRobotControl *robot, MyForceSensor *sensor, AdmittanceController *controller){
    pRobot = robot;
    pForceSensor = sensor;
    pAdmController = controller;
}
StepReturn RobotAssembleEnv::step(int action){
    if(!isCanControl())
        return StepReturn();
    double distanceX=0.01;
    double distanceY=0.01;
    double distanceZ=0.01;
    double angleX=M_PI/180*10;
    double angleY=M_PI/180*10;
    double angleZ=M_PI/180*10;
    Eigen::Matrix<double, 6, 1> vec;
    vec.setZero();
    switch (action) {
    case 0:
        vec(0)=distanceX;break;
    case 1:
        vec(0)=-distanceX;break;
    case 2:
        vec(1)=distanceY;break;
    case 3:
        vec(1)=-distanceY;break;
    case 4:
        vec(2)=distanceZ;break;
    case 5:
        vec(2)=-distanceZ;break;
    case 6:
        vec(3)=angleX;break;
    case 7:
        vec(3)=-angleX;break;
    case 8:
        vec(4)=angleY;break;
    case 9:
        vec(4)=-angleY;break;
    case 10:
        vec(5)=angleZ;break;
    case 11:
        vec(5)=-angleZ;break;
    }
    pAdmController->changeAimPosIncrement(vec);
    admParam+=Eigen::Matrix<double, 6, 1>(vec.data());
    QThread::msleep(1000);
    StepReturn res;
    res.obs=getObservation();
    res.reword=computeReword();
    res.isDone=isDone();
    return res;
}
TaskObservation RobotAssembleEnv::reset(){
    pAdmController->stopControll();
    std::vector<double> q=MyMath::Vector6d_to_stdvector(initPos);
    pRobot->moveRobot(q,1.05,false,false,true);
    return getObservation();

}
TaskObservation RobotAssembleEnv::getObservation(){
    TaskObservation obs;
    obs.vecTCPOffset=MyMath::relative_pose_deviation(pRobot->getTCPPos(),MyMath::Vector6d_to_stdvector(initPos));
    obs.vecForceData=pForceSensor->getChangeData();
    obs.vecAdmParam=MyMath::Vector6d_to_stdvector(admParam);
    return obs;
}
double RobotAssembleEnv::computeReword(){
    if(isReachAim())
        return 100;
    if(isBeyondLimits())
        return -10;
    double oneMeterReward=10/depth;
    std::vector<double> offset=getNowOffset();
    double reword=0;
    reword=(abs(offset[0])+abs(offset[1]))*-oneMeterReward;
    reword+=offset[2]*oneMeterReward*(offset[2]>0?1.0:-1.0);
    return reword;
}
bool RobotAssembleEnv::isReachAim(){
    std::vector<double> offset=getNowOffset();
    if(abs(offset[0])<0.001&&abs(offset[1])<0.001&&abs(offset[2]-depth)<0.001)
        return true;
    return false;
}
bool RobotAssembleEnv::isBeyondLimits(){
    double maxForce=100,maxTorque=100;
    double box=0.01;
    std::vector<double> offset=getNowOffset();
    if(abs(offset[0])>box||abs(offset[1])>box||offset[2]>box+depth||offset[2]<0)
        return true;
    Eigen::Matrix<double, 6, 1> force(pForceSensor->getChangeData().data());
    if(force.segment(0, 3).norm()>maxForce||force.segment(3, 3).norm()>maxTorque)
        return true;
    return false;
}
std::vector<double> RobotAssembleEnv::getNowOffset(){
    return MyMath::relative_pose_deviation(pRobot->getTCPPos(),MyMath::Vector6d_to_stdvector(initPos));
}
bool RobotAssembleEnv::isDone(){
    if(isReachAim()||isBeyondLimits())
        return true;
    return false;
}
bool RobotAssembleEnv::isCanControl(){
    if(!pRobot||!pForceSensor||!pAdmController)
        return false;
    return true;
}
