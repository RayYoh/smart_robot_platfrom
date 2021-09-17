#include "my_robot_loop.h"
RobotLoop::RobotLoop():
    pMyRobotControl(nullptr),
    pMyForceSensorOne(nullptr),
    pMyForceSensorTwo(nullptr)
{

}
void RobotLoop::init_param(MyForceSensorOne *pSensorOne, MyForceSensorTwo *pSensorTwo, MyRobotControl *pRobot,QString path){
    pMyForceSensorOne = pSensorOne;
    pMyForceSensorTwo = pSensorTwo;
    pMyRobotControl = pRobot;
    filePath = path;
}
void RobotLoop::run()
{
    if(isCanControl()&&flag){
        std::vector<double> startJointPos(6,0),actJointPos(6,0);
        startJointPos[0] = 90,startJointPos[1] = -90,startJointPos[2] = -90;
        startJointPos[3] = -130,startJointPos[4] = 60,startJointPos[5] = 90;
        pMyRobotControl->moveRobot(startJointPos,0.1,false,false,false);
        actJointPos = pMyRobotControl->getJointPos();  // Actual Joint Position (deg)
        QString fileName;
        bool isNeedReverse = true;
        double step_3 = 22 , step_4 = 30 , step_5 = 30;
        for (; actJointPos[3] < -21; actJointPos[3] += step_3)
        {
            for (; actJointPos[5] > -94; actJointPos[5] -= step_5)
            {
                if (isNeedReverse)
                {
                    actJointPos[4] = 60;
                    for (; actJointPos[4] < 124; actJointPos[4] += step_4)
                    {
                        if(!flag)
                            break;
                        pMyRobotControl->moveRobot(actJointPos,0.1,false,false,false);
                        actJointPos = pMyRobotControl->getJointPos();  // Actual Joint Position (deg)
                        fileName.sprintf("%4.2lf,%4.2lf,%4.2lf",actJointPos[3],actJointPos[4],actJointPos[5]);
                        fileName = filePath + "/" + fileName + ".txt";
                        txt.setFileName(fileName);
                        txt.open(QIODevice::WriteOnly | QIODevice::Text);
                        d_num=0;
                        for(int i =0 ;i < 200;++i){
                            saveLoopData();
                            msleep(50);
                        }
                        txt.close();
                    }
                    isNeedReverse = !isNeedReverse;
                    if(!flag)
                        break;
                }
                else
                {
                    actJointPos[4] = 120;
                    for (; actJointPos[4] > 56; actJointPos[4] -= step_4)
                    {
                        pMyRobotControl->moveRobot(actJointPos,0.1,false,false,false);
                        actJointPos = pMyRobotControl->getJointPos();  // Actual Joint Position (deg)
                        fileName.sprintf("%4.2lf,%4.2lf,%4.2lf",actJointPos[3],actJointPos[4],actJointPos[5]);
                        fileName = filePath + "/" + fileName + ".txt";
                        txt.setFileName(fileName);
                        txt.open(QIODevice::WriteOnly | QIODevice::Text);
                        d_num=0;
                        for(int i =0 ;i < 200;++i){
                            saveLoopData();
                            msleep(50);
                        }
                        txt.close();
                    }
                    isNeedReverse = !isNeedReverse;
                    if(!flag)
                        break;
                }
            }
            if(!flag)
                break;
            actJointPos[5] = 90;
        }
    }

}
void RobotLoop::saveLoopData(){
    d_num++;
    QString toSave = "No."+QString::number(d_num);
    std::vector<double> vTraction = pMyForceSensorTwo->getForceDdata();
    std::vector<double> vContact = pMyForceSensorOne->getForceDdata();
    std::vector<double> fTraction(12,0) ;
    std::vector<double> fContact(6,0);
    if(pMyForceSensorOne->isNeedChange||pMyForceSensorTwo->isNeedChange){
        fTraction = pMyForceSensorTwo->getChangeData();
        fContact = pMyForceSensorOne->getChangeData();
    }
    toSave+=" Traction sensor V: ";
    for(int i = 0;i < 11;++i){
        toSave += QString("%1,").arg(vTraction[i]);
    }
    toSave+=QString("%1").arg(vTraction[11]);
    toSave+=" Traction sensor F/M: ";
    for(int i = 0;i < 11;++i){
        toSave += QString("%1,").arg(fTraction[i]);
    }
    toSave+=QString("%1").arg(fTraction[11]);
    toSave+=" Contact sensor V: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vContact[i]);
    }
    toSave+=QString("%1").arg(vContact[5]);
    toSave+=" Contact sensor F/M: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(fContact[i]);
    }
    toSave+=QString("%1").arg(fContact[5]);
    std::vector<double> vecTCPPos(6,0);
    std::vector<double> vecJointPos(6,0);
    std::vector<double> vecTCPTor(6,0);
    std::vector<double> vecJointTor(6,0);
//    if(my_robot.checkRobotConnect()){
    vecTCPPos = pMyRobotControl->getTCPPos();
    vecJointPos = pMyRobotControl->getJointPos();
    vecTCPTor = pMyRobotControl->getTCPForce();
    vecJointTor = pMyRobotControl->getRobotJointTorques();
//    }
    toSave+=" Robot TCP position: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecTCPPos[i]);
    }
    toSave+=QString("%1").arg(vecTCPPos[5]);
    toSave+=" Robot Joint position: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecJointPos[i]);
    }
    toSave+=QString("%1").arg(vecJointPos[5]);
    toSave+=" Robot TCP Torque: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecTCPTor[i]);
    }
    toSave+=QString("%1").arg(vecTCPTor[5]);
    toSave+=" Robot Joint Torque: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecJointTor[i]);
    }
    toSave+=QString("%1").arg(vecJointTor[5]);
    txt.write(toSave.toUtf8()+"\n");
}
bool RobotLoop::isCanControl(){
    if(!pMyRobotControl||!pMyRobotControl->checkRobotConnect())
        return false;
    return true;
}

