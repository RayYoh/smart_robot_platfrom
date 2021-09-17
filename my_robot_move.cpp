#include "my_robot_move.h"
RobotMove::RobotMove():
    pMyRobotControl(nullptr),
    pMyForceSensorOne(nullptr),
    pMyForceSensorTwo(nullptr)
{

}
void RobotMove::init_param(MyForceSensorOne *pSensorOne, MyForceSensorTwo *pSensorTwo, MyRobotControl *pRobot,QString path){
    pMyForceSensorOne = pSensorOne;
    pMyForceSensorTwo = pSensorTwo;
    pMyRobotControl = pRobot;
    filePath = path;
}
void RobotMove::run()
{
    if(isCanControl()&&flag){
		/*double joint[16][3] = {{4.00,145.01,14.76},{3.98,-25.37,-15.30},
							{-5.83,124.99,119.93},{-5.85,25.37,104.89},
							{-15.71,74.84,-75.38},{-15.59,-114.96,89.88},
							{-25.27,54.82,119.92},{-25.33,-114.97,14.74},
							{-35.07,-54.84,89.87},{-35.10,34.756,44.79},
							{-45.50,-135.00,-105.46},{-45.41,105.55,14.76},
							{-55.20,-55.44,134.9},{-55.18,145,134.95},
							{-65.10,-4.73,104.91},{-65.11,85.49,44.81},
							{-75.01,-25.39,164.99},{-75.03,94.90,44.80},
							{-85.73,-95.53,59.80},{-85.73,35.37,14.71},
							{-95.70,65.44,-60.40},{-95.66,-115.58,44.76},
							{-105.55,-135.63,89.85},{-105.54,125.58,134.95},
							{-115.40,64.83,164.98},{-115.44,-105.55,74.83},
							{-125.18,104.93,149.96},{-125.21,-24.77,104.88},
							{-135.00,35.39,180.01},{-135.04,-34.80,89.85}};*/
		
                            
		std::vector<double> startJointPos(6, 0), actJointPos(6, 0);
		startJointPos[0] = 90, startJointPos[1] = -90, startJointPos[2] = -90;
		startJointPos[3] = -135, startJointPos[4] = -135, startJointPos[5] = -120;
		pMyRobotControl->moveRobot(startJointPos, 0.8, false, false, false);
		actJointPos = pMyRobotControl->getJointPos();  // Actual Joint Position (deg)
		QString fileName;
		double step_3 = 10, step_4 = 30, step_5 = 60;
		int i = 0;
		for (; actJoint[3] < -4; actJointPos[3] += step_3) {
			for (; , actJointPos[4] < 136; actJoint[4] += step_4) {
				for (; , actJointPos[5] < 121; actJointPos[5] += step_5) {
					i += 1;
					fileName.sprintf("%1", i);
					fileName = filePath + "/" + fileName + ".txt";
					txt.setFileName(fileName);
					txt.open(QIODevice::WriteOnly | QIODevice::Text);
					pMyRobotControl->moveRobot(actJointPos, 0.8, false, false, false);
					for (int j = 0; j < 50; ++j) {
						saveLoopData();
						msleep(50);
					}
					if (!flag)
						break;
					txt.close();
				}
				actJointPos[5] = -120£»
			}
			actJointPos[4] = -135;
		}
			
    }

}
void RobotMove::saveLoopData(){
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
bool RobotMove::isCanControl(){
    if(!pMyRobotControl||!pMyRobotControl->checkRobotConnect())
        return false;
    return true;
}

