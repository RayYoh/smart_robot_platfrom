#ifndef MY_ROBOT_CONTROL_H
#define MY_ROBOT_CONTROL_H
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <thread>

#include <QString>
#include <iostream>
using namespace ur_rtde;
using namespace std::chrono;
using namespace std;
class MyRobotControl{
public:
    MyRobotControl();
    ~MyRobotControl();
    bool connect(string strIp);
    bool disconnect();
    bool moveRobot(vector<double> &q,double speed=1.05,bool isSpeed=false,bool isIcrem=false,bool isTCP=false);
    vector<double> getJointPos();
    vector<double> getTCPPos();
    vector<double> getTCPForce();
    bool startTeachMode();
    bool endTeachMode();
    bool stopRobot();
    int32_t getRobotStatus();
    bool isCanControl();
    bool velocityControl(vector<double> vel);
    bool servoControl(vector<double> pos,double time);
    bool servoStop();
    bool posControl(vector<double> pos);
    double getRobotControlStepTime();
    vector<double> getRobotJointTorques();
    vector<double> poseTrans(vector<double> base,vector<double> offset);
    bool checkRobotConnect();
    bool checkRobotStatus();
private:
    RTDEControlInterface *pRtdeControl;
    RTDEReceiveInterface *pRtdeReceive;
    void showMessage(QString text);
};
#endif // MY_ROBOT_CONTROL_H
