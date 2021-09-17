#ifndef MY_ROBOT_LOOP_H
#define MY_ROBOT_LOOP_H
#include <QThread>
#include <vector>
#include "my_robot_control.h"
#include "my_force_sensor_one.h"
#include "my_force_sensor_two.h"
#include "my_math.h"
#include <QTime>
#include <QFile>
class RobotLoop : public QThread
{
public:
    RobotLoop();
    void run();
    void init_param(MyForceSensorOne *pSensorOne,MyForceSensorTwo *pSensorTwo,MyRobotControl *pRobot,QString path);
    void saveLoopData();
    bool flag; //true->loop  false->stop
    QString filePath;
    QString toSave;
    QFile txt;
    int d_num = 0;
private:
    MyRobotControl *pMyRobotControl;
    MyForceSensorOne *pMyForceSensorOne;
    MyForceSensorTwo *pMyForceSensorTwo;
    bool isCanControl();
};
#endif // MY_ROBOT_LOOP_H
