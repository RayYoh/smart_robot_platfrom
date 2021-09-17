#ifndef ROBOT_ASSEMBLE_ENV_H
#define ROBOT_ASSEMBLE_ENV_H

#include "admittance_controller.h"
#include <QThread>
#include <QTime>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
#include "my_robot_control.h"
#include "my_force_sensor.hpp"
#include <QMessageBox>
#include <iostream>
#include "my_math.h"
#include <dlib/dnn.h>
#include <dlib/data_io.h>

using namespace dlib;
struct TaskObservation{
    std::vector<double> vecTCPOffset;
    std::vector<double> vecForceData;
    std::vector<double> vecAdmParam;
};
struct StepReturn{
    TaskObservation obs;
    double reword;
    bool isDone;
};
class RobotAssembleEnv
{
public:
    RobotAssembleEnv();
    void set_param(MyRobotControl *robot,MyForceSensor *sensor,AdmittanceController *controller);
    StepReturn step(int action);
    TaskObservation reset();
private:
    MyRobotControl *pRobot;
    MyForceSensor *pForceSensor;
    AdmittanceController *pAdmController;
    Eigen::Matrix<double, 6, 1> initPos;
    Eigen::Matrix<double, 6, 1> admParam;
    double depth;
    TaskObservation getObservation();
    double computeReword();
    bool isReachAim();
    bool isBeyondLimits();
    std::vector<double> getNowOffset();
    bool isDone();
    bool isCanControl();
};

#endif // ROBOT_ASSEMBLE_ENV_H
