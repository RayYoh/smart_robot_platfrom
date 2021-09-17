#ifndef DRAG_CONTROLLER_H
#define DRAG_CONTROLLER_H
#include <QThread>
#include <QMessageBox>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
#include "my_robot_control.h"
#include "my_force_sensor.hpp"
#include "my_math.h"
#include <QTime>
using namespace Eigen;
using namespace ur_rtde;
using namespace std::chrono;
using namespace std;
typedef Matrix<double, 7, 1> Vector7d;  //使用typedef为现有类型创建别名，定义易于记忆的类型名
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
class DragController : public QThread
{
public:
    DragController();
    ~DragController();
    void init_param(MyForceSensor *pSensor, MyRobotControl *pRobot, double frequency,
                         std::vector<double> K_a,
                         std::vector<double> current_force,
                         double arm_max_vel);
    void run();
    bool flag;//true->start  false->stop
private:
    MyForceSensor *pMyForceSensor;
    MyRobotControl *pMyRobotControl;
    Vector6d wrench_external_;
    Vector6d wrench_external_in_world_;
    double loop_rate_;
    double duration_sec_;
    Matrix6d K_a_;
    Vector6d arm_desired_velocity_;
    Vector6d arm_in_tool_coor_velocity_;
    double arm_max_vel_;
    bool is_in_tool_coor_;
    void compute_force_in_world_coor();
    void compute_speed();
    bool isCanControl();
    void send_commands_to_robot();
    void showMessage(QString text);
    void getExtForce();
    void limit_to_workspace();
    Vector6d getRobotTCPPos();
};

#endif // DRAG_CONTROLLER_H
