#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H
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
using namespace Eigen;
using namespace ur_rtde;
using namespace std::chrono;
using namespace std;

typedef Matrix<double, 7, 1> Vector7d;  //使用typedef为现有类型创建别名，定义易于记忆的类型名
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class AdmittanceController : public QThread
{
    Q_OBJECT
public:
    AdmittanceController();
    ~AdmittanceController();
    void init_param(MyForceSensor *pSensor, MyRobotControl *pRobot, double frequency,
                         std::vector<double> M_a,
                         std::vector<double> D_a,
                         std::vector<double> K_a,
                         std::vector<double> current_force,
                         std::vector<double> workspace_limits,
                         double arm_max_vel,
                         double arm_max_acc,
                         std::vector<bool> vec_is_can_move);
    void run();
    // --- INPUT SIGNAL --- //
    // external wrench (force/torque sensor) in "robotiq_force_torque_frame_id" frame
    Vector6d wrench_external_;
    Vector6d wrench_external_in_world_;
    void setExtForce(vector<double> vecForce);
    void setControlForce(vector<double> vecForce);
    void setInToolCoor(bool is);
    void changeAimPosIncrement(Vector6d vec);
    void stopControll();

    bool flag;//true->start  false->stop
private:
    MyForceSensor *pMyForceSensor;
    MyRobotControl *pMyRobotControl;
    double loop_rate_;
    double duration_sec_;
    // --- ADMITTANCE PARAMETERS --- //
    // M_a_ -> Desired mass of arm
    // D_a_ -> Desired damping of arm
    Matrix6d M_a_, D_a_,K_a_;
    // --- OUTPUT COMMANDS --- //
    // final arm desired velocity
    Vector6d arm_desired_velocity_;
    Vector6d arm_in_tool_coor_velocity_;
    Vector6d aim_position_;
    Vector6d ee_position_;
    Vector6d should_position_;
    // limiting the workspace of the arm
    Vector6d workspace_limits_;
    std::vector<bool> vec_is_can_move_;
    double arm_max_vel_;
    double arm_max_acc_;
    bool is_in_tool_coor_;
    // Initialization
    void wait_for_transformations();

    // Control
    void compute_admittance();
    void limit_to_workspace();

    // void publish_debuggings_signals();

    void send_commands_to_robot();
    void showMessage(QString text);
    void getExtForce();
    Vector6d getRobotTCPPos();
    void compute_ee();
    void compute_force_in_world_coor();
    bool isCanControl();
};

#endif // ADMITTANCE_CONTROLLER_H
