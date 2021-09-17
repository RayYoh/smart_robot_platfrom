#include "drag_controller.h"

DragController::DragController():
    pMyForceSensor(nullptr),
    pMyRobotControl(nullptr)
{
    is_in_tool_coor_=true;
}
void DragController::init_param(MyForceSensor *pSensor, MyRobotControl *pRobot, double frequency,
                                 std::vector<double> K_a,
                                 std::vector<double> current_force,
                                 double arm_max_vel)
{
    pMyForceSensor=pSensor;
    pMyRobotControl=pRobot;
    loop_rate_=frequency;
    duration_sec_=1/frequency;
    K_a_=Matrix6d(K_a.data());
    wrench_external_=Vector6d(current_force.data());
    wrench_external_in_world_=wrench_external_;
    arm_max_vel_=arm_max_vel;
    arm_desired_velocity_.setZero();
}
void DragController::run(){
    while (isCanControl()&&flag) {
      getExtForce();
      if(is_in_tool_coor_)
          compute_force_in_world_coor();
      compute_speed();
      limit_to_workspace();
      send_commands_to_robot();
      QThread::msleep(1000.0/loop_rate_);
    }
    pMyRobotControl->stopRobot();
}
void DragController::getExtForce(){
    vector<double> vecReadForce=pMyForceSensor->getChangeData();
    vector<double> vecExtForce(6,0);
    for(int i=0;i<6;++i){
        vecExtForce[i]=vecReadForce[i];
    }
    wrench_external_=Vector6d(vecExtForce.data());
    wrench_external_in_world_=wrench_external_;
}
void DragController::compute_force_in_world_coor(){
    Vector6d pos_now=getRobotTCPPos();
    pos_now(0)=0;pos_now(1)=0;pos_now(2)=0;
    vector<double> fxfyfz{wrench_external_(0),wrench_external_(1),wrench_external_(2),0,0,0};
    vector<double> mxmymz{wrench_external_(3),wrench_external_(4),wrench_external_(5),0,0,0};
    fxfyfz = MyMath::trans_pose(MyMath::Vector6d_to_stdvector(pos_now),fxfyfz);
    mxmymz = MyMath::trans_pose(MyMath::Vector6d_to_stdvector(pos_now),mxmymz);
    wrench_external_in_world_(0)=fxfyfz[0];
    wrench_external_in_world_(1)=fxfyfz[1];
    wrench_external_in_world_(2)=fxfyfz[2];
    wrench_external_in_world_(3)=mxmymz[0];
    wrench_external_in_world_(4)=mxmymz[1];
    wrench_external_in_world_(5)=mxmymz[2];
    double norm_vel_force = (wrench_external_in_world_.segment(0, 3)).norm();
    double norm_vel_torque = (wrench_external_in_world_.segment(3, 3)).norm();
    if (norm_vel_force < 5) {
        wrench_external_in_world_.segment(0, 3) *= 0;
    }
    if(norm_vel_torque < 5)
        wrench_external_in_world_.segment(3, 3) *= 0;
}
Vector6d DragController::getRobotTCPPos(){
    vector<double> pos=pMyRobotControl->getTCPPos();
    return Vector6d(pos.data());
}
void DragController::compute_speed(){
    arm_desired_velocity_ = K_a_* wrench_external_in_world_;
}
bool DragController::isCanControl(){
    if(!pMyRobotControl||!pMyForceSensor||!pMyRobotControl->isCanControl())
        return false;
    return true;
}
void DragController::limit_to_workspace(){
    double norm_vel_des = (arm_desired_velocity_.segment(0, 3)).norm();
    double norm_vel_rot = (arm_desired_velocity_.segment(3, 3)).norm();
    if (norm_vel_des > arm_max_vel_) {
        //showMessage("Admittance generate fast arm movements! velocity norm: ");
        arm_desired_velocity_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
    }
    if (norm_vel_rot > 0.3) {
        arm_desired_velocity_.segment(3, 3) *= (0.3 / norm_vel_rot);
    }
    if (norm_vel_des < 1e-5)
        arm_desired_velocity_.segment(0,3).setZero();
    if (norm_vel_rot < 1e-5) {
        arm_desired_velocity_.segment(3,3).setZero();
    }
}
void DragController::send_commands_to_robot(){
    vector<double> vel(6,0);
    for(int i=0;i<6;++i){
            vel[i]=arm_desired_velocity_(i);
    }
    pMyRobotControl->velocityControl(vel);
}
void DragController::showMessage(QString text){
    QMessageBox::information(nullptr,"admittance control information",text,QMessageBox::Ok);
}
DragController::~DragController(){
    if(!pMyRobotControl)
        return;
    pMyRobotControl->stopRobot();
}
