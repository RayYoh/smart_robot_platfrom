#include "admittance_controller.h"

AdmittanceController::AdmittanceController():
    pMyForceSensor(nullptr),
    pMyRobotControl(nullptr)
{
    is_in_tool_coor_=false;
}
void AdmittanceController::init_param(MyForceSensor *pSensor,MyRobotControl *pRobot, double frequency,
                     std::vector<double> M_a,
                     std::vector<double> D_a,
                     std::vector<double> K_a,
                     std::vector<double> current_force,
                     std::vector<double> workspace_limits,
                     double arm_max_vel,
                     double arm_max_acc,
                     std::vector<bool> vec_is_can_move)
{
    pMyForceSensor=pSensor;
    pMyRobotControl=pRobot;
    loop_rate_=frequency;
    duration_sec_=1/frequency;
    M_a_=Matrix6d(M_a.data());
    D_a_=Matrix6d(D_a.data());
    K_a_=Matrix6d(K_a.data());
    wrench_external_=Vector6d(current_force.data());
    wrench_external_in_world_=wrench_external_;
    workspace_limits_=Vector6d(workspace_limits.data());
    arm_max_vel_=arm_max_vel;
    arm_max_acc_=arm_max_acc;
    // initializing the class variables
    aim_position_=getRobotTCPPos();
    // Init integrator
    arm_desired_velocity_.setZero();
    should_position_.setZero();
    vec_is_can_move_=vec_is_can_move;
}

void AdmittanceController::run(){
    if(!isCanControl())
        return;
    while (pMyRobotControl->isCanControl()&&flag) {
      // Admittance Dynamics computation
      compute_ee();
      getExtForce();
      if(is_in_tool_coor_)
          compute_force_in_world_coor();
      compute_admittance();
      // sum the vel from admittance to DS in this function
      //limit_to_workspace();
      // Here I can do the "workspace-modulation" idea

      // Copy commands to messages
      //compute_speed_in_tool_corr();
      send_commands_to_robot();
      QThread::msleep(1000.0/loop_rate_);
    }
    pMyRobotControl->stopRobot();
}
void AdmittanceController::compute_admittance(){
    // Vector6d platform_desired_acceleration;
    Vector6d arm_desired_accelaration;
    arm_desired_accelaration = M_a_.inverse() * ( -D_a_ * arm_desired_velocity_
                               -K_a_ * ee_position_
                               + wrench_external_in_world_);
    // limiting the accelaration for better stability and safety
    double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();
    if (a_acc_norm > arm_max_acc_) {
        //showMessage("Admittance generates high arm accelaration!");
        arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
    }
    // Integrate for velocity based interface
    arm_desired_velocity_  += arm_desired_accelaration * duration_sec_;
    for(int i=0;i<6;++i)
        if(vec_is_can_move_[i]==false)
            arm_desired_velocity_(i)=0.0;
    should_position_+=arm_desired_velocity_*duration_sec_;
}
void AdmittanceController::limit_to_workspace(){
    // velocity of the arm along x, y, and z axis
    double norm_vel_des = (arm_desired_velocity_.segment(0, 3)).norm();

    if (norm_vel_des > arm_max_vel_) {
        showMessage("Admittance generate fast arm movements! velocity norm: ");
        arm_desired_velocity_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
    }
    if (norm_vel_des < 1e-5)
      arm_desired_velocity_.segment(0,3).setZero();
    if (arm_desired_velocity_(3) < 1e-5)
        arm_desired_velocity_(3) = 0;
    if (arm_desired_velocity_(4)< 1e-5)
        arm_desired_velocity_(4) = 0;
    if (arm_desired_velocity_(5) < 1e-5)
        arm_desired_velocity_(5) = 0;
    // velocity of the arm along x, y, and z angles
    if (arm_desired_velocity_(3) > 0.3)
        arm_desired_velocity_(3) = 0.3;
    if (arm_desired_velocity_(4) > 0.3)
        arm_desired_velocity_(4) = 0.3;
    if (arm_desired_velocity_(5) > 0.3)
        arm_desired_velocity_(5) = 0.3;
}
void AdmittanceController::send_commands_to_robot(){
    // for the arm
    vector<double> vel(6,0);
    ostringstream oss;//创建一个流
    for(int i=0;i<6;++i){
            vel[i]=arm_desired_velocity_(i);
        oss<<" vel "<<i<<" : "<<vel[i];
    }
//    cout<<oss.str();
//    showMessage(QString::fromStdString(oss.str()));
//    vel[3]=0;
//    vel[4]=0;
//    vel[5]=0;
    pMyRobotControl->velocityControl(vel);
//    pMyRobotControl->velocityControlInToolCoor(MyMath::Vector6d_to_stdvector(aim_position_),vel);
}
void AdmittanceController::showMessage(QString text){
    QMessageBox::information(nullptr,"admittance control information",text,QMessageBox::Ok);
}
void AdmittanceController::setInToolCoor(bool is){
    is_in_tool_coor_=is;
}
void AdmittanceController::getExtForce(){
    vector<double> vecReadForce=pMyForceSensor->getChangeData();
    vector<double> vecExtForce(6,0);
    for(int i=0;i<6;++i){
        vecExtForce[i]=vecReadForce[i];
    }
    wrench_external_=Vector6d(vecExtForce.data());
    wrench_external_in_world_=wrench_external_;
}
void AdmittanceController::setExtForce(vector<double> vecForce){
    wrench_external_=Vector6d(vecForce.data());
}
void AdmittanceController::compute_ee(){
    Vector6d pos_now = getRobotTCPPos();
    //ee_position_ = Vector6d(MyMath::relative_pose_deviation(MyMath::Vector6d_to_stdvector(pos_now),MyMath::Vector6d_to_stdvector(aim_position_)).data());
    ee_position_=pos_now-aim_position_;
    ee_position_(3)=should_position_(3);
    ee_position_(4)=should_position_(4);
    ee_position_(5)=should_position_(5);
}
Vector6d AdmittanceController::getRobotTCPPos(){
    vector<double> pos=pMyRobotControl->getTCPPos();
    return Vector6d(pos.data());
}
void AdmittanceController::compute_force_in_world_coor(){
    Vector6d pos_now=getRobotTCPPos();
    pos_now(0)=0;pos_now(1)=0;pos_now(2)=0;
    vector<double> fxfyfz{wrench_external_(0),wrench_external_(1),wrench_external_(2),0,0,0};
    vector<double> mxmymz{wrench_external_(3),wrench_external_(4),wrench_external_(5),0,0,0};
    QTime startTime = QTime::currentTime();
    fxfyfz = MyMath::trans_pose(MyMath::Vector6d_to_stdvector(pos_now),fxfyfz);
    mxmymz = MyMath::trans_pose(MyMath::Vector6d_to_stdvector(pos_now),mxmymz);
    QTime stopTime = QTime::currentTime();
    int elapsed = startTime.msecsTo(stopTime);
    ostringstream oss;//创建一个流
    oss<<" compute time: "<<elapsed<<endl;
    cout<<oss.str();
    wrench_external_in_world_(0)=fxfyfz[0];
    wrench_external_in_world_(1)=fxfyfz[1];
    wrench_external_in_world_(2)=fxfyfz[2];
    wrench_external_in_world_(3)=mxmymz[0];
    wrench_external_in_world_(4)=mxmymz[1];
    wrench_external_in_world_(5)=mxmymz[2];
}
void AdmittanceController::changeAimPosIncrement(Vector6d vec){
    vector<double> vecRot{vec(3),vec(4),vec(5),0,0,0};
    Vector6d vecRotAdd=vec;
    if(is_in_tool_coor_){
        Vector6d pos_now=getRobotTCPPos();
        pos_now(0)=0;pos_now(1)=0;pos_now(2)=0;
        vec=Vector6d(MyMath::trans_pose(MyMath::Vector6d_to_stdvector(pos_now),MyMath::Vector6d_to_stdvector(vec)).data());
        vecRotAdd=Vector6d(MyMath::trans_pose(MyMath::Vector6d_to_stdvector(pos_now),vecRot).data());
        vecRotAdd(3)=vecRotAdd(0);vecRotAdd(4)=vecRotAdd(1);vecRotAdd(5)=vecRotAdd(2);
    }
    aim_position_+=vec;
    should_position_+=vecRotAdd;
}
void AdmittanceController::stopControll(){
    return;
}
bool AdmittanceController::isCanControl(){
    if(!pMyRobotControl||!pMyForceSensor)
        return false;
    return true;
}
AdmittanceController::~AdmittanceController(){
    if(!pMyRobotControl)
        return;
    pMyRobotControl->stopRobot();
}
