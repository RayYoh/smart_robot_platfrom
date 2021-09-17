#include "teach_repeat.h"

TeachRepeat::TeachRepeat()
{
    loop_rate_=20;
    duration_sec_=1/loop_rate_;
}
void TeachRepeat::run(){
    if(flag){
        trajectory_buf_.clear();
        while(isCanControl()&&flag==true)
        {
            trajectory_buf_.push_back(pMyRobotControl->getTCPPos());
            QThread::msleep(1000.0/loop_rate_);
            cout<<"teach"<<endl;
        }
    }
    else{
        if(trajectory_buf_.empty())
            return;
        std::list<std::vector<double>> temp=trajectory_buf_;
        pMyRobotControl->posControl(temp.front());

        temp.pop_front();
        while(isCanControl()&&flag==false&&temp.size())
        {
            pMyRobotControl->servoControl(temp.front(),duration_sec_);
            temp.pop_front();
            cout<<"repeat"<<endl;
            QThread::msleep(1000.0/loop_rate_);
        }
        pMyRobotControl->servoStop();
    }
}
void TeachRepeat::init_param(MyRobotControl *pRobot, double frequency){
    pMyRobotControl=pRobot;
    loop_rate_=frequency;
    duration_sec_=1/frequency;
}
void TeachRepeat::showMessage(QString text){
    QMessageBox::information(nullptr,"admittance control information",text,QMessageBox::Ok);
}
bool TeachRepeat::isCanControl(){
    if(!pMyRobotControl||!pMyRobotControl->checkRobotConnect())
        return false;
    return true;
}
