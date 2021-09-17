#ifndef TEACH_REPEAT_H
#define TEACH_REPEAT_H
#include <QThread>
#include <QMessageBox>
#include <vector>
#include <list>
#include "my_robot_control.h"
#include "my_math.h"
#include <QTime>
class TeachRepeat : public QThread
{
public:
    TeachRepeat();
    void run();
    void init_param(MyRobotControl *pRobot, double frequency);
    bool flag;//true->teach  false->repeat
private:
    std::list<std::vector<double>> trajectory_buf_;
    MyRobotControl *pMyRobotControl;
    double loop_rate_;
    double duration_sec_;
    void showMessage(QString text);
    bool isCanControl();
};

#endif // TEACH_REPEAT_H
