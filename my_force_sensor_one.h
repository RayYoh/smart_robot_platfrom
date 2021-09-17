#ifndef MY_FORCE_SENSOR_ONE_H
#define MY_FORCE_SENSOR_ONE_H
#include "my_force_sensor.hpp"
#include <deque>
#define		SENSOR_COUNT			1
class MyForceSensorOne : public MyForceSensor{
    Q_OBJECT
public:
    MyForceSensorOne(quint16 port);
    void handleData(char *str);
    void setZero();
    void changeData();
    vector<double> getForceDdata();
    vector<double> getChangeData();
private:
    vector<double> my_force_data;
    vector<double> my_force_zero;
    vector<double> my_change_data;
    void onNewConnection();
    void run();
};
#endif // MY_FORCE_SENSOR_ONE_H
