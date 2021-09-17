#ifndef MY_FORCE_SENSOR_TWO_H
#define MY_FORCE_SENSOR_TWO_H
#include "my_force_sensor.hpp"
class MyForceSensorTwo : public MyForceSensor
{
public:
    MyForceSensorTwo(quint16 port);
    void handleData(char *str);
    void setZero();
    void changeData();
    vector<double> getForceDdata();
    vector<double> getChangeData();
private:
    vector<double> my_force_data;
    vector<double> my_force_zero;
    vector<double> my_change_data;
    void onSocketReadyRead();
};

#endif // MY_FORCE_SENSOR_TWO_H
