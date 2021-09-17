#ifndef MY_FORCE_DATA_ONE_H
#define MY_FORCE_DATA_ONE_H
#include "my_force_data.h"
class MyForceDataOne : public MyForceData {
public:
    MyForceDataOne(int channel,int cap );
    void setChart(QChartView *pChartView);
    void setAxisY(bool flag);
    void setAxisX();
};
#endif // MY_FORCE_DATA_ONE_H
