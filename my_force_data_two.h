#ifndef MY_FORCE_DATA_TWO_H
#define MY_FORCE_DATA_TWO_H
#include "my_force_data.h"
class MyForceDataTwo : public MyForceData {
public:
    MyForceDataTwo(int channel,int cap );
    void setChart(QChartView *pChartView);
    void setAxisY(bool flag);
    void setAxisX();
};
#endif // MY_FORCE_DATA_TWO_H
