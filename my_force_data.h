#ifndef MY_FORCE_DATA_H
#define MY_FORCE_DATA_H
#include <vector>
#include <QtCharts>
using namespace std;
using namespace QtCharts;
class MyForceData{
public:
    MyForceData(int channel,int cap);
    ~MyForceData();
    bool addData(const vector<double> &vData);
    void updateChart(vector<bool> vB);
    void clearData();
    void setChart(QChartView *pChartView);

    int my_channel;
    int my_cap;
    QChartView *my_pChartView;
    QChart *my_pChart;
    vector<QLineSeries*> *my_pVecSeries;
    bool my_isShowForce;
private:
};
#endif // MY_FORCE_DATA_H
