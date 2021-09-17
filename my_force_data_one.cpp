#include "my_force_data_one.h"
using namespace std;
MyForceDataOne::MyForceDataOne(int channel,int cap ): MyForceData (channel,cap){
}
void MyForceDataOne::setChart(QChartView *pChartView){
    my_pChartView = pChartView;
    my_pChart->createDefaultAxes();
    if(pChartView != nullptr)
        pChartView->setChart(my_pChart);
    QValueAxis *axisX = new QValueAxis;
    axisX->setRange(0,my_cap);
    axisX->setLabelFormat("%g");
    axisX->setTitleText("data frame");
    for(int i=0;i<my_channel;++i)
    {
        (*my_pVecSeries)[i]->setName("channel "+QString::number(i));
        my_pChart->addSeries((*my_pVecSeries)[i]);
        my_pChart->setAxisX(axisX,(*my_pVecSeries)[i]);
        (*my_pVecSeries)[i]->setUseOpenGL(true);
    }
    setAxisY(my_isShowForce);
    my_pChart->setTitle("force sensor one data");
}
void MyForceDataOne::setAxisY(bool flag){
    my_isShowForce = flag;
    QValueAxis *axisY = new QValueAxis;
    if(!flag){
        axisY->setRange(0,4000);
        axisY->setTitleText("Voltage(mV)");
    }
    else {
        axisY->setRange(-50,50);
        axisY->setTitleText("Force(N/N*cm)");
    }
    for(int i=0;i<my_channel;++i)
    {
        my_pChart->setAxisY(axisY,(*my_pVecSeries)[i]);

    }
}
