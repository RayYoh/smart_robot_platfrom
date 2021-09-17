#include "my_force_data_two.h"
using namespace std;
MyForceDataTwo::MyForceDataTwo(int channel,int cap ): MyForceData (channel,cap){
}
void MyForceDataTwo::setChart(QChartView *pChartView){
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
    my_pChart->setTitle("force sensor two data");
}
void MyForceDataTwo::setAxisY(bool flag){
    my_isShowForce = flag;
    QValueAxis *axisY = new QValueAxis;
    if(!flag){
        axisY->setRange(0,4096);
        axisY->setTitleText("Digital");
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
