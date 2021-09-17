#include "my_force_data.h"
MyForceData::MyForceData(int channel,int cap):
    my_channel(channel),
    my_cap(cap),
    my_pChartView(nullptr),
    my_pChart(new QChart),
    my_pVecSeries(new vector<QLineSeries*>(channel)),
    my_isShowForce(false)
{
    for(int i=0;i<my_channel;++i)
    {
        (*my_pVecSeries)[i] = new QLineSeries;
    }
    for(int i=0;i<my_cap;++i)
    {
        for(int j=0;j<my_channel;++j)
        {
            (*my_pVecSeries)[j]->append(i,0);
        }
    }
}
MyForceData::~MyForceData(){
    delete my_pChartView;
    delete my_pChart;
    for(int i=0;i<my_channel;++i)
    {
        delete (*my_pVecSeries)[i];
    }
    delete my_pVecSeries;
}

bool MyForceData::addData(const vector<double> &vData){
    if(vData.size()!=my_channel)
        return false;
    for(int i=0;i<my_channel;++i)
    {
        auto p_series = (*my_pVecSeries)[i];
        QVector<QPointF> oldPoints = p_series->pointsVector();
        QVector<QPointF> points;
        for(int j=1;j<my_cap;++j)
        {
            points.append(QPointF(j-1 ,oldPoints.at(j).y()));
        }
        points.append(QPointF(my_cap-1,vData[i]));;
        p_series->replace(points);
    }
    return true;
}
void MyForceData::updateChart(vector<bool> vB){
    if(vB.size()!=my_channel)
        return;
    for(int i=0;i<my_channel;++i)
    {
        my_pChart->addSeries((*my_pVecSeries)[i]);
    }
    for(int i=0;i<my_channel;++i)
    {
        if(!vB[i])
            my_pChart->removeSeries((*my_pVecSeries)[i]);
    }
    return;
}
void MyForceData::clearData(){
    for(int i=0;i<my_cap;++i)
    {
        for(int j=0;j<my_channel;++j)
        {
            (*my_pVecSeries)[j]->replace(i,i,0);
        }
    }
}
