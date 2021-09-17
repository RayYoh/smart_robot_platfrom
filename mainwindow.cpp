#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QtCharts>
#include <iostream>
#include <arpa/inet.h>
using namespace std;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    my_force_sensor_one(6800),
    my_force_sensor_two(6000),
    pAdmitControl(new AdmittanceController),
    pDragControl(new DragController),
    pEnv(new RobotAssembleEnv),
    pTeachRepeat(new TeachRepeat),
    pRobotLoop(new RobotLoop),
    pRobotMove(new RobotMove),
    vForceDataOne(std::vector<double>(6,0)),
    vForceDataTwo(std::vector<double>(12,0)),
    my_force_data_one(6,300),
    my_force_data_two(12,300),
    pTimerTime(new QTimer(this)),
    pTimerRobotState(new QTimer(this)),
    pTimerPaint(new QTimer(this)),
    pTimerSave(new QTimer(this)),
    currentTimeLabel(new QLabel),
    currentRobotStatusLabel(new QLabel)
{
    ui->setupUi(this);
    if(!my_force_sensor_one.init())
        QMessageBox::warning(this,"connect erro","sensor server 1 create listen socket failed",QMessageBox::Ok);
    if(!my_force_sensor_two.init())
        QMessageBox::warning(this,"connect erro","sensor server 2 create listen socket failed",QMessageBox::Ok);
    connect(&my_force_sensor_one,SIGNAL(sensorConnect()),this,SLOT(onSensorConnectOne()));
    connect(&my_force_sensor_one,SIGNAL(sensorHaveData()),this,SLOT(onSensorHaveDataOne()));
    connect(&my_force_sensor_one,SIGNAL(sensorDisConnect()),this,SLOT(onSensorDisConnectOne()));
    connect(&my_force_sensor_two,SIGNAL(sensorConnect()),this,SLOT(onSensorConnectTwo()));
    connect(&my_force_sensor_two,SIGNAL(sensorHaveData()),this,SLOT(onSensorHaveDataTwo()));
    connect(&my_force_sensor_two,SIGNAL(sensorDisConnect()),this,SLOT(onSensorDisConnectTwo()));
    //createChart();
    ui->lineEdit_robotIp->setText("192.168.1.11");
    QDoubleValidator *pDouVal = new QDoubleValidator;
    pDouVal->setRange(-500,500,3);
    ui->lineEdit_axle0->setValidator(pDouVal);
    ui->lineEdit_axle1->setValidator(pDouVal);
    ui->lineEdit_axle2->setValidator(pDouVal);
    ui->lineEdit_axle3->setValidator(pDouVal);
    ui->lineEdit_axle4->setValidator(pDouVal);
    ui->lineEdit_axle5->setValidator(pDouVal);
    my_force_data_one.setChart(ui->graphicsView_one);
    my_force_data_two.setChart(ui->graphicsView_two);
    ui->statusBar->addWidget(currentTimeLabel);
    ui->statusBar->addWidget(currentRobotStatusLabel);
    pTimerTime->start(1000);
    connect(pTimerTime, SIGNAL(timeout()),this,SLOT(onTimerTimeTimeout()));
    connect(pTimerRobotState, SIGNAL(timeout()),this,SLOT(onTimerRobotStatusTimeout()));

    pTimerPaint->start(100);
    connect(pTimerPaint, SIGNAL(timeout()),this,SLOT(onTimerPaintTimeout()));
    connect(pTimerSave, SIGNAL(timeout()),this,SLOT(onTimerSaveTimeout()));

    refreshDataOne();
    refreshWidget();
}
void MainWindow::onSensorConnectOne(){
    QMessageBox::information(this,"connect information","sensor 1 on-line",QMessageBox::Ok);
    refreshWidget();
}
void MainWindow::onSensorDisConnectOne(){
    QMessageBox::information(this,"connect information","sensor 1 off-line",QMessageBox::Ok);
    refreshWidget();
}
void MainWindow::onSensorHaveDataOne(){
    if(my_force_sensor_one.isNeedChange)
        vForceDataOne=my_force_sensor_one.getChangeData();
    else
        vForceDataOne=my_force_sensor_one.getForceDdata();
    refreshDataOne();
    //my_force_data_one.addData(vForceDataOne);
}
void MainWindow::refreshDataOne(){
    ui->lineEdit_fx->setText(QString("%1").arg(vForceDataOne[0]));
    ui->lineEdit_fy->setText(QString("%1").arg(vForceDataOne[1]));
    ui->lineEdit_fz->setText(QString("%1").arg(vForceDataOne[2]));
    ui->lineEdit_mx->setText(QString("%1").arg(vForceDataOne[3]));
    ui->lineEdit_my->setText(QString("%1").arg(vForceDataOne[4]));
    ui->lineEdit_mz->setText(QString("%1").arg(vForceDataOne[5]));
}
void MainWindow::onSensorConnectTwo(){
    QMessageBox::information(this,"connect information","sensor 2 on-line",QMessageBox::Ok);
    refreshWidget();
}
void MainWindow::onSensorDisConnectTwo(){
    QMessageBox::information(this,"connect information","sensor 2 off-line",QMessageBox::Ok);
    refreshWidget();
}
void MainWindow::onSensorHaveDataTwo(){
    if(my_force_sensor_two.isNeedChange)
        vForceDataTwo=my_force_sensor_two.getChangeData();
    else
        vForceDataTwo=my_force_sensor_two.getForceDdata();
    refreshDataTwo();
    //my_force_data_two.addData(vForceDataTwo);
}
void MainWindow::refreshDataTwo(){
    QString str="";
    if(!my_force_sensor_two.isNeedChange)
    {
        str+="digital data:{";
    }
    else {
        str+="force data:{";
    }
    for (int i=0;i<12;++i) {
        if(!my_force_sensor_two.isNeedChange){
            str+=QString::number((int)vForceDataTwo[i]).sprintf("%04d",(int)vForceDataTwo[i]);
            str+=",";
        }
        else {
            str+=QString::number(vForceDataTwo[i],10,2);
            str+=",";
        }
    }
    str+="}";
    ui->lineEdit_sensor_data->setText(str);
}
void MainWindow::on_button_startCollect_clicked(){
    if(ui->button_startCollect->text()=="start collect")
    {
        unsigned short cmd = htons(0x2200);
        QByteArray str((char *)&cmd,2);
        my_force_sensor_one.sendMsg(str);
        ui->button_startCollect->setText("stop collect");
    }
    else {
        unsigned short cmd = htons(0x2400);
        QByteArray str((char *)&cmd,2);
        my_force_sensor_one.sendMsg(str);
        ui->button_startCollect->setText("start collect");
    }
}

void MainWindow::on_checkbox_show_fx_stateChanged(int state){
    refreshChartOne();
}
void MainWindow::on_checkbox_show_fy_stateChanged(int state){
    refreshChartOne();
}
void MainWindow::on_checkbox_show_fz_stateChanged(int state){
    refreshChartOne();
}
void MainWindow::on_checkbox_show_mx_stateChanged(int state){
    refreshChartOne();
}
void MainWindow::on_checkbox_show_my_stateChanged(int state){
    refreshChartOne();
}
void MainWindow::on_checkbox_show_mz_stateChanged(int state){
    refreshChartOne();
}

void MainWindow::on_checkbox_ch0_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch1_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch2_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch3_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch4_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch5_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch6_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch7_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch8_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch9_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch10_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_ch11_stateChanged(int state){
    refreshChartTwo();
}
void MainWindow::on_checkbox_toolSpeed_stateChanged(int state){
    if(ui->checkbox_toolSpeed->isChecked()){
        ui->checkbox_incremental->setEnabled(false);
        ui->checkbox_tcpPos->setEnabled(false);
    }
    else {
        ui->checkbox_incremental->setEnabled(true);
        ui->checkbox_tcpPos->setEnabled(true);
    }
}
void MainWindow::refreshChartOne(){
    std::vector<bool> vBOne(6,false);
    vBOne[0]=ui->checkbox_show_fx->isChecked();
    vBOne[1]=ui->checkbox_show_fy->isChecked();
    vBOne[2]=ui->checkbox_show_fz->isChecked();
    vBOne[3]=ui->checkbox_show_mx->isChecked();
    vBOne[4]=ui->checkbox_show_my->isChecked();
    vBOne[5]=ui->checkbox_show_mz->isChecked();
    my_force_data_one.setChart(nullptr);
    my_force_data_one.updateChart(vBOne);
}
void MainWindow::refreshChartTwo(){
    std::vector<bool> vBOne(12,false);
    vBOne[0]=ui->checkbox_ch0->isChecked();
    vBOne[1]=ui->checkbox_ch1->isChecked();
    vBOne[2]=ui->checkbox_ch2->isChecked();
    vBOne[3]=ui->checkbox_ch3->isChecked();
    vBOne[4]=ui->checkbox_ch4->isChecked();
    vBOne[5]=ui->checkbox_ch5->isChecked();
    vBOne[6]=ui->checkbox_ch6->isChecked();
    vBOne[7]=ui->checkbox_ch7->isChecked();
    vBOne[8]=ui->checkbox_ch8->isChecked();
    vBOne[9]=ui->checkbox_ch9->isChecked();
    vBOne[10]=ui->checkbox_ch10->isChecked();
    vBOne[11]=ui->checkbox_ch11->isChecked();
    my_force_data_two.setChart(nullptr);
    my_force_data_two.updateChart(vBOne);
}
void MainWindow::refreshWidget(){
    ui->button_startCollect->setEnabled(my_force_sensor_one.isCanSend||my_force_sensor_two.isCanSend);
    ui->button_setZero->setEnabled(my_force_sensor_one.isCanSet||my_force_sensor_two.isCanSet);
//    ui->button_setPath->setEnabled(my_force_sensor_one.isCanSet||my_force_sensor_two.isCanSet);
//    ui->button_saveData->setEnabled(my_force_sensor_one.isCanSet||my_force_sensor_two.isCanSet);
    //ui->button_loop->setEnabled(my_force_sensor_one.isCanSet||my_force_sensor_two.isCanSet);
    ui->button_changeForce->setEnabled(my_force_sensor_one.isCanChange||my_force_sensor_two.isCanChange);

}
void MainWindow::on_button_setZero_clicked(){
    my_force_sensor_one.setZero();
    my_force_sensor_two.setZero();
    refreshWidget();
}
void MainWindow::on_button_changeForce_clicked(){
    if(ui->button_changeForce->text()=="change to force"){
        ui->button_changeForce->setText("change to origin");
        my_force_sensor_one.changeForce(true);
        my_force_data_one.clearData();
        my_force_data_one.setAxisY(true);

        my_force_sensor_two.changeForce(true);
        my_force_data_two.clearData();
        my_force_data_two.setAxisY(true);
    }
    else{
        ui->button_changeForce->setText("change to force");
        my_force_sensor_one.changeForce(false);
        my_force_data_one.clearData();
        my_force_data_one.setAxisY(false);

        my_force_sensor_two.changeForce(false);
        my_force_data_two.clearData();
        my_force_data_two.setAxisY(false);
    }
}
void MainWindow::on_button_setPath_clicked(){
    if (ui->button_setPath->text()=="SetPath"){
        filePath = QFileDialog::getExistingDirectory(this,
                                                     tr("Please set save path"),
                                                     "all");
            if(filePath.isEmpty())
            {
                return;
            }
            else{
                ui->lineEdit_path->setText(filePath);
            }
    }
}
void MainWindow::on_button_saveData_clicked(){
    if(ui->button_saveData->text()=="SaveData"){
        QDateTime current_time = QDateTime::currentDateTime();
        QString timestr = current_time.toString( "yyyy年MM月dd日 hh时mm分ss秒"); //设置显示的格式
    //        QString fileName = QFileDialog::getSaveFileName(this,
    //                                                            tr("Save Force Data"),
    //                                                            "",
    //                                                            tr("txt(*.txt)"));
        QString fileName=filePath+"/"+timestr+".txt";
        txt.setFileName(fileName);
        txt.open(QIODevice::WriteOnly | QIODevice::Text);
        pTimerSave->start(40);
        ui->button_saveData->setText("StopSave");
    }
    else{
        pTimerSave->stop();
        txt.close();
        d_num = 0;
        ui->button_saveData->setText("SaveData");
    }
}
void MainWindow::on_button_caluG_clicked(){
    if(ui->button_caluG->text()=="CaluG"){
        pRobotMove->init_param(&my_force_sensor_one,&my_force_sensor_two,&my_robot,filePath);
        pRobotMove->flag=true;
        pRobotMove->start();
        ui->button_loop->setText("Stop");
    }
    else{
        pRobotMove->flag = false;
        my_robot.stopRobot();
        ui->button_caluG->setText("CaluG");
    }
}
void MainWindow::on_button_loop_clicked(){
//  joint3 -130---+20
//  joint4 +60---+120
//  joint5 +90--- -90
    if(ui->button_loop->text()=="Loop"){
        pRobotLoop->init_param(&my_force_sensor_one,&my_force_sensor_two,&my_robot,filePath);
        pRobotLoop->flag=true;
        pRobotLoop->start();
        ui->button_loop->setText("StopLoop");
    }
    else{
        pRobotLoop->flag = false;
        my_robot.stopRobot();
        ui->button_loop->setText("Loop");
    }
}
void MainWindow::on_button_robotConn_clicked(){
    ui->button_robotConn->setEnabled(false);
    if(ui->button_robotConn->text()=="connect" && my_robot.connect(ui->lineEdit_robotIp->text().toStdString()))
    {
        pTimerRobotState->start(40);
        ui->button_robotConn->setText("disconnect");
    }
    else if(ui->button_robotConn->text()=="disconnect" && my_robot.disconnect())
    {
        pTimerRobotState->stop();
        ui->button_robotConn->setText("connect");
    }
    ui->button_robotConn->setEnabled(true);
}
std::vector<double> MainWindow::getOffset(){
    std::vector<double> offSet(6,0);
    offSet[0]=ui->lineEdit_axle0->text().toDouble();
    offSet[1]=ui->lineEdit_axle1->text().toDouble();
    offSet[2]=ui->lineEdit_axle2->text().toDouble();
    offSet[3]=ui->lineEdit_axle3->text().toDouble();
    offSet[4]=ui->lineEdit_axle4->text().toDouble();
    offSet[5]=ui->lineEdit_axle5->text().toDouble();
    return offSet;
}
void MainWindow::on_button_robotMove_clicked(){
    std::vector<double> offSet(6,0);
    offSet=getOffset();
    bool isSpd,isInc,isTcp;
    isSpd=ui->checkbox_toolSpeed->isChecked();
    isInc=ui->checkbox_incremental->isChecked();
    isTcp=ui->checkbox_tcpPos->isChecked();
    my_robot.moveRobot(offSet,1.5,isSpd,isInc,isTcp);
}
void MainWindow::on_button_teach_clicked(){
    if(ui->button_teach->text()=="start teach"){
        if(!my_robot.startTeachMode())
            return;
        ui->button_teach->setText("end teach");
    }
    else{
        if(!my_robot.endTeachMode())
            return;
        ui->button_teach->setText("start teach");
    }
}
void MainWindow::on_button_stop_clicked(){
    my_robot.stopRobot();
}
void MainWindow::onTimerTimeTimeout(){
    QDateTime current_time = QDateTime::currentDateTime();
    QString timestr = current_time.toString( "yyyy年MM月dd日 hh:mm:ss"); //设置显示的格式
    currentTimeLabel->setText(timestr); //设置label的文本内容为时间
}
void MainWindow::onTimerRobotStatusTimeout(){
    std::vector<double> vecTCPRes=my_robot.getTCPPos();
    for(int i=0;i<3;++i){
        vecTCPRes[i]=vecTCPRes[i]*1000;
    }
    std::vector<double> vecJointRes=my_robot.getJointPos();
//    for(int i=0;i<6;++i){
//        vecJointRes[i]=vecJointRes[i]/3.1415926535898*180;
//    }
    if(vecTCPRes.size()<6||vecJointRes.size()<6)
    {
        currentRobotStatusLabel->setText("no robot date");
        return;
    }
    QString text;
    text.sprintf("robot pos:{%4.2lf,%4.2lf,%4.2lf,%4.2lf,%4.2lf,%4.2lf} robot TCP:{%4.2lf,%4.2lf,%4.2lf,%4.2lf,%4.2lf,%4.2lf}",
                 vecJointRes[0],vecJointRes[1],vecJointRes[2],vecJointRes[3],vecJointRes[4],vecJointRes[5],
            vecTCPRes[0],vecTCPRes[1],vecTCPRes[2],vecTCPRes[3],vecTCPRes[4],vecTCPRes[5]);
    text+="  robot status:" + QString::number(my_robot.getRobotStatus());
    currentRobotStatusLabel->setText(text);
}
void MainWindow::onTimerPaintTimeout(){
    my_force_data_one.addData(vForceDataOne);
    my_force_data_two.addData(vForceDataTwo);
}
void MainWindow::onTimerSaveTimeout(){
    d_num++;
    QString toSave = "No."+QString::number(d_num);
    std::vector<double> vTraction = my_force_sensor_two.getForceDdata();
    std::vector<double> vContact = my_force_sensor_one.getForceDdata();
    std::vector<double> fTraction(12,0) ;
    std::vector<double> fContact(6,0);
    if(my_force_sensor_one.isNeedChange||my_force_sensor_two.isNeedChange){
        fTraction = my_force_sensor_two.getChangeData();
        fContact = my_force_sensor_one.getChangeData();
    }
    toSave+=" Traction sensor V: ";
    for(int i = 0;i < 11;++i){
        toSave += QString("%1,").arg(vTraction[i]);
    }
    toSave+=QString("%1").arg(vTraction[11]);
    toSave+=" Traction sensor F/M: ";
    for(int i = 0;i < 11;++i){
        toSave += QString("%1,").arg(fTraction[i]);
    }
    toSave+=QString("%1").arg(fTraction[11]);
    toSave+=" Contact sensor V: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vContact[i]);
    }
    toSave+=QString("%1").arg(vContact[5]);
    toSave+=" Contact sensor F/M: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(fContact[i]);
    }
    toSave+=QString("%1").arg(fContact[5]);
    std::vector<double> vecTCPPos(6,0);
    std::vector<double> vecJointPos(6,0);
    std::vector<double> vecTCPTor(6,0);
    std::vector<double> vecJointTor(6,0);
//    if(my_robot.checkRobotConnect()){
    vecTCPPos = my_robot.getTCPPos();
    vecJointPos = my_robot.getJointPos();
    vecTCPTor = my_robot.getTCPForce();
    vecJointTor = my_robot.getRobotJointTorques();
//    }
    toSave+=" Robot TCP position: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecTCPPos[i]);
    }
    toSave+=QString("%1").arg(vecTCPPos[5]);
    toSave+=" Robot Joint position: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecJointPos[i]);
    }
    toSave+=QString("%1").arg(vecJointPos[5]);
    toSave+=" Robot TCP Torque: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecTCPTor[i]);
    }
    toSave+=QString("%1").arg(vecTCPTor[5]);
    toSave+=" Robot Joint Torque: ";
    for(int i = 0;i < 5;++i){
        toSave+=QString("%1,").arg(vecJointTor[i]);
    }
    toSave+=QString("%1").arg(vecJointTor[5]);
    txt.write(toSave.toUtf8()+"\n");
}
void MainWindow::on_button_setAdmitControlParam_clicked(){
    std::vector<double> M_a=getMass();
    std::vector<double> D_a=getDamp();
    std::vector<double> K_a=getK();
    std::vector<double> force=getForceControlAim();
    double control_freq=getControlFreq();
    double arm_max_vel=getMaxVel();
    double arm_max_acc=getMaxAcc();
    std::vector<double> workspace_limits{100,100,100,100,100,100};
    std::vector<bool> vec_is_can_move=getAdmitAxis();
    pAdmitControl->setInToolCoor(ui->checkbox_coorTrans->isChecked());
    MyForceSensor *pNeedSensor;
    if(ui->checkbox_use_traSensor->isChecked())
        pNeedSensor=&my_force_sensor_two;
    else {
        pNeedSensor=&my_force_sensor_one;
    }
    pAdmitControl->init_param(pNeedSensor,&my_robot,control_freq,M_a,D_a,K_a,force,workspace_limits,arm_max_vel,arm_max_acc,vec_is_can_move);
}
void MainWindow::on_button_setDragControlParam_clicked(){
    std::vector<double> K_a=getDragControlK();
    std::vector<double> force=getForceControlAim();
    double control_freq=getControlFreq();
    double arm_max_vel=getMaxVel();
    double speed_rate=getSpeedRate();
    for(int i=0;i<K_a.size();++i)
        K_a[i]=K_a[i]*speed_rate;
    pAdmitControl->setInToolCoor(ui->checkbox_coorTrans->isChecked());
    MyForceSensor *pNeedSensor;
    if(ui->checkbox_use_traSensor->isChecked())
        pNeedSensor=&my_force_sensor_two;
    else {
        pNeedSensor=&my_force_sensor_one;
    }
    pDragControl->init_param(pNeedSensor,&my_robot,control_freq,K_a,force,arm_max_vel);
}
void MainWindow::on_button_startAdmitControl_clicked(){
    if(ui->button_startAdmitControl->text()=="start admittance control"){
        pAdmitControl->flag=true;
        pAdmitControl->start();
        ui->button_startAdmitControl->setText("end admittance control");
    }
    else{
        pAdmitControl->flag=false;
        ui->button_startAdmitControl->setText("start admittance control");
    }
}
void MainWindow::on_button_startDragControl_clicked(){
    if(ui->button_startDragControl->text()=="start drag control"){
        pDragControl->flag=true;
        pDragControl->start();
        ui->button_startDragControl->setText("end drag control");
    }
    else{
        pDragControl->flag=false;
        ui->button_startDragControl->setText("start drag control");
    }
}
std::vector<double> MainWindow::getDamp(){
    std::vector<double> vecRes(36,0);
    vecRes[0]=ui->doubleSpinBox_damp_0->value();
    vecRes[7]=ui->doubleSpinBox_damp_1->value();
    vecRes[14]=ui->doubleSpinBox_damp_2->value();
    vecRes[21]=ui->doubleSpinBox_damp_3->value();
    vecRes[28]=ui->doubleSpinBox_damp_4->value();
    vecRes[35]=ui->doubleSpinBox_damp_5->value();
    return vecRes;
}
std::vector<double> MainWindow::getMass(){
    std::vector<double> vecRes(36,0);
    vecRes[0]=ui->doubleSpinBox_mass_0->value();
    vecRes[7]=ui->doubleSpinBox_mass_1->value();
    vecRes[14]=ui->doubleSpinBox_mass_2->value();
    vecRes[21]=ui->doubleSpinBox_mass_3->value();
    vecRes[28]=ui->doubleSpinBox_mass_4->value();
    vecRes[35]=ui->doubleSpinBox_mass_5->value();
    return vecRes;
}
std::vector<double> MainWindow::getK(){
    std::vector<double> vecRes(36,0);
    vecRes[0]=ui->doubleSpinBox_k_0->value();
    vecRes[7]=ui->doubleSpinBox_k_1->value();
    vecRes[14]=ui->doubleSpinBox_k_2->value();
    vecRes[21]=ui->doubleSpinBox_k_3->value();
    vecRes[28]=ui->doubleSpinBox_k_4->value();
    vecRes[35]=ui->doubleSpinBox_k_5->value();
    return vecRes;
}
std::vector<double> MainWindow::getDragControlK(){
    std::vector<double> vecRes(36,0);
    vecRes[0]=ui->doubleSpinBox_k_xyz->value();
    vecRes[7]=ui->doubleSpinBox_k_xyz->value();
    vecRes[14]=ui->doubleSpinBox_k_xyz->value();
    vecRes[21]=ui->doubleSpinBox_k_rot->value();
    vecRes[28]=ui->doubleSpinBox_k_rot->value();
    vecRes[35]=ui->doubleSpinBox_k_rot->value();
    return vecRes;
}
std::vector<double> MainWindow::getForceControlAim(){
    std::vector<double> vecRes(6,0);
    vecRes[0]=ui->doubleSpinBox_fxControl->value();
    vecRes[1]=ui->doubleSpinBox_fyControl->value();
    vecRes[2]=ui->doubleSpinBox_fzControl->value();
    vecRes[3]=ui->doubleSpinBox_mxControl->value();
    vecRes[4]=ui->doubleSpinBox_myControl->value();
    vecRes[5]=ui->doubleSpinBox_mzControl->value();
    return vecRes;
}
std::vector<bool> MainWindow::getAdmitAxis(){
    std::vector<bool> vecRes(6,0);
    vecRes[0]=ui->checkbox_enable_fx->isChecked();
    vecRes[1]=ui->checkbox_enable_fy->isChecked();
    vecRes[2]=ui->checkbox_enable_fz->isChecked();
    vecRes[3]=ui->checkbox_enable_mx->isChecked();
    vecRes[4]=ui->checkbox_enable_my->isChecked();
    vecRes[5]=ui->checkbox_enable_mz->isChecked();
    return vecRes;
}
double MainWindow::getSpeedRate(){
    return ui->doubleSpinBox_speed_rate->value()/100;
}
double MainWindow::getMaxAcc(){
    return ui->doubleSpinBox_max_acc->value();
}
double MainWindow::getMaxVel(){
    return ui->doubleSpinBox_max_vel->value();
}
double MainWindow::getControlFreq(){
    return ui->doubleSpinBox_control_freq->value();
}
void MainWindow::showMessage(QString text){
    QMessageBox::information(nullptr,"MainWindow information",text,QMessageBox::Ok);
}
void MainWindow::on_button_stepTime_clicked(){
    my_robot.getRobotControlStepTime();
    return;
}
void MainWindow::on_button_jointTorques_clicked(){
    std::vector<double> vecRes = my_robot.getRobotJointTorques();
    ostringstream oss;//创建一个流
    for(int i=0;i<6;++i){
    oss<<" torques "<<i<<" : "<<vecRes[i];
    }
    showMessage(QString::fromStdString(oss.str()));
    return;
}
void MainWindow::on_button_tcpForce_clicked(){
    std::vector<double> vecRes = my_robot.getTCPForce();
    ostringstream oss;//创建一个流
    for(int i=0;i<6;++i){
    oss<<" torques "<<i<<" : "<<vecRes[i];
    }
    showMessage(QString::fromStdString(oss.str()));
    return;
}
void MainWindow::on_button_moveInToolCoordinate_clicked(){
    std::vector<double> offset = getOffset();
    std::vector<double> base = my_robot.getTCPPos();
    my_robot.poseTrans(base,offset);
}
void MainWindow::on_button_coorTrans_clicked(){
    std::vector<double> vecBase=my_robot.getTCPPos();
    std::vector<double> vecOne{-174.91,-601.88,30.91,0.005,2.818,0.778};
    std::vector<double> vecTwo{-174.91,-601.88,30.91,0.520,2.853,0.847};
    //std::vector<double> vecTwo{-200.8,-624.65,323.18,0.009,3.005,0.828};
    std::vector<double> vecRes = MyMath::relative_pose_deviation(vecOne,vecTwo);
    ostringstream oss;//创建一个流
    for(int i=0;i<6;++i){
        oss<<" offset "<<i<<" : "<<vecRes[i];
    }
    showMessage(QString::fromStdString(oss.str()));
}
void MainWindow::on_button_plusX_clicked(){
    pEnv->step(0);
}
void MainWindow::on_button_subX_clicked(){
    pEnv->step(1);
}
void MainWindow::on_button_plusY_clicked(){
    pEnv->step(2);
}
void MainWindow::on_button_subY_clicked(){
    pEnv->step(3);
}
void MainWindow::on_button_plusZ_clicked(){
    pEnv->step(4);
}
void MainWindow::on_button_subZ_clicked(){
    pEnv->step(5);
}
void MainWindow::on_button_plusRx_clicked(){
    pEnv->step(6);
}
void MainWindow::on_button_subRx_clicked(){
    pEnv->step(7);
}
void MainWindow::on_button_plusRy_clicked(){
    pEnv->step(8);
}
void MainWindow::on_button_subRy_clicked(){
    pEnv->step(9);
}
void MainWindow::on_button_plusRz_clicked(){
    pEnv->step(10);
}
void MainWindow::on_button_subRz_clicked(){
    pEnv->step(11);
}

void MainWindow::on_pushbutton_teach_clicked(){
    if(ui->pushbutton_teach->text()=="start teach"){
        pTeachRepeat->init_param(&my_robot,100);
        pTeachRepeat->flag=true;
        pTeachRepeat->start();
        ui->pushbutton_teach->setText("end teach");
    }
    else{
        pTeachRepeat->flag=false;
        ui->pushbutton_teach->setText("start teach");
    }
}

void MainWindow::on_pushbutton_repeat_clicked(){
    pTeachRepeat->flag=false;
    pTeachRepeat->start();
}

void MainWindow::on_button_setEnvParam_clicked(){
    pEnv->set_param(&my_robot,&my_force_sensor_one,pAdmitControl);
}
void MainWindow::on_doubleSpinBox_mass_0_valueChanged(double value){
    ui->doubleSpinBox_mass_1->setValue(value);
    ui->doubleSpinBox_mass_2->setValue(value);
}
void MainWindow::on_doubleSpinBox_damp_0_valueChanged(double value){
    ui->doubleSpinBox_damp_1->setValue(value);
    ui->doubleSpinBox_damp_2->setValue(value);
}
void MainWindow::on_doubleSpinBox_k_0_valueChanged(double value){
    ui->doubleSpinBox_k_1->setValue(value);
    ui->doubleSpinBox_k_2->setValue(value);
}
void MainWindow::on_doubleSpinBox_mass_3_valueChanged(double value){
    ui->doubleSpinBox_mass_4->setValue(value);
    ui->doubleSpinBox_mass_5->setValue(value);
}
void MainWindow::on_doubleSpinBox_damp_3_valueChanged(double value){
    ui->doubleSpinBox_damp_4->setValue(value);
    ui->doubleSpinBox_damp_5->setValue(value);
}
void MainWindow::on_doubleSpinBox_k_3_valueChanged(double value){
    ui->doubleSpinBox_k_4->setValue(value);
    ui->doubleSpinBox_k_5->setValue(value);
}
void MainWindow::closeEvent(QCloseEvent *event){
    pDragControl->flag=false;
    pTeachRepeat->flag=false;
    pAdmitControl->flag=false;
    pRobotLoop->flag=false;
}
MainWindow::~MainWindow()
{
    delete ui;
}
