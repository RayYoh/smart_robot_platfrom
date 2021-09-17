#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "my_force_sensor_one.h"
#include "my_force_sensor_two.h"
#include <vector>
#include "my_force_data_one.h"
#include "my_force_data_two.h"
#include "my_robot_control.h"
#include "admittance_controller.h"
#include "drag_controller.h"
#include "my_math.h"
#include "robot_assemble_env.h"
#include "teach_repeat.h"
#include "my_robot_loop.h"
#include "my_robot_move.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QString filePath;
    QString toSave;
    QFile txt;
    int d_num = 0;

private:
    Ui::MainWindow *ui;
    MyForceSensorOne my_force_sensor_one;
    MyForceSensorTwo my_force_sensor_two;
    std::vector<double> vForceDataOne;
    std::vector<double> vForceDataTwo;
    MyForceDataOne my_force_data_one;
    MyForceDataTwo my_force_data_two;
    MyRobotControl my_robot;
    QTimer *pTimerTime;
    QTimer *pTimerRobotState;
    QTimer *pTimerPaint;
    QTimer *pTimerSave;
    QLabel *currentTimeLabel;
    QLabel *currentRobotStatusLabel;
    AdmittanceController *pAdmitControl;
    RobotAssembleEnv *pEnv;
    DragController *pDragControl;
    TeachRepeat *pTeachRepeat;
    RobotLoop *pRobotLoop;
    RobotMove *pRobotMove;


    //void createChart();
    void refreshDataOne();
    void refreshChartOne();
    void refreshDataTwo();
    void refreshChartTwo();
    void refreshWidget();
    std::vector<double> getMass();
    std::vector<double> getDamp();
    std::vector<double> getK();
    std::vector<double> getDragControlK();
    std::vector<double> getForceControlAim();
    double getMaxVel();
    double getMaxAcc();
    double getControlFreq();
    double getSpeedRate();
    std::vector<bool> getAdmitAxis();
    void showMessage(QString text);
    std::vector<double> getOffset();
    void closeEvent(QCloseEvent *event);
public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_checkbox_show_fx_stateChanged(int state);
    void on_checkbox_show_fy_stateChanged(int state);
    void on_checkbox_show_fz_stateChanged(int state);
    void on_checkbox_show_mx_stateChanged(int state);
    void on_checkbox_show_my_stateChanged(int state);
    void on_checkbox_show_mz_stateChanged(int state);

    void on_checkbox_ch0_stateChanged(int state);
    void on_checkbox_ch1_stateChanged(int state);
    void on_checkbox_ch2_stateChanged(int state);
    void on_checkbox_ch3_stateChanged(int state);
    void on_checkbox_ch4_stateChanged(int state);
    void on_checkbox_ch5_stateChanged(int state);
    void on_checkbox_ch6_stateChanged(int state);
    void on_checkbox_ch7_stateChanged(int state);
    void on_checkbox_ch8_stateChanged(int state);
    void on_checkbox_ch9_stateChanged(int state);
    void on_checkbox_ch10_stateChanged(int state);
    void on_checkbox_ch11_stateChanged(int state);

    void on_checkbox_toolSpeed_stateChanged(int state);

    void on_doubleSpinBox_mass_0_valueChanged(double value);
    void on_doubleSpinBox_damp_0_valueChanged(double value);
    void on_doubleSpinBox_k_0_valueChanged(double value);
    void on_doubleSpinBox_mass_3_valueChanged(double value);
    void on_doubleSpinBox_damp_3_valueChanged(double value);
    void on_doubleSpinBox_k_3_valueChanged(double value);

    void on_button_startCollect_clicked();
    void on_button_setZero_clicked();
    void on_button_changeForce_clicked();
    void on_button_saveData_clicked();
    void on_button_setPath_clicked();
    void on_button_loop_clicked();
    void on_button_caluG_clicked();
    void on_button_robotConn_clicked();
    void on_button_robotMove_clicked();
    void on_button_teach_clicked();
    void on_button_stop_clicked();

    void on_button_setAdmitControlParam_clicked();
    void on_button_startAdmitControl_clicked();
    void on_button_stepTime_clicked();
    void on_button_jointTorques_clicked();
    void on_button_tcpForce_clicked();
    void on_button_moveInToolCoordinate_clicked();
    void on_button_coorTrans_clicked();

    void on_button_plusX_clicked();
    void on_button_subX_clicked();
    void on_button_plusY_clicked();
    void on_button_subY_clicked();
    void on_button_plusZ_clicked();
    void on_button_subZ_clicked();

    void on_button_plusRx_clicked();
    void on_button_subRx_clicked();
    void on_button_plusRy_clicked();
    void on_button_subRy_clicked();
    void on_button_plusRz_clicked();
    void on_button_subRz_clicked();

    void on_button_setEnvParam_clicked();

    void on_button_setDragControlParam_clicked();
    void on_button_startDragControl_clicked();

    void on_pushbutton_teach_clicked();
    void on_pushbutton_repeat_clicked();
    /******************************************
    ** Manual connections
    *******************************************/
    void onSensorConnectOne();
    void onSensorDisConnectOne();
    void onSensorHaveDataOne();
    void onSensorConnectTwo();
    void onSensorDisConnectTwo();
    void onSensorHaveDataTwo();
    void onTimerTimeTimeout();
    void onTimerRobotStatusTimeout();
    void onTimerPaintTimeout();
    void onTimerSaveTimeout();
};

#endif // MAINWINDOW_H
