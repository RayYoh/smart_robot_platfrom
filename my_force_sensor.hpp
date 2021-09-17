#ifndef MY_FORCE_SENSOR_HPP
#define MY_FORCE_SENSOR_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
//#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QTcpSocket>
#include <QTcpServer>
#include <vector>
#include <QTime>
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

//namespace smart_robot_platform {

/*****************************************************************************
** Class
*****************************************************************************/

using namespace std;
class MyForceSensor : public QThread {
    Q_OBJECT
public:
  MyForceSensor(quint16 port);
  virtual ~MyForceSensor();
  bool init();
  void sendMsg(QByteArray str);
  void setZero();
  /*********************
  ** Logging
  **********************/
  enum LogLevel {
           Debug,
           Info,
           Warn,
           Error,
           Fatal
   };
  void log( const LogLevel &level, const std::string &msg);
  bool isAllowSend();
  void changeForce(bool isEnable);
  virtual vector<double> getChangeData();

  QTcpServer *pTcpServer;//TCP Server
  QTcpSocket *pTcpSocket;//TCP communicate Socket
  bool isCanSend;
  bool isCanSet;
  bool isCanChange;
  bool isNeedChange;
  QTime time_for_compute;
Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void sensorConnect();
  void sensorDisConnect();
  void sensorHaveData();
public Q_SLOTS:
  void clientConnected();
private:
  void startListen();

public Q_SLOTS:
  virtual void onSocketReadyRead();
  virtual void onNewConnection();
private Q_SLOTS:
  void onClientDisconnected();
  void onSocketStateChange(QAbstractSocket::SocketState socketState);
private:
  quint16 my_port;
  std::string senseor_name;
//  ros::Publisher data_publisher;
};

//}  // namespace smart_robot_platform

#endif // MY_FORCE_SENSOR_HPP
