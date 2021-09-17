/**
 * @file /src/my_force_sensor.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <ros/ros.h>
//#include <ros/network.h>
#include <iostream>
#include <string>
//#include <std_msgs/String.h>
#include <sstream>
#include "my_force_sensor.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
//namespace smart_robot_platform {

/*****************************************************************************
** Implementation
*****************************************************************************/
MyForceSensor::MyForceSensor(quint16 port) :
    isCanSend(false),
    isCanSet(false),
    isCanChange(false),
    isNeedChange(false),
      my_port(port)
  {
}

MyForceSensor::~MyForceSensor() {
}

bool MyForceSensor::init() {
  pTcpServer = new QTcpServer(this);
  connect(pTcpServer,SIGNAL(newConnection()),this,SLOT(onNewConnection()));
  startListen();
  return true;
}
void MyForceSensor::startListen(){
    pTcpServer->listen(QHostAddress("192.168.1.3"),my_port);
}
void MyForceSensor::onNewConnection(){
  pTcpSocket = pTcpServer->nextPendingConnection();//get socket
  connect(pTcpSocket,SIGNAL(disconnected()),this,SLOT(onClientDisconnected()));
  connect(pTcpSocket,SIGNAL(stateChanged(QAbstractSocket::SocketState)),this,SLOT(onSocketStateChange(QAbstractSocket::SocketState)));
  connect(pTcpSocket,SIGNAL(readyRead()),this,SLOT(onSocketReadyRead()));
  isCanSend=true;
  isCanSet=true;
  clientConnected();
  return;
}
void MyForceSensor::clientConnected(){
  Q_EMIT sensorConnect();
}
void MyForceSensor::onClientDisconnected(){
  exit();
  pTcpSocket->deleteLater();
  isCanSend=false;
  isCanSet=false;
  isCanChange=false;
  Q_EMIT sensorDisConnect();
}
void MyForceSensor::onSocketStateChange(QAbstractSocket::SocketState socketState){
  switch (socketState) {
  case QAbstractSocket::UnconnectedState:
    break;
  case QAbstractSocket::HostLookupState:
    break;
  case QAbstractSocket::ConnectingState:
    break;
  case QAbstractSocket::ConnectedState:
    break;
  case QAbstractSocket::BoundState:
    break;
  case QAbstractSocket::ClosingState:
    break;
  case QAbstractSocket::ListeningState:
    break;
  }
}
bool MyForceSensor::isAllowSend(){
  if(pTcpSocket==nullptr)
    return false;
  if(pTcpSocket->state()==QAbstractSocket::ConnectedState)
    return true;
  return false;
}
void MyForceSensor::onSocketReadyRead(){
}
void MyForceSensor::sendMsg(QByteArray str){
    if(pTcpSocket==nullptr)
        return;
    if(pTcpSocket->state()!=QAbstractSocket::ConnectedState)
        return;
    pTcpSocket->write(str);
}
void MyForceSensor::changeForce(bool isEnable){
    isNeedChange = isEnable;
}
vector<double> MyForceSensor::getChangeData(){
}
//}  // namespace smart_robot_platform
