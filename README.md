# smart_robot_platform_cmake

#### 介绍 Introduction
基于六维力传感器的机器人智能控制系统，适用于UR系列机器人  
A smart robot platform for UR series robot based on six-axis force sensor.

#### 软件架构 Software Architecture 
软件采用QT应用程序开发框架编写，使用CMAKE工具构建项目  
The software is written in QT application development framework, using CMAKE tools to build the project .

#### 开发平台 Develop Platform
- ubuntu 16.04
- QT 5.14.2


#### 安装教程 Install

##### 1.  从gitee代码仓库clone项目 Clone this repository
`git clone git@gitee.com:mr-tong/smart_robot_platform_cmake.git`
##### 2.  安装[ur_rtde](https://gitlab.com/sdurobotics/ur_rtde)机器人实时控制工具包 Install ur_rtde

```
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
```
 **注：如以上apt-get update命令出现404无法访问 错误，用以下方式手动安装 If there is a 404 inaccessible error, install it manually**
- 在[软件包源地址](http://ppa.launchpad.net/sdurobotics/ur-rtde/ubuntu/pool/main/u/ur-rtde/)下载librtde librtde-dev软件包.deb文件 
  Download librtde librtde-dev.
- 往您的 `/etc/apt/sources.list` 文件中像下面这样添加一行：`deb http://ftp.de.debian.org/debian sid main ` 
  Add `deb http://ftp.de.debian.org/debian sid main ` to your `sources.list`
- 执行以下命令，删掉旧版本依赖包、更新新版本依赖包、安装boost-thread包、手动安装librtde librtde-dev
  Execute the following command
```
sudo apt-get remove libstdc++6 librtde librtde-dev
sudo apt-get install --only-upgrade libstdc++6 libc6 libc6-dbg
sudo apt-get install libboost-thread1.71.0
sudo dpkg -i librtde-dev_1.3.2-1_amd64.deb  librtde_1.3.2-1_amd64.deb
```
##### 3.  安装boost库 Install Boost
- [boost](https://www.boost.org/)
- 解压.bz2 Decompress
`tar -jxvf xx.tar.bz2`
- 安装 Install
```
./bootstrap.sh
sudo ./b2
sudo ./b2 install
```
#### 使用说明 Guidence

1.  进入工程 cd the local path
`cd smart_robot_platform_cmake`
2.  cmake构建 Build
`cmake ./`
3.  编译 Compile
`make`
4.  运行 Run
`./smart_robot_platform`

#### 参与贡献 Participating Contributions 

1.  Fork 本仓库 Fork this repository
2.  新建 Feat_xxx 分支 Create Feat_xxx branch
3.  提交代码 Push your code
4.  新建 Pull Request Create Pull Request
