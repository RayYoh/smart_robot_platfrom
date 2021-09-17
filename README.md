# smart_robot_platform_cmake

#### Introduction
A smart robot platform for UR series robot based on six-axis force sensor.

#### Software Architecture 
The software is written in QT application development framework, using CMAKE tools to build the project .

#### Develop Platform
- ubuntu 16.04
- QT 5.14.2

#### Install

##### 1.  Clone this repository
`git clone git@gitee.com:mr-tong/smart_robot_platform_cmake.git` or `git clone https://github.com/RayYoh/smart_robot_platfrom.git`
##### 2.  Install[ur_rtde](https://gitlab.com/sdurobotics/ur_rtde)

```
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
```

 **PS: If there is a 404 inaccessible error, install it manually**
- Download librtde[librtde-dev.](http://ppa.launchpad.net/sdurobotics/ur-rtde/ubuntu/pool/main/u/ur-rtde/)   
- Add `deb http://ftp.de.debian.org/debian sid main ` to `/etc/apt/sources.list`
- Execute the following command
```
sudo apt-get remove libstdc++6 librtde librtde-dev
sudo apt-get install --only-upgrade libstdc++6 libc6 libc6-dbg
sudo apt-get install libboost-thread1.71.0
sudo dpkg -i librtde-dev_1.3.2-1_amd64.deb  librtde_1.3.2-1_amd64.deb
```

##### 3.  Install Boost
- [boost](https://www.boost.org/)
- Decompress
`tar -jxvf xx.tar.bz2`
- Install
```
./bootstrap.sh
sudo ./b2
sudo ./b2 install
```

#### Guidence

1.  cd the local path
`cd smart_robot_platform_cmake`
1.  Build
`cmake ./`
1.  Compile
`make`
1.  Run
`./smart_robot_platform`

#### Participating Contributions 

1.  Fork this repository
2.  Create Feat_xxx branch
3.  Push your code
4.  Create Pull Request