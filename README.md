# Required: #
ROS kinetic,
Ubuntu 16.04 64-bit
  
  
# Instructions: #
* Setup your sources.list 
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  
``` 

* Set up your keys
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116  
```
  
* Installation
```
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```
   
* Initialize rosdep
```
$ sudo rosdep init
$ rosdep update
```

* Environment setup
```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
* Dependencies for building packages
```
$ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
* Managing Your Environment
```
$ source /opt/ros/kinetic/setup.bash
```

* Create a ROS Workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

- Git Clone of project et other
  - $ git clone https://github.com/GiovannaMelchiorre/ROS_NAO.git
  - $ git clone https://github.com/orbbec/ros_astra_launch.git
  - $ git clone https://github.com/orbbec/ros_astra_camera.git


* To build project
```
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```
NB. To build Astra libraries, the package uses the network


* To run project
 ```
 roslaunch project_launch project.launch pport:=NAO_PORT pip:=NAO_IP 
 ```
 If we use simulator as Choregraphe, NAO_PORT and NAO_IP are respectively the port to which the robot is connected and the IP address 127.0.0.1, else they are the parameter of real robot.
 
