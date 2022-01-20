
## ROS

ROS is a software framework for programming robots. It provides infrastructure, tools, and capabilities for the same.
When programming in ROS it is encouraged to seperate the code into differen t modules performing different tasks called nodes.

### publishers and subscribers
A ROS Node can be a Publisher or a Subscriber. A Publisher is the one puts the messages of some standard Message Type to a particular Topic. The Subscriber on the other hand subscribes to the Topic so that it receives the messages whenever any message is published to the Topic

![image](https://user-images.githubusercontent.com/67821036/150355687-b02ea92a-0d81-43d0-b2cf-5dfa3a9285ef.png)


1. Nodes created in python or C++ should always be in src package under the catkin workspace
2. roscore should be activated first to run rosnodes and rostopics




## Authors

- [@Anshumaan](https://github.com/Anshumaan031)


## Documentations

[ROS](https://www.ros.org/) 

[Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 

[turtlesim](http://wiki.ros.org/turtlesim) 

[turtlebot3](http://wiki.ros.org/turtlebot3)

[OpenCV](https://pypi.org/project/opencv-python/)


## Installation and Usage

Configure your Ubuntu repositories
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the Ubuntu guide for instructions on doing this.

Setup your sources.list
Setup your computer to accept software from packages.ros.org.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```  

### Set up your keys:

```bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```  

### Installation
First, make sure your Debian package index is up-to-date:

```bash
sudo apt update
```  
Now pick how much of ROS you would like to install.

Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages

```bash
sudo apt install ros-noetic-desktop-full
```  
To find available packages:

```bash
apt search ros-noetic
```  

### Environment setup

You must source this script in every bash terminal you use ROS in.

```bash
source /opt/ros/noetic/setup.bash
```  
Bash:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```  

It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.

### Dependencies for building packages

Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```  
![image](https://user-images.githubusercontent.com/67821036/150355795-d20935b3-058c-4380-ab8d-ec68c7ac2196.png)
![image](https://user-images.githubusercontent.com/67821036/150355908-1148d3d1-08b8-49db-958b-380b8499881a.png)
![image](https://user-images.githubusercontent.com/67821036/150356002-489db07b-a210-4501-a07e-b982a030b645.png)



