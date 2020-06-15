# AdvanDiptera
## Introduction

In this repository you will find the scripts necessary to fly an autonomous drone which will change his direction depending on the detection of some RFID tags. 

## Environment installation

This repository contains code supported on the following installation:

* Install Linux in your Raspberry Pi using the following [webpage](https://ubuntu.com/download/raspberry-pi) 

* Install ROS Melodic following the steps included in this [webpage](http://wiki.ros.org/melodic)

* Install MAVROS following the steps in Source installation in this [webpage](https://github.com/mavlink/mavros/blob/master/mavros/README.md)

* If the previous step doesn't work install it using Binary installation

* Download our repository and copy it to the src folder of your catkin repository. 

* Open the bashrc file and copy one of the following lines at the end of your file:

    * If you installed MAVROS using Binary installation 
	
    ```
	source /opt/ros/melodic/setup.bash 
    ```
	
    * If you installed MAVROS using Source installation 
	
	```
	source /home/ubuntu/AdvanDiptera/devel/setup.bash
	```
	
If during the installation you have any problem due to permisions run the following command:

```
chmod 777
```

If you still have any problem run the following command (this command is mandatory to run some python scripts in this repository):

```
sudo -s
```

Edit the following line of your px4.launch file:

```
rosrun mavros mavros_node _fcu_url:=/dev/ttyAMA0:57600
```

To: 

```
rosrun mavros mavros_node _fcu_url:=/dev/ttyS0:921600
```
