# AdvanDiptera
## Introduction

In this repository you will find the scripts necessary to fly an autonomous drone which will change his direction depending on the detection of some RFID tags. 

## Environment installation

This repository contains code supported on the following installation:

* Install Linux in your Raspberry Pi using the following [webpage](https://ubuntu.com/download/raspberry-pi) 

* Install ROS Melodic following the steps included in this [webpage](http://wiki.ros.org/melodic)

* Install MAVROS using the following commands:

  ```
  mkdir -p ~/AdvanDiptera/src
  cd ~/AdvanDiptera
  catkin init
  wstool init src
  rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
  rosinstall_generator --upstream-development mavros mavros_extras mavros_msgs test_mavros sensor_msgs  control_toolbox realtime_tools tf tf2_ros python_orocos_kdl urdf |tee -a /tmp/mavros.rosinstall
  wstool merge -t src /tmp/mavros.rosinstall
  wstool update -t src -j4
  rosdep install --from-paths src --ignore-src -y
  sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
  catkin build -DCATKIN_ENABLE_TESTING=0 -j2
  ```
* Connect your SD Card to a laptop and change the following files:

  * btcmd.txt, you have to have at the end of the file the following lines (add the missing ones):
  
    ```
    net.ifnames=0 dwc_otg.lpm_enable=0 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc
	
    ``` 
 
  * config.txt, you have to have at the end of the file the following lines (add the missing ones):
    
    ```
    enable_uart=1
    dtoverlay=pi3-disable-bt
    setenv stdin nulldev
    include syscfg.txt
    include usercfg.txt
    ```  
    
  * nobtcfg.txt, you have to have at the end of the file the following lines (add the missing ones):
    
    ```
    enable_uart=1
    cmdline=nobtcmd.txt
    ```  
    
  * nobctmd.txt, you have to have at the end of the file the following lines (add the missing ones):
  
    ```
    dwc_otg.lpm_enable=0  console=tty3 root=/dev/mmcblk0p2 rootfstype=ext4  elevator=deadline fsck.repair=yes   rootwait
    ```	 
	     
  * syscfg.txt, you have to have at the end of the file the following lines (add the missing ones):
    
    ```
    dtparam=i2c_arm=on
    dtparam=spi=on
    enable_uart=1
    dtoverlay=pi3-disable-bt
    setenv stdin nulldev
    include nobtcfg.txt
    ```   

* Download our repository and copy it to the src folder of your catkin repository. 

* Open the bashrc file and copy one of the following lines at the end of your file and comment the other one:
	
    * Add the following line at the end 
    
    ```
    source /home/ubuntu/AdvanDiptera/devel/setup.bash
    ```
    
    * Comment the following line  
    
    ```
    source /home/ubuntu/AdvanDiptera/devel/setup.bash
    ```
	
Edit the following line of your px4.launch file:

```
rosrun mavros mavros_node _fcu_url:=/dev/ttyAMA0:57600
```

To: 

```
rosrun mavros mavros_node _fcu_url:=/dev/ttyAMA0:921600
```

Install RPi.GPIO:
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-pip python-dev
sudo pip install RPi.GPIO  
```

If during MAVROS launch you have any problem due to permisions run the following command:

```
chmod 777 /dev/ttyAMA0
```

Install minicom:
```
sudo apt‑get install minicom
```

With the following command you will be able to check the sonar sensor (change the number of the USB depending on your case):
```
minicom -b 9600 -o -D /dev/ttyUSB0
```
To check if it is the port where you have your sonar connected, check it with the following command:

```
dmesg | grep tty
```
