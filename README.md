Rosnode for true-north seeking gyrocompass. Tested on Ubuntu 16.04

## Installation:

- Install ROS. Instructions [here](http://wiki.ros.org/kinetic/Installation).
- Setup ROS workspace. Instructions [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
- Install eigen3
```
sudo apt-get install libeigen3-dev
```
- Clone node into src folder of ROS workspace.
```
git clone https://github.com/andrewspielvogel/truenorth.git
```
- Make you workspace. `cd` into the top level of your ROS workspace and run:
```
catkin_make
```
- Create a location for the log file storage or to use default make a KVH log directory in `/log` and give proper permissions
```
sudo mkdir /log   
sudo chown USER.USER /log
chmod +rwx /log           
mkdir /log/kvh
mkdir /log/microstrain
```

## Launch File Params

rate : Rostopic broadcast rate - Rate (hz) of publishing on gyro_data topic. Default: 10

hz : Sampling hz. Default: 5000

port : Serial port to connect to. Default: "/dev/ttyUSB0"

baud : Sensor baud rate. Default: 921600

lat : Latitude (degrees). Default: 39.32

r_align : Instrument frame to vehicle frame rotation (In RPY Euler Angles). Default: "[0,0,0]"

r0 : Initial vehicle estimate of vehicle attitude (In RPY Euler Angles). Default: "[0,0,0]"

ang_bias : Initial estimate of the angular rate bias (rad/s). Default: [0,0,0]"

acc_bias : Initial estimate of the acceleration bias (m/s^2). Default: [0,0,0]"

k_g: Diagonal of the gravity (local level) gain matrix for the attitude estimator. 

k_north: Diagonal of the north gain matrix of the attitude estimator. 

k_acc: Diagonal of the acceleration gain matrix of the bias estimator.

k_acc_bias: Diagonal of the acceleration bias gain matrix of the bias estimator.

k_ang_bias: Diagonal of the angular rate bias gain matrix of the bias estimator.

k_E_n: Diagonal of the Earth rate gain matrix of the bias estimator.


## Using the Node

- Start Node via two different launch files:
```
roslaunch truenorth truenorth.launch
```
launches only the KVH attitude estimator.
```
roslaunch truenorth kvh_ms.launch
```
launches the KVH attitude estimator, Microstrain attitude estimator, and PHINS.


## Log Files
Data logged in two log files. 

KVH IMU hex data logged to /log/kvh/YEAR_MONTH_DAY_HOUR_MINUTE.BKVH

KVH IMU data and attitude logged to /log/kvh/YEAR_MONTH_DAY_HOUR_MINUTE.BKVH



## Generate Documentation

To generate documentation, use doxygen.

- Install doxygen from [here](http://www.stack.nl/~dimitri/doxygen/download.html). Note: you need to install the packages flex and bison before making doxygen with:
```
sudo apt install flex
sudo apt install bison
```

- Then `cd` into the truenorth directory and run:
```
doxygen Doxyfile
```

- To view the documentation, open `index.html` located in the `html/` directory.
