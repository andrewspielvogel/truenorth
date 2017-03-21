Rosnode for true-north seeking gyrocompass. Tested on Ubuntu 14.04

##Installation:

- Install ROS. Instructions [here](http://wiki.ros.org/indigo/Installation).
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
- Create a location for the log file storage or to use default make a KVH log directory in `/var/log/` and give proper permissions
```
sudo mkdir /var/log/KVH
```
```
sudo chmod a+rwx /var/log/KVH
```

##Launch File Params

rate : Rostopic broadcast rate - Rate (hz) of publishing on gyro_data topic. Default is 10 hz.

hz : Sampling hz. Default is 1000 hz.

port : Serial port to connect to. Default is /dev/ttyUSB0.

baud : Sensor baud rate. Default is 921600.

latitude : Latitude. Default 39.32 degrees.

instr_align : Instrument frame to vehicle frame rotation. Default is identity. (Input in launch file as a comma separated string of 9 doubles.)

gains : Gains for sensor bias and attitude estimation. (Input in launch file as a comma separated string of 5 doubles.) Order is "acc,b_acc,b_ang,b_z,R_bar,kg,kw,east_cutoff frequency". Default is 1.0,0.005,0.005,0.005,1.0,1.0,0.05,0.005.

log_loc : Directory for writing log files to. Default is /var/log/KVH.

## Using the Node

- Start ROS core
```
roscore
```
- Start node
```
roslaunch truenorth truenorth.launch
```

## Generate Documentation

To generate documentation, use doxygen.

- Install doxygen from [here](http://www.stack.nl/~dimitri/doxygen/download.html). Note you need to install the packages flex and bison before making doxygen with:
```
sudo apt install flex
sudo apt instal bison
```

- Then `cd` into the truenorth directory and run:
```
doxygen Doxyfile
```

- To view the documentation, open `index.html` located in the `html/` directory.