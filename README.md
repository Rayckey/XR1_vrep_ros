# XR1_vrep_ros

##This is an example for communication between ROS and V-REP

### Compiling

0. Make sure V-REP and v_repExtRosInterface(https://github.com/Rayckey/v_repExtRosInterface.git) are installed.
1. Download, and name the package vrep_test
```
$ git clone https://github.com/Rayckey/XR1_vrep_ros.git vrep_test
```
2. Return to work space and build using catkin build (catkin make may cause issues)
```
$ catkin build
```
3. Launch the plugin
```
$ rqt
```
	and find XR1_VREP in plugins
	or
```
$ rqt --standalone vrep_test
```


###Running

##Joint Control
1. Launch V-REP by going into the V-REP installation folder and type the following in a terminal
```
$ ./vrep.sh
```
2. Open the scene file under `/PATH/TO/VREP_TEST/model `
3. Launch rqt , and Click 'Start Simulation' to begin the simulation on V-REP
4. The left hand side allow simple joint control

##Inverse Kinematics (Generated in V-REP)
5. Switch to Inverse Kinematics Mode by changing the combobox next to the `Start Simulation` button
6. The Right Hand Side allow manipulation of the End Effector

##Finger Manipulation
7. The lower Group allow simple finger control

##Record and Play
8. Use `rqt_bag` for publiher recording and playing


# XR1_vrep_ros

