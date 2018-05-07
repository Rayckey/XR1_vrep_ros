# XR1_vrep_ros

## This is an example for communication between ROS and V-REP

### Compiling

0. Make sure V-REP and v_repExtRosInterface(https://github.com/Rayckey/v_repExtRosInterface.git) are installed.
1. Download, and name the package vrep_test
```
$ git clone https://github.com/Rayckey/XR1_vrep_ros.git vrep_test
```
2. Return to work space and build using catkin build (or catkin_make)
```
$ catkin build
```
3. Launch the plugin
```
$ rqt
```
4. and find XR1_VREP in plugins or just
```
$ rqt --standalone vrep_test
```


### Running

## Joint Control
0. Launch roscore
1. Launch rqt and V-REP by going into the source folder and excecute script
```
$ cd /PATH/TO/VREP_TEST/
$ ./XR1_vrep.sh
```

## Inverse Kinematics (Generated in V-REP)
2. Switch to Inverse Kinematics Mode by Going into the inverse kinematics tab
3. Set Position of the End Effector OR just drag the `Left_Tip_Target` or `Right_Tip_Target` Dummies on V-REP simulation

## Vision Sensors
4. Click `Refresh` to detect all publishing images
5. Use the combo box to select image topics

## Record and Play
6. Use the Add Action button to record the current position, use the 'Set Time(s)' text box to modify the amount of time this action should take
7. Selecting the action will allow the user to review the configuration
8. Modify Action will overwrite the seleted action with the current configuration
9. Pressing Play will play the animation starting from the SELETED action
10. Use the Remove Action to remove the selected action
11. Use Save and Read button to save and load the META data (i.e. only the key frames will be saved)
12. Use Generate Button to write out the position data needed to drive the robot
 

# XR1_vrep_ros

