# Four-legged crawling robot

### This repo is to show the project I participated during my internship, I only upload .h file.

### This is the source code of the slave machine of a four-legged crawling robot. The robot is the 2nd generation of the wall-cleaning robot with a clean arm implemented on top of the robot. Each of the robot's leg consists of 3 electric putters and 1 servomotor, which is flexible enough to allow the robot performing difficult gestures.

### The four-legged crawling robot embedded system program using C with software Keil5, which inputs and analyzes data from a master machine and implements the robot’s gestures (including straight-walk, turn-around, and push-up) by solving the robot’s legs’ kinematical equation.

### Used MODBUS and created a 22-bit protocol designed to accomplish master-slave communications using C#. The communications software can remotely control the robot to switch to different gestures and move to designated physical locations.

# Image
![image](https://github.com/guojutaoo/Robotics/blob/master/STM32_Project/Image/robot.jpg)
