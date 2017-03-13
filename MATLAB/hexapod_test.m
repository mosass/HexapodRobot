close all
clear all

% startup_rtb
robot = hexapod();
robot.plot();

robot.setBodyTransl([0 0 -3 0 0 0]);
robot.animate();




