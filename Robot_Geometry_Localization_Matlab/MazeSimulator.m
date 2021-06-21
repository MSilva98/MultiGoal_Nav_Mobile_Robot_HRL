%##########################################################################
% File: MazeSimulator.m
% Date: 15-04-2019
% Purpose: this script simulates the motion of a mobile robot in a maze, 
%           as well as the infrared sensor readings (Sharp GP2Y=A41SK0F)  
%##########################################################################
clear all
close all
clc
warning('off')
%##########################################################################
% PARAMETER INITIALIZATION SCRIPT
%##########################################################################
IniParam        % Script 
%##########################################################################
% SIMULATE THE ROBOT'S MOTION
%##########################################################################
% Robot localization (x,y,fi), "ground-truth" --> pre-allocation for speed    
x = 0.10; y = 1.0; fi = 0*pi/180; % Initial robot's configuration
% Sensor distance & intersection point --> pre-allocation for speed
NUM_IR_sensors = length(IR_sensor_ori);
% Read IR sensor data
[Dist_IR_sensor,IPoint_IR_sensor] = IR_SensorData(A,B,x,y,fi,IR_sensor_ori);
%##########################################################################
% GRAPHICAL ANIMATION OF THE ROBOT'S MOTION
% A,B --> maze layout
% [x y,fi] --> robot location
% IPoint_IR_sensor --> IR sensor data
%##########################################################################
 GraphicalAnimation(A,B, [x y],fi, IPoint_IR_sensor)
%##########################################################################
% ESTIMATE ROBOT'S LOCAL POSITION & ORIENTATION
% Local position & orientation in an endless corridor
%##########################################################################
% CORRIDOR: analytical/geometrical solution
IR_sensor_max_dist = 0.3;
SOL_final = RobotLocalCorridorPosture(Dist_IR_sensor,IR_sensor_max_dist);
SOLUTION = [SOL_final(:,1) SOL_final(:,2)*180/pi]
%#######################################################################END