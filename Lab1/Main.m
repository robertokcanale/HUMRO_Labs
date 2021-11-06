%% HUMRO REPORT 1 TESTING FUNCTIONS AND RESULTS
clear all
clc
close all

%% reaction_force
%data and funtion for the computation of the reaction force 
q1 = 1; 
q2 = -0.5;

qd1= 0.1;
qd2 = 0.2;

qdd1 = 0.2;
qdd2 = 0.3;

theta = 2*pi/180;

%% Dynamic Model
[A,H] = function_dyn(q1, q2, qd1, qd2, theta)

%% Ground Reaction
[F] = function_reactionforce(q1,q2,qd1,qd2,qdd1,qdd2,theta)
%% impact

[A1, JR] = function_impact( q1, q2)
