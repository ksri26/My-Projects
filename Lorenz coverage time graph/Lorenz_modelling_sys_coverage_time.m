clc; clear all; close all;
%Written by Karan Sridharan
%Calculates the time taken for the Robot to cover a certain percentage of
%the map using Lorenz Equation Dynamical System. Plots the Robots
%Trajectory and shows the number of cells the Robot has covered

%Specifying the chaotic parameters. v is the Robot's velocity
sigma = 10; beta = 8/3; r = 28; v = 1; 

%Specifying the initial condition
x0 = 10; y0 = 363; z0 = 500; X0 = 0; Y0 = 0;
IC_vec = [x0,y0,z0,X0,Y0]; %Initial condition is stored as a column vector

%Obtaining the coverage time
[coverage_time] = Lorenz_coverage_orig(sigma,beta,r,v, IC_vec)


