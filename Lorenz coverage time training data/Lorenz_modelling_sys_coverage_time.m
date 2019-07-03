clc; clear all; close all;
%Written by Karan Sridharan
%Calculates the time taken for the Robot to cover a certain percentage of
%the map using Lorenz Equation Dynamical System. Plots the Robots
%Trajectory and shows the number of cells the Robot has covered

%Specifying the parameters to the dynamical system
sigma = 15; beta = 3; r = 96; v = 1; dt = 0.01; 

%Specifying a range of ICs 
x0 = [1:3:10]; y0 = [1:3:10]; z0 = [1:3:10]; X0 = 0; Y0 = 0; 

%getting the ICs that do not make trajectories go to Infinity or NaN
[allowable_IC_mat] = allowable_IC_lorenz(sigma,beta,r,v,dt,x0,y0,z0,X0,Y0);

%checking to see if there are no combinations of allowable ICs
%The code will stop executing
if isempty(allowable_IC_mat);
    error('Please select different chaotic parameters')
end

%Importing the excel spreadsheet. The spreadsheet will register all the
%coverage times with different parameters and allowed ICs
[temp,temp2,coveragetimelorenz] = xlsread('coverage_time_lorenz.xlsx');

%row_index will tell at what row should the elements be placed at 
%+1 indicates that the elements should be placed in the next row. 
%Once the Excel Spreadsheet gets imported, the next set of coverage time entries 
%for different parameter values will not overwrite the previous entries
row_index = size(coveragetimelorenz,1)+1;

%This variable tells the type of dynamical system we are testing
%1 = Henon Map, 2 = Logistic Map, 3 = Lorenz, 4 = Rossler
dynam_syst_num = 3;

%Index number for allowable_IC_mat
j = 1;
%The following code gets all the coverage times according to the number of
%combintions allowable_IC_mat has. There are 5 cols for parameters and 4
%cols for ICs. This is because certain dynamical systems might have 5
%parameters and 4 ICs. For example, those that do not have that many but have 3
%parameters and 2 ICs, the remaining cols are set to 0
for i = row_index:row_index+size(allowable_IC_mat,1)-1
    coveragetimelorenz{i,2} = sigma; 
    coveragetimelorenz{i,3} = beta;
    coveragetimelorenz{i,4} = r; 
    coveragetimelorenz{i,5} = 0; 
    coveragetimelorenz{i,6} = 0; 
    coveragetimelorenz{i,7} = allowable_IC_mat(j,1);
    coveragetimelorenz{i,8} = allowable_IC_mat(j,2); 
    coveragetimelorenz{i,9} = allowable_IC_mat(j,3); 
    coveragetimelorenz{i,10} = 0;
    coveragetimelorenz{i,11} = dynam_syst_num;
    IC_vec = [allowable_IC_mat(j,1),allowable_IC_mat(j,2),allowable_IC_mat(j,3),X0,Y0];
    coveragetimelorenz{i,1} = Lorenz_coverage_orig(sigma,beta,r,v,IC_vec);
    j = j + 1;
end
%The excel file writes all the coverage times and stores it for future use
xlswrite('coverage_time_lorenz.xlsx',coveragetimelorenz)
