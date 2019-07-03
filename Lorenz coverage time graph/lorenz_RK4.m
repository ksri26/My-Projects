function [xnew,ynew,znew,Xnew,Ynew] = lorenz_RK4(sigma,beta,r,v,x,y,z,X,Y,dt)
%Written by Karan Sridharan
%Returns a new, discretized coordinate position for the Robot using the Lorenz Dynamical
%system. 
%Input variables:
%x = Dynamical x coordinate
%y = Dynamical y coordinate
%z = Dynamical z coordinate
%X = Robotic X coordinate
%Y = Robotic Y coordinate
%sigma, beta and r= parametric variables of the Lorenz Dynamical system
%v = velocity of the Robot
%dt = time step

%Output variables:
%xnew = New dynamical x coordinate
%ynew = New dynamical y coordinate
%znew = New dynamical z coordinate
%Xnew = New dynamical X coordinate
%Ynew = New dynamical Y coordinate



K1_x = -beta*x + y*z;
K1_y = -sigma*(y-z);
K1_z = -x*y + r*y - z;

K2_x = -beta*(x+0.5*dt*K1_x) + (y+0.5*dt*K1_y)*(z+0.5*dt*K1_z);
K2_y = -sigma*((y+0.5*dt*K1_y) - (z+0.5*dt*K1_z));
K2_z = -(x+0.5*dt*K1_x)*(y+0.5*dt*K1_y) + r*(y+0.5*dt*K1_y)-(z+0.5*dt*K1_z);


K3_x = -beta*(x+0.5*dt*K2_x) + (y+0.5*dt*K2_y)*(z+0.5*dt*K2_z);
K3_y = -sigma*((y+0.5*dt*K2_y) - (z+0.5*dt*K2_z));
K3_z = -(x+0.5*dt*K2_x)*(y+0.5*dt*K2_y) + r*(y+0.5*dt*K2_y)-(z+0.5*dt*K2_z);


K4_x = -beta*(x+dt*K3_x) + (y+dt*K3_y)*(z+dt*K3_z);
K4_y = -sigma*((y+dt*K3_y) - (z+dt*K3_z));
K4_z = -(x+dt*K3_x)*(y+dt*K3_y) + r*(y+dt*K3_y)-(z+dt*K3_z);


xnew = x + (dt/6)*(K1_x + 2*K2_x + 2*K3_x + K4_x);
ynew = y + (dt/6)*(K1_y + 2*K2_y + 2*K3_y + K4_y);
znew = z + (dt/6)*(K1_z + 2*K2_z + 2*K3_z + K4_z);
Xnew = X + 0.1*v*cos(x);
Ynew = Y + 0.1*v*sin(x);


