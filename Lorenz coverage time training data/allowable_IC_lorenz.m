function [allowable_IC_mat] = allowable_IC_lorenz(sigma,beta,r,v,dt,x0,y0,z0,X0,Y0)
%Written by Karan Sridharan
%Gets a list of acceptable ICs. For an acceptable IC to occur, at every iteration 
%of obtaining new dynamical coordinates, the new dynamical coordinates 
%will not head to +/-Inf

%Input Variables:
%a = chaotic parameter
%b = chaotic parameter
%x0 = Dynamical x coordinate
%y0 = Dynamical y coordinate

%Output Variables:
%allowable_IC_mat = A matrix of allowable combinations of IC

%Dynamical coordinates that head to +/-Inf will lead Robotic coordinates
%to also be +/-Inf, causing errors to the analysis of obtaining new
%Robotic trajectory points

tmp_mat = []; %Temporary matrix that stores a list of acceptable ICs.
%Every page represents each IC_z and each column represents each IC_y. 
%For every IC_z and IC_y, the following lines of
%code will obtain new x,y,z coordinates. At the end of the iteration
%the tmp matrix will be assigned with entries of the new x coordinates at
%every row

i = 1;%page index for tmp_mat
for l = 1:length(z0); %l is an index that takes every IC_z
    j = 1; %col index for tmp_mat
    for m = 1:length(y0) %m is an index that takes every IC_y
        k = 1; %row index for tmp_mat
        for n = 1:length(x0) %n is an index that takes every IC_x
            IC_vec = [x0(n),y0(m),z0(l),X0,Y0];
            %For 10000 iterations, the following line will compute new
            %x,y,z coordinates
                for nn = 1:10000
                    [IC_vec(nn+1,1),IC_vec(nn+1,2),IC_vec(nn+1,3),IC_vec(nn+1,4),IC_vec(nn+1,5)] = ...
                    lorenz_RK4(sigma,beta,r,v,IC_vec(nn,1),IC_vec(nn,2),IC_vec(nn,3),IC_vec(nn,4),IC_vec(nn,5),dt);
                
                %If the new x coordinate becomes Inf, then the code will stop
                %getting new points
                if isnan(IC_vec(end,1)) || isinf(IC_vec(end,1))
                    break
                end
                end
                
                %tmp_mat will store the last x coordinate value
                tmp_mat(k,j,i) = IC_vec(end,1);
                k = k+1; %gets the next row index
        end
        j = j+1; %gets the next col index
    end
    i = i+1; %gets the next page index
end

%The following line of code will check to see whether each x coordinate in the matrix
%is Inf or NaN. It will get the ICs of those x coordinates that are not 
%Inf nor NaN and the ICs will pass on to a matrix that stores these acceptable ICs
allowable_IC_mat = [];

m = 1; %row index of the acceptable IC matrix
for i = 1:size(tmp_mat,3)
    for j = 1:size(tmp_mat,2)
        for k = 1:size(tmp_mat,1)
            if isnan(tmp_mat(k,j,i)) || isinf(tmp_mat(k,j,i))
                continue
            else
                allowable_IC_mat(m,1) = x0(k);%gets the accepted IC_x
                allowable_IC_mat(m,2) = y0(j);%gets the accepted IC_y
                allowable_IC_mat(m,3) = z0(i);%gets the accepted IC_z
                m = m+1;%gets the next row index
            end
        end
    end
end

        



     