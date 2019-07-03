function [Xcoord, Ycoord] = mirrormap(Xcoord,Ycoord,xmin,xmax,ymin,ymax)
%Written by Karan Sridharan
%Transforms points that are lying from out of the map into the map
%Input Variables:
%xmin: minimum x point located in the x-axis
%xmax: maximum x point located in the x-axis
%ymin: minimum y point located in the y-axis
%ymax: maximum y point located in the x-axis
%px: vector, p containing the x coordinates of the Robot's trajectory
%py: vector p containing the y coordinates of the Robot's trajectory

%Output variable
%px: mirrored vector, p containing the x coordinates of the Robot's trajectory
%py: mirrored vector p containing the y coordinates of the Robot's trajectory

%case 1

    if Xcoord < xmin && ymin <= Ycoord && Ycoord <= ymax
        Xcoord = -Xcoord + 2*xmin;

    
%case 2

    elseif Xcoord > xmax && ymin <= Ycoord && Ycoord <= ymax
        Xcoord = -Xcoord + 2*xmax;


%case 3

    elseif Ycoord < ymin && xmin <= Xcoord && Xcoord <= xmax
        Ycoord = -Ycoord + 2*ymin;

    

%case 4

    elseif Ycoord > ymax && xmin <= Xcoord && Xcoord <= xmax
        Ycoord = -Ycoord + 2*ymax;

    
   
    
%case 5

    elseif Xcoord < xmin && Ycoord < ymin
        Xcoord = -Xcoord + 2*xmin;
        Ycoord = -Ycoord + 2*ymin;
        
    

%case 6

    elseif Xcoord > xmax && Ycoord < ymin
        Xcoord = -Xcoord + 2*xmax;
        Ycoord = -Ycoord + 2*ymin;
    

%case 7

    elseif Ycoord > ymax && Xcoord < xmin
        Xcoord = -Xcoord + 2*xmin;
        Ycoord = -Ycoord + 2*ymax;

%case 8

    elseif Ycoord > ymax && Xcoord > xmax
        Xcoord = -Xcoord + 2*xmax;
        Ycoord = -Ycoord + 2*ymax;
    end
    
