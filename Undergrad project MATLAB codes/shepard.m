function [weight_func] = shepard(x_coord,y_coord,xc_vec,yc_vec,h)
%Written by Karan Sridharan
%Using shepards weighted function to get the average particle velocity per
%grid

weight_func =  exp(-(((x_coord-xc_vec)^2/h^2) + ((y_coord-yc_vec)^2/h^2)));
end