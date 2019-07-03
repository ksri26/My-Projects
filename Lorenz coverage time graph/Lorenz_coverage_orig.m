function [coverage_time] = Lorenz_coverage_orig(sigma,beta,r,v, IC_vec)
%Written by Karan Sridharan
%Computes the robots trajectory at a given map and time taken to complete
%the whole circuit. Henon Map dynamical system is used to compute the Robot's
%trajectory 

%Input Variables:
%sigma = chaotic parameter
%beta = chaotic parameter
%r = chaotic parameter
%v = Robots velocity
%IC_vec = vector of Initial conditions

%Output Variables
%coverage_time = time taken to cover 90% of the map
%The output comes along with the map showing the number of each cells
%covered and the computational time

tot_covrate = 0; %Total coverage rate. We set the coverage rate as 90%

%Specifying the dimensions of the map. Dimensions are 5x5 with 100 grid
%points per row
xmin = 0; xmax = 5; ymin = 0; ymax = 5; ns = 100; %ns is the number of cells per row
%Time step
dt = 0.01;

%Dividing the map into cells with equal distance. 
xpoints = linspace(xmin,xmax,ns); 
ypoints = linspace(ymin,ymax,ns);

%Cell visit map is a map that tracks the number of cells
%The Robot has visited. We set it to zero as the Robot has not started to move yet
cell_visit_map = zeros(length(ypoints),length(xpoints)); prev_cells_covered = 0;

%Index number for storing the x and y coordinates of the Robot's
%trajectory
m_storage = 1; n_storage = 0;

%At every iteration, the robot travells for 1 second. Since the step size
%is 0.01 then 1/0.01 = 101 points. nn is the number of trajectory points per second
nn = 101;

%We set the time count to be zero
time_count = 0;
tic;
while tot_covrate < 0.9
%To mirrormap robot coordinates that are out of bounds, the dynamical equation 
%must be discretized. Therefore, 4th order Runge Kutta method was used to
%discretize the dynamical system 

    for i = 1:nn-1
        [IC_vec(i+1,1),IC_vec(i+1,2),IC_vec(i+1,3),IC_vec(i+1,4),IC_vec(i+1,5)] = ...
            lorenz_RK4(sigma,beta,r,v,IC_vec(i,1),IC_vec(i,2),IC_vec(i,3),IC_vec(i,4),IC_vec(i,5),dt);
        
        [IC_vec(end,4),IC_vec(end,5)] = mirrormap(IC_vec(end,4),IC_vec(end,5),xmin,xmax,ymin,ymax);
        
        %time count iterates at step size of dt
        time_count = time_count + dt;
        
        %All of the coordinates must not have NaN values in case x, y or z
        %heads to +/- Inf
        if isnan(IC_vec(i,:))
            error('Cannot have NaN values')
        end
            
    end
    
    %If the Robot's coordinates is still out of bounds after using the Mirror
    %Map technique, the code will stop executing. Robot's coordinates must
    %lie within bounds
   for i = 1:length(IC_vec)
        if IC_vec(i,4) < 0 || IC_vec(i,5) < 0 || IC_vec(i,4) > 20 || IC_vec(i,5) > 20
          error('Coordinates are out of bounds')
        end
    end
    
    n_storage = n_storage + length(IC_vec);
    %At every iteration, new values in pX and pY will replace old values.
    %Therefore we are storing old values in a storage vector
    pX_storage(m_storage:n_storage) = IC_vec(:,4); pY_storage(m_storage:n_storage) = IC_vec(:,5);
    
    %Index will change according to the length of the vector containing the
    %new values 
    m_storage = n_storage+1; 
    
    %The function cellmap will now calculate how many cells the robot has now
    %visited. Extra output arguments are increase coverage rate and new
    %cells covered
    [cell_visit_map inc_coverage_rate new_cells_covered] = cellmap(xpoints,ypoints,IC_vec(:,4),IC_vec(:,5),ns,dt,cell_visit_map);
    
    %We get the last trajectory points as our new IC for the next iteration
    IC_vec = [IC_vec(end,1),IC_vec(end,2),IC_vec(end,3),IC_vec(end,4),IC_vec(end,5)];
    %If the increase coverage rate is greater than the criterion then the
    %Robot will continue its path
    prev_cells_covered = prev_cells_covered + new_cells_covered;
    tot_covrate = prev_cells_covered/ns^2; %Calculates the total coverage rate
end
toc;
coverage_time = time_count;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1st plot- scatter map to denote on where the robot has traversed
figure(1)
g = scatter(pX_storage,pY_storage,'ko');
grid on
axis([0 5 0 5])
xlabel('x-coordinate'); ylabel('y-coordiante')
title('Scatter plot of the chaotic path by using Lorenz Equation ODE')
       
% 2nd plot- color map to denote the number of cells the Robot has visited
figure(2)
h = pcolor(xpoints,ypoints,cell_visit_map);
colorbar
colormap(lines(1000))
grid on
xlabel('x-coordinate');ylabel('y-coordinate')
title('Cell visit map showing number of times the robot has visited in a small cell')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%END OF CODE%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%