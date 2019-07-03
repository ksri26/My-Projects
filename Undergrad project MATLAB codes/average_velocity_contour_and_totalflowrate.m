clc; clear all; close all;
%Written by Karan Sridharan
%Obtains the average particle velocity and total flow rate using Weighted
%Residual technique

        %importing velocity data
part_vel_data = xlsread('particle_speed_5s.xlsx');
x_coord = part_vel_data(:,1);
y_coord = part_vel_data(:,2);
u = part_vel_data(:,4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %specifying number of points for creating a grid map containing average
        %particle velocities by using Shepard's interpolation
Nx = 20; Ny=10;
xpoints = linspace(min(x_coord),max(x_coord),Nx+1);
ypoints = linspace(min(y_coord),max(y_coord),Ny+1);
u_avg_interp = [];h = 0.01; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %generates a vector containing x and y coordinate at the centre of each cell
for i = 1:length(xpoints)-1
     xc_vec(i) = xpoints(i)+(xpoints(i+1)-xpoints(i))/2;
end

for i = 1:length(ypoints)-1
     yc_vec(i) = ypoints(i)+(ypoints(i+1)-ypoints(i))/2;
end
u_avg_interp = zeros(length(yc_vec),length(xc_vec));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at cell(1,1)
i = 1; %yc_vec index
j = 1; %xc_vec index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(end)&&y_coord(k)<=yc_vec(i)
        weight_func =  shepard(x_coord(k)*-1,y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
        
    elseif x_coord(k)<=xc_vec(j)&&y_coord(k)<=yc_vec(i)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at cell(end,1)
i = length(yc_vec); %yc_vec index
j = 1; %xc_vec index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(end)&&y_coord(k)>=yc_vec(i-1)
        weight_func =  shepard(x_coord(k)*-1,y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
       
    elseif x_coord(k)<=xc_vec(j)&&y_coord(k)>=yc_vec(i-1)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at cell(1,end)
i = 1; %yc_vec index
j = length(xc_vec); %xc_vec index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(j)&&y_coord(k)<=yc_vec(i)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
        
    elseif x_coord(k)<=xc_vec(1)&&y_coord(k)<=yc_vec(i)
        weight_func =  shepard(x_coord(k)*-1,y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at cell(end,end)
i = length(yc_vec); %yc_vec index
j = length(xc_vec); %xc_vec index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(j)&&y_coord(k)>=yc_vec(i-1)
        weight_func =  shepard(x_coord(k)*-1,y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
       
    elseif x_coord(k)<=xc_vec(1)&&y_coord(k)>=yc_vec(i-1)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at middle row left column
j = 1; %xc_vec index
for i = 2:length(yc_vec)-1 %yc_vec index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(end)&&y_coord(k)>=yc_vec(i-1)&&y_coord(k)<=yc_vec(i+1)
        weight_func =  shepard(x_coord(k)*-1,y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
        
    elseif x_coord(k)<=xc_vec(j)&&y_coord(k)>=yc_vec(i-1)&&y_coord(k)<=yc_vec(i+1)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at middle row right column
j = length(xc_vec); %xc_vec index
for i = 2:length(yc_vec)-1; %yc_vec index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(j)&&y_coord(k)>=yc_vec(i-1)&&y_coord(k)<=yc_vec(i+1)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
        
    elseif x_coord(k)<=xc_vec(1)&&y_coord(k)>=yc_vec(i-1)&&y_coord(k)<=yc_vec(i+1)
        weight_func =  shepard(x_coord(k)*-1,y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at middle column top row
i=1; %row index
for j = 2:length(xc_vec)-1; %column index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(j-1)&&x_coord(k)<=xc_vec(j+1)&&y_coord(k)<=yc_vec(i+1)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at middle column bottom row
i=length(yc_vec); %row index
for j = 2:length(xc_vec)-1; %column index
vc = 0; nc = 0;
for k = 1:length(x_coord)
    if x_coord(k)>=xc_vec(j-1)&&x_coord(k)<=xc_vec(j+1)&&y_coord(k)>=yc_vec(i-1)
        weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
        vc = vc + weight_func*u(k);
        nc = nc + weight_func;
    end
end
u_avg_interp(i,j) = vc/nc;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Interpolates average velocities at middle column middle row
for i = 2:length(yc_vec)-1; %row index
    for j = 2:length(xc_vec)-1; %column index
        vc = 0; nc = 0;
        for k = 1:length(x_coord)
            if x_coord(k)>=xc_vec(j-1)&&x_coord(k)<=xc_vec(j+1)&&y_coord(k)<=yc_vec(i+1)&&y_coord(k)>=yc_vec(i-1)
                weight_func =  shepard(x_coord(k),y_coord(k),xc_vec(j),yc_vec(i),h);
                vc = vc + weight_func*u(k);
                nc = nc + weight_func;
            end
        end
        u_avg_interp(i,j) = vc/nc;

    end
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting average velocity profile
u_avg = zeros(length(yc_vec),1);

for i = 1:length(yc_vec)
    u_sum = 0;u_count = 0;
    for j = 1:length(xc_vec)
        if ~isnan(u_avg_interp(i,j))
        u_sum = u_sum + u_avg_interp(i,j);
        u_count = u_count + 1;
        end
    end
    
    if u_sum ~= 0 && u_count ~= 0 
    u_avg(i,1) = u_sum/u_count;
    end
    
end

figure(1)
contourf(xc_vec,yc_vec,u_avg_interp)
xlabel('x(m)');ylabel('y(m)');title('Average velocity contour plot')
grid on
colorbar
axis([-0.05 0.05 -0.003 0.01])

figure(2)
plot(x_coord,y_coord,'ro')
grid on

% figure(2)
% plot(u_avg,yc_vec,'ro-')
% grid on
% xlabel('average velocity through x direction (m/s)');ylabel('y(m)');
% title('Average Velocity profile')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculates the total average flow rate of particles
Q_avg = (sum(u_avg)/length(u))*(0.1*0.1)





   
       