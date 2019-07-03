clc;clear all;close all;
%Written by Karan Sridharan
%Obtains the average ripple velocity 

density_vec = xlsread('density_rice grain ellipsoid_mu0.1_rundp0.5hires.xlsx','A2:D502');
density_time = density_vec(:,1);
density_A = density_vec(:,2);
density_D = density_vec(:,3);
density_E = density_vec(:,4);
% 
figure(1)
subplot(3,1,1)
plot(density_time, density_A)
grid on
xlabel('time')
ylabel('density')
title('Density vs time for density A')

subplot(3,1,2)
plot(density_time, density_D)
grid on
xlabel('time')
ylabel('density')
title('Density vs time for density D')

subplot(3,1,3)
plot(density_time, density_E)
grid on
xlabel('time')
ylabel('density')
title('Density vs time for density E')
%% 
k = find(density_time>=1.5,1);
l = find(density_time==5.0,1);
density_time_filt = density_time(k:l);
density_A_filt = detrend(density_A(k:l));
density_D_filt = detrend(density_D(k:l));
density_E_filt = detrend(density_E(k:l));
% 
figure(2)
subplot(3,1,1)
plot(density_time_filt, density_A_filt)
grid on
xlabel('Time')
ylabel('Porosity')
title('Filtered porosity vs time at probe A')

subplot(3,1,2)
plot(density_time_filt, density_D_filt)
grid on
xlabel('Time')
ylabel('Porosity')
title('Filtered porosity vs time at probe D')

subplot(3,1,3)
plot(density_time_filt, density_E_filt)
grid on
xlabel('Time')
ylabel('Porosity')
title('Filtered porosity vs time at probe E')

%% 
%in xcorr later time is fixed and earlier time is moving in order to get positive
%time lag
%xcorr(later time fixed,earlier time moving)
[r_AD, lag_AD] = xcorr(density_D_filt, density_A_filt,'none');
[r_ED, lag_ED] = xcorr(density_D_filt, density_E_filt,'none');
[r_EA, lag_EA] = xcorr(density_A_filt, density_E_filt,'none');
timeDiff_AD = [];
timeDiff_ED = [];
timeDiff_EA = [];
for i = 1:length(lag_AD)
    timeDiff_AD(i) = lag_AD(i)/100;
end

for i = 1:length(lag_ED)
    timeDiff_ED(i) = lag_ED(i)/100;
end

for i = 1:length(lag_EA)
    timeDiff_EA(i) = lag_EA(i)/100;
end
%obtaining index values from 0 to positive time difference values
k_1 = find(timeDiff_AD == 0);
l_1 = find(timeDiff_ED == 0);
m_1 = find(timeDiff_EA == 0);

%obtaining time difference and correlation values from 0 to positive lag values
r_AD_filt = r_AD(k_1:length(r_AD));
timeDiff_AD_filt = timeDiff_AD(k_1:length(timeDiff_AD));

r_ED_filt = r_ED(l_1:length(r_ED));
timeDiff_ED_filt = timeDiff_ED(l_1:length(timeDiff_ED));

r_EA_filt = r_EA(m_1:length(r_EA));
timeDiff_EA_filt = timeDiff_EA(m_1:length(timeDiff_EA));

%plotting time difference and correlation values
figure(4)
subplot(3,1,1)
plot(timeDiff_AD_filt,r_AD_filt)
grid on
xlabel('time difference')
ylabel('cross correlation factor')
title('Cross correlation between probes A and D')

subplot(3,1,2)
plot(timeDiff_ED_filt,r_ED_filt)
grid on
xlabel('time difference')
ylabel('cross correlation factor')
title('Cross correlation between probes D and E')

subplot(3,1,3)
plot(timeDiff_EA_filt,r_EA_filt)
grid on
xlabel('time difference')
ylabel('cross correlation factor')
title('Cross correlation between probes A and E')

%obtaining time difference corresponding to max correlation value
findmax_AD = find(r_AD_filt == max(r_AD_filt(2:length(r_AD_filt))));
findmax_ED = find(r_ED_filt == max(r_ED_filt(2:length(r_ED_filt))));
findmax_EA = find(r_EA_filt == max(r_EA_filt(2:length(r_EA_filt))));

%obtaining the velocity of ripples from different probe points
velocity_AD = 0.04/timeDiff_AD_filt(findmax_AD)
velocity_ED = 0.08/timeDiff_ED_filt(findmax_ED)
velocity_EA = 0.04/timeDiff_EA_filt(findmax_EA)





