
% Maritime engineering LAB 2 Acceleration 
% read data from log file, extract data and post process to calculate the
% acceleration velocity and distance

% to be placed in the same folder as Aufzug.txt file
close all
clc
clear all
dat =dlmread('Aufzug.txt');                 % reads contents of LOG file
             
figure
plot(dat)                                   % plots raw data
title('Raw acceleration Data from SD card')
xlabel ({'time (s)'})
ylabel ({'Digital Sensor Values '})

                                            % A section of graph taken for
sens(1,:) = dat(160:260);                   % post processing
sens(2,:) = dat(320:420);
sens(3,:) = dat(540:640);
sens(4,:) = dat(730:830);
sens(5,:) = dat(990:1090);
sens(6,:) = dat(1154:1254);
distance =[0,0,0,0,0,0].';

for n = 1:6
    
    for k = 1:length(sens(n,:))
    acc(n,k) = 0.0018*sens(n,k)-4.9968;     % Acceleration is calculated
    end                                     % k0 = 0.0018 , k1 = -4.9968

                                            % Dynamic Acceleration is
    for k = 1:length(acc(n,:))              % calculated by subtracting 9.8    
    acc_corr(n,k) = acc(n,k) - 9.8;
    end

    vel(n,:) = cumtrapz(acc_corr(n,:));     % velocity calculation
    vel(n,:) =vel(n,:)*0.1;                 % calculated velocity divided

    

    m = (vel(n,101)-vel(n,2))/(100-1);      % Integration error slope 
    x = 2:101;                              % is calculated and remmoved
    y = m*x;                                % from velocity to get
    vel2(n,:)= vel(n,2:101)-y;              % correct velocity




    dist(n,:) =cumtrapz(vel2(n,:));         % Distance calculation
    dist(n,:) =dist(n,:) *0.1;              % by 0.1, as sampling frequency     
                                            % was 10Hz, and plotted.

    distance(n,:) = abs(trapz(vel2(n,:))*0.1)
end

figure
plot(acc_corr(1,:))                         % Acceleration plot
title('Acceleration')
xlabel ({'time (s)'})
ylabel ({'Acceleration in (m/s/s)'}) 

figure                                 
plot(vel(1,:))                         
title('Velocity')
xlabel ({'time (s)'})
ylabel ({'velocity (m/s)'})

figure
plot(vel2(1,:))                             % velocity without error plot
title('Corrected Velocity')
xlabel ({'time (s)'})
ylabel ({'velocity (m/s)'})

figure
plot(dist(1,:))
title('Distance')
xlabel ({'time (s)'})
ylabel ({'distance (m)'})

average =mean(distance)
standard_deviation = std(distance)
display('thank you for using Acceleration.m') 
%End Of Code