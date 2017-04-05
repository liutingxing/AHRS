clear all;
close all;
clc;

%% read data from file

ahrs_data = load('.\drift.txt');

Time = ahrs_data(:, 1);
Roll = ahrs_data(:, 3);
Pitch = ahrs_data(:, 4);
Yaw = ahrs_data(:, 5);
Gyro = ahrs_data(:, 6:8);               % x, y, z
Acc_liner = ahrs_data(:, 9:11) * 9.8;   % x , y, z liner acceleration ( m/s2 )
Mag = ahrs_data(:, 12:14);              % x , y, z

%% start calculation loop
N = length(Time);
Cnb = eye(3, 3);
Cbn = eye(3, 3);
Acc_liner_n = zeros(N, 3);
Pos_X = zeros(N, 1);
Pos_Y = zeros(N, 1);
Pos_Z = zeros(N, 1);
Vel_X = zeros(N, 1);
Vel_Y = zeros(N, 1);
Vel_Z = zeros(N, 1);

for i = 1 : N
    Yaw(i) = 0; % !!just for display easily
    %% DCM (ned)
    Cnb(1, 1) = cos(Yaw(i)) * cos(Pitch(i));
    Cnb(1, 2) = sin(Yaw(i)) * cos(Pitch(i));
    Cnb(1, 3) = -sin(Pitch(i));
    Cnb(2, 1) = -cos(Roll(i)) * sin(Yaw(i)) + sin(Roll(i)) * sin(Pitch(i)) * cos(Yaw(i));
    Cnb(2, 2) = cos(Roll(i)) * cos(Yaw(i)) + sin(Roll(i)) * sin(Pitch(i)) * sin(Yaw(i));
    Cnb(2, 3) = sin(Roll(i)) * cos(Pitch(i));
    Cnb(3, 1) = sin(Roll(i)) * sin(Yaw(i)) + cos(Roll(i)) * sin(Pitch(i)) * cos(Yaw(i));
    Cnb(3, 2) = -sin(Roll(i)) * cos(Yaw(i)) + cos(Roll(i)) * sin(Pitch(i)) * sin(Yaw(i));
    Cnb(3, 3) = cos(Roll(i)) * cos(Pitch(i));
    Cbn = Cnb';
    
    %% Acceleration on navigation frame
    Acc_liner_n(i, :) = (Cbn * Acc_liner(i, :)')';
    
    %% Velocity on navigation frame
    if i > 1
        dt = 10 / 1000;
        Vel_X(i) = Vel_X(i-1) + (Acc_liner_n(i, 1) + Acc_liner_n(i-1, 1))*dt/2;
        Vel_Y(i) = Vel_Y(i-1) + (Acc_liner_n(i, 2) + Acc_liner_n(i-1, 2))*dt/2;
        Vel_Z(i) = Vel_Z(i-1) + (Acc_liner_n(i, 3) + Acc_liner_n(i-1, 3))*dt/2;
    end
    
    %% Position on navigation frame
    if i > 2
        dt = 10 / 1000;
        Pos_X(i) = Pos_X(i-1) + (Vel_X(i) + Vel_X(i-1))*dt/2;
        Pos_Y(i) = Pos_Y(i-1) + (Vel_Y(i) + Vel_Y(i-1))*dt/2;
        Pos_Z(i) = Pos_Z(i-1) + (Vel_Z(i) + Vel_Z(i-1))*dt/2;
    end
end

%% display result
figure;
plot(Acc_liner_n(:, 3), 'r');
legend('Acc\_liner\_D');
figure;
plot(Acc_liner_n(:, 2), 'g');
legend('Acc\_liner\_E');
figure;
plot(Acc_liner_n(:, 1), 'b');
legend('Acc\_liner\_N');

figure;
plot(Pos_X, Pos_Y, 'r');

figure;
plot(Pos_X, Pos_Z, 'g');



