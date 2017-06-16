clear all;
clc;

%% read data from file

ahrs_data = load('.\output.txt');

time = ahrs_data(:, 1);                      % ( ms )
yaw = ahrs_data(:, 2);                       % ( rad )
pitch = ahrs_data(:, 3);                     % ( rad )
roll = ahrs_data(:, 4);                      % ( rad )


%% display result

% yaw
figure;
plot(yaw*180/pi, 'r');
xlabel('sample point');
ylabel('yaw (degree)');

% pitch
figure;
plot(pitch*180/pi, 'r');
xlabel('sample point');
ylabel('pitch (degree)');

% roll
figure;
plot(roll*180/pi, 'r');
xlabel('sample point');
ylabel('roll (degree)');
