clear all;
clc;

sensor_data = load('.\sensorData.txt');
extra_data = load('.\extData.txt');

Time = sensor_data(:, 1);                      % ( ms )
Gyro = sensor_data(:, 2:4);                    % ( rad/s )
Acc = sensor_data(:, 5:7);                     % ( m/s2 )
Mag = sensor_data(:, 8:10);                    % ( uT )
LinerAccPlat = extra_data(:, 2:4);             % ( m/s2 )

%% display gyroZ and LinerAccX
if 1
figure
plot(Gyro(:, 3), 'r');
hold on;
plot(LinerAccPlat(:, 1), 'b');
title('action character data');
legend('gyroZ', 'linerAccX');
end