clear all;
clc;

sensor_data = load('.\sensorData.txt');

Time = sensor_data(:, 1);                      % ( ms )
Gyro = sensor_data(:, 2:4);                    % ( rad/s )
Acc = sensor_data(:, 5:7);                     % ( m/s2 )
Mag = sensor_data(:, 8:10);                    % ( uT )

%% display result
% acc measurement
if 1
figure
plot(Acc(:, 1), 'r');
hold on;
plot(Acc(:, 2), 'g');
plot(Acc(:, 3), 'b');
title('acc measurement');
legend('x', 'y', 'z');
xlabel('sample point');
ylabel('acc (m/s2)');
end

% gyro measurement
if 1
figure
plot(Gyro(:, 1)*180/pi, 'r');
hold on;
plot(Gyro(:, 2)*180/pi, 'g');
plot(Gyro(:, 3)*180/pi, 'b');
title('gyro measurement');
legend('x', 'y', 'z');
xlabel('sample point');
ylabel('gyro (degree/s)');
end