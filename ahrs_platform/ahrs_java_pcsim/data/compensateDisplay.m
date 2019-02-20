clear all;
clc;

sensor_data_Raw = load('.\sensorDataOld.txt');
sensor_data_Fine = load('.\sensorData.txt');

Time = sensor_data_Raw(:, 1);                      % ( ms )
GyroRaw = sensor_data_Raw(:, 2:4)*180/pi;          % ( degree/s )
GyroFine = sensor_data_Fine(:, 2:4)*180/pi;        % ( degree/s )

figure;
plot(GyroRaw(:, 1), 'r', 'LineWidth',2);
hold on;
plot(GyroFine(:, 1), ':c');
title('gyro x compensate');
legend('raw', 'fine');
xlabel('sample point');
ylabel('gyro (m/s2)');

figure;
plot(GyroRaw(:, 2), 'r');
hold on;
plot(GyroFine(:, 2), ':c');
title('gyro y compensate');
legend('raw', 'fine');
xlabel('sample point');
ylabel('gyro (m/s2)');

figure;
plot(GyroRaw(:, 3), 'r');
hold on;
plot(GyroFine(:, 3), ':c');
title('gyro z compensate');
legend('raw', 'fine');
xlabel('sample point');
ylabel('gyro (m/s2)');
