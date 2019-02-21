clear all;
clc;

sensor_data_Raw = load('.\sensorDataOld.txt');
sensor_data_Fine = load('.\sensorData.txt');

Time = sensor_data_Raw(:, 1);                      % ( ms )
GyroRaw = sensor_data_Raw(:, 2:4)*180/pi;          % ( degree/s )
GyroFine = sensor_data_Fine(:, 2:4)*180/pi;        % ( degree/s )

figure;
plot(GyroFine(:, 1), ':c');
hold on;
plot(GyroRaw(:, 1), 'r', 'LineWidth',2);
title('gyro x compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('gyro (m/s2)');

figure;
plot(GyroFine(:, 2), ':c');
hold on;
plot(GyroRaw(:, 2), 'r');
title('gyro y compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('gyro (m/s2)');

figure;
plot(GyroFine(:, 3), ':c');
hold on;
plot(GyroRaw(:, 3), 'r');
title('gyro z compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('gyro (m/s2)');
