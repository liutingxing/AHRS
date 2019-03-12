clear all;
clc;

sensor_data_Raw = load('.\sensorDataRaw.txt');
sensor_data_Fine = load('.\sensorDataFine.txt');

Time = sensor_data_Raw(:, 1);                      % ( ms )
GyroRaw = sensor_data_Raw(:, 2:4)*180/pi;          % ( degree/s )
GyroFine = sensor_data_Fine(:, 2:4)*180/pi;        % ( degree/s )
AccRaw = sensor_data_Raw(:, 5:7);                  % ( m/s2 )
AccFine = sensor_data_Fine(:, 5:7);                % ( m/s2 )

%% gyro compensate dispaly
figure;
plot(GyroFine(:, 1), ':c');
hold on;
plot(GyroRaw(:, 1), 'r', 'LineWidth',2);
title('gyro x compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('gyro (degree)');

figure;
plot(GyroFine(:, 2), ':c');
hold on;
plot(GyroRaw(:, 2), 'r');
title('gyro y compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('gyro (degree)');

figure;
plot(GyroFine(:, 3), ':c');
hold on;
plot(GyroRaw(:, 3), 'r');
title('gyro z compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('gyro (degree)');

%% acc compensate display
figure;
plot(AccFine(:, 1), ':c');
hold on;
plot(AccRaw(:, 1), 'r', 'LineWidth',2);
title('acc x compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('acc (m/s2)');

figure;
plot(AccFine(:, 2), ':c');
hold on;
plot(AccRaw(:, 2), 'r');
title('acc y compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('acc (m/s2)');

figure;
plot(AccFine(:, 3), ':c');
hold on;
plot(AccRaw(:, 3), 'r');
title('acc z compensate');
legend('fine', 'raw');
xlabel('sample point');
ylabel('acc (m/s2)');
