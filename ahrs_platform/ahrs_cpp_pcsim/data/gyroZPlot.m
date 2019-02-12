clear all;
clc;

rawDataUn = load('gyroRawDataUn.txt', 'r');
rawDataEn = load('gyroRawDataEn.txt', 'r');
caliDataUn = load('gyroCaliDataUn.txt', 'r');
caliDataEn = load('gyroCaliDataEn.txt', 'r');

rawGyroZ = rawDataUn(:,4);
rawGyroZcompen = rawDataEn(:,4);
caliGyroZ = caliDataUn(:, 4);
caliGyroZcompen = caliDataEn(:, 4);

figure;
plot(rawGyroZ*180/pi, 'r');
hold on;
plot(rawGyroZcompen*180/pi, 'b');
legend('raw data uncompensate', 'raw data compensate');
title('raw gyro data');

figure;
plot(caliGyroZ*180/pi, 'r');
hold on;
plot(caliGyroZcompen*180/pi, 'b');
legend('filtered data uncompensate', 'filtered data compensate');
title('filtered gyro data');