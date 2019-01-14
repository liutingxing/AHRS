clear all;
clc;

sensor_data = load('.\sensorData.txt');

Time = sensor_data(:, 1);                      % ( ms )
Gyro = sensor_data(:, 2:4);                    % ( rad/s )
Acc = sensor_data(:, 5:7);                     % ( m/s2 )
Mag = sensor_data(:, 8:10);                    % ( uT )

%% constant value
cut_off_value = 900;
sample_num = 2;

%% display filtered gyroZ

% 5Hz low pass filter. 100Hz sample rate
[b, a] = butter(2, 4/(100/2), 'low');
GyroFiltered = filter(b, a, Gyro(:, 3));
figure
plot(GyroFiltered*180/pi, 'b');
title('gyro z filtered data');
hold on;
xlim = get(gca,'Xlim');
plot(xlim, [cut_off_value, cut_off_value], 'g');

%% spline simulation

curve = GyroFiltered*180/pi;
[value, index] = max(curve);

% determin the sample point
for i = index:-1:1
    if (curve(i) < cut_off_value)
        break
    end
end
x_left = i-sample_num:i;

for i = index : length(curve)
    if (curve(i) < cut_off_value)
        break
    end
end
x_right = i:i+sample_num;

% interpolation
x = [x_left x_right];
y = curve(x);
xx = x_left+1:x_right-1;
yy = spline(x,y,xx);
plot(x,y,'r o',xx,yy,'r');
legend('real data', 'cut off', 'sample data', 'spline data');



