clear all;
clc;

sensor_data = load('.\sensorData.txt');

Time = sensor_data(:, 1);                      % ( ms )
Gyro = sensor_data(:, 2:4);                    % ( rad/s )
Acc = sensor_data(:, 5:7);                     % ( m/s2 )
Mag = sensor_data(:, 8:10);                    % ( uT )

%% constant value
cut_off_value = 1100;
sample_num = 2;

%% display real gyroZ
GyroZ = Gyro(:,3)*180/pi;
figure
plot(GyroZ, 'b');
title('gyro z real data');
hold on;
xlim = get(gca,'Xlim');
plot(xlim, [cut_off_value, cut_off_value], 'g');

%% spline simulation

curve = GyroZ;
[value, index] = max(curve);

% determin the sample point
for i = index:-1:1
    if (curve(i) < cut_off_value)
        x_left_index = i;
        break
    end
end
x_left = x_left_index-sample_num:x_left_index;

for i = index : length(curve)
    if (curve(i) < cut_off_value)
        x_right_index = i;
        break
    end
end
x_right = x_right_index:x_right_index+sample_num;

% interpolation
x = [x_left x_right];
y = curve(x);
xx = x_left_index:x_right_index;
yy = spline(x,y,xx);
plot(x,y,'r o',xx,yy,'r');
legend('real data', 'cut off', 'sample data', 'spline data');

%% filter the real data and interpolation data
[b, a] = butter(2, 4/(100/2), 'low');
GyroFiltered = filter(b, a, GyroZ);
GyroInterpolation = [GyroZ(1:x_left_index-1)', yy, GyroZ(x_right_index+1:length(GyroZ))'];
GyroInterpolationFiltered = filter(b, a, GyroInterpolation);

figure;
plot(GyroFiltered, 'r');
title('data comparison');
hold on;
plot(GyroInterpolationFiltered, 'b');
legend('real data', 'spline data')






