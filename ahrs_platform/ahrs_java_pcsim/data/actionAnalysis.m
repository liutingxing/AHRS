clear all;
clc;

sensor_data = load('.\sensorData.txt');
extra_data = load('.\extData.txt');

Time = sensor_data(:, 1);                      % ( ms )
Gyro = sensor_data(:, 2:4);                    % ( rad/s )
Acc = sensor_data(:, 5:7);                     % ( m/s2 )
Mag = sensor_data(:, 8:10);                    % ( uT )
LinerAccPlat = extra_data(:, 2:4);             % ( m/s2 )
QuaternionPlat = extra_data(:, 5:8);

for i = 1:length(LinerAccPlat)
    for j = 1:3
        if abs(LinerAccPlat(i,j)) < 1
            LinerAccPlat(i,j) = 0;
        end
    end
end

%% display gyroZ and filtered gyroZ

% 5Hz low pass filter. 100Hz sample rate
[b, a] = butter(2, 4/(100/2), 'low');
GyroFiltered = filter(b, a, Gyro(:, 3));

if 0
figure
plot(Gyro(:, 3)*180/pi, 'r');
hold on;
plot(GyroFiltered*180/pi, 'b');
title('gyroZ low pass filter');
legend('raw data', 'filtered data');
end

%% display filtered gyroZ and LinerAccX
if 1
figure
plot(GyroFiltered*10, 'r');
hold on;
plot(LinerAccPlat(:, 1), 'b');
title('action character data');
legend('gyroZ', 'linerAccX');
end

%% calulate the trajectory
ActionStart = 1985;
ActionEnd = 2016;
dt = 1 / 100;

LinerAccX = 0;
LinerAccY = 0;
LinerAccZ = 0;
VelX = 0;
VelY = 0;
VelZ = 0;
PosX = 0;
PosY = 0;
PosZ = 0;

for i = ActionStart:ActionEnd
    
    linerAccAveX = (LinerAccPlat(i, 1) + LinerAccX) / 2;
    linerAccAveY = (LinerAccPlat(i, 2) + LinerAccY) / 2;
    linerAccAveZ = (LinerAccPlat(i, 3) + LinerAccZ) / 2;
    
    velPlatX = VelX + linerAccAveX * dt;
    velPlatY = VelY + linerAccAveY * dt;
    velPlatZ = VelZ + linerAccAveZ * dt;
    velAveX = (VelX + velPlatX) / 2;
    velAveY = (VelY + velPlatY) / 2;
    velAveZ = (VelZ + velPlatZ) / 2;
    
    PosX = PosX + velAveX * dt;
    PosY = PosY + velAveY * dt;
    PosZ = PosZ + velAveZ * dt;
    
    LinerAccX = LinerAccPlat(i, 1);
    LinerAccY = LinerAccPlat(i, 2);
    LinerAccZ = LinerAccPlat(i, 3);
    VelX = velPlatX;
    VelY = velPlatY;
    VelZ = velPlatZ;
    
    str = sprintf('%d %f %f %f %f %f %f %f x', i, PosX, -PosZ, -PosY, QuaternionPlat(i, 1), -QuaternionPlat(i, 2), -QuaternionPlat(i, 3), QuaternionPlat(i, 4));
    disp(str);
end


