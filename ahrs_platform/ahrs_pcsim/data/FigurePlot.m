clear all;
clc;

%% read data from file

ahrs_data = load('.\output.txt');
ref_data = load('.\rotate.txt');

time = ahrs_data(:, 1);                      % ( ms )
yaw = ahrs_data(:, 2);                       % ( rad )
pitch = ahrs_data(:, 3);                     % ( rad )
roll = ahrs_data(:, 4);                      % ( rad )
pos = ahrs_data(:, 8:10);                    % ( m )

RollRef = ref_data(:, 3);                    % ( degree )
PitchRef = ref_data(:, 4);                   % ( degree )
YawRef = ref_data(:, 5);                     % ( degree )


%% display result

% yaw
figure;
plot(yaw*180/pi, 'r');
hold on;
plot(YawRef, 'b');
legend('sensor', 'reference');
xlabel('sample point');
ylabel('yaw (degree)');

% pitch
figure;
plot(pitch*180/pi, 'r');
hold on;
plot(PitchRef, 'b');
legend('sensor', 'reference');
xlabel('sample point');
ylabel('pitch (degree)');

% roll
figure;
plot(roll*180/pi, 'r');
hold on;
plot(RollRef, 'b');
legend('sensor', 'reference');
xlabel('sample point');
ylabel('roll (degree)');

% position
figure;
plot3(pos(:,1), pos(:, 2), pos(:, 3), 'r', 'linewidth', 3);
title('position');
box on;
grid;
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
