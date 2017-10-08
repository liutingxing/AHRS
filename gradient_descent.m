clear all;
clc;

addpath('.\quaternion_library');

ahrs_data = load('.\rotate.txt');

Time = ahrs_data(:, 1);                      % ( ms )
Roll = ahrs_data(:, 3);                      % ( degree )
Pitch = ahrs_data(:, 4);                     % ( degree )
Yaw = ahrs_data(:, 5);                       % ( degree )
Gyro = ahrs_data(:, 6:8) * pi / 180;         % ( rad/s )
Acc = ahrs_data(:, 9:11) * 9.8;              % ( m/s2 )
% Mag = ahrs_data(:, 12:14);                   % ( count )


%% variable prepare
G_vector = [0, 0, 1]'; % NED frame, Unit: g
N = length(Time);
yaw = zeros(N, 1);
pitch = zeros(N, 1);
roll = zeros(N, 1);
gyro_bias = zeros(3, 1);
acc_bias = zeros(3, 1);
Cnb = eye(3, 3);
Cbn = eye(3, 3);

gyroMeasError = 10*pi/180; % gyroscope measurement error in rad/s (shown as 5 deg/s)
beta = sqrt(3.0 / 4.0) * gyroMeasError; % compute beta

%% initial alignment
yaw_initial = 0;
pitch_initial = 0;
roll_initial = 0;

window_length = 100;
action_start = 0;
action_end = 0;
action_start_index = 0;
action_end_index = 0;

% g~ in body frame is [g_x, g_y, g_z], and g~b = Cnb*[0, 0, g]
g_x = -mean(Acc(1:window_length,1));
g_y = -mean(Acc(1:window_length,2));
g_z = -mean(Acc(1:window_length,3));
yaw_initial = 33.55*pi/180;
pitch_initial = -asin(g_x/9.8);
roll_initial = atan2(g_y/9.8, g_z/9.8);
q = euler2q(yaw_initial, pitch_initial, roll_initial);

% gyro calibration
gyro_bias = mean(Gyro(1:window_length, :))';

%% main loop
for i = 101 : N
    dt = 1/100; % 40Hz output rate

    % Compute rate of change of quaternion
    Cbn = q2dcm(q);
    Cnb = Cbn';
    
    Wepp = zeros(3, 1); % no latitude information in computing latitude and longitude rate
    Wiep = zeros(3, 1); % no latitude information in computing earth rate in the navigation frame
    Wipp = Wiep + Wepp;
    Wipb = Cnb * Wipp;
    Wpbb = Gyro(i, :)' - gyro_bias - Wipb;
    
    % Normalise accelerometer measurement
    g_measurement = -Acc(i, :)';
    g_measurement = g_measurement / norm(g_measurement);
    
    % Gradient decent algorithm corrective step
    F = [2*(q(2)*q(4) - q(1)*q(3)) - g_measurement(1)
        2*(q(1)*q(2) + q(3)*q(4)) - g_measurement(2)
        2*(0.5 - q(2)^2 - q(3)^2) - g_measurement(3)];
    J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
        2*q(2),     2*q(1),     2*q(4),	2*q(3)
        0,         -4*q(2),    -4*q(3),	0    ];
    step = (J'*F);
    step = step / norm(step);	% normalise step magnitude
    
    % Compute rate of change of quaternion
    qDot = 0.5 * quaternProd(q, [0; Wpbb])' - beta * step;

    % Integrate to yield quaternion
    q = q + qDot * dt;
    q = q / norm(q); % normalise quaternion
    
    Cbn = q2dcm(q);
    [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);
    
end
    
% yaw
figure;
plot(yaw*180/pi, 'r');
hold on;
plot(Yaw, 'b');
legend('yuewu', 'daoyuan');
grid on;
title('yaw comparison');
xlabel('sample point');
ylabel('yaw (degree)');

% pitch
figure;
plot(pitch*180/pi, 'r');
hold on;
plot(Pitch, 'b');
legend('yuewu', 'daoyuan');
grid on;
title('pitch comparison');
xlabel('sample point');
ylabel('pitch (degree)');

% roll
figure;
plot(roll*180/pi, 'r');
hold on;
plot(Roll, 'b');
legend('yuewu', 'daoyuan');
grid on;
title('roll comparison');
xlabel('sample point');
ylabel('roll (degree)');




























