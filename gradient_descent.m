clear all;
clc;

MAG_SUPPORT = 1;

addpath('.\quaternion_library');

ahrs_data = load('.\data\actionData.txt');

Time = ahrs_data(:, 1);                      % ( ms )
Roll = ahrs_data(:, 3);                      % ( degree )
Pitch = ahrs_data(:, 4);                     % ( degree )
Yaw = ahrs_data(:, 5);                       % ( degree )
Gyro = ahrs_data(:, 6:8) * pi / 180;         % ( rad/s )
Acc = ahrs_data(:, 9:11) * 9.8;              % ( m/s2 )
if MAG_SUPPORT
Mag = ahrs_data(:, 12:14);                   % ( count )
end


%% variable prepare
SampleRate = 40;  % 40Hz sample rate
dt = 1 / SampleRate;
G_vector = [0, 0, 9.8]'; % NED frame, Unit: m/s2
N = length(Time);
gyro_bias = zeros(3, 1);
gyro_bias_array = zeros(N, 3);
acc_bias = zeros(3, 1);
yaw = zeros(N, 1);
pitch = zeros(N, 1);
roll = zeros(N, 1);
vN = zeros(N, 1);
vE = zeros(N, 1);
vD = zeros(N, 1);
pN = zeros(N, 1);
pE = zeros(N, 1);
pD = zeros(N, 1);
acc_liner_p = zeros(N, 3);

gyroMeasError = 10*pi/180; % gyroscope measurement error in rad/s (shown as 10 deg/s)
gyroMeasDrift = 0.2*pi/180; % gyroscope measurement error in rad/s/s (shown as 0.2 deg/s/s)
beta = sqrt(3.0 / 4.0) * gyroMeasError; % compute beta
zeta = sqrt(3.0 / 4.0) * gyroMeasDrift; % compute zeta

peace = 0;
step1 = 1;
step2 = 2;
step3 = 3;
curve_condition = 0;
down_time = 0;
action_time = 0;

action_count = 0;

platform_omega_Zmax = 0;
platform_omega_Zmin = 0;

%% low pass filter for acc
[b, a] = butter(2, 5/25, 'low');
% Acc = filter(b, a, Acc);

%% initial alignment
window_length = 100;
action_start = 0;
action_end = 0;
action_start_index = 0;
action_end_index = 0;

% g~ in body frame is [g_x, g_y, g_z], and g~b = Cnb*[0, 0, g]
g_x = -mean(Acc(1:window_length,1));
g_y = -mean(Acc(1:window_length,2));
g_z = -mean(Acc(1:window_length,3));
if MAG_SUPPORT
    m_x = 0;
    m_y = 0;
    m_z = 0;
    m_count = 0;
    for i = 1:window_length
        if Mag(i, 1) ~= 0 && Mag(i, 2) ~= 0 && Mag(i, 3) ~= 0
            m_count = m_count + 1;
            m_x = m_x + Mag(i, 1);
            m_y = m_y + Mag(i, 2);
            m_z = m_z + Mag(i, 3);
        end
    end
    m_x = m_x / m_count;
    m_y = m_y / m_count;
    m_z = m_z / m_count;
    Cnb_initial = ecompass_ned([g_x, g_y, g_z], [m_x, m_y, m_z]);
    Cbn_initial = Cnb_initial';
    [yaw_initial, pitch_initial, roll_initial] = dcm2euler(Cbn_initial);
    Cnp = euler2dcm(yaw_initial, 0, 0)'; % reverse the NED frame to platform frame(yaw_initial = 0)
else
    yaw_initial = 0*pi/180;
    pitch_initial = -asin(g_x/9.8);
    roll_initial = atan2(g_y/9.8, g_z/9.8);
end
q = euler2q(yaw_initial, pitch_initial, roll_initial);

% gyro calibration
gyro_bias = mean(Gyro(1:window_length, :))';
gyro_bias_error = 0;
validation_count = 0;
validation_num = SampleRate * 10; % 10s data size
gyro_bias_error_array = zeros(validation_num, 3);

for i = 101 : N
%% AHRS process
    qDotError = 0;
    Cbn = q2dcm(q);

    if action_start ~= 1 % acc mag aiding
        acc_norm = norm(Acc(i, :));
        if acc_norm < 12
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

            % adjust beta
            diff = norm(F);
            if diff < 0.1
                gyroMeasError = 3*pi/180;
                beta = sqrt(3.0 / 4.0) * gyroMeasError;
            else
                gyroMeasError = 10*pi/180;
                beta = sqrt(3.0 / 4.0) * gyroMeasError;
            end
            step = (J'*F);
            qDotError = qDotError + step;
        end
        if MAG_SUPPORT
            mag_norm = norm(Mag(i, :));
            if mag_norm ~= 0
                % Normalise magnetometer measurement
                Magnetometer = Mag(i, :) / mag_norm;

                % Reference direction of Earth's magnetic feild
                h = Cbn*Magnetometer';
                b = [0, norm(h(1), h(2)), 0, h(3)];
                F = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - Magnetometer(1)
                    2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - Magnetometer(2)
                    2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - Magnetometer(3)];
                J = [-2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
                    -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
                    2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];
                diff = norm(F);
                if diff < 0.1
                    gyroMeasError = 10*pi/180;
                    beta = sqrt(3.0 / 4.0) * gyroMeasError;
                else
                    gyroMeasError = 100*pi/180;
                    beta = sqrt(3.0 / 4.0) * gyroMeasError;
                end
                step = (J'*F);
                qDotError = qDotError + step;
            end
        end

        % normalise q dot error
        if qDotError ~= 0
            qDotError = qDotError / norm(qDotError);
        end

        % estimate gyro bias
        if MAG_SUPPORT
    %         biasDot = 2 * quaternProd([q(1), -q(2), -q(3), -q(4)],  qDotError)';
    %         gyro_bias_error = zeta * biasDot(2:4) * dt;
    % 
    %         % gyro bias validation (Time Interval > 30s, Standard Deviation / Mean < 10%)
    %         validation_count = validation_count + 1;
    %         if validation_count <= validation_num
    %             gyro_bias_error_array(validation_count, :) = gyro_bias_error;
    %         else
    %             for j = 2:validation_num
    %                 gyro_bias_error_array(j - 1, :) = gyro_bias_error_array(j, :);
    %             end
    %             gyro_bias_error_array(validation_num, :) = gyro_bias_error;
    %             gyro_bias_error_mean = mean(gyro_bias_error_array);
    %             gyro_bias_error_std = std(gyro_bias_error_array);
    %             if gyro_bias_error_std < 0.2*pi/180
    %                 gyro_bias = gyro_bias + gyro_bias_error_mean';
    %                 validation_count = 0;
    %             end
    %         end

        end

        gyro_bias_array(i, :) = gyro_bias';

        % Compute rate of change of quaternion
        Cbn = q2dcm(q);
        Cnb = Cbn';

        Wepp = zeros(3, 1); % no latitude information in computing latitude and longitude rate
        Wiep = zeros(3, 1); % no latitude information in computing earth rate in the navigation frame
        Wipp = Wiep + Wepp;
        Wipb = Cnb * Wipp;
        Wpbb = Gyro(i, :)' - Wipb - gyro_bias;

        % Compute rate of change of quaternion
        qDot = 0.5 * quaternProd(q, [0; Wpbb])' - beta * qDotError;

        % Integrate to yield quaternion
        q = q + qDot * dt;
        q = q / norm(q); % normalise quaternion
    else % gyro integrate
        Cnb = Cbn';

        Wepp = zeros(3, 1); % no latitude information in computing latitude and longitude rate
        Wiep = zeros(3, 1); % no latitude information in computing earth rate in the navigation frame
        Wipp = Wiep + Wepp;
        Wipb = Cnb * Wipp;
        Wpbb = Gyro(i, :)' - gyro_bias - Wipb;

        dq = zeros(4, 1);
        dq(1) = -(Wpbb(1)*q(2) + Wpbb(2)*q(3) + Wpbb(3)*q(4))/2;
        dq(2) = (Wpbb(1)*q(1) + Wpbb(3)*q(3) - Wpbb(2)*q(4))/2;
        dq(3) = (Wpbb(2)*q(1) - Wpbb(3)*q(2) + Wpbb(1)*q(4))/2;
        dq(4) = (Wpbb(3)*q(1) + Wpbb(2)*q(2) - Wpbb(1)*q(3))/2;

        q = q + dq*dt;
        q = q_norm(q); 
    end
    
    % output euler angle respect to the plarform
    Cbn = q2dcm(q);
    if MAG_SUPPORT
        Cbn = Cnp*Cbn;
    end
    [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);

%% platform omega
    platform_omega(i, :) = Cbn * Wpbb;

%% ins mechanization
    f_b = Acc(i, :)' - acc_bias;
    f_p = Cbn*f_b;
    
    % static constrain
    for j = 1:3
        if abs(f_p(j)) < 1
            f_p(j) = 0;
        end
    end
    
    % use liner acc to detect action start and end
    liner_acc_x = f_p(1);
    switch curve_condition
        case peace
            if liner_acc_x > 5
                action_start = 1;
                action_start_index = i;
                curve_condition = step1;
                action_time = 0;
            end

        case step1
            action_time = action_time + dt;
            if liner_acc_x > liner_acc_x_last
                slop = 1;
            else
                slop = -1;
                % reach the up peak
                if liner_acc_x_last < 12
                    % false peak
                    curve_condition = peace;
                    action_start = 0;
                else
                    curve_condition = step2;
                    down_time = 0;
                end
            end

        case step2
            down_time = down_time + dt;
            action_time = action_time + dt;
            if down_time > 0.2
                % timeout for trough
                curve_condition = peace;
                action_start = 0;
            end
            if liner_acc_x > liner_acc_x_last
                slop = 1;
                % reach the trough
                if liner_acc_x_last > -12
                    % false trough
                else
                    curve_condition = step3;
                end
            else
                slop = -1;
            end

        case step3
            action_time = action_time + dt;
            if liner_acc_x > liner_acc_x_last
                slop = 1;
            else
                slop = -1;
            end
            if liner_acc_x > -10 && liner_acc_x < 10
                if action_time > 0.1 && action_time < 0.4
                    action_end = 1;
                    action_end_index = i - 1;
                else
                    action_start = 0;
                end
                curve_condition = peace;
            end
    end
    liner_acc_x_last = liner_acc_x;
    
    if action_end == 1
        action_count = action_count + 1;
        action_array(action_count, :) = [action_start_index, action_end_index];
        action_end = 0;
        action_start = 0;
    end
    
    % ins
    acc_liner_p(i, :) = f_p - cross((2*Wiep+Wepp), [vN(i); vE(i); vD(i)]) + G_vector;
    if action_start == 1 && action_end == 0;
        if platform_omega(i, 3) > platform_omega_Zmax
            platform_omega_Zmax = platform_omega(i, 3);
        end
        
        if platform_omega(i, 3) < platform_omega_Zmin
            platform_omega_Zmin = platform_omega(i, 3);
        end
        
        vN(i) = vN(i-1) + (acc_liner_p(i, 1) + acc_liner_p(i-1, 1))*dt/2;
        vE(i) = vE(i-1) + (acc_liner_p(i, 2) + acc_liner_p(i-1, 2))*dt/2;
        vD(i) = vD(i-1) + (acc_liner_p(i, 3) + acc_liner_p(i-1, 3))*dt/2;
        pN(i) = pN(i-1) + (vN(i) + vN(i-1))*dt/2;
        pE(i) = pE(i-1) + (vE(i) + vE(i-1))*dt/2;
        pD(i) = pD(i-1) + (vD(i) + vD(i-1))*dt/2;
    end
end
    
%% display result
% acc measurement
if 0
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
if 0
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

% platform omega
if 0
figure;
% plot(platform_omega(:, 1), 'r');
% hold on;
% plot(platform_omega(:, 2), 'g');
plot(platform_omega(:, 3), 'b');
title('platform omega');
legend('x', 'y', 'z');
xlabel('sample point');
ylabel('omega (rad/s)');
end

% liner accelerate
if 1
figure;
plot(acc_liner_p(:, 1), 'r');
hold on;
% plot(acc_liner_p(:, 2), 'g');
% plot(acc_liner_p(:, 3), 'b');
legend('x', 'y', 'z');
title('liner acc');
end

% yaw
if 1
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
end

% position
if 0
for i = 1:action_count
    figure;
    action_start_index = action_array(i, 1);
    action_end_index = action_array(i, 2);
    plot3(pN(action_start_index:action_end_index), pE(action_start_index:action_end_index), pD(action_start_index:action_end_index), 'r', 'linewidth', 3);
    title(['position', num2str(i)]);
    box on;
    grid;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal; 
end
end

if 0
figure;
plot(gyro_bias_array(:, 1)*180/pi, 'r');
hold on;
plot(gyro_bias_array(:, 2)*180/pi, 'g');
plot(gyro_bias_array(:, 3)*180/pi, 'b');
xlabel('sample point');
ylabel('gyro bias (degree/s)');
legend('x', 'y', 'z');
title('gyro bias');
end

% velocity
if 0
figure;
plot(vN(action_start_index:action_end_index), 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('X velocity');

figure;
plot(vE(action_start_index:action_end_index), 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('Y velocity');

figure;
plot(vD(action_start_index:action_end_index), 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('Z velocity');
end
























