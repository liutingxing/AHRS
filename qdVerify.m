clear all;
clc;

%% read data from file

ahrs_data = load('.\data\actionData.txt');

Time = ahrs_data(:, 1);                      % ( ms )
Roll = ahrs_data(:, 3);                      % ( degree )
Pitch = ahrs_data(:, 4);                     % ( degree )
Yaw = ahrs_data(:, 5);                       % ( degree )
Gyro = ahrs_data(:, 6:8);% * pi / 180;         % ( rad/s )
Acc = ahrs_data(:, 9:11);% * 9.8;              % ( m/s2 )
Mag = ahrs_data(:, 12:14);                   % ( count )

%% variable prepare
SampleRate = 40;  % 40Hz sample rate
dt = 1 / SampleRate;
mag_bias = [44.1244, 46.2893, -209.2231];
for i = 1 : length(Mag)
    if Mag(i, 1) ~= 0 && Mag(i, 2) ~= 0 && Mag(i, 3) ~= 0
        Mag(i, :) = Mag(i, :) - mag_bias;
    end
end

%% initial alignment (1s)
N = SampleRate;
initial_alignment_flag = 0;
g_x = -mean(Acc(1:N,1));
g_y = -mean(Acc(1:N,2));
g_z = -mean(Acc(1:N,3));
m_x = 0;
m_y = 0;
m_z = 0;
m_count = 0;
for i = 1:N
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


%% start calculation loop
q = euler2q(yaw_initial, pitch_initial, roll_initial);
N = length(Time);
for i = 1 : N
    qDotError = 0;
    Cbn = q2dcm(q);
    
    acc_norm = norm(Acc(i, :));
    
    %% acc aiding
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
    
    %% mag aiding
    if Mag(i, 1) ~= 0 && Mag(i, 2) ~= 0 && Mag(i, 3) ~= 0
        mag_norm = norm(Mag(i, :));
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
            gyroMeasError = 100*pi/180;
            beta = sqrt(3.0 / 4.0) * gyroMeasError;
        else
            gyroMeasError = 100*pi/180;
            beta = sqrt(3.0 / 4.0) * gyroMeasError;
        end
        step = (J'*F);
        qDotError = qDotError + step;
    end
    
    % normalise q dot error
    if qDotError ~= 0
        qDotError = qDotError / norm(qDotError);
    end
    
    %% Compute rate of change of quaternion
    Cbn = q2dcm(q);
    Cnb = Cbn';

    Wepp = zeros(3, 1); % no latitude information in computing latitude and longitude rate
    Wiep = zeros(3, 1); % no latitude information in computing earth rate in the navigation frame
    Wipp = Wiep + Wepp;
    Wipb = Cnb * Wipp;
    Wpbb = Gyro(i, :)' - Wipb;

    % Compute rate of change of quaternion
    qDot = 0.5 * quaternProd(q, [0; Wpbb])' - beta * qDotError;

    % Integrate to yield quaternion
    q = q + qDot * dt;
    q = q / norm(q); % normalise quaternion
    
    [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);
end

% yaw
figure;
plot(yaw*180/pi, 'r');
hold on;
plot(Yaw, 'b');
legend('yuewu', 'daoyuan');
title('yaw comparison');
xlabel('sample point');
ylabel('yaw (degree)');

% pitch
figure;
plot(pitch*180/pi, 'r');
hold on;
plot(Pitch, 'b');
legend('yuewu', 'daoyuan');
title('pitch comparison');
xlabel('sample point');
ylabel('pitch (degree)');

% roll
figure;
plot(roll*180/pi, 'r');
hold on;
plot(Roll, 'b');
legend('yuewu', 'daoyuan');
title('roll comparison');
xlabel('sample point');
ylabel('roll (degree)');
















