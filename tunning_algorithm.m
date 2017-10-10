clear all;
clc;

ahrs_data = load('.\data\actionData.txt');

Time = ahrs_data(:, 1);                      % ( ms )
Roll = ahrs_data(:, 3);                      % ( degree )
Pitch = ahrs_data(:, 4);                     % ( degree )
Yaw = ahrs_data(:, 5);                       % ( degree )
Gyro = ahrs_data(:, 6:8) * pi / 180;         % ( rad/s )
Acc = ahrs_data(:, 9:11) * 9.8;              % ( m/s2 )
% Mag = ahrs_data(:, 12:14);                   % ( count )

%% variable prepare
dt = 1/40; % 40Hz output rate
G_vector = [0, 0, 9.8]';
N = length(Time);
yaw = zeros(N, 1);
pitch = zeros(N, 1);
roll = zeros(N, 1);
gyro_bias = zeros(3, 1);
acc_bias = zeros(3, 1);
Cnb = eye(3, 3);
Cbn = eye(3, 3);
acc_liner_p = zeros(N, 3);
vN = zeros(N, 1);
vE = zeros(N, 1);
vD = zeros(N, 1);
pN = zeros(N, 1);
pE = zeros(N, 1);
pD = zeros(N, 1);

x = zeros(9, 1); % roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z, acc_bias_x, acc_bias_y, acc_bias_z
F = zeros(9, 9);
PHIM = zeros(9, 9);
qdt = zeros(9, 9);
Q = zeros(9, 9);
G = zeros(9, 9);
Corr_time_gyro = 100;
Corr_time_acc = 100;
sigma_Win = 1.0e-6;
sigma_acc = ((5.0e-4) * 9.78032667 * (5.0e-4) * 9.78032667);
sigma_gyro = (20.0 * pi / 180.0 / 3600 * 20.0 * pi / 180.0 / 3600);
sigma_phim_e_n = 1.0*pi/180;
sigma_phim_u = 1.0*pi/180;
sigma_phim_gyro = 1000*pi/180/3600;
sigma_phim_acc = 0.3;
P = eye(9, 9);
P(1, 1) = sigma_phim_e_n^2;
P(2, 2) = sigma_phim_e_n^2;
P(3, 3) = sigma_phim_u^2;
P(4, 4) = sigma_phim_gyro^2;
P(5, 5) = sigma_phim_gyro^2;
P(6, 6) = sigma_phim_gyro^2;
P(7, 7) = sigma_phim_acc^2;
P(8, 8) = sigma_phim_acc^2;
P(9, 9) = sigma_phim_acc^2;
kf_count = 1;

peace = 0;
step1 = 1;
step2 = 2;
step3 = 3;
curve_condition = 0;

action_count = 0;

platform_omega_Zmax = 0;
platform_omega_Zmin = 0;

fd = fopen('DataForPlatform.txt', 'w+');
fp = fopen('Angle.txt', 'w+');

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
pitch_initial = -asin(g_x/9.8);
roll_initial = atan2(g_y/9.8, g_z/9.8);
q = euler2q(yaw_initial, pitch_initial, roll_initial);

% gyro calibration
gyro_bias = mean(Gyro(1:window_length, :))';


for i = 101 : N
%% attitude integration process
    Cbn = q2dcm(q);
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
    
%% acc fuison for bias estimate and attitude correction
    if action_start ~= 1
        deltaT = dt*kf_count;
        Cbn = q2dcm(q);
        F(1, 4) = -Cbn(1, 1);
        F(1, 5) = -Cbn(1, 2);
        F(1, 6) = -Cbn(1, 3);
        F(2, 4) = -Cbn(2, 1);
        F(2, 5) = -Cbn(2, 2);
        F(2, 6) = -Cbn(2, 3);
        F(3, 4) = -Cbn(3, 1);
        F(3, 5) = -Cbn(3, 2);
        F(3, 6) = -Cbn(3, 3);
        F(4, 4) = -1/Corr_time_gyro;
        F(5, 5) = -1/Corr_time_gyro;
        F(6, 6) = -1/Corr_time_gyro;
        F(7, 7) = -1/Corr_time_acc;
        F(8, 8) = -1/Corr_time_acc;
        F(9, 9) = -1/Corr_time_acc;

        qdt(1, 1) = sigma_Win;
        qdt(2, 2) = sigma_Win;
        qdt(3, 3) = sigma_Win;
        qdt(4, 4) = sigma_gyro;
        qdt(5, 5) = sigma_gyro;
        qdt(6, 6) = sigma_gyro;
        qdt(7, 7) = sigma_acc;
        qdt(8, 8) = sigma_acc;
        qdt(9, 9) = sigma_acc;

        G(1, 1) = -Cbn(1, 1);
        G(1, 2) = -Cbn(1, 2);
        G(1, 3) = -Cbn(1, 3);
        G(2, 1) = -Cbn(2, 1);
        G(2, 2) = -Cbn(2, 2);
        G(2, 3) = -Cbn(2, 3);
        G(3, 1) = -Cbn(3, 1);
        G(3, 2) = -Cbn(3, 2);
        G(3, 3) = -Cbn(3, 3);
        G(4, 4) = 1;
        G(5, 5) = 1;
        G(6, 6) = 1;
        G(7, 7) = 1;
        G(8, 8) = 1;
        G(9, 9) = 1;

        % Q matrix discretization-2 order
        Q_basic = G*qdt*G';
        M1 = Q_basic;
        M2 = Q_basic*F'+F*Q_basic;
        Q = deltaT*M1 + 1/2*deltaT*deltaT*M2;

        % PHIM matrix discretization-2 order
        I = eye(9, 9);
        PHIM = I + deltaT*F + 1/2*deltaT*deltaT*F*F;
        
        % predict
        x = PHIM*x;
        P = PHIM*P*PHIM' + Q;
        
        % update from acc
        H = zeros(3, 9);
        H(1, 2) = G_vector(3);
        H(2, 1) = -G_vector(3);
        H(1, 7) = Cbn(1, 1);
        H(1, 8) = Cbn(1, 2);
        H(1, 9) = Cbn(1, 3);
        H(2, 7) = Cbn(2, 1);
        H(2, 8) = Cbn(2, 2);
        H(2, 9) = Cbn(2, 3);
        H(3, 7) = Cbn(3, 1);
        H(3, 8) = Cbn(3, 2);
        H(3, 9) = Cbn(3, 3);

        R = eye(3, 3);
        R(1, 1) = 15^2;
        R(2, 2) = 15^2;
        R(3, 3) = 15^2;
        
        acc_liner_b = zeros(3, 1);
        g_estimate = Cbn*(acc_bias - Acc(i, :)' + acc_liner_b);
        Z = G_vector - g_estimate;

        K = P*H'*((H*P*H'+R)^-1);
        x = x + K*(Z - H*x);
        P = (I - K*H)*P;
        [deltCbn] = euler2dcm (x(3), x(2), x(1)); % (I+P)Cbn
        Cbn = deltCbn*Cbn;
        [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);

        acc_bias = acc_bias + x(7:9);
        gyro_bias = gyro_bias + x(4:6);
        x(1:9) = 0;
        kf_count = 1;
    else
        Cbn = q2dcm(q);
        [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);
        kf_count = kf_count + 1;
    end
    fprintf(fp, '%d %f %f %f\r\n', i, yaw(i), pitch(i), roll(i));
    
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
    %% use liner acc to detect action start and end
    liner_acc_x = f_p(1);
    if action_end ~= 1
        switch curve_condition
            case peace
                if liner_acc_x > 10
                    action_start = 1;
                    action_start_index = i;
                    curve_condition = step1;
                end

            case step1
                if liner_acc_x > liner_acc_x_last
                    slop = 1;
                else
                    slop = -1;
                    % reach the up peak
                    if liner_acc_x_last < 20
                        % false peak
                        curve_condition = peace;
                        action_start = 0;
                        action_start_index = 0;
                        platform_omega_Zmax = 0;
                        platform_omega_Zmin = 0;
                    else
                        curve_condition = step2;
                    end
                end

            case step2
                if liner_acc_x > liner_acc_x_last
                    slop = 1;
                    % reach the down peak
                    if liner_acc_x_last > -20
                        % false peak
                    else
                        curve_condition = step3;
                    end
                else
                    slop = -1;
                end

            case step3
                if liner_acc_x > liner_acc_x_last
                    slop = 1;
                else
                    slop = -1;
                end
                if liner_acc_x > -10 && liner_acc_x < 10
                    action_end = 1;
                    action_end_index = i - 1;
                    curve_condition = peace;
                end
        end
    end
    liner_acc_x_last = liner_acc_x;
    
    if action_end == 1
        action_count = action_count + 1;
        action_array(action_count, :) = [action_start_index, action_end_index];
        action_end = 0;
        action_start = 0;
    end

    %% ins
    acc_liner_p(i, :) = f_p - cross((2*Wiep+Wepp), [vN(i); vE(i); vD(i)]) + G_vector;
    if action_start == 1 && action_end == 0;
        vN(i) = vN(i-1) + (acc_liner_p(i, 1) + acc_liner_p(i-1, 1))*dt/2;
        vE(i) = vE(i-1) + (acc_liner_p(i, 2) + acc_liner_p(i-1, 2))*dt/2;
        vD(i) = vD(i-1) + (acc_liner_p(i, 3) + acc_liner_p(i-1, 3))*dt/2;
        pN(i) = pN(i-1) + (vN(i) + vN(i-1))*dt/2;
        pE(i) = pE(i-1) + (vE(i) + vE(i-1))*dt/2;
        pD(i) = pD(i-1) + (vD(i) + vD(i-1))*dt/2;
        
        if platform_omega(i, 3) > platform_omega_Zmax
            platform_omega_Zmax = platform_omega(i, 3);
        end
        
        if platform_omega(i, 3) < platform_omega_Zmin
            platform_omega_Zmin = platform_omega(i, 3);
        end
%% store result data
        % Rotate: NED(xyz) -> Unity3D(XYZ): right hand -> left hand
        % X = -x, Y = -y, Z = z
        
        % Transform: NED(xyz) -> Unity3D(XYZ):
        % X = x, Y = -z, Z = -Y
        
        unity_x = pN(i);
        unity_y = -pD(i);
        unity_z = -pE(i);
        unity_q = [q(1), -q(2), -q(3), q(4)];
        fprintf(fd, '%d %f %f %f %f %f %f %f\r\n', i,unity_x, unity_y, unity_z, unity_q(1), unity_q(2), unity_q(3), unity_q(4));
    end
end
fclose(fd);
fclose(fp);

if abs(platform_omega_Zmax) > abs(platform_omega_Zmin)
    fprintf('back hand');
else
    fprintf('fore hand');
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
plot(platform_omega(:, 1), 'r');
hold on;
plot(platform_omega(:, 2), 'g');
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
if 1
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


