clear all;
clc;

%% read data from file

ahrs_data = load('.\rotate.txt');

Time = ahrs_data(:, 1);                      % ( ms )
Roll = ahrs_data(:, 3);                      % ( degree )
Pitch = ahrs_data(:, 4);                     % ( degree )
Yaw = ahrs_data(:, 5);                       % ( degree )
Gyro = ahrs_data(:, 6:8) * pi / 180;         % ( rad/s )
Acc = ahrs_data(:, 9:11) * 9.8;              % ( m/s2 )
Mag = ahrs_data(:, 12:14);                   % ( count )

%% initial alignment (1s)
N = 100;
initial_alignment_flag = 0;
acc_alignment = Acc(1:N, :);
mag_alignment = Mag(1:N, :);
yaw_offset = 0;         % for ins mechanization
ins_yaw = [];  % for ins mechanization

% static check
acc_x_var = var(acc_alignment(1:N, 1));
acc_y_var = var(acc_alignment(1:N, 2));
acc_z_var = var(acc_alignment(1:N, 3));
mag_x_var = var(mag_alignment(1:N, 1));
mag_y_var = var(mag_alignment(1:N, 2));
mag_z_var = var(mag_alignment(1:N, 3));
acc_var_threshold = 0.01;
mag_var_threshold = 1;
if acc_x_var < acc_var_threshold && acc_y_var < acc_var_threshold && acc_z_var < acc_var_threshold && ...
   mag_x_var < mag_var_threshold && mag_y_var < mag_var_threshold && mag_z_var < mag_var_threshold
   initial_alignment_flag = 1;
end

if initial_alignment_flag == 1
    g_x_mean = -mean(acc_alignment(1:N, 1));
    g_y_mean = -mean(acc_alignment(1:N, 2));
    g_z_mean = -mean(acc_alignment(1:N, 3));
    mag_x_mean = mean(mag_alignment(1:N, 1));
    mag_y_mean = mean(mag_alignment(1:N, 2));
    mag_z_mean = mean(mag_alignment(1:N, 3));
    Cnb_initial = ecompass_ned([g_x_mean, g_y_mean, g_z_mean], [mag_x_mean, mag_y_mean, mag_z_mean]);
    Cbn_initial = Cnb_initial';
    [yaw_initial, pitch_initial, roll_initial] = dcm2euler(Cbn_initial);
    yaw_offset = yaw_initial;
    % compute the geomagnetic inclination angle
    geoB = norm([mag_x_mean, mag_y_mean, mag_z_mean]);
    gmod = norm([g_x_mean, g_y_mean, g_z_mean]);
    geo_inclination = asin(dot([mag_x_mean, mag_y_mean, mag_z_mean], [g_x_mean, g_y_mean, g_z_mean])/geoB/gmod); % rad
    Mag_vector = [geoB*cos(geo_inclination), 0, geoB*sin(geo_inclination)]';
% only for mag initial alignment test
%     Mag_vector_b = Cnb_initial*Mag_vector;
% only for mag initial alignment test
end

%% start calculation loop
G_vector = [0, 0, 9.8]';
N = length(Time);
yaw = zeros(N, 1);
pitch = zeros(N, 1);
roll = zeros(N, 1);
gyro_bias = zeros(3, 1);
acc_bias = zeros(3, 1);
ins_acc_bias = zeros(3, 1);
q = euler2q(yaw_initial, pitch_initial, roll_initial);
Cnb = eye(3, 3);
Cbn = eye(3, 3);
vN = zeros(N, 1);
vE = zeros(N, 1);
vD = zeros(N, 1);
pN = zeros(N, 1);
pE = zeros(N, 1);
pD = zeros(N, 1);
ahrs_acc_bias_array = zeros(N, 3);
ins_acc_bias_array = zeros(N, 3);
validation_segment_NUM = 60; % (0.6s)
validation_segment_cnt = 1;
validation_block_NUM = 4;
validation_block_cnt = 1;
acc_bias_buffer = zeros(validation_segment_NUM, 3);
acc_bias_mean_buffer = zeros(validation_block_NUM, 3);
acc_bias_std_buffer = zeros(validation_block_NUM, 3);
fusion_counter = 0;

x = zeros(9, 1); % roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z, acc_bias_x, acc_bias_y, acc_bias_z
F = zeros(9, 9);
PHIM = zeros(9, 9);
qdt = zeros(9, 9);
Q = zeros(9, 9);
G = zeros(9, 9);
Corr_time_gyro = 0.01;
Corr_time_acc = 0.01;
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

acc_liner_p = zeros(N, 3);

fd = fopen('DataForPlatform.txt', 'w+');
fd_linerAcc = fopen('LinerAcceleration.txt', 'w+');
for i = 1 : N
    %% start AHRS integration process

    dt = 10/1000; % 100Hz output rate
    
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
    
    %% start sensor fusion process
    
    Cbn = q2dcm(q);
%     [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);
    
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
    Q = dt*M1 + 1/2*dt*dt*M2;
    
    % PHIM matrix discretization-2 order
    I = eye(9, 9);
    PHIM = I + dt*F + 1/2*dt*dt*F*F;
    
    %% predict
    x = PHIM*x;
    P = PHIM*P*PHIM' + Q;
    
    %% update from acc
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
    R(1, 1) = 0.5^2;
    R(2, 2) = 0.5^2;
    R(3, 3) = 0.5^2;
    
    acc_liner_b = zeros(3, 1);
% only for acc bias estimate test
%     Acc(i, 1) = Acc(i, 1) + 1;
%     Acc(i, 2) = Acc(i, 2) + 1;
%     Acc(i, 3) = Acc(i, 3) + 1;
% only for acc bias estimate test
    g_estimate = Cbn*(acc_bias - Acc(i, :)' + acc_liner_b);
    Z = G_vector - g_estimate;
    
    K = P*H'*((H*P*H'+R)^-1);
%     acc_mod = norm(Acc(i, :));
%     fusion_threshold = 0.5;
%     if (acc_mod > (9.8 - fusion_threshold) && acc_mod < (9.8 + fusion_threshold))
%        fusion_counter = fusion_counter + 1;
    x = x + K*(Z - H*x);
    P = (I - K*H)*P;
%     end
    
    %% update from mag
    H = zeros(3, 9);
    H(1, 2) = Mag_vector(3);
    H(2, 1) = -Mag_vector(3);
    H(2, 3) = Mag_vector(1);
    H(3, 2) = -Mag_vector(1);
    
    R = eye(3, 3);
    R(1, 1) = 1^2;
    R(2, 2) = 1^2;
    R(3, 3) = 1^2;
    
    mag_estimate = Cbn*Mag(i, :)';
    Z = Mag_vector - mag_estimate;
    
    K = P*H'*((H*P*H'+R)^-1);
    x = x + K*(Z - H*x);
    P = (I - K*H)*P;
    
    %% feedback    
    [deltCbn] = euler2dcm (x(3), x(2), x(1)); % (I+P)Cbn
    Cbn = deltCbn*Cbn;
    [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);
    
    acc_bias = acc_bias + x(7:9);
    gyro_bias = gyro_bias + x(4:6);
    x(1:9) = 0;
    ahrs_acc_bias_array(i, :) = acc_bias';
    
    %% acc bias validation for ins mechanization
    acc_bias_buffer(validation_segment_cnt, :) = acc_bias;
    validation_segment_cnt = validation_segment_cnt + 1;
    if (validation_segment_cnt > validation_segment_NUM)
        validation_segment_cnt = 1;
        std_max = max(std(acc_bias_buffer, 1));
        if std_max < 0.2
            acc_bias_mean_buffer(validation_block_cnt, :) = mean(acc_bias_buffer);
            acc_bias_std_buffer(validation_block_cnt, :) = std(acc_bias_buffer, 1); % devided by N, not N-1
            validation_block_cnt = validation_block_cnt + 1;
        else
            validation_block_cnt = 1;
            acc_bias_mean_buffer = zeros(validation_block_NUM, 3);
            acc_bias_std_buffer = zeros(validation_block_NUM, 3);   
        end

        if (validation_block_cnt > validation_block_NUM)
            validation_block_cnt = 1;
            ins_acc_bias_mean = mean(acc_bias_mean_buffer);
            ins_acc_bias_std = std(acc_bias_mean_buffer, 1);
            
            if max(ins_acc_bias_std) < 0.1
                ins_acc_bias = ins_acc_bias_mean';
            end
        end
    end
    ins_acc_bias_array(i, :) = ins_acc_bias;
    
    %% convert quaternion to euler
    q = euler2q(yaw(i), pitch(i), roll(i));
    q = q_norm(q);
    
    % convert from NED frame
    ins_yaw(i) = yaw(i) - yaw_offset;
    if (ins_yaw(i) > pi)
        ins_yaw(i) = ins_yaw(i) - 2*pi;
    else if ins_yaw(i) < -pi
            ins_yaw(i) = ins_yaw(i) + 2*pi;
        end
    end
    
    %% ins mechanization
    ins_Cbn = euler2dcm(ins_yaw(i), pitch(i), roll(i));
%     ins_Cbn = euler2dcm(ins_yaw(i), pitch(i), 0);
%     f_b = Acc(i, :)' - ins_acc_bias;
    f_b = Acc(i, :)' - acc_bias;
    f_p = ins_Cbn*f_b;
    acc_liner_p(i, :) = f_p - cross((2*Wiep+Wepp), [vN(i); vE(i); vD(i)]) + G_vector;
    % static constrain
    for j = 1:3
        if abs(acc_liner_p(i, j)) < 0.1
            acc_liner_p(i, j) = 0;
        end
    end
    if i > 1
        vN(i) = vN(i-1) + (acc_liner_p(i, 1) + acc_liner_p(i-1, 1))*dt/2;
        vE(i) = vE(i-1) + (acc_liner_p(i, 2) + acc_liner_p(i-1, 2))*dt/2;
        vD(i) = vD(i-1) + (acc_liner_p(i, 3) + acc_liner_p(i-1, 3))*dt/2;
        pN(i) = pN(i-1) + (vN(i) + vN(i-1))*dt/2;
        pE(i) = pE(i-1) + (vE(i) + vE(i-1))*dt/2;
        pD(i) = pD(i-1) + (vD(i) + vD(i-1))*dt/2;
    end
    
    
    %% store fusion data
    fprintf(fd, '%d %f %f %f %f %f %f %f %f %f %f %f %f %f \r\n', Time(i), pN(i), pE(i), pD(i), q(1), q(2), q(3), q(4), vN(i), vE(i), vD(i), Acc(i, 1), Acc(i, 2), Acc(i, 3));
    fprintf(fd_linerAcc, '%d %f %f %f \r\n', Time(i), acc_liner_p(i, 1), acc_liner_p(i, 2), acc_liner_p(i, 3));
    
end
fclose(fd);
fclose(fd_linerAcc);

%% display result

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

% ahrs_acc_bias
figure;
plot(ahrs_acc_bias_array(:, 1), 'r');
hold on;
plot(ahrs_acc_bias_array(:, 2), 'g');
plot(ahrs_acc_bias_array(:, 3), 'b');
legend('X', 'Y', 'Z');
title('ahrs acc bias estimate');
xlabel('sample point');
ylabel('acc bias (m/s2)');

% ins_acc_bias
figure;
plot(ins_acc_bias_array(:, 1), 'r');
hold on;
plot(ins_acc_bias_array(:, 2), 'g');
plot(ins_acc_bias_array(:, 3), 'b');
legend('X', 'Y', 'Z');
title('ins acc bias estimate');
xlabel('sample point');
ylabel('acc bias (m/s2)');

% position
figure;
plot(pN, pE, 'r');
xlabel('Px');
ylabel('Py');
title('OXY');

figure;
plot(pN, pD, 'g');
xlabel('Px');
ylabel('Pz');
title('OXZ');

figure;
plot(pE, pD, 'b');
xlabel('Py');
ylabel('Pz');
title('OYZ');

% velocity
figure;
plot(vN, 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('X velocity');

figure;
plot(vE, 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('Y velocity');

figure;
plot(vD, 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('Z velocity');

% liner acceleration
figure;
plot(acc_liner_p(:, 1), 'r');
title('liner acceleration X');
xlabel('sample point');
ylabel('liner acc (m/s2)');

figure;
plot(acc_liner_p(:, 2), 'g');
title('liner acceleration Y');
xlabel('sample point');
ylabel('liner acc (m/s2)');

figure;
plot(acc_liner_p(:, 3), 'b');
title('liner acceleration Z');
xlabel('sample point');
ylabel('liner acc (m/s2)');

% ins yaw
figure;
plot(ins_yaw*180/pi, 'r');
title('ins yaw');
xlabel('sample point');
ylabel('ins yaw (degree)');

% x(t)
figure;
plot(pE, 'r');

