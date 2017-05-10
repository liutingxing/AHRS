clear all;
close all;
clc;

ahrs_data = load('.\data\快速正手一次1.txt');

Time = ahrs_data(:, 1);                      % ( ms )
Roll = ahrs_data(:, 3);                      % ( degree )
Pitch = ahrs_data(:, 4);                     % ( degree )
Yaw = ahrs_data(:, 5);                       % ( degree )
Gyro = ahrs_data(:, 6:8) * pi / 180;         % ( rad/s )
Acc = ahrs_data(:, 9:11) * 9.8;              % ( m/s2 )
Mag = ahrs_data(:, 12:14);                   % ( count )

%% variable prepare
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
fd = fopen('DataForPlatform.txt', 'w+');

%% initial alignment
yaw_initial = 0;
pitch_initial = 0;
roll_initial = 0;
q = euler2q(yaw_initial, pitch_initial, roll_initial);

%% find start and end timing
action_start = 476;
action_end = 500;

for i = 1 : N
%% attitude integration process
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
    Cbn = q2dcm(q);
    [yaw(i), pitch(i), roll(i)] = dcm2euler(Cbn);

%% acc fuison for bias estimate and attitude correction 
    if i < action_start
       
    end
    

%% ins mechanization
    if i >= action_start && i <= action_end
        f_b = Acc(i, :)' - acc_bias;
        f_p = Cbn*f_b;
        acc_liner_p(i, :) = f_p - cross((2*Wiep+Wepp), [vN(i); vE(i); vD(i)]) + G_vector;
        % static constrain
        for j = 1:3
            if abs(acc_liner_p(i, j)) < 1
                acc_liner_p(i, j) = 0;
            end
        end
        vN(i) = vN(i-1) + (acc_liner_p(i, 1) + acc_liner_p(i-1, 1))*dt/2;
        vE(i) = vE(i-1) + (acc_liner_p(i, 2) + acc_liner_p(i-1, 2))*dt/2;
        vD(i) = vD(i-1) + (acc_liner_p(i, 3) + acc_liner_p(i-1, 3))*dt/2;
        pN(i) = pN(i-1) + (vN(i) + vN(i-1))*dt/2;
        pE(i) = pE(i-1) + (vE(i) + vE(i-1))*dt/2;
        pD(i) = pD(i-1) + (vD(i) + vD(i-1))*dt/2;
%% store result data
        fprintf(fd, '%d %f %f %f %f %f %f %f\r\n', Time(i), pN(i), pE(i), pD(i), q(1), q(2), q(3), q(4));
    end
end
fclose(fd);

%% display result
% acc measurement
figure
plot(Acc(:, 1), 'r');
hold on;
plot(Acc(:, 2), 'g');
plot(Acc(:, 3), 'b');
title('acc measurement');
legend('x', 'y', 'z');
xlabel('sample point');
ylabel('acc (g)');

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

% position
figure;
plot3(pN(action_start-1:action_end), pE(action_start-1:action_end), pD(action_start-1:action_end), 'r', 'linewidth', 3);
title('position');
box on;
grid;
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;

if 0
figure;
plot(pN(action_start-1:action_end), pE(action_start-1:action_end), 'r');
xlabel('Px');
ylabel('Py');
title('OXY');

figure;
plot(pN(action_start-1:action_end), pD(action_start-1:action_end), 'g');
xlabel('Px');
ylabel('Pz');
title('OXZ');

figure;
plot(pE(action_start-1:action_end), pD(action_start-1:action_end), 'b');
xlabel('Py');
ylabel('Pz');
title('OYZ');
end

% velocity
figure;
plot(vN(action_start-1:action_end), 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('X velocity');

figure;
plot(vE(action_start-1:action_end), 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('Y velocity');

figure;
plot(vD(action_start-1:action_end), 'r');
xlabel('sample point');
ylabel('velocity (m/s)');
title('Z velocity');


