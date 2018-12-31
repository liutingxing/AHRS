clear all;
clc;

addpath('..\..\..\');

fin = fopen('outputData.txt', 'r');
if fin == -1
    fprintf('file not exist');
    exit(-1);
end

while ~feof(fin)
    line = fgets(fin);

    entries = regexp(line, ' ', 'split');
    
    time = char(entries(1));
    q0 = char(entries(5));
    q1 = char(entries(6));
    q2 = char(entries(7));
    q3 = char(entries(8));
    accX = char(entries(9));
    
    time = str2num(time);
    q0 = str2num(q0);
    q1 = str2num(q1);
    q2 = str2num(q2);
    q3 = str2num(q3);
    linerAccX(time) = str2num(accX);
    
    % convert the left hand frame to right hand frame
    q1 = -q1;
    q2 = -q2;
    
    % convert quarterion to euler
    q = [q0, q1, q2, q3];
    Cbn = q2dcm(q);
    [yaw(time), pitch(time), roll(time)] = dcm2euler(Cbn); 
end

fclose(fin);

% yaw
figure;
plot(yaw*180/pi, 'r');
grid on;
title('yaw');
xlabel('sample point');
ylabel('yaw (degree)');

% pitch
figure;
plot(pitch*180/pi, 'r');
grid on;
title('pitch');
xlabel('sample point');
ylabel('pitch (degree)');

% roll
figure;
plot(roll*180/pi, 'r');
grid on;
title('roll');
xlabel('sample point');
ylabel('roll (degree)');

