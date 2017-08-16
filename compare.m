clear all;
clc;

simulation_data = load('Angle.txt');
pratical_data = load('2as.txt');

% yaw
figure;
title('yaw')
plot(simulation_data(:,2)*180/pi, 'r');
hold on;
plot(pratical_data(:,2)*180/pi, 'b');
legend('simulation', 'practical');

% pitch
figure;
title('pitch')
plot(simulation_data(:,3)*180/pi, 'r');
hold on;
plot(pratical_data(:,3)*180/pi, 'b');
legend('simulation', 'practical');

% roll
figure;
title('roll')
plot(simulation_data(:,4)*180/pi, 'r');
hold on;
plot(pratical_data(:,4)*180/pi, 'b');
legend('simulation', 'practical');