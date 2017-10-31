clear all;
clc;

ahrs_data = load('actionData.txt');

audio = ahrs_data(:, 15);

figure;
plot(audio);