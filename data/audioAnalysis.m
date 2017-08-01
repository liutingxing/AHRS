clear all;
clc;

ahrs_data = load('actionData.txt');

audio = ahrs_data(:, 12);

figure;
plot(audio);