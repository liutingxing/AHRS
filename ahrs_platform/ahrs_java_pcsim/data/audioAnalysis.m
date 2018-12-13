clear all;
clc;

ahrs_data = load('sensorData.txt');

audio = ahrs_data(:, 11);

figure;
plot(audio);