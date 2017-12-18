clear all;
clc;

%% read data from file

ahrs_data = load('.\data\actionData.txt');

Time = ahrs_data(:, 1);                      % ( ms )
Roll = ahrs_data(:, 3);                      % ( degree )
Pitch = ahrs_data(:, 4);                     % ( degree )
Yaw = ahrs_data(:, 5);                       % ( degree )
Gyro = ahrs_data(:, 6:8);                    % ( rad/s )
Acc = ahrs_data(:, 9:11);                    % ( m/s2 )
Mag = ahrs_data(:, 12:14);                   % ( count )

%% variable prepare
SampleRate = 40;  % 40Hz sample rate
dt = 1 / SampleRate;
N = length(Time);
Calibration = 0;
Alignment = 1;
Fusion = 2;
Status = Calibration;
StaticFlag = -1;
alignNum = 100;
calibrationNum = 300;
alignCount = 1;
magCount = 1;
calibrationCount = 1;
alignGyroArray = zeros(alignNum, 3);
alignAccArray = zeros(alignNum, 3);
alignMagArray = zeros(alignNum, 3);
calibrationMagArray = zeros(calibrationNum, 3);

%% main loop
for i = 1:N
    
    %% context detect
    if alignCount <= alignNum
        alignGyroArray(alignCount,:) = Gyro(i,:);
        alignAccArray(alignCount,:) = Acc(i,:);
        alignCount = alignCount + 1;
        if Mag(i, 1) ~= 0 && Mag(i, 2) ~= 0 && Mag(i, 3) ~= 0
            alignMagArray(magCount,:) = Mag(i,:);
            magCount = magCount + 1;
        end
    else
        for j = 2:alignNum
            alignGyroArray(j-1, :) = alignGyroArray(j, :);
            alignAccArray(j-1, :) = alignAccArray(j, :);
        end
        alignGyroArray(alignNum, :) = Gyro(i, :);
        alignAccArray(alignNum, :) = Acc(i, :);
        if Mag(i, 1) ~= 0 && Mag(i, 2) ~= 0 && Mag(i, 3) ~= 0
            for j = 2:magCount-1
                alignMagArray(j-1, :) = alignMagArray(j, :);
            end
            alignMagArray(magCount-1,:) = Mag(i,:);
        end
    end
    
    if alignCount > alignNum
        % check static
        gyroNormVector = zeros(1, alignNum);
        accNormVector = zeros(1, alignNum);
        for j = 1:alignNum
            gyroNormVector(j) = norm(alignGyroArray(j,:));
            accNormVector(j) = norm(alignAccArray(j,:));
        end
        gyro_std = std(gyroNormVector);
        acc_std = std(accNormVector);
        
        if gyro_std < 0.01 && acc_std < 0.1
            StaticFlag = 1;
        else
            StaticFlag = 0;
        end
    end
    
    %% mag calibration
    if StaticFlag == 0 && Status == Calibration && Mag(i, 1) ~= 0 && Mag(i, 2) ~= 0 && Mag(i, 3) ~= 0
        % prepare mag calibration buffer
        if calibrationCount <= calibrationNum
            calibrationMagArray(calibrationCount, :) = Mag(i, :);
            calibrationCount = calibrationCount + 1;
        else
            for j = 2:calibrationNum
                calibrationMagArray(j-1, :) = calibrationMagArray(j, :);
            end
            calibrationMagArray(calibrationNum, :) = Mag(i, :);
        end
    end
    
    if calibrationCount > calibrationNum
        % 4 parameters calibration
        [B, V, W_inv, res] = cali4inv(calibrationMagArray);
        if B > 10 && B < 200 && res < 0.2
            Status = Alignment;
        end
    end
    
    
end




















