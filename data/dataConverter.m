clear all;
clc;

fin = fopen('leonPingPong.txt', 'r');
fout = fopen('actionData.txt', 'w');
if fin == -1
    fprintf('file not exist');
    exit(-1);
end

while ~feof(fin)
    line = fgets(fin);

    entries = regexp(line, ' ', 'split');
    
    header = char(entries(1));
    sequence = char(entries(2));
    
    acc_x_h = char(entries(3));
    acc_x_l = char(entries(4));
    acc_x = strcat(acc_x_h, acc_x_l);
    acc_x = hex2dec(acc_x);
    if acc_x > 32767
        acc_x = acc_x - 65536;
    end
    acc_x = acc_x / 2048; % g
    
    acc_y_h = char(entries(5));
    acc_y_l = char(entries(6));
    acc_y = strcat(acc_y_h, acc_y_l);
    acc_y = hex2dec(acc_y);
    if acc_y > 32767
        acc_y = acc_y - 65536;
    end
    acc_y = acc_y / 2048; % g
    
    acc_z_h = char(entries(7));
    acc_z_l = char(entries(8));
    acc_z = strcat(acc_z_h, acc_z_l);
    acc_z = hex2dec(acc_z);
    if acc_z > 32767
        acc_z = acc_z - 65536;
    end
    acc_z = acc_z / 2048; % g
    
    gyro_x_h = char(entries(9));
    gyro_x_l = char(entries(10));
    gyro_x = strcat(gyro_x_h, gyro_x_l);
    gyro_x = hex2dec(gyro_x);
    if gyro_x > 32767
        gyro_x = gyro_x - 65536;
    end
    gyro_x = gyro_x / 16.4; % degree/s
    
    gyro_y_h = char(entries(11));
    gyro_y_l = char(entries(12));
    gyro_y = strcat(gyro_y_h, gyro_y_l);
    gyro_y = hex2dec(gyro_y);
    if gyro_y > 32767
        gyro_y = gyro_y - 65536;
    end
    gyro_y = gyro_y / 16.4; % degree/s
    
    gyro_z_h = char(entries(13));
    gyro_z_l = char(entries(14));
    gyro_z = strcat(gyro_z_h, gyro_z_l);
    gyro_z = hex2dec(gyro_z);
    if gyro_z > 32767
        gyro_z = gyro_z - 65536;
    end
    gyro_z = gyro_z / 16.4; % degree/s
    
    audio_h = char(entries(17));
    audio_l = char(entries(18));
    audio = strcat(audio_h, audio_l);
    audio = hex2dec(audio);
    if audio > 32767
        audio = audio - 65536;
    end
    
    % sensor hal: sensor frame(xyz) -> device frame(XYZ)
    % X = -y, Y = -x, Z = -z
    temp = -gyro_x;
    gyro_x = -gyro_y;
    gyro_y = temp;
    gyro_z = -gyro_z;
    
    temp = -acc_x;
    acc_x = -acc_y;
    acc_y = temp;
    acc_z = -acc_z;
    
    % store in file
    fprintf(fout, '%d %d %d %d %d %f %f %f %f %f %f %f\r\n', 1, 2, 3, 4, 5, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, audio);
end
fclose(fin);
fclose(fout);
fprintf('data covert is completed.');





