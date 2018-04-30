clear all;
clc;

fin = fopen('leonPingPong.txt', 'r');
fout = fopen('actionData.txt', 'w');
if fin == -1
    fprintf('file not exist');
    return;
end

while ~feof(fin)
    line = fgets(fin);

    entries = regexp(line, ' ', 'split');
    
    for i = 1:8
        offset = (i-1)*31;
        header = char(entries(1+offset));
        sequence = hex2dec(char(entries(2+offset)));
        if (~strcmp(header, 'F3'))
            fprintf('header error!\r\n');
            return;
        end
        
        acc_x_h = char(entries(3+offset));
        acc_x_l = char(entries(4+offset));
        acc_x = strcat(acc_x_h, acc_x_l);
        acc_x = hex2dec(acc_x);
        if acc_x > 32767
            acc_x = acc_x - 65536;
        end
        acc_x = acc_x * 9.80665 / 2048; % m/s2

        acc_y_h = char(entries(5+offset));
        acc_y_l = char(entries(6+offset));
        acc_y = strcat(acc_y_h, acc_y_l);
        acc_y = hex2dec(acc_y);
        if acc_y > 32767
            acc_y = acc_y - 65536;
        end
        acc_y = acc_y * 9.80665 / 2048; % m/s2

        acc_z_h = char(entries(7+offset));
        acc_z_l = char(entries(8+offset));
        acc_z = strcat(acc_z_h, acc_z_l);
        acc_z = hex2dec(acc_z);
        if acc_z > 32767
            acc_z = acc_z - 65536;
        end
        acc_z = acc_z * 9.80665 / 2048; % m/s2

        gyro_x_h = char(entries(9+offset));
        gyro_x_l = char(entries(10+offset));
        gyro_x = strcat(gyro_x_h, gyro_x_l);
        gyro_x = hex2dec(gyro_x);
        if gyro_x > 32767
            gyro_x = gyro_x - 65536;
        end
        gyro_x = gyro_x*pi/180 / 16.4; % rad/s

        gyro_y_h = char(entries(11+offset));
        gyro_y_l = char(entries(12+offset));
        gyro_y = strcat(gyro_y_h, gyro_y_l);
        gyro_y = hex2dec(gyro_y);
        if gyro_y > 32767
            gyro_y = gyro_y - 65536;
        end
        gyro_y = gyro_y*pi/180 / 16.4; % rad/s

        gyro_z_h = char(entries(13+offset));
        gyro_z_l = char(entries(14+offset));
        gyro_z = strcat(gyro_z_h, gyro_z_l);
        gyro_z = hex2dec(gyro_z);
        if gyro_z > 32767
            gyro_z = gyro_z - 65536;
        end
        gyro_z = gyro_z*pi/180 / 16.4; % rad/s
        
        mag_x_h = char(entries(15+offset));
        mag_x_l = char(entries(16+offset));
        mag_x = strcat(mag_x_h, mag_x_l);
        mag_x = hex2dec(mag_x);
        if mag_x > 32767
            mag_x = mag_x - 65536;
        end
        mag_x = mag_x * 0.6;
        
        mag_y_h = char(entries(17+offset));
        mag_y_l = char(entries(18+offset));
        mag_y = strcat(mag_y_h, mag_y_l);
        mag_y = hex2dec(mag_y);
        if mag_y > 32767
            mag_y = mag_y - 65536;
        end
        mag_y = mag_y * 0.6;
        
        mag_z_h = char(entries(19+offset));
        mag_z_l = char(entries(20+offset));
        mag_z = strcat(mag_z_h, mag_z_l);
        mag_z = hex2dec(mag_z);
        if mag_z > 32767
            mag_z = mag_z - 65536;
        end
        mag_z = mag_z * 0.6;

        audio_h = char(entries(21+offset));
        audio_l = char(entries(22+offset));
        audio = strcat(audio_h, audio_l);
        audio = hex2dec(audio);

        % sensor hal: sensor frame(xyz) -> device frame(XYZ)
        % gyro/acc: X = x, Y = -y, Z = -z
        % mag:      X = y, Y = -x, Z = z      
        gyro_y = -gyro_y;
        gyro_z = -gyro_z;

        acc_y = -acc_y;
        acc_z = -acc_z;
        
        temp = -mag_x;
        mag_x = mag_y;
        mag_y = temp;
        
        fprintf(fout, '%d %d %d %d %d %6f %6f %6f %6f %6f %6f %6f %6f %6f %6f\r\n', 1, 2, 3, 4, 5, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, audio);
    end
end
 
fclose(fin);
fclose(fout);
fprintf('data covert is completed.');





