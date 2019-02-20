package bottomnav.hitherejoe.com.bottomnavigationsample;

import java.io.*;
import java.lang.reflect.Array;
import java.util.*;

public class test {

    private final boolean Data_Compensate = true;
    private ArrayList<double[]> GyroDataPool = new ArrayList<>(20);
    private ArrayList<double[]> AccDataPool = new ArrayList<>(20);
    private ArrayList<double[]> MagDataPool = new ArrayList<>(20);
    private ArrayList<Double> AudioDataPool = new ArrayList<>(20);
    private double[][] GyroLastCpy = new double[3][3];
    double[][] GyroLast = new double[3][3];
    private final static int Outlier_Peace = 0;
    private final static int Outlier_Start = 1;
    private final static int Outlier_End = 2;
    private int Outlier_Detect_Status = Outlier_Peace;
    private int Outlier_ExtData_Count = 0;
    private final static int CHX = 0;
    private final static int CHY = 1;
    private final static int CHZ = 2;
    private final static double MAX_OMEGA_DEG = 2000;
    private final static double OMEGA_MARGIN = 10;

    private ArrayList<Byte> byteArray = new ArrayList<Byte>();
    private final int Delimiter_Detect = 0;
    private final int Data_Save = 1;
    private int Parser_Status = Delimiter_Detect;
    private int Parser_Read_Point = 0;
    private int Parser_Receive_Length = 0;
    private int Parser_Delimiter_Index = 0;
    private ArrayList<Byte> dataArray = new ArrayList<Byte>();

    public SensorFusion sensorFusion = new SensorFusion();
    public String sAttitude = null;
    public BufferedReader reader = null;
    public BufferedWriter writer = null;
    public BufferedWriter sensorDataWriter = null;
    public BufferedWriter extraDataWriter = null;

	public static void main(String[] args) {
        test myTest = new test();

        try {
            myTest.reader = new BufferedReader(new FileReader("./data/rawData.txt"));
            myTest.writer = new BufferedWriter(new FileWriter("./data/outputData.txt"));
            myTest.sensorDataWriter = new BufferedWriter(new FileWriter("./data/sensorData.txt"));
            myTest.extraDataWriter = new BufferedWriter(new FileWriter("./data/extData.txt"));
            String tempString = null;
            int line = 1;

            while ((tempString = myTest.reader.readLine()) != null) {
            	//System.out.println("line " + line + ": " + tempString);
            	// parse the sensor data
                myTest.displayData(tempString);
                line++;
            }
            myTest.reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (myTest.reader != null) {
                try {
                    myTest.reader.close();
                } catch (IOException e1) {
                }
            }
            if (myTest.writer != null) {
                try {
                    myTest.writer.close();
                    myTest.sensorDataWriter.close();
                    myTest.extraDataWriter.close();
                } catch (IOException e1) {
                }
            }
        }
		// TODO Auto-generated method stub
	}

    public void displayData(final String data)
    {
        if(data != null)
        {
            char[] charArray = data.replaceAll(" ", "").toCharArray();
            for(int i = 0; i < charArray.length/2; i++)
            {
                int pos = i*2;
                Byte charData = new Byte((byte) (charToByte(charArray[pos]) << 4 | charToByte(charArray[pos+1])));
                byteArray.add(charData);
            }

            while(Parser_Read_Point < byteArray.size())
            {
                Byte ele = byteArray.get(Parser_Read_Point);
                Parser_Read_Point++;
                Byte charData = new Byte(ele.byteValue());
                switch(Parser_Status)
                {
                    case Delimiter_Detect:
                        if(ele == (byte) 0xAA)
                        {
                            Parser_Delimiter_Index = Parser_Read_Point;
                            Parser_Status = Data_Save;
                            dataArray.add(charData);
                        }
                        break;
                    case Data_Save:
                        dataArray.add(charData);
                        if(dataArray.size() == 4)
                        {
                            // sof type seq len payload checksum eof
                            byte payload_length = dataArray.get(3).byteValue();
                            if(payload_length < 0)
                            {
                                // wrong frame
                                // remove the first delimiter(0xAA)
                                for(int i = 0; i < Parser_Delimiter_Index; i++)
                                {
                                    byteArray.remove(0);
                                }
                                // reset parser status
                                dataArray.clear();
                                Parser_Delimiter_Index = 0;
                                Parser_Read_Point = 0;
                                Parser_Receive_Length = 0;
                                Parser_Status = Delimiter_Detect;
                                continue;
                            }
                            Parser_Receive_Length = payload_length+6;
                        }
                        if(Parser_Receive_Length > 0 && dataArray.size() >= Parser_Receive_Length)
                        {
                            // check frame
                            if(checkDataFrame(dataArray))
                            {
                                // frame match succeed
                                // remove the match frame from buffer
                                for(int i = 0; i < Parser_Read_Point; i++)
                                {
                                    byteArray.remove(0);
                                }
                                // process data array
                                processDataFrame(dataArray);
                            }
                            else
                            {
                                // frame match failed
                                // remove the first delimiter(0xAA)
                                for(int i = 0; i < Parser_Delimiter_Index; i++)
                                {
                                    byteArray.remove(0);
                                }
                            }
                            // reset parser status
                            dataArray.clear();
                            Parser_Delimiter_Index = 0;
                            Parser_Read_Point = 0;
                            Parser_Receive_Length = 0;
                            Parser_Status = Delimiter_Detect;
                        }
                        break;
                }
            }
        }
    }

    private byte charToByte(char c)
    {
        return (byte) "0123456789ABCDEF".indexOf(c);
    }

    private void processDataFrame(ArrayList<Byte> dataArray)
    {
        double[] fAcc = new double[3];
        double[] fGyro = new double[3];
        double[] fMag = new double[3];
        double[] fGyroCpy;
        double fAudio;
        double ftemp;
        byte type = dataArray.get(1).byteValue();

        switch(type)
        {
            // sensor data
            case (byte) 0x01:
            {
                // acc data
                int accxH = dataArray.get(4).intValue() < 0 ? dataArray.get(4).intValue()+256 :
                        dataArray.get(4).intValue();
                int accxL = dataArray.get(5).intValue() < 0 ? dataArray.get(5).intValue()+256 :
                        dataArray.get(5).intValue();
                int accyH = dataArray.get(6).intValue() < 0 ? dataArray.get(6).intValue()+256 :
                        dataArray.get(6).intValue();
                int accyL = dataArray.get(7).intValue() < 0 ? dataArray.get(7).intValue()+256 :
                        dataArray.get(7).intValue();
                int acczH = dataArray.get(8).intValue() < 0 ? dataArray.get(8).intValue()+256 :
                        dataArray.get(8).intValue();
                int acczL = dataArray.get(9).intValue() < 0 ? dataArray.get(9).intValue()+256 :
                        dataArray.get(9).intValue();
                // gyro data
                int gyroxH = dataArray.get(10).intValue() < 0 ? dataArray.get(10).intValue()+256 :
                        dataArray.get(10).intValue();
                int gyroxL = dataArray.get(11).intValue() < 0 ? dataArray.get(11).intValue()+256 :
                        dataArray.get(11).intValue();
                int gyroyH = dataArray.get(12).intValue() < 0 ? dataArray.get(12).intValue()+256 :
                        dataArray.get(12).intValue();
                int gyroyL = dataArray.get(13).intValue() < 0 ? dataArray.get(13).intValue()+256 :
                        dataArray.get(13).intValue();
                int gyrozH = dataArray.get(14).intValue() < 0 ? dataArray.get(14).intValue()+256 :
                        dataArray.get(14).intValue();
                int gyrozL = dataArray.get(15).intValue() < 0 ? dataArray.get(15).intValue()+256 :
                        dataArray.get(15).intValue();
                // mag data
                int magxH = dataArray.get(16).intValue() < 0 ? dataArray.get(16).intValue()+256 :
                        dataArray.get(16).intValue();
                int magxL = dataArray.get(17).intValue() < 0 ? dataArray.get(17).intValue()+256 :
                        dataArray.get(17).intValue();
                int magyH = dataArray.get(18).intValue() < 0 ? dataArray.get(18).intValue()+256 :
                        dataArray.get(18).intValue();
                int magyL = dataArray.get(19).intValue() < 0 ? dataArray.get(19).intValue()+256 :
                        dataArray.get(19).intValue();
                int magzH = dataArray.get(20).intValue() < 0 ? dataArray.get(20).intValue()+256 :
                        dataArray.get(20).intValue();
                int magzL = dataArray.get(21).intValue() < 0 ? dataArray.get(21).intValue()+256 :
                        dataArray.get(21).intValue();
                // audio data
                int audioH = dataArray.get(22).intValue() < 0 ? dataArray.get(22).intValue()+256 :
                        dataArray.get(22).intValue();
                int audioL = dataArray.get(23).intValue() < 0 ? dataArray.get(23).intValue()+256 :
                        dataArray.get(23).intValue();

                short accx = (short) (accxH << 8 | accxL);
                short accy = (short) (accyH << 8 | accyL);
                short accz = (short) (acczH << 8 | acczL);
                short gyrox = (short) (gyroxH << 8 | gyroxL);
                short gyroy = (short) (gyroyH << 8 | gyroyL);
                short gyroz = (short) (gyrozH << 8 | gyrozL);
                short magx = (short) (magxH << 8 | magxL);
                short magy = (short) (magyH << 8 | magyL);
                short magz = (short) (magzH << 8 | magzL);
                short audio = (short) (audioH << 8 | audioL);

                fAcc[0] = accx/2048.0*sensorFusion.GRAVITY; // m/s2
                fAcc[1] = accy/2048.0*sensorFusion.GRAVITY; // m/s2
                fAcc[2] = accz/2048.0*sensorFusion.GRAVITY; // m/s2
                fGyro[0] = Math.toRadians(gyrox/16.4); // rad/s
                fGyro[1] = Math.toRadians(gyroy/16.4); // rad/s
                fGyro[2] = Math.toRadians(gyroz/16.4); // rad/s
                fMag[0] = magx*0.15; // uT
                fMag[1] = magy*0.15; // uT
                fMag[2] = magz*0.15; // uT
                fAudio = audio; // count

                // sensor hal: sensor frame(xyz) -> device frame(XYZ)
                // gyro/acc: X = x, Y = -y, Z = -z
                fAcc[1] = -fAcc[1];
                fAcc[2] = -fAcc[2];

                fGyro[1] = -fGyro[1];
                fGyro[2] = -fGyro[2];

                // mag: X = y, Y = -x, Z = z
                ftemp = -fMag[0];
                fMag[0] = fMag[1];
                fMag[1] = ftemp;

                // copy the raw gyro data
                fGyroCpy = new double[]{fGyro[0], fGyro[1], fGyro[2]};

                // outlier data detection
                if (!Data_Compensate)
                {
                    sensorFusionEntry(fGyro, fAcc, fMag, fAudio);
                }
                else
                {
                    switch (Outlier_Detect_Status)
                    {
                        case Outlier_Peace:
                            if (Math.abs(fGyro[CHX]) > Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN) ||
                                Math.abs(fGyro[CHY]) > Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN) ||
                                Math.abs(fGyro[CHZ]) > Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN))
                            {
                                // outlier data happens
                                for (int j = CHX; j <= CHZ; j++)
                                {
                                    GyroLastCpy[0][j] = GyroLast[0][j];
                                    GyroLastCpy[1][j] = GyroLast[1][j];
                                    GyroLastCpy[2][j] = GyroLast[2][j];
                                }
                                GyroDataPool.add(new double[]{fGyro[CHX], fGyro[CHY], fGyro[CHZ]});
                                AccDataPool.add(new double[]{fAcc[CHX], fAcc[CHY], fAcc[CHZ]});
                                MagDataPool.add(new double[]{fMag[CHX], fMag[CHY], fMag[CHZ]});
                                AudioDataPool.add(fAudio);
                                Outlier_Detect_Status = Outlier_Start;
                            }
                            else
                            {
                                sensorFusionEntry(fGyro, fAcc, fMag, fAudio);
                            }
                            break;

                        case Outlier_Start:
                            GyroDataPool.add(new double[]{fGyro[CHX], fGyro[CHY], fGyro[CHZ]});
                            AccDataPool.add(new double[]{fAcc[CHX], fAcc[CHY], fAcc[CHZ]});
                            MagDataPool.add(new double[]{fMag[CHX], fMag[CHY], fMag[CHZ]});
                            AudioDataPool.add(fAudio);
                            if (Math.abs(fGyro[CHX]) < Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN) &&
                                Math.abs(fGyro[CHY]) < Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN) &&
                                Math.abs(fGyro[CHZ]) < Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN))
                            {
                                // outlier data disappear
                                Outlier_Detect_Status = Outlier_End;
                                Outlier_ExtData_Count = 1;
                            }
                            break;

                        case Outlier_End:
                            GyroDataPool.add(new double[]{fGyro[CHX], fGyro[CHY], fGyro[CHZ]});
                            AccDataPool.add(new double[]{fAcc[CHX], fAcc[CHY], fAcc[CHZ]});
                            MagDataPool.add(new double[]{fMag[CHX], fMag[CHY], fMag[CHZ]});
                            AudioDataPool.add(fAudio);
                            Outlier_ExtData_Count++;
                            if (Outlier_ExtData_Count >= 3)
                            {
                                // start process outlier data
                                if (Data_Compensate)
                                {
                                    outlierDataProcess(GyroLastCpy, GyroDataPool);
                                }

                                // restart the sensor fusion entry
                                for (int i = 0; i < GyroDataPool.size(); i++)
                                {
                                    sensorFusionEntry(GyroDataPool.get(i), AccDataPool.get(i), MagDataPool.get(i), AudioDataPool.get(i));
                                }

                                // clear the status
                                GyroDataPool.clear();
                                AccDataPool.clear();
                                MagDataPool.clear();
                                AudioDataPool.clear();
                                Outlier_Detect_Status = Outlier_Peace;
                            }

                    }
                    // record the last gyro data
                    for (int i = CHX; i <= CHZ; i++)
                    {
                        GyroLast[0][i] = GyroLast[1][i];
                        GyroLast[1][i] = GyroLast[2][i];
                        GyroLast[2][i] = fGyroCpy[i];
                    }
                }
                break;
            }
            // power data
            case (byte) 0x82:
                byte power = dataArray.get(4).byteValue();
                break;
        }
    }

    private void outlierDataProcess(double[][] gyroLast, ArrayList<double[]> gyroDataPool)
    {
        int[] left_index = new int[]{-1,-1,-1};
        int[] right_index = new int[]{-1,-1,-1};
        int index;
        boolean[] outlier_flag = new boolean[]{false, false, false};

        /* determine the outlier data boundary */
        for (int i = CHX; i <= CHZ; i++) {
            index = 0;
            for (double[] val : gyroDataPool) {
                if (left_index[i] == -1) {
                    if (Math.abs(val[i]) > Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN)) {
                        left_index[i] = index;
                    }
                } else {
                    if (right_index[i] == -1) {
                        if (Math.abs(val[i]) < Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN)) {
                            right_index[i] = index - 1;
                            outlier_flag[i] = true;
                        }
                    }
                }

                if (outlier_flag[i]) {
                    break;
                }
                index++;
            }
        }

        /* estimate the outlier data */
        for (int channel = CHX; channel <= CHZ; channel++) {
            if (outlier_flag[channel]) {
                double[] x0 = new double[6];
                double[] y0 = new double[6];
                int seq_left = left_index[channel] - 1;
                int seq_right = right_index[channel] + 1;

                x0[0] = seq_left - 2;
                x0[1] = seq_left - 1;
                x0[2] = seq_left;
                x0[3] = seq_right;
                x0[4] = seq_right + 1;
                x0[5] = seq_right + 2;
                for (int i = 0; i < 6; i++) {
                    if (x0[i] < 0)
                    {
                        y0[i] = gyroLast[(int)(x0[i] + 3)][channel];
                    }
                    else
                    {
                        y0[i] = gyroDataPool.get((int)x0[i])[channel];
                    }
                }
                for (int i = left_index[channel]; i <= right_index[channel]; i++) {
                    gyroDataPool.get(i)[channel] = sensorFusion.splineFitting(x0, y0, 6, i);
                }
            }
        }
    }

    private void sensorFusionEntry(double[] fGyro, double[] fAcc, double[] fMag, double fAudio)
    {
        sensorFusion.uTime++;
        try {
            sensorDataWriter.write(String.valueOf(sensorFusion.uTime) + " " +
                    String.valueOf(fGyro[0]) + " " + String.valueOf(fGyro[1]) + " " + String.valueOf(fGyro[2]) + " " +
                    String.valueOf(fAcc[0]) + " " + String.valueOf(fAcc[1]) + " " + String.valueOf(fAcc[2]) + " " +
                    String.valueOf(fMag[0]) + " " + String.valueOf(fMag[1]) + " " + String.valueOf(fMag[2]) +  " " +
                    String.valueOf(fAudio) + "\n");
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        sAttitude = sensorFusion.sensorFusionExec(sensorFusion.uTime, fGyro, fAcc, fMag, fAudio);
        try {
            if (sAttitude != null) {
                writer.write(sAttitude + "\r\n");
            }
            extraDataWriter.write(String.valueOf(sensorFusion.uTime) + " " +
                    String.valueOf(sensorFusion.extLinerAccIBP[0]) + " " +
                    String.valueOf(sensorFusion.extLinerAccIBP[1]) + " " +
                    String.valueOf(sensorFusion.extLinerAccIBP[2]) + " " +
                    String.valueOf(sensorFusion.extPlatQ[0]) + " " +
                    String.valueOf(sensorFusion.extPlatQ[1]) + " " +
                    String.valueOf(sensorFusion.extPlatQ[2]) + " " +
                    String.valueOf(sensorFusion.extPlatQ[3]) + " " +"\n");
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        if (sensorFusion.uActionComplete == true)
        {
            System.out.println("-------------------------------------------------------------------------------------------------------------------------------------");
            System.out.println(sensorFusion.trainData.sTrajectory);
            System.out.println("-------------------------------------------------------------------------------------------------------------------------------------");
        }
    }

    private boolean checkDataFrame(ArrayList<Byte> dataArray)
    {
        byte type = dataArray.get(1).byteValue();
        byte seq = dataArray.get(2).byteValue();
        byte length = dataArray.get(3).byteValue();
        byte eof = dataArray.get(Parser_Receive_Length-1).byteValue();
        byte checksum = dataArray.get(Parser_Receive_Length-2).byteValue();
        int sum = 0;

        // check end marker of frame
        if(eof != (byte) 0x55)
        {
            return false;
        }
        // check data type and data length
        switch(type)
        {
            // sensor data
            case (byte) 0x01:
                if(length != (byte) 0x14)
                {
                    return false;
                }
                break;
            // power data
            case (byte) 0x82:
                if (length != (byte)0x01)
                {
                    return false;
                }
                break;
            default:
                return false;
        }
        // check sum validation
        sum = (int) type + (int) seq + (int) length;
        for(int i = 0; i < length; i++)
        {
            sum += dataArray.get(4+i).intValue();
        }
        if(checksum != (byte) sum)
        {
            return false;
        }

        return true;
    }

}
