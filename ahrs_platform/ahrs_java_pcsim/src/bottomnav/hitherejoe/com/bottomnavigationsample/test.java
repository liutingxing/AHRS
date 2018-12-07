package bottomnav.hitherejoe.com.bottomnavigationsample;

import java.io.*;
import java.util.*;

public class test {

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

	public static void main(String[] args) {
        test myTest = new test();

        try {
            myTest.reader = new BufferedReader(new FileReader("./data/rawData.txt"));
            myTest.writer = new BufferedWriter(new FileWriter("./data/outputData.txt"));
            myTest.sensorDataWriter = new BufferedWriter(new FileWriter("./data/sensorData.txt"));
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
                fMag[0] = magx*0.6; // uT
                fMag[1] = magy*0.6; // uT
                fMag[2] = magz*0.6; // uT
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

                sensorFusion.uTime++;
                sAttitude = sensorFusion.sensorFusionExec(sensorFusion.uTime, fGyro, fAcc, fMag, fAudio);
                try {
                    sensorDataWriter.write(String.valueOf(sensorFusion.uTime) + " " + String.valueOf(fMag[0]) + " " + String.valueOf(fMag[1]) + " " + String.valueOf(fMag[2]) + "\r\n");
                }
                catch (IOException e) {
                    e.printStackTrace();
                }
                if (sAttitude != null)
                {
                    try {
                        writer.write(sAttitude + "\r\n");
                    }
                    catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                if (sensorFusion.uActionComplete == true)
                {
                    System.out.println("-------------------------------------------------------------------------------------------------------------------------------------");
                    System.out.println(sensorFusion.trainData.sTrajectory);
                    System.out.println("-------------------------------------------------------------------------------------------------------------------------------------");
                }
                break;
            }
            // power data
            case (byte) 0x82:
                byte power = dataArray.get(4).byteValue();
                break;
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
