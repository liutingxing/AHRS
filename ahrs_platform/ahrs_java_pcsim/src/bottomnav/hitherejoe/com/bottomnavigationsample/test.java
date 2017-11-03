package bottomnav.hitherejoe.com.bottomnavigationsample;

import java.io.*;

public class test {

	public static void main(String[] args) {
		SensorFusion sensorFusion = new SensorFusion();
        BufferedReader reader = null;
        BufferedWriter writer = null;

        try {
            reader = new BufferedReader(new FileReader("./data/actionData.txt"));
            writer = new BufferedWriter(new FileWriter("./data/outputData.txt"));
            String tempString = null;
            String sAttitude = null;
            int line = 1;

            while ((tempString = reader.readLine()) != null) {
            	double[] fAcc = new double[3];
                double[] fGyro = new double[3];
                double[] fMag = new double[3];
                double fAudio;
                int i;
                String[] sections = tempString.split(" ");
                
            	//System.out.println("line " + line + ": " + tempString);
            	// parse the sensor data
            	for (i = 0; i < 3; i++)
                {
                    fGyro[i] = Math.toRadians(Double.valueOf(sections[5+i]));        // rad/s
                    fAcc[i] = Double.valueOf(sections[8+i]) * sensorFusion.GRAVITY; // m/s2
                    fMag[i] = Double.valueOf(sections[11+i]);                        // uT
                }
                fAudio = Double.valueOf(sections[14]);
            	sensorFusion.uTime++;
            	sAttitude = sensorFusion.sensorFusionExec(sensorFusion.uTime, fGyro, fAcc, fMag, fAudio);
            	if (sAttitude != null)
            	{
            		writer.write(sAttitude + "\r\n");
            	}
            	if (sensorFusion.uActionComplete == true)
            	{
            		System.out.println("-------------------------------------------------------------------------------------------------------------------------------------");
            		System.out.println(sensorFusion.trainData.sTrajectory);
            		System.out.println("-------------------------------------------------------------------------------------------------------------------------------------");
            	}
                line++;
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (reader != null) {
                try {
                    reader.close();
                } catch (IOException e1) {
                }
            }
            if (writer != null) {
                try {
                    writer.close();
                } catch (IOException e1) {
                }
            }
        }
		// TODO Auto-generated method stub
	}

}
