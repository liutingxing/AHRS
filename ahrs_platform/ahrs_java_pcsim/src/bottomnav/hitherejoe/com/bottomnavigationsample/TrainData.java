package bottomnav.hitherejoe.com.bottomnavigationsample;

/**
 * Created by Jiang on 9/16/2017.
 */

public class TrainData {
    public Boolean bValid;
    public int iScore;
    public String sTrajectory;
    public int uActionCount;
    public String sActionType;
    public double fRangeMax;
    public double fVelocityMax;
    public double fVelocityStrike;
    public double fStrikeAudio;
    public String sTrajectorySweet;
    public String sStrikeSweet;
    public int uStrikePower;
    public int uPlayLoad;

    public TrainData()
    {
        bValid = false;
        iScore = 88;
        sTrajectory = null;
        uActionCount = 0;
        sActionType = null;
        fRangeMax = 0;
        fVelocityMax = 0;
        sTrajectorySweet = "perfect";
        sStrikeSweet = "perfect";
        uStrikePower = 0;
        uPlayLoad = 0;
    }

    public void resetTrainData()
    {
        bValid = false;
        iScore = 0;
        sTrajectory = null;
        uActionCount = 0;
        sActionType = null;
        fRangeMax = 0;
        fVelocityMax = 0;
        sTrajectorySweet = "perfect";
        sStrikeSweet = "perfect";
        uStrikePower = 0;
        uPlayLoad = 0;
    }
}
