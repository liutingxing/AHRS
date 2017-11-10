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
    public int iTrajectorySweet;
    public int iStrikeSweet;
    public int uStrikePower;
    public int uPlayLoad;

    //start chart
    public int iTrajectorySweetScore;
    public int iStrikeSweetScore;
    public int iStrikePowerScore;
    public int iRangeScore;
    public int iStrikeVelocityScore;

    public TrainData()
    {
        bValid = false;
        iScore = 88;
        sTrajectory = null;
        uActionCount = 0;
        sActionType = null;
        fRangeMax = 0;
        fVelocityMax = 0;
        fVelocityStrike = 0;
        iTrajectorySweet = 1;
        iStrikeSweet = 1;
        uStrikePower = 100;
        uPlayLoad = 20;

        //start chart
        iTrajectorySweetScore = iTrajectorySweet*20;
        iStrikeSweetScore = iTrajectorySweet*20;
        iStrikePowerScore = 80;
        iRangeScore = 70;
        iStrikeVelocityScore = 60;
    }

    public void resetTrainData()
    {
        uActionCount = 0;
    }
}
