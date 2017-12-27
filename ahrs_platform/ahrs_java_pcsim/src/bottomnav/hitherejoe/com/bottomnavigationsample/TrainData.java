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

    public static String actionComment(int uStrikePower, double fRangeMax, double fVelocityMax, int iStrikeSweet, int iTrajectorySweet,
                                        int iScore)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("发力偏晚了，导致整个挥拍动作还没有在最佳时机就可是击球\r\n");
        comment.append("增加动作幅度可能会有更好的击球力量效果。注意引拍和身体保持稳定\r\n");

        return String.valueOf(comment);

    }

    public static String trainComment(int uStrikePower, double fRangeMax, double fVelocityMax, int iStrikeSweet, int iTrajectorySweet,
                                       int iScore, int uActionCount, double fFrequency, int uPlayLoad)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("较高的击球频率和幅度，可能导致动作，请减小击球频率，来保持轨迹点的稳定性和合理性\r\n");
        comment.append("你的动作幅度偏小，不要去碰球，而要保持一定的推挡力度，才能把控好球的落点和质量\r\n");

        return String.valueOf(comment);
    }

    public static String powerComment(int uStrikePower, int iStrikeSweet, int iTrajectorySweet)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("发力太晚了，导致刚开始发力，就已经击球出去，很有可能是发力动作开始过慢，注意身体保持平衡，肘部不要抬起，迎前击球，才会有好的效果\r\n");

        return String.valueOf(comment);
    }

    public static String velocityComment(double fVelocityMax, double fRangeMax, int iTrajectorySweet, double fFrequency)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("注意击球频率，如果追求力量和速度的完美结合，可以放慢击球频率，提高每次击球的质量，并注意避免肌肉僵硬\r\n");

        return String.valueOf(comment);
    }

    public static String trajectoryComment(int iTrajectorySweet, double fFrequency)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("发力偏晚了，导致整个挥拍动作没有在最佳速度时刻就可是击球\r\n");
        comment.append("请略微迎前击球，并保持身体稳定且重心位于前脚心，可能会有好的效果\r\n");

        return String.valueOf(comment);
    }

    public static String sweetComment(int iStrikeSweet)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("拍面击打位置非常棒\r\n");

        return String.valueOf(comment);
    }

    public static String rangeComment(String sActionType, double fRangeMax)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("你的动作幅度偏小，不要去碰球，而要保持一定的推挡力度，才能把控好球的落点和质量\r\n");

        return String.valueOf(comment);
    }

    public static String frequencyComment(double fFrequency)
    {
        StringBuffer comment = new StringBuffer();

        comment.append("很适合的击球频率，保持各项技术数据稳定\r\n");

        return String.valueOf(comment);
    }
}
