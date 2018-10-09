//
// Created by jiangtianyu on 2018/10/8.
//

#ifndef AHRS_C_PCSIM_GD_TRAINING_H
#define AHRS_C_PCSIM_GD_TRAINING_H

using namespace std;

#include <string>

class TrainData
{
    public:
        bool bValid;
        int iScore;
        string sTrajectory;
        int uActionCount;
        string sActionType;
        double fRangeMax;
        double fVelocityMax;
        double fVelocityStrike;
        double fStrikeAudio;
        int iTrajectorySweet;
        int iStrikeSweet;
        int uStrikePower;
        int uPlayLoad;
        // audio
        int uAudioType;
        //start chart
        int iTrajectorySweetScore;
        int iStrikeSweetScore;
        int iStrikePowerScore;
        int iRangeScore;
        int iStrikeVelocityScore;

        TrainData();
};

#endif //AHRS_C_PCSIM_GD_TRAINING_H
