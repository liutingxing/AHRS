//
// Created by jiangtianyu on 2018/10/8.
//
#include "training.h"

PingPongTrainData::PingPongTrainData()
{
    bValid = false;
    iScore = 88;
    sTrajectory = "";
    uActionCount = 0;
    sActionType = "";
    fRangeMax = 0;
    fVelocityMax = 0;
    fVelocityStrike = 0;
    iTrajectorySweet = 1;
    iStrikeSweet = 1;
    uStrikePower = 100;
    uPlayLoad = 20;

    //star chart
    iTrajectorySweetScore = iTrajectorySweet * 20;
    iStrikeSweetScore = iTrajectorySweet * 20;
    iStrikePowerScore = 80;
    iRangeScore = 70;
    iStrikeVelocityScore = 60;
};
