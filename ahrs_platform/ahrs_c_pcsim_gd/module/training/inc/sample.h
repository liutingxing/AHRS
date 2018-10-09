//
// Created by jiangtianyu on 2018/10/9.
//

#ifndef AHRS_C_PCSIM_GD_SAMPLE_H
#define AHRS_C_PCSIM_GD_SAMPLE_H

class SampleData
{
    public:
        int          uTime;
        double       fPsiPlPlat;
        double       fThePlPlat;
        double       fPhiPlPlat;
        double       fCbnPla[3][3];
        double       fqPlPlat[4];
        double       fLinerAccN;
        double       fLinerAccE;
        double       fLinerAccD;
        double       fVelN;
        double       fVelE;
        double       fVelD;
        double       fPosN;
        double       fPosE;
        double       fPosD;
        double       fOmegaB[3];
        double       fAccelerate[3];
        double       fMagnetic[3];
        double       fAudio;
        double       fVel;
        double       fOmegaN[3];
};

#endif //AHRS_C_PCSIM_GD_SAMPLE_H
