#ifndef _FUSION_H_
#define _FUSION_H_

#include "types.h"
#include "misc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define     X               (0)
#define     Y               (1)
#define     Z               (2)
#define     CHN             (3)

    
    typedef struct quaternion
    {
        FLT  q0;	// scalar component
        FLT  q1;	// x vector component
        FLT  q2;	// y vector component
        FLT  q3;	// z vector component
    } quaternion_t;

	typedef struct fusionInputData
	{
        U32     uTime;      // ms
        FLT     fGyro[CHN];	// rad/s
        FLT     fAcc[CHN];	// m/s2
        FLT     fMag[CHN];	// uT
	} fusionInputData_t;
	
    typedef struct fusionOutputData
    {
        U32   uTime;                    // time tag (ms)
        FLT   fPsiPl;					// yaw (deg)
        FLT   fThePl;					// pitch (deg)
        FLT   fPhiPl;					// roll (deg)
        FLT   fRPl[3][3];               // a posteriori orientation matrix (Cnb)
        quaternion_t fqPl;              // a posteriori orientation quaternion
        FLT   fVelN;                    // velocity toward the North
        FLT   fVelE;                    // velocity toward the East
        FLT   fVelD;                    // velocity toward the Down
        FLT   fPosN;                    // Position in the North
        FLT   fPosE;                    // Position in the East
        FLT   fPosD;                    // Position in the Down
    } fusionOutputData_t;

    U32 sensorFusionInit(void);
    U32 sensorFusionExec(const fusionInputData_t* const pinputData, fusionOutputData_t* const poutputData);

#ifdef __cplusplus
}      /* extern "C" */
#endif

#endif