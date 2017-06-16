#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "types.h"
#include "sensor.h"
#include "fusion.h"

#ifdef DEBUG
FILE *FpOutput;
#endif
/*-------------------------------------------------------------------------*/
/**
  @brief    Remove blanks at the beginning and the end of a string.
  @param    s   String to parse.
  @return   ptr to statically allocated string.

  This function returns a pointer to a statically allocated string,
  which is identical to the input string, except that all blank
  characters at the end and the beg. of the string have been removed.
  Do not free or modify the returned string! Since the returned string
  is statically allocated, it will be modified at each function call
  (not re-entrant).
 */
/*--------------------------------------------------------------------------*/
#define ASCIILINESZ         (1024)

static char* strstrip(char* s)
{
    static char l[ASCIILINESZ + 1];
    char* last ;

    if (s == NULL)
    {
        return NULL ;
    }

    while (isspace((int)*s) && *s)
    {
        s++;
    }

    memset(l, 0, ASCIILINESZ + 1);
    strcpy_s(l, ASCIILINESZ + 1, s);
    last = l + strlen(l);

    while (last > l)
    {
        if (!isspace((int) * (last - 1)))
        {
            break ;
        }

        last -- ;
    }

    *last = (char)0;
    return (char*)l ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
#define MAX_SECTION     (20)

U32 praseSensorData(char *str, sensorData_t *psensorData)
{
    U32  count = 0;
    char *section[MAX_SECTION] = {NULL};
    char *buffer = str;
    char *ptr = NULL;

    while( (section[count] = strtok_s(buffer, ",", &ptr)) != NULL )
    {
        count ++;
        buffer = NULL;
    }

    // convert string to number value
    psensorData->uTime = atoi(section[0]);
    psensorData->fGyro[X] = (FLT)atof(section[5]);
    psensorData->fGyro[Y] = (FLT)atof(section[6]);
    psensorData->fGyro[Z] = (FLT)atof(section[7]);
    psensorData->fAcc[X] = (FLT)atof(section[8]);
    psensorData->fAcc[Y] = (FLT)atof(section[9]);
    psensorData->fAcc[Z] = (FLT)atof(section[10]);
    psensorData->fMag[X] = (FLT)atof(section[11]);
    psensorData->fMag[Y] = (FLT)atof(section[12]);
    psensorData->fMag[Z] = (FLT)atof(section[13]);

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
U32 sensorNavInit(void)
{
    sensorFusionInit();

    return 0;
}


/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
U32 sensorNavExec(sensorData_t *psensorData)
{
    fusionInputData_t fusionInputData;
    fusionOutputData_t fusionOutputData;

    memset(&fusionInputData, 0, sizeof(fusionInputData_t));
    memset(&fusionOutputData, 0, sizeof(fusionOutputData_t));

    //time
    fusionInputData.uTime = psensorData->uTime;

    // gyro
    fusionInputData.fGyro[X] = psensorData->fGyro[X] * DEG2RAD;
    fusionInputData.fGyro[Y] = psensorData->fGyro[Y] * DEG2RAD;
    fusionInputData.fGyro[Z] = psensorData->fGyro[Z] * DEG2RAD;
    // acc
    fusionInputData.fAcc[X] = psensorData->fAcc[X] * GRAVITY;
    fusionInputData.fAcc[Y] = psensorData->fAcc[Y] * GRAVITY;
    fusionInputData.fAcc[Z] = psensorData->fAcc[Z] * GRAVITY;
    // mag
    fusionInputData.fMag[X] = psensorData->fMag[X];
    fusionInputData.fMag[Y] = psensorData->fMag[Y];
    fusionInputData.fMag[Z] = psensorData->fMag[Z];

    sensorFusionExec(&fusionInputData, &fusionOutputData);

#ifdef DEBUG
    //format: time, yaw, pitch, roll, vx, vy, vz, px, py, pz
    fprintf(FpOutput, "%d %-5.3f %-5.3f %-5.3f %-5.3f %-5.3f %-5.3f %-5.3f %-5.3f %-5.3f\r\n",
            fusionOutputData.uTime, fusionOutputData.fPsiPl, fusionOutputData.fThePl, fusionOutputData.fPhiPl,
            fusionOutputData.fVelN, fusionOutputData.fVelE, fusionOutputData.fVelD, fusionOutputData.fPosN,
            fusionOutputData.fPosE, fusionOutputData.fPosD);
#endif

    return 0;
}