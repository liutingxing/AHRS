#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define     X       (0)
#define     Y       (1)
#define     Z       (2)
#define     CHN     (3)

	typedef	struct sensorData
	{
        U32     uTime;      // ms
		FLT     fGyro[CHN];	// degree/s
		FLT     fAcc[CHN];	// g
		FLT     fMag[CHN];	// uT
	} sensorData_t;

#ifdef DEBUG
    extern FILE *FpOutput;
#endif

    U32 praseSensorData(char *str, sensorData_t *pSensorData);
    U32 sensorNavInit(void);
    U32 sensorNavExec(sensorData_t *pSensorData);

#ifdef __cplusplus
}      /* extern "C" */
#endif

#endif