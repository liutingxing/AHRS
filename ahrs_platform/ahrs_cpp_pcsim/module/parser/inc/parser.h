//
// Created by jiangtianyu on 2018/9/19.
//

#ifndef AHRS_C_PCSIM_GD_PASER_H
#define AHRS_C_PCSIM_GD_PASER_H

#include <cstring>
#include <iostream>
#include "fusion.h"
#include "fitting.h"

typedef     signed char             int8_t;                  /* Signed 8 bits integer    */
typedef     unsigned char           uint8_t;                 /* Unsigned 8 bits integer  */
typedef     signed short            int16_t;                 /* Signed 16 bits integer   */
typedef     unsigned short          uint16_t;                /* Unsigned 16 bits integer */
typedef     signed int              int32_t;                 /* Signed 32 bits integer   */
typedef     unsigned int            uint32_t;                /* Unsigned 32 bits integer */

#define  ACC_SENSITIVITY    (1.0/2048)
#define  GYRO_SENSITIVITY   (1.0/16.4)
#define  MAG_SENSITIVITY    (0.15)
#define  MaxBufferSize      (1024)

#define Data_Compensate     (1)

typedef enum outlierStatus
{
    Outlier_Peace = 0,
    Outlier_Start = 1,
    Outlier_End = 2,
} outlierStatus_t;

typedef enum parserStatus
{
    DelimiterDetect = 0,
    DataSave = 1,
} parserStatus_t;

typedef struct dataArray
{
    uint8_t buffer[MaxBufferSize];
    int size;
} dataArray_t;

class BleDataParser
{
    private:
        dataArray_t DataBuffer;
        dataArray_t DataBufferFiltered;
        parserStatus_t ParserStatus ;
        int ParserReceiveLength ;
        int ParserReadIndex;
        int ParserDelimiterIndex;
        SensorFusion sensorFusion;

        vector<shared_ptr<double>> GyroDataPool;
        vector<shared_ptr<double>> AccDataPool;
        vector<shared_ptr<double>> MagDataPool;
        vector<double> AudioDataPool;
        double GyroLastCpy[3][3];
        double GyroLast[3][3];
        outlierStatus_t Outlier_Detect_Status;
        int Outlier_ExtData_Count;

        void strstrip(char* const str);

        int charToByte(char c);

        int clearBuffer(dataArray_t* const array);

        uint8_t getValueFromBuffer(const dataArray_t* const array, const int index);

        int addDataInBuffer(dataArray_t* const array, const uint8_t data);

        int updateDataBuffer(dataArray_t* const array, const int index);

        void resetParserStatus();

        int checkDataFrame(const dataArray_t* const array);

        void outlierDataProcess(double gyroLast[][3], vector<shared_ptr<double>>& gyroDataPool);

        void sensorFusionEntry(double fGyro[], double fAcc[], double fMag[], double fAudio);

    public:
        BleDataParser();

        void parserReceivedData(const char* const strData);

        int processDataFrame(const dataArray_t* const array);
};

#endif //AHRS_C_PCSIM_GD_PASER_H
