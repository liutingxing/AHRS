//
// Created by jiangtianyu on 2018/9/19.
//
#include "parser.h"

BleDataParser::BleDataParser()
{
    // clear buffer
    DataBuffer.size = 0;
    memset(DataBuffer.buffer, 0, sizeof(DataBuffer.buffer));

    DataBufferFiltered.size = 0;
    memset(DataBufferFiltered.buffer, 0, sizeof(DataBufferFiltered.buffer));

    // reset the variable
    ParserStatus = DelimiterDetect;
    ParserReceiveLength = 0;
    ParserReadIndex = 0;
    ParserDelimiterIndex = 0;

    Outlier_Detect_Status = Outlier_Peace;
}

void BleDataParser::strstrip(char* const str)
{
    char* src = str;
    char* dest = str;

    while (*src != '\0')
    {
        if (*src != ' ' && *src != '\n')
        {
            *dest++ = *src++;
        }
        else
        {
            src++;
        }
    }

    *dest = '\0';
}

int BleDataParser::charToByte(char c)
{
    char str[] = "0123456789ABCDEF";
    char* p = str;
    int index = 0;

    while (p)
    {
        if (*p != c)
        {
            p++;
            index++;
        }
        else
        {
            return index;
        }
    }

    return -1;
}

int BleDataParser::clearBuffer(dataArray_t* const array)
{
    array->size = 0;

    return 0;
}

uint8_t BleDataParser::getValueFromBuffer(const dataArray_t* const array, const int index)
{
    return array->buffer[index];
}

int BleDataParser::addDataInBuffer(dataArray_t* const array, const uint8_t data)
{
    if (array->size >= MaxBufferSize)
    {
        return -1;
    }

    array->buffer[array->size] = data;
    array->size++;

    return 0;
}

int BleDataParser::updateDataBuffer(dataArray_t* const array, const int index)
{
    if (index < 0 || index > array->size)
    {
        return -1;
    }

    if (index == array->size)
    {
        array->size = 0;
    }
    else
    {
        for (int i = index; i < array->size; i++)
        {
            array->buffer[i - index] = array->buffer[i];
        }

        array->size -= index;
    }

    return 0;
}

void BleDataParser::resetParserStatus()
{
    clearBuffer(&DataBufferFiltered);
    ParserDelimiterIndex = 0;
    ParserReadIndex = 0;
    ParserReceiveLength = 0;
    ParserStatus = DelimiterDetect;
}

int BleDataParser::checkDataFrame(const dataArray_t* const array)
{
    uint8_t type = getValueFromBuffer(&DataBufferFiltered, 1);
    uint8_t seq = getValueFromBuffer(&DataBufferFiltered, 2);
    uint8_t length = getValueFromBuffer(&DataBufferFiltered, 3);
    uint8_t eof = getValueFromBuffer(&DataBufferFiltered, ParserReceiveLength - 1);
    uint8_t checksum = getValueFromBuffer(&DataBufferFiltered, ParserReceiveLength - 2);
    uint32_t sum = 0;

    // check end marker of frame
    if (eof != (uint8_t)0x55)
    {
        return 0;
    }

    // check data type and data length
    switch (type)
    {
    // sensor data
    case (uint8_t)0x01:
        if (length != (uint8_t)0x14)
        {
            return 0;
        }

        break;

    // power data
    case (uint8_t)0x82:
        if (length != (uint8_t)0x01)
        {
            return 0;
        }

        break;

    default:
        return 0;
    }

    // check sum validation
    sum = (uint32_t)type + (uint32_t)seq + (uint32_t)length;

    for (int i = 0; i < length; i++)
    {
        sum += (uint32_t)getValueFromBuffer(&DataBufferFiltered, 4 + i);
    }

    if (checksum != (uint8_t)sum)
    {
        return 0;
    }

    //std::cout << "seq = " << (int)seq << std::endl;

    return 1;
}

int BleDataParser::processDataFrame(const dataArray_t* const array)
{
    double fAcc[3];
    double fGyro[3];
    double fMag[3];
    double fGyroCpy[3];
    double fAudio;
    double ftemp;
    uint8_t type = getValueFromBuffer(array, 1);

    switch (type)
    {
    case (uint8_t)0x01:
        {
            // sensor data
            // acc data
            uint8_t accxH = getValueFromBuffer(array, 4);
            uint8_t accxL = getValueFromBuffer(array, 5);
            uint8_t accyH = getValueFromBuffer(array, 6);
            uint8_t accyL = getValueFromBuffer(array, 7);
            uint8_t acczH = getValueFromBuffer(array, 8);
            uint8_t acczL = getValueFromBuffer(array, 9);
            // gyro data
            uint8_t gyroxH = getValueFromBuffer(array, 10);
            uint8_t gyroxL = getValueFromBuffer(array, 11);
            uint8_t gyroyH = getValueFromBuffer(array, 12);
            uint8_t gyroyL = getValueFromBuffer(array, 13);
            uint8_t gyrozH = getValueFromBuffer(array, 14);
            uint8_t gyrozL = getValueFromBuffer(array, 15);
            // mag data
            uint8_t magxH = getValueFromBuffer(array, 16);
            uint8_t magxL = getValueFromBuffer(array, 17);
            uint8_t magyH = getValueFromBuffer(array, 18);
            uint8_t magyL = getValueFromBuffer(array, 19);
            uint8_t magzH = getValueFromBuffer(array, 20);
            uint8_t magzL = getValueFromBuffer(array, 21);
            // audio data
            uint8_t audioH = getValueFromBuffer(array, 22);
            uint8_t audioL = getValueFromBuffer(array, 23);

            fAcc[0] = (int16_t)(accxH << 8 | accxL) * ACC_SENSITIVITY * sensorFusion.GRAVITY;  // m/s2
            fAcc[1] = (int16_t)(accyH << 8 | accyL) * ACC_SENSITIVITY * sensorFusion.GRAVITY;  // m/s2
            fAcc[2] = (int16_t)(acczH << 8 | acczL) * ACC_SENSITIVITY * sensorFusion.GRAVITY;  // m/s2
            fGyro[0] = (int16_t)(gyroxH << 8 | gyroxL) * GYRO_SENSITIVITY * DEG2RAD;  // rad/s
            fGyro[1] = (int16_t)(gyroyH << 8 | gyroyL) * GYRO_SENSITIVITY * DEG2RAD;  // rad/s
            fGyro[2] = (int16_t)(gyrozH << 8 | gyrozL) * GYRO_SENSITIVITY * DEG2RAD;  // rad/s

            fMag[0] = (int16_t)(magxH << 8 | magxL) * MAG_SENSITIVITY;  // uT
            fMag[1] = (int16_t)(magyH << 8 | magyL) * MAG_SENSITIVITY;  // uT
            fMag[2] = (int16_t)(magzH << 8 | magzL) * MAG_SENSITIVITY;  // uT

            fAudio = (int16_t)(audioH << 8 | audioL);  // count

            // sensor hal: sensor frame(xyz) -> device frame(XYZ)
            // gyro/acc: X = x, Y = -y, Z = -z
            fAcc[1] = -fAcc[1];
            fAcc[2] = -fAcc[2];
            fGyro[1] = -fGyro[1];
            fGyro[2] = -fGyro[2];
            // mag: X = y, Y = -x, Z = z
            ftemp = -fMag[0];
            fMag[0] = fMag[1];
            fMag[1] = ftemp;

            // copy the raw gyro data
            fGyroCpy[0] = fGyro[0];
            fGyroCpy[1] = fGyro[1];
            fGyroCpy[2] = fGyro[2];

            // outlier data detection
#if !Data_Compensate
            sensorFusionEntry(fGyro, fAcc, fMag, fAudio);
#else
        switch (Outlier_Detect_Status)
                    {
                        case Outlier_Peace:
                            if (abs(fGyro[CHX]) > (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD ||
                                abs(fGyro[CHY]) > (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD ||
                                abs(fGyro[CHZ]) > (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD)
                            {
                                // outlier data happens
                                for (int j = CHX; j <= CHZ; j++)
                                {
                                    GyroLastCpy[0][j] = GyroLast[0][j];
                                    GyroLastCpy[1][j] = GyroLast[1][j];
                                    GyroLastCpy[2][j] = GyroLast[2][j];
                                }
                                GyroDataPool.push_back(shared_ptr<double>(new double[3] {fGyro[CHX], fGyro[CHY], fGyro[CHZ]}, [](double * p)
                                {
                                    delete[] p;
                                }));
                                AccDataPool.push_back(shared_ptr<double>(new double[3] {fAcc[CHX], fAcc[CHY], fAcc[CHZ]}, [](double * p)
                                {
                                    delete[] p;
                                }));
                                MagDataPool.push_back(shared_ptr<double>(new double[3] {fMag[CHX], fMag[CHY], fMag[CHZ]}, [](double * p)
                                {
                                    delete[] p;
                                }));
                                AudioDataPool.push_back(fAudio);
                                Outlier_Detect_Status = Outlier_Start;
                            }
                            else
                            {
                                sensorFusionEntry(fGyro, fAcc, fMag, fAudio);
                            }
                            break;

                        case Outlier_Start:
                            GyroDataPool.push_back(shared_ptr<double>(new double[3] {fGyro[CHX], fGyro[CHY], fGyro[CHZ]}, [](double * p)
                            {
                                delete[] p;
                            }));
                            AccDataPool.push_back(shared_ptr<double>(new double[3] {fAcc[CHX], fAcc[CHY], fAcc[CHZ]}, [](double * p)
                            {
                                delete[] p;
                            }));
                            MagDataPool.push_back(shared_ptr<double>(new double[3] {fMag[CHX], fMag[CHY], fMag[CHZ]}, [](double * p)
                            {
                                delete[] p;
                            }));
                            AudioDataPool.push_back(fAudio);
                            if (abs(fGyro[CHX]) < (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD &&
                                abs(fGyro[CHY]) < (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD &&
                                abs(fGyro[CHZ]) < (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD)
                            {
                                // outlier data disappear
                                Outlier_Detect_Status = Outlier_End;
                                Outlier_ExtData_Count = 1;
                            }
                            break;

                        case Outlier_End:
                            GyroDataPool.push_back(shared_ptr<double>(new double[3] {fGyro[CHX], fGyro[CHY], fGyro[CHZ]}, [](double * p)
                            {
                                delete[] p;
                            }));
                            AccDataPool.push_back(shared_ptr<double>(new double[3] {fAcc[CHX], fAcc[CHY], fAcc[CHZ]}, [](double * p)
                            {
                                delete[] p;
                            }));
                            MagDataPool.push_back(shared_ptr<double>(new double[3] {fMag[CHX], fMag[CHY], fMag[CHZ]}, [](double * p)
                            {
                                delete[] p;
                            }));
                            AudioDataPool.push_back(fAudio);
                            Outlier_ExtData_Count++;
                            if (Outlier_ExtData_Count >= 3)
                            {
                                // start process outlier data
                                if (Data_Compensate)
                                {
                                    outlierDataProcess(GyroLastCpy, GyroDataPool);
                                }

                                // restart the sensor fusion entry
                                for (int i = 0; i < GyroDataPool.size(); i++)
                                {
                                    sensorFusionEntry(GyroDataPool.at(i).get(), AccDataPool.at(i).get(), MagDataPool.at(i).get(), AudioDataPool.at(i));
                                }

                                // clear the status
                                GyroDataPool.clear();
                                AccDataPool.clear();
                                MagDataPool.clear();
                                AudioDataPool.clear();
                                Outlier_Detect_Status = Outlier_Peace;
                            }

                    }
                    // record the last gyro data
                    for (int i = CHX; i <= CHZ; i++)
                    {
                        GyroLast[0][i] = GyroLast[1][i];
                        GyroLast[1][i] = GyroLast[2][i];
                        GyroLast[2][i] = fGyroCpy[i];
                    }
#endif
            break;
        }

    case (uint8_t)0x82:
        {
            // power data
            uint8_t power = getValueFromBuffer(array, 4);
            break;
        }
    }

    return 1;
}

void BleDataParser::outlierDataProcess(double gyroLast[][3], vector<shared_ptr<double>>& gyroDataPool)
{
    int left_index[3] = {-1, -1, -1};
    int right_index[3] = {-1, -1, -1};
    int index;
    bool outlier_flag[3] = {false, false, false};

    /* determine the outlier data boundary */
    for (int i = CHX; i <= CHZ; i++) {
        index = 0;
        for (auto p : gyroDataPool) {
            double* val = p.get();
            if (left_index[i] == -1) {
                if (abs(val[i]) > (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD) {
                    left_index[i] = index;
                }
            } else {
                if (right_index[i] == -1) {
                    if (abs(val[i]) < (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD) {
                        right_index[i] = index - 1;
                        outlier_flag[i] = true;
                    }
                }
            }

            if (outlier_flag[i]) {
                break;
            }
            index++;
        }
    }

    /* estimate the outlier data */
    for (int channel = CHX; channel <= CHZ; channel++)
    {
        if (outlier_flag[channel])
        {
            double x0[6];
            double y0[6];
            Eigen::VectorXd xvals(6);
            Eigen::VectorXd yvals(xvals.rows());
            int seq_left = left_index[channel] - 1;
            int seq_right = right_index[channel] + 1;

            x0[0] = seq_left - 2;
            x0[1] = seq_left - 1;
            x0[2] = seq_left;
            x0[3] = seq_right;
            x0[4] = seq_right + 1;
            x0[5] = seq_right + 2;

            for (int i = 0; i < 6; i++) {
                if (x0[i] < 0)
                {
                    y0[i] = gyroLast[(int)(x0[i] + 3)][channel];
                }
                else
                {
                    y0[i] = gyroDataPool.at((int)x0[i]).get()[channel];
                }
            }

            xvals << x0[0], x0[1], x0[2], x0[3], x0[4], x0[5];
            yvals << y0[0], y0[1], y0[2], y0[3], y0[4], y0[5];
            SplineFunction s(xvals, yvals);
            for (int i = left_index[channel]; i <= right_index[channel]; i++) {
                gyroDataPool.at(i).get()[channel] = s(i);
            }
        }
    }
}

void BleDataParser::sensorFusionEntry(double fGyro[], double fAcc[], double fMag[], double fAudio)
{
    string sAttitude;

    sensorFusion.uTime++;
    //Todo: remove it if integrated in iOS
    extern FILE* fpGyroRaw;
    fprintf(fpGyroRaw, "%d, %f, %f, %f\n", sensorFusion.uTime, fGyro[CHX], fGyro[CHY], fGyro[CHZ]);
    //Todo: remove it if integrated in iOS
    sAttitude = sensorFusion.sensorFusionExec(sensorFusion.uTime, fGyro, fAcc, fMag, fAudio);

    //Todo: remove it if integrated in iOS
    extern FILE* fpOutput;
    extern FILE* fpGyroCali;
    fprintf(fpGyroCali, "%d, %f, %f, %f\n", sensorFusion.uTime, fGyro[CHX], fGyro[CHY], fGyro[CHZ]);
    if (!sAttitude.empty())
    {
        fputs(sAttitude.c_str(), fpOutput);
        fputs("\r\n", fpOutput);
    }

    if (sensorFusion.uActionComplete == true)
    {
        cout << "-------------------------------------------------------------------------------------------------------------------------------------\n"
             << sensorFusion.trainData.sTrajectory << "\n" <<
             "-------------------------------------------------------------------------------------------------------------------------------------\n" << endl;
    }

    //Todo: remove it if integrated in iOS
}

void BleDataParser::parserReceivedData(const char* const strData)
{
    char str[MaxBufferSize];

    if (strData == NULL)
    {
        return;
    }

    strcpy(str, strData);
    // remove space and return character
    strstrip(str);

    // convert char to byte into data buffer
    for (int i = 0; i < strlen(str) / 2; i++)
    {
        int pos = i * 2;
        uint8_t val = (uint8_t)(charToByte(str[pos]) << 4 | charToByte(str[pos + 1]));
        addDataInBuffer(&DataBuffer, val);
    }

    // parser state machine
    while (ParserReadIndex < DataBuffer.size)
    {
        uint8_t charater = DataBuffer.buffer[ParserReadIndex];

        ParserReadIndex++;

        switch (ParserStatus)
        {
        case DelimiterDetect:
            if (charater == (uint8_t)0xAA)
            {
                ParserDelimiterIndex = ParserReadIndex;
                ParserStatus = DataSave;
                addDataInBuffer(&DataBufferFiltered, charater);
            }

            break;

        case DataSave:
            addDataInBuffer(&DataBufferFiltered, charater);

            if (DataBufferFiltered.size == 4)
            {
                // sof type seq len payload checksum eof
                int8_t payloadLength = (int8_t) getValueFromBuffer(&DataBufferFiltered, 3);

                if (payloadLength < 0)
                {
                    // wrong frame
                    // remove the first delimiter(0xAA)
                    updateDataBuffer(&DataBuffer, ParserDelimiterIndex);
                    // reset parser status
                    resetParserStatus();
                    continue;
                }

                ParserReceiveLength = payloadLength + 6;
            }

            if (ParserReceiveLength > 0 && DataBufferFiltered.size >= ParserReceiveLength)
            {
                // check frame
                if (checkDataFrame(&DataBufferFiltered))
                {
                    // frame match succeed
                    // remove the match frame from buffer
                    updateDataBuffer(&DataBuffer, ParserReadIndex);
                    // process data array
                    processDataFrame(&DataBufferFiltered);
                }
                else
                {
                    // frame match failed
                    // remove the first delimiter (0xAA)
                    updateDataBuffer(&DataBuffer, ParserDelimiterIndex);
                }

                //reset parser status
                resetParserStatus();
            }

            break;
        }
    }
}

