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
    double fAudio;
    double ftemp;
    uint8_t type = getValueFromBuffer(array, 1);
    string sAttitude;

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

            sensorFusion.uTime++;
            sAttitude = sensorFusion.sensorFusionExec(sensorFusion.uTime, fGyro, fAcc, fMag, fAudio);

            //Todo: remove it if integrated in iOS
            extern FILE* fpOutput;

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

