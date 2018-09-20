//
// Created by jiangtianyu on 2018/9/19.
//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include "parser.h"

#define MaxBufferSize   1024

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

static dataArray_t DataBuffer;
static dataArray_t DataBufferFiltered;
static parserStatus_t ParserStatus = DelimiterDetect;
static int ParserReceiveLength = 0;
static int ParserReadIndex = 0;
static int ParserDelimiterIndex = 0;

static void strstrip(char* const str)
{
    char* src = str;
    char* dest = str;

    while(*src != '\0')
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

static int charToByte(char c)
{
    char str[] = "0123456789ABCDEF";
    char *p = str;
    int index = 0;

    while(p)
    {
        if(*p != c)
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

static int clearBuffer(dataArray_t* const array)
{
    array->size = 0;

    return 0;
}

static uint8_t getValueFromBuffer(const dataArray_t* const array, const int index)
{
    return array->buffer[index];
}

static int addDataInBuffer(dataArray_t* const array, const uint8_t data)
{
    if (array->size >= MaxBufferSize)
    {
        return -1;
    }

    array->buffer[array->size] = data;
    array->size++;

    return 0;
}

static int updateDataBuffer(dataArray_t* const array, const int index)
{
    if (index < 0 || index > array->size) {
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

static void resetParserStatus()
{
    clearBuffer(&DataBufferFiltered);
    ParserDelimiterIndex = 0;
    ParserReadIndex = 0;
    ParserReceiveLength = 0;
    ParserStatus = DelimiterDetect;
}

static int checkDataFrame(const dataArray_t* const array)
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
    switch(type)
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
        sum += (uint32_t)getValueFromBuffer(&DataBufferFiltered, 4+i);
    }
    if (checksum != (uint8_t)sum)
    {
        return 0;
    }
    printf("seq = %d\n", (int)seq);

    return 1;
}

static int processDataFrame(const dataArray_t* const array)
{
    return 1;
}

void parserReceivedData(const char* const strData)
{
    char str[MaxBufferSize];

    if (strData == NULL)
        return;
    strcpy(str, strData);
    // remove space and return character
    strstrip(str);
    // convert char to byte into data buffer
    for (int i = 0; i < strlen(str)/2; i++)
    {
        int pos = i*2;
        uint8_t val = (uint8_t)(charToByte(str[pos]) << 4 | charToByte(str[pos+1]));
        addDataInBuffer(&DataBuffer, val);
    }

    // parser state machine
    while(ParserReadIndex < DataBuffer.size)
    {
        uint8_t charater = DataBuffer.buffer[ParserReadIndex];

        ParserReadIndex++;
        switch(ParserStatus)
        {
            case DelimiterDetect:
                if (charater == (uint8_t)0xAA) {
                    ParserDelimiterIndex = ParserReadIndex;
                    ParserStatus = DataSave;
                    addDataInBuffer(&DataBufferFiltered, charater);
                }
                break;
            case DataSave:
                addDataInBuffer(&DataBufferFiltered, charater);
                if (DataBufferFiltered.size == 4) {
                    // sof type seq len payload checksum eof
                    int8_t payloadLength = (int8_t) getValueFromBuffer(&DataBufferFiltered, 3);
                    if (payloadLength < 0) {
                        // wrong frame
                        // remove the first delimiter(0xAA)
                        updateDataBuffer(&DataBuffer, ParserDelimiterIndex);
                        // reset parser status
                        resetParserStatus();
                        continue;
                    }
                    ParserReceiveLength = payloadLength + 6;
                }
                if (ParserReceiveLength > 0 && DataBufferFiltered.size >= ParserReceiveLength) {
                    // check frame
                    if (checkDataFrame(&DataBufferFiltered)) {
                        // frame match succeed
                        // remove the match frame from buffer
                        updateDataBuffer(&DataBuffer, ParserReadIndex);
                        // process data array
                        processDataFrame(&DataBufferFiltered);
                    } else {
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



