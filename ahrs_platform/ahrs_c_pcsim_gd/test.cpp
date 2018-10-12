//
// Created by jiangtianyu on 2018/9/19.
//
#include <iostream>
#include <cstdio>
#include "parser.h"


#define  MAX_BUFF_LEN   1024

FILE* fpOutput;

int main(int argc, char* argv[])
{
    FILE* fpInput;
    char line[MAX_BUFF_LEN];
    BleDataParser bleDataParser;

    fpInput = fopen("../data/rawdata.txt", "r");
    fpOutput = fopen("../data/output.txt", "w");

    if (fpInput == NULL)
    {
        std::cout << "open file failed!" << std::endl;
        return -1;
    }

    while (fgets(line, MAX_BUFF_LEN, fpInput) != 0)
    {
        std::cout << line << std::endl;
        bleDataParser.parserReceivedData(line);
    }

    fclose(fpInput);
    fclose(fpOutput);

    return 0;
}
