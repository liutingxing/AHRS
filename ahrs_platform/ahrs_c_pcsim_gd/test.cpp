//
// Created by jiangtianyu on 2018/9/19.
//
#include <stdio.h>


#define  MAX_BUFF_LEN	1024

int main(int argc,char *argv[])
{
    FILE *fpInput;
    FILE *fpOutput;
    char line[MAX_BUFF_LEN];

    fpInput = fopen("../data/rawdata.txt", "r");
    fpOutput = fopen("../data/output.txt", "w");

    if (fpInput == NULL)
    {
        printf("open file failed!\r\n");
        return -1;
    }

    while ( fgets(line, MAX_BUFF_LEN, fpInput) != 0 )
    {
        puts(line);
    }

    fclose(fpInput);
    fclose(fpOutput);

    return 0;
}
