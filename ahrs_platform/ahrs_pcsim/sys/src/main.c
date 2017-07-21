#include <stdio.h>
#include <memory.h>
#include "types.h"
#include "sensor.h"


#define  MAX_BUFF_LEN	1024

int main(int argc,char *argv[])
{
	FILE *fp;
	U8 line[MAX_BUFF_LEN];

	fopen_s(&fp, "../../data/right_hand.txt", "r");
#ifdef DEBUG
    fopen_s(&FpOutput, "../../data/output.txt", "w");
#endif
	if (fp == NULL)
	{
		printf("open file failed!\r\n");
		return -1;
	}

    if (sensorNavInit())
    {
        return -1;
        printf("system init failed!\r\n");
    }

	while ( fgets(line, MAX_BUFF_LEN, fp) != 0 )
	{
        sensorData_t sensorData;
#ifdef DEBUG
        puts(line);
#endif
        memset(&sensorData, 0, sizeof(sensorData_t));
        if ( praseSensorData(line, &sensorData) )
        {
            printf("parsing failed!\r\n");
        }

        sensorNavExec(&sensorData);
    }
    fclose(fp);
    fclose(FpOutput);

	return 0;
}