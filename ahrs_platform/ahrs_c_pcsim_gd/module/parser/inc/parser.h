//
// Created by jiangtianyu on 2018/9/19.
//

#ifndef AHRS_C_PCSIM_GD_PASER_H
#define AHRS_C_PCSIM_GD_PASER_H

typedef		signed char			int8_t;                  /* Signed 8 bits integer    */
typedef		unsigned char			uint8_t;                 /* Unsigned 8 bits integer  */
typedef		signed short			int16_t;                 /* Signed 16 bits integer   */
typedef		unsigned short		uint16_t;                /* Unsigned 16 bits integer */
typedef		signed int				int32_t;                 /* Signed 32 bits integer   */
typedef		unsigned int			uint32_t;                /* Unsigned 32 bits integer */

#define  ACC_SENSITIVITY    (1.0/2048)
#define  GYRO_SENSITIVITY   (1.0/16.4)
#define  MAG_SENSITIVITY    (0.6)

void parserReceivedData(const char* const strData);

#endif //AHRS_C_PCSIM_GD_PASER_H
