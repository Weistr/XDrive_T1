#ifndef _XDCOM_H
#define _XDCOM_H

#include "main.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


#define SendBackPosSpeed 0x01
typedef __packed struct   
{
    uint8_t id;
    uint8_t mode;
    int32_t pos;
    int32_t speed;
    int16_t crc;
}xdriveComTypddef;



void xdUartRxIsr(char* rcvBuf,uint16_t len);

void xdUartTxIsr(void);


#endif
