#ifndef __DRV_DELAY_H__
#define __DRV_DELAY_H__

#include "stm32f4xx.h"
#include "typedefine.h"

extern void drv_delay_init( void );
extern void drv_delay_ms( INT32U nms );
extern void drv_delay_us( INT32U nus );
extern void drv_delay( INT32U nCount );


#endif


