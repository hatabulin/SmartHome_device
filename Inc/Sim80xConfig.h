#ifndef	_SIM80XCONF_H
#define	_SIM80XCONF_H

#include "stm32f1xx_hal.h"

//UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
//DMA_HandleTypeDef hdma_usart2_tx;

//	0: No DEBUG				1:High Level Debug .Use printf		2:All RX Data.Use printf

#define	_SIM80X_DEBUG				        2

#define	_SIM80X_USART				        huart3

#define	_SIM80X_USE_POWER_KEY   	  0

#define	_SIM80X_BUFFER_SIZE			    512

#define _SIM80X_DMA_TRANSMIT        1

#define _SIM80X_USE_BLUETOOTH       0
#define _SIM80X_USE_GPRS            0


#define	_SIM80X_POWER_KEY_GPIO		  GPIOB
#define	_SIM80X_POWER_KEY_PIN		    9

#endif
