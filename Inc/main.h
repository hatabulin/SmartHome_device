/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "cmsis_os.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ENC_BUTTON_UI_Pin GPIO_PIN_13
#define ENC_BUTTON_UI_GPIO_Port GPIOC
#define SENSOR5_Pin GPIO_PIN_0
#define SENSOR5_GPIO_Port GPIOC
#define SENSOR6_Pin GPIO_PIN_1
#define SENSOR6_GPIO_Port GPIOC
#define IR_DOOR_SENSOR_Pin GPIO_PIN_2
#define IR_DOOR_SENSOR_GPIO_Port GPIOC
#define ENC_A_DOOR_Pin GPIO_PIN_3
#define ENC_A_DOOR_GPIO_Port GPIOC
#define ENC_B_DOOR_Pin GPIO_PIN_0
#define ENC_B_DOOR_GPIO_Port GPIOA
#define ENC_A_UI_Pin GPIO_PIN_1
#define ENC_A_UI_GPIO_Port GPIOA
#define SENSOR7_Pin GPIO_PIN_2
#define SENSOR7_GPIO_Port GPIOA
#define IR_REMOTE_Pin GPIO_PIN_3
#define IR_REMOTE_GPIO_Port GPIOA
#define TOPLIGHT_SW_Pin GPIO_PIN_4
#define TOPLIGHT_SW_GPIO_Port GPIOA
#define TM1638_CLK_Pin GPIO_PIN_5
#define TM1638_CLK_GPIO_Port GPIOA
#define ENC_B_UI_Pin GPIO_PIN_6
#define ENC_B_UI_GPIO_Port GPIOA
#define TM1638_DIO_Pin GPIO_PIN_7
#define TM1638_DIO_GPIO_Port GPIOA
#define SX1278_RST_Pin GPIO_PIN_4
#define SX1278_RST_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOC
#define TM1638_STB_Pin GPIO_PIN_0
#define TM1638_STB_GPIO_Port GPIOB
#define SENSOR8_Pin GPIO_PIN_1
#define SENSOR8_GPIO_Port GPIOB
#define SENSOR7B2_Pin GPIO_PIN_2
#define SENSOR7B2_GPIO_Port GPIOB
#define USB_EN_Pin GPIO_PIN_12
#define USB_EN_GPIO_Port GPIOB
#define RGB_CH1_Pin GPIO_PIN_6
#define RGB_CH1_GPIO_Port GPIOC
#define RGB_CH2_Pin GPIO_PIN_7
#define RGB_CH2_GPIO_Port GPIOC
#define RGB_CH3_Pin GPIO_PIN_8
#define RGB_CH3_GPIO_Port GPIOC
#define HX711_DAT_Pin GPIO_PIN_9
#define HX711_DAT_GPIO_Port GPIOC
#define HX711_SCK_Pin GPIO_PIN_8
#define HX711_SCK_GPIO_Port GPIOA
#define KU5590_EXTI_Pin GPIO_PIN_15
#define KU5590_EXTI_GPIO_Port GPIOA
#define RELAY8_Pin GPIO_PIN_10
#define RELAY8_GPIO_Port GPIOC
#define RELAY9_Pin GPIO_PIN_11
#define RELAY9_GPIO_Port GPIOC
#define RELAY6_Pin GPIO_PIN_12
#define RELAY6_GPIO_Port GPIOC
#define RELAY7_Pin GPIO_PIN_2
#define RELAY7_GPIO_Port GPIOD
#define RELAY4_Pin GPIO_PIN_3
#define RELAY4_GPIO_Port GPIOB
#define RELAY5_Pin GPIO_PIN_4
#define RELAY5_GPIO_Port GPIOB
#define RELAY3_Pin GPIO_PIN_5
#define RELAY3_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_8
#define RELAY1_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_9
#define RELAY2_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define ESP8266
//#define KU5590
//#define RC522
#define _DEBUG
//#define SX1278_MODULE

#define MAX_BUF 128
#define LED GPIOC,GPIO_PIN_13
#define SOUND_PIN GPIOB, GPIO_PIN_0
#define MAX_CLAPS_COUNTER 5
#define MAX_ELAPSED_TIME 5
#define CLAPS_COUNTER_POS 8
#define CNT_OLED_TIMEOUT 5000
#define TIME_MAX_SX1278 1500
#define TIME_MAX_DEFAULT 2000
#define TIME_MAX_ENCODER 1000
#define TIME_MAX_KEYBOARD 2500
#define BRIGHT_DAY 2
#define OFF		1
#define ON		0
#define ZERO	0
//#define OFF 0
//#define ON 1
#define IR_DOOR_SENSOR HAL_GPIO_ReadPin(IR_DOOR_SENSOR_GPIO_Port,IR_DOOR_SENSOR_Pin)
#define ENC_A HAL_GPIO_ReadPin(ENC_A_GPIO_Port,ENC_A_Pin)
#define ENC_B HAL_GPIO_ReadPin(ENC_B_GPIO_Port,ENC_B_Pin)
//
#define BLUE   TIM3->CCR3
#define RED  TIM3->CCR1
#define GREEN  TIM3->CCR2

// defines poort devices names
#define TOP_LIGHT_PIN 		1
#define LAMP_PIN			2
#define SUBWOOFER_PIN 		3
#define MONITOR_PIN			4
#define PRINTER_PIN 		5
#define HARDWARE_TABLE_PIN 	6
#define SMARTBOX_LED1_PIN 	7
#define SMARTBOX_LED2_PIN	8
#define RESERVED3_PIN		9
//
// eeprom section
#define EE_values			1
#define EEP_CHECK_ADDR		10
#define CHECK_DATA			0x888
#define DEFAULT_GPIO_STATUS	0x00
//
// USB flags
#define GET_REGISTER_DATA_FROM_HOST	0x01
#define SEND_REGISTER_DATA_TO_HOST	0x02
#define SAVE_CFG1		0x03
#define SAVE_CFG2		0x04
#define LOAD_CFG1		0x05
#define LOAD_CFG2		0x06
// REGISTERS
#define REG_CMD_TIMER_RELOAD			0x01
#define REG_CMD_GPIO_STATUS				0x02 // set all new data to GPIO
#define REG_CMD_ENC0_COUNTER			0x03 // set encoder[0] data
#define REG_CMD_ENC1_COUNTER			0x04 // set encoder[0] data
#define REG_CMD_SLEEP_MASK				0x05
#define REG_CMD_HOT_BUTTON_MASK			0x06
#define REG_CMD_SET_BITS_GPIO_STATUS 	0x07
#define REG_CMD_RESET_BITS_GPIO_STATUS 	0x08
#define REG_CMD_IR_DOOR_STATUS 			0x09
#define REG_CMD_SET_FLAGS				0x0A
#define REG_CMD_GET_SENSORS				0x0B
//
#define IR_DOOR_SENSOR_INDEX	6
#define MENS_IN_ROOM_INDEX		3
#define TIMEOUT_SLEEPMODE_INDEX	1
#define IR_DOOR_SENSOR_INDEX	6
#define ENCODER0_INDEX			3
#define ENCODER1_INDEX			7

#define TM1638_SLEEP_MASK		4
#define TM1638_HOT_BUTTON_MASK	5
#define TM1638_STATUS			2

#define SLEEP_WITH_MASK			1
#define SLEEP_WITHOUT_MASK		0

#define ACTIVATE_HOT_BUTTON		1
#define DEACTIVATE_HOT_BUTTON	0

// UART
#define MAXSTRING				50
#define UART_PACKET_OK 0
#define UART_PACKET_TOO_LONG 1
#define UART_PACKET_CORRUPT 2
#define UART_BUSY 1
#define UART_CPLT 0

//
// RC522 section
#define GPIO_IRQ_Pin	1
//#define BUTTON_IRQ_Pin 2
void beep(uint16_t);
void CallBackDMA(void);
void ShowDateTime();
void RestoreFromSleep();
void start_title(uint8_t);
void ChangeCurrentGpioStatus(uint8_t command);
void IrdaToSd1306(uint32_t data);
void DynamicBrightness(uint8_t bright);
void StartKeyboardTask(void const * argument);
void StartSx1278Task(void const * argument);
void StartEncoderTask(void const * argument );
void StartIRremoteTask(void const * argument );
void esp8266SyncTask(void const * argument );
void ReadConstantsFromEEP(void);
void WriteDefaultsConstantToEEP(void);
void EEPROM_WriteValues (void);
void UsbFlagProcess(void);
void UART_CommandProcessor();
void UART_Send (const char message[]);
int uart3_get_char(portTickType block_time);
uint8_t TranslateFlagsToByte();
void TranslateByteToFlags(uint8_t reg_data);

typedef struct BeepFlagSettings {
	bool encoder0;
	bool encoder1;
	bool keyboard;
	bool remote;
	bool uart_incoming_packet;
	bool usb_incoming_packet;
	bool ir_door_sensor;
} BeepFlagSettings;

typedef struct Flags {
	uint8_t flag_usb;
	bool flag_timer4_counter;
	bool flag_sleep_mode_activated;
	bool flag_sleep_mode_remote;
	bool flag_user_sleep_mode;
	bool flag_new_clap;
	bool flag_oled_timeout;
	bool last_ir_door_sensor;
} Flags;

typedef struct GeneralValues {
	uint8_t current_gpio_status;
	uint8_t last_gpio_status;
	uint8_t sleep_mode_gpio_mask;
	uint8_t hot_button_gpio_mask;
	uint8_t tm1638_start_up; //
	bool use_door_sensor;
} GeneralValues;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
