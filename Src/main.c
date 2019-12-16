
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "rtc.h"
#include "stdbool.h"
#include "ssd1306.h"
#include "fonts.h"
#include "eeprom.h"
//#include "Sim80x.h"
//#include "Sim80xConfig.h"
#include "string.h"
#include "TM1638.h"
#include "RotaryEncoder.h"
#include "IRremote.h"
#include "transcend.h"
#ifdef RC522
#include <rc522.h>
#endif
#ifdef SX1278_MODULE
#include "SX1278.h"
#endif
//#include "main.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE {
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

osThreadId KeyboardTaskHandle;
osThreadId EncoderTaskHandle;
osThreadId StartIRremoteTaskHandle;
osThreadId SmartBoxHandle;
osThreadId esp8266SyncHandle;

osSemaphoreId myBinarySemTM1638Handle;
osSemaphoreId myBinarySemTM1638Handle2;
osSemaphoreId myBinarySemOledLcdHandle;
osSemaphoreId myBinarySemOledLcd2Handle1;
xSemaphoreHandle xSemaphoreUSART3;

uint8_t* str[50];
uint8_t relay_channels_map[9] = {TOP_LIGHT_PIN,LAMP_PIN,SUBWOOFER_PIN,MONITOR_PIN,PRINTER_PIN,HARDWARE_TABLE_PIN,SMARTBOX_LED1_PIN,SMARTBOX_LED2_PIN};
uint8_t key_pressed,old_key_pressed;
uint8_t status_task_sign;
uint8_t timer1_counter;
uint8_t timer4_counter;
uint8_t reg_number;
uint16_t reg_data;
uint16_t claps_counter;
uint16_t time_last;
uint16_t sleep_mode_timeout = 25;
uint32_t cnt_oled_timer;
uint32_t room_status[2];
RotaryEncoder encoder[3];
uint8_t time_elapsed[MAX_CLAPS_COUNTER];
bool UART_TX_Busy;

BeepFlagSettings beep_settings_flags;
Flags flags;
GeneralValues main_values;

const uint8_t Dig[4*2] = { 0x80,0x40,0x04,0x08,0x10,0x20,0x01,0x02 };
//
//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM7_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef ESP8266
void ESP8266_RxCallBack();
#endif
extern void	Sim80x_Init(osPriority Priority);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t hex2int(char *hex) {
    uint32_t val = 0;
    while (*hex) {
        // get current character then increment
        uint8_t byte = *hex++;
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}

uint8_t hex2int_byte(char *hex) {
    uint32_t val = 0;
    uint8_t i=0;
    while (i<2){
        // get current character then increment
        uint8_t byte = *hex++;
        i++;
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}

void beep(uint16_t i) {
	if (i>0) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET);
		HAL_Delay(i-1);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET);
		HAL_Delay(0);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
	}
}

void WriteRelayPort(uint8_t port_num,uint8_t state) {

	switch (port_num) {
	case 1: HAL_GPIO_WritePin(TOPLIGHT_SW_GPIO_Port,TOPLIGHT_SW_Pin,state);break;
//	case 1: HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,state);break;
	case 2: HAL_GPIO_WritePin(RELAY2_GPIO_Port,RELAY2_Pin,state);break;
	case 3: HAL_GPIO_WritePin(RELAY3_GPIO_Port,RELAY3_Pin,state);break;
	case 4: HAL_GPIO_WritePin(RELAY4_GPIO_Port,RELAY4_Pin,state);break;
	case 5: HAL_GPIO_WritePin(RELAY5_GPIO_Port,RELAY5_Pin,state);break;
	case 6: HAL_GPIO_WritePin(RELAY6_GPIO_Port,RELAY6_Pin,state);break;
	case 7: HAL_GPIO_WritePin(RELAY7_GPIO_Port,RELAY7_Pin,state);break;
	case 8: HAL_GPIO_WritePin(RELAY8_GPIO_Port,RELAY8_Pin,state);break;
	case 9: HAL_GPIO_WritePin(RELAY9_GPIO_Port,RELAY9_Pin,state);break;
	}
}
//
// EEPROM api functions
//
void ReadEEprom(void) {

	uint32_t data_;
	RTC_TimeTypeDef RTC_Time;

	EE_Read(EEP_CHECK_ADDR, &data_);
	if (data_!= CHECK_DATA) {
		beep(500);
		HAL_Delay(50);
		beep(100);
		HAL_Delay(50);
		beep(50);
		HAL_Delay(50);
		beep(50);

		EE_Format();
		RTC_Time.Hours = 0x16;//0x0;
		RTC_Time.Minutes = 0x45;//0x0;
		RTC_Time.Seconds = 0x0;

		if (HAL_RTC_SetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD) != HAL_OK) {
		    _Error_Handler(__FILE__, __LINE__);
		}

		WriteDefaultsConstantToEEP();
	 } else {
		 beep(10);
		 ReadConstantsFromEEP();
	 }
}

void WriteDefaultsConstantToEEP(void) {

	sleep_mode_timeout = 99;
	encoder[0].value = 1;
	encoder[1].value = ZERO;
	main_values.current_gpio_status = DEFAULT_GPIO_STATUS;
	main_values.sleep_mode_gpio_mask = 0b11111111;	//0xFF;
	main_values.hot_button_gpio_mask = 0b00000010;	//0x02;
	main_values.use_door_sensor = true;

	beep_settings_flags.encoder0 = true;
	beep_settings_flags.encoder1 = false;
	beep_settings_flags.keyboard = true;
	beep_settings_flags.remote = true;
	beep_settings_flags.uart_incoming_packet = true;
	beep_settings_flags.usb_incoming_packet = false;
	beep_settings_flags.ir_door_sensor = true;

	EEPROM_WriteValues ();
	EE_Write(EEP_CHECK_ADDR,CHECK_DATA);
}

void ReadConstantsFromEEP(void) {

	uint32_t data_;

	EE_Read(EE_values, &data_);
	encoder[1].value = (data_>>8) & 0xFF;
	encoder[0].value = (data_ >>16) & 0xFF;
	sleep_mode_timeout = (data_>>24) & 0xFF;

	EE_Read(EE_values+1, &data_);
	main_values.use_door_sensor = (bool)data_;
	main_values.hot_button_gpio_mask = (data_>>8) & 0xFF;
	main_values.sleep_mode_gpio_mask = (data_>>16) &0xFF;
	main_values.current_gpio_status = ZERO;

	EE_Read(EE_values+2, &data_);
	TranslateByteToFlags((uint8_t)data_);
}

void EEPROM_WriteValues () {

	uint32_t data_;

	data_ = TranslateFlagsToByte();
	EE_Write(EE_values + 2, data_);

	data_ = sleep_mode_timeout;
	data_<<= 8;
	data_ |= encoder[0].value;
	data_<<= 8;
	data_ |= encoder[1].value;
	data_<<= 8;
	data_ |= 0xFF;
	EE_Write(EE_values, data_);

	data_ = main_values.current_gpio_status;data_<<= 8;
	data_ |= main_values.sleep_mode_gpio_mask; data_ <<= 8;
	data_ |= main_values.hot_button_gpio_mask; data_ <<= 8;
	data_ |= 0xFF;
	EE_Write(EE_values+1, data_);
}
//
//
void ActivateGpioPorts(uint8_t value) { // in data: mask for used pins
	for (uint8_t used_device_number = 0;used_device_number<8;used_device_number++) {
		if ((value >> used_device_number) & 0x01) {
			WriteRelayPort(relay_channels_map[used_device_number],0); //ON_PCF);
			if (myBinarySemTM1638Handle != pdFALSE) {
				xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
				tm1638_LedSet(used_device_number+1, 1); // ON
				xSemaphoreGive(myBinarySemTM1638Handle );
			}
		}
		main_values.current_gpio_status |= value;
	}
}

void DeActivateGpioPorts(uint8_t value) { // in data: mask for used pins
	for (uint8_t used_device_number = 0;used_device_number<8;used_device_number++) {
		if ((value >> used_device_number) & 0x01) {
			WriteRelayPort(relay_channels_map[used_device_number],1); //OFF_PCF);
			if (myBinarySemTM1638Handle != pdFALSE) {
				xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
				tm1638_LedSet(used_device_number+1, 0 ); // OFF
				xSemaphoreGive(myBinarySemTM1638Handle );
			}
		}
		main_values.current_gpio_status &= ~value;
	}
}

// setting all pinouts equalent with tm1638_status value
//
void SyncGpioWithCurrentGpioStatus() {
	for (uint8_t used_device_number = 0;used_device_number<8;used_device_number++) {
		if ((main_values.current_gpio_status >> used_device_number) & 0x01) {
			WriteRelayPort(relay_channels_map[used_device_number],0); //ON_PCF pin);
			if (myBinarySemTM1638Handle != pdFALSE) {
				xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
				tm1638_LedSet(used_device_number+1, 1); // ON
				xSemaphoreGive(myBinarySemTM1638Handle );
			}
		} else
		{
			WriteRelayPort(relay_channels_map[used_device_number],1); //OFF_PCF pin);
			if (myBinarySemTM1638Handle != pdFALSE) {
				xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
				tm1638_LedSet(used_device_number+1, 0); // OFF
				xSemaphoreGive(myBinarySemTM1638Handle );
			}
		}
	}
}


// in: reg_data - flags status (bit0, bit1, ....)
//
void TranslateByteToFlags(uint8_t reg_data) {

	beep_settings_flags.encoder0 = reg_data & 0x01;
	beep_settings_flags.encoder1 = reg_data >> 1 & 0x01;
	beep_settings_flags.keyboard = reg_data >> 2 & 0x01;
	beep_settings_flags.remote =  reg_data >> 3 & 0x01;
	beep_settings_flags.uart_incoming_packet = reg_data >> 4 & 0x01;
	beep_settings_flags.usb_incoming_packet = reg_data  >> 5 & 0x01;
	beep_settings_flags.ir_door_sensor = reg_data >> 6 & 0x01;
}

uint8_t TranslateFlagsToByte() {

	uint8_t smarthome_beep_setting = 0;

	if (beep_settings_flags.encoder0) smarthome_beep_setting |= 0x01;
	if (beep_settings_flags.encoder1) smarthome_beep_setting |= 0x02;
	if (beep_settings_flags.keyboard) smarthome_beep_setting |= 0x04;
	if (beep_settings_flags.remote) smarthome_beep_setting |= 0x08;
	if (beep_settings_flags.uart_incoming_packet) smarthome_beep_setting |= 0x10;
	if (beep_settings_flags.usb_incoming_packet) smarthome_beep_setting |= 0x20;
	if (beep_settings_flags.ir_door_sensor) smarthome_beep_setting |= 0x40;

	return smarthome_beep_setting;
}

uint8_t TranslateSensorsGpioToByte() {

	uint8_t data;
	data = HAL_GPIO_ReadPin(IR_DOOR_SENSOR_GPIO_Port,IR_DOOR_SENSOR_Pin) & 0x01;
	return data;
}

//SendRegToHost(REG_CMD_GET_SENSORS,TranslateSensorsGpioToByte());

//
// Function 'SleepMode'
//
//
void SleepMode(uint8_t mode) { // mode 1 - off with mask, mode 2 - off all
	char string[25];

	uint8_t mask = main_values.sleep_mode_gpio_mask;
	main_values.last_gpio_status = main_values.current_gpio_status;

	if (mode == SLEEP_WITHOUT_MASK) mask = 0xFF;
	DeActivateGpioPorts( mask);
	main_values.current_gpio_status &= ~mask;
	for (uint8_t command = 0;command<8;command++) {
		if ((mask >> command) & 0x01) {
			if (myBinarySemTM1638Handle != pdFALSE) {
				xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY ); {
					tm1638_LedSet(command+1, 0 ); // ON
				}
				xSemaphoreGive(myBinarySemTM1638Handle );
			}
		}
	}
	snprintf(string, sizeof(string),"SLEEP_MODE");
	CDC_Transmit_FS((uint8_t*)string, strlen(string));
}
//
//
void RestoreFromSleep() {
	char string[25];

	main_values.current_gpio_status = main_values.last_gpio_status;
	ActivateGpioPorts(main_values.current_gpio_status);
	snprintf(string, sizeof(string),"NORMAL_MODE\n");
	CDC_Transmit_FS((uint8_t*)string, strlen(string));
}
//
void ShowTaskSign(uint8_t task_num) {

	uint8_t i,digit = 0;

	status_task_sign ^= (1<<(task_num-1));
	for (i=0;i<4;i++) digit  = digit | Dig[i*2 + ((status_task_sign >> i) & 0x01)];
	segmentN_Set2( 1, digit);
	osDelay(1);
}

void SendRegToHost(uint8_t reg, uint16_t value) {
	char str[20];
	snprintf(str,20,"WREG:%02X=%04X:",reg,value);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	CDC_Transmit_FS((uint8_t*)'\0', 0);
}

void SendRegToEsp8266(uint8_t reg, uint16_t value) {
	char str[20];
	snprintf(str,20,"WREG:%02X=%04X",reg,value);
}

void UsbFlagProcess(void)
{
	char str[100];
	switch (flags.flag_usb)
	{
		case SEND_REGISTER_DATA_TO_HOST:
			switch (reg_number) {
				case REG_CMD_TIMER_RELOAD:
					SendRegToHost(REG_CMD_TIMER_RELOAD,sleep_mode_timeout);
					break;
				case REG_CMD_GPIO_STATUS:
					SendRegToHost(REG_CMD_GPIO_STATUS,main_values.current_gpio_status);
					break;
				case REG_CMD_ENC0_COUNTER:
					SendRegToHost(REG_CMD_ENC0_COUNTER,encoder[0].value);
					break;
				case REG_CMD_SLEEP_MASK:
					SendRegToHost(REG_CMD_SLEEP_MASK,main_values.sleep_mode_gpio_mask);
					break;
				case REG_CMD_HOT_BUTTON_MASK:
					SendRegToHost(REG_CMD_HOT_BUTTON_MASK,main_values.hot_button_gpio_mask);
					break;
				flags.flag_usb = 0;
			}
		break;

		case GET_REGISTER_DATA_FROM_HOST:
			if (beep_settings_flags.usb_incoming_packet) beep(0);
			switch (reg_number) {
				case REG_CMD_TIMER_RELOAD:
					sleep_mode_timeout = reg_data; 	// REG:01
					break;
				case REG_CMD_GPIO_STATUS:
					main_values.current_gpio_status = reg_data;
					SyncGpioWithCurrentGpioStatus();
					break;
				case REG_CMD_ENC0_COUNTER:
					encoder[0].counter = encoder[0].value  = reg_data;
					if (myBinarySemTM1638Handle != pdFALSE) {
						xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
						if (encoder[0].value <10) {
							tm1638_Digit(encoder[0].value ,4);
							segmentN_Set2(4,0);
						}
						else tm1638_Digit(encoder[0].value ,3);
						xSemaphoreGive(myBinarySemTM1638Handle );
					}
					break;
				case REG_CMD_SLEEP_MASK:
					main_values.sleep_mode_gpio_mask = reg_data;
					break;
				case REG_CMD_HOT_BUTTON_MASK:
					main_values.hot_button_gpio_mask = reg_data;
					break;
				case REG_CMD_SET_BITS_GPIO_STATUS:
					main_values.current_gpio_status |= reg_data;
					SyncGpioWithCurrentGpioStatus();
					break;
				case REG_CMD_RESET_BITS_GPIO_STATUS:
					main_values.current_gpio_status &= ~reg_data;
					SyncGpioWithCurrentGpioStatus();
					break;
				case REG_CMD_SET_FLAGS:
					TranslateByteToFlags(reg_data);
					break;
			} // switch (RegNumber)
			snprintf(str,20,"REG:%02X write:OK",reg_number);
			CDC_Transmit_FS((uint8_t*)str, strlen(str));
		break; // case GET_REGISTER_DATA_FROM_HOST:

		case SAVE_CFG1:
			EEPROM_WriteValues();
			break;
		case LOAD_CFG1:
			snprintf(str,sizeof(str),"CONFIG1:%02X;%02X;%02X;%02X;%02X;%04X;%02X;%02X;%02X",
					1,
					main_values.current_gpio_status,main_values.sleep_mode_gpio_mask,main_values.hot_button_gpio_mask,
					encoder[0].value,encoder[1].value,
					sleep_mode_timeout,
					TranslateFlagsToByte(),
					TranslateSensorsGpioToByte());
			CDC_Transmit_FS((uint8_t*)str, strlen(str));
		break;
	} // end switch flag_usb
	flags.flag_usb = ZERO;
}

void ProcessHotButton(uint8_t mode)
{
	char str[10];

	if (mode == ACTIVATE_HOT_BUTTON) {
		ActivateGpioPorts(main_values.hot_button_gpio_mask);
		snprintf(str,10,"HOT_BUTTON:1");
		CDC_Transmit_FS((uint8_t*)str, strlen(str));
		CDC_Transmit_FS((uint8_t*)'\0', 0);
	}
	else {
		DeActivateGpioPorts(main_values.hot_button_gpio_mask); // 			sleep_mode_activated = true;
		snprintf(str,10,"HOT_BUTTON:0");
		CDC_Transmit_FS((uint8_t*)str, strlen(str));
		CDC_Transmit_FS((uint8_t*)'\0', 0);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_TIM7_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//  HAL_I2C_Init(&hi2c1);
  //if(!(*(volatile uint32_t *) (BDCR_RTCEN_BB)))__HAL_RCC_RTC_ENABLE();

  //HAL_GPIO_TogglePin(LED);

  printf("value: %4.2f\n",3.5);

  beep(4);
  WriteRelayPort(TOP_LIGHT_PIN,0); HAL_Delay(50); WriteRelayPort(TOP_LIGHT_PIN,1);
  WriteRelayPort(LAMP_PIN,0); HAL_Delay(100); WriteRelayPort(LAMP_PIN,1);

  ReadEEprom();

  ssd1306_Init(MAIN_LCD);
  ssd1306_UpdateScreen(MAIN_LCD);

  ssd1306_Init(SECOND_LCD);
  ssd1306_UpdateScreen(SECOND_LCD);

  tm1638_Init(7);
  tm1638_Digit(0,1);
  tm1638_Clear(0);
  tm1638_Bright(5);

  for (uint8_t i = 0;i<8;i++) {
	  tm1638_Digit(i+1,i);
	  HAL_Delay(50); }
  for (uint8_t i = 7;i>0;i--) {
	  tm1638_Bright(i);
	  HAL_Delay(150); }

  //MFRC522_Init();

  start_title(MAIN_LCD);
  tm1638_Clear(0);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(myBinarySem);
  myBinarySemTM1638Handle = osSemaphoreCreate(osSemaphore(myBinarySem), 1);
  myBinarySemTM1638Handle2 = osSemaphoreCreate(osSemaphore(myBinarySem), 1);
  myBinarySemOledLcdHandle = osSemaphoreCreate(osSemaphore(myBinarySem), 1);
  myBinarySemOledLcd2Handle1 = osSemaphoreCreate(osSemaphore(myBinarySem), 1);

  vSemaphoreCreateBinary( xSemaphoreUSART3 );
  xSemaphoreTake( xSemaphoreUSART3, ( portTickType ) portMAX_DELAY );
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    /* Wait until a byte is received */
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 71;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SENSOR6_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOPLIGHT_SW_GPIO_Port, TOPLIGHT_SW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SX1278_RST_Pin|RELAY8_Pin|RELAY9_Pin|RELAY6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TM1638_STB_Pin|SENSOR7B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USB_EN_Pin|RELAY4_Pin|RELAY5_Pin|RELAY3_Pin 
                          |RELAY1_Pin|RELAY2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY7_GPIO_Port, RELAY7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ENC_BUTTON_UI_Pin SENSOR5_Pin IR_DOOR_SENSOR_Pin ENC_A_DOOR_Pin */
  GPIO_InitStruct.Pin = ENC_BUTTON_UI_Pin|SENSOR5_Pin|IR_DOOR_SENSOR_Pin|ENC_A_DOOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR6_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = SENSOR6_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_B_DOOR_Pin ENC_A_UI_Pin ENC_B_UI_Pin */
  GPIO_InitStruct.Pin = ENC_B_DOOR_Pin|ENC_A_UI_Pin|ENC_B_UI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR7_Pin IR_REMOTE_Pin */
  GPIO_InitStruct.Pin = SENSOR7_Pin|IR_REMOTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TOPLIGHT_SW_Pin */
  GPIO_InitStruct.Pin = TOPLIGHT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOPLIGHT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SX1278_RST_Pin RELAY8_Pin RELAY9_Pin RELAY6_Pin */
  GPIO_InitStruct.Pin = SX1278_RST_Pin|RELAY8_Pin|RELAY9_Pin|RELAY6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TM1638_STB_Pin */
  GPIO_InitStruct.Pin = TM1638_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TM1638_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR8_Pin */
  GPIO_InitStruct.Pin = SENSOR8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR7B2_Pin USB_EN_Pin RELAY4_Pin RELAY5_Pin 
                           RELAY3_Pin RELAY1_Pin RELAY2_Pin */
  GPIO_InitStruct.Pin = SENSOR7B2_Pin|USB_EN_Pin|RELAY4_Pin|RELAY5_Pin 
                          |RELAY3_Pin|RELAY1_Pin|RELAY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HX711_DAT_Pin */
  GPIO_InitStruct.Pin = HX711_DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HX711_DAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HX711_SCK_Pin */
  GPIO_InitStruct.Pin = HX711_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HX711_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KU5590_EXTI_Pin */
  GPIO_InitStruct.Pin = KU5590_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KU5590_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY7_Pin */
  GPIO_InitStruct.Pin = RELAY7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY7_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
		case 0: //BUTTON_IRQ_Pin:
			HAL_TIM_Base_Stop_IT(&htim1);
			__disable_irq ();
			ShowClapsTitle(MAIN_LCD);
			if (claps_counter == 0) {
				time_elapsed[0] = timer1_counter = 0;
				flags.flag_new_clap = true;
				claps_counter++;
				HAL_TIM_Base_Start_IT(&htim1);
			}
			else {
				if (claps_counter<MAX_CLAPS_COUNTER) {
					time_elapsed[claps_counter] = timer1_counter - time_elapsed[claps_counter-1];
					flags.flag_new_clap = true;
					claps_counter++;
				}
				else {
					ChangeCurrentGpioStatus(claps_counter);
					claps_counter = 0;
				}
			}
			__enable_irq ();
			HAL_TIM_Base_Start_IT(&htim1);
			break;
	}
}
#ifdef KU5590
void ResetKU5590()
{
	HAL_GPIO_WritePin(KU5590_RST_GPIO_Port,KU5590_RST_Pin,RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(KU5590_RST_GPIO_Port,KU5590_RST_Pin,SET);
	//HAL_Delay(50);
}
#endif
// Function: ExecuteCommand(uint8_t command)
// in: 'command' - command number for activate device on/off
// out: none
// exec: Set led_on on the tm1638 shield
//
void OutStringToLoggerLcd(char str[20] )
{
	if (myBinarySemOledLcdHandle != pdFALSE) {
		xSemaphoreTake(myBinarySemOledLcdHandle, portMAX_DELAY );
		ssd1306_SetCursor(0,0,MAIN_LCD);
		ssd1306_WriteString(LOGGER_STRING ,Font_7x10,White,MAIN_LCD);
		UpdateGridBox(str,MAIN_LCD);
		xSemaphoreGive(myBinarySemOledLcdHandle);
	}

}

void ChangeCurrentGpioStatus(uint8_t command) {
	char str[17];
	snprintf(str, sizeof(str),"CMD#%u activated\n\r",command);
	OutStringToLoggerLcd(str);
// check bit of the command, toggle it and write to port pcf857x

	if ((main_values.current_gpio_status >> (command-1)) & 0x01) {
		WriteRelayPort(relay_channels_map[command-1],OFF);
		main_values.current_gpio_status &= (~(1<<(command-1))); // clear bit
		if (myBinarySemTM1638Handle != pdFALSE) {
			xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
			tm1638_LedSet(command,0);
			xSemaphoreGive(myBinarySemTM1638Handle );
		}
	}
	else {
		WriteRelayPort(relay_channels_map[command-1],ON);
		main_values.current_gpio_status |= (1<<(command-1));
		if (myBinarySemTM1638Handle != pdFALSE) {
			xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
			tm1638_LedSet(command,1);
			xSemaphoreGive(myBinarySemTM1638Handle );
		}
	}

	if (myBinarySemTM1638Handle != pdFALSE) {
		xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
		DynamicBrightness(BRIGHT_DAY); // BRIGHT_DAY - 0-7 bright on day time, BRIGHT_NIGHT - night
		xSemaphoreGive(myBinarySemTM1638Handle );
	}
}

void esp8266SyncTask(void const * argument ) {

//	HAL_UART_Receive_DMA(&huart3, &UART3_rxBuffer, 1);
	UART_Send("SmartHome v2.0 by }{aTa6. Uart started\n");

	for (;;) {

		osDelay(1);
	}
}

void UART_Send (const char message[]) {
	while(UART_TX_Busy){};
	UART_TX_Busy = true;
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)message, strlen(message));
}

void KeyboardTask(void const * argument ) {

	uint8_t data = 0;
	uint8_t debonce_Threshold = 10;
	uint8_t debonce_Counter = 0;
	uint16_t cnt=0;

	for(;;)	{
		if (myBinarySemTM1638Handle != pdFALSE) {
			xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
			cnt++;
			if (cnt > TIME_MAX_KEYBOARD) {
				ShowTaskSign(1);
				cnt = 0;
			}

			data = tm1638_ReadKeys();
			switch (data) {
			case 0x00:
				debonce_Counter=0;
				key_pressed = 0;
				break;
			case 0x01:
				if (debonce_Counter < debonce_Threshold) debonce_Counter++; else key_pressed = 1;
				break;
			case 0x02:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key_pressed = 2;
				break;
			case 0x04:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key_pressed =  3;
				break;
			case 0x08:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++;else key_pressed = 4;
				break;
			case 0x10:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key_pressed = 5;
				break;
			case 0x20:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key_pressed =  6;
				break;
			case 0x40:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key_pressed =  7;
				break;
			case 0x80:
				if(debonce_Counter<debonce_Threshold)debonce_Counter++; else key_pressed = 8;
				break;
			default:
				key_pressed = 0;
				break;
			}
			xSemaphoreGive(myBinarySemTM1638Handle);
		}
		osDelay(1);
	}
}

void outputValueTosegmentsLed(uint8_t data, uint8_t pos_x)
{
	if (myBinarySemTM1638Handle != pdFALSE) {
		xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
		if (data <10) {
			tm1638_Digit(data ,pos_x);
			segmentN_Set2(pos_x,0);
		} else
			tm1638_Digit(data ,pos_x-1);
		xSemaphoreGive(myBinarySemTM1638Handle );
	}
}

void EncoderTask(void const * argument ) {

	uint16_t cnt = ZERO;
	init_encoder(&encoder[0]);
	init_encoder(&encoder[1]);
	init_encoder(&encoder[2]);
	encoder[0].value = encoder[1].value = 1;

	outputValueTosegmentsLed(encoder[0].value,4); // output to seven segment lcd on position 4 (count from 1...8)
	outputValueTosegmentsLed(encoder[1].value,7);

	void CheckEncoder(uint8_t num)
	{
		switch (num) {
		case 0:
			read_encoders(&encoder[0], 0);
			if (encoder[0].dirty) {
				encoder[0].dirty = false;
				reg_number = REG_CMD_ENC0_COUNTER;
				flags.flag_usb = SEND_REGISTER_DATA_TO_HOST;
				outputValueTosegmentsLed(encoder[0].value,4);
				if (beep_settings_flags.encoder0) beep(1);

				if (encoder[0].value < 1)  {
					flags.flag_timer4_counter = true;
					timer4_counter = sleep_mode_timeout;
					HAL_TIM_Base_Start(&htim4);
					HAL_TIM_Base_Start_IT(&htim4);
				}
				else {
					HAL_TIM_Base_Stop(&htim4);
					HAL_TIM_Base_Stop_IT(&htim4);
					flags.flag_timer4_counter = false;
					if (flags.flag_sleep_mode_activated) {
						flags.flag_sleep_mode_activated = false;
						RestoreFromSleep();
					}
				}
			} break;

		case 1:
			read_encoders(&encoder[1], 0);
			if (encoder[1].dirty) {
				encoder[1].dirty = false;
			} break;

		case 2:
			read_encoders(&encoder[2], 1);
			if (encoder[2].dirty) {
				encoder[2].dirty = false;
				if (beep_settings_flags.encoder1) beep(1);
				outputValueTosegmentsLed(encoder[2].value,7);
			} break;
		} // end switch
	}

	for(;;) {
		if (main_values.use_door_sensor == true) {
			if (HAL_GPIO_ReadPin(IR_DOOR_SENSOR_GPIO_Port,IR_DOOR_SENSOR_Pin) == false) {
				CheckEncoder(0);
			}
			else {
				CheckEncoder(1);
			}
		}
		else CheckEncoder(0);

		CheckEncoder(2);

		if (flags.flag_timer4_counter) {
			outputValueTosegmentsLed(timer4_counter,7);
			if (timer4_counter < 1) {
				HAL_TIM_Base_Stop(&htim4);
				HAL_TIM_Base_Stop_IT(&htim4);
				flags.flag_sleep_mode_activated = true;
				SleepMode(1);
			}
			flags.flag_timer4_counter = false;
		}

		cnt++;
		if (cnt > TIME_MAX_ENCODER) {
			cnt = ZERO;
			if (myBinarySemTM1638Handle2 != pdFALSE) {
				xSemaphoreTake(myBinarySemTM1638Handle2 , portMAX_DELAY );
				ShowTaskSign(2);
				xSemaphoreGive(myBinarySemTM1638Handle2);
			}
		}
		osDelay(1);
	}
}
//
//void IRremoteTask(void const * argument ) {
//	char trans_str[64] = {0,};
//
//	for(;;) {
//		if (ir_remote_decode(&results)) {
//			if (myBinarySemOledLcd2Handle1 != pdFALSE) {
//				xSemaphoreTake(myBinarySemOledLcd2Handle1, portMAX_DELAY );
//				ShowUartMonitor(MAIN_LCD);
//				UpdateGridBox(trans_str,MAIN_LCD);
//				cnt_oled_timer = ZERO;// CNT_OLED_TIMEOUT;
//				flags.flag_oled_timeout = false;
//				xSemaphoreGive(myBinarySemOledLcd2Handle1);
//			}
//			for (uint8_t count = 0; count<IR_REMOTE_COMMANDS_MAX;count++) {
//				if (IR_Remote[count].code == results.value) {
//					snprintf(trans_str, 64, "Cod: HEX %08X , (RemoteDevice Button: %s)", IR_Remote[count].code , IR_Remote[count].name);
//					CDC_Transmit_FS((uint8_t*)trans_str, strlen(trans_str));
//					my_enableIRIn();
//				}
//			}
//		}
//		else osDelay(1);
//	}
//}

//signed int GetRemoteKeyID(uint32_t value) {
//	for (uint8_t i=0; i <IR_REMOTE_COMMANDS_MAX;i++)
//		if (IR_Remote[i].code == value) return i;
//	return (-1);
//}

void SmartBoxTask() {

	ssd1306_WriteCommand(0x81,2);
	ssd1306_WriteCommand(0x80,2);
	for(;;) {
		ShowDateTime();
		osDelay(700);
	}
}

void ShowDateTime() {
	RTC_TimeTypeDef RTC_Time= {0};
	char timeBuff[20];
	HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
	sprintf(timeBuff,"%d:%02d:%02d", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);

	if (myBinarySemOledLcdHandle != pdFALSE) {
		xSemaphoreTake(myBinarySemOledLcdHandle, portMAX_DELAY );
		ssd1306_SetCursor(0,0,SECOND_LCD);
		ssd1306_WriteString(timeBuff,Font_11x18,White,SECOND_LCD);//Font_16x26,White);
		ssd1306_UpdateScreen(SECOND_LCD);

//		UpdateGridBox(timeBuff);
		xSemaphoreGive(myBinarySemOledLcdHandle); }
}


// Convert Date/Time structures to epoch time
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint8_t  a;
	uint16_t y;
	uint8_t  m;
	uint32_t JDN;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	// Calculate some coefficients
	a = (14 - date->Month) / 12;
	y = (date->Year + 2000) + 4800 - a; // years since 1 March, 4801 BC
	m = date->Month + (12 * a) - 3; // since 1 March, 4801 BC

	// Gregorian calendar date compute
    JDN  = date->Date;
    JDN += (153 * m + 2) / 5;
    JDN += 365 * y;
    JDN += y / 4;
    JDN += -y / 100;
    JDN += y / 400;
    JDN  = JDN - 32045;
    JDN  = JDN - JULIAN_DATE_BASE;    // Calculate from base date
    JDN *= 86400;                     // Days to seconds
    JDN += time->Hours * 3600;    // ... and today seconds
    JDN += time->Minutes * 60;
    JDN += time->Seconds;

	return JDN;
}

// Convert epoch time to Date/Time structures
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint32_t tm;
	uint32_t t1;
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t e;
	uint32_t m;
	int16_t  year  = 0;
	int16_t  month = 0;
	int16_t  dow   = 0;
	int16_t  mday  = 0;
	int16_t  hour  = 0;
	int16_t  min   = 0;
	int16_t  sec   = 0;
	uint64_t JD    = 0;
	uint64_t JDN   = 0;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	JD  = ((epoch + 43200) / (86400 >>1 )) + (2440587 << 1) + 1;
	JDN = JD >> 1;

    tm = epoch; t1 = tm / 60; sec  = tm - (t1 * 60);
    tm = t1;    t1 = tm / 60; min  = tm - (t1 * 60);
    tm = t1;    t1 = tm / 24; hour = tm - (t1 * 24);

    dow   = JDN % 7;
    a     = JDN + 32044;
    b     = ((4 * a) + 3) / 146097;
    c     = a - ((146097 * b) / 4);
    d     = ((4 * c) + 3) / 1461;
    e     = c - ((1461 * d) / 4);
    m     = ((5 * e) + 2) / 153;
    mday  = e - (((153 * m) + 2) / 5) + 1;
    month = m + 3 - (12 * (m / 10));
    year  = (100 * b) + d - 4800 + (m / 10);

    date->Year   = year - 2000;
    date->Month = month;
    date->Date= mday;
    date->WeekDay= dow;
    time->Hours = hour;
    time->Minutes = min;
    time->Seconds = sec;
}

// Adjust time with time zone offset
// input:
//   time - pointer to RTC_Time structure with time to adjust
//   date - pointer to RTC_Date structure with date to adjust
//   offset - hours offset to add or subtract from date/time (hours)
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset) {
	uint32_t epoch;

	epoch  = RTC_ToEpoch(time,date);
	epoch += offset * 3600;
	RTC_FromEpoch(epoch,time,date);
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart1)
//{
//	UART_TX_Busy = 0;
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart3)
//{
//	  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//	  int statusreg = USART2->SR;
//	  char uart_messages;
//
//	  uart_messages = 0;
//	  if(statusreg & (1<<4) )   // IDLE. ONLY this interrupt is enabled!!!
//	  {
//	     uart_messages = 3;
//	     USART2->DR;      // dummy read to clear idle flag
//	  }
//
//	  if(uart_messages)
//	  {
//	      xSemaphoreGiveFromISR( xSemaphoreUSART3, &xHigherPriorityTaskWoken );
//	  }
//
//	  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//}

//void CallBackDMA(void) {
//    portBASE_TYPE xHigherPriorityTaskWoken;
//
//    DMA1->IFCR = 7 << 20;    // clear DMA interrupt flags and continue
//
//    xSemaphoreGiveFromISR( xSemaphoreUSART3, &xHigherPriorityTaskWoken );
//    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//}

//void UART_CommandProcessor (void)
//{
//	// copy command to buffer
//	static char cmd_buf[MAXSTRING];
//	strcpy(cmd_buf, rxString);
//
//	CDC_Transmit_FS((uint8_t*)rxString, strlen(rxString));
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
	uint16_t cnt_sign_change=0;
	char string[64] = {0,};

	osThreadDef(KeyboardTask, KeyboardTask, osPriorityHigh, 0, 256);
	KeyboardTaskHandle = osThreadCreate(osThread(KeyboardTask), NULL);

	osThreadDef(EncoderTask,EncoderTask,osPriorityHigh,0,64);
	EncoderTaskHandle = osThreadCreate(osThread(EncoderTask),NULL);

	osThreadDef(SmartBoxTask,SmartBoxTask,osPriorityLow,0,128);
	SmartBoxHandle = osThreadCreate(osThread(SmartBoxTask),NULL);

	osThreadDef(esp8266SyncTask,esp8266SyncTask,osPriorityNormal,0,128);
	esp8266SyncHandle = osThreadCreate(osThread(esp8266SyncTask),NULL);

    vSemaphoreCreateBinary( xSemaphoreUSART3 );
    xSemaphoreTake( xSemaphoreUSART3, ( portTickType ) portMAX_DELAY );

//	osThreadDef(IRremoteTask,IRremoteTask,osPriorityNormal,0,64);
//	StartIRremoteTaskHandle = osThreadCreate(osThread(IRremoteTask),NULL);

//	ds1307_set_calendar_date(DS1307_WEDNESDAY, 9, 3, 16);
//	ds1307_set_time_24(10, 10, 30);

	ActivateGpioPorts(main_values.current_gpio_status);
	my_enableIRIn();
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	for(;;)	{

		if (ir_remote_decode(&results)) {
			snprintf(string, 64, "REMOTE:%p", (void*)results.value);
			CDC_Transmit_FS((uint8_t*)string, strlen(string));
			if (beep_settings_flags.remote) beep(0);

			if (results.value==TR_HOT_BUTTON1) { // use HOT BUTTON function
				cnt_oled_timer = ZERO;
				flags.flag_oled_timeout = false;
				if (flags.flag_user_sleep_mode) {
					ProcessHotButton(DEACTIVATE_HOT_BUTTON);
					flags.flag_user_sleep_mode = false; }
				else {
					ProcessHotButton(ACTIVATE_HOT_BUTTON);
					flags.flag_user_sleep_mode = true;
				}
			}

			if (results.value == TR_HOME) { // Change SLEEP mode state
				if (!flags.flag_sleep_mode_remote) {
					SleepMode(SLEEP_WITH_MASK); //SLEEP_WITH_MASK
					flags.flag_sleep_mode_remote = true;
				}
				else {
					RestoreFromSleep();
					flags.flag_sleep_mode_remote = false;
				}
			}
			remote_isr_restart_state(); // reset IR remote machine state
		}

		if (HAL_GPIO_ReadPin(IR_DOOR_SENSOR_GPIO_Port,IR_DOOR_SENSOR_Pin) != flags.last_ir_door_sensor) {
			flags.last_ir_door_sensor = HAL_GPIO_ReadPin(IR_DOOR_SENSOR_GPIO_Port,IR_DOOR_SENSOR_Pin);
			SendRegToHost(IR_DOOR_SENSOR_INDEX,flags.last_ir_door_sensor);
		}
// output sign for show task working...
		cnt_sign_change++;
		if (cnt_sign_change>TIME_MAX_DEFAULT) {
			if (myBinarySemTM1638Handle2 != pdFALSE) {
				xSemaphoreTake(myBinarySemTM1638Handle2 , portMAX_DELAY );
				cnt_sign_change = ZERO;
				ShowTaskSign(3);
				xSemaphoreGive(myBinarySemTM1638Handle2);
			}
		}

		if (key_pressed != old_key_pressed) {
			if (key_pressed>ZERO) {
				if (beep_settings_flags.keyboard) beep(1);
				ChangeCurrentGpioStatus(key_pressed);
				SendRegToHost(REG_CMD_GPIO_STATUS, main_values.current_gpio_status);
				claps_counter = ZERO;
				cnt_oled_timer = ZERO;// CNT_OLED_TIMEOUT;
				flags.flag_oled_timeout = false;
			}
			old_key_pressed = key_pressed;
		}

		cnt_oled_timer++;
		if (cnt_oled_timer>CNT_OLED_TIMEOUT) {
			if (flags.flag_oled_timeout != true) {
				flags.flag_oled_timeout = true;
				if (myBinarySemOledLcdHandle != pdFALSE) { // clear MAIN ssd1306 oled display
					xSemaphoreTake(myBinarySemOledLcdHandle, portMAX_DELAY );
					ssd1306_Fill(Black,MAIN_LCD);
					ssd1306_UpdateScreen(MAIN_LCD);
					xSemaphoreGive(myBinarySemOledLcdHandle); // end of...
				}
			} else cnt_oled_timer = CNT_OLED_TIMEOUT+1;
		}

		if (claps_counter> ZERO) { // check claps counter
			if (timer1_counter - time_elapsed[claps_counter-1] > MAX_ELAPSED_TIME) {
				ChangeCurrentGpioStatus(claps_counter);
				claps_counter = ZERO;
				cnt_oled_timer = ZERO;// clear cnt_oled_timer
				flags.flag_oled_timeout = false;
			}
			if (flags.flag_new_clap) {
				if (myBinarySemTM1638Handle != pdFALSE) { // output counter to segment leds display
					xSemaphoreTake(myBinarySemTM1638Handle , portMAX_DELAY );
					tm1638_Digit(claps_counter,CLAPS_COUNTER_POS);
					xSemaphoreGive(myBinarySemTM1638Handle ); // end of...
				}
				flags.flag_new_clap = false; // reset counter
#ifdef KU5590
				ResetKU5590();
#endif
			}
		}
		if (flags.flag_usb > ZERO) {
			UsbFlagProcess();
			flags.flag_usb = ZERO;
		}
		//			rc522Task();
		osDelay(1);
	}
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
