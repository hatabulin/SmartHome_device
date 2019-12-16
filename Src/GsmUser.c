#include "Sim80x.h"
UART_HandleTypeDef huart1;

extern uint8_t cnt;

//##################################################################################################################
void  Gsm_User(uint32_t StartupTime)
{
    cnt++;
    if (cnt>50)
    {
        char answer[35];
    	snprintf(answer, sizeof(answer),"GsmUser HeartBeat...\n\r");
    	HAL_UART_Transmit(&huart1, (uint8_t*)answer,strlen(answer), HAL_MAX_DELAY);
    	cnt = 0;
    }
}
//##################################################################################################################
void  Gsm_UserNewCall(const char *CallerNumber)
{
	char answer[25];
	char buffer[15];

	snprintf(answer, sizeof(answer),"Call:%s\n\r",Sim80x.Gsm.CallerNumber);
	HAL_UART_Transmit(&huart1, (uint8_t*)answer,strlen(answer), HAL_MAX_DELAY);
	UpdateGridBox(answer);

	Gsm_CallDisconnect();
	osDelay(10000);
	strcpy(buffer, &CallerNumber[3]);

	snprintf(answer, sizeof(answer),"Dial:%s\n\r",Sim80x.Gsm.CallerNumber);
	HAL_UART_Transmit(&huart1, (uint8_t*)answer,strlen(answer), HAL_MAX_DELAY);
	UpdateGridBox(answer);
	Gsm_Dial(buffer,30);
	//Gsm_CallAnswer();

}
//##################################################################################################################
void  Gsm_UserNewMsg(char *Number,char *Date,char *Time,char *msg)
{
  
  
  
}
//##################################################################################################################
