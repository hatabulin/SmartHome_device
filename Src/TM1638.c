#include "math.h"
#include "TM1638.h"
#include "cmsis_os.h"

#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44
#define REG_READ 0x42 	
#define STARTSEGADDR  0xc0//
#define STARTLEDADDR  0xc1 //
#define INDEX_NEGATIVE_SIGN	16
#define INDEX_BLANK     17 // 17 nothing ,0  zero beter for clock

//#define CS_H (GPIOB->BSRR = GPIO_BSRR_BR1)
//#define CS_L (GPIOB->BRR = GPIO_BSRR_BR1)
#define CS_H (GPIOB->BSRR = ((uint16_t)0x0001))
#define CS_L (GPIOB->BRR = ((uint16_t)0x0001))

uint8_t BlankingFlag;
uint8_t SegArray[8];
uint8_t SegArray2[8];
uint8_t  reg_mas[4];
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] =
{
  0x3f,    // 0 0b00111111
  0x06,    // 1 0b00000110
  0x5b,    // 2 0b01011011
  0x4f,    // 3 0b01001111
  0x66,    // 4 0b01100110
  0x6d,    // 5 0b01101101
  0x7d,    // 6 0b01111101
  0x07,    // 7 0b00000111
  0x7f,    // 8 0b01111111
  0x6f,    // 9 0b01101111
  0x77,    // A 0b01110111
  0x7c,    // b 0b01111100
  0x39,    // C 0b00111001
  0x5e,    // d 0b01011110
  0x79,    // E 0b01111001
  0x71,    // F 0b01110001 /15
  0x40,    // - 0b01000000 /16
  0x00,     // nothing 0b00000000 /17
  0x80,     // dot /18
  0x01, // num 19
  0x02, // num 20
  0x04, // num 21
  0x08, // num 22
  0x10, // num 23
  0x20 // num 24
};

const uint8_t digitToSegmentDP[] =
{
  0xbf,    // 0 0b10111111
  0x86,    // 1 0b10000110
  0xdb,    // 2 0b11011011
  0xcf,    // 3 0b11001111
  0xe6,    // 4 0b11100110
  0xed,    // 5 0b11101101
  0xfd,    // 6 0b11111101
  0x87,    // 7 0b10000111
  0xff,    // 8 0b11111111
  0xef,    // 9 0b11101111
  0xf7,    // A 0b11110111
  0xfc,    // b 0b11111100
  0xb9,    // C 0b10111001
  0xde,    // d 0b11011110
  0xf9,    // E 0b11111001
  0xf1,    // F 0b11110001
  0x40,    // - 0b11000000
  0x80,     // nothing 0b00000000
  0x80     // dot
  };

void spi_Send(uint8_t data)
{
	extern SPI_HandleTypeDef hspi1;
	HAL_SPI_Transmit(&hspi1, &data, 1, 2000);
}
void spi_Read_Reg( uint8_t data_mas[])
{//+
	extern SPI_HandleTypeDef hspi1;
	uint8_t key_Reg_Adr =0x42;
	  SPI1->CR1 |= SPI_CR1_BIDIOE;
		HAL_SPI_Transmit(&hspi1,&key_Reg_Adr , 1, 1000);
 	  SPI1->CR1 &= ~SPI_CR1_BIDIOE;
	  HAL_SPI_Receive(&hspi1, data_mas, 4, 1000);
	  SPI1->CR1 |= SPI_CR1_BIDIOE;
}
//=========================================================================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//=========================================================================//
void tm1638_Init(uint8_t brightness)
{
	if(brightness>7)return;
	CS_L;
	HAL_Delay(1);
	spi_Send(0x88|brightness);//0x88
	HAL_Delay(1);
	CS_H;
	tm1638_Clear(0);
}

void tm1638_Bright(uint8_t brightness)
{
	if(brightness>7)return;
	CS_L;
	HAL_Delay(1);
	spi_Send(0x88|brightness);//0x88
	HAL_Delay(1);
	CS_H;
}
//++++++++++++++++++++++управление одним светодиодом+++++++++++++++++++++++//
//=========================================================================//
void tm1638_LedSet(uint8_t led_Num, uint8_t state){//led_Num 1-8
	if(led_Num>8 || led_Num<1)return;
	CS_L;
	spi_Send(ADDR_FIXED);  
	CS_H;

	CS_L;
	spi_Send(STARTLEDADDR+(led_Num-1)*2);
	spi_Send(state);
	CS_H;
}
//=========================================================================//
//+++++++++++++++++++++++управление всеми светодиодами+++++++++++++++++++++//
//=========================================================================//
void leds_Set(uint8_t state){//+
	uint8_t led_Num = 0;
	CS_L;
	spi_Send(ADDR_FIXED);  
	CS_H;
 for(;led_Num<8;led_Num++){
	CS_L;
	spi_Send(STARTLEDADDR+led_Num*2);
	spi_Send(state&0x01);
	state >>= 1; 
	CS_H;
 }
}
//=========================================================================//
//++++++++++++++++++++++++отображает i-ый сегмент++++++++++++++++++++++++++//
//=========================================================================//
void segmentN_Set(uint8_t seg_Num, uint8_t data){//seg_Num 1-8;
	if(seg_Num>8 || seg_Num<1)return;
	seg_Num=seg_Num-1;
	data=digitToSegment[data];
	CS_L;
	spi_Send(ADDR_FIXED); 
	CS_H;

	CS_L;
	spi_Send(STARTSEGADDR+seg_Num*2);//0,2,4,6,8,10....
	spi_Send(data);
	CS_H;
}
void segmentN_Set2(uint8_t seg_Num, uint8_t data){//seg_Num 1-8;
	if(seg_Num>8 || seg_Num<1)return;
	seg_Num=seg_Num-1;
	CS_L;
	spi_Send(ADDR_FIXED);
	CS_H;

	CS_L;
	spi_Send(STARTSEGADDR+seg_Num*2);//0,2,4,6,8,10....
	spi_Send(data);
	CS_H;
}

//=========================================================================//
//+++++++++++++++++++++++++отображает массив+++++++++++++++++++++++++++++++//
//=========================================================================//
void segments_Set(uint8_t start_Seg, uint8_t stop_Seg, uint8_t data[]){//+
	uint8_t i=0;
	CS_L;
	spi_Send(ADDR_FIXED); 
	CS_H;

		for(;start_Seg < stop_Seg;start_Seg++,i++){
			CS_L;
			spi_Send(STARTSEGADDR+start_Seg*2);
			spi_Send(data[i]);
			CS_H;
		}
}
//=========================================================================//
//+++++++++++++++++++++читаем состояние кнопок+++++++++++++++++++++++++++++//
//=========================================================================//
uint8_t tm1638_ReadKeys(){//+
	uint8_t keys = 0; //reg_mas[4];
  uint8_t i=0;
	
	CS_L;
	spi_Read_Reg(reg_mas);//
	CS_H;
  for(;i<4;i++) keys |= (reg_mas[i]&0x11)<<i;
	return keys;
}

void tm1638_Clear(uint8_t parameter){//0-all;1-segments;2-led
	uint8_t i=0;

	CS_L;
	if(!parameter)spi_Send(ADDR_AUTO);
	   else spi_Send(ADDR_FIXED);
	CS_H;

		switch(parameter){
			case 0://all
				CS_L;
				spi_Send(STARTSEGADDR);
	     	for(; i < 16; i ++) spi_Send(0);
			  CS_H;
			break;
		  case 1://segments
				for(; i < 8; i ++){
		    CS_L;
		    spi_Send(STARTSEGADDR+i*2);
		    spi_Send(0);
		    CS_H;
	     }
			break;
			case 2://led
				for(;i<8;i++){
	         CS_L;
	         spi_Send(STARTLEDADDR+i*2);
	         spi_Send(0);
	         CS_H;
     }
			break;
   }

}

//=========================================================================//
//|
//=========================================================================//
void tm1638_Digit(int digit,uint8_t pos)//
{
//	uint8_t offset=0;
	uint8_t i=0;
	uint8_t digit_starts=0;
//	if (digit>9) offset = 1;
	if((digit > 99999999)||(digit < -9999999))return;
	if(digit < 0)
	{
		SegArray[0] = INDEX_NEGATIVE_SIGN;
		digit = (digit & 0x7fffffff);
		SegArray[1] = digit/1000000;
		digit %= 1000000;
		SegArray[2] = digit/1000000;
		digit %= 1000000;
		SegArray[3] = digit/1000000;
		digit %= 1000000;
		SegArray[4] = digit/1000000;
		digit %= 1000000;
		SegArray[5] = digit/1000000;
		digit %= 1000000;
		SegArray[6] = digit / 10;
		SegArray[6] = digit % 10;
		if(BlankingFlag)
		{
			if(SegArray[1] == 0)
			{
				SegArray[1] = INDEX_BLANK;
				if(SegArray[2] == 0){
					SegArray[2] = INDEX_BLANK;
				  if(SegArray[3] == 0){
					   SegArray[3] = INDEX_BLANK;
						 if(SegArray[4] == 0){
					   SegArray[4] = INDEX_BLANK;
							 if(SegArray[5] == 0){
					        SegArray[5] = INDEX_BLANK;
								  if(SegArray[6] == 0){
					           SegArray[7] = INDEX_BLANK;
									  }
							 }
						 }
					}
				}
			}
		}
	}
	else
	{
//		if (digit>9) offset = 1;
		SegArray[0] = digit/10000000;
		digit %= 10000000;
		SegArray[1] = digit/1000000;
		digit %= 1000000;
		SegArray[2] = digit / 100000;
		digit %= 100000;
		SegArray[3] = digit / 10000;
		digit %= 10000;
		SegArray[4] = digit / 1000;
		digit %= 1000;
		SegArray[5] = digit / 100;
		digit %= 100;
		SegArray[6] = digit / 10;
		SegArray[7] = digit % 10;
		if(BlankingFlag)
		{
			if(SegArray[0] == 0)
			{
				SegArray[0] = INDEX_BLANK;
				if(SegArray[1] == 0)
				{
					SegArray[1] = INDEX_BLANK;
					if(SegArray[2] == 0)
					{
						SegArray[2] = INDEX_BLANK;
						if(SegArray[3] == 0)
						{
							SegArray[3] = INDEX_BLANK;
							if(SegArray[4] == 0)
							{
								SegArray[4] = INDEX_BLANK;
								if(SegArray[5] == 0)
								{
									SegArray[5] = INDEX_BLANK;
									if(SegArray[6] == 0)
									{
										SegArray[6] = INDEX_BLANK;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	BlankingFlag = 1;

	while(SegArray[i]==INDEX_BLANK)i++;
	digit_starts=(i);
	for(;i<MAX_SEGMENTS;i++)
	{
		SegArray[i-digit_starts]=digitToSegment[SegArray[i]];
		SegArray[i]=0;
	}

	segments_Set(pos  ,(MAX_SEGMENTS -digit_starts) + pos ,SegArray);//0-8
	//segments_Set(pos-1,(8-digit_starts)+pos-1,SegArray);//0-8
}
//=========================================================================//
//++++++++++++++++++++++++чистит дисплей+++++++++++++++++++++++++++++++++++//
//=========================================================================//


void tm1638_FloatDigit(float digit,uint8_t num_signes, uint8_t pos,uint8_t aligment) //
{
	int a = 10; // ��� ������ ���� �� ����
	uint8_t digit_starts=0;
	uint8_t i;

	if (aligment == ALIGMENT)
	{
	  	uint8_t pos = digit;
	  	uint8_t i=0;
	  	if (pos>0)
	  	{
	  		while (pos > 0)
	  	  	{
	  			i++;
	  	  		pos = pos / 10;
	  	  	}
	  	  	i--;
	  	}
	  	pos = MAX_SEGMENTS-i-num_signes;
	}

	for(i = 1; i < num_signes; i++) // ������� ���
	{
		a= a*10;
	}
	a = digit * a;

	if(a < 0)
	{
		SegArray[0] = INDEX_NEGATIVE_SIGN;
		a = (a & 0x7fffffff);
		SegArray[1] = a/10000;
		a %= 10000;
		SegArray[2] = a/1000;
		a %= 1000;
		SegArray[3] = a/100;
		a %= 100;
		SegArray[4] = a / 10;
		SegArray[4] = a % 10;

		if(BlankingFlag)
		{
			if (SegArray[0] == 0)
			{
				SegArray[0] = INDEX_BLANK;
				if ( (SegArray[1] == 0) & (num_signes != 3) )
				{
					SegArray[1] = INDEX_BLANK;
					if ( (SegArray[2] == 0) & (num_signes != 2) )
					{
						SegArray[2] = INDEX_BLANK;
						if ( (SegArray[3] == 0) & (num_signes != 1))
						{
							SegArray[3] = INDEX_BLANK;
	//						if(SegArray[4] == 0)
							{
	//							SegArray[4] = INDEX_BLANK;
							}
						}
					}
				}
			}
		}
	}
	else
	{
		SegArray[0] = a/100000;
		a %= 100000;
		SegArray[1] = a/10000;
		a %= 10000;
		SegArray[2] = a/1000;
		a %= 1000;
		SegArray[3] = a/100;
		a %= 100;
		SegArray[4] = a / 10;
		SegArray[5] = a % 10;

	if(BlankingFlag)
	{
		if (SegArray[0] == 0)
		{
			SegArray[0] = INDEX_BLANK;
			if (SegArray[1] == 0)
			{
				SegArray[1] = INDEX_BLANK;
				if ( (SegArray[2] == 0) & (num_signes != 3) )
				{
					SegArray[2] = INDEX_BLANK;
					if ( (SegArray[3] == 0) & (num_signes != 2))
					{
						SegArray[3] = INDEX_BLANK;
						if ( (SegArray[4] == 0) & (num_signes != 1))
						{
							SegArray[4] = INDEX_BLANK;
						}
					}
				}
			}
		}
	}
	}

	BlankingFlag = 1;

	i = 0;
	while (SegArray[i]==INDEX_BLANK) i++;
	digit_starts = i;
	for(; i<MAX_SEGMENTS; i++)
	{
		if ( ((MAX_SEGMENTS-1)-i) == num_signes )
		{
			SegArray[i-digit_starts] = digitToSegmentDP[SegArray[i]];
		}
		else
		{
			SegArray[i-digit_starts]=digitToSegment[SegArray[i]];
		}
		SegArray[i]=0;
	}
#ifdef VER1
		for(i=0;i<3;i++)
		{
			uint8_t a = SegArray[i+3];
			SegArray[i+3] = SegArray[i];
			SegArray[i] = a;
		}
#endif
	segments_Set(pos-1,MAX_SEGMENTS-1-digit_starts+pos,SegArray);//0-8
}

//
// tm1638 api function
//
void DynamicBrightness(uint8_t bright)
{
	uint8_t j=0,i=0;
	extern osThreadId KeyboardTaskHandle;

	while (j<2)
	{
		j++;
		while (i<7)
		{
			HAL_Delay(j);
//			vTaskSuspend(KeyboardTaskHandle);
			tm1638_Bright(i);
//			vTaskResume(KeyboardTaskHandle);
			i++;
		}
		i=0;
	}
	tm1638_Bright(bright);
}

