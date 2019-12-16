#include"ssd1306.h"
#include "string.h"

// Screenbuffer
static uint8_t SSD1306_Buffer[2][SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Screen object
static SSD1306_t SSD1306[2];
static const uint8_t addr_i2c [2] = {SSD1306_I2C_ADDR1, SSD1306_I2C_ADDR2 };

//
//  Send a byte to the command register
//
void ssd1306_WriteCommand(uint8_t command,uint8_t lcd_id )
{
	HAL_I2C_Mem_Write(&hi2c1,addr_i2c[lcd_id],0x00,1,&command,1,SSD1306_TIMEOUT);
}


//
//	Initialize the oled screen
//
uint8_t ssd1306_Init(uint8_t lcd_id)
{	
	// Wait for the screen to boot
	HAL_Delay(100);
	
	/* Init LCD */
	ssd1306_WriteCommand(0xAE,lcd_id); //display off
	ssd1306_WriteCommand(0x20,lcd_id); //Set Memory Addressing Mode
	ssd1306_WriteCommand(0x10,lcd_id); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_WriteCommand(0xB0,lcd_id); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_WriteCommand(0xC8,lcd_id); //Set COM Output Scan Direction
	ssd1306_WriteCommand(0x00,lcd_id); //---set low column address
	ssd1306_WriteCommand(0x10,lcd_id); //---set high column address
	ssd1306_WriteCommand(0x40,lcd_id); //--set start line address
	ssd1306_WriteCommand(0x81,lcd_id); //--set contrast control register
	ssd1306_WriteCommand(0xFF,lcd_id);
	ssd1306_WriteCommand(0xA1,lcd_id); //--set segment re-map 0 to 127
	ssd1306_WriteCommand(0xA6,lcd_id); //--set normal display
	ssd1306_WriteCommand(0xA8,lcd_id); //--set multiplex ratio(1 to 64)
	ssd1306_WriteCommand(0x3F,lcd_id); //
	ssd1306_WriteCommand(0xA4,lcd_id); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_WriteCommand(0xD3,lcd_id); //-set display offset
	ssd1306_WriteCommand(0x00,lcd_id); //-not offset
	ssd1306_WriteCommand(0xD5,lcd_id); //--set display clock divide ratio/oscillator frequency
	ssd1306_WriteCommand(0xF0,lcd_id); //--set divide ratio
	ssd1306_WriteCommand(0xD9,lcd_id); //--set pre-charge period
	ssd1306_WriteCommand(0x22,lcd_id); //
	ssd1306_WriteCommand(0xDA,lcd_id); //--set com pins hardware configuration
	ssd1306_WriteCommand(0x12,lcd_id);
	ssd1306_WriteCommand(0xDB,lcd_id); //--set vcomh
	ssd1306_WriteCommand(0x20,lcd_id); //0x20,0.77xVcc
	ssd1306_WriteCommand(0x8D,lcd_id); //--set DC-DC enable
	ssd1306_WriteCommand(0x14,lcd_id); //
	ssd1306_WriteCommand(0xAF,lcd_id); //--turn on SSD1306 panel

	// Clear screen
	ssd1306_Fill(Black,lcd_id);

// Flush buffer to screen
	ssd1306_UpdateScreen(lcd_id);

// Set default values for screen object
	SSD1306[lcd_id].CurrentX = 0;
	SSD1306[lcd_id].CurrentY = 0;
	SSD1306[lcd_id].Initialized = 1;
	return 1;
}

//
//  Fill the whole screen with the given color
//
void ssd1306_Fill(SSD1306_COLOR color, uint8_t lcd_id)
{
	/* Set memory */
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buffer[lcd_id]); i++)
	{
		SSD1306_Buffer[lcd_id][i] = (color == Black) ? 0x00 : 0xFF;
	}
}

//
//  Write the screenbuffer with changed to the screen
//
void ssd1306_UpdateScreen(uint8_t lcd_id) {
	uint8_t i;
	
	for (i = 0; i < 8; i++) {
		ssd1306_WriteCommand(0xB0 + i, lcd_id);
		ssd1306_WriteCommand(0x00,lcd_id);
		ssd1306_WriteCommand(0x10,lcd_id);

		HAL_I2C_Mem_Write(&hi2c1,addr_i2c[lcd_id],0x40,1,&SSD1306_Buffer[lcd_id][SSD1306_WIDTH * i],SSD1306_WIDTH,100);
	}
}

//
//	Draw one pixel in the screenbuffer
//	X => X Coordinate
//	Y => Y Coordinate
//	color => Pixel color
//
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color, uint8_t lcd_id)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) 
	{
		// Don't write outside the buffer
		return;
	}
	
	// Check if pixel should be inverted
	if (SSD1306[lcd_id].Inverted)
	{
		color = (SSD1306_COLOR)!color;
	}
	
	// Draw in the right color
	if (color == White)
	{
		SSD1306_Buffer[lcd_id][x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} 
	else 
	{
		SSD1306_Buffer[lcd_id][x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

//
//  Draw 1 char to the screen buffer
//	ch 		=> char om weg te schrijven
//	Font 	=> Font waarmee we gaan schrijven
//	color 	=> Black or White
//
uint8_t ssd1306_WriteChar(uint8_t ch, FontDef Font, SSD1306_COLOR color, uint8_t lcd_id)
{
	uint32_t i, b, j;
	
	// Check remaining space on current line
	if (SSD1306_WIDTH <= (SSD1306[lcd_id].CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306[lcd_id].CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}
	
	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000) 
			{
				ssd1306_DrawPixel( SSD1306[lcd_id].CurrentX + j, (SSD1306[lcd_id].CurrentY + i), (SSD1306_COLOR) color, lcd_id);
			} 
			else 
			{
				ssd1306_DrawPixel(SSD1306[lcd_id].CurrentX + j, (SSD1306[lcd_id].CurrentY + i), (SSD1306_COLOR)!color, lcd_id);
			}
		}
	}
	
	// The current space is now taken
	SSD1306[lcd_id].CurrentX += Font.FontWidth;
	
	// Return written char for validation
	return ch;
}

//
//  Write full string to screenbuffer
//
uint8_t ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color,uint8_t lcd_id)
{
	// Write until null-byte
	while (*str) 
	{
		if (ssd1306_WriteChar(*str, Font, color,lcd_id) != *str)
		{
			// Char could not be written
			return *str;
		}
		
		// Next char
		str++;
	}
	
	// Everything ok
	return *str;
}

//
//	Position the cursor
//
void ssd1306_SetCursor(uint8_t x, uint8_t y,uint8_t lcd_id )
{
	SSD1306[lcd_id].CurrentX = x;
	SSD1306[lcd_id].CurrentY = y;
}
//
// API function by }{aTa6 (c) '19
//
//
void UpdateGridBox(char* str,uint8_t lcd_id)
{
	uint8_t i,y = 0;
	extern uint8_t grid_y;

	char str_temp[MAX_STRING_LEN];
	str[strlen(str) - 2]=0;

	if (grid_y>3) {
		for (i=0;i<3;i++) {
			for (y=0;y<MAX_STRING_LEN-1;y++) {
				stringGridBox[lcd_id][i*MAX_STRING_LEN+y] = stringGridBox[lcd_id][(i+1)*MAX_STRING_LEN+y];
				str_temp[y] = stringGridBox[lcd_id][i*MAX_STRING_LEN+y]; }
			ssd1306_SetCursor(0,i*11+16,lcd_id);
			ssd1306_WriteString(str_temp,Font_7x10,White,lcd_id);
		}
		y=0;
		while (str[y] && (y<MAX_STRING_LEN) ) {
			stringGridBox[lcd_id][i*MAX_STRING_LEN+y] = str[y];
			str_temp[y] = str[y];
			y++;
		}
		while (y<MAX_STRING_LEN-1) {
			stringGridBox[lcd_id][i*MAX_STRING_LEN+y] = 0x20;
			str_temp[y] = 0x20;
			y++;
		}
		str_temp[y]=0;
		ssd1306_SetCursor(0,i*11+16,lcd_id);
		ssd1306_WriteString(str_temp,Font_7x10,White,lcd_id);
		ssd1306_UpdateScreen(lcd_id);
	}
	else {
		y=0;
		while (str[y]) {
			stringGridBox[lcd_id][grid_y * MAX_STRING_LEN+y] = str[y];
			str_temp[y]=str[y];
			y++;
		}
		while (y<MAX_STRING_LEN-1) {
			stringGridBox[lcd_id][grid_y * MAX_STRING_LEN+y] = 0x20;
			str_temp[y] = 0x20;
			y++;
		}
		ssd1306_SetCursor(0,grid_y*11+16,lcd_id);
		ssd1306_WriteString(str_temp,Font_7x10,White,lcd_id);
		ssd1306_UpdateScreen(lcd_id);
		grid_y++;
	}
}

void start_title(uint8_t lcd_id)
{
	ssd1306_SetCursor(16,0,lcd_id);
	ssd1306_WriteString("SmartRoom",Font_11x18,White,lcd_id);
	ssd1306_UpdateScreen(lcd_id);
	HAL_Delay(50);
	ssd1306_SetCursor(35,30,lcd_id);
	ssd1306_WriteString("by XaTa6",Font_7x10,White,lcd_id);
	ssd1306_UpdateScreen(lcd_id);
	HAL_Delay(150);
	ssd1306_SetCursor(0,41,lcd_id);
	ssd1306_WriteString("Lutsk,UKRAINE,2018",Font_7x10,White,lcd_id);
	ssd1306_UpdateScreen(lcd_id);
	HAL_Delay(1500);
	ShowUartMonitor(lcd_id);
}

void ShowClapsTitle(uint8_t lcd_id)
{
	ssd1306_Fill(0,lcd_id);
	ssd1306_SetCursor(0,0,lcd_id);
	ssd1306_WriteString("CLAPS !!!",Font_16x26,White,lcd_id);
	ssd1306_UpdateScreen(lcd_id);
}

void ShowUartMonitor(uint8_t lcd_id)
{
	ssd1306_Fill(0,lcd_id);
	ssd1306_SetCursor(0,0,lcd_id);
	ssd1306_WriteString(LOGGER_STRING ,Font_7x10,White,lcd_id);
	ssd1306_UpdateScreen(lcd_id);
}
