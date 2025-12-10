#include <fonts.h>
#include <i2c.h>

#define OLED_ADD (0x3C << 1)
#define CMD 0x00
#define DAT 0x40

void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowChar(uint8_t x,uint8_t y,char ch);
void OLED_ShowString(unsigned char x, unsigned char y,char ch[]);
