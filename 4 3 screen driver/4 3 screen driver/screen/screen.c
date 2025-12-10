#include <screen.h>

void WriteCmd(unsigned char I2C_Command) 
 {
	HAL_I2C_Mem_Write(&hi2c2, OLED_ADD, CMD, I2C_MEMADD_SIZE_8BIT, &I2C_Command, 1, 1000);
 }
		
void WriteDat(unsigned char I2C_Data)    
 {
		HAL_I2C_Mem_Write(&hi2c2,OLED_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&I2C_Data,1,100);
  }
 
void OLED_Init(void)
{
	HAL_Delay(100); 
	
	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //亮度调节 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
}
 
void OLED_SetPos(unsigned char x, unsigned char y) //设置起始点坐标
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);
}
 
void OLED_Fill(unsigned char fill_Data)//全屏填充
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}
 
 
void OLED_CLS(void)//清屏
{
	OLED_Fill(0x00);
}
 
void OLED_ON(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED唤醒
}
 
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
}

//显示单个字符
void OLED_ShowChar(uint8_t x,uint8_t y,char ch){
    uint8_t i = 0;
    uint8_t c = ch-32;

    if(c >= 95){
        return;
    }

    OLED_SetPos(x,y);
    for(;i<8;i++){
        WriteDat(Font8x16[c][i]);
    }
    OLED_SetPos(x,y+1);
    for(;i<16;i++){
        WriteDat(Font8x16[c][i]);
    }
}

//显示字符串
void OLED_ShowString(unsigned char x, unsigned char y,char ch[]){
    uint8_t j = 0;
    while(ch[j] != '\0'){
        if(x <= 127-8){
            OLED_ShowChar(x,y,ch[j]);
            x+=8;
            j++;
        }		
	}
}

