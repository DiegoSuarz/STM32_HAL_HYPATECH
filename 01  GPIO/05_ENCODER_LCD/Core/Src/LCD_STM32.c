/*
 * LCD_STM32.c
 *
 *  Created on: Mar 25, 2023
 *      Author: dg_su
 */

#include "LCD_STM32.h"


void LCD_STM32_Cmd(uint8_t a){
	E_OFF;
	E_ON;
	LCD_STM32_DataBus(a);
	HAL_Delay(2);
	E_OFF;
}
void LCD_STM32_DataBus(uint8_t a){
	// 0001
	if (a & 0x01){D4_ON;}
	else{D4_OFF;}
	// 0010
	if (a & 0x02){D5_ON;}
	else{D5_OFF;}
	// 0100
	if (a & 0x04){D6_ON;}
	else{D6_OFF;}
	// 1000
	if (a & 0x08){D7_ON;}
	else{D7_OFF;}
}
void LCD_STM32_Init(void){
	LCD_STM32_Cmd(0X03);
	LCD_STM32_Cmd(0X03);
	LCD_STM32_Cmd(0X03);
	LCD_STM32_Cmd(0X02);
	LCD_STM32_Cmd(0X02);
	LCD_STM32_Cmd(0X08);
	LCD_STM32_Cmd(0X00);
	LCD_STM32_Cmd(0X08);
	LCD_STM32_Cmd(0X00);
	LCD_STM32_Cmd(0X01);
	LCD_STM32_Cmd(0X00);
	LCD_STM32_Cmd(0X0D);
}
void LCD_STM32_Set_Cursor(uint8_t x,Ubicacion y){
	uint8_t m,n;

	RS_OFF;
	switch (y)
	{
		case Fila1:
				m=0x80+x;
				n=(0x80+x)>>4;
				LCD_STM32_Cmd(n);
				LCD_STM32_Cmd(m);
		break;
		case Fila2:
				m=0xC0+x;
				n=(0XC0+x)>>4;
				LCD_STM32_Cmd(n);
				LCD_STM32_Cmd(m);
		break;
		case Fila3:
				m=0x94+x;
				n=(0x94+x)>>4;
				LCD_STM32_Cmd(n);
				LCD_STM32_Cmd(m);
		break;
		case Fila4:
				m=0xD4+x;
				n=(0xD4+x)>>4;
				LCD_STM32_Cmd(n);
				LCD_STM32_Cmd(m);
		break;
	}

}
void LCD_STM32_Print_Char(char c){
	uint8_t m,n;
	m=c>>4;
	n=c;
	RS_ON;
	LCD_STM32_Cmd(m);
	LCD_STM32_Cmd(n);
}
void LCD_STM32_Print_String(char *c){
	char *ptr;
	ptr = c;
	while(*ptr)
		LCD_STM32_Print_Char(*ptr++);
}
void LCD_STM32_New_Char(uint8_t a,uint8_t b,uint8_t c,uint8_t d ,uint8_t e ,uint8_t f,uint8_t g,uint8_t h,uint8_t i){

	  uint8_t m,n;
		  switch(a){
			  case 1:
			  LCD_STM32_Cmd(0x04);
			  LCD_STM32_Cmd(0x00);
			  RS_ON;
			  m=b>>4;
			  n=b;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=c>>4;
			  n=c;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=d>>4;
			  n=d;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=e>>4;
			  n=e;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=f>>4;
			  n=f;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=g>>4;
			  n=g;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=h>>4;
			  n=h;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=i>>4;
			  n=i;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  RS_OFF;
			  LCD_STM32_Cmd(0x00);
			  LCD_STM32_Cmd(0x01);
			  HAL_Delay(100);
			  break;
		  case 2:
			  LCD_STM32_Cmd(0x04);
			  LCD_STM32_Cmd(0x08);
			  RS_ON;
			  m=b>>4;
			  n=b;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=c>>4;
			  n=c;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=c>>4;
			  n=c;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=d>>4;
			  n=d;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=e>>4;
			  n=e;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=f>>4;
			  n=f;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=g>>4;
			  n=g;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=h>>4;
			  n=h;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=i>>4;
			  n=i;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  RS_OFF;
			  LCD_STM32_Cmd(0x00);
			  LCD_STM32_Cmd(0x01);
			  HAL_Delay(100);
			  break;
		  case 3:
			  LCD_STM32_Cmd(0x04);
			  LCD_STM32_Cmd(0xFF);
			  RS_ON;
			  m=b>>4;
			  n=b;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=c>>4;
			  n=c;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=d>>4;
			  n=d;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=e>>4;
			  n=e;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=f>>4;
			  n=f;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=g>>4;
			  n=g;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=h>>4;
			  n=h;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  m=i>>4;
			  n=i;
			  LCD_STM32_Cmd(m);
			  LCD_STM32_Cmd(n);
			  RS_OFF;
			  LCD_STM32_Cmd(0x00);
			  LCD_STM32_Cmd(0x01);
			  HAL_Delay(100);
			  break;
		  }



}
void LCD_STM32_Print_New_Char(uint8_t a){
	RS_ON;
	LCD_STM32_Cmd(0x00);
	LCD_STM32_Cmd(a-1);
	RS_OFF;
}
void LCD_STM32_Clear(void){
	RS_OFF;
	LCD_STM32_Cmd(0x00);
	LCD_STM32_Cmd(0x01);
}
void LCD_STM32_Home(void){
	RS_OFF;
    LCD_STM32_Cmd(0x00);
    LCD_STM32_Cmd(0x02);
}
