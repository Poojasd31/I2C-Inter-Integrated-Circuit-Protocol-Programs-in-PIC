/*
 * File:   main.c
 * Author: Khaled Magdy
 * LAB:    34 - I2C LCD Interfacing
 */

#include <pic16f877a.h>
#include <xc.h>
#include "config.h"
#include "I2C.h"
#include "LCD.h"
#define _XTAL_FREQ 20000000

void main(void) {
    
    I2C_Master_Init();
    LCD_Init(0x4E);    // Initialize LCD module with I2C address = 0x4E
 
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("Welcome");
    LCD_Set_Cursor(2, 1);  
    LCD_Write_String("to Envision");
    __delay_ms(2500);

    while(1)
    {
      /* LCD_SR();
       __delay_ms(350); 
       LCD_SR();
       __delay_ms(350); 
       LCD_SL();
       __delay_ms(350);
       LCD_SL();
       __delay_ms(350);*/
    }
    return;
}
