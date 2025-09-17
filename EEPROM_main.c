/*
 * File:   EEPROM_main.c
 * Author: pooja
 *
 * Created on 4 June, 2025, 12:42 PM
 */


// PIC16F877A Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic16f877a.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define _XTAL_FREQ 20000000
#define I2C_BaudRate 100000
//#include"LCD_4bit_mode_Def.h"
#include"LCD_8_Bit.h"
#include "I2C.h"



//Pin Declarations FOR Switch and LED
#define SW1 RC2 //SW1 = RB0
#define SW2 RB5 //SW2 = RB1
#define LED1 RB1 //LED1 = RB2
#define LED2 RB2 //LED2 = RB3

//DIRECTIONS FOR Switch and LED
#define SW1_DIR TRISC2
#define SW2_DIR TRISB5
#define LED1_DIR TRISB1
#define LED2_DIR TRISB2



void main(void)
{
    unsigned char read_data;
     
    // I/O directions
    SW1_DIR = 1;
    SW2_DIR = 1;
    LED1_DIR = 0;
    LED2_DIR = 0;
    LED1 = 0; LED2 = 0;
    
    //Initialization of I2C
    I2C_Master_Init();
    
    // Initialize LCD
    LCD_Initialize();
    LCD_CMD(0x80);
    LCD_String("I2C:-");

    while (1)
    {
        

      if (SW1 == 0) 
      {
        __delay_ms(50);
        LED1 = 1;
        I2C_Master_Start();
        I2C_Master_Write(0xA0);         // EEPROM SLAVE Address for Write Address
        I2C_Master_Write(0x00);  //Word Address and Page No. EEPROM Memory Location
        I2C_Master_Write('M');   // Data
        I2C_Master_Stop();

        LCD_CMD(0x85);
        LCD_String("WRITE!");
        __delay_ms(500);
        LED1 = 0;
     }

    if (SW2 == 0) 
    {
        __delay_ms(50);
         LED2 = 0;
        I2C_Master_Start();
        I2C_Master_Write(0xA0);         // EEPROM Write Address
        I2C_Master_Write(0x00);  // EEPROM Memory Location
        I2C_Master_RepeatedStart();
        I2C_Master_Write(0xA1);         // EEPROM Read Address
        read_data = I2C_Read_Byte();
        I2C_NACK();
        I2C_Master_Stop();

        LCD_CMD(0xC0);
        LCD_String("READ: ");
        LCD_Data(read_data);
        __delay_ms(500);
        LED2 = 1;
    }
 }
}


//void lcd_init()
//{
//    TRISC0=0; //RS
//    TRISC1=0;//RW
//    TRISC2=0; //en
//    TRISD = 0x00;
//    lcd_cmd(0x02);
//    lcd_cmd(0x28);
//    lcd_cmd(0x0C);
//    lcd_cmd(0x04);
//    lcd_cmd(0x01);
//}
//
//void lcd_cmd(unsigned char cmd){
//    // 0x80 0xC0 0x94 0xD4
//    LCD = (cmd & 0xF0);
//    EN = 1;
//    RW = 0;
//    RS = 0;
//    __delay_ms(2);
//    EN = 0 ;
//    LCD = ((cmd<<4)& 0xF0);
//    EN = 1;
//    RW = 0;
//    RS = 0;
//    __delay_ms(2);
//    EN = 0;
//}
//
//void lcd_data(unsigned char data)
//{
//    LCD = (data & 0xF0);
//    EN = 1;
//    RW = 0;
//    RS = 1;
//    __delay_ms(2);
//    EN = 0 ;
//    LCD = ((data<<4)& 0xF0);
//    EN = 1;
//    RW = 0;
//    RS = 1;
//    __delay_ms(2);
//    EN = 0;
//}
//
//void lcd_caracter(const char *str)
//{
//    unsigned int i,num = strlen(str) ;
//    for(i=0;i<num;i++){
//        lcd_data(str[i]);
//    }
//}


//I2C FUNCTIONS

//void I2C_Initialize()
//{
//    SCL_DIR = 1;
//    SDA_DIR = 1;
//    SSPSTAT = 0X00;
//    SSPCON =  0X28;
//    SSPCON2 = 0X00; 
//    SSPADD = ((_XTAL_FREQ/4)/I2C_BaudRate) - 1;
//}

//I2C START FUNCTION

//void I2C_Start() 
//{
//    SDA = 1; 
//    SCL = 1; 
//    __delay_us(10);
//    __delay_us(10);
//    SDA = 0; 
//    SCL = 0;
//}

//I2C STOP FUNCTION
//void I2C_Stop() 
//{
//    SDA = 0; 
//    SCL = 1; 
//    __delay_us(10);
//    __delay_us(10);
//    SDA = 1; 
//    SCL = 0;
//}

//void I2C_Ack() 
//{
//    SCL = 1; 
//    __delay_us(10);
//    __delay_us(10);
//    SCL = 0;
//}

//void I2C_SendByte(unsigned char value) 
//{
//    unsigned char i;
//    unsigned char send;
//    send = value;
//    for (i = 0; i < 8; i++) 
//    {
//        SDA = send / 128;
//        send = send << 1;
//        SCL = 1; 
//        __delay_us(10);
//        SCL = 0;
//    }
//    
//    ack = SDA;
//    SDA = 0;
//}
//
//unsigned char I2C_ReadByte(void) 
//{
//    unsigned int i;
//    SDA = 1;
//    reead=0;
//    for (i = 0; i < 8; i++) 
//    {
//        reead <<= 1;
//        SCL = 1; 
//        __delay_us(10);
//        __delay_us(10);
//        if (SDA == 1) 
//            reead++;
//        SCL = 0;
//    }
//    SDA = 0;
//    return reead;
//}
//
//// ========================== EEPROM FUNCTIONS ==============================
//
///********************************************************************************
//* Function    : Eeprom_Save store data in EEPROM     *
//*********************************************************************************/
//void EEPROM_Write(unsigned char WR1, unsigned char WR2)
//{
//    I2C_Start();
//    I2C_SendByte(0xA0); // Write address
//    I2C_Ack();
//    I2C_SendByte(0x07); // Memory location
//    I2C_Ack();
//    I2C_SendByte(WR1);
//    I2C_Ack();
//    I2C_SendByte(WR2);
//    I2C_Ack();
//    I2C_Stop();
//
//    if(ack == 0)
//    {
//        LED1=0;
//        __delay_ms(100);
//        LED1=1;
//        __delay_ms(100);
//        cmd(0x88);
//        lcddata(WR1);
//        cmd(0x89);
//        lcddata(WR2);
//    }
//    else
//    LED2=0;
//    I2C_Ack();;
//}
//
///********************************************************************************
//* Function    : Eeprom_Read Read data from EEPROM
//*********************************************************************************/
//void EEPROM_Read(unsigned char RD1, unsigned char RD2) 
//{
//    I2C_Start();
//    I2C_SendByte(0xA0); // Write mode to set address
//    I2C_Ack();
//    I2C_SendByte(0x07);
//    I2C_Ack();
//
//    I2C_Start(); // Restart
//    I2C_SendByte(0xA1); // Read mode
//    I2C_Ack();
//
//    byte1 = I2C_ReadByte(); 
//    I2C_Ack();
//    byte2 = I2C_ReadByte();
//    I2C_Ack();
//    I2C_Stop();
//
//    if (byte1 == RD1) 
//    {
//        LED2 = 0; 
//        __delay_ms(100); 
//        LED2 = 1;
//        __delay_ms(100); 
//        
//        cmd(0xC6);
//        lcddata(byte1);
//    } 
//    else 
//    {
//        LED1 = 1;
//    }
//
//    if (byte2 == RD2) 
//    {
//       cmd(0xC7);
//       lcddata(byte2); 
//    }
//    I2C_Ack();
//}
//
//


//int ack;
//unsigned char reead,write,write2;
//unsigned char byte1,byte2;
////Forward Declartions for I2C
////void _nop_ (void);
//void I2C_Initialize();
//void I2C_Start();
//void I2C_Stop();
//void I2C_Ack();
//void I2C_SendByte(unsigned char);
//unsigned char I2C_ReadByte(void) ;
//
////Forward Declartions for EEPROM
//void EEPROM_Write(unsigned char, unsigned char);
//void EEPROM_Read(unsigned char, unsigned char);