
#include <xc.h>
#include "I2C.h"
#include "LCD.h"



//---------------[ I2C Routines ]-------------------
//--------------------------------------------------
void I2C_Master_Init() //For Initialization of I2C
{
  SSPCON  = 0x28; //SSPEN = 1,  I2C Master mode, clock = FOSC/(4 * (SSPADD + 1)) =1000
  SSPCON2 = 0x00; //0000 0000
  SSPSTAT = 0x00; //0000 0000
  SSPADD = ((_XTAL_FREQ/4)/I2C_BaudRate) - 1; //SSPADD = (FOSC / 4 * Clock ) - 1
  SCL_D = 1; //Input
  SDA_D = 1; //Input
}

void I2C_Master_Wait()
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
    I2C_Master_Wait();
    SEN = 1; // Initiate Start condition on SDA and SCL pins. Automatically cleared by hardware.
}

void I2C_Master_RepeatedStart()
{
    I2C_Master_Wait();
    RSEN = 1;//1 = Initiate Repeated Start condition on SDA and SCL pins. Automatically cleared by hardware
}

void I2C_Master_Stop()
{
    I2C_Master_Wait();
    PEN = 1; //1 = Initiate Stop condition on SDA and SCL pins. Automatically cleared by hardware. 
}

void I2C_ACK(void)
{
	ACKDT = 0;			// 0 -> ACK //0 = Acknowledge
	I2C_Master_Wait();
    ACKEN = 1;// Send ACK //1 = Initiate Acknowledge sequence on SDA and SCL pins and transmit ACKDT data bit.Automatically cleared by hardware.
}

void I2C_NACK(void)
{
	ACKDT = 1;			// 1 -> NACK ,1 = Not Acknowledge 
    I2C_Master_Wait();
	ACKEN = 1;			// Send NACK, Initiate Acknowledge sequence on SDA and SCL pins and transmit ACKDT data bit.Automatically cleared by hardware. 
}

unsigned char I2C_Master_Write(unsigned char data)
{
    I2C_Master_Wait(); //call wait function
    SSPBUF = data;  // SSPBUF is the buffer register to which data bytes are written to or read from.
    while(!SSPIF);  // Wait Until Completion, Monitor the flag
	SSPIF = 0; //Clear the flag
    return ACKSTAT; // ACKSTAT: Acknowledge Status bit 
}

unsigned char I2C_Read_Byte(void)
{
    //---[ Receive & Return A Byte ]---
	I2C_Master_Wait();
    RCEN = 1;		  // Enable & Start Reception , 1 = Enables Receive mode for I2C 
	while(!SSPIF);	  // Wait Until Completion
	SSPIF = 0;		  // Clear The Interrupt Flag Bit
    I2C_Master_Wait();
    return SSPBUF;	  // Return The Received Byte
}
