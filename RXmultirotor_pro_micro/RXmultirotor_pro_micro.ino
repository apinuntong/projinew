#include <SPI.h>
#include "API.h"
#include "nRF24L01.h"
#include <Servo.h>
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo motor5;
Servo motor6;

//***************************************************
#define TX_ADR_WIDTH    5   
#define TX_PLOAD_WIDTH  32  

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10,0x01
}; 
int sovota1=0;
int sovota2=0;
unsigned char rx_buf[TX_PLOAD_WIDTH] = {0}; 
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};
//***************************************************
void setup() 
{
 
  motor1.attach(3);
  motor2.attach(4);
  motor3.attach(5);
  motor4.attach(6);
  motor5.attach(7);
  motor6.attach(8);
  motor1.write(0);
  motor2.write(90);
  motor3.write(90);
  motor4.write(90);
  motor5.write(90);
  motor6.write(90);
  pinMode(CE,  OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(IRQ, INPUT);
  SPI.begin();
  delay(50);
  init_io();                      
  unsigned char sstatus=SPI_Read(STATUS);
  delay(100);
  RX_Mode();                     
  delay(5000);
}

void loop() 
{
  for(;;)
  {
    unsigned char status = SPI_Read(STATUS);                        
    if(status&RX_DR)                                                 
    {
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);            
      SPI_RW_Reg(FLUSH_RX,0);       // clear RX_FIFO
//      Serial.print(((int16_t)rx_buf[1])<<8 | (int16_t)rx_buf[2]);
//      Serial.println(" ");
//      Serial.print(((int16_t)rx_buf[3])<<8 | (int16_t)rx_buf[4]);
//      Serial.println(" ");
//      Serial.print(((int16_t)rx_buf[5])<<8 | (int16_t)rx_buf[6]);
//      Serial.println(" ");
//      Serial.print(((int16_t)rx_buf[7])<<8 | (int16_t)rx_buf[8]);
//      Serial.println(" ");
      Serial.print(((((int16_t)rx_buf[9])<<8 | (int16_t)rx_buf[10])+15)*0.2945990180032733f);//ต้องทำการค่าริเบส
      Serial.println(" ");
      motor1.write(((((int16_t)rx_buf[9])<<8 | (int16_t)rx_buf[10])+15)*0.2945990180032733f);//ต้องทำการค่าริเบส
      
    }
    SPI_RW_Reg(WRITE_REG+STATUS,status);                            
    delay(10);
  }
}
void init_io(void)
{
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);			
  digitalWrite(CSN, 1);                
}
unsigned char SPI_RW(unsigned char Byte)
{
  return SPI.transfer(Byte);
}
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  digitalWrite(CSN, 0);                   
  SPI_RW(reg);                            
  SPI_RW(value);                        
  digitalWrite(CSN, 1);                  

  return(status);                  
}
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  digitalWrite(CSN, 0);                
  SPI_RW(reg);                         
  reg_val = SPI_RW(0);              
  digitalWrite(CSN, 1);               

  return(reg_val);                    
}
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char sstatus,i;

  digitalWrite(CSN, 0);                  
  sstatus = SPI_RW(reg);       	   

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);   
  }

  digitalWrite(CSN, 1);                   

  return(sstatus);                
}
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char sstatus,i;

  digitalWrite(CSN, 0);                
  sstatus = SPI_RW(reg);             
  for(i=0;i<bytes; i++)           
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                
  return(sstatus);                 
}
void RX_Mode(void)
{
  digitalWrite(CE, 0);
  
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + RF_CH, 30);        // Select RF channel 40
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0b);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. RX_DR enabled..
  digitalWrite(CE, 1);                             // Set CE pin high to enable RX device
  //  This device is now ready to receive one packet of 16 unsigned chars payload from a TX device sending to address
  //  '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
}
