/*********************************************************************
**  Device:  nRF24L01+                                              **
**  File:   EF_nRF24L01_TX.c                                        **
**                                                                  **
**                                                                  **
**  Copyright (C) 2011 ElecFraks.                                   **
**  This example code is in the public domain.                      **
**                                                                  **
**  Description:                                                    **
**  This file is a sample code for your reference.                  **
**  It's the v1.0 nRF24L01+ Hardware SPI by arduino                 **
**  Created by ElecFreaks. Robi.W,11 June 2011                      **
**                                                                  **
**  http://www.elecfreaks.com                                       **
**                                                                  **
**   SPI-compatible                                                 **
**   CS - to digital pin 8                                          **
**   CSN - to digital pin 9  (SS pin)                               **
**   MOSI - to digital pin 11 (MOSI pin)                            **
**   MISO - to digital pin 12 (MISO pin)                            **
**   CLK - to digital pin 13 (SCK pin)                              **
*********************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include "API.h"
#include "nRF24L01.h"
#include <LCD5110_Basic.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[32]; // FIFO storage buffer

LCD5110 myGLCD(7,6,5,4,3);
//      SCK  - Pin 7
//      MOSI - Pin 6
//      DC   - Pin 5
//      RST  - Pin 4
//      CS   - Pin 3
//      LED  - Pin 2
extern uint8_t arduino_logo[];
extern uint8_t SmallFont[];
//***************************************************
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 32 unsigned chars TX payload

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10,0x01
}; // Define a static TX address

unsigned char rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};
//***************************************************
Quaternion q;           // [w, x, y, z]         quaternion caontainer
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int yaw,pitch,roll;
int set_1=0,set_2=0,set_3=0,set_4=0;
int rew_1=1,rew_2=1,rew_3=1;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
/////////////////////////////////////////////////
int LED=8;
byte iInput=0;
void setup() 
{
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  Serial.begin(9600);
  pinMode(CE,  OUTPUT);
  pinMode(CSN, OUTPUT);
  //pinMode(IRQ, INPUT);
  SPI.begin();
  delay(50);
  init_io();    // Initialize IO port
  Wire.begin();
  Wire.requestFrom(0x38,1);
  pinMode(LED, OUTPUT);
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  myGLCD.drawBitmap(0, 0, arduino_logo, 84, 48);
  delay(4000);
  unsigned char sstatus=SPI_Read(STATUS);
  Serial.println("*******************TX_Mode Start****************************");
  Serial.print("status = ");    
  Serial.println(sstatus,HEX);     // There is read the mode’s status register, the default value should be ‘E’
  TX_Mode();                       // set TX mode
   Timer1.initialize(6000);         
  Timer1.attachInterrupt( TX );  
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(920);
    mpu.setYGyroOffset(-610);
    mpu.setZGyroOffset(-105);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip
    if(EEPROM.read(0)==1){
          rew_1=1023+16;
        }else{
          rew_1=0;
          EEPROM.write(0, 0);
        }
    if(EEPROM.read(1)==1){
          rew_2=1023+103;
        }else{
          rew_2=0;
          EEPROM.write(1, 0);
        }
    if(EEPROM.read(2)==1){
          rew_3=1120;
        }else{
          rew_3=0;
          EEPROM.write(2, 0);
        }    
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() 
{
  while(1)
  {
SET_UP2:
     MPU6050_tx();
    Wire.requestFrom(0x38,1);
    if(Wire.available())   
      {
        iInput = Wire.read();
    if(iInput==253){set_2=set_2+1;EEPROM.write(5, (uint8_t)(set_2>>8));EEPROM.write(6,(uint8_t)set_2);delay(100);}//pin1
    if(iInput==254){set_2=set_2-1;EEPROM.write(5, (uint8_t)(set_2>>8));EEPROM.write(6,(uint8_t)set_2);delay(100);}//pin2
    if(set_2>202){set_2=200;EEPROM.write(5, (uint8_t)(set_2>>8));EEPROM.write(6,(uint8_t)set_2);}
    if(iInput==251){set_4=set_4-1;EEPROM.write(9, (uint8_t)(set_4>>8));EEPROM.write(10,(uint8_t)set_4);delay(100);}//pin3
    if(iInput==247){set_4=set_4+1;EEPROM.write(9, (uint8_t)(set_4>>8));EEPROM.write(10,(uint8_t)set_4);delay(100);}//pin4
    if(iInput==127){set_3=set_3+1;EEPROM.write(7, (uint8_t)(set_3>>8));EEPROM.write(8,(uint8_t)set_3);delay(100);}//pin5
    if(iInput==191){set_3=set_3-1;EEPROM.write(7, (uint8_t)(set_3>>8));EEPROM.write(8,(uint8_t)set_3);delay(100);}//pin6
    if(iInput==223){set_1=set_1+1;EEPROM.write(3, (uint8_t)(set_1>>8));EEPROM.write(4,(uint8_t)set_1);delay(100);}//pin7
    if(iInput==239){set_1=set_1-1;EEPROM.write(3, (uint8_t)(set_1>>8));EEPROM.write(4,(uint8_t)set_1);delay(100);}//pin8
    if(iInput==255){}//ไม่ได้กด//แก้บรรทล่างด้วย
    if(iInput==252){
      delay(200);
      myGLCD.clrScr();
      myGLCD.print("set_up", CENTER, 0);
      if(EEPROM.read(2)==0){
      myGLCD.print("                 ", CENTER, 8);
      myGLCD.print("yaw reward", CENTER, 8);
      }
      if(EEPROM.read(2)==1){
      myGLCD.print("                  ", CENTER, 8);
      myGLCD.print("yaw forward", CENTER, 8);
      }
      if(EEPROM.read(0)==0){
      myGLCD.print("                  ", CENTER, 16);
      myGLCD.print("pitch reward", CENTER, 16);
      }
      if(EEPROM.read(0)==1){
      myGLCD.print("                  ", CENTER, 16);
      myGLCD.print("pitch forward", CENTER, 16);
      }
      if(EEPROM.read(1)==0){
      myGLCD.print("                    ", CENTER, 24);
      myGLCD.print("roll reward", CENTER, 24);
      }
      if(EEPROM.read(1)==1){
      myGLCD.print("                  ", CENTER, 24);
      myGLCD.print("roll forward", CENTER, 24);
      }
      goto SET_UP1;
      }
    //Serial.println(iInput);
      } 
      set_1=((int16_t)EEPROM.read(3))<<8 | ((int16_t)EEPROM.read(4));
      set_2=((int16_t)EEPROM.read(5))<<8 | ((int16_t)EEPROM.read(6));
      set_3=((int16_t)EEPROM.read(7))<<8 | ((int16_t)EEPROM.read(8));
      set_4=((int16_t)EEPROM.read(9))<<8 | ((int16_t)EEPROM.read(10));
    int sensorValue1 = abs(analogRead(A0)-rew_3)-(559+set_1);
    int sensorValue2 = analogRead(A1)-(204+set_2);
    int sensorValue3 = abs(analogRead(A2)-rew_1)-(519+set_3); 
    int sensorValue4 = abs(analogRead(A3)-rew_2)-(563+set_4);
    tx_buf[7] = (uint8_t)(sensorValue1>>8);    
    tx_buf[8] = (uint8_t)sensorValue1;
    tx_buf[9] = (uint8_t)(sensorValue2>>8);    
    tx_buf[10] = (uint8_t)sensorValue2;  
    tx_buf[11] = (uint8_t)(sensorValue3>>8);    
    tx_buf[12] = (uint8_t)sensorValue3;
    tx_buf[13] = (uint8_t)(sensorValue4>>8);    
    tx_buf[14] = (uint8_t)sensorValue4;     
                         // clear RX_DR or TX_DS or MAX_RT interrupt flag
    
  }
  while(1){
SET_UP1:
    Wire.requestFrom(0x38,1);
    if(Wire.available())   
      {
        iInput = Wire.read();
    if(iInput==63){
        delay(200);
        
        if(EEPROM.read(0)==0){
          myGLCD.print("                         ", CENTER, 16);
          myGLCD.print("pitch forward", CENTER, 16);
          rew_1=1039;
          EEPROM.write(0, 1);
        }else{
          myGLCD.print("                         ", CENTER, 16);
          myGLCD.print("pitch reward", CENTER, 16);
          rew_1=0;
          EEPROM.write(0, 0);
        }
        
    }
    if(iInput==243){
      delay(200);
        
        if(EEPROM.read(1)==0){
          myGLCD.print("                         ", CENTER, 24);
          myGLCD.print("roll forward", CENTER, 24);
          rew_2=1023+103;
          EEPROM.write(1, 1);
        }else{
          myGLCD.print("                         ", CENTER, 24);
          myGLCD.print("roll reward", CENTER, 24);
          rew_2=0;
          EEPROM.write(1, 0);
        }
    }//pin2
    if(iInput==207){
      delay(200);
        
        if(EEPROM.read(2)==0){
          myGLCD.print("                         ", CENTER, 8);
          myGLCD.print("yaw forward", CENTER, 8);
          rew_3=1120;
          EEPROM.write(2, 1);
        }else{
          myGLCD.print("                         ", CENTER, 8);
          myGLCD.print("yaw reward", CENTER, 8);
          rew_3=0;
          EEPROM.write(2, 0);
        }
    }//pin3
    if(iInput==253){set_2=set_2+1;delay(100);}//pin1
    if(iInput==254){set_2=set_2-1;delay(100);}//pin2
    if(set_2>202){set_2=202;}
    if(iInput==251){set_4=set_4-1;delay(100);}//pin3
    if(iInput==247){set_4=set_4+1;delay(100);}//pin4
    if(iInput==127){set_3=set_3+1;delay(100);}//pin5
    if(iInput==191){set_3=set_3-1;delay(100);}//pin6
    if(iInput==223){set_1=set_1+1;delay(100);}//pin7
    if(iInput==239){set_1=set_1-1;delay(100);}//pin8
    if(iInput==255){}//ไม่ได้กด
    if(iInput==253){delay(150);myGLCD.drawBitmap(0, 0, arduino_logo, 84, 48);delay(100);goto SET_UP2; }
    //Serial.println(iInput);
      } 
  }
}
void TX(void){
 
  int sensorValue1 = abs(analogRead(A0)-rew_3)-(559+set_1);
    int sensorValue2 = analogRead(A1)-(204+set_2);
    int sensorValue3 = abs(analogRead(A2)-rew_1)-(519+set_3); 
    int sensorValue4 = abs(analogRead(A3)-rew_2)-(563+set_4);
    tx_buf[7] = (uint8_t)(sensorValue1>>8);    
    tx_buf[8] = (uint8_t)sensorValue1;
    tx_buf[9] = (uint8_t)(sensorValue2>>8);    
    tx_buf[10] = (uint8_t)sensorValue2;  
    tx_buf[11] = (uint8_t)(sensorValue3>>8);    
    tx_buf[12] = (uint8_t)sensorValue3;
    tx_buf[13] = (uint8_t)(sensorValue4>>8);    
    tx_buf[14] = (uint8_t)sensorValue4; 
  unsigned char sstatus = SPI_Read(STATUS);                   // read register STATUS's value
    if(sstatus&TX_DS)                                           // if receive data ready (TX_DS) interrupt
    {
      SPI_RW_Reg(FLUSH_TX,0); 
      // mpu.resetFIFO();      
      SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);       // write playload to TX_FIFO
    }
    if(sstatus&MAX_RT)                                         // if receive data ready (MAX_RT) interrupt, this is retransmit than  SETUP_RETR                          
    {
      SPI_RW_Reg(FLUSH_TX,0);
      SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);      // disable standy-mode
    }
    SPI_RW_Reg(WRITE_REG+STATUS,sstatus);
    //delay(10);
}
//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{
  //digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);			// chip enable
  digitalWrite(CSN, 1);                 // Spi disable	
}

/************************************************************************
**   * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
************************************************************************/
unsigned char SPI_RW(unsigned char Byte)
{
  return SPI.transfer(Byte);
}

/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  digitalWrite(CSN, 0);                   // CSN low, init SPI transaction
  SPI_RW(reg);                            // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(CSN, 1);                   // CSN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  digitalWrite(CSN, 0);                // CSN low, initialize SPI communication...
  SPI_RW(reg);                         // Select register to read from..
  reg_val = SPI_RW(0);                 // ..then read register value
  digitalWrite(CSN, 1);                // CSN high, terminate SPI communication

  return(reg_val);                     // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char sstatus,i;

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(CSN, 1);                   // Set CSN high again

  return(sstatus);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char sstatus,i;

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                   // Set CSN high again
  return(sstatus);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: TX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 * 
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
void TX_Mode(void)
{
  digitalWrite(CE, 0);

  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack

  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...
  SPI_RW_Reg(WRITE_REG + RF_CH, 120);        // Select RF channel 40
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0b);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
  SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);

  digitalWrite(CE, 1);
}
