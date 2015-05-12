void MPU6050_tx(void){
  if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw=(ypr[0] * ((180/M_PI)+70));
            pitch=ypr[1] * 180/M_PI;
            roll=ypr[2] * 180/M_PI;
            tx_buf[1] = (uint8_t)(yaw>>8);    
            tx_buf[2] = (uint8_t)yaw;
            tx_buf[3] = (uint8_t)(pitch>>8);    
            tx_buf[4] = (uint8_t)pitch;
            tx_buf[5] = (uint8_t)(roll>>8);    
            tx_buf[6] = (uint8_t)roll;
//            if(yaw<0){
//              tx_buf[1] =1;
//              tx_buf[2] =-(yaw);
//            }
//            if(yaw>=0){
//              tx_buf[1] =2;
//              tx_buf[2] = yaw;
//            }
//            ////
//            if(pitch<0){
//              tx_buf[3] =1;
//              tx_buf[4] =-(pitch);
//            }
//            if(pitch>=0){
//              tx_buf[3] =2;
//              tx_buf[4] =pitch;
//            }
//            ////////
//            if(roll<0){
//              tx_buf[5] =1;
//              tx_buf[6] =-(roll);
//            }
//            if(roll>=0){
//              tx_buf[5] =2;
//              tx_buf[6] =roll;
//            }
//            Serial.print("ypr\t");
            Serial.println(yaw);
//            Serial.print("\t");
//            Serial.print(pitch);
//            Serial.print("\t");
//            Serial.println(roll);
//            unsigned char sstatus = SPI_Read(STATUS);                   // read register STATUS's value
//            if(sstatus&TX_DS)                                           // if receive data ready (TX_DS) interrupt
//            {
//               SPI_RW_Reg(FLUSH_TX,0);                                  
//              SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);       // write playload to TX_FIFO
//            }
//            if(sstatus&MAX_RT)                                         // if receive data ready (MAX_RT) interrupt, this is retransmit than  SETUP_RETR                          
//            {
//              SPI_RW_Reg(FLUSH_TX,0);
//              SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);      // disable standy-mode
//            }
//            SPI_RW_Reg(WRITE_REG+STATUS,sstatus);                     // clear RX_DR or TX_DS or MAX_RT interrupt flag
          // delay(1);
        #endif
    }
}
