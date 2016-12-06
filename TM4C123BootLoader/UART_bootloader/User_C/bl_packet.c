//*****************************************************************************
//
// bl_packet.c - Packet handler functions used by the boot loader.
//
// Copyright (c) 2006-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include "../User_h/bl_config.h"
#include "../User_h/bl_commands.h"
#include "../User_h/bl_i2c.h"
#include "../User_h/bl_packet.h"
#include "../User_h/bl_ssi.h"
#include "../User_h/bl_uart.h"

//*****************************************************************************
//
//! \addtogroup bl_packet_api
//! @{
//
//*****************************************************************************
#if defined(I2C_ENABLE_UPDATE) || defined(SSI_ENABLE_UPDATE) || \
    defined(UART_ENABLE_UPDATE) || defined(DOXYGEN)

//*****************************************************************************
//
// The packet that is sent to acknowledge a received packet.
//
//*****************************************************************************
static const uint8_t g_pui8ACK[2] = { 0, COMMAND_ACK };

//*****************************************************************************
//
// The packet that is sent to not-acknowledge a received packet.
//
//*****************************************************************************
static const uint8_t g_pui8NAK[2] = { 0, COMMAND_NAK };

//*****************************************************************************
//
//! Calculates an 8-bit checksum
//!
//! \param pui8Data is a pointer to an array of 8-bit data of size ui32Size.
//! \param ui32Size is the size of the array that will run through the checksum
//! algorithm.
//!
//! This function simply calculates an 8-bit checksum on the data passed in.
//!
//! \return Returns the calculated checksum.
//
//*****************************************************************************
uint32_t
CheckSum(const uint8_t *pui8Data, uint32_t ui32Size)
{
    uint32_t ui32CheckSum;

    //
    // Initialize the checksum to zero.
    //
    ui32CheckSum = 0;

    //
    // Add up all the bytes, do not do anything for an overflow.
    //
    while(ui32Size--)
    {
        ui32CheckSum += *pui8Data++;
    }

    //
    // Return the caculated check sum.
    //
    return(ui32CheckSum & 0xff);
}

//*****************************************************************************
//
//! Sends an Acknowledge packet.
//!
//! This function is called to acknowledge that a packet has been received by
//! the microcontroller.
//!
//! \return None.
//
//*****************************************************************************
void
AckPacket(void)
{
    //
    // ACK/NAK packets are the only ones with no size.
    //
    SendData(g_pui8ACK, 2);
}

//*****************************************************************************
//
//! Sends a no-acknowledge packet.
//!
//! This function is called when an invalid packet has been received by the
//! microcontroller, indicating that it should be retransmitted.
//!
//! \return None.
//
//*****************************************************************************
void
NakPacket(void)
{
    //
    // ACK/NAK packets are the only ones with no size.
    //
    SendData(g_pui8NAK, 2);
}

//*****************************************************************************
//
//! Receives a data packet.
//!
//! \param pui8Data is the location to store the data that is sent to the boot
//! loader.
//! \param pui32Size is the number of bytes returned in the pui8Data buffer
//! that was provided.
//!
//! This function receives a packet of data from specified transfer function.
//!
//! \return Returns zero to indicate success while any non-zero value indicates
//! a failure.
//
//*****************************************************************************
int
ReceivePacket(uint8_t *pucData, uint32_t *pulSize)
{
    unsigned long ulSize, ulCheckSum;
    unsigned char Sts;
    ulSize = 0;
    for(;;)
     {
      Sts=ReceiveData((unsigned char *)&ulSize, 1);   //数据长度字节
      if((Sts==0)||                                   //判断是否为接收超时
         (ulSize <3)||                                //包长度小于3 包长度错误
         (ulSize == 0xCC))                            //包长度为0xCC 	
       continue;                                      //等待
      else
       break;
     } 
    ulSize -= 2;                                      //减去长度和校验和 实际数据长度
    Sts=ReceiveData((unsigned char *)&ulCheckSum, 1); //接收校验和字节
    if(Sts==0)                                        //判断是否为接收超时
     return(-1);                                      //返回错误 回主程序                             	
    if(*pulSize >= ulSize)                            //判断 需要接收的数据长度>=包实际长度
     {                                                //
      Sts=ReceiveData(pucData, ulSize);               //接收包数据
      if((Sts==0)||                                   //判断是否为接收超时
      	(CheckSum(pucData, ulSize)!= (ulCheckSum & 0xff))) //检测校验和是否正确
       {                                              //校验和错误 返回错误
        NakPacket();
        return(-1);
       }
     }
    else                                               //包太长 舍弃
     {
      for(;;)
       {
        Sts=ReceiveData(pucData, 1);
        ulSize--;
        if((Sts==0)||                                  //判断是否为接收超时
           (ulSize==0))                                //判断是否接收完毕
        return(-1);
       }
     }
    *pulSize = ulSize;
    return(0);
}

//*****************************************************************************
//
//! Sends a data packet.
//!
//! \param pui8Data is the location of the data to be sent.
//! \param ui32Size is the number of bytes to send.
//!
//! This function sends the data provided in the \e pui8Data parameter in the
//! packet format used by the boot loader.  The caller only needs to specify
//! the buffer with the data that needs to be transferred.  This function
//! addresses all other packet formatting issues.
//!
//! \return Returns zero to indicate success while any non-zero value indicates
//! a failure.
//
//*****************************************************************************
int SendPacket(unsigned char *pucData, unsigned long ulSize)
{

    unsigned long ulTemp;
    unsigned char Sts=0;
    //
    // Caculate the checksum to be sent out with the data.
    //
    ulTemp = CheckSum(pucData, ulSize);

    //
    // Need to include the size and checksum bytes in the packet.
    //
    ulSize += 2;

    //
    // Send out the size followed by the data.
    //
    SendData((unsigned char *)&ulSize, 1);
    SendData((unsigned char *)&ulTemp, 1);
    SendData(pucData, ulSize - 2);

    //
    // Wait for a non zero byte.
    //
    ulTemp = 0;
    Sts=ReceiveData((unsigned char *)&ulTemp, 1);	 //等待响应
    if((Sts==0)||                                    //判断是否接收超时
       (ulTemp != COMMAND_ACK))                      //不为响应命令
     return(-1);  
    //
    // This packet was sent and received successfully.
    //
    return(0);

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
