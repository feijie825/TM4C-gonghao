//*****************************************************************************
//
// bl_uart.c - Functions to transfer data via the UART port.
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
#include "../inc/hw_gpio.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_sysctl.h"
#include "../inc/hw_types.h"
#include "../inc/hw_uart.h"
#include "../User_h/bl_config.h"
#include "../User_h/bl_uart.h"
extern unsigned char Board_Id;
extern volatile unsigned long BOOT_EN[];
//extern volatile unsigned char Timer_Uart;
extern volatile unsigned long Timer_Uart;

//*****************************************************************************
//
//! \addtogroup bl_uart_api
//! @{
//
//*****************************************************************************
#if defined(UART_ENABLE_UPDATE) || defined(DOXYGEN)

//*****************************************************************************
//
//! Sends data over the UART port.
//!
//! \param pui8Data is the buffer containing the data to write out to the UART
//! port.
//! \param ui32Size is the number of bytes provided in \e pui8Data buffer that
//! will be written out to the UART port.
//!
//! This function sends \e ui32Size bytes of data from the buffer pointed to by
//! \e pui8Data via the UART port.
//!
//! \return None.
//
//*****************************************************************************
void
UARTSend(const unsigned char *pucData, unsigned long ulSize)
{
    if(Board_Id!=BOOT_EN[2])    //只有板号设定的表位回送数据
	 return;

    //
    // Transmit the number of bytes requested on the UART port.
    //
    while(ulSize--)
    {
#ifdef BOOT_UART1         //是否通过UART1引导
        //
        // Make sure that the transmit FIFO is not full.
        //
        while((HWREG(UART1_BASE + UART_O_FR) & UART_FR_TXFF))
        {
        }

        //
        // Send out the next byte.
        //
        HWREG(UART1_BASE + UART_O_DR) = *pucData++;
#else
        //
        // Make sure that the transmit FIFO is not full.
        //
        while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF))
        {
        }

        //
        // Send out the next byte.
        //
        HWREG(UART0_BASE + UART_O_DR) = *pucData++;
#endif
    }
    //
    // Wait until the UART is done transmitting.
    //
    UARTFlush();
}

//*****************************************************************************
//
//! Waits until all data has been transmitted by the UART port.
//!
//! This function waits until all data written to the UART port has been
//! transmitted.
//!
//! \return None.
//
//*****************************************************************************
void
UARTFlush(void)
{
#ifdef BOOT_UART1         //是否通过UART1引导
    //
    // Wait for the UART FIFO to empty and then wait for the shifter to get the
    // bytes out the port.
    //
    while(!(HWREG(UART1_BASE + UART_O_FR) & UART_FR_TXFE))
    {
    }

    //
    // Wait for the FIFO to not be busy so that the shifter completes.
    //
    while((HWREG(UART1_BASE + UART_O_FR) & UART_FR_BUSY))
    {
    }
#else
    //
    // Wait for the UART FIFO to empty and then wait for the shifter to get the
    // bytes out the port.
    //
    while(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFE))
    {
    }

    //
    // Wait for the FIFO to not be busy so that the shifter completes.
    //
    while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY))
    {
    }
#endif
}

//*****************************************************************************
//
//! Receives data over the UART port.
//!
//! \param pui8Data is the buffer to read data into from the UART port.
//! \param ui32Size is the number of bytes provided in the \e pui8Data buffer
//! that should be written with data from the UART port.
//!
//! This function reads back \e ui32Size bytes of data from the UART port, into
//! the buffer that is pointed to by \e pui8Data.  This function will not
//! return until \e ui32Size number of bytes have been received.
//!
//! \return None.
//
// 2015.3.12 张力阵修改 添加超时处理
//接收成功 返回 真 不成功返回0
//*****************************************************************************
unsigned char UARTReceive(uint8_t *pucData, uint32_t ulSize)
{
	
    //
    // Send out the number of bytes requested.
    //
    Timer_Uart=0;                              //接收前清除超时定时器
    for(;;)
     {
#ifdef BOOT_UART1         //是否通过UART1引导
      //
      // Wait for the FIFO to not be empty.
      //
       while((HWREG(UART1_BASE + UART_O_FR) & UART_FR_RXFE))
       {
        if(Timer_Uart>200)                      //200ms收不到数据
         return(0);                             //回主程序	
       }
      //
      // Receive a byte from the UART.
      //
      *pucData++ = HWREG(UART1_BASE + UART_O_DR);
#else
      //
      // Wait for the FIFO to not be empty.
      //
      while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE))
       {
        if(Timer_Uart>200)                      //200ms收不到数据
         return(0);                             //回主程序	
       }
      //
      // Receive a byte from the UART.
      //
      *pucData++ = HWREG(UART0_BASE + UART_O_DR);
#endif
      Timer_Uart=0;                            //接收成功 重置定时器
      ulSize--;
      if(ulSize==0) 
       return(0xFF);                           //
     }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
