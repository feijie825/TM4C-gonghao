//*****************************************************************************
//
// bl_check.c - Code to check for a forced update.
//
// Copyright (c) 2006-2013 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 1.0 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "../User_h/bl_config.h"
#include "../User_h/bl_check.h"
#include "../User_h/bl_hooks.h"


extern unsigned long BOOT_EN[];
extern unsigned char Board_Id;

//*****************************************************************************
//
//! \addtogroup bl_check_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// This global is used to remember if a forced update occurred.
//
//*****************************************************************************
#ifdef ENABLE_UPDATE_CHECK
uint32_t g_ui32Forced;
#endif

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);

//*****************************************************************************
//
//! Checks a GPIO for a forced update.
//!
//! This function checks the state of a GPIO to determine if a update is being
//! requested.
//!
//! \return Returns a non-zero value if an update is being requested and zero
//! otherwise.
//
//*****************************************************************************
#ifdef ENABLE_UPDATE_CHECK
uint32_t
CheckGPIOForceUpdate(void)
{
    //
    // Enable the required GPIO module.
    //
    HWREG(SYSCTL_RCGC2) |= FORCED_UPDATE_PERIPH;  //端口使能 使能运行模式时钟使能寄存器对应位

    //
    // Wait a while before accessing the peripheral.
    //
    Delay(3);

#ifdef FORCED_UPDATE_KEY                          //强制更新 管脚解锁
    //
    // Unlock the GPIO Access.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_LOCK) = FORCED_UPDATE_KEY;      //解锁
    HWREG(FORCED_UPDATE_PORT + GPIO_O_CR) = (1 << FORCED_UPDATE_PIN);   //GPIOAFSEL, GPIOPUR, GPIOPDR, or GPIODEN bits can be written

#endif

    //
    // Enable the pin used to see if an update is being requested.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_AFSEL) &= ~(1 << FORCED_UPDATE_PIN);    //IO口

    HWREG(FORCED_UPDATE_PORT + GPIO_O_DEN) |= 1 << FORCED_UPDATE_PIN;

    HWREG(FORCED_UPDATE_PORT + GPIO_O_PCTL) &= ~(0x0f << FORCED_UPDATE_PIN);    //IO口

#ifdef FORCED_UPDATE_WPU                                //是否定义强制上拉
    //
    // Set the output drive strength.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_DR2R) |= 1 << FORCED_UPDATE_PIN;

    //
    // Enable the weak pull up.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_PUR) |= 1 << FORCED_UPDATE_PIN;

    //
    // Make sure that the analog mode select register is clear for this pin.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_AMSEL) &= ~(1 << FORCED_UPDATE_PIN);
#endif
#ifdef FORCED_UPDATE_WPD                                //是否定义强制下拉 bl_config.h
    //
    // Set the output drive strength.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_DR2R) |= 1 << FORCED_UPDATE_PIN;

    //
    // Enable the weak pull down.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_PDR) |= 1 << FORCED_UPDATE_PIN;

    //
    // Make sure that the analog mode select register is clear for this pin.
    // This register only appears in DustDevil-class (and later) devices, but
    // is a harmless write on Sandstorm- and Fury-class devices.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_AMSEL) &= ~(1 << FORCED_UPDATE_PIN);
#endif

#ifdef FORCED_UPDATE_KEY
    //
    // Unlock the GPIO Access.
    //
    HWREG(FORCED_UPDATE_PORT + GPIO_O_LOCK) = FORCED_UPDATE_KEY;
    HWREG(FORCED_UPDATE_PORT + GPIO_O_CR) = 0;
#endif

    //
    // Wait a while before reading the pin.
    //
    Delay(1000);

    //
    // Check the pin to see if an update is being requested.
    //
    if(HWREG(FORCED_UPDATE_PORT + (1 << (FORCED_UPDATE_PIN + 2))) ==
       (FORCED_UPDATE_POLARITY << FORCED_UPDATE_PIN))
    {
        //
        // Remember that this was a forced update.
        //
        g_ui32Forced = 1;

        return(1);
    }

    //
    // No update is being requested so return 0.
    //
    return(0);
}
#endif

//*****************************************************************************
//
//! Checks if an update is needed or is being requested.
//!
//! This function detects if an update is being requested or if there is no
//! valid code presently located on the microcontroller.  This is used to tell
//! whether or not to enter update mode.
//!
//! \return Returns a non-zero value if an update is needed or is being
//! requested and zero otherwise.
//
//*****************************************************************************
uint32_t
CheckForceUpdate(void)
{
#ifdef BL_CHECK_UPDATE_FN_HOOK
    //
    // If the update check function is hooked, call the application to determine
    // how to proceed.
    //
    return(BL_CHECK_UPDATE_FN_HOOK());
#else
    uint32_t *pui32App;

#ifdef ENABLE_UPDATE_CHECK
    g_ui32Forced = 0;
#endif
    Delay(200);
//  读板号
    //
    // Enable the required GPIO module.
    //
    HWREG(SYSCTL_RCGC2) |= BOARD_ID_PERIPH;       //使能端口


    //
    // Wait a while before accessing the peripheral.
    //
    Delay(3);

    //
    // Enable the pin used to see if an update is being requested.
    //
    HWREG(BOARD_ID_PORT + GPIO_O_DEN) |= 0xFF;     //使能数字功能

    //
    // Set the output drive strength.
    //
    HWREG(BOARD_ID_PORT + GPIO_O_DR2R) |= 0xFF;

    //
    // Disable the weak pull up.
    //
    HWREG(BOARD_ID_PORT + GPIO_O_PUR) = 0;         //禁用上拉  
 

    //
    // Disable the weak pull down.
    //
    HWREG(BOARD_ID_PORT + GPIO_O_PDR) = 0;         //禁用下拉
		
    //
    // Make sure that the analog mode select register is clear for this pin.
    //
    HWREG(BOARD_ID_PORT + GPIO_O_AMSEL) = 0;       //关闭备用功能

    //
    // 设定为输入
    //
    HWREG(BOARD_ID_PORT + GPIO_O_DIR) = 0;         //输入

    //
    // Wait a while before reading the pin.
    //
    Delay(1000);

    Board_Id=HWREG(BOARD_ID_PORT + 0x3fc);         //端口数据
    Board_Id++;                                    //板号 

    //
    // See if the first location is 0xfffffffff or something that does not
    // look like a stack pointer, or if the second location is 0xffffffff or
    // something that does not look like a reset vector.
    //
    if((BOOT_EN[0]=='C')&&
       (BOOT_EN[1]=='C'))                          //是否为通过 应用程序跳转来的UART升级请求
     return(1);
    else			
     BOOT_EN[2]=Board_Id;                          //不是 该板 返回数据

    //
    // See if the first location is 0xfffffffff or something that does not
    // look like a stack pointer, or if the second location is 0xffffffff or
    // something that does not look like a reset vector.
    // 检查用户程序区第1个字 是否为空 或 不为堆栈指针
    // 检查用户程序区第2个字 是否为空 或 不像复位向量
    pui32App = (uint32_t *)APP_START_ADDRESS;  //用户程序地址
    if((pui32App[0] == 0xffffffff) ||
       ((pui32App[0] & 0xfff00000) != 0x20000000) ||
       (pui32App[1] == 0xffffffff) ||
       ((pui32App[1] & 0xfff00001) != 0x00000001))
    {
        return(1);
    }

#ifdef ENABLE_UPDATE_CHECK
    //
    // If simple GPIO checking is configured, determine whether or not to force
    // an update.
    // 检查用户指定I/O口
    return(CheckGPIOForceUpdate()); 
#else
    //
    // GPIO checking is not required so, if we get here, a valid image exists
    // and no update is needed.
    //
    return(0);
#endif
#endif
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
