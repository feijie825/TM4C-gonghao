//*****************************************************************************
//
// bl_main.c - The file holds the main control loop of the boot loader.
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
#include <stdbool.h>
#include "../inc/hw_gpio.h"
#include "../inc/hw_flash.h"
#include "../inc/hw_i2c.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_nvic.h"
#include "../inc/hw_ssi.h"
#include "../inc/hw_sysctl.h"
#include "../inc/hw_types.h"
#include "../inc/hw_uart.h"
#include "../inc/hw_ints.h"
#include "../User_h/bl_config.h"
#include "../User_h/bl_commands.h"
#include "../User_h/bl_decrypt.h"
#include "../User_h/bl_flash.h"
#include "../User_h/bl_hooks.h"
#include "../User_h/bl_i2c.h"
#include "../User_h/bl_packet.h"
#include "../User_h/bl_ssi.h"
#include "../User_h/bl_uart.h"
#include "disp.h"
#ifdef CHECK_CRC
#include "../User_h/bl_crc32.h"
#endif

#include "../inc/debug.h"
#include "../inc/sysctl.h"
#include "../driverlib/ssi.h"
#include "define.h"
#include "../inc/gpio.h"
#include "../inc/pin_map.h"
#include "../inc/systick.h"
#include "../inc/interrupt.h"
//*****************************************************************************
//
// The base addresses of the various peripheral control registers.
//
//*****************************************************************************
#define SYSCTL_PPBASE           0x400fe300
#define SYSCTL_SRBASE           0x400fe500
#define SYSCTL_RCGCBASE         0x400fe600
#define SYSCTL_SCGCBASE         0x400fe700
#define SYSCTL_DCGCBASE         0x400fe800
#define SYSCTL_PCBASE           0x400fe900
#define SYSCTL_PRBASE           0x400fea00

//中断优先级定义
#define INT_PRIORITY_0          0x00//优先级0 最高
#define INT_PRIORITY_1          0x20//优先级1 
#define INT_PRIORITY_2          0x40//优先级2 
#define INT_PRIORITY_3          0x60//优先级3 
#define INT_PRIORITY_4          0x80//优先级4
#define INT_PRIORITY_5          0xa0//优先级5
#define INT_PRIORITY_6          0xc0//优先级6
#define INT_PRIORITY_7          0xe0//优先级7 最低

//*****************************************************************************
//
// Make sure that the application start address falls on a flash page boundary
//
//*****************************************************************************
#if (APP_START_ADDRESS & (FLASH_PAGE_SIZE - 1))
#error ERROR: APP_START_ADDRESS must be a multiple of FLASH_PAGE_SIZE bytes!
#endif

//*****************************************************************************
//
// Make sure that the flash reserved space is a multiple of flash pages.
//
//*****************************************************************************
#if (FLASH_RSVD_SPACE & (FLASH_PAGE_SIZE - 1))
#error ERROR: FLASH_RSVD_SPACE must be a multiple of FLASH_PAGE_SIZE bytes!
#endif

//*****************************************************************************
//
//! \addtogroup bl_main_api
//! @{
//
//*****************************************************************************
#if defined(I2C_ENABLE_UPDATE) || defined(SSI_ENABLE_UPDATE) || \
    defined(UART_ENABLE_UPDATE) || defined(DOXYGEN)

//*****************************************************************************
//
// A prototype for the function (in the startup code) for calling the
// application.
//
//*****************************************************************************
extern void CallApplication(uint32_t ui32Base);

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);

//*****************************************************************************
//
// Holds the current status of the last command that was issued to the boot
// loader.
//
//*****************************************************************************
uint8_t g_ui8Status;

//*****************************************************************************
//
// This holds the current remaining size in bytes to be downloaded.
//
//*****************************************************************************
uint32_t g_ui32TransferSize;

//*****************************************************************************
//
// This holds the total size of the firmware image being downloaded (if the
// protocol in use provides this).
//
//*****************************************************************************
#if (defined BL_PROGRESS_FN_HOOK) || (defined CHECK_CRC)
uint32_t g_ui32ImageSize;
#endif

//*****************************************************************************
//
// This holds the current address that is being written to during a download
// command.
//
//*****************************************************************************
uint32_t g_ui32TransferAddress;
#ifdef CHECK_CRC
uint32_t g_ui32ImageAddress;
#endif

//*****************************************************************************
//
// This is the data buffer used during transfers to the boot loader.
//
//*****************************************************************************
uint32_t g_pui32DataBuffer[BUFFER_SIZE];

//*****************************************************************************
//
// This is an specially aligned buffer pointer to g_pui32DataBuffer to make
// copying to the buffer simpler.  It must be offset to end on an address that
// ends with 3.
//
//*****************************************************************************
uint8_t *g_pui8DataBuffer;

unsigned long BOOT_EN[BOOT_EN_SIZE];

unsigned char Board_Id;
unsigned char BootLoader_En; //引导区更新使能命令
volatile unsigned int  Timer_1ms;
//volatile unsigned char Timer_Uart;
volatile unsigned long Timer_Uart;
unsigned char Disp_Sts;      //显示状态 bit7 显示状态

//*****************************************************************************
//
// Converts a word from big endian to little endian.  This macro uses compiler-
// specific constructs to perform an inline insertion of the "rev" instruction,
// which performs the byte swap directly.
//
//*****************************************************************************
#if defined(ewarm)
#include <intrinsics.h>
#define SwapWord(x)             __REV(x)
#endif
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
#define SwapWord(x) __extension__                                             \
        ({                                                                    \
             register uint32_t __ret, __inp = x;                              \
             __asm__("rev %0, %1" : "=r" (__ret) : "r" (__inp));              \
             __ret;                                                           \
        })
#endif
#if defined(rvmdk) || defined(__ARMCC_VERSION)
#define SwapWord(x)             __rev(x)
#endif
#if defined(ccs)
uint32_t
SwapWord(uint32_t x)
{
    __asm("    rev     r0, r0\n"
          "    bx      lr\n"); // need this to make sure r0 is returned
    return(x + 1); // return makes compiler happy - ignored
}
#endif

//*****************************************************************************
//
//! Handles the SysTick interrupt.
//!
//! This function is called when the SysTick interrupt occurs.  It simply
//! keeps a running count of interrupts, used as a time basis for the BOOTP and
//! TFTP protocols.
//!
//! This function is contained in <tt>bl_enet.c</tt>.
//!
//! \return None.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Increment the tick count.
    //
    Timer_1ms++;
    Timer_Uart++;
    if((Timer_1ms % 0x100) == 0)                              //200ms
//     GPIOPinWrite(GPIOH,TXXZ_MC,~GPIOPinRead(GPIOH,TXXZ_MC)); //复位外部硬件看门狗 ;
      GPIOPinWrite(WDI_GPIO,WDI,~GPIOPinRead(WDI_GPIO,WDI));    //复位外部硬件看门狗
}
//*****************************************************************************
//
//! Configures the synchronous serial interface.
//!
//! \param ui32Base specifies the SSI module base address.
//! \param ui32SSIClk is the rate of the clock supplied to the SSI module.
//! \param ui32Protocol specifies the data transfer protocol.
//! \param ui32Mode specifies the mode of operation.
//! \param ui32BitRate specifies the clock rate.
//! \param ui32DataWidth specifies number of bits transferred per frame.
//!
//! This function configures the synchronous serial interface.  It sets
//! the SSI protocol, mode of operation, bit rate, and data width.
//!
//! The \e ui32Protocol parameter defines the data frame format.  The
//! \e ui32Protocol parameter can be one of the following values:
//! \b SSI_FRF_MOTO_MODE_0, \b SSI_FRF_MOTO_MODE_1, \b SSI_FRF_MOTO_MODE_2,
//! \b SSI_FRF_MOTO_MODE_3, \b SSI_FRF_TI, or \b SSI_FRF_NMW.  The Motorola
//! frame formats encode the following polarity and phase configurations:
//!
//! <pre>
//! Polarity Phase       Mode
//!   0       0   SSI_FRF_MOTO_MODE_0
//!   0       1   SSI_FRF_MOTO_MODE_1
//!   1       0   SSI_FRF_MOTO_MODE_2
//!   1       1   SSI_FRF_MOTO_MODE_3
//! </pre>
//!
//! The \e ui32Mode parameter defines the operating mode of the SSI module.
//! The SSI module can operate as a master or slave; if it is a slave, the SSI
//! can be configured to disable output on its serial output line.  The
//! \e ui32Mode parameter can be one of the following values:
//! \b SSI_MODE_MASTER, \b SSI_MODE_SLAVE, or \b SSI_MODE_SLAVE_OD.
//!
//! The \e ui32BitRate parameter defines the bit rate for the SSI.  This bit
//! rate must satisfy the following clock ratio criteria:
//!
//! - FSSI >= 2 * bit rate (master mode); this speed cannot exceed 25 MHz.
//! - FSSI >= 12 * bit rate or 6 * bit rate (slave modes), depending on the
//! capability of the specific microcontroller
//!
//! where FSSI is the frequency of the clock supplied to the SSI module.
//!
//! The \e ui32DataWidth parameter defines the width of the data transfers and
//! can be a value between 4 and 16, inclusive.
//!
//! The peripheral clock is the same as the processor clock.  This value is
//! returned by SysCtlClockGet(), or it can be explicitly hard coded if it is
//! constant and known (to save the code/execution overhead of a call to
//! SysCtlClockGet()).
//!
//! \return None.
//
//*****************************************************************************
void
SSIConfigSetExpClk(uint32_t ui32Base, uint32_t ui32SSIClk,
                   uint32_t ui32Protocol, uint32_t ui32Mode,
                   uint32_t ui32BitRate, uint32_t ui32DataWidth)
{
    uint32_t ui32MaxBitRate;
    uint32_t ui32RegVal;
    uint32_t ui32PreDiv;
    uint32_t ui32SCR;
    uint32_t ui32SPH_SPO;

    //
    // Check the arguments.
    //
    ASSERT(_SSIBaseValid(ui32Base));
    ASSERT((ui32Protocol == SSI_FRF_MOTO_MODE_0) ||
           (ui32Protocol == SSI_FRF_MOTO_MODE_1) ||
           (ui32Protocol == SSI_FRF_MOTO_MODE_2) ||
           (ui32Protocol == SSI_FRF_MOTO_MODE_3) ||
           (ui32Protocol == SSI_FRF_TI) ||
           (ui32Protocol == SSI_FRF_NMW));
    ASSERT((ui32Mode == SSI_MODE_MASTER) ||
           (ui32Mode == SSI_MODE_SLAVE) ||
           (ui32Mode == SSI_MODE_SLAVE_OD));
    ASSERT(((ui32Mode == SSI_MODE_MASTER) &&
            (ui32BitRate <= (ui32SSIClk / 2))) ||
           ((ui32Mode != SSI_MODE_MASTER) &&
            (ui32BitRate <= (ui32SSIClk / 12))));
    ASSERT((ui32SSIClk / ui32BitRate) <= (254 * 256));
    ASSERT((ui32DataWidth >= 4) && (ui32DataWidth <= 16));

    //
    // Set the mode.
    //
    ui32RegVal = (ui32Mode == SSI_MODE_SLAVE_OD) ? SSI_CR1_SOD : 0;
    ui32RegVal |= (ui32Mode == SSI_MODE_MASTER) ? 0 : SSI_CR1_MS;
    HWREG(ui32Base + SSI_O_CR1) = ui32RegVal;

    //
    // Set the clock predivider.
    //
    ui32MaxBitRate = ui32SSIClk / ui32BitRate;
    ui32PreDiv = 0;
    do
    {
        ui32PreDiv += 2;
        ui32SCR = (ui32MaxBitRate / ui32PreDiv) - 1;
    }
    while(ui32SCR > 255);
    HWREG(ui32Base + SSI_O_CPSR) = ui32PreDiv;

    //
    // Set protocol and clock rate.
    //
    ui32SPH_SPO = (ui32Protocol & 3) << 6;
    ui32Protocol &= SSI_CR0_FRF_M;
    ui32RegVal = (ui32SCR << 8) | ui32SPH_SPO | ui32Protocol |
                 (ui32DataWidth - 1);
    HWREG(ui32Base + SSI_O_CR0) = ui32RegVal;
}
//*****************************************************************************
//
//! Enables the synchronous serial interface.
//!
//! \param ui32Base specifies the SSI module base address.
//!
//! This function enables operation of the synchronous serial interface.  The
//! synchronous serial interface must be configured before it is enabled.
//!
//! \return None.
//
//*****************************************************************************
void
SSIEnable(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT(_SSIBaseValid(ui32Base));

    //
    // Read-modify-write the enable bit.
    //
    HWREG(ui32Base + SSI_O_CR1) |= SSI_CR1_SSE;
}
/*****************************************************************************
* 初始化GPIO口 
*****************************************************************************/
void Init_Gpio(void)
{
//设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能端口A时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能端口B时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能端口C时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能端口D时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能端口E时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能端口F时钟
//	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);//使能端口G时钟，在读地址子程序中使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//使能端口H时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);//使能端口J时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);//使能端口K时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);//使能端口L时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);//使能端口M时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);//使能端口N时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);//使能端口P时钟
//初始化GPIOA口
	   GPIOPinConfigure(GPIO_PA0_U0RX);
	   GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(UART0_GPIO,             //端口
                    U0RX|                   //管脚 初始化UART0接口管脚	 PA.0 PA.1
                    U0TX);                  //管脚 初始化UART0接口管脚	 PA.0 PA.1

	   GPIOPinTypeGPIOOutput(DISP_RST_GPIO,    //端口
                          DISP_RST);        //管脚 PA6 HD7219复位输出
																										
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
				GPIOPinConfigure(GPIO_PA3_SSI0FSS);
				GPIOPinConfigure(GPIO_PA4_SSI0RX);
				GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(SPI0_GPIO,               //端口
                   SSICLK|SSIFSS|           //管脚 SSICLK SSIFSS SSIRX SSITX设置为SSI管脚
																			SSIRX|SSITX);            //PA2 PA3 PA4 PA5

    GPIOPinTypeGPIOInput(KEY_IN_GPIO,       //端口
                         KEY_IN);           //按键输入口PA7
//初始化GPIOB口   
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
				GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(CAN0_GPIO,               //端口，初始化CAN0接口管脚
                   CAN0RX|                  //管脚PB4
                   CAN0TX);                 //PB5
																			
    GPIOPinTypeGPIOInput(YK1NO_GPIO,        //端口
                         YK1NO);            //遥控信号1常开
    GPIOPinTypeGPIOInput(YK1NC_GPIO,        //端口
                         YK1NC);            //遥控信号1常闭
				GPIOPinTypeGPIOInput(YK2NO_GPIO,        //端口
                         YK2NO);            //遥控信号2常开	
    GPIOPinTypeGPIOInput(YK2NC_GPIO,        //端口
                         YK2NC);            //遥控信号2常闭																									
//初始化GPIOC口																									
				GPIOPinConfigure(GPIO_PC4_U1RX);
				GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(UART1_GPIO,             //端口
                    U1RX|                   //管脚PC4，PC5
                    U1TX);                  //初始化UART1接口管脚
				GPIOPinConfigure(GPIO_PC6_U3RX);																
				GPIOPinConfigure(GPIO_PC7_U3TX);
				GPIOPinTypeUART(UART3_GPIO,             //端口
				                U3RX|                   //管脚PC6，PC7
																				U3TX);                  //初始化UART3接口管脚
//初始化GPIOD口
				GPIOPinConfigure(GPIO_PD4_U6RX);
				GPIOPinConfigure(GPIO_PD5_U6TX);
    GPIOPinTypeUART(UART6_GPIO,             //端口
                    U6RX|                   //管脚PD4，PD5
                    U6TX);                  //初始化UART6接口管脚    

//初始化GPIOE口
				GPIOPinConfigure(GPIO_PE0_U7RX);
				GPIOPinConfigure(GPIO_PE1_U7TX);
    GPIOPinTypeUART(UART7_GPIO,             //端口
                    U7RX|                   //管脚
                    U7TX);                  //初始化UART7接口管脚
    GPIOPinTypeGPIOOutput(TZ_ZS_GPIO,       //端口
                          TZ_ZS);           //跳闸指示灯控制
				GPIOPinConfigure(GPIO_PE4_U5RX);
				GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(UART5_GPIO,             //端口
                    U5RX|                   //管脚
                    U5TX);                  //初始化UART5接口管脚
    GPIOPinConfigure(GPIO_PE6_CAN1RX);
				GPIOPinConfigure(GPIO_PE7_CAN1TX);
    GPIOPinTypeCAN(CAN1_GPIO,               //端口
                   CAN1RX|                  //管脚
                   CAN1TX);                 //初始化CAN1接口管脚
//初始化GPIOF口
/* 解锁IO口,PF0*/
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0xFF;               /* bits can be written */
    GPIOPinTypeGPIOOutput(UC_ESWC_GPIO,     //端口
                          UC_ESWC);         //C相电压电子开关控制
    GPIOPinTypeGPIOOutput(UB_ESWC_GPIO,     //端口
                          UB_ESWC);         //B相电压电子开关控制
    GPIOPinTypeGPIOOutput(UA_ESWC_GPIO,     //端口
                          UA_ESWC);         //A相电压电子开关控制 
    GPIOPinTypeGPIOOutput(GBC_C_GPIO,       //端口
                          GBC_C);           //C相续流继电器断开控制 		
    GPIOPinTypeGPIOOutput(TEST_LAMP_GPIO,   //端口
                          TEST_LAMP);	      //管脚PN5 检定指示灯控制引脚																										
    GPIOPinTypeGPIOOutput(UC_JC_GPIO,       //端口
                          UC_JC);           //C相电压继电器控制 
    GPIOPinTypeGPIOOutput(UA_JC_GPIO,       //端口
                          UA_JC);	          //管脚PN6 A相电压继电器控制
    GPIOPinTypeGPIOOutput(UB_JC_GPIO,       //端口
                          UB_JC);	          //管脚PN7 B相电压继电器控制 																										
   
//初始化GPIOG口     
//初始化GPIOH口   
    GPIOPinTypeGPIOOutput(ABBA_GPIO,        //端口
                          ABBA);            //RS485极性翻转控制引脚
//初始化GPIOJ口
    GPIOPinTypeGPIOOutput(MC_OUT1_GPIO,     //端口
                          MC_OUT1);         //脉冲输出1,PJ0  
    GPIOPinTypeGPIOOutput(MC_OUT2_GPIO,     //端口
                          MC_OUT2);         //脉冲输出2,PJ1
    GPIOPinTypeGPIOOutput(MC_OUT3_GPIO,     //端口
                          MC_OUT3);         //脉冲输出3,PJ2
    GPIOPinTypeGPIOOutput(MC_OUT4_GPIO,     //端口
                          MC_OUT4);         //脉冲输出4,PJ3
    GPIOPinTypeGPIOOutput(MC_OUT5_GPIO,     //端口
                          MC_OUT5);         //脉冲输出5,PJ4
    GPIOPinTypeGPIOOutput(MC_OUT6_GPIO,     //端口
                          MC_OUT6);         //脉冲输出6,PJ5
    GPIOPinTypeGPIOOutput(MC_OUT7_GPIO,     //端口
                          MC_OUT7);         //脉冲输出7,PJ6
    GPIOPinTypeGPIOOutput(MC_OUT8_GPIO,     //端口
                          MC_OUT8);         //脉冲输出8,PJ7
//初始化GPIOK口       
    GPIOPinTypeGPIOInput(YK4NO_GPIO,        //端口
                         YK4NO);            //遥控信号4常开,PK4
    GPIOPinTypeGPIOInput(YK4NC_GPIO,        //端口
                         YK4NC);            //遥控信号4常闭,PK5
				GPIOPinTypeGPIOInput(YK3NO_GPIO,        //端口
                         YK3NO);            //遥控信号3常开,PK6	
    GPIOPinTypeGPIOInput(YK3NC_GPIO,        //端口
                         YK3NC);            //遥控信号3常闭,PK7
//初始化GPIOL口    
    GPIOPinConfigure(GPIO_PL0_T0CCP0);      //配置I/O口复用功能；PL0
				GPIOPinConfigure(GPIO_PL1_T0CCP1);      //配置I/O口复用功能；PL1
				GPIOPinConfigure(GPIO_PL2_T1CCP0);      //配置I/O口复用功能；PL2
				GPIOPinConfigure(GPIO_PL3_T1CCP1);      //配置I/O口复用功能；PL3
				GPIOPinConfigure(GPIO_PL4_T2CCP0);      //配置I/O口复用功能；PL4
				GPIOPinConfigure(GPIO_PL5_T2CCP1);      //配置I/O口复用功能；PL5
				GPIOPinConfigure(GPIO_PL6_T3CCP0);      //配置I/O口复用功能；PL6
				GPIOPinConfigure(GPIO_PL7_T3CCP1);      //配置I/O口复用功能；PL7				
    GPIOPinTypeTimer(HFIN1_GPIO,            //端口
				                 HFIN1);                //管脚
    GPIOPinTypeTimer(HFIN2_GPIO,            //端口
				                 HFIN2);                //管脚
    GPIOPinTypeTimer(HFIN3_GPIO,            //端口
				                 HFIN3);                //管脚		
    GPIOPinTypeTimer(FH_IN_GPIO,            //端口
                     FH_IN);                //管脚
    GPIOPinTypeTimer(JZ_IN_GPIO,            //端口
                     JZ_IN);                //管脚
    GPIOPinTypeTimer(QFH_IN_GPIO,
				                 QFH_IN);																			
    GPIOPinTypeTimer(WGFMC_GPIO,            //端口 
                     WGFMC);                //管脚		
    GPIOPinTypeTimer(YGFMC_GPIO,            //端口
                     YGFMC);                //管脚  																					
//初始化GPIOM口				    
    GPIOPinConfigure(GPIO_PM0_T4CCP0);      //配置I/O口复用功能；PM0
				GPIOPinConfigure(GPIO_PM1_T4CCP1);      //配置I/O口复用功能；PM1
				GPIOPinConfigure(GPIO_PM2_T5CCP0);      //配置I/O口复用功能；PM2
				
    GPIOPinTypeTimer(WGMC_GPIO,             //端口 
                     WGMC);                 //管脚 																								
				GPIOPinTypeTimer(YGMC_GPIO,             //端口 
                     YGMC);                 //管脚
    GPIOPinTypeTimer(SZ_MC_GPIO,            //端口 
                     SZ_MC);                //管脚 																					
																						
    GPIOPinTypeGPIOInput(GDT_MC_GPIO,       //端口
                         GDT_MC);           //管脚PM3 光电头脉冲输入
    GPIOPinTypeGPIOOutput(GDT_RST_GPIO,     //端口
                          GDT_RST);         //管脚PM4 光电头复位
    GPIOPinTypeGPIOInput(XL_MC_GPIO,        //端口
                         XL_MC);            //管脚PM5 需量周期输入
    GPIOPinTypeGPIOInput(TQ_MC_GPIO,        //端口
                         TQ_MC);            //备用PM6 时段投切输入    
				GPIOPinTypeGPIOOutput(GOG_KZ_GPIO,      //端口
                          GOG_KZ);          //管脚PM7 被检表脉冲共高共低选择控制 																										
//初始化GPION口															
    GPIOPinTypeGPIOOutput(TTAB1_GPIO,       //端口
				                      TTAB1);           //管脚PN0 双回路切换1
    GPIOPinTypeGPIOOutput(TTAB2_GPIO,       //端口
				                      TTAB2);           //管脚PN1 双回路切换2
    GPIOPinTypeGPIOOutput(WDI_GPIO,         //端口
				                      WDI);             //管脚PN3 外置看门狗喂狗引脚
    GPIOPinTypeGPIOInput(OPEN_IN_GPIO,      //端口
				                     OPEN_IN);          //管脚PN4 续流保护继电器状态输入引脚
    GPIOPinTypeGPIOOutput(BGB_C_GPIO,       //端口
                          BGB_C);           //管脚PN5 三相续流继电器闭合控制
    GPIOPinTypeGPIOOutput(GBA_C_GPIO,       //端口
                          GBA_C);           //管脚PN6 A相续流继电器断开控制
    GPIOPinTypeGPIOOutput(GBB_C_GPIO,       //端口
                          GBB_C);           //管脚PN7 B相续流继电器断开控制																									
              
//初始化GPIOP口  			
    GPIOPinTypeGPIOOutput(DOOR_GPIO,        //端口
                          DOOR);            //管脚PP0 门控信号
    GPIOPinTypeGPIOInput(BJ_SIG_GPIO,       //端口
                         BJ_SIG);           //管脚PP1 电表报警信号																									
       
//初始化端口中断 GPIOA
    GPIOPadConfigSet(KEY_IN_GPIO,           //端口 设置管脚类型    
                     KEY_IN,                //管脚             
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(KEY_IN_GPIO,             //端口 设置管脚中断发送
                   KEY_IN,                  //管脚             
                   GPIO_FALLING_EDGE);      //中断方式             
//初始化端口中断 GPIOM
    GPIOPadConfigSet(GDT_MC_GPIO,           //端口 设置管脚类型    
                     GDT_MC,                //管脚                 
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉            
    GPIOIntTypeSet(GDT_MC_GPIO,             //端口 设置管脚中断发送
                   GDT_MC,                  //管脚                 
                   GPIO_FALLING_EDGE);      //中断方式
    GPIOPadConfigSet(XL_MC_GPIO,            //端口 设置管脚类型    
                     XL_MC,                 //管脚 需量输入 时段投切 合闸脉冲
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(XL_MC_GPIO,              //端口 设置管脚中断方式
                   XL_MC,                   //管脚 需量输入 时段投切 合闸脉冲
                   GPIO_FALLING_EDGE);      //中断方式             
    GPIOPadConfigSet(TQ_MC_GPIO,            //端口 设置管脚类型    
                     TQ_MC,                 //管脚 需量输入 时段投切 合闸脉冲
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(TQ_MC_GPIO,              //端口 设置管脚中断方式
                   TQ_MC,                   //管脚 需量输入 时段投切 合闸脉冲
                   GPIO_FALLING_EDGE);      //中断方式             
//初始化端口中断 GPION
    GPIOIntTypeSet(OPEN_IN_GPIO,            //端口 设置管脚中断方式
				               OPEN_IN,                 //管脚 续流继电器状态信号输入
																			GPIO_FALLING_EDGE);      //中断方式
                     
//端口初步设置
    DOWN_JOIN;                              //脉冲输出共E
    DISP_RST_EN;                            //显示复位
    GDT_RST_DN;                             //
    TEST_LAMP_OFF;                          //校验指示灯灭
    WDI_HIGH;                               //
}

/*****************************************************************************
* 初始化SSI
*****************************************************************************/
void Init_Ssi(void)
{
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);//使能SSI0时钟	
    HWREG(SYSCTL_RCGC1)|=SYSCTL_RCGC1_SSI0;	
    SSIConfigSetExpClk(SSI0_BASE,CRYSTAL_FREQ ,\
                            SSI_FRF_MOTO_MODE_0,\
                            SSI_MODE_MASTER,\
                            SSI_BITRATE,\
                            SSI_CR0_DSS_8);    //配置SSI CR0
    SSIEnable(SSI0_BASE);                      //使能SSI0
}
/*****************************************************************************
* 初始化系统节拍定时器 
*****************************************************************************/
void Init_SysTick(void)
{
    SysTickPeriodSet((CRYSTAL_FREQ/1000000)*SYS_TIME); //设置系统节拍定时周期
    SysTickEnable();			                  //使能系统节拍定时器
    SysTickIntEnable();			                //使能系统节拍定时器中断
}

//*****************************************************************************
//
//! Configures the microcontroller.
//!
//! This function configures the peripherals and GPIOs of the microcontroller,
//! preparing it for use by the boot loader.  The interface that has been
//! selected as the update port will be configured, and auto-baud will be
//! performed if required.
//!
//! \return None.
//
//*****************************************************************************
void
ConfigureDevice(void)
{
#ifdef UART_ENABLE_UPDATE
    uint32_t ui32ProcRatio;
#endif

#ifdef CRYSTAL_FREQ  //使用晶振 不用PLL
    //
    // Since the crystal frequency was specified, enable the main oscillator
    // and clock the processor from it.
    //
    HWREG(SYSCTL_RCC) &= ~(SYSCTL_RCC_MOSCDIS);
    Delay(100000);
    HWREG(SYSCTL_RCC) = ((HWREG(SYSCTL_RCC) & ~(SYSCTL_RCC_OSCSRC_M)) |
                         SYSCTL_RCC_OSCSRC_MAIN);
#endif

#ifdef I2C_ENABLE_UPDATE
    //
    // Enable the clocks to the I2C and GPIO modules.
    //
    HWREG(SYSCTL_RCGC2) |= SYSCTL_RCGC2_GPIOB;
    HWREG(SYSCTL_RCGC1) |= SYSCTL_RCGC1_I2C0;
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);         //使能GPIOB
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);          //使能I2C0

    //
    // Configure the GPIO pins for hardware control, open drain with pull-up,
    // and enable them.
    //
    HWREG(GPIO_PORTB_BASE + GPIO_O_AFSEL) |= (1 << 7) | I2C_PINS;
    HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= (1 << 7) | I2C_PINS;
    HWREG(GPIO_PORTB_BASE + GPIO_O_ODR) |= I2C_PINS;

    //
    // Enable the I2C Slave Mode.
    //
    HWREG(I2C0_BASE + I2C_O_MCR) = I2C_MCR_MFE | I2C_MCR_SFE;

    //
    // Setup the I2C Slave Address.
    //
    HWREG(I2C0_BASE + I2C_O_SOAR) = I2C_SLAVE_ADDR;

    //
    // Enable the I2C Slave Device on the I2C bus.
    //
    HWREG(I2C0_BASE + I2C_O_SCSR) = I2C_SCSR_DA;
#endif

#ifdef SSI_ENABLE_UPDATE
    //
    // Enable the clocks to the SSI and GPIO modules.
    //
    HWREG(SYSCTL_RCGC2) |= SYSCTL_RCGC2_GPIOA;         //zlz modify use next
    HWREG(SYSCTL_RCGC1) |= SYSCTL_RCGC1_SSI0;
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);         //使能GPIOA
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);          //使能SSI0
		

    //
    // Make the pin be peripheral controlled.
    //
    HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= SSI_PINS;
    HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= SSI_PINS;

    //
    // Set the SSI protocol to Motorola with default clock high and data
    // valid on the rising edge.
    //
    HWREG(SSI0_BASE + SSI_O_CR0) = (SSI_CR0_SPH | SSI_CR0_SPO |
                                    (DATA_BITS_SSI - 1));

    //
    // Enable the SSI interface in slave mode.
    //
    HWREG(SSI0_BASE + SSI_O_CR1) = SSI_CR1_MS | SSI_CR1_SSE;
#endif

    Init_Gpio();        //参数I/O口
    Delay(5000);        //
    Init_Ssi();		      //初始化SSI 控制显示
    Delay(500);         //
    Reset_HD7279();     //复位显示
    Delay(500);         //
    Disp_Boot();        //按方式0译码 显示引导状态
    Disp_Sts=0x80;      //显示状态为显示
    Init_SysTick();     //初始化Systick
    IntPrioritySet(FAULT_SYSTICK,INT_PRIORITY_5);   //系统节拍发生器 中断优先级  5 系统定时器中断
    IntMasterEnable();  //CPU中断允许
		
    // 2010.3.25 张力阵添加 系统中断
    // Setup SysTick.
    // 系统频率 8mHz 系统晶振频率

#ifdef UART_ENABLE_UPDATE
    //
    // Enable the the clocks to the UART and GPIO modules.
    //
    HWREG(SYSCTL_RCGC2) |= SYSCTL_RCGC2_GPIOA;
    HWREG(SYSCTL_RCGC1) |= SYSCTL_RCGC1_UART0;
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);         //使能GPIOA
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);         //使能UART0

    //
    // Keep attempting to sync until we are successful.
    //
#ifdef UART_AUTOBAUD
    while(UARTAutoBaud(&ui32ProcRatio) < 0)
    {
    }
#else
    ui32ProcRatio = UART_BAUD_RATIO(UART_FIXED_BAUDRATE);
#endif

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= UART_PINS;

    //
    // Set the pin type.
    //
    HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= UART_PINS;

    //
    // Set the baud rate.
    //
    HWREG(UART0_BASE + UART_O_IBRD) = ui32ProcRatio >> 6;
    HWREG(UART0_BASE + UART_O_FBRD) = ui32ProcRatio & UART_FBRD_DIVFRAC_M;

    //
    // Set data length, parity, and number of stop bits to 8-N-1.
    //
    HWREG(UART0_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8 | UART_LCRH_FEN;

    //
    // Enable RX, TX, and the UART.
    //
    HWREG(UART0_BASE + UART_O_CTL) = (UART_CTL_UARTEN | UART_CTL_TXE |
                                      UART_CTL_RXE);

#ifdef UART_AUTOBAUD
    //
    // Need to ack in the UART case to hold it up while we get things set up.
    //
    AckPacket();
#endif
#endif
}

//*****************************************************************************
//
//! This function performs the update on the selected port.
//!
//! This function is called directly by the boot loader or it is called as a
//! result of an update request from the application.
//!
//! \return Never returns.
//
//*****************************************************************************
void
Updater(void)
{
    uint32_t ui32Size, ui32Temp, ui32FlashSize;
#ifdef CHECK_CRC
    uint32_t ui32Retcode;
#endif

    //
    // This ensures proper alignment of the global buffer so that the one byte
    // size parameter used by the packetized format is easily skipped for data
    // transfers.
    //
    g_pui8DataBuffer = ((uint8_t *)g_pui32DataBuffer) + 3;

    //
    // Insure that the COMMAND_SEND_DATA cannot be sent to erase the boot
    // loader before the application is erased.
    //
    g_ui32TransferAddress = 0xffffffff;

    Timer_Uart=0;                  //串口定时器清零
    BootLoader_En=0;               //默认禁止引导区更新
	
    //
    // Read any data from the serial port in use.
    //
    while(1)
    {
        //
        // Receive a packet from the port in use.
        //
        ui32Size = sizeof(g_pui32DataBuffer) - 3;
        if(ReceivePacket(g_pui8DataBuffer, &ui32Size) != 0)
        {
            continue;
        }

        //
        // The first byte of the data buffer has the command and determines
        // the format of the rest of the bytes.
        //
        switch(g_pui8DataBuffer[0])
        {
            //
            // This was a simple ping command.
            //
            case COMMAND_PING:
            {   //0x20 ECHO 计算机查询命令 
                //
                // This command always sets the status to COMMAND_RET_SUCCESS.
                //
                g_ui8Status = COMMAND_RET_SUCCESS;

                //
                // Just acknowledge that the command was received.
                //
                AckPacket();

                //
                // Go back and wait for a new command.
                //
                break;
            }

            //
            // This command indicates the start of a download sequence.
            //
            case COMMAND_DOWNLOAD:
            {   //0x21 传输程序偏移地址和程序长度
                //
                // Until determined otherwise, the command status is success.
                //
                g_ui8Status = COMMAND_RET_SUCCESS;

                //
                // A simple do/while(0) control loop to make error exits
                // easier.
                //
                do
                {
                    //
                    // See if a full packet was received.
                    //
                    if(ui32Size != 9)
                    {
                        //
                        // Indicate that an invalid command was received.
                        //
                        g_ui8Status = COMMAND_RET_INVALID_CMD;

                        //
                        // This packet has been handled.
                        //
                        break;
                    }

                    //
                    // Get the address and size from the command.
                    //
                    g_ui32TransferAddress = SwapWord(g_pui32DataBuffer[1]);
                    g_ui32TransferSize = SwapWord(g_pui32DataBuffer[2]);

                    //
                    // Depending upon the build options set, keep a copy of
                    // the original size and start address because we will need
                    // these later.
                    //
#if (defined BL_PROGRESS_FN_HOOK) || (defined CHECK_CRC)
                    g_ui32ImageSize = g_ui32TransferSize;
#endif
#ifdef CHECK_CRC
                    g_ui32ImageAddress = g_ui32TransferAddress;
#endif

                    //
                    // Check for a valid starting address and image size.
                    //
                    if(!BL_FLASH_AD_CHECK_FN_HOOK(g_ui32TransferAddress,          //判断是否合法
                                                  g_ui32TransferSize)) 
                    {
                        //
                        // Set the code to an error to indicate that the last
                        // command failed.  This informs the updater program
                        // that the download command failed.
                        //
                        g_ui8Status = COMMAND_RET_INVALID_ADR;

                        //
                        // This packet has been handled.
                        //
                        break;
                    }


                    //
                    // Only erase the space that we need if we are not
                    // protecting the code, otherwise erase the entire flash.
                    //
#ifdef FLASH_CODE_PROTECTION
                    ui32FlashSize = BL_FLASH_SIZE_FN_HOOK();
#ifdef FLASH_RSVD_SPACE
                    if((ui32FlashSize - FLASH_RSVD_SPACE) !=
                       g_ui32TransferAddress)
                    {
                        ui32FlashSize -= FLASH_RSVD_SPACE;
                    }
#endif
#else
                    ui32FlashSize = g_ui32TransferAddress + g_ui32TransferSize;
#endif
									
                    //
                    // Clear the flash access interrupt.
                    //
                    BL_FLASH_CL_ERR_FN_HOOK();

                    //
                    // Leave the boot loader present until we start getting an
                    // image.
                    //
                    for(ui32Temp = g_ui32TransferAddress;
                        ui32Temp < ui32FlashSize; ui32Temp += FLASH_PAGE_SIZE)
                    {
                        //
                        // Erase this block.
                        //
                        BL_FLASH_ERASE_FN_HOOK(ui32Temp);
                    }

                    //
                    // Return an error if an access violation occurred.
                    //
                    if(BL_FLASH_ERROR_FN_HOOK())
                    {
                        g_ui8Status = COMMAND_RET_FLASH_FAIL;
                    }
                }
                while(0);

                //
                // See if the command was successful.
                //
                if(g_ui8Status != COMMAND_RET_SUCCESS)
                {
                    //
                    // Setting g_ui32TransferSize to zero makes
                    // COMMAND_SEND_DATA fail to accept any data.
                    //
                    g_ui32TransferSize = 0;
                }

                //
                // Acknowledge that this command was received correctly.  This
                // does not indicate success, just that the command was
                // received.
                //
                AckPacket();

                //
                // If we have a start notification hook function, call it
                // now if everything is OK.
                //
#ifdef BL_START_FN_HOOK
                if(g_ui32TransferSize)
                {
                    BL_START_FN_HOOK();
                }
#endif

                //
                // Go back and wait for a new command.
                //
                break;
            }

            //
            // This command indicates that control should be transferred to
            // the specified address.
            //
            case COMMAND_RUN:
            {   //0x22 +4字节 程序地址 程序运行
                //
                // Acknowledge that this command was received correctly.  This
                // does not indicate success, just that the command was
                // received.
                //
                AckPacket();

                //
                // See if a full packet was received.
                //
                if(ui32Size != 5)
                {
                    //
                    // Indicate that an invalid command was received.
                    //
                    g_ui8Status = COMMAND_RET_INVALID_CMD;

                    //
                    // This packet has been handled.
                    //
                    break;
                }

                //
                // Get the address to which control should be transferred.
                //
                g_ui32TransferAddress = SwapWord(g_pui32DataBuffer[1]);

                //
                // This determines the size of the flash available on the
                // device in use.
                //
                ui32FlashSize = BL_FLASH_SIZE_FN_HOOK();

                //
                // Test if the transfer address is valid for this device.
                //
                if(g_ui32TransferAddress >= ui32FlashSize)
                {
                    //
                    // Indicate that an invalid address was specified.
                    //
                    g_ui8Status = COMMAND_RET_INVALID_ADR;

                    //
                    // This packet has been handled.
                    //
                    break;
                }

                //
                // Make sure that the ACK packet has been sent.
                //
                FlushData();

                //
                // Reset and disable the peripherals used by the boot loader.
                //
#ifdef I2C_ENABLE_UPDATE
                HWREG(SYSCTL_RCGC1) &= ~SYSCTL_RCGC1_I2C0;
                HWREG(SYSCTL_SRCR1) = SYSCTL_SRCR1_I2C0;
//								SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C0);   //禁能I2C0
//								SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);     //复位I2C0

#endif
#ifdef UART_ENABLE_UPDATE
                HWREG(SYSCTL_RCGC1) &= ~SYSCTL_RCGC1_UART0;
                HWREG(SYSCTL_SRCR1) = SYSCTL_SRCR1_UART0;
//								SysCtlPeripheralDisable(SYSCTL_PERIPH_UART0);   //禁能UART0
//								SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);     //复位UART0
#endif
#ifdef SSI_ENABLE_UPDATE
                HWREG(SYSCTL_RCGC1) &= ~SYSCTL_RCGC1_SSI0;
                HWREG(SYSCTL_SRCR1) = SYSCTL_SRCR1_SSI0;
//								SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI0);   //禁能SSI0
//								SysCtlPeripheralReset(SYSCTL_PERIPH_SSI0);     //复位SSI0
#endif
                HWREG(SYSCTL_SRCR1) = 0;

                //
                // Branch to the specified address.  This should never return.
                // If it does, very bad things will likely happen since it is
                // likely that the copy of the boot loader in SRAM will have
                // been overwritten.
                //
                ((void (*)(void))g_ui32TransferAddress)();

                //
                // In case this ever does return and the boot loader is still
                // intact, simply reset the device.
                //
                HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
                                     NVIC_APINT_SYSRESETREQ);

                //
                // The microcontroller should have reset, so this should
                // never be reached.  Just in case, loop forever.
                //
                while(1)
                {
                }
            }

            //
            // This command just returns the status of the last command that
            // was sent.
            //
            case COMMAND_GET_STATUS:
            {   //0x23 获取状态
                //
                // Acknowledge that this command was received correctly.  This
                // does not indicate success, just that the command was
                // received.
                //
                AckPacket();

                //
                // Return the status to the updater.
                //
                SendPacket(&g_ui8Status, 1);

                //
                // Go back and wait for a new command.
                //
                break;
            }

            //
            // This command is sent to transfer data to the device following
            // a download command.
            //
            case COMMAND_SEND_DATA:
            {   //0x24 下载数据
                //
                // Until determined otherwise, the command status is success.
                //
                g_ui8Status = COMMAND_RET_SUCCESS;

                //
                // If this is overwriting the boot loader then the application
                // has already been erased so now erase the boot loader.
                //
                if(g_ui32TransferAddress == 0)
                {
                    //
                    // Clear the flash access interrupt.
                    //
                    BL_FLASH_CL_ERR_FN_HOOK();

                    //
                    // Erase the boot loader.
                    //
                    for(ui32Temp = 0; ui32Temp < APP_START_ADDRESS;
                        ui32Temp += FLASH_PAGE_SIZE)
                    {
                        //
                        // Erase this block.
                        //
                        BL_FLASH_ERASE_FN_HOOK(ui32Temp);
                    }

                    //
                    // Return an error if an access violation occurred.
                    //
                    if(BL_FLASH_ERROR_FN_HOOK())
                    {
                        //
                        // Setting g_ui32TransferSize to zero makes
                        // COMMAND_SEND_DATA fail to accept any more data.
                        //
                        g_ui32TransferSize = 0;

                        //
                        // Indicate that the flash erase failed.
                        //
                        g_ui8Status = COMMAND_RET_FLASH_FAIL;
                    }
                }

                //
                // Take one byte off for the command.
                //
                ui32Size = ui32Size - 1;

                //
                // Check if there are any more bytes to receive.
                //
                if(g_ui32TransferSize >= ui32Size)
                {
                    //
                    // If we have been provided with a decryption hook function
                    // call it here.
                    //
#ifdef BL_DECRYPT_FN_HOOK
                    BL_DECRYPT_FN_HOOK(g_pui8DataBuffer + 1, ui32Size);
#endif

                    //
                    // Write this block of data to the flash
                    //
                    BL_FLASH_PROGRAM_FN_HOOK(g_ui32TransferAddress,
                                             (uint8_t *) &g_pui32DataBuffer[1],
                                             ((ui32Size + 3) & ~3));

                    //
                    // Return an error if an access violation occurred.
                    //
                    if(BL_FLASH_ERROR_FN_HOOK())
                    {
                        //
                        // Indicate that the flash programming failed.
                        //
                        g_ui8Status = COMMAND_RET_FLASH_FAIL;
                    }
                    else
                    {
                        //
                        // Now update the address to program.
                        //
                        g_ui32TransferSize -= ui32Size;
                        g_ui32TransferAddress += ui32Size;

                        //
                        // If a progress hook function has been provided, call
                        // it here.
                        //
#ifdef BL_PROGRESS_FN_HOOK
                        BL_PROGRESS_FN_HOOK(g_ui32ImageSize -
                                            g_ui32TransferSize,
                                            g_ui32ImageSize);
#endif

#ifdef CHECK_CRC
                        //
                        // If we've reached the end, check the CRC in the
                        // image to determine whether or not we report an error
                        // back to the host.
                        //
                        if(g_ui32TransferSize == 0)
                        {
                            InitCRC32Table();
                            ui32Retcode = CheckImageCRC32(
                                    (uint32_t *)g_ui32ImageAddress);

                            //
                            // Was the CRC good?  We consider the CRC good if
                            // the header is found and the embedded CRC matches
                            // the calculated value or, if ENFORCE_CRC is not
                            // defined, if the header exists but is unpopulated.
                            //
#ifdef ENFORCE_CRC
                            if(ui32Retcode == CHECK_CRC_OK)
#else
                            if((ui32Retcode == CHECK_CRC_OK) ||
                               (ui32Retcode == CHECK_CRC_NO_LENGTH))
#endif
                            {
                                //
                                // The calculated CRC didn't match the expected
                                // value or the image didn't contain an embedded
                                // CRC.
                                //
                                g_ui8Status = COMMAND_RET_SUCCESS;
                            }
                            else
                            {
                                //
                                // The calculated CRC agreed with the embedded
                                // value.
                                //
                                g_ui8Status = COMMAND_RET_CRC_FAIL;
                            }
                        }
#endif
                    }
                }
                else
                {
                    //
                    // This indicates that too much data is being sent to the
                    // device.
                    //
                    g_ui8Status = COMMAND_RET_INVALID_ADR;
                }

                //
                // Acknowledge that this command was received correctly.  This
                // does not indicate success, just that the command was
                // received.
                //
                Disp_Sts++;
                if(Disp_Sts&0x04)    //每4条指令改变一次显示
                 {
                  if(Disp_Sts&0x80)  //判断当前是否显示
                   {
                    Disp_Sts=0x00;   //清除显示标志
                    Disp_Blank();    //清除显示
                   }
                  else
                   {
                    Disp_Sts=0x80;
                    Disp_Boot();
                   }
                 } 	 	 			
                AckPacket();

                //
                // If we have an end notification hook function, and we've
                // reached the end, call it now.
                //
#ifdef BL_END_FN_HOOK
                if(g_ui32TransferSize == 0)
                {
                    BL_END_FN_HOOK();
                }
#endif

                //
                // Go back and wait for a new command.
                //
                break;
            }

            //
            // This command is used to reset the device.
            //
            case COMMAND_RESET:
            {   //0x25 复位
                //
                // Send out a one-byte ACK to ensure the byte goes back to the
                // host before we reset everything.
                //
                AckPacket();

                //
                // Make sure that the ACK packet has been sent.
                //
                FlushData();

                //
                // Perform a software reset request.  This will cause the
                // microcontroller to reset; no further code will be executed.
                //
                HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
                                     NVIC_APINT_SYSRESETREQ);

                //
                // The microcontroller should have reset, so this should never
                // be reached.  Just in case, loop forever.
                //
                while(1)
                {
                }
            }
           case COMMAND_BOOTID:
            {//设置引导响应ID
             if(ui32Size != 2)
              break;
             BOOT_EN[2]=g_pui8DataBuffer[1];    //更新引导响应ID 
             break;				
            }
           case COMMAND_BOOTLOADER:
            {//引导区更新使能命令
             if(ui32Size != 2)
              break;
             BootLoader_En=g_pui8DataBuffer[1]; //更新引导响应ID  	 
             break;				
            }	

            //
            // Just acknowledge the command and set the error to indicate that
            // a bad command was sent.
            //
            default:
            {
                //
                // Acknowledge that this command was received correctly.  This
                // does not indicate success, just that the command was
                // received.
                //
                AckPacket();

          //
          // Indicate that a bad comand was sent.
          //
          if(Board_Id==BOOT_EN[2])    //只有板号设定的表位回送数据
           g_ui8Status = COMMAND_RET_UNKNOWN_CMD;
         
          //
          // Go back and wait for a new command.
          //
          break;
            }
        }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
