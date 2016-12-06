//*****************************************************************************
//
// bl_can.c - Functions to transfer data via the CAN port.
//
// Copyright (c) 2008-2013 Texas Instruments Incorporated.  All rights reserved.
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
#include "../inc/hw_can.h"
#include "../inc/hw_gpio.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_nvic.h"
#include "../inc/hw_flash.h"
#include "../inc/hw_sysctl.h"
#include "../inc/hw_types.h"
#include "../inc/hw_uart.h"
#include "../inc/hw_ssi.h"
#include "../inc/hw_ints.h"
#include "../User_h/bl_config.h"
#include "../User_h/bl_can.h"
#include "../User_h/bl_can_timing.h"
#include "../User_h/bl_check.h"
#include "../User_h/bl_crystal.h"
#include "../User_h/bl_flash.h"
#include "../User_h/bl_hooks.h"
#include "../User_h/bl_uart.h"
#include "../inc/debug.h"
#include "../inc/sysctl.h"
#include "../driverlib/ssi.h"
#include "../User_h/define.h"
#include "../inc/gpio.h"
#include "../inc/pin_map.h"
#include "../inc/systick.h"
#include "../inc/interrupt.h"
#include "../User_h/disp.h"

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
//! \addtogroup bl_can_api
//! @{
//
//*****************************************************************************
#if defined(CAN_ENABLE_UPDATE) || defined(DOXYGEN)

//*****************************************************************************
//
// The results that can be returned by the CAN APIs.
//
//*****************************************************************************
#define CAN_CMD_SUCCESS         0x00
#define CAN_CMD_FAIL            0x01
#define CAN_BL_SET              0x02           //引导区更新配置
//*****************************************************************************
//
// Macros used to generate correct pin definitions.
//
//*****************************************************************************
#define CAN_RX_PIN_M            (1 << CAN_RX_PIN)
#define CAN_TX_PIN_M            (1 << CAN_TX_PIN)

//*****************************************************************************
//
// Convenience macros for accessing CAN registers.
//
//*****************************************************************************
#define CANRegWrite(ui32Address, ui32Value)                                   \
                                HWREG(ui32Address) = ui32Value

#define CANRegRead(ui32Address)                                               \
                                HWREG(ui32Address)

//*****************************************************************************
//
// The message object number and index to the local message object memory to
// use when accessing the messages.
//
//*****************************************************************************
#define MSG_OBJ_BCAST_RX_ID     2
#define MSG_OBJ_BCAST_TX_ID     5

//*****************************************************************************
//
// A prototype for the function (in the startup code) for calling the
// application.
//
//*****************************************************************************
extern void StartApplication(void);

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);

//*****************************************************************************
//
// Holds the current address to write to when data is received via the Send
// Data Command.
//
//*****************************************************************************
static uint32_t g_ui32TransferAddress;

//*****************************************************************************
//
// Holds the remaining bytes expected to be received.
//
//*****************************************************************************
static uint32_t g_ui32TransferSize;

//*****************************************************************************
//
// The buffer used to receive data from the update.
//
//*****************************************************************************
static uint8_t g_pui8CommandBuffer[8];

//*****************************************************************************
//
// These globals are used to store the first two words to prevent a partial
// image from being booted.
//
//*****************************************************************************
static uint32_t g_ui32StartValues[2];
static uint32_t g_ui32StartSize;
static uint32_t g_ui32StartAddress;

static uint8_t  Program_EN=0;      //编程使能标志

static uint8_t  Can_Rx_ID;         //message object 接收编号
static uint8_t  BLEN=0;            //引导区更新使能标志
static uint8_t  Currnet_Rx_ID=1;   //当前处理消息编号

static uint8_t  CAN_TX_ID=MSG_OBJ_BCAST_RX_ID+1;  //

unsigned long   BOOT_EN[BOOT_EN_SIZE];

unsigned char   Board_Id;
unsigned char   BootLoader_En; //引导区更新使能命令
volatile unsigned int  Timer_1ms;
volatile unsigned char Timer_Uart;
volatile unsigned char Timer_CAN;
unsigned char Disp_Sts;       //显示状态 bit7 显示状态

//*****************************************************************************
//
// The active interface when the UART bridge is enabled.
//
//*****************************************************************************
#ifdef CAN_UART_BRIDGE
static uint32_t g_ui32Interface;
#define IFACE_UNKNOWN           0
#define IFACE_CAN               1
#define IFACE_UART              2
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
    Timer_CAN++; 	
    if((Timer_1ms % 0x100) == 0)                              //200ms
     GPIOPinWrite(GPIOH,TXXZ_MC,~GPIOPinRead(GPIOH,TXXZ_MC)); //复位外部硬件看门狗 ;
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
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOA;//使能端口A时钟	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOB;//使能端口B时钟	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOC;//使能端口C时钟	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOD;//使能端口D时钟	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOE;//使能端口E时钟	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOF;//使能端口F时钟	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOH;//使能端口H时钟	
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能端口A时钟
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能端口B时钟
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能端口C时钟
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能端口D时钟
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能端口E时钟
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能端口F时钟
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//使能端口H时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);//使能端口K时钟
//初始化GPIOA口
    GPIOPinTypeUART(UART0_GPIO,             //端口
                    U0RX|                   //管脚 初始化UART0接口管脚	 PA.0 PA.1
                    U0TX);                  //管脚 初始化UART0接口管脚	 PA.0 PA.1

    GPIOPinConfigure(GPIO_PA0_U0RX);        //第二功能定义
    GPIOPinConfigure(GPIO_PA1_U0TX);
    
    GPIOPinTypeGPIOInput(GDT_MC_GPIO,       //端口
                         GDT_MC);           //管脚PA.6 光电头脉冲输入

    GPIOPinTypeGPIOOutput(GDT_RST_GPIO,     //端口
                          GDT_RST);         //光电头复位

	  GPIOPinTypeGPIOOutput(DISP_RST_GPIO,    //端口
                          DISP_RST);        //管脚 PA.4 HD7219复位输出

    GPIOPinTypeSSI(SPI0_GPIO,               //端口
                   SSICLK|SSIFSS|SSITX);    //管脚 SSICLK SSIFSS SSITX设置为SSI管脚  PA.2 PA.3 PA.5

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

//初始化GPIOB口                             
    GPIOPinTypeCAN(CAN0_GPIO,               //端口
                   CANRX|                   //管脚
                   CANTX);                  //初始化CAN接口管脚
    GPIOPinConfigure(GPIO_PB4_CAN0RX);      //第二功能定义
    GPIOPinConfigure(GPIO_PB5_CAN0TX);      //第二功能定义

    GPIOPinTypeUART(UART1_GPIO,             //端口
                    U1RX|                   //管脚
                    U1TX);                  //初始化UART1接口管脚
    GPIOPinConfigure(GPIO_PB0_U1RX);        //第二功能定义
    GPIOPinConfigure(GPIO_PB1_U1TX);        //第二功能定义

    GPIOPinTypeGPIOOutput(TEST_LAMP_GPIO,   //端口
                          TEST_LAMP);       //校验指示灯
//初始化GPIOC口                             
    GPIOPinTypeGPIOOutput(P3_OR_P4_GPIO,    //端口
                          P3_OR_P4);        //功耗 P3 P4

//初始化GPIOD口                             
    GPIOPinTypeGPIOOutput(UA_JC_GPIO,       //端口
                          UA_JC);	          //管脚 PD.0 A相电压继电器控制
    GPIOPinTypeGPIOOutput(UB_JC_GPIO,       //端口
                          UB_JC);	          //管脚 PD.1 B相电压继电器控制
    GPIOPinTypeGPIOOutput(UC_JC_GPIO,       //端口
                          UC_JC);	          //管脚 PD.2 C相电压继电器控制
    GPIOPinTypeGPIOOutput(UA_ESWC_GPIO,     //端口
                          UA_ESWC);	        //管脚 PD.3 A相电压电子开关控制
    GPIOPinTypeGPIOOutput(CD4094_DIN_GPIO,  //端口
                          CD4094_DIN);      //备用 * PD.4 4094 数据输入
    GPIOPinTypeGPIOOutput(CD4094_STR_GPIO,  //端口
                          CD4094_STR);      //备用 * PD.5 4094 数据输入
//初始化GPIOE口  
    GPIOPinTypeGPIOOutput(HC165_SL_GPIO,    //端口
                          HC165_SL);        //HC165移位/锁存 高电平移位使能 低电平锁存使能

    GPIOPinTypeGPIOOutput(CD4094_CLK_GPIO,  //端口
                          CD4094_CLK);      //CD4094时钟

    GPIOPinTypeGPIOInput(HC165_PIN_GPIO,    //端口
                         HC165_PIN);        //HC165数据输入         

    GPIOPinTypeGPIOOutput(MC_OUT1_GPIO,     //端口
                          MC_OUT1);         //专变终端脉冲输出1
    GPIOPinTypeGPIOOutput(MC_OUT2_GPIO,     //端口
                          MC_OUT2);         //专变终端脉冲输出2
    GPIOPinTypeGPIOInput(KEY_IN_GPIO,       //端口
                         KEY_IN);           //HC165数据输入         
                          
//初始化GPIOF口                             
    GPIOPinTypeTimer(DZ_MC_GPIO,            //端口 PF0
                     DZ_MC);                //管脚 
    GPIOPinTypeTimer(JZ_IN_GPIO,            //端口
                     JZ_IN);                //管脚 PF1
    GPIOPinTypeTimer(SZ_MC_GPIO,            //端口 
                     SZ_MC);                //管脚 PF2
    GPIOPinTypeTimer(GP_BK_GPIO,            //端口
                     GP_BK);                //管脚 PF3 CCP3 被检标准表高频输入
    GPIOPinTypeTimer(FH_IN_GPIO,            //端口
                     FH_IN);                //管脚 PF4
    GPIOPinTypeTimer(PWM_DAC_GPIO,          //端口
                     PWM_DAC);              //管脚 PF5

    GPIOPinTypeGPIOOutput(UB_ESWC_GPIO,     //端口
                          UB_ESWC);         //备用 * PF7 B相电压电子开关控制 三相台定义  UB_ESWC
    GPIOPinTypeGPIOOutput(UC_ESWC_GPIO,     //端口
                          UC_ESWC);         //备用 * PF6 C相电压电子开关控制 三相台定义  UC_ESWC

//初始化GPIOH口                             
    GPIOPinTypeGPIOOutput(GOG_KZ_GPIO,      //端口
                          GOG_KZ);          //管脚 PH0 被检表脉冲共高共低选择控制 
    GPIOPinTypeGPIOOutput(MC_PN_KZ_GPIO,    //端口
                          MC_PN_KZ);        //     PH1 被检表电子脉冲正反向控制
    GPIOPinTypeGPIOOutput(MC_WV_KZ_GPIO,    //端口
                          MC_WV_KZ);        //     PH2 被检表电子脉冲有无功控制

    GPIOPinTypeGPIOInput(XL_MC_GPIO,        //端口
                         XL_MC);            //管脚 PH3 需量周期输入
    GPIOPinTypeGPIOInput(TQ_MC_GPIO,        //端口
                         TQ_MC);            //备用 PH4 时段投切输入
    GPIOPinTypeGPIOInput(HZ_MC_GPIO,        //端口
                         HZ_MC);            //备用 PH5 合闸脉冲
                        
    GPIOPinTypeGPIOOutput(TX_MC_GPIO,        //端口
                         TX_MC);            //管脚 PH6 通信指示
    GPIOPinTypeGPIOOutput(TXXZ_MC_GPIO,      //端口
                         TXXZ_MC);          //管脚 PH7 通信通道切换           
//初始化GPIOK
    GPIOPinTypeGPIOOutput(HC165_PDN_GPIO,   //端口
                          HC165_PDN);       //HC165时钟禁能 高电平禁能 低电平时钟有效 
    GPIOPinTypeGPIOOutput(HC165_CLK_GPIO,   //端口
                          HC165_CLK);       //HC165时钟输入 上升沿数据输出

//初始化端口中断 GPIOA
    GPIOPadConfigSet(GDT_MC_GPIO,           //端口 设置管脚类型    
                     GDT_MC,                //管脚                 
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉            
    GPIOIntTypeSet(GDT_MC_GPIO,             //端口 设置管脚中断发送
                   GDT_MC,                  //管脚                 
                   GPIO_FALLING_EDGE);      //中断方式             
//初始化端口中断 GPIOE
    GPIOPadConfigSet(KEY_IN_GPIO,           //端口 设置管脚类型    
                     KEY_IN,                //管脚             
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(KEY_IN_GPIO,             //端口 设置管脚中断发送
                   KEY_IN,                  //管脚             
                   GPIO_FALLING_EDGE);      //中断方式             
//初始化端口中断 GPIOH
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

    GPIOPadConfigSet(HZ_MC_GPIO,            //端口 设置管脚类型    
                     HZ_MC,                 //管脚 需量输入 时段投切 合闸脉冲
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(HZ_MC_GPIO,              //端口 设置管脚中断方式
                   HZ_MC,                   //管脚 需量输入 时段投切 合闸脉冲
                   GPIO_FALLING_EDGE);      //中断方式             
                                    
//端口初步设置
    CD4094_DIN_L;                           //
    CD4094_CLK_L;                           //4094时钟低电平
    CD4094_STR_L;                           //
    DOWN_JOIN;                              //脉冲输出共E
    POS_Watt_SEL;                           //电子脉冲默认正向有功输入
    TX_MC_OFF;                              //通信指示灯亮
    DISP_RST_EN;                            //显示复位
    GDT_RST_DN;                             //
    TEST_LAMP_OFF;                          //校验指示灯灭
    HC165_DN;                               //HC165时钟禁能
    HC165_CLK_L;                            //HC165时钟低电平
    HC165_SHIFT;                            //HC165移位使能
    WIRE_P4_CTL;
//    RED_485_DN;                             //切换到表第二485通道 默认
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
//! Initializes the CAN controller after reset.
//!
//! After reset, the CAN controller is left in the disabled state.  However,
//! the memory used for message objects contains undefined values and must be
//! cleared prior to enabling the CAN controller the first time.  This prevents
//! unwanted transmission or reception of data before the message objects are
//! configured.  This function must be called before enabling the controller
//! the first time.
//!
//! \return None.
//
//*****************************************************************************
static void
CANInit(void)
{
    int iMsg;

    //
    // Place CAN controller in init state, regardless of previous state.  This
    // will put the controller in idle, and allow the message object RAM to be
    // programmed.
    //
    CANRegWrite(CAN0_BASE + CAN_O_CTL, CAN_CTL_INIT | CAN_CTL_CCE);   //开始初始化

    //
    // Loop through to program all 32 message objects
    //
    for(iMsg = 1; iMsg <= 32; iMsg++)                                 //初始化CAN消息结构图体 共32个
    {
        //
        // Wait for busy bit to clear.
        //
        while(CANRegRead(CAN0_BASE + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY) //查看接口是否忙
        {
        }

        //
        // Clear the message value bit in the arbitration register.  This
        // indicates the message is not valid and is a "safe" condition to
        // leave the message object.
        //
        CANRegWrite(CAN0_BASE + CAN_O_IF1CMSK,                                  //写命令屏蔽寄存器
                    CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_ARB | CAN_IF1CMSK_CONTROL); //WRNRD 读写控制 访问仲裁位 访问控制位
        CANRegWrite(CAN0_BASE + CAN_O_IF1ARB2, 0); 
        CANRegWrite(CAN0_BASE + CAN_O_IF1MCTL, 0);

        //
        // Initiate programming of the message object
        //
        CANRegWrite(CAN0_BASE + CAN_O_IF1CRQ, iMsg);    //访问消息结构体
    }

    //
    // Acknowledge any pending status interrupts.
    //
    CANRegRead(CAN0_BASE + CAN_O_STS);                  //读状态 
}

//*****************************************************************************
//
//! This function configures the message object used to receive commands.
//!
//! This function configures the message object used to receive all firmware
//! update messages.  This will not actually read the data from the message it
//! is used to prepare the message object to receive the data when it is sent.
//!
//! \return None.
//  设置接收消息过滤  只接收 扩展帧ID号为0x1f02xxxx
//*****************************************************************************
static void
CANMessageSetRx(void)
{
    uint8_t m;
    uint16_t ui16CmdMaskReg;
    uint16_t ui16MaskReg[2];
    uint16_t ui16ArbReg[2];
    uint16_t ui16MsgCtrl;
    for(m=1;m<=MSG_OBJ_BCAST_RX_ID;m++)
    {
    //
    // Wait for busy bit to clear
    //
    while(CANRegRead(CAN0_BASE + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY)
    {
    }

    //
    // This is always a write to the Message object as this call is setting a
    // message object.  This call will also always set all size bits so it sets
    // both data bits.  The call will use the CONTROL register to set control
    // bits so this bit needs to be set as well.
    //
    // Set the MASK bit so that this gets transferred to the Message Object.
    // Set the Arb bit so that this gets transferred to the Message object.
    // 设置命令屏蔽寄存器
    ui16CmdMaskReg = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_DATAA |
                      CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL |
                      CAN_IF1CMSK_MASK | CAN_IF1CMSK_ARB);

    //
    // Set the UMASK bit to enable using the mask register.
    // Set the data length since this is set for all transfers.  This is also a
    // single transfer and not a FIFO transfer so set EOB bit.
    // 设置消息控制寄存器 配置接收过滤  单帧消息
    if(m==MSG_OBJ_BCAST_RX_ID)
     ui16MsgCtrl = CAN_IF1MCTL_UMASK | CAN_IF1MCTL_EOB;  
    else
     ui16MsgCtrl = CAN_IF1MCTL_UMASK ;  

    //
    // Configure the Mask Registers.
    //
    //
    // Set the 29 bits of Identifier mask that were requested.
    // 设置29位 ID 屏蔽
    ui16MaskReg[0] = (uint16_t)LM_API_UPD;

    //
    // If the caller wants to filter on the extended ID bit then set it.
    //
    ui16MaskReg[1] =
        (uint16_t)(CAN_IF1MSK2_MXTD | (LM_API_UPD >> 16));   //扩展帧位参与过滤

    //
    // Set the 29 bit version of the Identifier for this message object.
    // Mark the message as valid and set the extended ID bit.
    //
    ui16ArbReg[0] = LM_API_UPD & CAN_IF1ARB1_ID_M;
    ui16ArbReg[1] = (((LM_API_UPD >> 16) & CAN_IF1ARB2_ID_M) |
                     (CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD));   //扩展帧

    //
    // Write out the registers to program the message object.
    //
    CANRegWrite(CAN0_BASE + CAN_O_IF1CMSK, ui16CmdMaskReg);
    CANRegWrite(CAN0_BASE + CAN_O_IF1MSK1, ui16MaskReg[0]);
    CANRegWrite(CAN0_BASE + CAN_O_IF1MSK2, ui16MaskReg[1]);
    CANRegWrite(CAN0_BASE + CAN_O_IF1ARB1, ui16ArbReg[0]);
    CANRegWrite(CAN0_BASE + CAN_O_IF1ARB2, ui16ArbReg[1]);
    CANRegWrite(CAN0_BASE + CAN_O_IF1MCTL, ui16MsgCtrl);
 
    //
    // Transfer the message object to the message object specific by
    // MSG_OBJ_BCAST_RX_ID.
    //
    CANRegWrite(CAN0_BASE + CAN_O_IF1CRQ,
                m & CAN_IF1CRQ_MNUM_M);  //接收object 1   发送object 2MSG_OBJ_BCAST_RX_ID
    } 
}

//*****************************************************************************
//
//! This function reads data from the receive message object.
//!
//! \param pui8Data is a pointer to the buffer to store the data read from the
//! CAN controller.
//! \param pui32MsgID is a pointer to the ID that was received with the data.
//!
//! This function will reads and acknowledges the data read from the message
//! object used to receive all CAN firmware update messages.  It will also
//! return the message identifier as this holds the API number that was
//! attached to the data.  This message identifier should be one of the
//! LM_API_UPD_* definitions.
//!
//! \return The number of valid bytes returned in the \e pui8Data buffer or
//! 0xffffffff if data was overwritten in the buffer.
//
//*****************************************************************************
static uint32_t
CANMessageGetRx(uint8_t *pui8Data, uint32_t *pui32MsgID)
{
    uint16_t ui16CmdMaskReg;
    uint16_t ui16ArbReg0, ui16ArbReg1;
    uint16_t ui16MsgCtrl;
    uint32_t ui32Bytes;
    uint16_t *pui16Data;

    //
    // This is always a read to the Message object as this call is setting a
    // message object.
    // Clear a pending interrupt and new data in a message object.
    //
    ui16CmdMaskReg = (CAN_IF2CMSK_DATAA | CAN_IF2CMSK_DATAB |
                      CAN_IF1CMSK_CONTROL | CAN_IF2CMSK_CLRINTPND |
                      CAN_IF2CMSK_ARB);

    //
    // Set up the request for data from the message object.
    //
    CANRegWrite(CAN0_BASE + CAN_O_IF2CMSK, ui16CmdMaskReg);

    //
    // Transfer the message object to the message object specific by
    // MSG_OBJ_BCAST_RX_ID.
    //
    CANRegWrite(CAN0_BASE + CAN_O_IF2CRQ,
                Can_Rx_ID & CAN_IF1CRQ_MNUM_M);//MSG_OBJ_BCAST_RX_ID

    //
    // Wait for busy bit to clear
    //
    while(CANRegRead(CAN0_BASE + CAN_O_IF2CRQ) & CAN_IF1CRQ_BUSY)
    {
    }

    //
    // Read out the IF Registers.
    //
    ui16ArbReg0 = CANRegRead(CAN0_BASE + CAN_O_IF2ARB1);
    ui16ArbReg1 = CANRegRead(CAN0_BASE + CAN_O_IF2ARB2);
    ui16MsgCtrl = CANRegRead(CAN0_BASE + CAN_O_IF2MCTL);

    //
    // Set the 29 bit version of the Identifier for this message object.
    //
    *pui32MsgID = ((ui16ArbReg1 & CAN_IF1ARB2_ID_M) << 16) | ui16ArbReg0;

    //
    // See if there is new data available.
    //
    if((ui16MsgCtrl & (CAN_IF1MCTL_NEWDAT | CAN_IF1MCTL_MSGLST)) ==
       CAN_IF1MCTL_NEWDAT)
    {
        //
        // Get the amount of data needed to be read.
        //
        ui32Bytes = ui16MsgCtrl & CAN_IF1MCTL_DLC_M;

        //
        // Read out the data from the CAN registers 16 bits at a time.
        //
        pui16Data = (uint16_t *)pui8Data;

        pui16Data[0] = CANRegRead(CAN0_BASE + CAN_O_IF2DA1);
        pui16Data[1] = CANRegRead(CAN0_BASE + CAN_O_IF2DA2);
        pui16Data[2] = CANRegRead(CAN0_BASE + CAN_O_IF2DB1);
        pui16Data[3] = CANRegRead(CAN0_BASE + CAN_O_IF2DB2);

        //
        // Now clear out the new data flag.
        //
        CANRegWrite(CAN0_BASE + CAN_O_IF2CMSK, CAN_IF1CMSK_NEWDAT);

        //
        // Transfer the message object to the message object specific by
        // MSG_OBJ_BCAST_RX_ID.
        //
        CANRegWrite(CAN0_BASE + CAN_O_IF2CRQ, Can_Rx_ID); //MSG_OBJ_BCAST_RX_ID

        //
        // Wait for busy bit to clear
        //
        while(CANRegRead(CAN0_BASE + CAN_O_IF2CRQ) & CAN_IF2CRQ_BUSY)
        {
        }
    }
    else
    {
        //
        // Data was lost so inform the caller.
        //
        ui32Bytes = 0xffffffff;
    }
    return(ui32Bytes);
}

//*****************************************************************************
//
//! This function sends data using the transmit message object.
//!
//! \param ui32Id is the ID to use with this message.
//! \param pui8Data is a pointer to the buffer with the data to be sent.
//! \param ui32Size is the number of bytes to send and should not be more than
//! 8 bytes.
//!
//! This function will reads and acknowledges the data read from the message
//! object used to receive all CAN firmware update messages.  It will also
//! return the message identifier as this holds the API number that was
//! attached to the data.  This message identifier should be one of the
//! LM_API_UPD_* definitions.
//!
//! \return None.
//
//*****************************************************************************
static void
CANMessageSetTx(uint32_t ui32Id, const uint8_t *pui8Data, uint32_t ui32Size)
{
    uint16_t ui16CmdMaskReg;
    uint16_t ui16ArbReg0, ui16ArbReg1;
    uint16_t ui16MsgCtrl;
    uint16_t *pui16Data;

    //
    // Wait for busy bit to clear
    //
    while(CANRegRead(CAN0_BASE + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY)
    {
    }

    //
    // This is always a write to the Message object as this call is setting a
    // message object.  This call will also always set all size bits so it sets
    // both data bits.  The call will use the CONTROL register to set control
    // bits so this bit needs to be set as well.
    //
    ui16CmdMaskReg = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_DATAA |
                      CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL |
                      CAN_IF1CMSK_ARB);

    //
    // Set the 29 bit version of the Identifier for this message object.
    //
    ui16ArbReg0 = ui32Id & CAN_IF1ARB1_ID_M;

    //
    // Mark the message as valid and set the extended ID bit.
    //
    ui16ArbReg1 = (((ui32Id >> 16) & CAN_IF1ARB2_ID_M) |
                   (CAN_IF1ARB2_DIR | CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD));

    //
    // Set the TXRQST bit and the reset the rest of the register.
    // Set the data length since this is set for all transfers.  This is also a
    // single transfer and not a FIFO transfer so set EOB bit.
    //
    //
    ui16MsgCtrl = (CAN_IF1MCTL_TXRQST | CAN_IF1MCTL_EOB |
                   (ui32Size & CAN_IF1MCTL_DLC_M));

    pui16Data = (uint16_t *)pui8Data;

    //
    // Write the data out to the CAN Data registers if needed.
    //
    CANRegWrite(CAN0_BASE + CAN_O_IF1DA1, pui16Data[0]);
    CANRegWrite(CAN0_BASE + CAN_O_IF1DA2, pui16Data[1]);
    CANRegWrite(CAN0_BASE + CAN_O_IF1DB1, pui16Data[2]);
    CANRegWrite(CAN0_BASE + CAN_O_IF1DB2, pui16Data[3]);

    //
    // Write out the registers to program the message object.
    //
    CANRegWrite(CAN0_BASE + CAN_O_IF1CMSK, ui16CmdMaskReg);
    CANRegWrite(CAN0_BASE + CAN_O_IF1ARB1, ui16ArbReg0);
    CANRegWrite(CAN0_BASE + CAN_O_IF1ARB2, ui16ArbReg1);
    CANRegWrite(CAN0_BASE + CAN_O_IF1MCTL, ui16MsgCtrl);

    //
    // Transfer the message object to the message object specifiec by
    // MSG_OBJ_BCAST_TX_ID.
    //
    CANRegWrite(CAN0_BASE + CAN_O_IF1CRQ,
                CAN_TX_ID & CAN_IF1CRQ_MNUM_M);            //请求发送

    CAN_TX_ID++;
    if(CAN_TX_ID>MSG_OBJ_BCAST_TX_ID)
     CAN_TX_ID=MSG_OBJ_BCAST_RX_ID+1;
}

//*****************************************************************************
//
//! Configures the CAN interface.
//!
//! \param ui32SetTiming determines if the CAN bit timing should be configured.
//!
//! This function configures the CAN controller, preparing it for use by
//! the boot loader.   If the \e ui32SetTiming parameter is 0, the bit timing
//! for the CAN bus will be left alone.  This occurs when the boot loader was
//! entered from a running application that already has configured the timing
//! for the system.  When \e ui32SetTiming is non-zero the bit timing will be
//! set to the defaults defined in the <tt>bl_config.h</tt> file in the
//! project.
//!
//! \return None.
//
//*****************************************************************************
static void
ConfigureCANInterface(uint32_t ui32SetTiming)
{
    //
    // Reset the state of all the message object and the state of the CAN
    // module to a known state.
    //
    CANInit();

    //
    // If a device identifier was specified then this was due to an update from
    // a running CAN application so don't change the CAN bit timing.
    //
    if(ui32SetTiming != 0)
    {
        //
        // Set the bit fields of the bit timing register according to the
        // parms.
        //
        CANRegWrite(CAN0_BASE + CAN_O_BIT, CAN_BIT_TIMING);

        //
        // Set the divider upper bits in the extension register.
        //
        CANRegWrite(CAN0_BASE + CAN_O_BRPE, 0);
    }

    //
    // Take the CAN0 device out of INIT state.
    //
    CANRegWrite(CAN0_BASE + CAN_O_CTL, 0);               //结束初始化

    //
    // Configure the broadcast receive message object.
    //
    CANMessageSetRx();                                   //
}

//*****************************************************************************
//
// Reads the next packet that is sent to the boot loader.
//
//*****************************************************************************
static uint32_t
PacketRead(uint8_t *pui8Data, uint32_t *pui32Size)
{
    uint32_t ui32MsgID;
    uint32_t NewData,m;
#ifdef CAN_UART_BRIDGE
    uint32_t ui32Size, ui32Length, ui32Mode, ui32Char;
    uint8_t pui8Buffer[12];

    //
    // Initialize the size and length of the packet.
    //
    ui32Length = 0;
    ui32Size = 0;

    //
    // If no interface has been determined then wait for either CAN or UART
    // data until either responds.
    //
    if(g_ui32Interface == IFACE_UNKNOWN)
    {
        //
        // Wait for CAN or UART data.
        //
        while((CANRegRead(CAN0_BASE + CAN_O_NWDA1) == 0) &&
              ((HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE) == UART_FR_RXFE))
        {
        }

        //
        // If the UART FIFO was empty then the loop exited due to a CAN
        // message.
        //
        if((HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE) == UART_FR_RXFE)
        {
            g_ui32Interface = IFACE_CAN;
        }
        else
        {
            //
            // The UART FIFO was not empty so the UART interface was used.
            //
            g_ui32Interface = IFACE_UART;
        }
    }

    //
    // Read a data packet from the CAN controller.
    //
    if(g_ui32Interface == IFACE_CAN)
    {
#endif
        //
        // Wait until a packet has been received.
        //
        for(;;)
         {
          NewData=CANRegRead(CAN0_BASE + CAN_O_NWDA1);  
          NewData &= (0xffff>>(16-MSG_OBJ_BCAST_RX_ID));
          if(NewData)           //有新数据
           break;
          Currnet_Rx_ID=1;      //
         }
        if((Currnet_Rx_ID==0)||(Currnet_Rx_ID>MSG_OBJ_BCAST_RX_ID)) //当前报文是否非法
         Currnet_Rx_ID=1;
        for(m=1;m<=MSG_OBJ_BCAST_RX_ID;m++)
         {
         	 if(NewData&(0x01<<(Currnet_Rx_ID-1)))
         	  break;
         	 Currnet_Rx_ID++;
           if(Currnet_Rx_ID>MSG_OBJ_BCAST_RX_ID)
            Currnet_Rx_ID=1;	
         }
        Can_Rx_ID=Currnet_Rx_ID;
        Currnet_Rx_ID++;                        //指向下一个报文区
        if(Currnet_Rx_ID>MSG_OBJ_BCAST_RX_ID)
          Currnet_Rx_ID=1;	
        //
        // Read the packet.
        //
        *pui32Size = CANMessageGetRx(pui8Data, &ui32MsgID);
#ifdef CAN_UART_BRIDGE
    }
    else
    {
        //
        // Read a data packet from the UART controller.
        //
        ui32Mode = 0;

        while(1)
        {
            //
            // Wait until a char is available.
            //
            while(HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE)
            {
            }

            //
            // Now get the char.
            //
            ui32Char = HWREG(UART0_BASE + UART_O_DR);

            if(ui32Char == 0xff)
            {
                ui32Mode = 1;
                ui32Length = 0;
            }
            else if(ui32Mode == 1)
            {
                if(ui32Char > 12)
                {
                    ui32Mode = 0;
                }
                else
                {
                    ui32Size = ui32Char;
                    ui32Mode = 2;
                }
            }
            else if(ui32Mode == 3)
            {
                if(ui32Char == 0xfe)
                {
                    pui8Buffer[ui32Length++] = 0xff;
                    ui32Mode = 2;
                }
                else if(ui32Char == 0xfd)
                {
                    pui8Buffer[ui32Length++] = 0xfe;
                    ui32Mode = 2;
                }
                else
                {
                    ui32Mode = 0;
                }
            }
            else if(ui32Mode == 2)
            {
                if(ui32Char == 0xfe)
                {
                    ui32Mode = 3;
                }
                else
                {
                    pui8Buffer[ui32Length++] = ui32Char;
                }
            }

            if((ui32Length == ui32Size) && (ui32Mode == 2))
            {
                ui32MsgID = *(uint32_t *)pui8Buffer;

                if((ui32MsgID & (CAN_MSGID_MFR_M | CAN_MSGID_DTYPE_M)) ==
                   LM_API_UPD)
                {
                    *(uint32_t *)pui8Data =
                        *(uint32_t *)(pui8Buffer + 4);
                    *(uint32_t *)(pui8Data + 4) =
                        *(uint32_t *)(pui8Buffer + 8);
                    *pui32Size = ui32Size - 4;
                    break;
                }
            }
        }
    }
#endif

    //
    // Return the message ID of the packet that was received.
    //
    return(ui32MsgID);
}

//*****************************************************************************
//
// This function writes out an individual character over the UART and
// handles sending out special sequences for handling 0xff and 0xfe values.
//
//*****************************************************************************
#ifdef CAN_UART_BRIDGE
static void
UARTBridgeWrite(uint32_t ui32Char)
{
    //
    // See if the character being sent is 0xff.
    //
    if(ui32Char == 0xff)
    {
        //
        // Send 0xfe 0xfe, the escaped version of 0xff.  A sign extended
        // version of 0xfe is used to avoid the check below for 0xfe, thereby
        // avoiding an infinite loop.  Only the lower 8 bits are actually sent,
        // so 0xfe is what is actually transmitted.
        //
        UARTBridgeWrite(0xfffffffe);
        UARTBridgeWrite(0xfffffffe);
    }

    //
    // Otherwise, see if the character being sent is 0xfe.
    //
    else if(ui32Char == 0xfe)
    {
        //
        // Send 0xfe 0xfd, the escaped version of 0xfe.  A sign extended
        // version of 0xfe is used to avoid the check above for 0xfe, thereby
        // avoiding an infinite loop.  Only the lower 8 bits are actually sent,
        // so 0xfe is what is actually transmitted.
        //
        UARTBridgeWrite(0xfffffffe);
        UARTBridgeWrite(0xfd);
    }

    //
    // Otherwise, simply send this character.
    //
    else
    {
        //
        // Wait until space is available in the UART transmit FIFO.
        //
        while(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF)
        {
        }

        //
        // Send the char.
        //
        HWREG(UART0_BASE + UART_O_DR) = ui32Char & 0xff;
    }
}
#endif

//*****************************************************************************
//
// Sends a packet to the controller that is communicating with the boot loader.
//
//*****************************************************************************
static void
PacketWrite(uint32_t ui32Id, const uint8_t *pui8Data, uint32_t ui32Size)
{
    uint32_t ui32Idx;

    if(Board_Id!=BOOT_EN[2])    //只有板号设定的表位回送数据
     return;

#ifdef CAN_UART_BRIDGE
    //
    // Check if the boot loader is in CAN mode.
    //
    if(g_ui32Interface == IFACE_CAN)
    {
#endif
        //
        // Wait until the previous packet has been sent, providing a time out so
        // that the boot loader does not hang here.
        //
        for(ui32Idx = 1000;
            (ui32Idx != 0) && (CANRegRead(CAN0_BASE + CAN_O_TXRQ1) != 0);
            ui32Idx--)
        {
        }

        //
        // If the previous packet was sent, then send this packet.
        //
        if(ui32Idx != 0)
        {
            CANMessageSetTx(ui32Id, pui8Data, ui32Size);
        }
#ifdef CAN_UART_BRIDGE
    }
    else
    {
        //
        // The boot loader is in UART modes so write the packet using the UART
        // functions.  Write the start pattern followed by the size, and the ID.
        //
        UARTBridgeWrite(0xffffffff);
        UARTBridgeWrite(ui32Size + 4);
        UARTBridgeWrite(ui32Id & 0xff);
        UARTBridgeWrite((ui32Id >> 8) & 0xff);
        UARTBridgeWrite((ui32Id >> 16) & 0xff);
        UARTBridgeWrite((ui32Id >> 24) & 0xff);

        //
        // Now write out the remaining data bytes.
        //
        while(ui32Size--)
        {
            UARTBridgeWrite(*pui8Data++);
        }
    }
#endif
}

//*****************************************************************************
//
//! This is the main routine for handling updating over CAN.
//!
//! This function accepts boot loader commands over CAN to perform a firmware
//! update over the CAN bus.  This function assumes that the CAN bus timing
//! and message objects have been configured elsewhere.
//!
//! \return None.
//
//*****************************************************************************
void
UpdaterCAN(void)
{
    uint32_t ui32Bytes;
    uint32_t ui32Cmd;
    uint32_t ui32FlashSize;
    uint32_t ui32Temp;
    uint8_t ui8Status;

    Delay(CRYSTAL_FREQ / 100);                    //等待10ms

    PacketWrite(LM_API_UPD_REQUEST, 0, 0);        //发送请求帧

    Delay(CRYSTAL_FREQ / 20);                     //等待50ms
/*                                      //张力阵去掉 无用 
#ifdef ENABLE_UPDATE_CHECK
    //
    // Check the application is valid and check the pin to see if an update is
    // being requested.
    //
    if(g_ui32Forced == 1)               //是否为通过强制更新管脚 进入升级程序
    {
        //
        // Send out the CAN request.
        //
#ifdef CAN_UART_BRIDGE
        g_ui32Interface = IFACE_CAN;
#endif
        PacketWrite(LM_API_UPD_REQUEST, 0, 0);        //发送请求帧

        //
        // Send out the UART request.
        //
#ifdef CAN_UART_BRIDGE                                //CAN UART 桥
        g_ui32Interface = IFACE_UART;
        PacketWrite(LM_API_UPD_REQUEST, 0, 0);
        g_ui32Interface = IFACE_UNKNOWN;
#endif

        //
        // Wait only 50ms for the response and move on otherwise.
        //
        Delay(CRYSTAL_FREQ / 20);                     //等待50ms

        //
        // Wait until a packet has been received.
        //
#ifdef CAN_UART_BRIDGE
        if((CANRegRead(CAN0_BASE + CAN_O_NWDA1) == 0) &&
           ((HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE) == UART_FR_RXFE))
#else
        if(CANRegRead(CAN0_BASE + CAN_O_NWDA1) == 0)   //
#endif
        {
            //
            // Call the application.
            //
            StartApplication();                        //启动应用
        }
    }
#endif

    //
    // Loop forever processing packets.
    //
*/
    while(1)
    {
        //
        // Read the next packet.
        //
        ui32Bytes = 0;
        ui32Cmd = PacketRead(g_pui8CommandBuffer, &ui32Bytes);  //接收帧

        //
        // Handle this packet.
        //
        ui8Status = CAN_CMD_SUCCESS;                            //处理帧 成功收到CAN指令
        switch(ui32Cmd)                                         //命令处理 
        {
            //
            // This is an update request packet.
            //
            case LM_API_UPD_REQUEST:                            //请求回应 0x1f020180
            {
                //
                // This packet is ignored (other than generating an ACK).
                //
                break;
            }

            //
            // This is a ping packet.
            //
            case LM_API_UPD_PING:                               //查询帧 0x1f020000
            {
                //
                // 回送 ID号
                //
                break;
            }

            //
            // This is a reset packet.
            //
            case LM_API_UPD_RESET:                              //复位帧 0x1f0200c0
            {
                // 
                // Perform a software reset request.  This will cause the
                // microcontroller to reset; no further code will be executed.
                //
                HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
                                     NVIC_APINT_SYSRESETREQ);   //复位请求

                //
                // The microcontroller should have reset, so this should never
                // be reached.  Just in case, loop forever.
                //
                while(1)
                {
                }
            }

            //
            // This is a data packet.
            //
            case LM_API_UPD_SEND_DATA:                          //数据发送 0x1f020080
            {
                if(!Program_EN)                                 //是否允许编程
                 {
                  ui8Status = CAN_CMD_FAIL;                     //报错误  
                  break;
                 }      
                //
                // If this is overwriting the boot loader then the application
                // has already been erased so now erase the boot loader.
                // 检查是否为 更新bootloader
                if(g_ui32TransferAddress < APP_START_ADDRESS)  //烧写地址在引导程序内 
                 {
#ifndef ENABLE_BL_UPDATE                                       //是否不允许引导程序升级 
                  ui8Status = CAN_CMD_FAIL;
                  break;                                       //报错
#else
                  if(g_ui32TransferAddress == 0)               //起始地址为0 引导区起始地址
                  {
                   if(BLEN=='E')                               //引导区更新使能
                   	{
                      //
                      // Clear the flash access interrupt.
                      // 清除FLASH 访问中断
                      BL_FLASH_CL_ERR_FN_HOOK();
  
                      //
                      // Erase the application before the boot loader.
                      // 擦除 引导程序
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
                      // 是否有擦除错误
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
                          ui8Status = CAN_CMD_FAIL;
                      }
                    }  
                  }
#endif
                }  

                //
                // Check if there are any more bytes to receive.
                // 检查接收包字节长度是否超限
                if(g_ui32TransferSize >= ui32Bytes)      //判断 剩余待编程字节数 大于 收到的字节数 
                {   //接收包数据长度正常
                    //
                    // Decrypt the data if required.
                    //
#ifdef BL_DECRYPT_FN_HOOK
                    BL_DECRYPT_FN_HOOK(g_pui8CommandBuffer, ui32Bytes);
#endif

                    //
                    // Clear the flash access interrupt.
                    // 清除FLASH 访问中断
                    BL_FLASH_CL_ERR_FN_HOOK();

                    //
                    // Skip the first transfer.
                    // 判断是否为第一包数据 最后烧写 防止出错
                    if(g_ui32StartSize == g_ui32TransferSize)
                    {
                        g_ui32StartValues[0] =
                            *((uint32_t *)&g_pui8CommandBuffer[0]);
                        g_ui32StartValues[1] =
                            *((uint32_t *)&g_pui8CommandBuffer[4]);
                    }
                    else
                    {
                        //
                        // Loop over the words to program.
                        // 往FLASH 中烧写程序数据
                        BL_FLASH_PROGRAM_FN_HOOK(g_ui32TransferAddress,  //烧写地址
                                                 g_pui8CommandBuffer,    //烧写数据
                                                 ui32Bytes);             //数据长度
                    }

                    //
                    // Return an error if an access violation occurred.
                    //
                    if(BL_FLASH_ERROR_FN_HOOK())
                    {
                        //
                        // Indicate that the flash programming failed.
                        //
                        ui8Status = CAN_CMD_FAIL;
                    }
                    else
                    {
                        //
                        // Now update the address to program.
                        //
                        g_ui32TransferSize -= ui32Bytes;
                        g_ui32TransferAddress += ui32Bytes;

                        //
                        // If a progress hook function has been provided, call
                        // it here.
                        //
#ifdef BL_PROGRESS_FN_HOOK
                        BL_PROGRESS_FN_HOOK(g_ui32StartSize -
                                            g_ui32TransferSize,
                                            g_ui32StartSize);
#endif
                    }
                }
                else
                {
                    //
                    // This indicates that too much data is being sent to the
                    // device.
                    //
                    ui8Status = CAN_CMD_FAIL;
                }

                //
                // If the last expected bytes were received then write out the
                // first two words of the image to allow it to boot.
                //
                if(g_ui32TransferSize == 0)
                {
                    //
                    // Loop over the words to program.
                    // 最后写应用程序的头两个字(8 bytes)
                    BL_FLASH_PROGRAM_FN_HOOK(g_ui32StartAddress,
                                             (uint8_t *)&g_ui32StartValues,
                                             8);

                    //
                    // If an end signal hook function has been provided, call
                    // it here since we have finished a download.
                    //
                    Program_EN=0;                   //烧写完成清标志 
#ifdef BL_END_FN_HOOK
                    BL_END_FN_HOOK();
#endif
                }
               if( ui8Status == CAN_CMD_FAIL)       //编写过程中出错
                {
                 Program_EN=0;                      //烧写完成清标志
                 g_ui32TransferSize = 0;            // 
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
               break;
            }

            //
            // This is a start download packet.
            //
            case LM_API_UPD_DOWNLOAD:                          //开始下载包 0x1f020040
            {
//                ui8Status = 0;
                //
                // Get the application address and size from the packet data.
                //
                g_ui32TransferAddress = *((uint32_t *)&g_pui8CommandBuffer[0]); //起始地址
                g_ui32TransferSize = *((uint32_t *)&g_pui8CommandBuffer[4]);    //数据长度
                g_ui32StartSize = g_ui32TransferSize;                           //数据长度
                g_ui32StartAddress = g_ui32TransferAddress;                     //起始地址

                //
                // Check for a valid starting address and image size.
                //
                if(!BL_FLASH_AD_CHECK_FN_HOOK(g_ui32TransferAddress,        //BLInternalFlashStartAddrCheck((ui32Addr), (ui32Size)函数)
                                              g_ui32TransferSize))
                {
                    //
                    // Set the code to an error to indicate that the last
                    // command failed.  This informs the updater program
                    // that the download command failed.
                    // 代码错误标志 状态
                    ui8Status = CAN_CMD_FAIL;

                    //
                    // This packet has been handled.
                    //
                    break;
                }

                //
                // Only erase the space that we need if we are not protecting
                // the code, otherwise erase the entire flash.
                //
#ifdef FLASH_CODE_PROTECTION                                   //是否代码保护
                ui32FlashSize = BL_FLASH_SIZE_FN_HOOK();       //读取flash大小 单位byte
#ifdef FLASH_RSVD_SPACE                                        //是否有保留空间
                if((ui32FlashSize - FLASH_RSVD_SPACE) != g_ui32TransferAddress)
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
                BL_FLASH_CL_ERR_FN_HOOK(); //清除flash 访问中断

                //
                // Leave the boot loader present until we start getting an
                // image.
                //
                for(ui32Temp = g_ui32TransferAddress; ui32Temp < ui32FlashSize;
                    ui32Temp += FLASH_PAGE_SIZE)
                {
                    //
                    // Erase this block.
                    // 擦除 块 扇区
                    BL_FLASH_ERASE_FN_HOOK(ui32Temp);
                }

                //
                // Return an error if an access violation occurred.
                // 如果访问错误 返回错误状态
                if(BL_FLASH_ERROR_FN_HOOK())
                {
                    ui8Status = CAN_CMD_FAIL;
                }

                //
                // See if the command was successful.
                // 查看命令是否成功
                if(ui8Status != CAN_CMD_SUCCESS)
                {
                    //
                    // Setting g_ui32TransferSize to zero makes
                    // COMMAND_SEND_DATA fail to accept any data.
                    //
                    g_ui32TransferSize = 0;
                }
#ifdef BL_START_FN_HOOK
                else
                {
                    //
                    // If a start signal hook function has been provided, call
                    // it here since we are about to start a new download.
                    //
                    BL_START_FN_HOOK();
                }
#endif
                if((g_ui32TransferSize!=0)&&(ui8Status != CAN_CMD_FAIL))
                  Program_EN='E';                         //打开编写使能开关

                break;
            }
            case LM_API_UPD_BLEN:
            {
              BLEN=g_pui8CommandBuffer[0];                //引导区是否允许更新标志
              ui8Status = CAN_BL_SET;
              break;
            }
            case LM_API_UPD_BOOTID:                  
            {
							BOOT_EN[2]=g_pui8CommandBuffer[0];
              break;
						}							
            //
            // This is an unknown packet.
            //
            default:                                      //非法指令
            {
							
                //
                // Set the status to indicate a failure.
                //
              if(Board_Id==BOOT_EN[2])      //只有板号设定的表位回送数据
                ui8Status = CAN_CMD_FAIL;
              break;
            }
        }

        //
        // Send an ACK packet in response to indicate that the packet was
        // received.  The status in the ACK data indicates if the command was
        // successfully processed.
        //
        PacketWrite(LM_API_UPD_ACK, &ui8Status, 1);
    }
}

//*****************************************************************************
//
// Configures the UART used for CAN traffic bridging.
//
//*****************************************************************************
#ifdef CAN_UART_BRIDGE
void
ConfigureBridge(void)
{
    //
    // Enable the GPIO module if necessary.
    //
#if (CAN_RX_PERIPH != SYSCTL_RCGC2_GPIOA) && \
    (CAN_TX_PERIPH != SYSCTL_RCGC2_GPIOA)
    HWREG(SYSCTL_RCGC2) |= SYSCTL_RCGC2_GPIOA;
#endif

    //
    // Enable the UART module.
    //
    HWREG(SYSCTL_RCGC1) |= SYSCTL_RCGC1_UART0;

    //
    // Enable the GPIO pins used for the UART.
    //
    HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= 0x3;
    HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= 0x03;

    //
    // Configure the UART.
    //
    HWREG(UART0_BASE + UART_O_IBRD) = UART_BAUD_RATIO(115200) >> 6;
    HWREG(UART0_BASE + UART_O_FBRD) = (UART_BAUD_RATIO(115200) &
                                       UART_FBRD_DIVFRAC_M);
    HWREG(UART0_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    HWREG(UART0_BASE + UART_O_CTL) = (UART_CTL_UARTEN | UART_CTL_TXE |
                                      UART_CTL_RXE);
}
#endif

//*****************************************************************************
//
//! This is the application entry point to the CAN updater.
//!
//! This function should only be entered from a running application and not
//! when running the boot loader with no application present.
//!
//! \return None.
//
//*****************************************************************************
void
AppUpdaterCAN(void)
{
    //
    // If the boot loader is being called from the application the UART needs
    // to be configured.
    //
#ifdef CAN_UART_BRIDGE
    ConfigureBridge();
#endif

    //
    // Configure the CAN controller but don't change the bit timing.
    //
    ConfigureCANInterface(0);  //不初始化 波特率

    //
    // Call the main update routine.
    //
    UpdaterCAN();              //进入更新程序
}

//*****************************************************************************
//
//! Generic configuration is handled in this function.
//!
//! This function is called by the start up code to perform any configuration
//! necessary before calling the update routine.
//!
//! \return None.
// com from Reset_Handler
//*****************************************************************************
void
ConfigureCAN(void)
{
#ifdef CRYSTAL_FREQ
    //
    // Since the crystal frequency was specified, enable the main oscillator
    // and clock the processor from it.
    //
    HWREG(SYSCTL_RCC) &= ~(SYSCTL_RCC_MOSCDIS);

    //
    // Delay while the main oscillator starts up.
    //
    Delay(100000);

    //
    // Set the crystal frequency and switch to the main oscillator.
    //
    HWREG(SYSCTL_RCC) = ((HWREG(SYSCTL_RCC) &
                          ~(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M)) |
                         XTAL_VALUE | SYSCTL_RCC_OSCSRC_MAIN);
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
	
    //
    // Enable the CAN controller.
    //
    HWREG(SYSCTL_RCGC0) |= SYSCTL_RCGC0_CAN0;

#if CAN_RX_PERIPH == CAN_TX_PERIPH
    //
    // Enable the GPIO associated with CAN0
    //
    HWREG(SYSCTL_RCGC2) |= CAN_RX_PERIPH;

    //
    // Wait a while before accessing the peripheral.
    //
    Delay(3);

    //
    // Set the alternate function selects.
    //
    HWREG(CAN_RX_PORT + GPIO_O_AFSEL) |= CAN_RX_PIN_M | CAN_TX_PIN_M;

    //
    // Set the pin type to it's digital function.
    //
    HWREG(CAN_RX_PORT + GPIO_O_DEN) |= CAN_RX_PIN_M | CAN_TX_PIN_M;

    // 张力阵 添加 2013.5.28
    // 选择管脚方向
    HWREG(CAN_RX_PORT + GPIO_O_DIR) &=~CAN_RX_PIN_M;        //RX 管脚为输入
    HWREG(CAN_TX_PORT + GPIO_O_DIR) |= CAN_TX_PIN_M;        //TX 管脚为输出

    // 选择管脚功能

    HWREG(CAN_RX_PORT + GPIO_O_PCTL) &= ~(0x0F<<(4*CAN_RX_PIN));
    HWREG(CAN_RX_PORT + GPIO_O_PCTL) |= (0x08<<(4*CAN_RX_PIN));

    HWREG(CAN_TX_PORT + GPIO_O_PCTL) &= ~(0x0F<<(4*CAN_TX_PIN));
    HWREG(CAN_TX_PORT + GPIO_O_PCTL) |= (0x08<<(4*CAN_TX_PIN));

#else
    //
    // Enable the GPIO associated with CAN0
    //
    HWREG(SYSCTL_RCGC2) |= CAN_RX_PERIPH | CAN_TX_PERIPH;

    //
    // Wait a while before accessing the peripheral.
    //
    Delay(3);

    //
    // Set the alternate function selects.
    //
    HWREG(CAN_RX_PORT + GPIO_O_AFSEL) |= CAN_RX_PIN_M;
    HWREG(CAN_TX_PORT + GPIO_O_AFSEL) |= CAN_TX_PIN_M;

    //
    // Set the pin type to it's digital function.
    //
    HWREG(CAN_RX_PORT + GPIO_O_DEN) |= CAN_RX_PIN_M;
    HWREG(CAN_TX_PORT + GPIO_O_DEN) |= CAN_TX_PIN_M;

    // 张力阵 添加 2013.5.28
    // 选择管脚方向
    HWREG(CAN_RX_PORT + GPIO_O_DIR) &=~CAN_RX_PIN_M;        //RX 管脚为输入
    HWREG(CAN_TX_PORT + GPIO_O_DIR) |= CAN_TX_PIN_M;        //TX 管脚为输出

    // 选择管脚功能

    HWREG(CAN_RX_PORT + GPIO_O_PCTL) &= ~(0x0F<<(4*CAN_RX_PIN));
    HWREG(CAN_RX_PORT + GPIO_O_PCTL) |= (0x08<<(4*CAN_RX_PIN));

    HWREG(CAN_TX_PORT + GPIO_O_PCTL) &= ~(0x0F<<(4*CAN_TX_PIN));
    HWREG(CAN_TX_PORT + GPIO_O_PCTL) |= (0x08<<(4*CAN_TX_PIN));

#endif

    //
    // Configure the UART used for bridging.
    //
#ifdef CAN_UART_BRIDGE
    ConfigureBridge();
#endif

    //
    // Configure the CAN interface.
    //
    ConfigureCANInterface(1); //初始化波特率
		
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
