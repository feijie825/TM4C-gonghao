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

//�ж����ȼ�����
#define INT_PRIORITY_0          0x00//���ȼ�0 ���
#define INT_PRIORITY_1          0x20//���ȼ�1 
#define INT_PRIORITY_2          0x40//���ȼ�2 
#define INT_PRIORITY_3          0x60//���ȼ�3 
#define INT_PRIORITY_4          0x80//���ȼ�4
#define INT_PRIORITY_5          0xa0//���ȼ�5
#define INT_PRIORITY_6          0xc0//���ȼ�6
#define INT_PRIORITY_7          0xe0//���ȼ�7 ���

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
#define CAN_BL_SET              0x02           //��������������
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

static uint8_t  Program_EN=0;      //���ʹ�ܱ�־

static uint8_t  Can_Rx_ID;         //message object ���ձ��
static uint8_t  BLEN=0;            //����������ʹ�ܱ�־
static uint8_t  Currnet_Rx_ID=1;   //��ǰ������Ϣ���

static uint8_t  CAN_TX_ID=MSG_OBJ_BCAST_RX_ID+1;  //

unsigned long   BOOT_EN[BOOT_EN_SIZE];

unsigned char   Board_Id;
unsigned char   BootLoader_En; //����������ʹ������
volatile unsigned int  Timer_1ms;
volatile unsigned char Timer_Uart;
volatile unsigned char Timer_CAN;
unsigned char Disp_Sts;       //��ʾ״̬ bit7 ��ʾ״̬

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
     GPIOPinWrite(GPIOH,TXXZ_MC,~GPIOPinRead(GPIOH,TXXZ_MC)); //��λ�ⲿӲ�����Ź� ;
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
* ��ʼ��GPIO�� 
*****************************************************************************/
void Init_Gpio(void)
{
//�豸ʱ��ʹ��
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOA;//ʹ�ܶ˿�Aʱ��	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOB;//ʹ�ܶ˿�Bʱ��	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOC;//ʹ�ܶ˿�Cʱ��	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOD;//ʹ�ܶ˿�Dʱ��	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOE;//ʹ�ܶ˿�Eʱ��	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOF;//ʹ�ܶ˿�Fʱ��	
    HWREG(SYSCTL_RCGC2)|=SYSCTL_RCGC2_GPIOH;//ʹ�ܶ˿�Hʱ��	
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//ʹ�ܶ˿�Aʱ��
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ�ܶ˿�Bʱ��
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//ʹ�ܶ˿�Cʱ��
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ�ܶ˿�Dʱ��
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ�ܶ˿�Eʱ��
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//ʹ�ܶ˿�Fʱ��
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//ʹ�ܶ˿�Hʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);//ʹ�ܶ˿�Kʱ��
//��ʼ��GPIOA��
    GPIOPinTypeUART(UART0_GPIO,             //�˿�
                    U0RX|                   //�ܽ� ��ʼ��UART0�ӿڹܽ�	 PA.0 PA.1
                    U0TX);                  //�ܽ� ��ʼ��UART0�ӿڹܽ�	 PA.0 PA.1

    GPIOPinConfigure(GPIO_PA0_U0RX);        //�ڶ����ܶ���
    GPIOPinConfigure(GPIO_PA1_U0TX);
    
    GPIOPinTypeGPIOInput(GDT_MC_GPIO,       //�˿�
                         GDT_MC);           //�ܽ�PA.6 ���ͷ��������

    GPIOPinTypeGPIOOutput(GDT_RST_GPIO,     //�˿�
                          GDT_RST);         //���ͷ��λ

	  GPIOPinTypeGPIOOutput(DISP_RST_GPIO,    //�˿�
                          DISP_RST);        //�ܽ� PA.4 HD7219��λ���

    GPIOPinTypeSSI(SPI0_GPIO,               //�˿�
                   SSICLK|SSIFSS|SSITX);    //�ܽ� SSICLK SSIFSS SSITX����ΪSSI�ܽ�  PA.2 PA.3 PA.5

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

//��ʼ��GPIOB��                             
    GPIOPinTypeCAN(CAN0_GPIO,               //�˿�
                   CANRX|                   //�ܽ�
                   CANTX);                  //��ʼ��CAN�ӿڹܽ�
    GPIOPinConfigure(GPIO_PB4_CAN0RX);      //�ڶ����ܶ���
    GPIOPinConfigure(GPIO_PB5_CAN0TX);      //�ڶ����ܶ���

    GPIOPinTypeUART(UART1_GPIO,             //�˿�
                    U1RX|                   //�ܽ�
                    U1TX);                  //��ʼ��UART1�ӿڹܽ�
    GPIOPinConfigure(GPIO_PB0_U1RX);        //�ڶ����ܶ���
    GPIOPinConfigure(GPIO_PB1_U1TX);        //�ڶ����ܶ���

    GPIOPinTypeGPIOOutput(TEST_LAMP_GPIO,   //�˿�
                          TEST_LAMP);       //У��ָʾ��
//��ʼ��GPIOC��                             
    GPIOPinTypeGPIOOutput(P3_OR_P4_GPIO,    //�˿�
                          P3_OR_P4);        //���� P3 P4

//��ʼ��GPIOD��                             
    GPIOPinTypeGPIOOutput(UA_JC_GPIO,       //�˿�
                          UA_JC);	          //�ܽ� PD.0 A���ѹ�̵�������
    GPIOPinTypeGPIOOutput(UB_JC_GPIO,       //�˿�
                          UB_JC);	          //�ܽ� PD.1 B���ѹ�̵�������
    GPIOPinTypeGPIOOutput(UC_JC_GPIO,       //�˿�
                          UC_JC);	          //�ܽ� PD.2 C���ѹ�̵�������
    GPIOPinTypeGPIOOutput(UA_ESWC_GPIO,     //�˿�
                          UA_ESWC);	        //�ܽ� PD.3 A���ѹ���ӿ��ؿ���
    GPIOPinTypeGPIOOutput(CD4094_DIN_GPIO,  //�˿�
                          CD4094_DIN);      //���� * PD.4 4094 ��������
    GPIOPinTypeGPIOOutput(CD4094_STR_GPIO,  //�˿�
                          CD4094_STR);      //���� * PD.5 4094 ��������
//��ʼ��GPIOE��  
    GPIOPinTypeGPIOOutput(HC165_SL_GPIO,    //�˿�
                          HC165_SL);        //HC165��λ/���� �ߵ�ƽ��λʹ�� �͵�ƽ����ʹ��

    GPIOPinTypeGPIOOutput(CD4094_CLK_GPIO,  //�˿�
                          CD4094_CLK);      //CD4094ʱ��

    GPIOPinTypeGPIOInput(HC165_PIN_GPIO,    //�˿�
                         HC165_PIN);        //HC165��������         

    GPIOPinTypeGPIOOutput(MC_OUT1_GPIO,     //�˿�
                          MC_OUT1);         //ר���ն��������1
    GPIOPinTypeGPIOOutput(MC_OUT2_GPIO,     //�˿�
                          MC_OUT2);         //ר���ն��������2
    GPIOPinTypeGPIOInput(KEY_IN_GPIO,       //�˿�
                         KEY_IN);           //HC165��������         
                          
//��ʼ��GPIOF��                             
    GPIOPinTypeTimer(DZ_MC_GPIO,            //�˿� PF0
                     DZ_MC);                //�ܽ� 
    GPIOPinTypeTimer(JZ_IN_GPIO,            //�˿�
                     JZ_IN);                //�ܽ� PF1
    GPIOPinTypeTimer(SZ_MC_GPIO,            //�˿� 
                     SZ_MC);                //�ܽ� PF2
    GPIOPinTypeTimer(GP_BK_GPIO,            //�˿�
                     GP_BK);                //�ܽ� PF3 CCP3 �����׼���Ƶ����
    GPIOPinTypeTimer(FH_IN_GPIO,            //�˿�
                     FH_IN);                //�ܽ� PF4
    GPIOPinTypeTimer(PWM_DAC_GPIO,          //�˿�
                     PWM_DAC);              //�ܽ� PF5

    GPIOPinTypeGPIOOutput(UB_ESWC_GPIO,     //�˿�
                          UB_ESWC);         //���� * PF7 B���ѹ���ӿ��ؿ��� ����̨����  UB_ESWC
    GPIOPinTypeGPIOOutput(UC_ESWC_GPIO,     //�˿�
                          UC_ESWC);         //���� * PF6 C���ѹ���ӿ��ؿ��� ����̨����  UC_ESWC

//��ʼ��GPIOH��                             
    GPIOPinTypeGPIOOutput(GOG_KZ_GPIO,      //�˿�
                          GOG_KZ);          //�ܽ� PH0 ��������干�߹���ѡ����� 
    GPIOPinTypeGPIOOutput(MC_PN_KZ_GPIO,    //�˿�
                          MC_PN_KZ);        //     PH1 ���������������������
    GPIOPinTypeGPIOOutput(MC_WV_KZ_GPIO,    //�˿�
                          MC_WV_KZ);        //     PH2 ���������������޹�����

    GPIOPinTypeGPIOInput(XL_MC_GPIO,        //�˿�
                         XL_MC);            //�ܽ� PH3 ������������
    GPIOPinTypeGPIOInput(TQ_MC_GPIO,        //�˿�
                         TQ_MC);            //���� PH4 ʱ��Ͷ������
    GPIOPinTypeGPIOInput(HZ_MC_GPIO,        //�˿�
                         HZ_MC);            //���� PH5 ��բ����
                        
    GPIOPinTypeGPIOOutput(TX_MC_GPIO,        //�˿�
                         TX_MC);            //�ܽ� PH6 ͨ��ָʾ
    GPIOPinTypeGPIOOutput(TXXZ_MC_GPIO,      //�˿�
                         TXXZ_MC);          //�ܽ� PH7 ͨ��ͨ���л�           
//��ʼ��GPIOK
    GPIOPinTypeGPIOOutput(HC165_PDN_GPIO,   //�˿�
                          HC165_PDN);       //HC165ʱ�ӽ��� �ߵ�ƽ���� �͵�ƽʱ����Ч 
    GPIOPinTypeGPIOOutput(HC165_CLK_GPIO,   //�˿�
                          HC165_CLK);       //HC165ʱ������ �������������

//��ʼ���˿��ж� GPIOA
    GPIOPadConfigSet(GDT_MC_GPIO,           //�˿� ���ùܽ�����    
                     GDT_MC,                //�ܽ�                 
                     GPIO_STRENGTH_8MA,     //��������             
                     GPIO_PIN_TYPE_STD_WPU);//����/����            
    GPIOIntTypeSet(GDT_MC_GPIO,             //�˿� ���ùܽ��жϷ���
                   GDT_MC,                  //�ܽ�                 
                   GPIO_FALLING_EDGE);      //�жϷ�ʽ             
//��ʼ���˿��ж� GPIOE
    GPIOPadConfigSet(KEY_IN_GPIO,           //�˿� ���ùܽ�����    
                     KEY_IN,                //�ܽ�             
                     GPIO_STRENGTH_8MA,     //��������             
                     GPIO_PIN_TYPE_STD_WPU);//����/����               
    GPIOIntTypeSet(KEY_IN_GPIO,             //�˿� ���ùܽ��жϷ���
                   KEY_IN,                  //�ܽ�             
                   GPIO_FALLING_EDGE);      //�жϷ�ʽ             
//��ʼ���˿��ж� GPIOH
    GPIOPadConfigSet(XL_MC_GPIO,            //�˿� ���ùܽ�����    
                     XL_MC,                 //�ܽ� �������� ʱ��Ͷ�� ��բ����
                     GPIO_STRENGTH_8MA,     //��������             
                     GPIO_PIN_TYPE_STD_WPU);//����/����               
    GPIOIntTypeSet(XL_MC_GPIO,              //�˿� ���ùܽ��жϷ�ʽ
                   XL_MC,                   //�ܽ� �������� ʱ��Ͷ�� ��բ����
                   GPIO_FALLING_EDGE);      //�жϷ�ʽ             

    GPIOPadConfigSet(TQ_MC_GPIO,            //�˿� ���ùܽ�����    
                     TQ_MC,                 //�ܽ� �������� ʱ��Ͷ�� ��բ����
                     GPIO_STRENGTH_8MA,     //��������             
                     GPIO_PIN_TYPE_STD_WPU);//����/����               
    GPIOIntTypeSet(TQ_MC_GPIO,              //�˿� ���ùܽ��жϷ�ʽ
                   TQ_MC,                   //�ܽ� �������� ʱ��Ͷ�� ��բ����
                   GPIO_FALLING_EDGE);      //�жϷ�ʽ             

    GPIOPadConfigSet(HZ_MC_GPIO,            //�˿� ���ùܽ�����    
                     HZ_MC,                 //�ܽ� �������� ʱ��Ͷ�� ��բ����
                     GPIO_STRENGTH_8MA,     //��������             
                     GPIO_PIN_TYPE_STD_WPU);//����/����               
    GPIOIntTypeSet(HZ_MC_GPIO,              //�˿� ���ùܽ��жϷ�ʽ
                   HZ_MC,                   //�ܽ� �������� ʱ��Ͷ�� ��բ����
                   GPIO_FALLING_EDGE);      //�жϷ�ʽ             
                                    
//�˿ڳ�������
    CD4094_DIN_L;                           //
    CD4094_CLK_L;                           //4094ʱ�ӵ͵�ƽ
    CD4094_STR_L;                           //
    DOWN_JOIN;                              //���������E
    POS_Watt_SEL;                           //��������Ĭ�������й�����
    TX_MC_OFF;                              //ͨ��ָʾ����
    DISP_RST_EN;                            //��ʾ��λ
    GDT_RST_DN;                             //
    TEST_LAMP_OFF;                          //У��ָʾ����
    HC165_DN;                               //HC165ʱ�ӽ���
    HC165_CLK_L;                            //HC165ʱ�ӵ͵�ƽ
    HC165_SHIFT;                            //HC165��λʹ��
    WIRE_P4_CTL;
//    RED_485_DN;                             //�л�����ڶ�485ͨ�� Ĭ��
    WDI_HIGH;                               //
}

/*****************************************************************************
* ��ʼ��SSI
*****************************************************************************/
void Init_Ssi(void)
{
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);//ʹ��SSI0ʱ��	
    HWREG(SYSCTL_RCGC1)|=SYSCTL_RCGC1_SSI0;	
    SSIConfigSetExpClk(SSI0_BASE,CRYSTAL_FREQ ,\
                            SSI_FRF_MOTO_MODE_0,\
                            SSI_MODE_MASTER,\
                            SSI_BITRATE,\
                            SSI_CR0_DSS_8);    //����SSI CR0
    SSIEnable(SSI0_BASE);                      //ʹ��SSI0
}
/*****************************************************************************
* ��ʼ��ϵͳ���Ķ�ʱ�� 
*****************************************************************************/
void Init_SysTick(void)
{
    SysTickPeriodSet((CRYSTAL_FREQ/1000000)*SYS_TIME); //����ϵͳ���Ķ�ʱ����
    SysTickEnable();			                  //ʹ��ϵͳ���Ķ�ʱ��
    SysTickIntEnable();			                //ʹ��ϵͳ���Ķ�ʱ���ж�
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
    CANRegWrite(CAN0_BASE + CAN_O_CTL, CAN_CTL_INIT | CAN_CTL_CCE);   //��ʼ��ʼ��

    //
    // Loop through to program all 32 message objects
    //
    for(iMsg = 1; iMsg <= 32; iMsg++)                                 //��ʼ��CAN��Ϣ�ṹͼ�� ��32��
    {
        //
        // Wait for busy bit to clear.
        //
        while(CANRegRead(CAN0_BASE + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY) //�鿴�ӿ��Ƿ�æ
        {
        }

        //
        // Clear the message value bit in the arbitration register.  This
        // indicates the message is not valid and is a "safe" condition to
        // leave the message object.
        //
        CANRegWrite(CAN0_BASE + CAN_O_IF1CMSK,                                  //д�������μĴ���
                    CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_ARB | CAN_IF1CMSK_CONTROL); //WRNRD ��д���� �����ٲ�λ ���ʿ���λ
        CANRegWrite(CAN0_BASE + CAN_O_IF1ARB2, 0); 
        CANRegWrite(CAN0_BASE + CAN_O_IF1MCTL, 0);

        //
        // Initiate programming of the message object
        //
        CANRegWrite(CAN0_BASE + CAN_O_IF1CRQ, iMsg);    //������Ϣ�ṹ��
    }

    //
    // Acknowledge any pending status interrupts.
    //
    CANRegRead(CAN0_BASE + CAN_O_STS);                  //��״̬ 
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
//  ���ý�����Ϣ����  ֻ���� ��չ֡ID��Ϊ0x1f02xxxx
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
    // �����������μĴ���
    ui16CmdMaskReg = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_DATAA |
                      CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL |
                      CAN_IF1CMSK_MASK | CAN_IF1CMSK_ARB);

    //
    // Set the UMASK bit to enable using the mask register.
    // Set the data length since this is set for all transfers.  This is also a
    // single transfer and not a FIFO transfer so set EOB bit.
    // ������Ϣ���ƼĴ��� ���ý��չ���  ��֡��Ϣ
    if(m==MSG_OBJ_BCAST_RX_ID)
     ui16MsgCtrl = CAN_IF1MCTL_UMASK | CAN_IF1MCTL_EOB;  
    else
     ui16MsgCtrl = CAN_IF1MCTL_UMASK ;  

    //
    // Configure the Mask Registers.
    //
    //
    // Set the 29 bits of Identifier mask that were requested.
    // ����29λ ID ����
    ui16MaskReg[0] = (uint16_t)LM_API_UPD;

    //
    // If the caller wants to filter on the extended ID bit then set it.
    //
    ui16MaskReg[1] =
        (uint16_t)(CAN_IF1MSK2_MXTD | (LM_API_UPD >> 16));   //��չ֡λ�������

    //
    // Set the 29 bit version of the Identifier for this message object.
    // Mark the message as valid and set the extended ID bit.
    //
    ui16ArbReg[0] = LM_API_UPD & CAN_IF1ARB1_ID_M;
    ui16ArbReg[1] = (((LM_API_UPD >> 16) & CAN_IF1ARB2_ID_M) |
                     (CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD));   //��չ֡

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
                m & CAN_IF1CRQ_MNUM_M);  //����object 1   ����object 2MSG_OBJ_BCAST_RX_ID
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
                CAN_TX_ID & CAN_IF1CRQ_MNUM_M);            //������

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
    CANRegWrite(CAN0_BASE + CAN_O_CTL, 0);               //������ʼ��

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
          if(NewData)           //��������
           break;
          Currnet_Rx_ID=1;      //
         }
        if((Currnet_Rx_ID==0)||(Currnet_Rx_ID>MSG_OBJ_BCAST_RX_ID)) //��ǰ�����Ƿ�Ƿ�
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
        Currnet_Rx_ID++;                        //ָ����һ��������
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

    if(Board_Id!=BOOT_EN[2])    //ֻ�а���趨�ı�λ��������
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

    Delay(CRYSTAL_FREQ / 100);                    //�ȴ�10ms

    PacketWrite(LM_API_UPD_REQUEST, 0, 0);        //��������֡

    Delay(CRYSTAL_FREQ / 20);                     //�ȴ�50ms
/*                                      //������ȥ�� ���� 
#ifdef ENABLE_UPDATE_CHECK
    //
    // Check the application is valid and check the pin to see if an update is
    // being requested.
    //
    if(g_ui32Forced == 1)               //�Ƿ�Ϊͨ��ǿ�Ƹ��¹ܽ� ������������
    {
        //
        // Send out the CAN request.
        //
#ifdef CAN_UART_BRIDGE
        g_ui32Interface = IFACE_CAN;
#endif
        PacketWrite(LM_API_UPD_REQUEST, 0, 0);        //��������֡

        //
        // Send out the UART request.
        //
#ifdef CAN_UART_BRIDGE                                //CAN UART ��
        g_ui32Interface = IFACE_UART;
        PacketWrite(LM_API_UPD_REQUEST, 0, 0);
        g_ui32Interface = IFACE_UNKNOWN;
#endif

        //
        // Wait only 50ms for the response and move on otherwise.
        //
        Delay(CRYSTAL_FREQ / 20);                     //�ȴ�50ms

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
            StartApplication();                        //����Ӧ��
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
        ui32Cmd = PacketRead(g_pui8CommandBuffer, &ui32Bytes);  //����֡

        //
        // Handle this packet.
        //
        ui8Status = CAN_CMD_SUCCESS;                            //����֡ �ɹ��յ�CANָ��
        switch(ui32Cmd)                                         //����� 
        {
            //
            // This is an update request packet.
            //
            case LM_API_UPD_REQUEST:                            //�����Ӧ 0x1f020180
            {
                //
                // This packet is ignored (other than generating an ACK).
                //
                break;
            }

            //
            // This is a ping packet.
            //
            case LM_API_UPD_PING:                               //��ѯ֡ 0x1f020000
            {
                //
                // ���� ID��
                //
                break;
            }

            //
            // This is a reset packet.
            //
            case LM_API_UPD_RESET:                              //��λ֡ 0x1f0200c0
            {
                // 
                // Perform a software reset request.  This will cause the
                // microcontroller to reset; no further code will be executed.
                //
                HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
                                     NVIC_APINT_SYSRESETREQ);   //��λ����

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
            case LM_API_UPD_SEND_DATA:                          //���ݷ��� 0x1f020080
            {
                if(!Program_EN)                                 //�Ƿ�������
                 {
                  ui8Status = CAN_CMD_FAIL;                     //������  
                  break;
                 }      
                //
                // If this is overwriting the boot loader then the application
                // has already been erased so now erase the boot loader.
                // ����Ƿ�Ϊ ����bootloader
                if(g_ui32TransferAddress < APP_START_ADDRESS)  //��д��ַ������������ 
                 {
#ifndef ENABLE_BL_UPDATE                                       //�Ƿ����������������� 
                  ui8Status = CAN_CMD_FAIL;
                  break;                                       //����
#else
                  if(g_ui32TransferAddress == 0)               //��ʼ��ַΪ0 ��������ʼ��ַ
                  {
                   if(BLEN=='E')                               //����������ʹ��
                   	{
                      //
                      // Clear the flash access interrupt.
                      // ���FLASH �����ж�
                      BL_FLASH_CL_ERR_FN_HOOK();
  
                      //
                      // Erase the application before the boot loader.
                      // ���� ��������
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
                      // �Ƿ��в�������
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
                // �����հ��ֽڳ����Ƿ���
                if(g_ui32TransferSize >= ui32Bytes)      //�ж� ʣ�������ֽ��� ���� �յ����ֽ��� 
                {   //���հ����ݳ�������
                    //
                    // Decrypt the data if required.
                    //
#ifdef BL_DECRYPT_FN_HOOK
                    BL_DECRYPT_FN_HOOK(g_pui8CommandBuffer, ui32Bytes);
#endif

                    //
                    // Clear the flash access interrupt.
                    // ���FLASH �����ж�
                    BL_FLASH_CL_ERR_FN_HOOK();

                    //
                    // Skip the first transfer.
                    // �ж��Ƿ�Ϊ��һ������ �����д ��ֹ����
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
                        // ��FLASH ����д��������
                        BL_FLASH_PROGRAM_FN_HOOK(g_ui32TransferAddress,  //��д��ַ
                                                 g_pui8CommandBuffer,    //��д����
                                                 ui32Bytes);             //���ݳ���
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
                    // ���дӦ�ó����ͷ������(8 bytes)
                    BL_FLASH_PROGRAM_FN_HOOK(g_ui32StartAddress,
                                             (uint8_t *)&g_ui32StartValues,
                                             8);

                    //
                    // If an end signal hook function has been provided, call
                    // it here since we have finished a download.
                    //
                    Program_EN=0;                   //��д������־ 
#ifdef BL_END_FN_HOOK
                    BL_END_FN_HOOK();
#endif
                }
               if( ui8Status == CAN_CMD_FAIL)       //��д�����г���
                {
                 Program_EN=0;                      //��д������־
                 g_ui32TransferSize = 0;            // 
                }
                //
                // Acknowledge that this command was received correctly.  This
                // does not indicate success, just that the command was
                // received.
                //
                Disp_Sts++;
                if(Disp_Sts&0x04)    //ÿ4��ָ��ı�һ����ʾ
                 {
                  if(Disp_Sts&0x80)  //�жϵ�ǰ�Ƿ���ʾ
                   {
                    Disp_Sts=0x00;   //�����ʾ��־
                    Disp_Blank();    //�����ʾ
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
            case LM_API_UPD_DOWNLOAD:                          //��ʼ���ذ� 0x1f020040
            {
//                ui8Status = 0;
                //
                // Get the application address and size from the packet data.
                //
                g_ui32TransferAddress = *((uint32_t *)&g_pui8CommandBuffer[0]); //��ʼ��ַ
                g_ui32TransferSize = *((uint32_t *)&g_pui8CommandBuffer[4]);    //���ݳ���
                g_ui32StartSize = g_ui32TransferSize;                           //���ݳ���
                g_ui32StartAddress = g_ui32TransferAddress;                     //��ʼ��ַ

                //
                // Check for a valid starting address and image size.
                //
                if(!BL_FLASH_AD_CHECK_FN_HOOK(g_ui32TransferAddress,        //BLInternalFlashStartAddrCheck((ui32Addr), (ui32Size)����)
                                              g_ui32TransferSize))
                {
                    //
                    // Set the code to an error to indicate that the last
                    // command failed.  This informs the updater program
                    // that the download command failed.
                    // ��������־ ״̬
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
#ifdef FLASH_CODE_PROTECTION                                   //�Ƿ���뱣��
                ui32FlashSize = BL_FLASH_SIZE_FN_HOOK();       //��ȡflash��С ��λbyte
#ifdef FLASH_RSVD_SPACE                                        //�Ƿ��б����ռ�
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
                BL_FLASH_CL_ERR_FN_HOOK(); //���flash �����ж�

                //
                // Leave the boot loader present until we start getting an
                // image.
                //
                for(ui32Temp = g_ui32TransferAddress; ui32Temp < ui32FlashSize;
                    ui32Temp += FLASH_PAGE_SIZE)
                {
                    //
                    // Erase this block.
                    // ���� �� ����
                    BL_FLASH_ERASE_FN_HOOK(ui32Temp);
                }

                //
                // Return an error if an access violation occurred.
                // ������ʴ��� ���ش���״̬
                if(BL_FLASH_ERROR_FN_HOOK())
                {
                    ui8Status = CAN_CMD_FAIL;
                }

                //
                // See if the command was successful.
                // �鿴�����Ƿ�ɹ�
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
                  Program_EN='E';                         //�򿪱�дʹ�ܿ���

                break;
            }
            case LM_API_UPD_BLEN:
            {
              BLEN=g_pui8CommandBuffer[0];                //�������Ƿ�������±�־
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
            default:                                      //�Ƿ�ָ��
            {
							
                //
                // Set the status to indicate a failure.
                //
              if(Board_Id==BOOT_EN[2])      //ֻ�а���趨�ı�λ��������
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
    ConfigureCANInterface(0);  //����ʼ�� ������

    //
    // Call the main update routine.
    //
    UpdaterCAN();              //������³���
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

    Init_Gpio();        //����I/O��
    Delay(5000);        //
    Init_Ssi();		      //��ʼ��SSI ������ʾ
    Delay(500);         //
    Reset_HD7279();     //��λ��ʾ
    Delay(500);         //
    Disp_Boot();        //����ʽ0���� ��ʾ����״̬
    Disp_Sts=0x80;      //��ʾ״̬Ϊ��ʾ
    Init_SysTick();     //��ʼ��Systick
    IntPrioritySet(FAULT_SYSTICK,INT_PRIORITY_5);   //ϵͳ���ķ����� �ж����ȼ�  5 ϵͳ��ʱ���ж�
    IntMasterEnable();  //CPU�ж�����
	
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

    // ������ ��� 2013.5.28
    // ѡ��ܽŷ���
    HWREG(CAN_RX_PORT + GPIO_O_DIR) &=~CAN_RX_PIN_M;        //RX �ܽ�Ϊ����
    HWREG(CAN_TX_PORT + GPIO_O_DIR) |= CAN_TX_PIN_M;        //TX �ܽ�Ϊ���

    // ѡ��ܽŹ���

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

    // ������ ��� 2013.5.28
    // ѡ��ܽŷ���
    HWREG(CAN_RX_PORT + GPIO_O_DIR) &=~CAN_RX_PIN_M;        //RX �ܽ�Ϊ����
    HWREG(CAN_TX_PORT + GPIO_O_DIR) |= CAN_TX_PIN_M;        //TX �ܽ�Ϊ���

    // ѡ��ܽŹ���

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
    ConfigureCANInterface(1); //��ʼ��������
		
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif
