/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.c
;* Author             : 张力阵
;* 显示程序
;* 没有标准电能脉冲   显示 no bEP
;* 没有标准时钟脉冲   显示 no bCP
;* 没有被检表时钟脉冲 显示 no cLP
;* 时钟脉冲没有稳定   显示 no Stb
;* 电压跌落完成       显示 dL   End
;* 时钟频率           显示 Fxxxxxxx
;* 日计时误差         显示 dxxxxxxx
;* 需量周期           显示 Lxxxxxxx
;* 收到合闸脉冲       显示 H  R  H2
;* 收到投切脉冲       显示 5  R  Sd
;* 内置跳闸           OPEN      01
;* 继电器故障         ERR       01
** 
*******************************************************************************/
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "string.h"
#include "stdlib.h"
#include "sysctl.h"
#include "../inc/debug.h"
#include "Disp.h"
#include "define.h"
extern void Delay(unsigned long ulCount);

unsigned char        Disp_Buf[8];                    //显示缓冲区
extern unsigned char Board_Id;
//*****************************************************************************
//
//! Puts a data element into the SSI transmit FIFO.
//!
//! \param ui32Base specifies the SSI module base address.
//! \param ui32Data is the data to be transmitted over the SSI interface.
//!
//! This function places the supplied data into the transmit FIFO of the
//! specified SSI module.  If there is no space available in the transmit FIFO,
//! this function waits until there is space available before returning.
//!
//! \note The upper 32 - N bits of \e ui32Data are discarded by the hardware,
//! where N is the data width as configured by SSIConfigSetExpClk().  For
//! example, if the interface is configured for 8-bit data width, the upper 24
//! bits of \e ui32Data are discarded.
//!
//! \return None.
//
//*****************************************************************************
void
SSIDataPut(uint32_t ui32Base, uint32_t ui32Data)
{
    //
    // Check the arguments.
    //
    ASSERT(_SSIBaseValid(ui32Base));
    ASSERT((ui32Data & (0xfffffffe << (HWREG(ui32Base + SSI_O_CR0) &
                                       SSI_CR0_DSS_M))) == 0);

    //
    // Wait until there is space.
    //
    while(!(HWREG(ui32Base + SSI_O_SR) & SSI_SR_TNF))
    {
    }

    //
    // Write the data to the SSI.
    //
    HWREG(ui32Base + SSI_O_DR) = ui32Data;
}

/*****************************************************************************
* 设置SSI数据长度
*****************************************************************************/
void SSIDataLen(unsigned long SSIx,unsigned char Len)
{
    while(!(HWREG(SSIx + SSI_O_SR) & SSI_SR_TFE))
     { //发送缓冲区不空 等待
     }
    HWREG(SSIx + SSI_O_CR0)  = ((HWREG(SSIx + SSI_O_CR0) & (~SSI_CR0_DSS_M)) | Len); 
}

/*****************************************************************************
* 7279复位 初始化
*****************************************************************************/
void Reset_HD7279(void)
{
    SSIDataLen(SSI0_BASE,SSI_CR0_DSS_8);	     //设置数据长度为8bit
    DISP_RST_EN;
    SysCtlDelay(625);                          //每个数延时160ns  延时100us
    DISP_RST_DN;
    SysCtlDelay(6250);                         //每个数延时160ns  延时1mS
    SSIDataPut(SSI0_BASE,LED_RESET);           //送数据  
    SysCtlDelay(62500);                        //每个数延时160ns  延时10mS
//    Disp_Data(Disp_Code_Mode);               //复位并显示 按方式0译码(与原误差板译码方式相同     
}
/*****************************************************************************
* 方式3译码时,数据转换表
*****************************************************************************/
const unsigned char Disp_Code_Tab[]=
{
  // 0     1    2    3   4     5   6    7     8   9
    0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,
  // A    B     C    D   E     F   g    H    L     R 
    0x77,0x7F,0x4E,0x7E,0x4F,0x47,0x7B,0x37,0x0E,0x77, 
  // -   空格   b    d   U    t    n    o    P    N
    0x01,0x00,0x1F,0x3D,0x3e,0x0F,0x15,0x1d,0x67,0x76,
  // c   8.   h
    0x0d,0xFF,0x17        
};
/*****************************************************************************
* 刷新显示缓冲区
* 刷新数据区
* Disp_Mode 数据刷新方式
* Disp_Buf[0] 对应 最左面数码管(最高位) Disp_Buf[7] 对应 最右面数码管(最低位) 8位数码管
* Disp_Buf[0] 对应 最左面数码管(最高位) Disp_Buf[5] 对应 最右面数码管(最低位) 6位数码管
*****************************************************************************/
void Disp_Data(unsigned char Disp_Mode)
{
    unsigned char m,n;
    unsigned short t;
    SSIDataLen(SSI0_BASE,SSI_CR0_DSS_16);    //设置数据长度为16bit
    n=6;
    for(m=0;m<n;m++)
     {
      t=((Disp_Mode|m)<<8);
      if(Disp_Mode==LED_SEND_DATA_CODE2)
       {
        if((Disp_Buf[m]&0x7f)<sizeof(Disp_Code_Tab))
         t|=Disp_Code_Tab[Disp_Buf[m]&0x7f];
        else
         t|=Disp_Code_Tab[DISP_BLANK];
        t|=(Disp_Buf[m]&0x80);         //判断是否有小数点
       }
      else    
       t|=Disp_Buf[m];
      SSIDataPut(SSI0_BASE,t);
      SysCtlDelay(250);                //延时40uS
     }
}
//显示引导状态
void Disp_Boot(void)
{
    unsigned char m;
    Disp_Buf[0]=DISP_b;                  //负误差
    Disp_Buf[1]=DISP_o;                  //负误差
    Disp_Buf[2]=DISP_o;                  //负误差
    Disp_Buf[3]=DISP_t;                  //负误差
    m=Board_Id;                          //表位号
    Disp_Buf[5]=m%10;                    //表位号最低位
    m/=10;                               
    Disp_Buf[4]=m%10;                    //表位号高位
    Disp_Buf[6]=DISP_BLANK;              //显示空白	
    Disp_Buf[7]=DISP_BLANK;              //显示空白	
    Disp_Data(LED_SEND_DATA_CODE2);      //按方式0译码(与原误差板译码方式相同)	
}
//显示空白状态
void Disp_Blank(void)
{
    unsigned char n;
    for(n=0;n<8;n++)
     {
      Disp_Buf[n]=DISP_BLANK;            //显示空白	
     }	
    Disp_Data(LED_SEND_DATA_CODE2);      //按方式0译码(与原误差板译码方式相同)	
}
