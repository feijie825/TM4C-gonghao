/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.c
;* Author             : ������
;* ��ʾ����
;* û�б�׼��������   ��ʾ no bEP
;* û�б�׼ʱ������   ��ʾ no bCP
;* û�б����ʱ������ ��ʾ no cLP
;* ʱ������û���ȶ�   ��ʾ no Stb
;* ��ѹ�������       ��ʾ dL   End
;* ʱ��Ƶ��           ��ʾ Fxxxxxxx
;* �ռ�ʱ���         ��ʾ dxxxxxxx
;* ��������           ��ʾ Lxxxxxxx
;* �յ���բ����       ��ʾ H  R  H2
;* �յ�Ͷ������       ��ʾ 5  R  Sd
;* ������բ           OPEN      01
;* �̵�������         ERR       01
** 
*******************************************************************************/
#include <stdint.h>
#include "../inc/hw_types.h"
#include "../inc/hw_ssi.h"
#include "../inc/hw_memmap.h"
#include "../driverlib/ssi.h"
#include "string.h"
#include "stdlib.h"
#include "sysctl.h"
#include "../inc/debug.h"
#include "Disp.h"
#include "define.h"
extern void Delay(unsigned long ulCount);

unsigned char        Disp_Buf[8];                    //��ʾ������
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
* ����SSI���ݳ���
*****************************************************************************/
void SSIDataLen(unsigned long SSIx,unsigned char Len)
{
    while(!(HWREG(SSIx + SSI_O_SR) & SSI_SR_TFE))
     { //���ͻ��������� �ȴ�
     }
    HWREG(SSIx + SSI_O_CR0)  = ((HWREG(SSIx + SSI_O_CR0) & (~SSI_CR0_DSS_M)) | Len); 
}

/*****************************************************************************
* 7279��λ ��ʼ��
*****************************************************************************/
void Reset_HD7279(void)
{
    SSIDataLen(SSI0_BASE,SSI_CR0_DSS_8);	     //�������ݳ���Ϊ8bit
    DISP_RST_EN;
    SysCtlDelay(625);                          //ÿ������ʱ160ns  ��ʱ100us
    DISP_RST_DN;
    SysCtlDelay(6250);                         //ÿ������ʱ160ns  ��ʱ1mS
    SSIDataPut(SSI0_BASE,LED_RESET);           //������  
    SysCtlDelay(62500);                        //ÿ������ʱ160ns  ��ʱ10mS
//    Disp_Data(Disp_Code_Mode);               //��λ����ʾ ����ʽ0����(��ԭ�������뷽ʽ��ͬ     
}
/*****************************************************************************
* ��ʽ3����ʱ,����ת����
*****************************************************************************/
const unsigned char Disp_Code_Tab[]=
{
  // 0     1    2    3   4     5   6    7     8   9
    0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,
  // A    B     C    D   E     F   g    H    L     R 
    0x77,0x7F,0x4E,0x7E,0x4F,0x47,0x7B,0x37,0x0E,0x77, 
  // -   �ո�   b    d   U    t    n    o    P    N
    0x01,0x00,0x1F,0x3D,0x3e,0x0F,0x15,0x1d,0x67,0x76,
  // c   8.   h
    0x0d,0xFF,0x17        
};
/*****************************************************************************
* ˢ����ʾ������
* ˢ��������
* Disp_Mode ����ˢ�·�ʽ
* Disp_Buf[0] ��Ӧ �����������(���λ) Disp_Buf[7] ��Ӧ �����������(���λ) 8λ�����
* Disp_Buf[0] ��Ӧ �����������(���λ) Disp_Buf[5] ��Ӧ �����������(���λ) 6λ�����
*****************************************************************************/
void Disp_Data(unsigned char Disp_Mode)
{
    unsigned char m,n;
    unsigned short t;
    SSIDataLen(SSI0_BASE,SSI_CR0_DSS_16);    //�������ݳ���Ϊ16bit
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
        t|=(Disp_Buf[m]&0x80);         //�ж��Ƿ���С����
       }
      else    
       t|=Disp_Buf[m];
      SSIDataPut(SSI0_BASE,t);
      SysCtlDelay(250);                //��ʱ40uS
     }
}
//��ʾ����״̬
void Disp_Boot(void)
{
    unsigned char m;
    Disp_Buf[0]=DISP_b;                  //�����
    Disp_Buf[1]=DISP_o;                  //�����
    Disp_Buf[2]=DISP_o;                  //�����
    Disp_Buf[3]=DISP_t;                  //�����
    m=Board_Id;                          //��λ��
    Disp_Buf[5]=m%10;                    //��λ�����λ
    m/=10;                               
    Disp_Buf[4]=m%10;                    //��λ�Ÿ�λ
    Disp_Buf[6]=DISP_BLANK;              //��ʾ�հ�	
    Disp_Buf[7]=DISP_BLANK;              //��ʾ�հ�	
    Disp_Data(LED_SEND_DATA_CODE2);      //����ʽ0����(��ԭ�������뷽ʽ��ͬ)	
}
//��ʾ�հ�״̬
void Disp_Blank(void)
{
    unsigned char n;
    for(n=0;n<8;n++)
     {
      Disp_Buf[n]=DISP_BLANK;            //��ʾ�հ�	
     }	
    Disp_Data(LED_SEND_DATA_CODE2);      //����ʽ0����(��ԭ�������뷽ʽ��ͬ)	
}