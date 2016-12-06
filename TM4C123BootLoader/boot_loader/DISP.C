/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Disp.c
;* Author             : ������
;* ��ʾ����
*******************************************************************************/
#include "LM3S2139.h"
#include "LM3S21xx_Lib.h"
#include "LM3S21xx_types.h"
#include "Disp.h"
//*********************ȡ�豸����DCx**************************************
#define SYSCTL_PERIPH_INDEX(a)  (((a) >> 8) & 0xf)

//*********************ȡ�豸����λ*******************************************
#define SYSCTL_PERIPH_MASK(a)   ( 1<< ((a) & 0xff))

extern void Delay(unsigned long ulCount);

u8          Disp_Buf[8];                    //��ʾ������
extern unsigned char Board_Id;

/*****************************************************************************
* �豸ʱ��ʹ���ӳ���
* ���:�豸��
*****************************************************************************/
void SysCtlPeripheralEnable(u32 ulPeripheral)
{
	SYSCTL->RCGC[(SYSCTL_PERIPH_INDEX(ulPeripheral)<3)?\
	              SYSCTL_PERIPH_INDEX(ulPeripheral):2] |=
                 SYSCTL_PERIPH_MASK(ulPeripheral);
}
/*****************************************************************************
* 7279��λ ��ʼ��
*****************************************************************************/
void Reset_HD7279(void)
{
    SSIDataLen(SSI0,SSI_CR0_DSS_8);	      //�������ݳ���Ϊ8bit
    GPIOPinWrite(GPIOA,DISP_RST,0);       //��λ�ܽ���0
    Delay(625);                           //ÿ������ʱ160ns  ��ʱ100us
    GPIOPinWrite(GPIOA,DISP_RST,DISP_RST);//��λ�ܽ���1
    Delay(6250);                          //ÿ������ʱ160ns  ��ʱ1mS
    SSIDataPut(SSI0,LED_RESET);           //������  
    Delay(62500);                         //ÿ������ʱ160ns  ��ʱ10mS
//    Disp_Data(LED_SEND_DATA_CODE2);       //��λ����ʾ ����ʽ0����(��ԭ�������뷽ʽ��ͬ     
}
/*****************************************************************************
* ��ʽ3����ʱ,����ת����
*****************************************************************************/
const u8 Disp_Code_Tab[]=
{
  // 0     1    2    3   4     5   6    7     8   9
    0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,
  // A    B     C    D   E     F   g    H    L     R 
    0x77,0x7F,0x4E,0x7E,0x4F,0x47,0x7B,0x37,0x0E,0x77, 
  // -   �ո�   b    d   U    t    n    o    P    N
    0x01,0x00,0x1F,0x3D,0x3e,0x0F,0x15,0x1d,0x67,0x76      
};
/*****************************************************************************
* ˢ����ʾ������
* ˢ��������
* Disp_Mode ����ˢ�·�ʽ
* Disp_Buf[0] ��Ӧ �����������(���λ) Disp_Buf[7] ��Ӧ �����������(���λ)
*****************************************************************************/
void Disp_Data(u8 Disp_Mode)
{
    u8 m;
    u16 t;
    SSIDataLen(SSI0,SSI_CR0_DSS_16);	  //�������ݳ���Ϊ8bit
    for(m=0;m<8;m++)
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
      SSIDataPut(SSI0,t);
     }
}
//��ʾ����״̬
void Disp_Boot(void)
{
    u8 m;
    Disp_Buf[0]=DISP_b;                  //�����
    Disp_Buf[1]=DISP_o;                  //�����
    Disp_Buf[2]=DISP_o;                  //�����
    Disp_Buf[3]=DISP_t;                  //�����
    Disp_Buf[4]=DISP_BLANK;              //�����
    m=Board_Id;                          //��λ��
    Disp_Buf[7]=m%10;                    //��λ�����λ
    m/=10;                               
    Disp_Buf[6]=m%10;                    //��λ�Ÿ�λ
    m/=10;                               
    if(m!=0)                             
     Disp_Buf[5]=m%10;                   //��λ�����λ
    else                                 
     Disp_Buf[5]=DISP_BLANK;             //��λ�����λ
    Disp_Data(LED_SEND_DATA_CODE2);      //����ʽ0����(��ԭ�������뷽ʽ��ͬ)	
}
//��ʾ�հ�״̬
void Disp_Blank(void)
{
    u8 n;
    for(n=0;n<8;n++)
     {
      Disp_Buf[n]=DISP_BLANK;            //��ʾ�հ�	
     }	
    Disp_Data(LED_SEND_DATA_CODE2);      //����ʽ0����(��ԭ�������뷽ʽ��ͬ)	
}