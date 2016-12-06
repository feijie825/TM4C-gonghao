/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : UART.c
;* Author             : ������
;* �������ݴ�������
*******************************************************************************/
#include "Function.h"
#include "string.h"
#include "DISP.h"
//CAN �ӿڵ�ַ����
const u32 UART_PORT[8]=
{
	 UART0_BASE,
	 UART1_BASE,
	 UART3_BASE,
	 UART5_BASE,
	 UART6_BASE,
		UART4_BASE,
 	UART2_BASE,
 	UART7_BASE,
};
extern bool IntMasterDisable(void);
/*****************************************************************************
* ����BOOT �ر��ж�
*****************************************************************************/
void BOOT_INT_DIS(void)
{
    IntMasterDisable();          //����BOOTǰ �����ж�
    IntDisable(FAULT_SYSTICK);   //ϵͳ���ķ�����NVIC�ж�ʹ��  ϵͳ��ʱ���ж�
    IntDisable(INT_UART0);       //UART0         NVIC�ж�ʹ��  UART0�ж�
    IntDisable(INT_UART1);       //UART1         NVIC�ж�ʹ��  UART1�ж�
	   IntDisable(INT_UART3);       //UART3         NVIC�ж�ʹ��  UART3�ж�
	   IntDisable(INT_UART5);       //UART5         NVIC�ж�ʹ��  UART5�ж�
	   IntDisable(INT_UART6);       //UART6         NVIC�ж�ʹ��  UART6�ж�
    IntDisable(INT_TIMER0A);     //TIMER0-A      NVIC�ж�ʹ��  ���ø�Ƶ�ź�1����
    IntDisable(INT_TIMER0B);     //TIMER0-B      NVIC�ж�ʹ��  ���ø�Ƶ�ź�2����
    IntDisable(INT_TIMER1A);     //TIMER1-A      NVIC�ж�ʹ��  ���ø�Ƶ�ź�3����
    IntDisable(INT_TIMER1B);     //TIMER1-B      NVIC�ж�ʹ��  500kHz��׼�������
    IntDisable(INT_TIMER2A);     //TIMER2-A      NVIC�ж�ʹ��  ��׼����Ƶ��������ж� 
    IntDisable(INT_TIMER2B);     //TIMER2-B      NVIC�ж�ʹ��  ��׼���޹���Ƶ��������ж�
	   IntDisable(INT_TIMER3A);     //TIMER3_A      NVIC�ж�ʹ��  �����ʱ�������������ж�
	   IntDisable(INT_TIMER3B);     //TIMER3_B      NVIC�ж�ʹ��  ����������й������������ж�
    IntDisable(INT_TIMER4A);     //TIMER4-A      NVIC�ж�ʹ��  ����������޹������������ж�
    IntDisable(INT_TIMER4B);     //TIMER4-B      NVIC�ж�ʹ��  ����������й������������ж� 
    IntDisable(INT_TIMER5A);     //TIMER5-A      NVIC�ж�ʹ��  ����������޹������������ж� 
   	IntDisable(INT_CAN0);        //CAN           NVIC�ж�ʹ��  CAN �ж�
//    IntDisable(INT_WATCHDOG);    //���Ź�        NVIC�ж�ʹ��
    IntDisable(INT_GPIOA);       //GPIOA         NVIC�ж�ʹ��  GPIOA�ж� ��������
    IntDisable(INT_GPIOM);       //GPIOM         NVIC�ж�ʹ��  GPIOM�ж� ���ͷ�������� �������� ʱ��Ͷ�� ��բ����
    IntDisable(INT_GPION);       //GPION         NVIC�ж�ʹ��  GPION�ж� ���������̵���״̬���������ж�
				IntDisable(INT_ADC0SS0);     //ADC0SSS0      NVIC�ж�ʹ��  ADC0ת���ж�	
//    IntMasterEnable();           //CPU�ж�����
}
/*****************************************************************************
* COM���ջ���������
*****************************************************************************/
void Proc_Com_IBuf(u8 COM)
{
    if(Com_Rx_Sts[COM]!=COM_RX_END)
     return;                                //û������Ҫ����
    if(Com_IHead[COM]<4)                    //����С��4 ��Ϊ�Ǹ���
     {
      Com_IHead[COM]=0;                     //�������ָ��                     
      Com_ITail[COM]=0;                     //������ջ�����βָ�� ����ָ��
      Com_Rx_Sts[COM]=COM_RX_NO;            //����״̬ ��Ϊ����״̬ 
      return;
     }		
    if(Com_IHead[COM]<7)                    //����С��7 ��Ϊ�Ǹ���
     {
      if(COM==MTRCOM)                       //�ж��Ƿ�Ϊ���ýӿ�
       {
        if(Com_IHead[MTRCOM]==6)            //����Ϊ6��Ϊ
         {	
          u32 t1;
          u32 *Ptr;
          Ptr=(u32*)0x20000000;		  
          memcpy(&t1,Com0_IBuf,4);          //�����յ�������
          if(t1==0x544F4F42)                //"BOOT+��������ID"
           {   
            if(Com0_IBuf[5]=='U')												
            {
													 *Ptr     = 'U';                 //λ������ַ����0x20000000 
              *(Ptr+1) = 'U';                 //λ������ַ����0x20000004
												}
												else
												{
													 *Ptr     = 'C';                 //λ������ַ����0x20000000 
              *(Ptr+1) = 'C';                 //λ������ַ����0x20000004													
												}
            *(Ptr+2) = Com0_IBuf[4];          //�������ݵ�ID  
            Com_Rx_Sts[COM]=0;	               //
            BOOT_INT_DIS();                   //BOOT ǰ�ر������ж�
            for(;;)	                          //��ѭ���ȴ����Ź���λ
             {}
           }
         }  		
       }
     }	
    if(CAN_LDATA_TX_STS==SLV_LDATA_TX_REQ)  //�ж��Ƿ�������״̬
     return;         
    if(CAN_LDATA_TX_STS==SLV_LDATA_TX_NO)   //����״̬
     {                                      
      CAN_LMSG_TX_TYPE=COM;                 //����������
      SLV_REQ_TX();                         //��������֡
      return;                               
     }                                      
    else if(CAN_LMSG_TX_TYPE!=COM)          //�ж����ڷ��͵��Ƿ�ΪCOM����
     return;                                //�˳�
    else if(CAN_LDATA_TX_STS==SLV_LDATA_TX_ACK)
     {                                      //����õ���׼ ��������
      memcpy(&CAN_LMSG_TX_Ptr,
             &CAN_LMSG_TX_TAB[CAN_LMSG_TX_TYPE],
             16);                           //�������ʹ����ṹ��
      memcpy(&CAN_LMSG_Tx,
             &CAN_MSG_SET_TAB[SLV_LDATA_TX-1],
             8);                            //�����������ٲ��� MSG
      CAN_LMSG_Tx.ID.BIT.MNUM = Mtr_Numb_ID;//��λ��
      CAN_LDATA_TX_STS = SLV_LDATA_TX_IS;   //���ڷ��ͳ����� ״̬
      *CAN_LMSG_TX_Ptr.TAIL=0;              //���ʹ���ָ��
     }
    CAN_LDATA_TX_Pr();                      //���͵�֡ 
}  
/*****************************************************************************
* COM���ͻ���������
* �˿�:COM ���ڱ��
* ���ڷ��ͺͽ���FIFO��Ϊ16�ֽ�
* ���ڷ��ͻ�����FIFO����Ϊ<=1/8 �ж� �����ͻ������ֽ���������2�ֽ�ʱ ���뷢���ж�
*****************************************************************************/
void Proc_Com_OBuf(u8 COM)
{
    u32 UARTx;
    u8 *Ptr;
    if(COM>=UART_NUMB)
     return;
    if(Com_Tx_Sts[COM]!=COM_TX_EN)    //���COM���ͻ������Ƿ���������
     return;                          //û���������˳�
    UARTx = UART_PORT[COM];  

    switch(COM)				
				{
					  case MTRCOM: Ptr= Com0_OBuf; break;
					  case LCTCOM: Ptr= Com1_OBuf; break;
					  case RTECOM: Ptr= Com3_OBuf; break;
							case ATECOM: Ptr= Com5_OBuf; break;
							case IRECOM: Ptr= Com6_OBuf; break;
							default: break;
				}
    if(UARTBusy(UARTx))               //���UART�Ƿ����ڷ�������
     return;                          //���ͻ���������
    UARTIntDisable(UARTx,UART_INT_TX);//�����η����ж� ���ⷢ��ʱ�����ж�
    if(Com_OHead[COM]<12)             //�жϷ��ͻ������ֽ������Ƿ�С��12
     {                                //С��12�ֽ� �������뷢�ͻ����������ж�
      Com_Tx_Sts[COM] = COM_TX_NO;    //���ͻ�������
      for(Com_OTail[COM]=0;Com_OTail[COM]<Com_OHead[COM];)	//����С��12�ֽ�ֱ�ӷ��ͳ�ȥ ������������������
       UARTCharPut(UARTx,*(Ptr+Com_OTail[COM]++));
     }
    else
     {                                //���ݴ���12�ֽ� ����������������
      Com_Tx_Sts[COM] = COM_TX_IS;    //���ͻ��������ڷ������ݱ�־
      for(Com_OTail[COM]=0;Com_OTail[COM]<12;)
       UARTCharPut(UARTx,*(Ptr+Com_OTail[COM]++));
     }
    UARTIntEnable(UARTx,UART_INT_TX); //����д����ϴ򿪷����ж� �ȴ��ж�
}
/*****************************************************************************
* COM���ճ�ʱ����
* �˿�:COM ���ڱ��
*****************************************************************************/
void Proc_Com_Rx_OvTm(u8 COM)
{
    if(Com_Rx_Sts[COM]!=COM_RX_IS)
     return;                             //���ڲ��ڽ���״̬�˳�
    if((u8)(Timer_8ms-Com_Rx_Time[COM])>Com_Rx_OvTm[COM])//�ж�UART�����Ƿ�ʱ
     {
      Com_Rx_Time[COM]=Timer_8ms;        //�����ʱ��
      if(Com_IHead[COM]!=Com_ITail[COM]) //�ж��Ƿ��յ���Ч����
       {	
        Com_Rx_Sts[COM]=COM_RX_END;      //���ս���״̬ �յ���Ч���ݵȴ�����
       } 
      else
       Com_Rx_Sts[COM]=COM_RX_NO;        //�ڹ涨ʱ����δ�յ�����
     } 	 
}