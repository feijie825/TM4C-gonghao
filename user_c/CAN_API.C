/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CAN.C
;* Author             : ������
;* CAN�������ݴ������� 
;* ������CAN�ӿڵ�LM3S ϵ�д����� ����CANʱ�Ӻ�CPUʱ��֮��Ƶ�ʵĲ�һ��,��ɷ���CAN�Ĵ���ʱ
;* Ҫ���Ӷ�����ʱ CPU ʱ��Ƶ�ʸ���CANʱ��Ƶ��
*******************************************************************************/
#include "Function.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
#include "string.h"
/*****************************************************************************
* CAN�����ݽ��մ����ṹ�����
* CAN_LDT_TYPE_NUMB ���ճ��������͸��� ��define.h�ж��� 
* ���յ��ĳ�����ֱ�Ӵ��뷢�ͻ�����
*****************************************************************************/
const CAN_LMSG_PR CAN_LMSG_RX_TAB[CAN_LDT_TYPE_NUMB]=
{ //���յ��Ǹ������� ����ͷָ�� ����βָ�� ���ջ�����״ָ̬��
    {Com0_OBuf,&Com_OHead[MTRCOM],&Com_OTail[MTRCOM],&Com_Tx_Sts[MTRCOM]},
    {Com1_OBuf,&Com_OHead[LCTCOM],&Com_OTail[LCTCOM],&Com_Tx_Sts[LCTCOM]},
    {Com3_OBuf,&Com_OHead[RTECOM],&Com_OTail[RTECOM],&Com_Tx_Sts[RTECOM]},
    {Com5_OBuf,&Com_OHead[RTECOM],&Com_OTail[RTECOM],&Com_Tx_Sts[ATECOM]},
    {Com6_OBuf,&Com_OHead[IRECOM],&Com_OTail[IRECOM],&Com_Tx_Sts[IRECOM]},
    {Com0_OBuf,&Com_OHead[MTRCOM],&Com_OTail[MTRCOM],&Com_Tx_Sts[MTRCOM]},
    {Com0_OBuf,&Com_OHead[MTRCOM],&Com_OTail[MTRCOM],&Com_Tx_Sts[MTRCOM]},
    {Com0_OBuf,&Com_OHead[MTRCOM],&Com_OTail[MTRCOM],&Com_Tx_Sts[MTRCOM]},
    {Com0_OBuf,&Com_OHead[MTRCOM],&Com_OTail[MTRCOM],&Com_Tx_Sts[MTRCOM]},
    {Com0_OBuf,&Com_OHead[MTRCOM],&Com_OTail[MTRCOM],&Com_Tx_Sts[MTRCOM]},
};
/*****************************************************************************
* CAN�����ݷ��ʹ����ṹ�����
* CAN_LDT_TYPE_NUMB ���������͸��� ��define.h�ж��� 
* ���յ��ĳ�����ֱ�Ӵ��뷢�ͻ�����
*****************************************************************************/
const CAN_LMSG_PR CAN_LMSG_TX_TAB[CAN_LDT_TYPE_NUMB]=
{ //���յ��Ǹ������� ����ͷָ�� ����βָ�� ���ջ�����״ָ̬��
    {Com0_IBuf,&Com_IHead[MTRCOM],&Com_ITail[MTRCOM],&Com_Rx_Sts[MTRCOM]},
    {Com1_IBuf,&Com_IHead[LCTCOM],&Com_ITail[LCTCOM],&Com_Rx_Sts[LCTCOM]},
    {Com3_IBuf,&Com_IHead[RTECOM],&Com_ITail[RTECOM],&Com_Rx_Sts[RTECOM]},
    {Com5_IBuf,&Com_IHead[ATECOM],&Com_ITail[ATECOM],&Com_Rx_Sts[ATECOM]},
    {Com6_IBuf,&Com_IHead[IRECOM],&Com_ITail[IRECOM],&Com_Rx_Sts[IRECOM]},
    {Com0_IBuf,&Com_IHead[MTRCOM],&Com_ITail[MTRCOM],&Com_Rx_Sts[MTRCOM]},
    {Com0_IBuf,&Com_IHead[MTRCOM],&Com_ITail[MTRCOM],&Com_Rx_Sts[MTRCOM]},
    {Com0_IBuf,&Com_IHead[MTRCOM],&Com_ITail[MTRCOM],&Com_Rx_Sts[MTRCOM]},
    {Com0_IBuf,&Com_IHead[MTRCOM],&Com_ITail[MTRCOM],&Com_Rx_Sts[MTRCOM]},
    {Com0_IBuf,&Com_IHead[MTRCOM],&Com_ITail[MTRCOM],&Com_Rx_Sts[MTRCOM]},
};
/****************************************************************************
* CAN������֡����
****************************************************************************/
const CAN_MSG CAN_TX_SMSG=
{   
    0x000D0050,    //������ EXD_ID=1 TX_INT_EN=1 
    0x10010008,    //�ٲ��� EXD=1 DIR=1 END=1
    0x0000,        //DA1
    0x0000,        //DA2
    0x0000,        //DA3
    0x0000,        //DA4
}; 
/****************************************************************************
* CAN������֡���ջ���������
****************************************************************************/
void Proc_LDATA_MSG_IBUF(void)
{
    CAN_MSG  *CAN_RX_MSG;	                            //����CAN����ָ֡��
    if(CAN_LDATA_MSG_ITail==CAN_LDATA_MSG_IHead)
     return;
    CAN_RX_MSG = &CAN_LDATA_MSG_IBUF[CAN_LDATA_MSG_ITail]; //��ʼ�����ձ���ָ��
    switch(CAN_RX_MSG->CTL.BIT.IDx)                   //���ݱ��ı����ת
     {                                                
      case MST_LDATA_ACK:                             //������׼�ӻ����ͳ�������Ӧ
       {                                              //�����ж�			    ID4
        if(CAN_RX_MSG->ID.BIT.TYPE==CAN_LMSG_TX_TYPE) //��׼�����������Ƿ�Ϊ�������������
         CAN_LDATA_TX_STS = SLV_LDATA_TX_ACK;         //��׼�����ݷ���
        break;
       }
      case MST_LCDATA_TX:                             //�������ڷ��ͳ��㲥���ݴ�ID5 
      case MST_LSDATA_TX:                             //�������ڷ��ͳ��������ݴ�ID6 
       {                                              //�����ݽ��մ��� �����ж�
        CAN_LDATA_RX_Pr(CAN_RX_MSG);                  //�����ݽ��մ���
        break;                                        
       }                                              
      case MST_LDATA_REQRT:                           //��������ӻ��ط�����    ID7
       {                                              //�����ж�
        if(CAN_RX_MSG->ID.BIT.TYPE >= CAN_LDT_ACT_NUMB)//�жϳ����������Ƿ����趨��Χ��
         break;                                       //������Χ �˳�
        if((CAN_LDATA_TX_STS!=SLV_LDATA_TX_NO)&&      //�жϸû������Ƿ��ڿ���״̬ ����״̬ 
           (CAN_RX_MSG->ID.BIT.TYPE == CAN_LMSG_TX_TYPE))//�ж����ڷ��͵����ݺ�����������Ƿ�һ��
         {
          CAN_LDATA_TX_STS = SLV_LDATA_TX_ACK;        //��׼�����ݷ���
         }
        else
         {
          CAN_LMSG_TX_TYPE=CAN_RX_MSG->ID.BIT.TYPE;
          memcpy(&CAN_LMSG_TX_Ptr,
                 &CAN_LMSG_TX_TAB[CAN_LMSG_TX_TYPE],
                 16);                                 //�������ʹ����ṹ��
          if(*CAN_LMSG_TX_Ptr.STS == COM_RX_IS)       //�жϸû������Ƿ����ڽ�������
      	   break;                                     //���ڽ����˳�
          CAN_LDATA_TX_STS = SLV_LDATA_TX_ACK;        //��׼�����ݷ���
          *CAN_LMSG_TX_TAB[CAN_RX_MSG->ID.BIT.TYPE].STS = COM_RX_END;//�����ݱ�־ �ȴ�����
          CAN_LMSG_TX_TYPE = CAN_RX_MSG->ID.BIT.TYPE; 
         } 
        break;                      
       }                            
     }
    CAN_LDATA_MSG_ITail++;                              //����ָ���1 ָ����һ��CAN������֡
    if(CAN_LDATA_MSG_ITail>=CAN_LDILEN)                 //�ж��Ƿ񳬳�ѭ��
     CAN_LDATA_MSG_ITail=0;                             //����
    memset(CAN_RX_MSG,0,16);                            //��������ݽ��ջ��������ձ���
}
/*****************************************************************************
* �����ݷ��ʹ���
*****************************************************************************/
void CAN_LDATA_TX_Pr(void)
{
    u16 Len;                                      //���ݳ���
    if(CAN_LMSG_TX_STS != SLV_LDATA_TX_NO)        //�жϵ�֡�����Ƿ����
     return;                                      //�����˳�
    if(CAN_LDATA_TX_STS==SLV_LDATA_TX_LAST)       //���һ֡���ݷ�����ɴ���
     {
      *CAN_LMSG_TX_Ptr.STS  = COM_RX_NO;          //��������������ݱ�־
      *CAN_LMSG_TX_Ptr.TAIL =0;                   //
      CAN_LDATA_TX_STS=SLV_LDATA_TX_NO;           //��������ݷ���״̬      ��Ϊ���Ϳ���״̬
      CAN_LDATA_SERR_Cnt=0;                       //�����ݷ��ʹ������                    
      return;                   
     }
    Len = (*CAN_LMSG_TX_Ptr.HEAD -                
           *CAN_LMSG_TX_Ptr.TAIL);                //ʣ�����ݳ���
    if(Len<9)                                     //ʣ�����ݳ��ȴ���8�ֽ�
     CAN_LMSG_Tx.ID.BIT.END=1;                    //���һ֡
    else
     Len=8;
    CAN_LMSG_Tx.CTL.BIT.LEN = Len;
    memcpy(&CAN_LMSG_Tx.Data,
           CAN_LMSG_TX_Ptr.BUF,
           Len);
    CAN_LMSG_TX_Ptr.BUF += Len;                   //���·���ָ��
    *CAN_LMSG_TX_Ptr.TAIL +=Len;
    CAN_LMSG_TX_STS = SLV_LDATA_TX_IS;            //��֡���ڷ��� �ȴ����뷢������ж����
    CAN_LTX_OVTimer=(u16)Timer_1ms;               //���÷��Ͷ�ʱ
    if(CAN_LMSG_Tx.ID.BIT.END)
     CAN_LDATA_TX_STS = SLV_LDATA_TX_LAST;        //���ڷ������һ֡
    CAN_Tx_Msg(CAN0,
               &CAN_LMSG_Tx,
               MSG_OBJ_TYPE_TX);                  //��������֡
    CAN_LMSG_Tx.ID.BIT.IDX++;                     //��һ֡���
    if(CAN_LMSG_Tx.ID.BIT.IDX==0)                  
     CAN_LMSG_Tx.ID.BIT.IDX=1;                    //��һ֡���ѭ��
}

/*****************************************************************************
* �����ݽ��մ���
* �˿�:*CAN_RX_MSG	���ڴ���CAN����ָ��
* ����ʱ��28us
* ��������ʱ �����������ݿ��ܻᶪ֡ ��������ʱһ��û����
*****************************************************************************/
void CAN_LDATA_RX_Pr(CAN_MSG *CAN_RX_MSG)
{  
    if(CAN_RX_MSG->ID.BIT.TYPE >= CAN_LDT_ACT_NUMB)//�жϳ����������Ƿ����趨��Χ��
     return;                                      //�Ƿ������˳�
    if(CAN_RX_MSG->ID.BIT.IDX ==0)                //�ж��Ƿ�Ϊ��һ֡����
     {                                            //�յ���һ֡�������ý���״̬
      memcpy(&CAN_LMSG_RX_Ptr,
             &CAN_LMSG_RX_TAB[CAN_RX_MSG->ID.BIT.TYPE],
             16);	                                //�������մ����ṹ��
      memcpy(CAN_LMSG_RX_Ptr.BUF,
             &CAN_RX_MSG->Data,
             CAN_RX_MSG->CTL.BIT.LEN);	          //�������ݵ����ջ�����
      CAN_LMSG_RX_TYPE = CAN_RX_MSG->ID.BIT.TYPE; //��ǰ���յ�CAN����������
      CAN_LMSG_RX_Ptr.BUF += CAN_RX_MSG->CTL.BIT.LEN;//ˢ�»�����ָ��
      *CAN_LMSG_RX_Ptr.STS = SLV_LDATA_RX_IS;     //���û�����״̬Ϊ����
      CAN_NEXT_MSG_IDx = 1;                       //��һ�������ݱ�������IDx
     }
    else if(CAN_RX_MSG->ID.BIT.TYPE!=CAN_LMSG_RX_TYPE)
     return;                                     //���Ͳ���ȷ
    else if(*CAN_LMSG_RX_Ptr.STS==SLV_LDATA_RX_NO)
     return;                                     //����״̬�˳� 
    else if(CAN_RX_MSG->ID.BIT.IDX != CAN_NEXT_MSG_IDx) //��������������ȷ
     {                                           //�����ж�ʧ 
      if(*CAN_LMSG_RX_Ptr.STS==SLV_LDATA_RX_IS)  //���ڽ��ճ�����
       {
        *CAN_LMSG_RX_Ptr.STS=0;                  //�����ط���������
        CAN_NEXT_MSG_IDx=0;                      //��һ֡���ı��
        SLV_REQ_RETX();	                         //���ʹӻ������ط�����
       }  
     }
    else
     {
      memcpy(CAN_LMSG_RX_Ptr.BUF,
             &CAN_RX_MSG->Data,
             CAN_RX_MSG->CTL.BIT.LEN);
      CAN_LMSG_RX_Ptr.BUF += CAN_RX_MSG->CTL.BIT.LEN; //ˢ�»�����ָ��
      CAN_NEXT_MSG_IDx++;                        //������һ����������IDx
      if(CAN_NEXT_MSG_IDx>7)
       CAN_NEXT_MSG_IDx =1 ;
     }
    if(CAN_RX_MSG->ID.BIT.END== 1)
     {                                           //�����ݷ��ͽ�������   
      *CAN_LMSG_RX_Ptr.TAIL=0;
      *CAN_LMSG_RX_Ptr.HEAD=(CAN_LMSG_RX_Ptr.BUF-
                             CAN_LMSG_RX_TAB[CAN_RX_MSG->ID.BIT.TYPE].BUF);//ˢ��ָ��
      *CAN_LMSG_RX_Ptr.STS=COM_RX_END;	             //���ͻ������������ݱ�־
     }
}
/*****************************************************************************
* �ӻ�CAN���ݷ��ͳ�ʱ����
*****************************************************************************/
void Proc_CAN_OvTm(void)
{
    if(CAN_LDATA_TX_STS)                             //�����ݷ��ͻ�����״̬
     {
      if((u16)(Timer_1ms-CAN_LTX_OVTimer)>=CAN_TX_OVTM)//�����ݷ��ͳ�ʱ����
       {	 
        CAN_LMSG_TX_STS=COM_TX_NO;                   //���CAN������֡����״̬ ��Ϊ���Ϳ���״̬
        CAN_LDATA_TX_STS=SLV_LDATA_TX_NO;            //��������ݷ���״̬      ��Ϊ���Ϳ���״̬         
        CAN_LDATA_SERR_Cnt++;                        //�����ݷ��ʹ������                    
        if(CAN_LDATA_SERR_Cnt>=CAN_LDATA_SERR_MAX)   //�����ݷ��ʹ���������                    
         {
          CAN_LDATA_SERR_Cnt=0;                      //����������
          *CAN_LMSG_TX_Ptr.STS  = COM_RX_NO;         //��������������ݱ�־
          *CAN_LMSG_TX_Ptr.TAIL =0;                  //�������������ָ��
         }	     
       } 
     }		
    if(CAN_SMSG_TX_STS)                              //������֡�Ƿ��ڷ���״̬
     {
      if((u16)(Timer_1ms-CAN_STX_OVTimer)>=CAN_TX_OVTM)//�����ݷ��ͳ�ʱ����
       {
        CAN_SMSG_TX_STS=COM_TX_NO;                   //��������ݷ��ͱ�־      ��Ϊ���Ϳ���״̬
       }		
     }		
}
/*****************************************************************************
* �ӻ�CAN1���ݷ��ͳ�ʱ����
*****************************************************************************/
void Proc_CAN1_OvTm(void)
{
    if(CAN1_SMSG_TX_STS)                              //������֡�Ƿ��ڷ���״̬
     {
      if((u16)(Timer_1ms-CAN1_STX_OVTimer)>=CAN_TX_OVTM)//�����ݷ��ͳ�ʱ����
       {
        CAN1_SMSG_TX_STS=COM_TX_NO;                   //��������ݷ��ͱ�־      ��Ϊ���Ϳ���״̬
       }		
     }		
}
/*****************************************************************************
* �ӻ�������������֡
* CAN_LMSG_TX_TYPE ������������
* �ӻ��г�����Ҫ����ʱ,�ȷ�������֡ �ȴ�������׼
*****************************************************************************/
void SLV_REQ_TX(void)
{
    CAN_MSG CAN_REQ_TX_MSG;
    memcpy(&CAN_REQ_TX_MSG,
           &CAN_MSG_SET_TAB[SLV_LDATA_REQTX-1],
           8);                                    //�����������ٲ���
    CAN_REQ_TX_MSG.ID.BIT.TYPE = CAN_LMSG_TX_TYPE;//��������
    CAN_REQ_TX_MSG.ID.BIT.MNUM = Mtr_Numb_ID;     //��λ��
    CAN_LDATA_TX_STS=SLV_LDATA_TX_REQ;            //���ó����ݷ���״̬	Ϊ�ѷ�������֡ �ȴ���׼
    CAN_LTX_OVTimer=(u16)Timer_1ms;               //���ö�ʱ��
    CAN_Tx_Msg(CAN0,&CAN_REQ_TX_MSG,MSG_OBJ_TYPE_TX);
}
/*****************************************************************************
* �ӻ������ط�����֡
* CAN_LMSG_RX_TYPE �ط���������
* �ӻ��������ݳ����� Ҫ�������ط�����
*****************************************************************************/
void SLV_REQ_RETX(void)
{
    CAN_MSG CAN_REQ_RETX_MSG;
    memcpy(&CAN_REQ_RETX_MSG,
           &CAN_MSG_SET_TAB[SLV_LDATA_REQRT-1],
           8);                                       //�����������ٲ���
    CAN_REQ_RETX_MSG.ID.BIT.TYPE = CAN_LMSG_RX_TYPE; //���ͳ���������
    CAN_REQ_RETX_MSG.ID.BIT.MNUM = Mtr_Numb_ID;      //��λ��
    CAN_Tx_Msg(CAN0,&CAN_REQ_RETX_MSG,MSG_OBJ_TYPE_TX);
}
/*****************************************************************************
* ����������ѯ��Ϣ
* 2011.8.23 ����������ѯ 
* ����1�ֽ����� ������λ��Ϣ�͹���״̬
*****************************************************************************/
void Proc_Mst_Check(void)
{
	if(Echo_Sts==MST_CHK_RCVD)
	 {                                            //�յ�������ѯ֡
      u8 m;
      CAN_MSG CAN_ECHO_MSG;
      memcpy(&CAN_ECHO_MSG,
             &CAN_MSG_SET_TAB[SLV_CHK_ECHO-1],
             8);                                //�����������ٲ���
      CAN_ECHO_MSG.ID.BIT.MNUM = Mtr_Numb_ID;   //��λ��
      CAN_ECHO_MSG.CTL.BIT.LEN=1;               //���ݳ��� 1���ֽ�
      m=WORK_MODE;
      if(RST_FLAG)                              //������λ��־
       {
        m|=0x10;                                //BIT4 ��λ��־
        RST_FLAG=0;                             //�����λ��־
       }  
      CAN_ECHO_MSG.Data.BYTE[0]=m;
      CAN_Tx_Msg(CAN0,&CAN_ECHO_MSG,MSG_OBJ_TYPE_TX);
      Echo_Sts= SLV_ECHO_SEND;                  //�Ѿ����뷢�� ����MSG RAM
      CAN_RX_OVTimer=(u16)Timer_1ms;            //��ʼ��CAN��ʱ��ʱ��
	 }
}
/*****************************************************************************
* ����CAN�����ݽ���ָ�����
*****************************************************************************/
void Proc_SDATA_IBUF(void)
{
    if(CAN_SDATA_MSG_IHead==CAN_SDATA_MSG_ITail)
     return;                                                     //û���յ��������˳�
    CAN_RX_OVTimer=(u16)Timer_1ms;                               //��ʼ��CAN��ʱ��ʱ��
    MASTER_START=1;                                              //����������
    switch(CAN_MSG_IPtr->ID.BIT.CMD)                             //����������ת����Ӧλ�ô���
     {
//��Ԫԭ������     
      case ERR_ICMD_GD:       Set_STD_CST_Pr()           ;break; //���ձ�׼����Ƶ����   XXXX float ���� 
      case ERR_ICMD_GE:       Set_N_Pr()                 ;break; //����У��Ȧ��
      case ERR_ICMD_GF:       ERR_CLR_Pr()               ;break; //�����������
      case ERR_ICMD_GG:       ELEC_HEAD_RESET_Pr()       ;break; //���չ��ͷ�Թ�
      case ERR_ICMD_GH:       MTR_PLUG_Pr()              ;break; //ѡ��λ �ұ�����
      case ERR_ICMD_GI:       CATCH_HB_Pr()              ;break; //�Ժڰ�         ;0��ȥ�������飬�ص�У��״̬
      case ERR_ICMD_GJ:       Start_Stop_Pr()            ;break; //���ӹ��ͷ���� ;1��/Ǳ��ʱ���� 0��ȥ�������飬�ص�У��״̬
      case ERR_ICMD_GK:       Set_PLS_QUAD()             ;break; //����������������
      case ERR_ICMD_GL:       BEEP_EN_Pr()               ;break; //���ȿ���
      case ERR_ICMD_GM:       Login_Verify_Sts()         ;break; //����У�˳�������״̬,׼����ʼ��������
      case ERR_ICMD_GN:       Verify_Start_Pr()          ;break; //��ʼ����У�˳�����������
      case ERR_ICMD_GO:       Set_Mtr_Cst_Pr()           ;break; //���ձ�У������
      case ERR_ICMD_GP:       PLS_SEL_Pr()               ;break; //����ѡ��  �������廹�ǹ��ͷ����
      case ERR_ICMD_GQ:       Cnt_ENG_Pr()               ;break; //�Ƶ�������      1:��ʼ  0:��ֹ
      case ERR_ICMD_GR:       ENG_CLR_Pr()               ;break; //������ܼ���
      case ERR_ICMD_GS:       PZ_ERR_Pr()                ;break; //����Ƚ�  ��������͹��ͷ����Ƚ�
      case ERR_ICMD_GT:       Set_Pzbz_Pr()              ;break; //�����������ת��ֵ
      case ERR_ICMD_GU:       Set_Pzzs_Pr()              ;break; //�����������ת�趨�Ƚ�Ȧ��
      case ERR_ICMD_GV:       Login_Sy_Sts()             ;break; //����ʧѹ״̬
      case ERR_ICMD_GW:       Start_Sy_Pr()              ;break; //ʧѹ���鿪ʼ
      case ERR_ICMD_GX:       CYCLE_PLS_SEL_Pr()         ;break; //ѡ����Ҫ�����������ں�Ƶ�ʵ�����
      case ERR_ICMD_GY:       DIVIDE_COEF_Pr()           ;break; //�ͷ�Ƶϵ��
      case ERR_ICMD_GZ:       MEA_MTR_CST_Pr()           ;break; //�ƶ����巨������������
      case ERR_ICMD_Gd:       Power_Test_Pr()            ;break; //���Ĳ��� ��λ�����ѡ��
      case ERR_ICMD_Ge:                                   break; //���Ĳ����� ��ѹ��Ȧ�й�����ֵ
      case ERR_ICMD_Gf:                                   break; //��ѹ��Ȧ���ڹ�������ֵ
      case ERR_ICMD_Gg:                                   break; //���յ�����Ȧ���ڹ�������ֵ
      case ERR_ICMD_Gh:                                   break; //���ù��Ĳ�����Ԫ��Ԫ��
      case ERR_ICMD_Gi:       MEA_PLS_CYCLE_Pr()         ;break; //���岨�β���
      case ERR_ICMD_Gj:       WIRE_TYPE_Pr()             ;break; //���ս��߷�ʽ
      case ERR_ICMD_Gk:       ELEC_PLS_TYPE_Pr()         ;break; //��У�������������ѡ�� ���߹���
      case ERR_ICMD_Gl:       Set_ZZ_PLS_Pr()            ;break; //��������������
      case ERR_ICMD_Gm:       PLS_ZZ_Pr()                ;break; //������������
      case ERR_ICMD_Gn:       ReSet_ZZ_PLS_Pr()          ;break; //Ԥ��������
//��Ԫ��������(�๦��������ѹ̨ͨѶ����)             
      case ERR_ICMD_Go:       Start_Ny_Pr()              ;break; //��ʼ��ѹ����
      case ERR_ICMD_Gp:       Ny_Time_End_Pr()           ;break; //��ѹʱ�䵽 ����
      case ERR_ICMD_Gq:       Chg_U_In_Pr()              ;break; //�л���β��ѹ����
      case ERR_ICMD_Gr:       STS_Light_Pr()             ;break; //���У��ָʾ�ƿ�������
//      case ERR_ICMD_Gs:       Relay_Close_Pr()           ;break; //�̵�������        
//      case ERR_ICMD_Gt:       Relay_Open_Pr()            ;break; //�̵����Ͽ�
//      case ERR_ICMD_Gu:       I_Open_Tst_Pr()            ;break; //������·��·���
//��Ԫ���������                                   
      case ERR_ICMD_SYPH:     Set_Sy_Ph()                ;break; //����ʧѹ���
      case ERR_ICMD_MIN_CST:  Set_Min_CST_Pr()           ;break; //������С�������
      case ERR_ICMD_ZZDS:     Set_ZZ_Ds()                ;break; //����У�˳������ֶ���      
      case ERR_ICMD_MCHC:     Set_PLSHC();               ;break; //��������ϳɷ�ʽ
      case ERR_ICMD_SYFAN:    Set_Sy()                   ;break; //����ʧѹ����
      case ERR_ICMD_MFCLKMD:  Set_MFCLK_Mode()           ;break; //���ö๦�����巽ʽ
      case ERR_ICMD_MFCLKTY:  Set_MFCLK_Type()           ;break; //��ǰ�๦����������
      case ERR_ICMD_ELEPLS:   Set_Elec_Pls()             ;break; //���õ����������� PA PR QA QR ������          	
      case ERR_ICMD_IJDQR:    RST_IJDQ()                 ;break; //��λ������·�̵���
      case ERR_ICMD_LEDTST:   LED_TST()                  ;break; //����������ʾ8.8.8.8.8.8.8.8.
      case ERR_ICMD_RSTLED:   RST_LED()                  ;break; //��λ7279
      case ERR_ICMD_TZEN:     Set_TZEN()                 ;break; //��բ���ʹ������       
      case ERR_ICMD_MBJEN:    Set_MBJEN()                ;break; //�������źż��ʹ�� 
      case ERR_ICMD_TS:       Set_TS()                   ;break; //������̨��������
      case ERR_ICMD_DXTZ:     Set_DXTZ()                 ;break; //���õ�����բ
      case ERR_ICMD_START:    Set_Master_Start()         ;break; //�ܿ�������������
      case ERR_ICMD_SETCK:                               ;break; //���ò忨 �Ƹǲ忨ר��ר��ָ��
      case ERR_ICMD_UCLOP:    Set_UCLOP()                ;break; //���ñ�λ��ѹ���� �Ͽ� 
      case ERR_ICMD_ZBTST:    Set_ZBTST()                ;break; //�ر�����
      case ERR_ICMD_LIGHT:    Set_Light()                ;break; //����״ָ̬ʾ�ƿ���
      case ERR_ICMD_SLEDN:    Set_SLEDN()                ;break; //��������LEDλ�� 	
      case ERR_ICMD_VSET:                                 break; //�����ز��������ѹ�������
      case ERR_ICMD_COM0SEL:  Set_COM0SEL()              ;break; //���ý���UART0��ͨѶ��ʽ
      case ERR_ICMD_COM1SEL:  Set_COM1SEL()              ;break; //���ý���UART1��ͨѶ��ʽ       
      case ERR_ICMD_ZBLBIN:   Set_ZBLBIN()               ;break; //�ز��˲���·����ѡ��          
      case ERR_ICMD_YXCTL:    Set_YXCTL()                ;break; //ң���źſ�������              
      case ERR_ICMD_DOORCTL:  Set_DOORCTL()              ;break; //�ſ��źſ�������                  
      case ERR_ICMD_EQPMOD:   Set_EQPMOD()               ;break; //����豸��������              
      case ERR_ICMD_PULSET:   Set_PULSET()               ;break; //���������������(���ר���ն�)
      case ERR_ICMD_PULRUN:   Set_PULRUN()               ;break; //���������������(���ר���ն�)
      case ERR_ICMD_YKSET:    Set_YKSET()                ;break; //�ն�ң���ź�����              
      case ERR_ICMD_YKPULSE:  Set_YKPULSE()              ;break; //����ң���ź����崥����ʽ 
      case ERR_ICMD_NPRS485:  Set_NPRS485()              ;break; //�޼���RS485����
      case ERR_ICMD_MTYPE:    Set_MTYPE()                ;break; //������ ������ 'N'  ������'S' 						
						case ERR_ICMD_ADCONV:   Set_ADCONV()               ;break; //12Vֱ����ѹ��������
						case ERR_ICMD_ULVL:                                 break; //���õ�ѹѡ��70%or100%
						case ERR_ICMD_WZTZCTL:                              break; //������բ�źŶ��Ӽ̵������ơ�O���Ͽ���C���պ�						
						case ERR_ICMD_TTAB:     Set_TTAB()                 ;break; //����װ��˫��·�л�����
						case ERR_ICMD_MEAPN:    Set_MEAPN()                ;break; //���й��Ĳ��Կ���
						case ERR_ICMD_CHKP:     Set_CHKP()                 ;break; //���й��ĸ���������ݲ�ѯ
//��Ԫ��������(ʱ��У���ǲ�������)
      case ERR_ICMD_MBAUD:    Set_MTR_LCT_Com(MTRCOM)    ;break; //���ö๦�ܱ�(ģ���)����ͨ�Ų���
      case ERR_ICMD_LBAUD:    Set_MTR_LCT_Com(LCTCOM)    ;break; //���ø����ն˴���ͨ�Ų���
      case ERR_ICMD_CLKFRQ:   Set_Clk_Freq_Pr()          ;break; //���ø���λʱ��Ƶ��
      case ERR_ICMD_CLKTIM:   Set_Clk_Time_Pr()          ;break; //���øñ�λʱ��Ƶ�ʲ���ʱ��
      case ERR_ICMD_CLKCTL:   Set_Clk_Ctl_Pr()           ;break; //ʱ��Ƶ�ʲ�������
      case ERR_ICMD_XULCTL:   Set_XuL_Ctl_Pr()           ;break; //�������ڲ�������
      case ERR_ICMD_XULPLS:   Set_XuL_Pls_Num()          ;break; //�����������ڲ�������
      case ERR_ICMD_EDIS:     Set_Disp_Mode()            ;break; //������ʾģʽ
      case ERR_ICMD_RLMDT:    Set_Reload_MtrD()          ;break; //����װ��ģ�������     ��λ��   -> ģ���   
      case ERR_ICMD_RLLDT:    Set_Reload_LctD()          ;break; //����װ�ظ����ն�����   ��λ��   -> �����ն� 
      case ERR_ICMD_RTMDT:    Set_ReTx_MtrD()            ;break; //�ط��յ���ģ�������   ģ���   -> ��λ��   
      case ERR_ICMD_RTLDT:    Set_ReTx_LctD()            ;break; //�ط��յ��ĸ����ն����� �����ն� -> ��λ��   
      case ERR_ICMD_SEC:      Set_Time_Base()            ;break; //����ʱ���׼
      case ERR_ICMD_RED485:   Set_Red485()               ;break; //����RS485 �ڶ�ͨ��
      case ERR_ICMD_READ_VER: Set_Read_Ver()             ;break; //���汾��
      case ERR_ICMD_RSTS:     Set_RSTS()                 ;break; //��С��ʾ����״̬
      case ERR_ICMD_SMTR:                                 break; //���ñ�׼��
					 case ERR_ICMD_RBAUD:    Set_MTR_LCT_Com(RTECOM)    ;break; //����RTECOM����ͨ�Ų���  
      case ERR_ICMD_ABAUD:    Set_MTR_LCT_Com(ATECOM)    ;break; //����ATECOM����ͨ�Ų���
      case ERR_ICMD_IBAUD:    Set_MTR_LCT_Com(IRECOM)    ;break; //����IRECOM����ͨ�Ų���
      	
//���Ӹ����ն������

//��������
      case JTAG_EN:           JTAG_EN_Pr()               ;break; //����ʹ�� JTAG �ڿ���      
      default:break;     
     }
    CAN_SDATA_MSG_ITail++;
    if(CAN_SDATA_MSG_ITail>=CAN_SDILEN)
     {
      CAN_SDATA_MSG_ITail=0;
      CAN_MSG_IPtr=&CAN_SDATA_MSG_IBUF[0];                       //��ʼ�����ձ��Ĵ���ָ��
     }                                    
    else                                  
     CAN_MSG_IPtr+=1;                                            //ָ�� 

}
/*****************************************************************************
* ����CAN1�����ݽ���ָ�����
*****************************************************************************/
void Proc_CNA1_SDATA_IBUF(void)
{
	   if(CAN1_SDATA_MSG_IHead==CAN1_SDATA_MSG_ITail)
     return;                                                     //û���յ��������˳�
    CAN1_RX_OVTimer=(u16)Timer_1ms;                               //��ʼ��CAN��ʱ��ʱ��
                  
    switch(CAN1_MSG_IPtr->ID.BIT.CMD)                             //����������ת����Ӧλ�ô���
				{
					  
					  default:break;
				}
				CAN1_SDATA_MSG_ITail++;
				if(CAN1_SDATA_MSG_ITail>=CAN_SDILEN)
				{
					  CAN1_SDATA_MSG_ITail=0;
					  CAN1_MSG_IPtr=&CAN1_SDATA_MSG_IBUF[0];                    //��ʼ�����ձ��Ĵ���ָ��
				}
				else
					CAN1_MSG_IPtr+=1;                                           //ָ��
}
/*****************************************************************************
* CAN�����ݷ���ָ�����ͷָ���1����
*****************************************************************************/
void SDATA_MSG_OHead_ADD_ONE(void)
{
    CAN_SDATA_MSG_OHead++;
    if(CAN_SDATA_MSG_OHead>=CAN_SDOLEN)
     {
      CAN_SDATA_MSG_OHead=0;              //����ָ��
      CAN_MSG_OPtr=&CAN_SDATA_MSG_OBUF[0];//��ʼ�����ͱ��Ĵ���ָ��
     }
    else
     CAN_MSG_OPtr+=1;
}           
/*****************************************************************************
* ����CAN�����ݷ���ָ�����
*****************************************************************************/
void Proc_SDATA_OBUF(void)
{
    if(CAN_SDATA_MSG_OHead==CAN_SDATA_MSG_OTail)
     return;                                  //û�ж�����Ҫ�����˳�
    if(CAN_SMSG_TX_STS != 0 )                 //CAN�������Ƿ��ڷ���״̬
     return;
    if(!MASTER_START)                         //�ܿ������Ƿ�����
     return;	
    if(CAN_SEND_DELAY)                        //Ϊ��ֹ���ɾ��� ���ӱ�λ��ʱ
     {
      CAN_SEND_DELAY--;
      return;
     }	
    CAN_SEND_DELAY=CAN_SEND_DTIME;            // 	
    CAN_SMSG_TX_STS = SLV_SMSG_TX_IS;
    CAN_STX_OVTimer=(u16)Timer_1ms;           //���÷��Ͷ�ʱ
    CAN_Tx_Msg(CAN0,&CAN_SDATA_MSG_OBUF[CAN_SDATA_MSG_OTail],MSG_OBJ_TYPE_TX);  //���Ͷ�֡
    CAN_SDATA_MSG_OTail++;                    //ָ���1 ָ����һ֡ 
    if(CAN_SDATA_MSG_OTail>=CAN_SDOLEN)       //�ж��Ƿ񳬳�ѭ��   
     CAN_SDATA_MSG_OTail=0;                   //��ͷ��ʼѭ��       
}
/*****************************************************************************
* ����CAN1�����ݷ���ָ�����
*****************************************************************************/
void Proc_CAN1_SDATA_OBUF(void)
{
    if(CAN1_SDATA_MSG_OHead==CAN1_SDATA_MSG_OTail)
     return;                                  //û�ж�����Ҫ�����˳�
    if(CAN1_SMSG_TX_STS != 0 )                 //CAN�������Ƿ��ڷ���״̬
     return;
//    if(!MASTER_START)                         //�ܿ������Ƿ�����
//     return;	
//    if(CAN_SEND_DELAY)                        //Ϊ��ֹ���ɾ��� ���ӱ�λ��ʱ
//     {
//      CAN_SEND_DELAY--;
//      return;
//     }	
//    CAN_SEND_DELAY=CAN_SEND_DTIME;            // 	
    CAN1_SMSG_TX_STS = SLV_SMSG_TX_IS;
    CAN1_STX_OVTimer=(u16)Timer_1ms;           //���÷��Ͷ�ʱ
    CAN1_Tx_Msg(CAN1,&CAN1_SDATA_MSG_OBUF[CAN1_SDATA_MSG_OTail],MSG_OBJ_TYPE_TX);  //���Ͷ�֡
    CAN1_SDATA_MSG_OTail++;                    //ָ���1 ָ����һ֡ 
    if(CAN1_SDATA_MSG_OTail>=CAN_SDOLEN)       //�ж��Ƿ񳬳�ѭ��   
     CAN1_SDATA_MSG_OTail=0;                   //��ͷ��ʼѭ��       	
}
/*****************************************************************************
* ����CAN����״̬
* ������ʱ
*****************************************************************************/
void Proc_CAN_STS(void)        
{
    if((CAN_STS.BYTE&(CAN_STS_BOFF|CAN_STS_EWARN|CAN_STS_EPASS))||
      ((u16)(Timer_1ms-CAN_RX_OVTimer)>CAN_RX_OVTM)) 
     {
      CAN_RX_OVTimer=(u16)Timer_1ms;      //��ʱ��ʱ	
      CAN_STS.BYTE=0;                     //�������
      CAN_ERR=1;                          //CAN����״̬��־
      CAN_SMSG_TX_STS=COM_TX_NO;          //��������ݷ��ͱ�־      ��Ϊ���Ϳ���״̬
      CAN_LMSG_TX_STS=COM_TX_NO;          //���CAN������֡����״̬ ��Ϊ���Ϳ���״̬
      CAN_LDATA_TX_STS = SLV_LDATA_TX_NO; //��������ݷ���״̬      ��Ϊ���Ϳ���״̬         
      Init_CAN0();                         //���³�ʼ������
     } 	
}
/*****************************************************************************
* ����CAN1����״̬
* ������ʱ
*****************************************************************************/
void Proc_CAN1_STS(void)        
{
    if((CAN1_STS.BYTE&(CAN_STS_BOFF|CAN_STS_EWARN|CAN_STS_EPASS))||
      ((u16)(Timer_1ms-CAN1_RX_OVTimer)>CAN_RX_OVTM)) 
     {
      CAN1_RX_OVTimer=(u16)Timer_1ms;      //��ʱ��ʱ	
      CAN1_STS.BYTE=0;                     //�������
      CAN1_ERR=1;                          //CAN����״̬��־
      CAN1_SMSG_TX_STS=COM_TX_NO;          //��������ݷ��ͱ�־      ��Ϊ���Ϳ���״̬
//      CAN_LMSG_TX_STS=COM_TX_NO;          //���CAN������֡����״̬ ��Ϊ���Ϳ���״̬
//      CAN_LDATA_TX_STS = SLV_LDATA_TX_NO; //��������ݷ���״̬      ��Ϊ���Ϳ���״̬         
      Init_CAN1();                         //���³�ʼ������
     } 	
}
