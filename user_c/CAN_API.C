/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CAN.C
;* Author             : 张力阵
;* CAN总线数据处理程序 
;* 集成了CAN接口的LM3S 系列处理器 由于CAN时钟和CPU时钟之间频率的不一致,造成访问CAN寄存器时
;* 要添加额外延时 CPU 时钟频率高于CAN时钟频率
*******************************************************************************/
#include "Function.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
#include "string.h"
/*****************************************************************************
* CAN长数据接收处理结构体表格
* CAN_LDT_TYPE_NUMB 接收长数据类型个数 在define.h中定义 
* 接收到的长数据直接存入发送缓冲区
*****************************************************************************/
const CAN_LMSG_PR CAN_LMSG_RX_TAB[CAN_LDT_TYPE_NUMB]=
{ //接收到那个缓冲区 接收头指针 接收尾指针 接收缓冲区状态指针
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
* CAN长数据发送处理结构体表格
* CAN_LDT_TYPE_NUMB 长数据类型个数 在define.h中定义 
* 接收到的长数据直接存入发送缓冲区
*****************************************************************************/
const CAN_LMSG_PR CAN_LMSG_TX_TAB[CAN_LDT_TYPE_NUMB]=
{ //接收到那个缓冲区 接收头指针 接收尾指针 接收缓冲区状态指针
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
* CAN短数据帧发送
****************************************************************************/
const CAN_MSG CAN_TX_SMSG=
{   
    0x000D0050,    //控制域 EXD_ID=1 TX_INT_EN=1 
    0x10010008,    //仲裁域 EXD=1 DIR=1 END=1
    0x0000,        //DA1
    0x0000,        //DA2
    0x0000,        //DA3
    0x0000,        //DA4
}; 
/****************************************************************************
* CAN长数据帧接收缓冲区处理
****************************************************************************/
void Proc_LDATA_MSG_IBUF(void)
{
    CAN_MSG  *CAN_RX_MSG;	                            //定义CAN接收帧指针
    if(CAN_LDATA_MSG_ITail==CAN_LDATA_MSG_IHead)
     return;
    CAN_RX_MSG = &CAN_LDATA_MSG_IBUF[CAN_LDATA_MSG_ITail]; //初始化接收报文指针
    switch(CAN_RX_MSG->CTL.BIT.IDx)                   //根据报文编号跳转
     {                                                
      case MST_LDATA_ACK:                             //主机批准从机发送长数据响应
       {                                              //接收中断			    ID4
        if(CAN_RX_MSG->ID.BIT.TYPE==CAN_LMSG_TX_TYPE) //批准的数据类型是否为请求的数据类型
         CAN_LDATA_TX_STS = SLV_LDATA_TX_ACK;         //批准长数据发送
        break;
       }
      case MST_LCDATA_TX:                             //主机正在发送长广播数据串ID5 
      case MST_LSDATA_TX:                             //主机正在发送长单播数据串ID6 
       {                                              //长数据接收处理 接收中断
        CAN_LDATA_RX_Pr(CAN_RX_MSG);                  //长数据接收处理
        break;                                        
       }                                              
      case MST_LDATA_REQRT:                           //主机请求从机重发数据    ID7
       {                                              //接收中断
        if(CAN_RX_MSG->ID.BIT.TYPE >= CAN_LDT_ACT_NUMB)//判断长数据类型是否在设定范围内
         break;                                       //超过范围 退出
        if((CAN_LDATA_TX_STS!=SLV_LDATA_TX_NO)&&      //判断该缓冲区是否在空闲状态 空闲状态 
           (CAN_RX_MSG->ID.BIT.TYPE == CAN_LMSG_TX_TYPE))//判断正在发送的数据和请求的数据是否一致
         {
          CAN_LDATA_TX_STS = SLV_LDATA_TX_ACK;        //批准长数据发送
         }
        else
         {
          CAN_LMSG_TX_TYPE=CAN_RX_MSG->ID.BIT.TYPE;
          memcpy(&CAN_LMSG_TX_Ptr,
                 &CAN_LMSG_TX_TAB[CAN_LMSG_TX_TYPE],
                 16);                                 //拷贝发送处理结构体
          if(*CAN_LMSG_TX_Ptr.STS == COM_RX_IS)       //判断该缓冲区是否正在接收数据
      	   break;                                     //正在接收退出
          CAN_LDATA_TX_STS = SLV_LDATA_TX_ACK;        //批准长数据发送
          *CAN_LMSG_TX_TAB[CAN_RX_MSG->ID.BIT.TYPE].STS = COM_RX_END;//有数据标志 等待发送
          CAN_LMSG_TX_TYPE = CAN_RX_MSG->ID.BIT.TYPE; 
         } 
        break;                      
       }                            
     }
    CAN_LDATA_MSG_ITail++;                              //处理指针加1 指向下一个CAN长数据帧
    if(CAN_LDATA_MSG_ITail>=CAN_LDILEN)                 //判断是否超出循环
     CAN_LDATA_MSG_ITail=0;                             //重置
    memset(CAN_RX_MSG,0,16);                            //清除长数据接收缓冲区接收报文
}
/*****************************************************************************
* 长数据发送处理
*****************************************************************************/
void CAN_LDATA_TX_Pr(void)
{
    u16 Len;                                      //数据长度
    if(CAN_LMSG_TX_STS != SLV_LDATA_TX_NO)        //判断单帧发送是否空闲
     return;                                      //不是退出
    if(CAN_LDATA_TX_STS==SLV_LDATA_TX_LAST)       //最后一帧数据发送完成处理
     {
      *CAN_LMSG_TX_Ptr.STS  = COM_RX_NO;          //清除缓冲区有数据标志
      *CAN_LMSG_TX_Ptr.TAIL =0;                   //
      CAN_LDATA_TX_STS=SLV_LDATA_TX_NO;           //清除长数据发送状态      置为发送空闲状态
      CAN_LDATA_SERR_Cnt=0;                       //长数据发送错误计数                    
      return;                   
     }
    Len = (*CAN_LMSG_TX_Ptr.HEAD -                
           *CAN_LMSG_TX_Ptr.TAIL);                //剩余数据长度
    if(Len<9)                                     //剩余数据长度大于8字节
     CAN_LMSG_Tx.ID.BIT.END=1;                    //最后一帧
    else
     Len=8;
    CAN_LMSG_Tx.CTL.BIT.LEN = Len;
    memcpy(&CAN_LMSG_Tx.Data,
           CAN_LMSG_TX_Ptr.BUF,
           Len);
    CAN_LMSG_TX_Ptr.BUF += Len;                   //更新发送指针
    *CAN_LMSG_TX_Ptr.TAIL +=Len;
    CAN_LMSG_TX_STS = SLV_LDATA_TX_IS;            //单帧正在发送 等待进入发送完成中断清除
    CAN_LTX_OVTimer=(u16)Timer_1ms;               //重置发送定时
    if(CAN_LMSG_Tx.ID.BIT.END)
     CAN_LDATA_TX_STS = SLV_LDATA_TX_LAST;        //正在发送最后一帧
    CAN_Tx_Msg(CAN0,
               &CAN_LMSG_Tx,
               MSG_OBJ_TYPE_TX);                  //发送数据帧
    CAN_LMSG_Tx.ID.BIT.IDX++;                     //下一帧编号
    if(CAN_LMSG_Tx.ID.BIT.IDX==0)                  
     CAN_LMSG_Tx.ID.BIT.IDX=1;                    //下一帧编号循环
}

/*****************************************************************************
* 长数据接收处理
* 人口:*CAN_RX_MSG	正在处理CAN报文指针
* 处理时间28us
* 单步调试时 连续发送数据可能会丢帧 正常运行时一般没问题
*****************************************************************************/
void CAN_LDATA_RX_Pr(CAN_MSG *CAN_RX_MSG)
{  
    if(CAN_RX_MSG->ID.BIT.TYPE >= CAN_LDT_ACT_NUMB)//判断长数据类型是否在设定范围内
     return;                                      //非法命令退出
    if(CAN_RX_MSG->ID.BIT.IDX ==0)                //判断是否为第一帧数据
     {                                            //收到第一帧数据重置接收状态
      memcpy(&CAN_LMSG_RX_Ptr,
             &CAN_LMSG_RX_TAB[CAN_RX_MSG->ID.BIT.TYPE],
             16);	                                //拷贝接收处理结构体
      memcpy(CAN_LMSG_RX_Ptr.BUF,
             &CAN_RX_MSG->Data,
             CAN_RX_MSG->CTL.BIT.LEN);	          //考出数据到接收缓冲区
      CAN_LMSG_RX_TYPE = CAN_RX_MSG->ID.BIT.TYPE; //当前接收的CAN长数据类型
      CAN_LMSG_RX_Ptr.BUF += CAN_RX_MSG->CTL.BIT.LEN;//刷新缓冲区指针
      *CAN_LMSG_RX_Ptr.STS = SLV_LDATA_RX_IS;     //设置缓冲区状态为接收
      CAN_NEXT_MSG_IDx = 1;                       //下一条长数据报文索引IDx
     }
    else if(CAN_RX_MSG->ID.BIT.TYPE!=CAN_LMSG_RX_TYPE)
     return;                                     //类型不正确
    else if(*CAN_LMSG_RX_Ptr.STS==SLV_LDATA_RX_NO)
     return;                                     //空闲状态退出 
    else if(CAN_RX_MSG->ID.BIT.IDX != CAN_NEXT_MSG_IDx) //长数据索引不正确
     {                                           //报文有丢失 
      if(*CAN_LMSG_RX_Ptr.STS==SLV_LDATA_RX_IS)  //正在接收长数据
       {
        *CAN_LMSG_RX_Ptr.STS=0;                  //发送重发申请命令
        CAN_NEXT_MSG_IDx=0;                      //下一帧报文编号
        SLV_REQ_RETX();	                         //发送从机请求重发命令
       }  
     }
    else
     {
      memcpy(CAN_LMSG_RX_Ptr.BUF,
             &CAN_RX_MSG->Data,
             CAN_RX_MSG->CTL.BIT.LEN);
      CAN_LMSG_RX_Ptr.BUF += CAN_RX_MSG->CTL.BIT.LEN; //刷新缓冲区指针
      CAN_NEXT_MSG_IDx++;                        //设置下一个报文索引IDx
      if(CAN_NEXT_MSG_IDx>7)
       CAN_NEXT_MSG_IDx =1 ;
     }
    if(CAN_RX_MSG->ID.BIT.END== 1)
     {                                           //长数据发送结束命令   
      *CAN_LMSG_RX_Ptr.TAIL=0;
      *CAN_LMSG_RX_Ptr.HEAD=(CAN_LMSG_RX_Ptr.BUF-
                             CAN_LMSG_RX_TAB[CAN_RX_MSG->ID.BIT.TYPE].BUF);//刷新指针
      *CAN_LMSG_RX_Ptr.STS=COM_RX_END;	             //发送缓冲区有新数据标志
     }
}
/*****************************************************************************
* 从机CAN数据发送超时处理
*****************************************************************************/
void Proc_CAN_OvTm(void)
{
    if(CAN_LDATA_TX_STS)                             //长数据发送或请求状态
     {
      if((u16)(Timer_1ms-CAN_LTX_OVTimer)>=CAN_TX_OVTM)//长数据发送超时处理
       {	 
        CAN_LMSG_TX_STS=COM_TX_NO;                   //清除CAN长数据帧发送状态 置为发送空闲状态
        CAN_LDATA_TX_STS=SLV_LDATA_TX_NO;            //清除长数据发送状态      置为发送空闲状态         
        CAN_LDATA_SERR_Cnt++;                        //长数据发送错误计数                    
        if(CAN_LDATA_SERR_Cnt>=CAN_LDATA_SERR_MAX)   //长数据发送错误最大计数                    
         {
          CAN_LDATA_SERR_Cnt=0;                      //清除错误计数
          *CAN_LMSG_TX_Ptr.STS  = COM_RX_NO;         //清除缓冲区有数据标志
          *CAN_LMSG_TX_Ptr.TAIL =0;                  //清除缓冲区处理指针
         }	     
       } 
     }		
    if(CAN_SMSG_TX_STS)                              //短数据帧是否处于发送状态
     {
      if((u16)(Timer_1ms-CAN_STX_OVTimer)>=CAN_TX_OVTM)//短数据发送超时处理
       {
        CAN_SMSG_TX_STS=COM_TX_NO;                   //清除短数据发送标志      置为发送空闲状态
       }		
     }		
}
/*****************************************************************************
* 从机CAN1数据发送超时处理
*****************************************************************************/
void Proc_CAN1_OvTm(void)
{
    if(CAN1_SMSG_TX_STS)                              //短数据帧是否处于发送状态
     {
      if((u16)(Timer_1ms-CAN1_STX_OVTimer)>=CAN_TX_OVTM)//短数据发送超时处理
       {
        CAN1_SMSG_TX_STS=COM_TX_NO;                   //清除短数据发送标志      置为发送空闲状态
       }		
     }		
}
/*****************************************************************************
* 从机发送数据请求帧
* CAN_LMSG_TX_TYPE 发送数据类型
* 从机有长数据要发送时,先发送请求帧 等待主机批准
*****************************************************************************/
void SLV_REQ_TX(void)
{
    CAN_MSG CAN_REQ_TX_MSG;
    memcpy(&CAN_REQ_TX_MSG,
           &CAN_MSG_SET_TAB[SLV_LDATA_REQTX-1],
           8);                                    //导入控制域和仲裁域
    CAN_REQ_TX_MSG.ID.BIT.TYPE = CAN_LMSG_TX_TYPE;//请求类型
    CAN_REQ_TX_MSG.ID.BIT.MNUM = Mtr_Numb_ID;     //表位号
    CAN_LDATA_TX_STS=SLV_LDATA_TX_REQ;            //设置长数据发送状态	为已发送请求帧 等待批准
    CAN_LTX_OVTimer=(u16)Timer_1ms;               //重置定时器
    CAN_Tx_Msg(CAN0,&CAN_REQ_TX_MSG,MSG_OBJ_TYPE_TX);
}
/*****************************************************************************
* 从机发送重发请求帧
* CAN_LMSG_RX_TYPE 重发数据类型
* 从机接收数据出错后 要求主机重发数据
*****************************************************************************/
void SLV_REQ_RETX(void)
{
    CAN_MSG CAN_REQ_RETX_MSG;
    memcpy(&CAN_REQ_RETX_MSG,
           &CAN_MSG_SET_TAB[SLV_LDATA_REQRT-1],
           8);                                       //导入控制域和仲裁域
    CAN_REQ_RETX_MSG.ID.BIT.TYPE = CAN_LMSG_RX_TYPE; //发送长数据类型
    CAN_REQ_RETX_MSG.ID.BIT.MNUM = Mtr_Numb_ID;      //表位号
    CAN_Tx_Msg(CAN0,&CAN_REQ_RETX_MSG,MSG_OBJ_TYPE_TX);
}
/*****************************************************************************
* 处理主机查询信息
* 2011.8.23 回送主机查询 
* 回送1字节数据 包括复位信息和工作状态
*****************************************************************************/
void Proc_Mst_Check(void)
{
	if(Echo_Sts==MST_CHK_RCVD)
	 {                                            //收到主机查询帧
      u8 m;
      CAN_MSG CAN_ECHO_MSG;
      memcpy(&CAN_ECHO_MSG,
             &CAN_MSG_SET_TAB[SLV_CHK_ECHO-1],
             8);                                //导入控制域和仲裁域
      CAN_ECHO_MSG.ID.BIT.MNUM = Mtr_Numb_ID;   //表位号
      CAN_ECHO_MSG.CTL.BIT.LEN=1;               //数据长度 1个字节
      m=WORK_MODE;
      if(RST_FLAG)                              //开机复位标志
       {
        m|=0x10;                                //BIT4 复位标志
        RST_FLAG=0;                             //清除复位标志
       }  
      CAN_ECHO_MSG.Data.BYTE[0]=m;
      CAN_Tx_Msg(CAN0,&CAN_ECHO_MSG,MSG_OBJ_TYPE_TX);
      Echo_Sts= SLV_ECHO_SEND;                  //已经申请发送 送入MSG RAM
      CAN_RX_OVTimer=(u16)Timer_1ms;            //初始化CAN超时定时器
	 }
}
/*****************************************************************************
* 处理CAN短数据接收指令缓冲区
*****************************************************************************/
void Proc_SDATA_IBUF(void)
{
    if(CAN_SDATA_MSG_IHead==CAN_SDATA_MSG_ITail)
     return;                                                     //没有收到短数据退出
    CAN_RX_OVTimer=(u16)Timer_1ms;                               //初始化CAN超时定时器
    MASTER_START=1;                                              //主机已启动
    switch(CAN_MSG_IPtr->ID.BIT.CMD)                             //根据命令跳转到相应位置处理
     {
//误差单元原有命令     
      case ERR_ICMD_GD:       Set_STD_CST_Pr()           ;break; //接收标准表高频常数   XXXX float 数据 
      case ERR_ICMD_GE:       Set_N_Pr()                 ;break; //接收校验圈数
      case ERR_ICMD_GF:       ERR_CLR_Pr()               ;break; //接收误差清零
      case ERR_ICMD_GG:       ELEC_HEAD_RESET_Pr()       ;break; //接收光电头对光
      case ERR_ICMD_GH:       MTR_PLUG_Pr()              ;break; //选表位 挂表处理
      case ERR_ICMD_GI:       CATCH_HB_Pr()              ;break; //对黑斑         ;0退去其他试验，回到校验状态
      case ERR_ICMD_GJ:       Start_Stop_Pr()            ;break; //监视光电头脉冲 ;1起动/潜动时记数 0退去其他试验，回到校验状态
      case ERR_ICMD_GK:       Set_PLS_QUAD()             ;break; //接收脉冲所在象限
      case ERR_ICMD_GL:       BEEP_EN_Pr()               ;break; //喇叭开关
      case ERR_ICMD_GM:       Login_Verify_Sts()         ;break; //进入校核常数试验状态,准备开始走字试验
      case ERR_ICMD_GN:       Verify_Start_Pr()          ;break; //开始进行校核常数走字试验
      case ERR_ICMD_GO:       Set_Mtr_Cst_Pr()           ;break; //接收被校表常数
      case ERR_ICMD_GP:       PLS_SEL_Pr()               ;break; //脉冲选择  电子脉冲还是光电头脉冲
      case ERR_ICMD_GQ:       Cnt_ENG_Pr()               ;break; //计电能试验      1:开始  0:终止
      case ERR_ICMD_GR:       ENG_CLR_Pr()               ;break; //清除电能计数
      case ERR_ICMD_GS:       PZ_ERR_Pr()                ;break; //脉冲比较  电子脉冲和光电头脉冲比较
      case ERR_ICMD_GT:       Set_Pzbz_Pr()              ;break; //接收脉冲和盘转比值
      case ERR_ICMD_GU:       Set_Pzzs_Pr()              ;break; //接收脉冲和盘转设定比较圈数
      case ERR_ICMD_GV:       Login_Sy_Sts()             ;break; //进入失压状态
      case ERR_ICMD_GW:       Start_Sy_Pr()              ;break; //失压试验开始
      case ERR_ICMD_GX:       CYCLE_PLS_SEL_Pr()         ;break; //选择需要测量脉冲周期和频率的脉冲
      case ERR_ICMD_GY:       DIVIDE_COEF_Pr()           ;break; //送分频系数
      case ERR_ICMD_GZ:       MEA_MTR_CST_Pr()           ;break; //计读脉冲法常数测试试验
      case ERR_ICMD_Gd:       Power_Test_Pr()            ;break; //功耗测试 表位和相别选择
      case ERR_ICMD_Ge:                                   break; //功耗测试仪 电压线圈有功修正值
      case ERR_ICMD_Gf:                                   break; //电压线圈视在功率修正值
      case ERR_ICMD_Gg:                                   break; //接收电流线圈视在功率修正值
      case ERR_ICMD_Gh:                                   break; //设置功耗测量单元单元号
      case ERR_ICMD_Gi:       MEA_PLS_CYCLE_Pr()         ;break; //脉冲波形测试
      case ERR_ICMD_Gj:       WIRE_TYPE_Pr()             ;break; //接收接线方式
      case ERR_ICMD_Gk:       ELEC_PLS_TYPE_Pr()         ;break; //被校表脉冲输出类型选择 共高共低
      case ERR_ICMD_Gl:       Set_ZZ_PLS_Pr()            ;break; //设置走字脉冲数
      case ERR_ICMD_Gm:       PLS_ZZ_Pr()                ;break; //脉冲走字试验
      case ERR_ICMD_Gn:       ReSet_ZZ_PLS_Pr()          ;break; //预置脉冲数
//误差单元接收命令(多功能走字耐压台通讯命令)             
      case ERR_ICMD_Go:       Start_Ny_Pr()              ;break; //开始耐压试验
      case ERR_ICMD_Gp:       Ny_Time_End_Pr()           ;break; //耐压时间到 命令
      case ERR_ICMD_Gq:       Chg_U_In_Pr()              ;break; //切换表尾电压端子
      case ERR_ICMD_Gr:       STS_Light_Pr()             ;break; //误差校验指示灯控制命令
//      case ERR_ICMD_Gs:       Relay_Close_Pr()           ;break; //继电器吸合        
//      case ERR_ICMD_Gt:       Relay_Open_Pr()            ;break; //继电器断开
//      case ERR_ICMD_Gu:       I_Open_Tst_Pr()            ;break; //电流回路开路检测
//误差单元新增命令部分                                   
      case ERR_ICMD_SYPH:     Set_Sy_Ph()                ;break; //接收失压相别
      case ERR_ICMD_MIN_CST:  Set_Min_CST_Pr()           ;break; //设置最小电表常数
      case ERR_ICMD_ZZDS:     Set_ZZ_Ds()                ;break; //接收校核常数走字度数      
      case ERR_ICMD_MCHC:     Set_PLSHC();               ;break; //设置脉冲合成方式
      case ERR_ICMD_SYFAN:    Set_Sy()                   ;break; //接收失压方案
      case ERR_ICMD_MFCLKMD:  Set_MFCLK_Mode()           ;break; //设置多功能脉冲方式
      case ERR_ICMD_MFCLKTY:  Set_MFCLK_Type()           ;break; //当前多功能脉冲类型
      case ERR_ICMD_ELEPLS:   Set_Elec_Pls()             ;break; //设置电子脉冲输入 PA PR QA QR 调试用          	
      case ERR_ICMD_IJDQR:    RST_IJDQ()                 ;break; //复位电流旁路继电器
      case ERR_ICMD_LEDTST:   LED_TST()                  ;break; //测试命令显示8.8.8.8.8.8.8.8.
      case ERR_ICMD_RSTLED:   RST_LED()                  ;break; //复位7279
      case ERR_ICMD_TZEN:     Set_TZEN()                 ;break; //合闸检测使能命令       
      case ERR_ICMD_MBJEN:    Set_MBJEN()                ;break; //表报警信号检测使能 
      case ERR_ICMD_TS:       Set_TS()                   ;break; //单三相台设置命令
      case ERR_ICMD_DXTZ:     Set_DXTZ()                 ;break; //设置单相跳闸
      case ERR_ICMD_START:    Set_Master_Start()         ;break; //总控中心启动命令
      case ERR_ICMD_SETCK:                               ;break; //设置插卡 掀盖插卡专机专用指令
      case ERR_ICMD_UCLOP:    Set_UCLOP()                ;break; //设置表位电压接入 断开 
      case ERR_ICMD_ZBTST:    Set_ZBTST()                ;break; //载表试验
      case ERR_ICMD_LIGHT:    Set_Light()                ;break; //误差板状态指示灯控制
      case ERR_ICMD_SLEDN:    Set_SLEDN()                ;break; //设置误差板LED位数 	
      case ERR_ICMD_VSET:                                 break; //设置载波虚拟表电压接入相别
      case ERR_ICMD_COM0SEL:  Set_COM0SEL()              ;break; //设置接入UART0的通讯方式
      case ERR_ICMD_COM1SEL:  Set_COM1SEL()              ;break; //设置接入UART1的通讯方式       
      case ERR_ICMD_ZBLBIN:   Set_ZBLBIN()               ;break; //载波滤波电路接入选择          
      case ERR_ICMD_YXCTL:    Set_YXCTL()                ;break; //遥信信号控制命令              
      case ERR_ICMD_DOORCTL:  Set_DOORCTL()              ;break; //门控信号控制命令                  
      case ERR_ICMD_EQPMOD:   Set_EQPMOD()               ;break; //检测设备类型设置              
      case ERR_ICMD_PULSET:   Set_PULSET()               ;break; //脉冲参数设置命令(针对专变终端)
      case ERR_ICMD_PULRUN:   Set_PULRUN()               ;break; //脉冲输出设置命令(针对专变终端)
      case ERR_ICMD_YKSET:    Set_YKSET()                ;break; //终端遥控信号设置              
      case ERR_ICMD_YKPULSE:  Set_YKPULSE()              ;break; //设置遥控信号脉冲触发方式 
      case ERR_ICMD_NPRS485:  Set_NPRS485()              ;break; //无极性RS485测试
      case ERR_ICMD_MTYPE:    Set_MTYPE()                ;break; //表类型 国网表 'N'  南网表'S' 						
						case ERR_ICMD_ADCONV:   Set_ADCONV()               ;break; //12V直流电压测量控制
						case ERR_ICMD_ULVL:                                 break; //设置电压选择，70%or100%
						case ERR_ICMD_WZTZCTL:                              break; //外置跳闸信号端子继电器控制‘O’断开‘C’闭合						
						case ERR_ICMD_TTAB:     Set_TTAB()                 ;break; //单相装置双回路切换控制
						case ERR_ICMD_MEAPN:    Set_MEAPN()                ;break; //并行功耗测试控制
						case ERR_ICMD_CHKP:     Set_CHKP()                 ;break; //并行功耗各项测试数据查询
//误差单元接收命令(时钟校验仪部分命令)
      case ERR_ICMD_MBAUD:    Set_MTR_LCT_Com(MTRCOM)    ;break; //设置多功能表(模拟表)串口通信参数
      case ERR_ICMD_LBAUD:    Set_MTR_LCT_Com(LCTCOM)    ;break; //设置负控终端串口通信参数
      case ERR_ICMD_CLKFRQ:   Set_Clk_Freq_Pr()          ;break; //设置各表位时钟频率
      case ERR_ICMD_CLKTIM:   Set_Clk_Time_Pr()          ;break; //设置该表位时钟频率测量时间
      case ERR_ICMD_CLKCTL:   Set_Clk_Ctl_Pr()           ;break; //时钟频率测量控制
      case ERR_ICMD_XULCTL:   Set_XuL_Ctl_Pr()           ;break; //需量周期测量控制
      case ERR_ICMD_XULPLS:   Set_XuL_Pls_Num()          ;break; //设置需量周期测量个数
      case ERR_ICMD_EDIS:     Set_Disp_Mode()            ;break; //设置显示模式
      case ERR_ICMD_RLMDT:    Set_Reload_MtrD()          ;break; //重新装载模拟表数据     上位机   -> 模拟表   
      case ERR_ICMD_RLLDT:    Set_Reload_LctD()          ;break; //重新装载负控终端数据   上位机   -> 负控终端 
      case ERR_ICMD_RTMDT:    Set_ReTx_MtrD()            ;break; //重发收到的模拟表数据   模拟表   -> 上位机   
      case ERR_ICMD_RTLDT:    Set_ReTx_LctD()            ;break; //重发收到的负控终端数据 负控终端 -> 上位机   
      case ERR_ICMD_SEC:      Set_Time_Base()            ;break; //接收时间基准
      case ERR_ICMD_RED485:   Set_Red485()               ;break; //设置RS485 第二通道
      case ERR_ICMD_READ_VER: Set_Read_Ver()             ;break; //读版本号
      case ERR_ICMD_RSTS:     Set_RSTS()                 ;break; //读小显示工作状态
      case ERR_ICMD_SMTR:                                 break; //设置标准表
					 case ERR_ICMD_RBAUD:    Set_MTR_LCT_Com(RTECOM)    ;break; //设置RTECOM串口通信参数  
      case ERR_ICMD_ABAUD:    Set_MTR_LCT_Com(ATECOM)    ;break; //设置ATECOM串口通信参数
      case ERR_ICMD_IBAUD:    Set_MTR_LCT_Com(IRECOM)    ;break; //设置IRECOM串口通信参数
      	
//添加负控终端命令处理

//保留命令
      case JTAG_EN:           JTAG_EN_Pr()               ;break; //调试使能 JTAG 口开放      
      default:break;     
     }
    CAN_SDATA_MSG_ITail++;
    if(CAN_SDATA_MSG_ITail>=CAN_SDILEN)
     {
      CAN_SDATA_MSG_ITail=0;
      CAN_MSG_IPtr=&CAN_SDATA_MSG_IBUF[0];                       //初始化接收报文处理指针
     }                                    
    else                                  
     CAN_MSG_IPtr+=1;                                            //指针 

}
/*****************************************************************************
* 处理CAN1短数据接收指令缓冲区
*****************************************************************************/
void Proc_CNA1_SDATA_IBUF(void)
{
	   if(CAN1_SDATA_MSG_IHead==CAN1_SDATA_MSG_ITail)
     return;                                                     //没有收到短数据退出
    CAN1_RX_OVTimer=(u16)Timer_1ms;                               //初始化CAN超时定时器
                  
    switch(CAN1_MSG_IPtr->ID.BIT.CMD)                             //根据命令跳转到相应位置处理
				{
					  
					  default:break;
				}
				CAN1_SDATA_MSG_ITail++;
				if(CAN1_SDATA_MSG_ITail>=CAN_SDILEN)
				{
					  CAN1_SDATA_MSG_ITail=0;
					  CAN1_MSG_IPtr=&CAN1_SDATA_MSG_IBUF[0];                    //初始化接收报文处理指针
				}
				else
					CAN1_MSG_IPtr+=1;                                           //指针
}
/*****************************************************************************
* CAN短数据发送指令缓冲区头指针加1处理
*****************************************************************************/
void SDATA_MSG_OHead_ADD_ONE(void)
{
    CAN_SDATA_MSG_OHead++;
    if(CAN_SDATA_MSG_OHead>=CAN_SDOLEN)
     {
      CAN_SDATA_MSG_OHead=0;              //发送指针
      CAN_MSG_OPtr=&CAN_SDATA_MSG_OBUF[0];//初始化发送报文处理指针
     }
    else
     CAN_MSG_OPtr+=1;
}           
/*****************************************************************************
* 处理CAN短数据发送指令缓冲区
*****************************************************************************/
void Proc_SDATA_OBUF(void)
{
    if(CAN_SDATA_MSG_OHead==CAN_SDATA_MSG_OTail)
     return;                                  //没有短数据要发送退出
    if(CAN_SMSG_TX_STS != 0 )                 //CAN控制器是否在发送状态
     return;
    if(!MASTER_START)                         //总控中心是否启动
     return;	
    if(CAN_SEND_DELAY)                        //为防止过渡竞争 添加表位延时
     {
      CAN_SEND_DELAY--;
      return;
     }	
    CAN_SEND_DELAY=CAN_SEND_DTIME;            // 	
    CAN_SMSG_TX_STS = SLV_SMSG_TX_IS;
    CAN_STX_OVTimer=(u16)Timer_1ms;           //重置发送定时
    CAN_Tx_Msg(CAN0,&CAN_SDATA_MSG_OBUF[CAN_SDATA_MSG_OTail],MSG_OBJ_TYPE_TX);  //发送短帧
    CAN_SDATA_MSG_OTail++;                    //指针加1 指向下一帧 
    if(CAN_SDATA_MSG_OTail>=CAN_SDOLEN)       //判断是否超出循环   
     CAN_SDATA_MSG_OTail=0;                   //从头开始循环       
}
/*****************************************************************************
* 处理CAN1短数据发送指令缓冲区
*****************************************************************************/
void Proc_CAN1_SDATA_OBUF(void)
{
    if(CAN1_SDATA_MSG_OHead==CAN1_SDATA_MSG_OTail)
     return;                                  //没有短数据要发送退出
    if(CAN1_SMSG_TX_STS != 0 )                 //CAN控制器是否在发送状态
     return;
//    if(!MASTER_START)                         //总控中心是否启动
//     return;	
//    if(CAN_SEND_DELAY)                        //为防止过渡竞争 添加表位延时
//     {
//      CAN_SEND_DELAY--;
//      return;
//     }	
//    CAN_SEND_DELAY=CAN_SEND_DTIME;            // 	
    CAN1_SMSG_TX_STS = SLV_SMSG_TX_IS;
    CAN1_STX_OVTimer=(u16)Timer_1ms;           //重置发送定时
    CAN1_Tx_Msg(CAN1,&CAN1_SDATA_MSG_OBUF[CAN1_SDATA_MSG_OTail],MSG_OBJ_TYPE_TX);  //发送短帧
    CAN1_SDATA_MSG_OTail++;                    //指针加1 指向下一帧 
    if(CAN1_SDATA_MSG_OTail>=CAN_SDOLEN)       //判断是否超出循环   
     CAN1_SDATA_MSG_OTail=0;                   //从头开始循环       	
}
/*****************************************************************************
* 处理CAN总线状态
* 处理超时
*****************************************************************************/
void Proc_CAN_STS(void)        
{
    if((CAN_STS.BYTE&(CAN_STS_BOFF|CAN_STS_EWARN|CAN_STS_EPASS))||
      ((u16)(Timer_1ms-CAN_RX_OVTimer)>CAN_RX_OVTM)) 
     {
      CAN_RX_OVTimer=(u16)Timer_1ms;      //超时定时	
      CAN_STS.BYTE=0;                     //清除错误
      CAN_ERR=1;                          //CAN错误状态标志
      CAN_SMSG_TX_STS=COM_TX_NO;          //清除短数据发送标志      置为发送空闲状态
      CAN_LMSG_TX_STS=COM_TX_NO;          //清除CAN长数据帧发送状态 置为发送空闲状态
      CAN_LDATA_TX_STS = SLV_LDATA_TX_NO; //清除长数据发送状态      置为发送空闲状态         
      Init_CAN0();                         //重新初始化总线
     } 	
}
/*****************************************************************************
* 处理CAN1总线状态
* 处理超时
*****************************************************************************/
void Proc_CAN1_STS(void)        
{
    if((CAN1_STS.BYTE&(CAN_STS_BOFF|CAN_STS_EWARN|CAN_STS_EPASS))||
      ((u16)(Timer_1ms-CAN1_RX_OVTimer)>CAN_RX_OVTM)) 
     {
      CAN1_RX_OVTimer=(u16)Timer_1ms;      //超时定时	
      CAN1_STS.BYTE=0;                     //清除错误
      CAN1_ERR=1;                          //CAN错误状态标志
      CAN1_SMSG_TX_STS=COM_TX_NO;          //清除短数据发送标志      置为发送空闲状态
//      CAN_LMSG_TX_STS=COM_TX_NO;          //清除CAN长数据帧发送状态 置为发送空闲状态
//      CAN_LDATA_TX_STS = SLV_LDATA_TX_NO; //清除长数据发送状态      置为发送空闲状态         
      Init_CAN1();                         //重新初始化总线
     } 	
}

