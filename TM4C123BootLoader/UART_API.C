/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : UART.c
;* Author             : 张力阵
;* 串口数据处理程序
*******************************************************************************/
#include "Function.h"
#include "string.h"
#include "DISP.h"
//CAN 接口地址表格
const u32 UART_PORT[8]=
{
	UART0_BASE,
	UART1_BASE,
	UART2_BASE,
	UART3_BASE,
	UART4_BASE,
	UART5_BASE,
	UART6_BASE,
	UART7_BASE,
};
extern bool IntMasterDisable(void);
/*****************************************************************************
* 进入BOOT 关闭中断
*****************************************************************************/
void BOOT_INT_DIS(void)
{
    IntMasterDisable();          //进入BOOT前 禁能中断
    IntDisable(FAULT_SYSTICK);   //系统节拍发生器NVIC中断使能  系统定时器中断
    IntDisable(INT_UART0);       //UART0         NVIC中断使能  UART0中断
    IntDisable(INT_UART1);       //UART1         NVIC中断使能  UART1中断
    IntDisable(INT_TIMER0A);     //TIMER0-A      NVIC中断使能  电子脉冲计数中断
    IntDisable(INT_TIMER0B);     //TIMER0-B      NVIC中断使能  标准晶振高频计数溢出中断
    IntDisable(INT_TIMER1A);     //TIMER1-A      NVIC中断使能  时钟脉冲计数溢出中断
#ifndef __TEST
    IntDisable(INT_TIMER1B);     //TIMER1-B      NVIC中断使能  被检表高频计数溢出中断
#endif
    IntDisable(INT_TIMER2A);     //TIMER2-A      NVIC中断使能  标准表高频计数溢出中断 
//    IntDisable(INT_TIMER2B);   //TIMER2-B      NVIC中断使能  PWM 输出
    IntDisable(INT_CAN0);        //CAN           NVIC中断使能  CAN 中断
//    IntDisable(INT_WATCHDOG);    //看门狗        NVIC中断使能
    IntDisable(INT_GPIOB);       //GPIOB         NVIC中断使能  GPIOB中断 光电头脉冲输入
    IntDisable(INT_GPIOC);       //GPIOC         NVIC中断使能  GPIOC中断 按键输入
    IntDisable(INT_GPIOF);       //GPIOF         NVIC中断使能  GPIOF中断 需量脉冲 时段投切 合闸脉冲
//    IntMasterEnable();           //CPU中断允许
}
/*****************************************************************************
* COM接收缓冲区处理
*****************************************************************************/
void Proc_Com_IBuf(u8 COM)
{
    if(Com_Rx_Sts[COM]!=COM_RX_END)
     return;                                //没有数据要发送
    if(Com_IHead[COM]<4)                    //长度小于4 认为是干扰
     {
      Com_IHead[COM]=0;                     //清除接收指针                     
      Com_ITail[COM]=0;                     //清除接收缓冲区尾指针 处理指针
      Com_Rx_Sts[COM]=COM_RX_NO;            //接收状态 置为空闲状态 
      return;
     }		
    if(Com_IHead[COM]<7)                    //长度小于6 认为是干扰
     {
      if(COM==MTRCOM)                       //判断是否为备用接口
       {
        if(Com_IHead[MTRCOM]==6)            //长度为6认为
         {	
          u32 t1;
          u32 *Ptr;
          Ptr=(u32*)0x20000000;		  
          memcpy(&t1,Com0_IBuf,4);          //拷贝收到的数据
          if(t1==0x544F4F42)                //"BOOT+回送数据ID"
           {
            if(Com0_IBuf[5]=='U')					
             {							
              *Ptr     = 'U';                 //位变量地址分配0x20000000 
              *(Ptr+1) = 'U';                 //位变量地址分配0x20000004
						 }
            else
             {
              *Ptr     = 'C';                 //位变量地址分配0x20000000 
              *(Ptr+1) = 'C';                 //位变量地址分配0x20000004
						 }							
            *(Ptr+2) = Com0_IBuf[4];        //回送数据的ID  
            Com_Rx_Sts[COM]=0;	            //
            BOOT_INT_DIS();                 //BOOT 前关闭所有中断
            for(;;)	                        //死循环等待看门狗复位
             {}
           }
         }  		
       }
     }	
    if(CAN_LDATA_TX_STS==SLV_LDATA_TX_REQ)  //判断是否还在请求状态
     return;         
    if(CAN_LDATA_TX_STS==SLV_LDATA_TX_NO)   //空闲状态
     {                                      
      CAN_LMSG_TX_TYPE=COM;                 //长数据类型
      SLV_REQ_TX();                         //发送请求帧
      return;                               
     }                                      
    else if(CAN_LMSG_TX_TYPE!=COM)          //判断正在发送的是否为COM0数据
     return;                                //退出
    else if(CAN_LDATA_TX_STS==SLV_LDATA_TX_ACK)
     {                                      //请求得到批准 启动发送
      memcpy(&CAN_LMSG_TX_Ptr,
             &CAN_LMSG_TX_TAB[CAN_LMSG_TX_TYPE],
             16);                           //拷贝发送处理结构体
      memcpy(&CAN_LMSG_Tx,
             &CAN_MSG_SET_TAB[SLV_LDATA_TX-1],
             8);                            //导入控制域和仲裁域 MSG
      CAN_LMSG_Tx.ID.BIT.MNUM = Mtr_Numb_ID;//表位号
      CAN_LDATA_TX_STS = SLV_LDATA_TX_IS;   //正在发送长数据 状态
      *CAN_LMSG_TX_Ptr.TAIL=0;              //发送处理指针
     }
    CAN_LDATA_TX_Pr();                      //发送单帧 
}  
/*****************************************************************************
* COM发送缓冲区处理
* 人口:COM 串口编号
* 串口发送和接收FIFO都为16字节
* 串口发送缓冲区FIFO设置为<=1/8 中断 即发送缓冲区字节数不大于2字节时 申请发送中断
*****************************************************************************/
void Proc_Com_OBuf(u8 COM)
{
    u32 UARTx;
    u8 *Ptr;
    if(COM>=UART_NUMB)
     return;
    if(Com_Tx_Sts[COM]!=COM_TX_EN)    //检查COM发送缓冲区是否有新数据
     return;                          //没有新数据退出
    UARTx = UART_PORT[COM];
    if(COM==MTRCOM)
     Ptr= Com0_OBuf;
    else
     Ptr= Com1_OBuf;
    if(UARTBusy(UARTx))               //检查UART是否正在发送数据
     return;                          //发送缓冲区不空
    UARTIntDisable(UARTx,UART_INT_TX);//先屏蔽发送中断 以免发送时申请中断
    if(Com_OHead[COM]<12)             //判断发送缓冲区字节数据是否小于12
     {                                //小于12字节 不会申请发送缓冲区发送中断
      Com_Tx_Sts[COM] = COM_TX_NO;    //发送缓冲区空
      for(Com_OTail[COM]=0;Com_OTail[COM]<Com_OHead[COM];)	//数据小于12字节直接发送出去 不进入连续发送流程
       UARTCharPut(UARTx,*(Ptr+Com_OTail[COM]++));
     }
    else
     {                                //数据大于12字节 进入连续发送流程
      Com_Tx_Sts[COM] = COM_TX_IS;    //发送缓冲区正在发送数据标志
      for(Com_OTail[COM]=0;Com_OTail[COM]<12;)
       UARTCharPut(UARTx,*(Ptr+Com_OTail[COM]++));
     }
    TX_ZS_BIT=1;
    TX_MC_ON;	
    UARTIntEnable(UARTx,UART_INT_TX); //数据写入完毕打开发送中断 等待中断
}
/*****************************************************************************
* COM接收超时处理
* 人口:COM 串口编号
*****************************************************************************/
void Proc_Com_Rx_OvTm(u8 COM)
{
    if(Com_Rx_Sts[COM]!=COM_RX_IS)
     return;                             //串口不在接收状态退出
    if((u8)(Timer_8ms-Com_Rx_Time[COM])>Com_Rx_OvTm[COM])//判断UART接收是否超时
     {
      TX_MC_OFF;                         //通信指示灯 关	
      TX_ZS_BIT=0;                       //通信指示灯 标志
      Com_Rx_Time[COM]=Timer_8ms;        //清除定时器
      if(Com_IHead[COM]!=Com_ITail[COM]) //判断是否收到有效数据
       {	
        Com_Rx_Sts[COM]=COM_RX_END;      //接收结束状态 收到有效数据等待处理
       } 
      else
       Com_Rx_Sts[COM]=COM_RX_NO;        //在规定时间内未收到数据
     } 	 
}
