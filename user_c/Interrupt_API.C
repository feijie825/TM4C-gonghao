/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : Interrupt.c
;* Author             : 张力阵
;* 中断处理函数库
;* 要改变发送或接收FIFO触发设置 要更改相应程序 
*******************************************************************************/			   
#include "Function.h"
#include "CLK_ERR.h"
#include "ENG_ERR.h"
#include "string.h"
/********************************************************
* 错误中断服务程序
********************************************************/
void IntDefaultHandler(void)
{
    for(;;)
     {}
}

/********************************************************
* 看门狗中断服务程序
********************************************************/
void WatchDogHandler(void)
{
    WatchdogIntClear(WATCHDOG0_BASE);	  //清除看门狗中断
}
/********************************************************
* 系统节拍时钟定时器中断服务程序
********************************************************/
void SysTickIntHandler(void)
{
    Timer_1ms++;                           //1ms定时器
//    STD_CLK_Timer++;                       //检测标准时钟脉冲定时
    CLK_Timer++;                           //时钟脉冲定时
    if(WORK_Timer)                         //进入工作模式定时
     WORK_Timer--;                         //	
    if(Disp_En_Timer)                      //显示使能定时
     Disp_En_Timer--;                      //
    if(!(Timer_1ms%8))
				{
					 Timer_8ms++;
				}
    if(Com_Rx_Sts[MTRCOM]==COM_RX_IS)      //判断是否正在接收
     {    	                               
      if(UARTCharsAvail(UART0))            //判断接收缓冲区是否空
       Com_Rx_Time[MTRCOM]=Timer_8ms;      //COM0接收定时
     }  
    if(Com_Rx_Sts[LCTCOM]==COM_RX_IS)      //判断是否正在接收
     {	                                   
      if(UARTCharsAvail(UART1))            //判断接收缓冲区是否空
       Com_Rx_Time[LCTCOM]=Timer_8ms;      //COM1接收定时
     }
}
/********************************************************
* 采样序列发生器0中断服务程序
********************************************************/
void ADC0Handler(void)
{
//#ifdef _ADC
    ADCIntClear(ADC0_BASE,0);                        //清除中断
    ADCSequenceDataGet(ADC0_BASE,0,ADC_SEQ_Data);    //读值
    ADC_Data.Data=(u16)ADC_SEQ_Data[0];
    ADC_Data.New_Data=1;                             //新数据标志
    ADC_Data.Trig=0;                                 //ADC软件触发标志
//#endif
}
/*****************************************************************************
*
* 串口0中断服务程序
*
*****************************************************************************/
void UART0IntHandler(void)
{
    u32 ulStatus;
    u8 m;
    ulStatus = UARTIntStatus(UART0, true);
    UARTIntClear(UART0, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {                                     //发送FIFO中断
      if(Com_Tx_Sts[MTRCOM]!=COM_TX_IS)
       {
        TX_ZS_BIT=0;
        return;                            //发送缓冲区不在发送数据状态 退出
       }
      else
       {
        if(TX_ZS_BIT)                      //通信指示灯状态 
         {
          TX_ZS_BIT=0;
         }	
        else
         {
          TX_ZS_BIT=1;                     //
         }	 
        for(m=0;m<12;m++)                  //一次性写入12字节(降低发送中断次数 提高发送效率) 
         {                                 //发送FIFO深度16字节
          if(Com_OTail[MTRCOM]<Com_OHead[MTRCOM])
           UARTCharPut(UART0,Com0_OBuf[Com_OTail[MTRCOM]++]);
          else
           {                               //剩余字节数小于12字节时 认为发送完成
            Com_Tx_Sts[MTRCOM]=COM_TX_NO;  //发送完成 清除发送状态标志 退出循环
            TX_ZS_BIT=0;
            break;
           }  
         }
       } 
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {                                     //接收FIFO中断或接收超时中断
      if(TX_ZS_BIT)                        //通信指示灯状态 
       {
        TX_ZS_BIT=0;
       }	
      else
       {
        TX_ZS_BIT=1;                       //
       }	 
      if(Com_Rx_Sts[MTRCOM]==COM_RX_END)
       {
        for(;UARTCharsAvail(UART0);)
         UARTCharGet(UART0);               //清空接收缓冲区
        return;                            //上次接收的数据未处理 不接收新数据 退出
       }
      if(Com_Rx_Sts[MTRCOM]==COM_RX_NO)    //判断是否为新帧数据
       {                                   //新数据
        Com_Rx_Sts[MTRCOM]=COM_RX_IS;
        Com_IHead[MTRCOM] =0;              //重置接收指针
       }
      if(ulStatus&UART_MIS_RXMIS)
       m=12;
      else
       {
        m=16;
       } 
      for(;UARTCharsAvail(UART0);)
       {
        if(Com_IHead[MTRCOM]>=MTRCOM_ILEN)
         Com_IHead[MTRCOM]=0;
        Com0_IBuf[Com_IHead[MTRCOM]]=(u8)UARTCharGet(UART0);
        Com_IHead[MTRCOM]++;
        m--;
        if(m==0)
         break;
       }  
      Com_Rx_Time[MTRCOM]=Timer_8ms;              //重置接收定时 
     }
}
/*****************************************************************************
*
* 串口1中断服务程序
*
*****************************************************************************/
void UART1IntHandler(void)
{
    u32 ulStatus;
    u8 m;
    ulStatus = UARTIntStatus(UART1, true);
    UARTIntClear(UART1, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {       //发送FIFO中断
      if(Com_Tx_Sts[LCTCOM]!=COM_TX_IS)
       {
        TX_ZS_BIT=0;
        return;                            //发送缓冲区不在发送数据状态 退出
       }
      else
       {
        if(TX_ZS_BIT)                      //通信指示灯状态 
         {
          TX_ZS_BIT=0;
         }	
        else
         {
          TX_ZS_BIT=1;                     //
         }	 
        for(m=0;m<12;m++)    //一次性写入12字节(降低发送中断次数 提高发送效率) 
         {				  	 //发送FIFO深度16字节
          if(Com_OTail[LCTCOM]<Com_OHead[LCTCOM])
           UARTCharPut(UART1,Com1_OBuf[Com_OTail[LCTCOM]++]);
          else
           {				 //剩余字节数小于12字节时 认为发送完成
            Com_Tx_Sts[LCTCOM]=0;   //发送完成 清除发送状态标志 退出循环
            TX_ZS_BIT=0;
            break;
           }  
         }
       } 
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {	                     //接收FIFO中断或接收超时中断
      if(TX_ZS_BIT)                        //通信指示灯状态 
       {
        TX_ZS_BIT=0;
       }	
      else
       {
        TX_ZS_BIT=1;                       //
       }	 
      if(Com_Rx_Sts[LCTCOM]==COM_RX_END)
       {
        for(;UARTCharsAvail(UART1);)
         UARTCharGet(UART1);
        return;              //上次接收的数据未处理 不接收新数据 退出
       }
      if(Com_Rx_Sts[LCTCOM]==COM_RX_NO)//判断是否为新帧数据
       {                     //新数据
        Com_Rx_Sts[LCTCOM]=COM_RX_IS;
        Com_IHead[LCTCOM] =0;  //重置接收指针
       }
      if(ulStatus&UART_MIS_RXMIS)
       m=12;                 //接收中断 接收到14个字符触发中断 读取12个留2个产生超时中断
      else
       {
        m=16;                //超时中断
       } 
      for(;UARTCharsAvail(UART1);)
       {
        if(Com_IHead[LCTCOM]>=LCTCOM_ILEN)
         Com_IHead[LCTCOM]=0;
        Com1_IBuf[Com_IHead[LCTCOM]]=(u8)UARTCharGet(UART1);
        Com_IHead[LCTCOM]++;
        m--;
        if(m==0)
         break;
       }  
      Com_Rx_Time[LCTCOM]=Timer_8ms;              //重置接收定时 
     }
}
/*****************************************************************************
*
* 串口3中断服务程序
*
*****************************************************************************/
void UART3IntHandler(void)
{
    u32 ulStatus;
    u8 m;
    ulStatus = UARTIntStatus(UART3, true);
    UARTIntClear(UART3, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {                                     //发送FIFO中断
      if(Com_Tx_Sts[RTECOM]!=COM_TX_IS)
       {
        TX_ZS_BIT=0;
        return;                            //发送缓冲区不在发送数据状态 退出
       }
      else
       {
        if(TX_ZS_BIT)                      //通信指示灯状态 
         {
          TX_ZS_BIT=0;
         }	
        else
         {
          TX_ZS_BIT=1;                     //
         }	 
        for(m=0;m<12;m++)                  //一次性写入12字节(降低发送中断次数 提高发送效率) 
         {                                 //发送FIFO深度16字节
          if(Com_OTail[RTECOM]<Com_OHead[RTECOM])
           UARTCharPut(UART3,Com3_OBuf[Com_OTail[RTECOM]++]);
          else
           {                               //剩余字节数小于12字节时 认为发送完成
            Com_Tx_Sts[RTECOM]=COM_TX_NO;  //发送完成 清除发送状态标志 退出循环
            TX_ZS_BIT=0;
            break;
           }  
         }
       } 
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {                                     //接收FIFO中断或接收超时中断
      if(TX_ZS_BIT)                        //通信指示灯状态 
       {
        TX_ZS_BIT=0;
       }	
      else
       {
        TX_ZS_BIT=1;                       //
       }	 
      if(Com_Rx_Sts[RTECOM]==COM_RX_END)
       {
        for(;UARTCharsAvail(UART3);)
         UARTCharGet(UART3);               //清空接收缓冲区
        return;                            //上次接收的数据未处理 不接收新数据 退出
       }
      if(Com_Rx_Sts[RTECOM]==COM_RX_NO)    //判断是否为新帧数据
       {                                   //新数据
        Com_Rx_Sts[RTECOM]=COM_RX_IS;
        Com_IHead[RTECOM] =0;              //重置接收指针
       }
      if(ulStatus&UART_MIS_RXMIS)
       m=12;
      else
       {
        m=16;
       } 
      for(;UARTCharsAvail(UART3);)
       {
        if(Com_IHead[RTECOM]>=RTECOM_ILEN)
         Com_IHead[RTECOM]=0;
        Com3_IBuf[Com_IHead[RTECOM]]=(u8)UARTCharGet(UART3);
        Com_IHead[RTECOM]++;
        m--;
        if(m==0)
         break;
       }  
      Com_Rx_Time[RTECOM]=Timer_8ms;              //重置接收定时 
     }
}
/*****************************************************************************
*
* 串口5中断服务程序
*
*****************************************************************************/
void UART5IntHandler(void)
{
    u32 ulStatus;
    u8 m;
    ulStatus = UARTIntStatus(UART5, true);
    UARTIntClear(UART5, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {                                     //发送FIFO中断
      if(Com_Tx_Sts[ATECOM]!=COM_TX_IS)
       {
        TX_ZS_BIT=0;
        return;                            //发送缓冲区不在发送数据状态 退出
       }
      else
       {
        if(TX_ZS_BIT)                      //通信指示灯状态 
         {
          TX_ZS_BIT=0;
         }	
        else
         {
          TX_ZS_BIT=1;                     //
         }	 
        for(m=0;m<12;m++)                  //一次性写入12字节(降低发送中断次数 提高发送效率) 
         {                                 //发送FIFO深度16字节
          if(Com_OTail[ATECOM]<Com_OHead[ATECOM])
           UARTCharPut(UART5,Com5_OBuf[Com_OTail[ATECOM]++]);
          else
           {                               //剩余字节数小于12字节时 认为发送完成
            Com_Tx_Sts[ATECOM]=COM_TX_NO;  //发送完成 清除发送状态标志 退出循环
            TX_ZS_BIT=0;
            break;
           }  
         }
       } 
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {                                     //接收FIFO中断或接收超时中断
      if(TX_ZS_BIT)                        //通信指示灯状态 
       {
        TX_ZS_BIT=0;
       }	
      else
       {
        TX_ZS_BIT=1;                       //
       }	 
      if(Com_Rx_Sts[ATECOM]==COM_RX_END)
       {
        for(;UARTCharsAvail(UART5);)
         UARTCharGet(UART5);               //清空接收缓冲区
        return;                            //上次接收的数据未处理 不接收新数据 退出
       }
      if(Com_Rx_Sts[ATECOM]==COM_RX_NO)    //判断是否为新帧数据
       {                                   //新数据
        Com_Rx_Sts[ATECOM]=COM_RX_IS;
        Com_IHead[ATECOM] =0;              //重置接收指针
       }
      if(ulStatus&UART_MIS_RXMIS)
       m=12;
      else
       {
        m=16;
       } 
      for(;UARTCharsAvail(UART5);)
       {
        if(Com_IHead[ATECOM]>=ATECOM_ILEN)
         Com_IHead[ATECOM]=0;
        Com5_IBuf[Com_IHead[ATECOM]]=(u8)UARTCharGet(UART5);
        Com_IHead[ATECOM]++;
        m--;
        if(m==0)
         break;
       }  
      Com_Rx_Time[ATECOM]=Timer_8ms;              //重置接收定时 
     }
}
/*****************************************************************************
*
* 串口6中断服务程序
*
*****************************************************************************/
void UART6IntHandler(void)
{
    u32 ulStatus;
    u8 m;
    ulStatus = UARTIntStatus(UART6, true);
    UARTIntClear(UART6, ulStatus);
    if((ulStatus&UART_MIS_TXMIS))
     {                                     //发送FIFO中断
      if(Com_Tx_Sts[IRECOM]!=COM_TX_IS)
       {
        TX_ZS_BIT=0;
        return;                            //发送缓冲区不在发送数据状态 退出
       }
      else
       {
        if(TX_ZS_BIT)                      //通信指示灯状态 
         {
          TX_ZS_BIT=0;
         }	
        else
         {
          TX_ZS_BIT=1;                     //
         }	 
        for(m=0;m<12;m++)                  //一次性写入12字节(降低发送中断次数 提高发送效率) 
         {                                 //发送FIFO深度16字节
          if(Com_OTail[IRECOM]<Com_OHead[IRECOM])
           UARTCharPut(UART6,Com6_OBuf[Com_OTail[IRECOM]++]);
          else
           {                               //剩余字节数小于12字节时 认为发送完成
            Com_Tx_Sts[IRECOM]=COM_TX_NO;  //发送完成 清除发送状态标志 退出循环
            TX_ZS_BIT=0;
            break;
           }  
         }
       } 
     }
    if(ulStatus&(UART_MIS_RXMIS|UART_MIS_RTMIS))
     {                                     //接收FIFO中断或接收超时中断
      if(TX_ZS_BIT)                        //通信指示灯状态 
       {
        TX_ZS_BIT=0;
       }	
      else
       {
        TX_ZS_BIT=1;                       //
       }	 
      if(Com_Rx_Sts[IRECOM]==COM_RX_END)
       {
        for(;UARTCharsAvail(UART6);)
         UARTCharGet(UART6);               //清空接收缓冲区
        return;                            //上次接收的数据未处理 不接收新数据 退出
       }
      if(Com_Rx_Sts[IRECOM]==COM_RX_NO)    //判断是否为新帧数据
       {                                   //新数据
        Com_Rx_Sts[IRECOM]=COM_RX_IS;
        Com_IHead[IRECOM] =0;              //重置接收指针
       }
      if(ulStatus&UART_MIS_RXMIS)
       m=12;
      else
       {
        m=16;
       } 
      for(;UARTCharsAvail(UART6);)
       {
        if(Com_IHead[IRECOM]>=IRECOM_ILEN)
         Com_IHead[IRECOM]=0;
        Com6_IBuf[Com_IHead[IRECOM]]=(u8)UARTCharGet(UART6);
        Com_IHead[IRECOM]++;
        m--;
        if(m==0)
         break;
       }  
      Com_Rx_Time[IRECOM]=Timer_8ms;              //重置接收定时 
     }
}
/*****************************************************************************
* 电子脉冲中断处理
* 入口:TIMERx  计数器
* 入口:ulTimer 部件 TIMER_A TIMER_B
*****************************************************************************/
void DZPLS_Interrupt_Pr(u32 TIMERx,u32 ulTimer)
{	
    u16  m;                           //保存计数器值
    u32  Cnt;                         //保存电能脉冲临时计数值
    m=(u16)TimerValueGet(TIMER1_BASE, //读取标准表脉冲计数低位
                         TIMER_B);    //读取当前计数值 计数器减计数 计数初值0xFFFF
    Cnt=STD_ENG_Cnt;                  //读标准表脉冲计数高位
    DZ_NEW_PLS=1;	                    //收到电子脉冲标志
    if(WORK_MODE==MEA_ENG_DUTY_M)     //判断是否在测量脉冲周期状态
     return;                          //退出
    if(WORK_MODE==PANZHUAN_ERR_M)     //判断是否在盘转误差试验
     {
      TimerLoadSet(TIMERx,            //定时器
                   ulTimer,           //通道
     	             1);
      ELEC_PLS_CNT++;
      return;
     } 
    else                              //误差计算试验
     TimerLoadSet(TIMERx,             //定时器
                  ulTimer,            //通道
                  DIVIDE_Coef);
    TimerEnable(TIMERx,               //定时器
                ulTimer);             //定时器启动
    if(ENG_CLK_CH==DZ_PLS)            //判断是否选择电子脉冲输入
     {                                //电子脉冲输入
      m=~m;
      Cnt=(Cnt<<16)+m-Cnt;
      CUR_ENG_VAL=Cnt;                //当前电能脉冲计数值
      NEW_ENG_PLS=1;                  //收到新脉冲标志 
     }
}     
/*****************************************************************************
* 时钟脉冲中断处理
*****************************************************************************/
void SZPLS_Interrupt_Pr(u32 TIMERx,u32 ulTimer)
{	
    u16  m;                                         //保存计数器值
    u32  Cnt;                                       //保存标准时钟脉冲临时计数值
    m=(u16)TimerValueGet(TIMER2_BASE,               //读标准时钟脉冲计数低位
                         TIMER_A);                  //读取当前计数值 计数器减计数 计数初值0xFFFF
    TimerLoadSet(TIMERx,                            //定时器
                 ulTimer,                           //通道
                 CLK_RELOAD_VAL_N);
    TimerEnable(TIMERx,                             //定时器
                ulTimer);                           //定时器启动
    m=~m;
    Cnt=STD_CLK_Cnt;                                //读标准时钟脉冲计数高位
    Cnt=(Cnt<<16)+m-Cnt;  
    CUR_CLK_VAL=Cnt;                                //当前标准时钟脉冲计数值
    NO_CLK_PLS=0;                                   //收到时钟脉冲
    SZ_NEW_PLS=1;                                   //收到时钟脉冲标志
    if(MFClk_Mode==UNION_PLS)                       //判断多功能脉冲是否为联合脉冲
     if(MFClk_Type!=SZCLK_PLS)                      //判断是否为测量时钟脉冲周期
      {
       TimerDisable(TIMERx,
                    ulTimer); 
       return;
      } 
    if(CLK_Timer<CLK_Timer_Min)                     //中断时间太短 频率比设定值高
     {                                              //去时钟处理程序调整时钟频率规格化值
      if(SZCLK_SET_TooM_T)                          //判断是否已经检测到中断过频
       SZCLK_SET_TooM=1;                            //时钟频率设置太小标志 去处理
      else
       {	
        CLK_Timer=0;                                //清除时钟脉冲超时定时器
      	 SZCLK_SET_TooM_T=1;                         //首次检测到过频 保证可靠
      	 NEW_CLK_PLS=0;
      	 return;
       }	
       TimerDisable(TIMERx,                         //Timer1-A 停止计数
                    ulTimer); 
     }
    CLK_Timer=0;                                    //清除时钟脉冲超时定时器
    NEW_CLK_PLS=1;                                  //收到新脉冲标志   
}
/*****************************************************************************
* Timer0-A中断服务程序
* 备用高频信号1计数中断
*****************************************************************************/
void Timer0AIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER0_BASE,
	                             TIMER_CAPA_MATCH);    //
    TimerIntClear(TIMER0_BASE,ulstatus);	           //清溢出中断
    TimerLoadSet(TIMER0_BASE,TIMER_A,0xFFFF);       //重置计数 0xFFFF
    TimerEnable(TIMER0_BASE,TIMER_A);               //Timer0-A启动
}
/*****************************************************************************
* Timer0-B中断服务程序
* 备用高频信号2计数中断
*****************************************************************************/
void Timer0BIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER0_BASE,
	                             TIMER_CAPB_MATCH);    //
    TimerIntClear(TIMER0_BASE,ulstatus);	           //清溢出中断
    TimerLoadSet(TIMER0_BASE,TIMER_B,0xFFFF);       //重置计数 0xFFFF
    TimerEnable(TIMER0_BASE,TIMER_B);               //Timer0-B启动
}
/*****************************************************************************
* Timer1-A中断服务程序
* 备用高频信号3计数中断
*****************************************************************************/
void Timer1AIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER1_BASE,
	                             TIMER_CAPA_MATCH);    //
    TimerIntClear(TIMER1_BASE,ulstatus);	           //清溢出中断
    TimerLoadSet(TIMER1_BASE,TIMER_A,0xFFFF);       //重置计数 0xFFFF
    TimerEnable(TIMER1_BASE,TIMER_A);               //Timer1-A启动
}
/*****************************************************************************
* Timer1-B中断服务程序
* 标准表有功高频计数溢出中断
* 30K 中断时间为 2.1845s
* 10K 中断时间为 6.5535s
*****************************************************************************/
void Timer1BIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER1_BASE,
	                             TIMER_CAPB_MATCH);    //
    TimerIntClear(TIMER1_BASE,ulstatus);	           //清溢出中断
    TimerLoadSet(TIMER1_BASE,TIMER_B,0xFFFF);       //重置计数 0xFFFF
    TimerEnable(TIMER1_BASE,TIMER_B);               //Timer1-B启动
    STD_ENG_Cnt++;                                  //标准电能脉冲高位   
    NO_STD_ENG=0;                                   //有标准脉冲
    STD_ECLK_Timer=(u16)Timer_1ms;                  //重置定时器
}
/*****************************************************************************
* Timer2-A中断服务程序
* 用于标准晶振高频计数 正常131.07ms中断一次
* 计数分频值0xFFFF 
* 当前计数值=STD_CLK_Cnt*0xFFFF+(0xFFFF-TIMER0B)
*           =((STD_CLK_Cnt+1)*0xFFFF)-TIMER0B
*           =(STD_CLK_Cnt+1)*(0x10000-1)-TIMER0B
*           =(STD_CLK_Cnt+1)*0x10000-(STD_CLK_Cnt+1)-TIMER0B 
*           =(STD_CLK_Cnt+1)*0x10000-STD_CLK_Cnt-TIMER0B-1
*           =(STD_CLK_Cnt+1)*0x10000-STD_CLK_Cnt+(~TIMER0B+1)-1
*           =(STD_CLK_Cnt<<16)-STD_CLK_Cnt+0x10000+(~TIMER0B)
* 连续两次计数差值 第一次计数高位 STD_CLK_Cnt1 TIMER0B1
*                  第二次计数高位 STD_CLK_Cnt2 TIMER0B2
*                  计算差值时 常量0x10000 消掉
* 差值=
*****************************************************************************/
void Timer2AIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER2_BASE,
	                             TIMER_CAPA_MATCH);    //
    TimerIntClear(TIMER2_BASE,ulstatus);	           //捕捉匹配中断
    TimerLoadSet(TIMER2_BASE,TIMER_A,0xFFFF);       //重置计数 0xFFFF
	   TimerEnable(TIMER2_BASE,TIMER_A);               //Timer0-B启动
    STD_CLK_Cnt++;                                  //标准晶振时钟脉冲高位
    NO_STD_CLK=0;                                   //标准时钟脉冲存在
    STD_CLK_Timer=Timer_8ms;                        //重启检测存在定时
    NEW_PLL_Ref_Timer=(u16)Timer_1ms;               //锁相环参考定时器
    REF_JZ_INT=1;                                   //参考时钟中断标志	
}

/*****************************************************************************
* 定时器2-B中断服务程序
* 标准表无功高频计数溢出中断
*****************************************************************************/
void Timer2BIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER2_BASE,
	                             TIMER_CAPB_MATCH);    //
    TimerIntClear(TIMER2_BASE,ulstatus);	           //清溢出中断
    TimerLoadSet(TIMER2_BASE,TIMER_B,0xFFFF);       //重置计数 0xFFFF
    TimerEnable(TIMER2_BASE,TIMER_B);               //Timer2-A启动
}

/*****************************************************************************
* Timer3-A中断服务程序
* 被检表反向无功电能脉冲计数中断
*****************************************************************************/
void Timer3AIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER3_BASE,
	                             TIMER_CAPA_MATCH);
    TimerIntClear(TIMER3_BASE,ulstatus);		
    TimerLoadSet(TIMER3_BASE,                       //定时器
                 TIMER_A,                           //通道
                 1);                                
    TimerEnable(TIMER3_BASE,                        //定时器
                TIMER_A);                           //定时器启动	
	   if(PLSHC_MODE==HC_2)
     return;
				if((PLSHC_MODE==HC_3)||
					  (PLSHC_MODE==HC_4))
				{
					 if(PLS_QUAD==QR_PLS)
						{
						  DZPLS_Interrupt_Pr(TIMER3_BASE,TIMER_A);    //按电子脉冲处理
						}
				}
}
/*****************************************************************************
* Timer3-B中断服务程序
* 被检表反向有功电能脉冲计数中断
*****************************************************************************/
void Timer3BIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER3_BASE,
	                             TIMER_CAPB_MATCH);
    TimerIntClear(TIMER3_BASE,ulstatus);		
    TimerLoadSet(TIMER3_BASE,                       //定时器
                 TIMER_B,                           //通道
                 1);                                
    TimerEnable(TIMER3_BASE,                        //定时器
                TIMER_B);                           //定时器启动		
			 if((PLSHC_MODE==HC_4)&&                         //被检表四路脉冲分开
					  (PLS_QUAD==PR_PLS))                          //并且脉冲象限为反向有功
				{
      DZPLS_Interrupt_Pr(TIMER3_BASE,TIMER_B);      //按电子脉冲处理	
				}
}
/*****************************************************************************
* Timer4-A中断服务程序
* 被检表正向无功电能脉冲计数中断
*****************************************************************************/
void Timer4AIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER4_BASE,
	                             TIMER_CAPA_MATCH);
    TimerIntClear(TIMER4_BASE,ulstatus);		
    TimerLoadSet(TIMER4_BASE,                       //定时器
                 TIMER_A,                           //通道
                 1);                                
    TimerEnable(TIMER4_BASE,                        //定时器
                TIMER_A);                           //定时器启动	    
	   if(PLSHC_MODE==HC_2)                            //被检表为两路脉冲
				{
					 if((PLS_QUAD==QA_PLS)||                       //被检表脉冲象限为正向无功或反向无功
							  (PLS_QUAD==QR_PLS))                        
						{                                             
							 DZPLS_Interrupt_Pr(TIMER4_BASE,TIMER_A);    //按电子脉冲处理
						}                                             
				}                                               
				if((PLSHC_MODE==HC_3)||                         //被检表为三路脉冲
					  (PLSHC_MODE==HC_4))                          //或者为四路脉冲
				{                                               
					 if(PLS_QUAD==QA_PLS)                          //被检表脉冲象限为正向无功
						{                                             
							 DZPLS_Interrupt_Pr(TIMER4_BASE,TIMER_A);    //按电子脉冲处理
						}
				}
}
/*****************************************************************************
* Timer4-B中断服务程序
* 默认被检表时钟脉冲计数溢出中断
* 南网表时为被检表有功电能脉冲计数溢出中断
*****************************************************************************/
void Timer4BIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER4_BASE,
	                             TIMER_CAPB_MATCH);
    TimerIntClear(TIMER4_BASE,ulstatus);	           //清溢出中断
    TimerLoadSet(TIMER4_BASE,                       //定时器
                 TIMER_B,                           //通道
                 1);                                
    TimerEnable(TIMER4_BASE,                        //定时器
                TIMER_B);                           //定时器启动	    
    if(MTYPE==SOUTH)                                //判断是否为南网表
			 {
					 if((PLSHC_MODE==HC_2)||
         (PLSHC_MODE==HC_3))	                  					//被检表为两路脉冲或者三路脉冲
						{
							 if((PLS_QUAD==PA_PLS)||
									  (PLS_QUAD==PR_PLS))
								 DZPLS_Interrupt_Pr(TIMER4_BASE,TIMER_B);   //按电子脉冲处理
						}
					 if(PLSHC_MODE==HC_4)                          //
						{
							 if(PLS_QUAD==PA_PLS)
								 DZPLS_Interrupt_Pr(TIMER4_BASE,TIMER_B);   //按电子脉冲处理
						}							
				}
    else                                                   
				{
      SZPLS_Interrupt_Pr(TIMER4_BASE,TIMER_B);      //按时钟脉冲处理
				}
}
/*****************************************************************************
* Timer5-A中断服务程序
* 默认被检表有功电能脉冲计数中断
* 南网表时为被检表时钟脉冲计数中断
*****************************************************************************/
void Timer5AIntHandler(void)
{
	   u32 ulstatus;
	   ulstatus = TimerIntStatus(TIMER5_BASE,
	                             TIMER_CAPA_MATCH);
    TimerIntClear(TIMER5_BASE,ulstatus);	
    TimerLoadSet(TIMER5_BASE,                        //定时器
                 TIMER_A,                            //通道
                 1);                                 
    TimerEnable(TIMER5_BASE,                         //定时器
                TIMER_A);                            //定时器启动    
    if(MTYPE==SOUTH)                                 //判断是否为南网表
				{
      SZPLS_Interrupt_Pr(TIMER5_BASE,TIMER_A);       //按时钟脉冲处理
				}
    else                                           
				{						
					 if((PLSHC_MODE==HC_2)||
         (PLSHC_MODE==HC_3))	                  						//被检表为两路脉冲或者三路脉冲
						{
							 if((PLS_QUAD==PA_PLS)||
									  (PLS_QUAD==PR_PLS))
								DZPLS_Interrupt_Pr(TIMER5_BASE,TIMER_A);     //按电子脉冲处理	
						}
					 if(PLSHC_MODE==HC_4)                           //
						{
							 if(PLS_QUAD==PA_PLS)
								DZPLS_Interrupt_Pr(TIMER5_BASE,TIMER_A);     //按电子脉冲处理							
						}
				}
}
/*****************************************************************************
* 测量周期处理
* 入口 PORT 端口 PORTL
* 入口 Int  中断值
*****************************************************************************/
void Mea_Period_Pr(u8 PORT,u8 Int)
{
    u32 GPIOx;
    vu8_Ptr Timer;
    u32_Ptr FLAG;
    u8 Pin;
    GPIOx=PORT_BASE_ADDR_TAB[PORT];                //端口
    if(CYCLE_MEA_ID)                               //是否不是测量电能脉冲周期
				{
      Pin=CYCLE_MEA_ID+1;
				}
    else 
				{
					 if(ENG_CLK_CH==GDT_PLS)                      //测量电能脉冲周期 判断是否为光电头脉冲
       Pin=(GDT_PLS-GDT_PLS);                      //光电头脉冲
      else
       Pin=(DZ_PLS-GDT_PLS);                       //电子脉冲
				}
    Timer=IO_Timer_Tab[Pin];                       //管脚重开定时器            
    FLAG=IO_REEN_TAB[Pin];                         //管脚重开中断标志
    if(MTYPE==SOUTH)                               //判断是否为南网表
     {                                             //南网表处理
     	if(Pin==(DZ_PLS-GDT_PLS))                    //是否选择电子脉冲
     	 Pin=(SZ_PLS-GDT_PLS);                       //管脚换到时钟脉冲上	
     	else if(Pin==(SZ_PLS-GDT_PLS))               //是否选择时钟脉冲 
     	 Pin=(DZ_PLS-GDT_PLS);                       //管脚换到电子脉冲上			
     }
    PORT=CYCLE_PIN_TAB[Pin];                       //对应管脚   
    if(Int&PORT)                                   //是否为该管脚中断
     {
      GPIOIntDisable(GPIOx,                        //端口 设置管脚中断禁能
                     PORT);	                       //管脚	
      *(Timer.Ptr)=Timer_8ms;   
      *(FLAG.Ptr)=1;                               //重使能标志
      if(FIRST_ENG_PLS)                            //判断是否为第一个脉冲
       {                                           
        FIRST_ENG_PLS=0;                           //清除第一个脉冲标志
        CYCLE_OLD_Timer=Timer_1ms;                 //记录当前时间
        RISE_FALL_LVL=0;                           //本次中断为上升沿
        GPIOIntTypeSet(GPIOx,                      //端口
                       PORT,                       //管脚
                       GPIO_FALLING_EDGE);         //下次中断改为下降沿         
       }                                             
      else                                           
       {	                                           
        CYCLE_NEW_Timer=Timer_1ms;                 //更新当前时间
        NEW_ENG_PLS=1;                             //置位有新脉冲标志
        if(RISE_FALL_LVL)                          //上次中断为下降沿
         {                                         
          RISE_FALL_LVL=0;                         //本次中断为上升沿
          GPIOIntTypeSet(GPIOx,                    //端口
                         PORT,                     //管脚
                         GPIO_FALLING_EDGE);       //下次中断改为下降沿         
         }                                           
        else                                         
         {                                           
          RISE_FALL_LVL=1;                         //本次中断为下将沿
          GPIOIntTypeSet(GPIOx,                    //端口
                         PORT,                     //管脚
                         GPIO_RISING_EDGE);        //下次中断改为上升沿中断         
         }
       }   
     }		
}
/*****************************************************************************
* GPIOA 端口 中断处理函数
* 按键中断
*****************************************************************************/
void GPIOAHandler(void)
{
    u8 Int;
    Int=GPIOIntStatus(KEY_IN_GPIO,
	                    TRUE);           //读取I/O口中断状态
    GPIOIntClear(KEY_IN_GPIO,         //端口
                 Int);	
    if(Int&KEY_IN)                    //是否为按键中断
     {
      GPIOIntDisable(KEY_IN_GPIO,     //端口 设置管脚中断禁能
                     KEY_IN);	        //管脚	
      KEY_Timer=Timer_8ms;            //重启中断使能定时
      KEY_INT_REEN=1;                 //置位重使能标志 
      if(NEW_KEY_FLAG)                //上次按键没有处理 认为是连续按键 挂表or解除挂表
       {	
        NEW_KEY_FLAG=0;               //收到按键中断标志
        PLUG_CHG_FLAG=1;	
       }
      else
       {	
        NEW_KEY_FLAG=1;               //收到按键中断标志
        PLUG_CHG_FLAG=0;	
       }
      Con_KEY_Timer=Timer_8ms;        //连续按键定时
     }
}

/*****************************************************************************
* GPIOM 端口 中断处理函数
* 光电头脉冲输入中断
* 电子脉冲中断(测量电子脉冲周期) 
* 时钟脉冲中断输入(测量时钟脉冲周期)
* 需量脉冲输入中断
* 时段投切脉冲
*****************************************************************************/
void GPIOMHandler(void)
{
    u16 m,n;                               //保存计数器值
    u32 BENG_H;                            //标准电能计数高位
    u32 BCLK_H;                            //标准晶振计数高位
    u8  Int,ucPins;                        //中断状态
    n=(u16)TimerValueGet(TIMER1_BASE,      
                         TIMER_B);         //读取标准电能当前计数值 计数器减计数 计数初值0xFFFF
    BENG_H=STD_ENG_Cnt;                    //读标准表脉冲计数高位
    m=(u16)TimerValueGet(TIMER2_BASE,      
                         TIMER_A);         //读取标准晶振当前计数值 计数器减计数 计数初值0xFFFF
    BCLK_H=STD_CLK_Cnt;                    //标准晶振计数高位    
    Int=GPIOIntStatus(GDT_MC_GPIO,         
	                     TRUE);               //读取I/O口中断状态
    GPIOIntClear(GDT_MC_GPIO,              //端口
                 Int);	                    //清I/O口中断状态
	#ifdef PULSE                              //脉冲检测程序 不编译该段
    if(MTYPE==SOUTH)                       //判断是否为南网表
     ucPins=CYCLE_PIN_TAB[SZ_PLS-GDT_PLS]; //选择时钟脉冲输入口作为 电子脉冲输入 南网表
    else         
     ucPins=CYCLE_PIN_TAB[DZ_PLS-GDT_PLS]; //选择电子脉冲输入口作为 电子脉冲输入 国网表
    if(Int&ucPins)                         //判断是否为电子脉冲中断 	 		
     {                                     
      GPIOIntDisable(GDT_MC_GPIO,          //端口 设置管脚中断禁能
                     ucPins);	             //管脚	
      DZ_Timer=Timer_8ms;                  //
      DZ_PLS_Timer=(u8)Timer_1ms;          //电子脉冲自动消抖延时
      DZ_INT_REEN=1;                       //重使能定时 
      DZ_NEW_PLS=1;                        //收到电子脉冲标志
     }                                     
#else
    if(WORK_MODE==MEA_ENG_DUTY_M)          //判断是否在测量脉冲周期状态
     {                                     
      Mea_Period_Pr(PORTM,Int);            //脉冲周期测量处理
      return;                              
     }                                     
#endif						                               
    if(Int&GDT_MC)                         //判断是否为光电头中断 	 		
     {                                     
      GPIOIntDisable(GDT_MC_GPIO,          //端口 设置管脚中断禁能
                     GDT_MC);	             //管脚	
      GDT_Timer=Timer_8ms;                 //
      GDT_PLS_Timer=(u8)Timer_1ms;         //光电头脉冲自动消抖延时
      GDT_INT_REEN=1;                      //重使能定时 
      GD_NEW_PLS=1;                        //光电头脉冲中断
      if(WORK_MODE==PANZHUAN_ERR_M)        //判断是否在盘转误差试验状态
       {                                   
       	CUR_PZ_Cnt_Val=ELEC_PLS_CNT;       //采样点电子脉冲计数值
        NEW_ENG_PLS=1;                     //收到新脉冲标志
       }                                   
      else if(ENG_CLK_CH==GDT_PLS)         //判断脉冲选择是否为光电头脉冲
       {                                   
        n=~n;                              
        BENG_H=(BENG_H<<16)+n-BENG_H;      
        CUR_ENG_VAL=BENG_H;                //当前电能脉冲计数值
        NEW_ENG_PLS=1;                     //收到新脉冲标志 
       } 
     } 
    if(Int&XL_MC)                           
     {                                     //需量脉冲中断
      GPIOIntDisable(XL_MC_GPIO,           //端口 设置管脚中断禁能
                     XL_MC);	              //管脚	
      XUL_Timer=Timer_8ms;                 //重启中断使能定时
      XUL_INT_REEN=1;                      //清除重使能标志 
      if(MFClk_Mode!=UNION_PLS)            //判断多功能脉冲是否为联合脉冲
       {                                   
        m=~m;                              
        BCLK_H=(BCLK_H<<16)+m-BCLK_H;      //
        CUR_XUL_VAL=BCLK_H;                    
        NEW_XUL_PLS=1;                     //收到新脉冲标志          	
       }                                       
     }                                         
    if(Int&TQ_MC)                          //判断是否为时段投切脉冲
     {                                     //联合脉冲状态下 该口禁能   
      GPIOIntDisable(TQ_MC_GPIO,           //端口 设置管脚中断禁能
                     TQ_MC);	              //管脚	
      TQ_Timer=Timer_8ms;                  //投切口中断重启定时器
      TQ_INT_REEN=1;                       //清除重使能标志 
      if(MFClk_Mode!=UNION_PLS)            //判断多功能脉冲是否为联合脉冲
       {                                    
       	NEW_TQ_PLS=1;
       }                                    
     }	   
					
    if(MTYPE==SOUTH)                       //判断是否为南网表
     ucPins=CYCLE_PIN_TAB[DZ_PLS-GDT_PLS]; //选择电子脉冲输入口作为 时钟脉冲输入 南网表
    else         
     ucPins=CYCLE_PIN_TAB[SZ_PLS-GDT_PLS]; //选择时钟脉冲输入口作为 时钟脉冲输入 国网表
    if(Int&ucPins)                         //时钟脉冲管脚中断
     {                                     
      GPIOIntDisable(GPIOM,                //端口 设置管脚中断禁能
                     ucPins);	             //管脚	
      SZ_Timer=Timer_8ms;                  //重启中断使能定时
      SZ_PLS_Timer=(u8)Timer_1ms;          //时钟脉冲自动消抖延时
      SZ_INT_REEN=1;                       //清除重使能标志  
      SZ_NEW_PLS=1;                        //收到时钟脉冲标志
      if(MFClk_Mode==UNION_PLS)            //判断是否为联合脉冲
       {                                   
        if(MFClk_Type==XULCLK_PLS)         //多功能脉冲输入类型 需量脉冲
         {                                 
          m=~m;                            
          BCLK_H=(BCLK_H<<16)+m-BCLK_H;    //
          CUR_XUL_VAL=BCLK_H;
          NEW_XUL_PLS=1;                   //收到新脉冲标志          	
         }                                 
        else if(MFClk_Type==TQCLK_PLS)     //多功能脉冲输入类型 时段投切脉冲
         NEW_TQ_PLS=1;                     //收到投切信号标志         
       }
     }					
}
/*****************************************************************************
* GPION 端口 中断处理函数
* 续流保护继电器状态输入引脚中断
*****************************************************************************/
void GPIONHandler(void)
{
	   u8  Int;                               //中断状态
    Int=GPIOIntStatus(OPEN_IN_GPIO,         
	                     TRUE);               //读取I/O口中断状态
    GPIOIntClear(OPEN_IN_GPIO,             //端口
                 Int);	                    //清I/O口中断状态	
    if(Int&OPEN_IN)                        //判断是否为开路信号中断 	 		
     {                                     
      NZTZ_PLS_Timer=(u8)Timer_1ms;        //开路脉冲信号自动消抖延时
						NZTZ_NEW_PLS=1;                      //是否收到开路脉冲信号标志
     }	
}
/*****************************************************************************
* CAN0 中断处理函数
*****************************************************************************/
void CAN0Handler(void)
{
    u32 ulStatus;
    ulStatus = CANIntStatus(CAN0, CAN_INT_STS_CAUSE);  //读引起中断的报文对象的编号
    if(ulStatus==0)                                    //误中断
     return;                                           //返回
    if(ulStatus==0x8000)                               //判断是否为状态中断
     {                                                 //状态中断
      CAN_STS.BYTE=CANStatusGet(CAN0, CAN_STS_CONTROL);//读总线错误状态
      CANErrCntrGet(CAN0,&CANERR_RxCNT,&CANERR_TxCNT);
      return;
     }                                                 
    if(ulStatus>MSG_OBJ_NUMB)                          
     {                                                 //误中断
      CANStatusGet(CAN0, CAN_STS_CONTROL);             //
      return;                                          //超出报文对象定义个数 认为为错误接收或发送
     }
    CAN_ERR=0;                                         //总线连接 
    CAN_STS.BYTE =0;                                   //正确接收或发送 清除状态中断
    CANERR_RxCNT=0;                                    //接收错误计数清零 
    CANERR_TxCNT=0;                                    //发送错误计数清零 
    if(CAN_MSG_SET_TAB[(ulStatus-1)].CTL.BIT.RX_INT_EN)//判断是否为接收报文中断
     {                                                 //接收中断
      memset(&CAN_MSG_Rx,0,8);                         //清除接收报文控制域和仲裁域
      CAN_MSG_Rx.CTL.BIT.IDx=ulStatus;                 //报文对象编号
      CAN_Rx_Msg(CAN0, &CAN_MSG_Rx, 1);                //接收帧 并清除中断
      switch(ulStatus)                                 //0x8000状态中断  0x01---0x20 引起中断的报文对象
       {	                                             
        case MST_CHK_BCAST:                             //BroadCast   接收中断  ID1
        case MST_CHK_SCAST:                             //Single Cast 接收中断  ID2
         {                                              //主机广播查询中断
          Echo_Sts = MST_CHK_RCVD;                      //收到查询标志 CHECK
          MASTER_START=1;                               //主机已启动
          break;                                        
         }                                              
        case MST_SCDATA_TX:                             //主机正在发送短广播数据串ID8
        case MST_SSDATA_TX:                             //主机正在发送短单播数据串ID9
         {                                              //短数据接收处理 接收中断
          if((CAN_SDATA_MSG_IHead+1)!=CAN_SDATA_MSG_ITail)//判断是否有空闲空间
           {
            memcpy(&CAN_SDATA_MSG_IBUF[CAN_SDATA_MSG_IHead],
                   &CAN_MSG_Rx,16);                    //拷贝数据
            CAN_SDATA_MSG_IHead++;                     //指针加1 指向下一帧
            if(CAN_SDATA_MSG_IHead>=CAN_SDILEN)        //判断是否超出循环
             CAN_SDATA_MSG_IHead=0;                    //从头开始循环 
           }
           break;
         }
        default:
         {
          if((CAN_LDATA_MSG_IHead+1)!=CAN_LDATA_MSG_ITail)//判断是否有空闲空间
           {
            memcpy(&CAN_LDATA_MSG_IBUF[CAN_LDATA_MSG_IHead],
                   &CAN_MSG_Rx,16);                    //拷贝数据
            CAN_LDATA_MSG_IHead++;                     //指针加1 指向下一帧
            if(CAN_LDATA_MSG_IHead>=CAN_LDILEN)        //判断是否超出循环
             CAN_LDATA_MSG_IHead=0;                    //从头开始循环 
           }
          break;
         }
       }
     }
    else
     {                                                 //发送中断
      switch(ulStatus)                                 //0x8000状态中断  0x01---0x20 引起中断的报文对象
       {	                                             
        case SLV_CHK_ECHO:                             //ECHO 发送中断		    ID3
         {                                             //分控模块回应完成中断
          Echo_Sts = SLV_ECHO_ACK;                     //回应帧依成功发送标志
          break;                                       
         }                                             
        case SLV_LDATA_TX:                          //从机发送长数据串单帧完成 ID11
         {                                          //发送中断
          CAN_LMSG_TX_STS = SLV_LDATA_TX_NO;        //长数据单帧发送完成 置发送空闲状态
          CAN_LTX_OVTimer=(u16)Timer_1ms;           //重置发送定时
          break;
         }                                 
        case SLV_SDATA_TX:                          //从机发送短数据串单帧完成 ID13
         {                                          //发送中断
          CAN_SMSG_TX_STS =0;                       //短帧发送完成
          CAN_STX_OVTimer=(u16)Timer_1ms;           //重置发送定时
          break;
         }
        default:
         break;
       }
      CANIntClear(CAN0,ulStatus);                      //清除发送中断
     }  
}
/*****************************************************************************
* CAN1 中断处理函数
*****************************************************************************/
void CAN1Handler(void)
{
	
}

