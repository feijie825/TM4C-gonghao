/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : ENG_ERR.c
;* Author             : 张力阵
;* 电能脉冲处理函数库
;* 误差板工作模式 见 vari.h 中MODE 定义
*******************************************************************************/
#include "Function.h"
#include "stdio.h"	      //工程中头文件
#include "string.h"
#include "Disp.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
/********************************************************
* IO口基址定义
********************************************************/
const u32 PORT_BASE_ADDR_TAB[]=
{
    GPIO_PORTA_BASE,            //GPIOA 基址
    GPIO_PORTB_BASE,            //GPIOB 基址
    GPIO_PORTC_BASE,            //GPIOC 基址
    GPIO_PORTD_BASE,            //GPIOD 基址
    GPIO_PORTE_BASE,            //GPIOE 基址
    GPIO_PORTF_BASE,            //GPIOF 基址
    GPIO_PORTG_BASE,            //GPIOG 基址
    GPIO_PORTH_BASE,            //GPIOH 基址
	   GPIO_PORTJ_BASE,            //GPIOJ 基址
	   GPIO_PORTK_BASE,            //GPIOK 基址
	   GPIO_PORTL_BASE,            //GPIOL 基址
	   GPIO_PORTM_BASE,            //GPIOM 基址
	   GPIO_PORTN_BASE,            //GPION 基址
	   GPIO_PORTP_BASE,            //GPIOP 基址
};
/********************************************************
* 周期测量脉冲输入管脚对应端口表格 
* GDT_PLS   '1'      //光电头脉冲 GDT_MC
* DZ_PLS    '2'      //电子脉冲   DZ_MC
* SZ_PLS    '3'      //时钟脉冲   SZ_MC
* XUL_PLS   '4'      //需量脉冲   XL_MC
* TQ_PLS    '5'      //投切脉冲   TQ_MC
* HZ_PLS    '6'      //合闸脉冲   HZ_MC
********************************************************/
const u32 CYCLE_PORT_TAB[]=
{
     GDT_MC_GPIO,    //PORTM  
     YGMC_GPIO,      //PORTM  
     SZ_MC_GPIO,     //PORTM  
     XL_MC_GPIO,     //PORTM  
     TQ_MC_GPIO,     //PORTM  
};
/********************************************************
* 周期测量脉冲输入管脚表格 
* GDT_PLS   '1'      //光电头脉冲 GDT_MC
* DZ_PLS    '2'      //电子脉冲   DZ_MC
* SZ_PLS    '3'      //时钟脉冲   SZ_MC
* XUL_PLS   '4'      //需量脉冲   XL_MC
* TQ_PLS    '5'      //投切脉冲   TQ_MC
* HZ_PLS    '6'      //合闸脉冲   HZ_MC
********************************************************/
const u8 CYCLE_PIN_TAB[]=
{
    GDT_MC,
    YGMC, 
    SZ_MC, 
    XL_MC, 
    TQ_MC, 
};
/********************************************************
* 管脚重开定时器 
* 光电头脉冲重开定时器 GDT_MC
* 电子脉冲重开定时器   DZ_MC
* 时钟脉冲重开定时器   SZ_MC
* 需量脉冲重开定时器   XL_MC
* 投切脉冲重开定时器   TQ_MC
* 合闸脉冲重开定时器   HZ_MC
********************************************************/
const vu8_Ptr IO_Timer_Tab[]=
{
    &GDT_Timer,                      //光电头口中断重启定时器
    &DZ_Timer,                       //电子脉冲口中断重启定时器
    &SZ_Timer,                       //时钟脉冲管脚重开中断定时
    &XUL_Timer,                      //需量口中断重启定时器
    &TQ_Timer,                       //投切口中断重启定时器
    &HZ_Timer,                       //合闸口中断重启定时器
};
/********************************************************
* 管脚重开中断标志 
* 光电头脉冲重开定时器 GDT_MC
* 电子脉冲重开定时器   DZ_MC
* 时钟脉冲重开定时器   SZ_MC
* 需量脉冲重开定时器   XL_MC
* 投切脉冲重开定时器   TQ_MC
* 合闸脉冲重开定时器   HZ_MC
********************************************************/
const u32_Ptr IO_REEN_TAB[]=
{
    &GDT_INT_REEN,           //光电头口中断重启定时器
    &DZ_INT_REEN ,           //电子脉冲口中断重启定时器
    &SZ_INT_REEN ,           //时钟脉冲管脚重开中断定时
    &XUL_INT_REEN,           //需量口中断重启定时器
    &TQ_INT_REEN ,           //投切口中断重启定时器
    &HZ_INT_REEN ,           //合闸口中断重启定时器
    &KEY_INT_REEN,           //按键口中断重启定时器
};
/********************************************************
* 发送数据到CAN缓冲区
* 人口: CMD 命令编号
* 人口: Len 数据长度
* 人口: Ptr 待发送数据指针
********************************************************/
void Send_Data(u16 CMD,u8 Len,u8 *Ptr)
{
    u8 m;
    if(CAN_ERR)                           //总线错误
     return;                              //退出
    if(Len>8)                             //判断长度是否超过最大数据长度
     Len=8;                               
    memcpy(CAN_MSG_OPtr,                  //CAN发送指针
           &CAN_TX_SMSG,                  //默认帧
           16);                           
    CAN_MSG_OPtr->ID.BIT.MNUM=Mtr_Numb_ID;//表位号   仲裁域
    CAN_MSG_OPtr->ID.BIT.CMD=CMD;         //命令     仲裁域
    if(Len!=0)
     {
      m=0;
      for(;;)                             //剔除字符串首空格
       {
        if(*Ptr!=' ')                     //首空格不发送 剔除空格
         break;
        Ptr++;
        Len--;
        if(Len==0)
         break;	 		
       }
      if(Len==0)                          //没有有效位
       {
        Ptr--;
        *Ptr='0';                         //发送0
        Len=1;	
       }	 
      CAN_MSG_OPtr->CTL.BIT.LEN=Len;      //数据长度 控制域
      for(m=0;m<Len;m++)                  //拷贝有效数据
       {
       	CAN_MSG_OPtr->Data.BYTE[m]=*Ptr;
       	Ptr++;
       }	 
     }         
    SDATA_MSG_OHead_ADD_ONE();            //发送指针加1处理           
}                                    
/********************************************************
* 电流继电器定时处理
********************************************************/
void I_JDQ_Time_Pr(void)
{
    if(I_JDQ_CHG)                     //判断电流继电器状态是否改变
     {
/*                                    //贝斯特继电器用
      if(I_JDQ_EN)                    //判断使能信号是否有效
       {
        if((u8)(Timer_8ms-IJDQ_Timer)<IJDQ_EN_DELAY) //判断使能信号延时是否到 在ENG_ERR.h中定义
         return;
        IJDQ_Timer=Timer_8ms;         //重启定时
        I_JDQ_EN=0;                   //使能信号无效   
        I_JDQ_EN_CNCL;                //撤销使能信号 低电平
       }
      else  
*/
       {
        if((u8)(Timer_8ms-IJDQ_Timer)<IJDQ_CTL_DELAY) //判断控制信号延时是否到 在ENG_ERR.h中定义
         return;
        I_JDQ_CHG=0;                  //继电器动作结束
								if(I_JDQ)
								{	
          IJDQ_CLOSE_CANCEL;            //撤销信号
								}
								else
								{
          IAJDQ_OPEN_CANCEL;            //撤销信号
//							  	IBJDQ_OPEN_CANCEL;            //撤销信号
 							  ICJDQ_OPEN_CANCEL;            //撤销信号
								}
       }
     }
					if(TTAB_JDQ_DELY)                //双回路继电器动作延时标志
					{
       if((u16)(Timer_1ms-TTAB_JDQ_DELY_Timer)>=
								  (Mtr_Numb_ID*20))					      //双回路继电器动作延时定时
       {
         TTAB_JDQ_DELY=0;             //清双回路继电器动作延时标志
								 TTAB_JDQ_CHG=1;              //双回路继电器动作标志
								 TTAB_JDQ_CHG_Timer=Timer_8ms;//双回路继电器动作定时
								 if(TTAB_JDQ_STS=='2')
									{
									  TTAB2JDQ_CLOSE;                         //回路2继电器闭合
				       TTAB1JDQ_CANCEL;                        //回路1继电器断开
									}
									else
									{
  									TTAB1JDQ_CLOSE;                         //回路1继电器闭合
		       		TTAB2JDQ_CANCEL;                        //回路2继电器断开
									}
							}								
					}
					if(TTAB_JDQ_CHG)                 //双回路继电器动作标志
					{
						  if((u8)(Timer_8ms-TTAB_JDQ_CHG_Timer)>=
								   IJDQ_CTL_DELAY)					      //双回路继电器动作定时
								{
								  TTAB1JDQ_CANCEL;                        //回路1继电器动作撤销 
          TTAB2JDQ_CANCEL;                        //回路1继电器动作撤销			
          TTAB_JDQ_CHG=0;                         //双回路继电器动作标志            									
								}
					}
}
/********************************************************
* 电压继电器定时处理
* 电子开关定时处理
********************************************************/
void UJDQ_ESwitch_Time_Pr(void)
{
    if(UJDQ_FLAG)                    //判断继电器状态是否改变 
     {
      if((u8)(Timer_8ms-UJDQ_Timer)>=UJDQ_DELAY)  //继电器定时到
       {                              
        UJDQ_FLAG=0;                  //清除继电器动作标志
        UJDQ_Timer=Timer_8ms;         //继电器定时器清零
        if(U_JDQ[0])
         UA_JDQ_ON;                   //A相继电器断开 电压接入 
        else
         UA_JDQ_OFF;                  //A相继电器吸合 电压断开 
        if(U_JDQ[1])
         UB_JDQ_ON;                   //三相台B相继电器断开 电压接入 单相台电压低端
        else                          
         UB_JDQ_OFF;                  //三相台B相继电器吸合 电压断开 单相台电压低端
//        if(SINGLE_OR_THREE)           //判断是否为三相台 不为三相台不处理B C相
         {
          if(U_JDQ[2])
           UC_JDQ_ON;                 //C相继电器断开 电压接入 
          else
           UC_JDQ_OFF;                //C相继电器吸合 电压断开
         }
       }  
      }                             
    if(ESwitch_FLAG)                  //判断电子开关状态是否改变
     {	
      if((u8)(Timer_8ms-ESwitch_Timer)>ESwitch_DELAY)
       {
        ESwitch_FLAG=0;               //清除电子开关动作标志
        ESwitch_Timer=Timer_8ms;      //电子开关定时器清零
        if(U_ESwitch[0])
         UA_ESW_ON;                   //A相电子开关吸合 电压接入 
        else                          
         UA_ESW_OFF;                  //A相电子开关断开 电压断开
//        if(SINGLE_OR_THREE)           //判断是否为三相台 不为三相台不处理B C相
         {  
          if(U_ESwitch[1])
           UB_ESW_ON;                 //B相电子开关吸合 电压接入
          else                                                 
           UB_ESW_OFF;                //B相电子开关断开 电压断开 
          if(U_ESwitch[2])
           UC_ESW_ON;                 //C相电子开关吸合 电压接入
          else                                                 
           UC_ESW_OFF;                //C相电子开关断开 电压断开 
         }
       }   
     }  
}
/********************************************************
* 电子开关所在端口
********************************************************/
const u32 ESwitch_PORT_TAB[]=
{
    UA_ESWC_GPIO,                  //A相电压电子开关端口
    UB_ESWC_GPIO,                  //B相电压电子开关端口
    UC_ESWC_GPIO,                  //C相电压电子开关端口
};
/********************************************************
* 电子开关管脚
********************************************************/
const u32 ESwitch_PIN_TAB[]=
{
    UA_ESWC,                //A相电压电子开关管脚
    UB_ESWC,                //B相电压电子开关管脚
    UC_ESWC,                //C相电压电子开关管脚
};
/********************************************************
* 电子开关控制处理
* 处理标志 在主循环中处理吸合还是断开
* 入口:Phase UA_PHASE UB_PHASE UC_PHASE 的组合或ALL_PHASE
* 人口:Pre 预处理 0:预置断开 其它:预置吸合 
* 入口:Sts 状态   0:延时断开 1:延时吸合
********************************************************/
void ESwitch_Control_Pr(u8 Phase,u8 Pre,u8 Sts)
{
    u8 m;
    if((Phase>ALL_PHASE)||    //错误相设置
       (Phase==0)||	          //没有相
       (Sts>1)||              //错误数据设置
       (Pre>2))               //预置
     return;                  
    if(SINGLE_OR_THREE)       //判断是单相台还是三相台       
     {                        //三相台处理 
      u32 GPIOx;
      for(m=0;m<3;m++)
       {
        if((Phase&UA_PHASE))  //该相是否跌落
         { 
          GPIOx=ESwitch_PORT_TAB[m]; //端口
          if(Pre)                    //
           GPIOPinWrite(GPIOx,
                        ESwitch_PIN_TAB[m],  //高电平
                        ESwitch_PIN_TAB[m]);
          else
           GPIOPinWrite(GPIOx,       //低电平
                        ESwitch_PIN_TAB[m],
                        0);
          U_ESwitch[m]=Sts;          //分相设置 
         }
        Phase>>=1;   
       } 
     }
    else
     {                        //单相台处理
      if((Phase&UA_PHASE)==0) //单相台 不允许设置UB 和 UC
       return;                
      if(Pre)                 //判断预处理
       UA_ESW_ON;
      else
       UA_ESW_OFF;
      U_ESwitch[0]=Sts;       //设置A相电压电子开关状态
     }
    if(Pre!=Sts)              //判断状态是否和最终值一致
     {                        //不一致 延时后改变	                           
      ESwitch_FLAG=1;         //电子开关新数据标志
      ESwitch_Timer=Timer_8ms;//电子开关定时器清零 
     }    
}
/********************************************************
* 电压继电器所在端口
********************************************************/
/*
const u32 UJDQ_PORT_TAB[]=
{
    GPIO_PORTE_BASE,       //A相电压继电器端口
    GPIO_PORTE_BASE,       //B相电压继电器端口
    GPIO_PORTE_BASE,       //C相电压继电器端口
};
*/
/********************************************************
* 电压继电器管脚
********************************************************/
/*
const u32 UJDQ_PIN_TAB[]=
{
    UA_JC,                 //A相电压继电器管脚
    UB_JC,                 //B相电压继电器管脚
    UC_JC,                 //C相电压继电器管脚
};
*/
/********************************************************
* 电压继电器控制处理
* 处理标志 在主循环中处理吸合还是断开
* Phase UA_PHASE UB_PHASE UC_PHASE 的组合或ALL_PHASE
* Sts=0 断开 Sts=1 吸合
* 2010.8.16 单相台增加电压低端处理
********************************************************/
void UJDQ_Control_Pr(u8 Phase,u8 Sts)
{
    u8 c=0;                     //状态挂表标志
    if((Phase>ALL_PHASE)||      //错误相设置
       (Phase==0)||	            //没有相
       (Sts>1))                 //错误数据设置
     return;                    
    if(SINGLE_OR_THREE)         
     {                          //三相台处理 
      u8 m;                     
      for(m=0;m<3;m++)          
       {                        
        if((Phase&UA_PHASE))    
         if(U_JDQ[m]!=Sts)      //分相设置 判断状态是否改变
          { 	                  
           U_JDQ[m]=Sts;        //新状态
           c=0xFF;              //状态改变标志
          } 	                  
        Phase>>=1;              
       }                        
     }                          
    else                        //                        
     {                          //单相台处理
      if(Phase&UA_PHASE)        //单相台 
       {               
        if(U_JDQ[0]!=Sts)         
         {	                      
          U_JDQ[0]=Sts;         //设置电压继电器状态(火线)
          c=0xFF;                 
         }
       }
      if(Phase&UB_PHASE)        //设置电压继电器状态(零线)
       {
        if(U_JDQ[1]!=Sts)         
         {	                      
          U_JDQ[1]=Sts;         //设置电压继电器状态(零线)
          c=0xFF;                 
         }
       }
      if(Phase&UC_PHASE)        //设置电压继电器状态(公共端) 用于解决单相载表通信问题
       {                        //正常工作时吸合
        if(U_JDQ[2]!=Sts)         
         {	                      
          U_JDQ[2]=Sts;         //设置C相电压继电器低端状态
          c=0xFF;                 
         }
       }
     } 
    if(c)
     {	                          
      UJDQ_FLAG=1;              //继电器新数据标志
      UJDQ_Timer=Timer_8ms;
     } 
}
/********************************************************
* 电压电流接入处理
********************************************************/
void Uin_Iin_Pr(void)
{
   UJDQ_Close_ESwitch_Open();              //电压继电器接入 电子开关断开
		 I_JDQ=1;
   I_In_Out_Pr(1);                         //设置电流接入
	  TZZS_LED_OFF;                           //关闭跳闸指示灯
   NZTZ_FLAG=0;                            //跳闸恢复 清除
   GZ_FLAG=0;                              //清除故障状态标志
}   
/********************************************************
* 电流接入处理
* 入口: Sts:0 电流旁路  Sts:1 电流接入(继电器断开) 
********************************************************/
void I_In_Out_Pr(u8 Sts)
{
    if(I_JDQ)                                  //电流继电器状态是否为接入(电流旁路)
     {                                         //原先电流旁路
      if(Sts==0)                               //判断是否仍为设置电流旁路
       return;                                 //状态不变 退出
      I_JDQ=0;                                 //电流继电器断开
						IAJDQ_OPEN;                              //A相电流继电器断开(电流接入)
						IBJDQ_OPEN;                              //B相电流继电器断开(电流接入)
						ICJDQ_OPEN;                              //C相电流继电器断开(电流接入)
//      I_IN_EN;                                 //操作控制端口
     }
    else                                       //原先电流接入状态
     {
      if(Sts)                                  //是否设置电流接入
       return;                                 //状态不变 退出
      I_JDQ=1;                                 //电流继电器吸合(电流旁路 bypass)
      IJDQ_CLOSE;                              //电流继电器吸合(电流旁路 bypass)
//      I_BYPASS_EN;                             //操作控制端口
     }   
    I_JDQ_EN=1;                                //电流继电器使能脚有效标志
    I_JDQ_CHG=1;                               //状态改变
    IJDQ_Timer=Timer_8ms;                      //开始计时
}                         
/********************************************************
* 电压继电器都接入 u in
* 电子开关接入
* 继电器接入前先吸合电子开关 防止拉弧
* 电压通过电压继电器接入回路
********************************************************/
/*
void UJDQ_Close_ESwitch_Close(void)
{
    ESwitch_Control_Pr(ALL_PHASE,    //三相电压设置
                       ON,           //电子开关预置 吸合
                       ON);          //电子开关最终 吸合 
    UJDQ_Control_Pr(ALL_PHASE,       //三相电压设置
                    ON);             //电压继电器吸合

//    ESwitch_FLAG=1;                  //电子开关改变标志
//    UJDQ_FLAG=1;                     //电压继电器改变标志
//    UJDQ_Timer=Timer_8ms;            //启动电压继电器定时器
//    ESwitch_Timer=Timer_8ms;         //启动电子开关定时器
}
*/
/********************************************************
* 电压继电器都接入 u in
* 电子开关都断开
* 继电器接入前先吸合电子开关 防止拉弧
* 电压通过电压继电器接入回路
********************************************************/
void UJDQ_Close_ESwitch_Open(void)
{
#ifdef PULSE                         //是否为照相专机脉冲检测
    ESwitch_Control_Pr(ALL_PHASE,    //三相电压设置
                       ON,           //电子开关预置 吸合
                       ON);          //电子开关最终 吸合 
    UJDQ_Control_Pr(UA_PHASE,        //A相电压高端设置
                    OFF);            //A电压继电器断开    火线断开
    if(SINGLE_OR_THREE)              //判断是否为三相台
     UJDQ_Control_Pr(UB_PHASE,       //B相电压高端设置
                     OFF);           //B电压继电器断开    火线断开
    else
     UJDQ_Control_Pr(UB_PHASE,       //单相电压低端设置  零线接入
                     ON);            //电压继电器断开
    UJDQ_Control_Pr(UC_PHASE,        //单相电压高端设置
                    OFF);            //电压继电器断开    火线断开
#else
    ESwitch_Control_Pr(ALL_PHASE,    //三相电压设置
                       ON,           //电子开关预置 吸合
                       OFF);         //电子开关最终断开 
    UJDQ_Control_Pr(ALL_PHASE,       //三相电压设置
                    ON);             //电压继电器吸合
#endif
/*
    ESwitch_FLAG=1;                  //电子开关改变标志
    UJDQ_FLAG=1;                     //电压继电器改变标志
    UJDQ_Timer=Timer_8ms;            //启动电压继电器定时器
    ESwitch_Timer=Timer_8ms;         //启动电子开关定时器
*/
}
/********************************************************
* 电压继电器都断开 u out
* 电子开关接入
* 继电器接入前先吸合电子开关 防止拉弧
* 电压通过电子开关接入回路
* 该状态只在跌落试验中用到
********************************************************/
void UJDQ_Open_ESwitch_Close(void)
{
    ESwitch_Control_Pr(ALL_PHASE,    //三相电压设置
                       ON,           //电子开关预置 吸合
                       ON);          //电子开关最终吸合
    if(SINGLE_OR_THREE)              //判断是否为三相台
     UJDQ_Control_Pr(ALL_PHASE,      //三相电压设置
                     OFF);           //电压继电器断开
    else                             //
     {
      UJDQ_Control_Pr(UA_PHASE,      //单相电压高端设置
                      OFF);          //电压继电器断开    火线断开
      UJDQ_Control_Pr(UB_PHASE,      //单相电压低端设置  零线接入
                      ON);           //电压继电器断开
     }		                 
/*
    ESwitch_FLAG=1;                  //电子开关改变标志
    UJDQ_FLAG=1;                     //电压继电器改变标志
    UJDQ_Timer=Timer_8ms;            //启动电压继电器定时器
    ESwitch_Timer=Timer_8ms;         //启动电子开关定时器
*/
}
/********************************************************
* 电压继电器都断开 u out
* 电子开关都断开
* 继电器接入前先吸合电子开关 防止拉弧
********************************************************/
void UJDQ_Open_ESwitch_Open(void)
{
    ESwitch_Control_Pr(ALL_PHASE,    //三相电压设置
                       ON,           //电子开关预置 吸合
                       OFF);         //电子开关最终断开 
    if(SINGLE_OR_THREE)              //判断是否为三相台
     UJDQ_Control_Pr(ALL_PHASE,      //三相电压设置
                     OFF);           //电压继电器断开
    else                             //
     {
      UJDQ_Control_Pr(UA_PHASE,      //单相电压高端设置
                      OFF);          //电压继电器断开    火线断开
      UJDQ_Control_Pr(UB_PHASE,      //单相电压低端设置  零线接入
                      ON);           //电压继电器断开
     }		                 
/*
    ESwitch_FLAG=1;                  //电子开关改变标志
    UJDQ_FLAG=1;                     //电压继电器改变标志
    UJDQ_Timer=Timer_8ms;            //启动电压继电器定时器
    ESwitch_Timer=Timer_8ms;         //启动电子开关定时器
*/
}
/********************************************************
* 电能脉冲输入处理   
********************************************************/
void ENG_PLS_IN_Pr(void)
{
#ifdef PULSE                                   //是否为脉冲检测程序
    GPIOIntDisable(GDT_MC_GPIO,                //端口 设置管脚中断禁能
                   GDT_MC);	                   //管脚	
    GPIOIntClear(GDT_MC_GPIO,                  //端口 清除端口中断
                 GDT_MC));	                    //管脚
    GPIOPinTypeGPIOInput(GDT_MC_GPIO,          //端口 设置I/O口为输入
                         GDT_MC);              //管脚 PB.2 光电脉冲输入
    GPIOPadConfigSet(GDT_MC_GPIO,              //端口 设置管脚类型
                     GDT_MC,                   //管脚
                     GPIO_STRENGTH_8MA,        //驱动能力
                     GPIO_PIN_TYPE_STD_WPU);   //上拉/下拉
    GPIOIntTypeSet(GDT_MC_GPIO,                //端口 设置管脚中断发送
                   GDT_MC,                     //管脚
                   GPIO_FALLING_EDGE);         //中断方式

    GPIOIntDisable(DZ_MC_GPIO,                 //端口 设置管脚中断禁能
                   DZ_MC);	                    //管脚	
    GPIOIntClear(DZ_MC_GPIO,                   //端口 清除端口中断
                 DZ_MC);	                      //管脚
    GPIOPinTypeGPIOInput(DZ_MC_GPIO,           //端口 设置I/O口为输入
                         DZ_MC);               //管脚 PB.0 电子脉冲输入        
    GPIOPadConfigSet(DZ_MC_GPIO,               //端口 设置管脚类型
                     DZ_MC,                    //管脚
                     GPIO_STRENGTH_8MA,        //驱动能力
                     GPIO_PIN_TYPE_STD_WPU);   //上拉/下拉
    GPIOIntTypeSet(DZ_MC_GPIO,                 //端口 设置管脚中断发送
                   DZ_MC,                      //管脚
                   GPIO_FALLING_EDGE);         //中断方式

    GDT_INT_REEN=1;                            //光电头中断使能
    DZ_INT_REEN=1;                             //光电头中断使能
    GDT_Timer=Timer_8ms;                       //光电头中断使能定时
    DZ_Timer=Timer_8ms;                        //光电头中断使能定时
#else
    if(WORK_MODE==MEA_ENG_DUTY_M)              //脉冲周期测量状态 退出
     return;
    if(WORK_MODE==PANZHUAN_ERR_M)              //判断是否在盘转试验状态
     {                                          
      u16 m=DIVIDE_Coef;                       //保存分频值
      GDT_PLS_IN_DEN(1);                       //光电头脉冲输入使能
      DIVIDE_Coef=0x1;                         //不分频
      ELSC_PLS_IN_DEN(1);                      //电子脉冲输入使能
      DIVIDE_Coef=m;                           //                                         
     } 
    else
     {
      FIRST_ENG_PLS=1;                         //重新计算误差
      if(ENG_CLK_CH==GDT_PLS)                  //判断是否为光电头脉冲输入                  
       {  
        GDT_PLS_IN_DEN(1);                     //光电头脉冲输入使能
        ELSC_PLS_IN_DEN(0);                    //电子脉冲输入禁能                                              
       }
      else                                     //电子脉冲输入
       {                                       //关光电头脉冲输入中断
        GDT_PLS_IN_DEN(0);                     //光电头脉冲输入禁能
        ELSC_PLS_IN_DEN(1);                    //电子脉冲输入使能                                              
       } 
     }
#endif	  
}
/********************************************************
* 重新开始误差校验
* 相当于原误差板 GDCLS()子程序
********************************************************/
void ReStart_Mea_Err(void)
{
    u8 m;
    m=Disp_Choose;                               //
    Disp_Choose=DISP_TEST_DATA;
    memset(Disp_Buf,DISP_BLANK,8);               //显示同一设置 清显示
    if(WORK_MODE==PANZHUAN_ERR_M)                //判断是否在计算盘转误差状态
     CURRENT_N=PZZS;                             //设定转数
    else 
     CURRENT_N=ACT_N;                            //设定圈数
    FIRST_ENG_PLS=1;                             //重新接收脉冲
    FIRST_CLK_PLS=1;                             //首个时钟脉冲标志
    FIRST_XUL_PLS=1;                             //首个需量脉冲标志
    Update_N_Buf();                              //更新圈数区 
    Update_Mtr_Num();                            //更新表位号区
    Disp_Timer=(Timer_8ms-DISP_TIME);            //更新显示
    Disp_Choose=m;                               //恢复
//    Disp_Data(Disp_Code_Mode);                   //按方式0译码(与原误差板译码方式相同)
    Send_Data(ERR_OCMD_CN,2,CURRENT_N_ASC);      //回送当前圈数2bytes
}
/********************************************************
* 计算被检表高频重装值和重装次数
********************************************************/
void Cal_Gp_Relaod(void)
{
    if(GP_CLK_SET<100)                           //判断高频脉冲数设置是否太小
     GP_CLK_SET=100;
    if(GP_CLK_SET<60000)                         //是否超过最大分频设置
     {                                           //不超过
      GP_RELOAD_VAL=GP_CLK_SET;                  //高频重装值
      GP_RELOAD_TIME=1;                          //高频重装次数
     }
    else
     {                                           //超过最大分频设置
      GP_RELOAD_VAL=60000;                       //默认最大重装值
      GP_RELOAD_TIME=((GP_CLK_SET+30000)/60000); //计算重装次数 四舍五入
      if(GP_RELOAD_TIME==0)                      //重装次数是否为零
       GP_RELOAD_TIME=1;                         //置1
     }
    GP_CLK_ACT = GP_RELOAD_VAL;                  //
    GP_CLK_ACT *= GP_RELOAD_TIME;                //实际计数值
    GP_RELOAD_Cnt = GP_RELOAD_TIME;              //重装次数
}
/********************************************************
* 计算实际应计标准电能脉冲个数
* 计数实际校验圈数(归一化)
* 计算变差 5.0% 变差超过5.0% 重新计算误差
* 计算后 所有表位误差校验 基本同时完成
********************************************************/
void Cal_STD_ENG_CNT_VAL(void)
{
    float f;
    f=MTR_ENG_CST;                               //本表位低频常数
    f*=SET_N;                                    //设定圈数
    f/=MTR_MIN_CST;                              //所有表位最小低频常数
    ACT_N=(u16)(f+0.5);                          //计算实际圈数 四舍五入
    if(ACT_N==0)                                 //判断实际圈数是否为0
     ACT_N=1;                                    //最小1圈
    CURRENT_N=ACT_N;                             //刷新当前圈数
    STD_ENG_CNT_VAL=STD_ENG_CST;                 //标准表高频常数 1kwh
    STD_ENG_CNT_VAL/=MTR_ENG_CST;                //每圈应计的标准电能脉冲数
    STD_ENG_CNT_VAL*=DIVIDE_Coef;                //分频系数 2012.4.15添加
    ENG_STB_RNG = (u16)(STD_ENG_CNT_VAL/10+0.5); //变差范围 10% 每圈误差变差
    STD_ENG_CNT_VAL*=ACT_N;                      //在实际圈数内应计的标准电能脉冲数
    STD_ENG_CNT_VAL= (u32)(STD_ENG_CNT_VAL+0.5); //化整 四舍五入
    if((WORK_MODE==CAL_ENG_ERR_M)||              //判断是否在校验误差状态
       (WORK_MODE==PANZHUAN_ERR_M))              //盘转误差模式
     {                                        
      FIRST_ENG_PLS=1;                           //重新计算电能误差
      Update_N_Buf();                            //
     } 
}
/********************************************************
* 接收标准表高频常数处理
* 数据格式 低位->高位 数据低位0 数据低位1 数据低位2 符号位+指数位
********************************************************/
void Set_STD_CST_Pr(void)
{
    float f;
    memcpy(&f,
           &CAN_MSG_IPtr->Data.BYTE,
           4);
    if((f==0)||                       //高频常数不能为零
       (f==STD_ENG_CST))              //高频常数没改变退出
     return; 
    STD_ENG_CST=f;                    //更新标准表高频常数
    Cal_STD_ENG_CNT_VAL();            //计算
    if(WORK_MODE==CAL_ENG_ERR_M)      //判断是否在校验误差状态
     FIRST_ENG_PLS=1;                 //重新计算电能误差
}
/********************************************************
* 接收校验圈数并处理
* 设定 16进制值 
********************************************************/
void Set_N_Pr(void)
{
    u8 N;
    N=CAN_MSG_IPtr->Data.BYTE[0];     //读取设定圈数
    if(N==0)                          //圈数设置非法
     return;                          //退出
    if(WORK_MODE==PANZHUAN_ERR_M)     //判断当前是否在盘转误差试验状态
     {
      if(N==PZZS)                     //设置未变
       return;	     		
      PZZS = N;                       //更新盘转圈数
      PZ_STD_CNT=PZZS*PZBZ;           //标准脉冲数
      NEW_CMD=1;                      //新命令标志
     }
    if(N!=SET_N)	                     //判断设定圈数是否改变
     {
      SET_N = N;                      //更改圈数
      Cal_STD_ENG_CNT_VAL();          //计算实际圈数和标准值
	    }
    if(WORK_MODE==CAL_ENG_ERR_M)      //判断是否在校验误差状态
     {
      FIRST_ENG_PLS=1;                //重新计算电能误差
//      NEW_CMD=1;                      //
     } 
}
/********************************************************
* 接收误差清零处理
********************************************************/
void ERR_CLR_Pr(void)
{
    if(!MTR_PLUG)                      //判断是否挂表
     return;                           //没挂表 退出
    if(WORK_MODE==CAL_ENG_ERR_M)       //判断是否在校电能误差状态
     {
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);  //显示同一设置 清显示
      FIRST_ENG_PLS=1;                 //重新计算电能误差
      CURRENT_N=ACT_N;                 //重置圈数
      Update_N_Buf();                  //更新圈数区 
      Update_Mtr_Num();                //更新表位号区
     }
    else if(WORK_MODE==START_STOP_M)   //监视电能脉冲计数状态
     NEW_CMD=1;                        //新命令处理
    else if(WORK_MODE==MEA_CST_M)      //判断是否为常数校验状态
     {
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);  //显示同一设置 清显示
      Update_Mtr_Num();                //更新表位号区
      CURRENT_PLS_CNT=0;               //清除脉冲计数
     }  
    Disp_Timer=(Timer_8ms-DISP_TIME);  //更新显示
}      
/********************************************************
* 光电头复位处理
* 延时到后 清除光电头复位信号
********************************************************/
void GDT_RST_Pr(void)          
{
    if(GDT_RST_FLAG)                 //判断光电头是否在复位状态
     {
      if((u8)(Timer_8ms-GDT_RST_Timer)<GDT_RST_TIME)//判断定时是否到
       return;                         //定时未到 退出
      GDT_RST_Timer=Timer_8ms; 
      GDT_RST_FLAG=0;                  //清除光电头复位标志
      GDT_RST_DN;                      //光电头复位禁能
     }  
}
/********************************************************
* 接收光电头对光  处理
********************************************************/
void ELEC_HEAD_RESET_Pr(void)
{
    if(!MTR_PLUG)                     //判断是否挂表
     return;                          //未挂表 退出
    GDT_RST_EN;                       //光电头复位使能
    GDT_RST_FLAG=1;                   //光电头对光标志 
    GDT_RST_Timer=Timer_8ms;          //清除光电头复位定时器
    ERR_CLR_Pr();                     //光电头复位同时误差清零			
}
/********************************************************
* 选表位 挂表  处理
* 2009.12.10 改变协议
* 携带数据 DATA0 DATA1 DATA2
* DATA0 起始表位 DATA1 终止表位
* DATA2 挂表状态
* 2010.11.16 继电器动作延时加长
* 2011.7.1 根据命令为 单块表命令 还是 多块表命令选择是否延时
********************************************************/
void MTR_PLUG_Pr(void)
{
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                   //不在设置范围 退出	 	
    if(CAN_MSG_IPtr->Data.BYTE[2]==GB)         //判断是否设置为挂表
     {                                         //设置为不挂表
      if(!MTR_PLUG)                            //判断挂表状态是否改变
       {                                       //原先不挂表
        MTR_PLUG=1;                            //挂表标志
        NEW_CMD=1;                             //新命令处理
       }
     }
    else if(CAN_MSG_IPtr->Data.BYTE[2]==BGB)   //设置为不挂表
     {                                         //
      if(MTR_PLUG)                             //判断挂表状态是否改变
       {                                       //改变处理
        MTR_PLUG=0;                            //设置为不挂表
        NEW_CMD=1;                             //新命令处理
       }
     }
    else
     return;
    WORK_MODE=MTR_PLUG_M;                      //挂表状态
    if(CAN_MSG_IPtr->Data.BYTE[0]==CAN_MSG_IPtr->Data.BYTE[1]) //是否为单块表命令
     {	
      KEY_PC_PLUG=1;                           //按键/单块电表选择挂表 不延时
      WORK_Timer=20;                           //挂表延时 防止开关电源同时动作 电流太大 拉低电源
     } 
    else                                       //多块表挂表命令
     {	
      KEY_PC_PLUG=0;                           //广播指令挂表 不延时
      WORK_Timer=(Mtr_Numb_ID*40);             //挂表延时 防止开关电源同时动作 电流太大 拉低电源
     }
} 
/********************************************************
* 对黑斑 处理   
* 0退出其他试验，回到校验状态 
* 1 前沿对斑
* 2 后沿对斑      
********************************************************/
void CATCH_HB_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    WORK_Timer=(Mtr_Numb_ID*40);               //进入工作状态延时 防止开关电源同时动作 电流太大 拉低电源
    if(CAN_MSG_IPtr->Data.BYTE[0]==BACK_MEA_ERR_D)//判断是否退出对斑
     {                                         //结束对斑回校验状态
      WORK_MODE=CAL_ENG_ERR_M;                 //设置为校验误差模式
      CURRENT_PLS_CNT=0;                       //清除脉冲计数
      CURRENT_ENG_KWh=0;                       //清除电能累计
      NEW_CMD=1;                               //新命令标志
						ABBA_HIGH;                               //RS485极性正常
      return;
     }
    else if(CAN_MSG_IPtr->Data.BYTE[0]==CATCH_HB_BWEG)//判断是否为后延对斑
     HB_BACK_EDGE=1;                           //后延对斑
    else
     HB_BACK_EDGE=0;                           //前延对斑 
    NEW_CMD=1;                                 //新命令标志
    WORK_MODE=CATCH_HB_M;                      //对斑模式
//    UJDQ_Close_ESwitch_Close();                //继电器接入 电子开关接入
//    I_In_Out_Pr(1);                            //电流接入   
}  
/********************************************************
* 监视光电头脉冲处理   
* 1起动/潜动时记数 0退出其它试验，回到校验状态       
********************************************************/
void Start_Stop_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    WORK_Timer=(Mtr_Numb_ID*40);               //进入工作状态延时 防止开关电源同时动作 电流太大 拉低电源
    if(CAN_MSG_IPtr->Data.BYTE[0]==BACK_MEA_ERR_D)//判断是否退出对斑
     {
      WORK_MODE=CAL_ENG_ERR_M;                 //设置为校验误差模式
      CURRENT_PLS_CNT=0;                       //清除脉冲计数
      CURRENT_ENG_KWh=0;                       //清除电能累计
      NEW_CMD=1;                               //新命令标志
						ABBA_HIGH;                               //RS485极性正常				
     }
    else
     {
      WORK_MODE=START_STOP_M;                  //监视电能脉冲状态
      NEW_CMD=1;                               //新命令标志
//      Uin_Iin_Pr();                            //继电器接通控制
     }  
}    
/********************************************************
* 电子脉冲输入处理
********************************************************/
void SEL_ELEPLS_IN(void)
{
/*
    if(PLSHC_MODE==HC_2)                      //脉冲合成为有功脉冲和无功脉冲
     {
      if((PLS_QUAD==PA_PLS)||                 //正向有功
      	 (PLS_QUAD==PR_PLS))                   //反向有功
       POS_Watt_SEL;                          //选择正向有功脉冲
      else
       POS_Var_SEL;                           //选择正向无功脉冲
     }	
    else if(PLSHC_MODE==HC_3)                 //有功合成 无功分开
     {
      if((PLS_QUAD==PA_PLS)||                 //正向有功
      	 (PLS_QUAD==PR_PLS))                   //反向有功
       POS_Watt_SEL;                          //选择正向有功脉冲
      else if(PLS_QUAD==QA_PLS)               //正向无功
       POS_Var_SEL;                           //选择正向无功脉冲
      else
       NEG_Var_SEL;                           //选择反向无功脉冲
     }	
    else                                      //四路脉冲分开
     {
      if(PLS_QUAD==PA_PLS)                    //判断是否为正向有功 
       POS_Watt_SEL;                          //选择正向有功脉冲
      else if(PLS_QUAD==QA_PLS)               //判断是否为正向无功
       POS_Var_SEL;                           //选择正向无功脉冲
      else if(PLS_QUAD==PR_PLS)               //判断是否为反向有功 
       NEG_Watt_SEL;                          //选择反向有功脉冲
      else                                    //判断是否为反向无功 
       NEG_Var_SEL;                           //选择反向无功脉冲
     }	
*/					
}
/********************************************************
* 接收脉冲所在象限
#define PA_PLS           '0'      //正向有功
#define QA_PLS           '1'      //正向无功
#define PR_PLS           '2'      //反向有功
#define QR_PLS           '3'      //反向无功
********************************************************/
void Set_PLS_QUAD(void)                 
{
    u8 m;
    if(CAN_MSG_IPtr->CTL.BIT.LEN!=1)//数据长度是否错误
     return;
    m=CAN_MSG_IPtr->Data.BYTE[0];
    if((m<PA_PLS)||                //判断参数是否错误
    	  (m>QR_PLS)||
				   (m==PLS_QUAD))              //判断脉冲象限是否改变
     return;
    PLS_QUAD=m;                    //更新 	 	  
    SEL_ELEPLS_IN();               //电子脉冲输入处理
}               
/********************************************************
* 喇叭开关 处理   
* 0:喇叭关  1:喇叭开      
********************************************************/
void BEEP_EN_Pr(void)
{
    if(CAN_MSG_IPtr->Data.BYTE[0]==BEEP_ONN)        //判断喇叭是否开
     BEEP_EN=1;                                     //喇叭使能标志
    else
     BEEP_EN=0;                                     //喇叭禁能标志
} 
/********************************************************
* 准备开始走字试验 处理 
* GMXXYYYY(0Dh) 原命令XX表位号 YYYY走字圈数  
********************************************************/
void Login_Verify_Sts(void)
{
    if(!MTR_PLUG)                                //判断是否挂表
     return;                                     //不挂表退出
    if(WORK_MODE!=CAL_ENG_ERR_M)                 //判断是否在误差计算状态
     return;                                     //不在 退出
    WORK_MODE=VERIFY_READY_M;                    //准备开始走字状态
    NEW_CMD=1;                                   //新命令标志
    WORK_Timer=(Mtr_Numb_ID*40);                 //进入工作状态延时 防止开关电源同时动作 电流太大 拉低电源
    VERIFY_END=0;                                //清除校核常数走字试验结束标志
    if(SOLID_CFG.LED_NUM==LED_6)                 //是否为6位数码管
     Disp_Long_Data(6,                           //显示数据长度 
                    6,                           //显示起始地址 1---6 第一位到第六位
                    CURRENT_VERIFY_PLS,          //待显示数据
                    DISP_FILL_BLANK);            //显示走字圈数 数据长度 显示起始位 显示值  空位填充
    else                                         
     Disp_Long_Data(8,                           //显示数据长度 
                    8,                           //显示起始地址 1---8 第一位到第八位
                    CURRENT_VERIFY_PLS,          //待显示数据
                    DISP_FILL_BLANK);            //显示走字圈数 数据长度 显示起始位 显示值  空位填充
}    
/********************************************************
* 开始进行走字试验 处理   
* 原命令 GNXXYYYY(0dh) XX表位号 YYYY走字电能 现不用
********************************************************/
void Verify_Start_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    if(WORK_MODE!=VERIFY_READY_M)              //是否准备好
     return;                                   //没有准备好 退出	
    WORK_MODE=VERIFY_START_M;                  //开始走字试验 进入相应状态处理
    NEW_CMD=1;                                 //新命令标志
    WORK_Timer=(Mtr_Numb_ID*40);               //进入工作状态延时 防止开关电源同时动作 电流太大 拉低电源
}    
/********************************************************
* 接收被校表常数  处理  
* 接收浮点数 4字节 
********************************************************/
void Set_Mtr_Cst_Pr(void)
{
    float f;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                   //不在设置范围 退出	 	
    memcpy(&f,
           &CAN_MSG_IPtr->Data.BYTE[2],
           4);
    if((f==0)||                                //判断是否合法
       (f==MTR_ENG_CST))                       //判断常数是否改变
     return;
    MTR_ENG_CST=f;                             //更新常数
    Cal_STD_ENG_CNT_VAL();                     //处理实际校验圈数
    if(WORK_MODE==CAL_ENG_ERR_M)               //判断是否在校验误差状态
     FIRST_ENG_PLS=1;                          //重新计算误差
//以下为重新计算走字脉冲数
    f=VERIFY_ENG_Kwh;
    f*=MTR_ENG_CST;                            //脉冲数
    f/=DIVIDE_Coef;                            //除以分频系数 2012.4.15更改
    f+=0.5;
    VERIFY_PLS_Set=(u32)f;                     //换算成脉冲数
    CURRENT_VERIFY_PLS=VERIFY_PLS_Set;         //初始化当前走字脉冲数
    sprintf((char*)TEMP_STR,
            "%6.2f",
            VERIFY_ENG_Kwh);                   //显示 走字度数
    Fill_Space_StrCpy(TEMP_STR,                //待拷贝的字符串
                      VERIFY_ENG_ASC,          //待写入的字符串
                      7);                      //字符串最大长度                 
}   
/********************************************************
* 脉冲选择  电子脉冲还是光电头脉冲  处理   
* 0:光电头脉冲 1:电子脉冲 
********************************************************/
void PLS_SEL_Pr(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //电能脉冲输入方式
    if(((m==GDT_PLS)||(m==DZ_PLS))&&           //光电头脉冲 电子脉冲
       (m!=ENG_CLK_CH))                        //电能脉冲输入方式是否改变
     {	 
      ENG_CLK_CH=m;                            //更新电能脉冲输入标志
      ENG_PLS_IN_Pr();                         //电能脉冲输入处理   
     }
}   
/********************************************************
* 光电头脉冲输入使/禁能 
* 入口: DEN  0:输入禁能 1:输入使能  
********************************************************/
void GDT_PLS_IN_DEN(u8 DEN)       
{
    GPIOIntDisable(GDT_MC_GPIO,                //端口 设置管脚中断禁能
                   GDT_MC);                   //管脚	
    GPIOIntClear(GDT_MC_GPIO,                 //端口 清除端口中断
                 GDT_MC);	                    //管脚
    if(DEN)
     {	
      GPIOPinTypeGPIOInput(GDT_MC_GPIO,       //端口 设置I/O口为输入
                           GDT_MC);           //管脚 PB.2 光电头脉冲输入
      GPIOPadConfigSet(GDT_MC_GPIO,           //端口 设置管脚类型
                       GDT_MC,                //管脚
                       GPIO_STRENGTH_8MA,     //驱动能力
                       GPIO_PIN_TYPE_STD_WPU);//上拉/下拉
      if((WORK_MODE==CATCH_HB_M)&&            //对斑模式处理
         (!HB_CATCHED)&&                      //判断黑斑是否对准
         (HB_BACK_EDGE))                      //判断是否为后延对斑
       GPIOIntTypeSet(GDT_MC_GPIO,            //端口
                      GDT_MC,                 //管脚
                      GPIO_RISING_EDGE);      //上升沿中断(后延)
      else
       GPIOIntTypeSet(GDT_MC_GPIO,            //端口 设置管脚中断发送
                      GDT_MC,                 //管脚
                      GPIO_FALLING_EDGE);     //中断方式
      GDT_INT_REEN=1;                         //光电头中断使能
      GDT_Timer=Timer_8ms;                    //光电头中断使能定时
     } 
}
/********************************************************
* 电子脉冲输入使能/禁能 
* 入口: DEN  0:输入禁能 1:输入使能  
********************************************************/
void ELSC_PLS_IN_DEN(u8 DEN)       
{
    u8  ucPins;                                //管脚定义
    u8  GPIOx;                                 //端口
    u32 TIMERx;                                //计数器
    u32 ulTimer,ulIntFlags;                    //TIMER_A TIMER_B 标志
    if(MTYPE==SOUTH)                           //判断是否为南网表
     {	
      ucPins=CYCLE_PIN_TAB[SZ_PLS-GDT_PLS];    //选择时钟脉冲输入口作为 电子脉冲输入 南网表
      GPIOx=CYCLE_PORT_TAB[SZ_PLS-GDT_PLS];    //选择时钟脉冲输入口作为 电子脉冲输入 南网表
      TIMERx=TIMER4_BASE;                      //默认的时钟脉冲计数器 
      ulTimer=TIMER_B;                         //默认B
      ulIntFlags=TIMER_CAPB_MATCH;             //中断溢出标志
     } 
    else                                       //国网表 
     {	                                       
      ucPins=CYCLE_PIN_TAB[DZ_PLS-GDT_PLS];    //选择电子脉冲输入口作为 电子脉冲输入 国网表
      GPIOx=CYCLE_PORT_TAB[DZ_PLS-GDT_PLS];    //选择电子脉冲输入口作为 电子脉冲输入 国网表
      TIMERx=TIMER5_BASE;                      //默认的电子脉冲计数器 
      ulTimer=TIMER_A;                         //默认A
      ulIntFlags=TIMER_CAPA_MATCH;             //中断溢出标志
     }
    TimerDisable(TIMERx,
                 ulTimer); 
    TimerIntClear(TIMERx,                      //清溢出中断
                  ulIntFlags);
    if(DEN)                                    
     {                                         
      GPIOPinTypeTimer(GPIOx,                  //端口
                       ucPins);                //管脚
      if((WORK_MODE==CATCH_HB_M)&&             //对斑模式处理
         (!HB_CATCHED)&&                       //判断黑斑是否对准
         (HB_BACK_EDGE))                       //判断是否为后延对斑
       TimerControlEvent(TIMERx,                //定时器
                         ulTimer,               //通道 
                         TIMER_EVENT_POS_EDGE);//脉冲输入方式
      else
       TimerControlEvent(TIMERx,                //定时器
                         ulTimer,               //通道 
                         TIMER_EVENT_NEG_EDGE);//脉冲输入方式
       TimerLoadSet(TIMERx,                     //定时器
                    ulTimer,                    //通道
                    DIVIDE_Coef);               //设置(分频系数)
       TimerMatchSet(TIMERx,                    //定时器
                     ulTimer,                   //通道
                     0);
       TimerIntEnable(TIMERx,                   //定时器
                      ulIntFlags);              //
       TimerEnable(TIMERx,                      //定时器
                   ulTimer);                    //定时器启动
     }
}
/********************************************************
* //计电能试验  处理   
* 0:终止    1:开始   
********************************************************/
void Cnt_ENG_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    if(CAN_MSG_IPtr->Data.BYTE[0]=='1')        //判断是否停止计电能
     {                                         //暂停计电能 同时断开电压
      WORK_MODE=MEASURE_ENG_M;                 //计电能状态
      WORK_Timer=(Mtr_Numb_ID*40);             //挂表延时 防止开关电源同时动作 电流太大 拉低电源
      CURRENT_PLS_CNT=0;                       //脉冲计数
      CURRENT_ENG_KWh=0;                       //清除当前电能
      sprintf((char*)TEMP_STR,
              "%8.5f",
              CURRENT_ENG_KWh);                //显示0.0000
      Fill_Space_StrCpy(TEMP_STR,              //待拷贝的字符串
                        CURRENT_ENG_ASC,       //待写入的字符串
                        9);                    //字符串最大长度                 
      Send_Data(ERR_OCMD_CP,8,CURRENT_ENG_ASC);//回送当前电能
      if(Disp_Choose==DISP_TEST_DATA)          //判断是否显示当前试验数据
       Copy_Str_To_DSBUF(DISP_PWR_LEN,         //电能数据长度
                         DISP_PWR_OFFSET,      //电能显示偏移量
                         CURRENT_ENG_ASC);     //电能ASC地址   
     }
    else if(WORK_MODE==MEASURE_ENG_M)          //判断原先是否为计电能试验
     {
      UJDQ_Open_ESwitch_Open();                //断开电压接入
     } 
}   
/********************************************************
* 清除电能计数  处理   
********************************************************/
void ENG_CLR_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    if(WORK_MODE==MEASURE_ENG_M)               //判断是否在计电能状态
     {
      CURRENT_PLS_CNT=0;                       //脉冲计数
      CURRENT_ENG_KWh=0;                       //清除当前电能
      sprintf((char*)TEMP_STR,
              "%8.5f",
              CURRENT_ENG_KWh);                //显示0.0000
      Fill_Space_StrCpy(TEMP_STR,              //待拷贝的字符串
                        CURRENT_ENG_ASC,       //待写入的字符串
                        9);                    //字符串最大长度                 
      if(Disp_Choose==DISP_TEST_DATA)          //是否在显示当前试验数据状态
       Copy_Str_To_DSBUF(DISP_PWR_LEN,         //电能数据长度
                         DISP_PWR_OFFSET,      //电能显示偏移量
                         CURRENT_ENG_ASC);     //电能ASC地址   
     }                                        
    else if(WORK_MODE==PULSE_ZZ_M)             //判断是否在定脉冲走字试验状态
     {
      CURRENT_PLS_CNT=0;                       //脉冲计数
      if(Disp_Choose==DISP_TEST_DATA)          //是否在显示当前试验数据状态
       {	
        memset(Disp_Buf,DISP_BLANK,8);         //显示同一设置 清显示
        if(SOLID_CFG.LED_NUM==LED_6)           //是否为6位数码管
         Disp_Buf[5]=0;                        //显示0    最后一位
        else
         Disp_Buf[7]=0;                        //显示0    最后一位
       } 
     }
    Disp_Timer=(Timer_8ms-DISP_TIME);          //更新显示
//    Disp_Data(Disp_Code_Mode);                 //显示
}   
/********************************************************
* 盘转试验 电子脉冲和光电头脉冲比较   处理   
* 进入试验处理程序后 将电子脉冲计数分频 初始化为1
********************************************************/
void PZ_ERR_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    WORK_MODE=PANZHUAN_ERR_M;                  //进入盘转试验
    NEW_CMD=1;                                 //新命令标志
    WORK_Timer=(Mtr_Numb_ID*40);               //挂表延时 防止开关电源同时动作 电流太大 拉低电源
}  
/********************************************************
* 接收脉冲和盘转比值   处理   
********************************************************/
void Set_Pzbz_Pr(void)
{
    u8 m;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                   //不在设置范围 退出	 	
    m=CAN_MSG_IPtr->Data.BYTE[2];              //盘转比值
    if((m==PZBZ)||                             //判断盘转比值是否改变
       (m==0))                                 //盘转比值设置非法
     return;                                   //盘转比值不变 退出
    PZBZ=m;                                    //更新盘转比值 		
    PZ_STD_CNT=PZZS*PZBZ;                      //标准脉冲数
    if(WORK_MODE==PANZHUAN_ERR_M)              //判断当前是否在盘转误差试验状态
     NEW_CMD=1;                                //新命令标志
}                             
/********************************************************
* 接收脉冲和盘转设定比较圈数  处理   
********************************************************/
void Set_Pzzs_Pr(void)  
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //盘转转数
    if((m==PZZS)||                             //盘转转数是否改变
       (m==0))                                 //盘转转数设置非法
     return;                                   //退出	
    PZZS = m;                                  //更新盘转圈数
    PZ_STD_CNT=PZZS*PZBZ;                      //标准脉冲数
    if(WORK_MODE==PANZHUAN_ERR_M)              //判断当前是否在盘转误差试验状态
     NEW_CMD=1;                                //新命令标志
}
/********************************************************
* 进入失压状态处理
* Data.BYTE[0] 失压试验模式   
********************************************************/
void Login_Sy_Sts(void)
{
    u8 m;	
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    if(WORK_MODE!=CAL_ENG_ERR_M)               //判断是否在误差试验状态 
     return;                                   //失压状态不接受新失压方案 
    WORK_MODE=VOLTAGE_DOWN_M;                  //进入失压试验
    m=CAN_MSG_IPtr->Data.BYTE[0];              //模式
    if((m>=VLOSS_1)&&                          //是否超出跌落方案
       (m<=VLOSS_3)&&                          //数据非法
       (m!=SY_MODE))                           //方案不变 退出
     SY_MODE=m;	                               //
    WORK_Timer=(Mtr_Numb_ID*40);               //挂表延时 防止开关电源同时动作 电流太大 拉低电源
    SY_START=0;                                //清除失压启动标志  
    NEW_CMD=1;                                 //新命令标志 失压准备
}  
/********************************************************
* 失压试验开始  处理   
********************************************************/
void Start_Sy_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    if(WORK_MODE!=VOLTAGE_DOWN_M)              //判断当前是否在失压试验状态    
     return;                                   //不在 退出
    SY_START=1;                                //失压启动标志  
} 
/********************************************************
* 选择需要测量脉冲周期和频率的脉冲处理 
* EPLS_T           '1'      //测量电能脉冲周期
* SZ_T             '2'      //测量时钟脉冲周期
* XUL_T            '3'      //测量需量脉冲周期
* TQ_T             '4'      //投切脉冲
* HZ_T             '5'      //合闸脉冲
********************************************************/
void CYCLE_PLS_SEL_Pr(void) 
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];
    if((m<EPLS_T)||                             //判断是否为非法设置 超出范围
       (m>HZ_T))                                //判断是否为非法设置 超出范围
     return;
    CYCLE_MEA_SEL=m;                            //更新设置结果
    CYCLE_MEA_ID=((m&0x0f)-1);                  //编号
    if(WORK_MODE==MEA_ENG_DUTY_M)               //判断是否在测量脉冲周期状态
     NEW_CMD=1;                                 //重新计算
}   
/********************************************************
* 设置电子脉冲分频系数 处理 
********************************************************/
void DIVIDE_COEF_Pr(void)
{
    u16 m;
    float f;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                   //不在设置范围 退出	 	
    memcpy(&m,
           &CAN_MSG_IPtr->Data.BYTE[2],
           2);
    if((m==0)||                                //判断是否为非法分频
       (m==DIVIDE_Coef))                       //分频系数是否改变  
     return;		
    DIVIDE_Coef=m; 
    if(WORK_MODE==CAL_ENG_ERR_M)               //判断是否在电能误差计算模式
     {
      u32 TIMERx;                                //计数器
      u32 ulTimer;                               //TIMER_A TIMER_B 标志
      if(MTYPE==SOUTH)                           //判断是否为南网表
       {	
        TIMERx=TIMER4_BASE;                      //默认的时钟脉冲计数器 
        ulTimer=TIMER_B;                         //默认B
       } 
      else                                       
       {	                                       //国网表
        TIMERx=TIMER5_BASE;                      //默认的电子脉冲计数器 
        ulTimer=TIMER_A;                         //默认A
       }
      TimerDisable(TIMERx,                      //Timer禁能
                   ulTimer); 
      TimerLoadSet(TIMERx,                      //定时器
                   ulTimer,                     //通道
                   DIVIDE_Coef);                //设置(分频系数)
      TimerMatchSet(TIMERx,                     //定时器
                    ulTimer,                    //通道
                    0);
      TimerEnable(TIMERx,                       //定时器
                  ulTimer);                     //定时器启动
      Cal_STD_ENG_CNT_VAL();                   //重新计算
      FIRST_ENG_PLS=1;                         //重新计算电能误差
     }
//以下为重新计算走字脉冲数
    f=VERIFY_ENG_Kwh;
    f*=MTR_ENG_CST;                            //脉冲数
    f/=DIVIDE_Coef;                            //除以分频系数 2012.4.15更改
    f+=0.5;
    VERIFY_PLS_Set=(u32)f;                     //换算成脉冲数
    CURRENT_VERIFY_PLS=VERIFY_PLS_Set;         //初始化当前走字脉冲数
    sprintf((char*)TEMP_STR,
            "%6.2f",
            VERIFY_ENG_Kwh);                   //显示 走字度数
    Fill_Space_StrCpy(TEMP_STR,                //待拷贝的字符串
                      VERIFY_ENG_ASC,          //待写入的字符串
                      7);                      //字符串最大长度                 
}
/********************************************************
* 计读脉冲法常数测试试验 处理 
* 返回被检表脉冲
********************************************************/
void MEA_MTR_CST_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    WORK_MODE=MEA_CST_M;                       //置为常数测试试验
    NEW_CMD=1;                                 //新命令标志 失压开始
    CURRENT_PLS_CNT=0;                         //脉冲计数累加值清零      
    WORK_Timer=(Mtr_Numb_ID*40);               //挂表延时 防止开关电源同时动作 电流太大 拉低电源
}
/********************************************************
* 功耗测试 表位和相别选择 处理 
* XXXY或XXY  XXX或XX 要测量的表位
* Y 要测量的相 0:停止测量 1:A相 2:B相 3:C相
********************************************************/
void Power_Test_Pr(void)
{
    u8 m,n,ID=0,c;
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    if((CAN_MSG_IPtr->CTL.BIT.LEN<3)||
       (CAN_MSG_IPtr->CTL.BIT.LEN>4))
     return;	 
    m=CAN_MSG_IPtr->CTL.BIT.LEN-1; 
    c=CAN_MSG_IPtr->Data.BYTE[m];              //测量控制 
    if((c<'0')||
       (c>'3'))                                //判断控制是否错误
     return;
    for(n=0;n<m;n++)
     {
      if((CAN_MSG_IPtr->Data.BYTE[n]<'0')||
         (CAN_MSG_IPtr->Data.BYTE[n]>'9'))
       return;	 
      ID*=10;
      ID+=(CAN_MSG_IPtr->Data.BYTE[n]&0x0f);   //要测量的表位
     } 	 
    if(ID==0)                                  //表位号=0 退出测量 回校验状态
     {
      if(WORK_MODE!=CAL_ENG_ERR_M)
       {	
        WORK_MODE=CAL_ENG_ERR_M;               //退出试验回误差校验状态
        NEW_CMD=1;                             //新命令标志  
						  ABBA_HIGH;                             //RS485极性正常							
        WORK_Timer=(Mtr_Numb_ID*40);           //延时防止开关电源同时动作 电流太大 拉低电源
       } 
     }     
    else //if(ID==Mtr_Numb_ID)                   //判断是否为本表位
     {
      Pwr_Phase=c;
      if(c=='0')                               //退出功耗测试
       {
        WORK_MODE=CAL_ENG_ERR_M;               //设置为校验误差模式
        CURRENT_PLS_CNT=0;                     //清除脉冲计数
        CURRENT_ENG_KWh=0;                     //清除电能累计
        NEW_CMD=1;                             //新命令标志
			  			ABBA_HIGH;                             //RS485极性正常						
        WORK_Timer=(Mtr_Numb_ID*40);           //延时 防止开关电源同时动作 电流太大 拉低电源
        return;
       }		
      WORK_MODE=MEA_POWER_D_M;                 //进入功耗试验
      WORK_Timer=(Mtr_Numb_ID*40);             //挂表延时 防止开关电源同时动作 电流太大 拉低电源
      NEW_CMD=1;                               //首次进入试验标志 
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);          //显示同一设置 清显示
      Update_N_Buf();                          //更新圈数区 
      Update_Mtr_Num();                        //更新表位号区
     }
} 
/********************************************************
* 脉冲波形测试 处理 
* 进入工作模式处理程序后 
********************************************************/
void MEA_PLS_CYCLE_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    WORK_MODE=MEA_ENG_DUTY_M;                  //进入周期,占空比测量试验
    NEW_CMD=1;                                 //新命令标志 
    WORK_Timer=(Mtr_Numb_ID*40);               //挂表延时 防止开关电源同时动作 电流太大 拉低电源
}  
/********************************************************
* 接线方式 处理 
* 功耗测试用
* 也可用在 四象限电能脉冲输入处理
********************************************************/
void WIRE_TYPE_Pr(void)
{
    u8 Mode;
    Mode=CAN_MSG_IPtr->Data.BYTE[0];           //接收接线方式
    if((Mode<WIRE_P1)||                        //判断是否为非法设置
       (Mode>WIRE_Q1)||
       (Mode==WIRE_TYPE))                      //判断接线方式是否改变
     return;                                   //退出	 
    WIRE_TYPE=Mode;
    NEW_CMD=1;                                 //新命令标志 
    WORK_Timer=(Mtr_Numb_ID*40);               //延时 防止开关电源同时动作 电流太大 拉低电源
}  
/********************************************************
* 被校表脉冲输出类型选择 处理 
* 电子脉冲共高共低选择
* GD_E    '1'      共发射集         
* GD_C    '2'      共集电极
********************************************************/
void ELEC_PLS_TYPE_Pr(void)
{
    u8 m;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                    //不在设置范围 退出	 	
    m=CAN_MSG_IPtr->Data.BYTE[2];
    if(m==PLSGD)                                //判断脉冲输出类型是否改变
     return;	
    if(m==GD_E)                                 //是否为共E
     DOWN_JOIN; 
    else if(m==GD_C)                            //是否为共C
     UP_JOIN;                             
    else
     return;
    PLSGD=m;                                    //更新脉冲输出类型 
}  
/********************************************************
* 设置走字脉冲数 处理 
* 定脉冲走字
* 参数:4字节长整型
********************************************************/
void Set_ZZ_PLS_Pr(void)
{
    u32 m;	
    memcpy(&m,                                  //数据拷贝
           &CAN_MSG_IPtr->Data.BYTE,            
           4);                                  
    if((m==0)||                                 //判断是否为非法设置
       (m==ZZ_PLS_Set))                         //走字脉冲数是否改变
     return;	                                  
    ZZ_PLS_Set=m;                               //更新走字脉冲数
}                                  
/********************************************************
* 脉冲走字试验 处理 
********************************************************/
void PLS_ZZ_Pr(void)
{
    if(!MTR_PLUG)                         //判断是否挂表
     return;                              //不挂表退出
    if(WORK_MODE==PULSE_ZZ_M)             //判断是否在脉冲走字试验状态
     return;                              //退出
    WORK_MODE=PULSE_ZZ_M;                 //设置为脉冲走字试验状态
    NEW_CMD=1;                            //新命令标志 
    WORK_Timer=(Mtr_Numb_ID*40);          //挂表延时 防止开关电源同时动作 电流太大 拉低电源
/*
    else                                  //判断是否停止脉冲走字试验
     {
      if(WORK_MODE!=PULSE_ZZ_M)           //判断是否在脉冲走字试验状态
       return;                            //不在脉冲走字试验状态 退出
      UJDQ_Open_ESwitch_Open();           //停止脉冲计数 断开电压  
     }
*/     
}      
/********************************************************
* 预置脉冲数 处理 
********************************************************/
void ReSet_ZZ_PLS_Pr(void)
{
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                     //不在设置范围 退出	 	
    memcpy(&CURRENT_PLS_CNT,                     //数据拷贝
           &CAN_MSG_IPtr->Data.BYTE[2],            
           4);                                  
    ZZ_PLS_LOADED=1;	        
}      
/********************************************************
* 开始耐压试验   处理 
* 进入耐压状态，耐压指示灯绿色
********************************************************/
void Start_Ny_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    WORK_MODE= NYSY_M;                         //设置为耐压试验状态
    NY_CHK_Timer=Timer_8ms;                    //耐压结果查询定时
    NY_SEND_Timer=(u16)Timer_1ms;
    NY_SEND_Timer-=NY_SEND_TIME;               //结果回送定时
    NY_RESULT=NY_UNKW;                         //重置耐压查询结果						
				if(SOLID_CFG.LED_NUM==LED_6)         //是否为6位数码管
           {	
            Disp_Buf[0]=DISP_BLANK;                //显示'0'
            Disp_Buf[1]=DISP_BLANK;                //显示'P'
            Disp_Buf[2]=DISP_BLANK;                //显示'N'
         
            Disp_Buf[3]=DISP_BLANK;           //显示空白
            	     	
            Disp_Buf[4]=DISP_H;       //中位
            Disp_Buf[5]=DISP_U;       //低位
           } 
          else
           {	
            Disp_Buf[0]=DISP_BLANK;                //显示'0'
            Disp_Buf[1]=DISP_BLANK;                //显示'P'
            Disp_Buf[2]=DISP_BLANK;                //显示'N'
         
            Disp_Buf[3]=DISP_BLANK;           //显示空白
            	     	
            Disp_Buf[4]=DISP_BLANK;       //中位
            Disp_Buf[5]=DISP_BLANK;       //低位
            Disp_Buf[6]=DISP_H;       //中位
            Disp_Buf[7]=DISP_U;       //低位												
           } 
} 
/********************************************************
* 耐压时间到 命令 处理 
********************************************************/
void Ny_Time_End_Pr(void)
{
    return;
} 
/********************************************************
* 切换表尾电压端子  处理 
* 0:接入1号端子(默认) 1:接入3号端子 
********************************************************/
void Chg_U_In_Pr(void)
{
    if(!MTR_PLUG)                              //判断是否挂表
     return;                                   //不挂表退出
    WORK_MODE= GDGZ_M;                         //设置为接地故障试验状态
    WORK_Timer=(Mtr_Numb_ID*40);               //进入工作状态延时 防止开关电源同时动作 电流太大 拉低电源
    NEW_CMD=1;                                 //新命令标志
}              
/********************************************************
* 误差校验指示灯控制命令  处理 
********************************************************/
void STS_Light_Pr(void)
{
    if(CAN_MSG_IPtr->Data.BYTE[0]=='0')        //判断校验指示灯是否亮
     TEST_LAMP_OFF;                            //灭
    else
     TEST_LAMP_ON;                             //亮
}
/********************************************************
* 接收失压相别
* 0x3y  字节最低三位代表A B C三相
********************************************************/
void Set_Sy_Ph(void)  
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //失压相
    m&=0x07;                                   
    if(m==0)                                   //非法设置 默认都失压
     m=ALL_PHASE;                              
    SY_PHASE=m;                                //跌落相选择
}               
/********************************************************
* 设置最小电表常数
********************************************************/
void Set_Min_CST_Pr(void)            
{
    float f;
    memcpy(&f,
           &CAN_MSG_IPtr->Data.BYTE,
           4);
    if((f==0)||                                //判断是否合法
       (f==MTR_MIN_CST))                       //最小电表常数是否改变 
     return;                                   
    MTR_MIN_CST=f;                             //更新最小电表常数
    Cal_STD_ENG_CNT_VAL();                     //计算实际校验圈数
}               
/********************************************************
* 接收校核常数走字度数 
********************************************************/
void Set_ZZ_Ds(void)                      
{
    float f;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                     //不在设置范围 退出	 	
    memcpy(&f,
           &CAN_MSG_IPtr->Data.BYTE[2],
           4);
    if(f==0)                                     //走字电能不能为0
     f=0.1;                                      
    if(f==VERIFY_ENG_Kwh)                        //判断走字电能是否发生改变
     return;
    VERIFY_ENG_Kwh=f;
    f*=MTR_ENG_CST;                              //脉冲数
    f/=DIVIDE_Coef;                              //除以分频系数 2012.4.15更改
    f+=0.5;
    VERIFY_PLS_Set=(u32)f;                       //换算成脉冲数
    CURRENT_VERIFY_PLS=VERIFY_PLS_Set;           //初始化当前走字脉冲数
    sprintf((char*)TEMP_STR,
            "%6.3f",
            VERIFY_ENG_Kwh);                     //显示 走字度数
    Fill_Space_StrCpy(TEMP_STR,                  //待拷贝的字符串
                      VERIFY_ENG_ASC,            //待写入的字符串
                      7);                        //字符串最大长度                 
}               
/********************************************************
* 设置脉冲合成方式   
* HC_2 '1'  合成两路 正反有功合成 正反无功合成
* HC_3 '2'  合成三路 正反有功合成 正反无功分开
* HC_4 '3'  合成四路
********************************************************/
void Set_PLSHC(void)
{
    u8 m;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1]))    //判断是否在设置表位范围
     return;                                        //不在设置范围 退出	 	
    m=CAN_MSG_IPtr->Data.BYTE[2];
    if(m==PLSHC_MODE)                               //合成方式没有改变 退出
     return;	
    if((m==HC_2)||                                  //判断是否为二路脉冲
       (m==HC_3)||	                                //判断是否为三路脉冲													 
       (m==HC_4))                                   //判断是否为四路脉冲											 
     {
      if(m==PLSHC_MODE)                             //合成方式是否改变
       return;	
      PLSHC_MODE=m;                                 //更新脉冲合成方式
      SEL_ELEPLS_IN();                              //电子脉冲输入处理
     }  
}  
/********************************************************
* 接收失压方案
********************************************************/
void Set_Sy(void)                    
{
    u8 mode;
    mode=CAN_MSG_IPtr->Data.BYTE[0];           //跌落方案	
    if((mode<VLOSS_1)||                        //是否超出跌落方案
       (mode>VLOSS_3)||                        //数据非法
       (mode==SY_MODE))                        //方案不变 退出
     return;
    SY_MODE=mode;        
}    
/********************************************************
* 电流回路开路检测  处理 
********************************************************/
void I_Open_Tst_Pr(void) 
{

} 
/********************************************************
* 设置多功能脉冲方式
* '1' UNION_PLS 联合多功能脉冲 时钟 需量 投切 等共用
* '2' ALONE_PLS 独立多功能脉冲 时钟 需量 投切 分开输入         
********************************************************/
void Set_MFCLK_Mode(void)
{
    u8 mode;
    mode=CAN_MSG_IPtr->Data.BYTE[0];           //多功能脉冲方式
    if((mode==UNION_PLS)||                     //是否为联合脉冲
       (mode==ALONE_PLS))                      //是否为独立脉冲
     {  
      MFClk_Mode=mode;                         //更新多功能脉冲接入方式
      MFuction_Clk_Cfg();                      //初始化多功能脉冲接口
     } 
}
/********************************************************
* 设置当前多功能脉冲输入类型
* 只有在联合脉冲输入方式下有效
* SZCLK_PLS    '1'      当前输入为时钟脉冲
* XULCLK_PLS   '2'      当前输入为需量脉冲
* TQCLK_PLS    '3'      当前输入为投切脉冲
********************************************************/
void Set_MFCLK_Type(void)
{
    u8 type;
    type=CAN_MSG_IPtr->Data.BYTE[0];           //多功能脉冲输入类型
    if((type==SZCLK_PLS)||                     //是否为时钟脉冲
       (type==XULCLK_PLS)||                    //是否为需量脉冲
       (type==TQCLK_PLS))                      //是否为投切脉冲
     {
      CANT_STR[0]=MFClk_Type;                  //保存原先脉冲类型  
      MFClk_Type=type;                         //更新多功能脉冲输入类型
      type=CANT_STR[0]; 					   //
      if(MFClk_Mode==UNION_PLS)                //判断是否为联合脉冲
       if((MFClk_Type==SZCLK_PLS)||            //
          (type==SZCLK_PLS))                   //端口工作方式改变	 
        MFuction_Clk_Cfg();                    //初始化多功能脉冲接口
     }         
}
/********************************************************
* 设置电子脉冲输入 PA PR QA QR 调试用 
* 只有在电子脉冲输入方式下有效
* PA_PLS           '0'      正向有功
* QA_PLS           '1'      正向无功
* PR_PLS           '2'      反向有功
* QR_PLS           '3'      反向无功
********************************************************/
void Set_Elec_Pls(void)
{
    u8 m;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                    //不在设置范围 退出	 	
    if(CAN_MSG_IPtr->CTL.BIT.LEN!=3)//数据长度是否错误
     return;
    m=CAN_MSG_IPtr->Data.BYTE[2];   //电子脉冲输入类型
    if((m<PA_PLS)||                 
       (m>QR_PLS)||                 //判断参数是否错误
       (m==PLS_QUAD))               //判断脉冲象限是否改变   
     return;                        
                     
    PLS_QUAD=m;                     //更新 	 	  
/*				
    if(m==PA_PLS)                   //判断是否为正向有功 
     POS_Watt_SEL;                  //选择正向有功脉冲
    else if(m==PR_PLS)              //判断是否为反向有功
     NEG_Watt_SEL;                  //选择反向有功脉冲
    else if(m==QA_PLS)              //判断是否为正向无功 
     POS_Var_SEL;                   //选择正向无功脉冲
    else if(m==QR_PLS)              //判断是否为反向无功 
     NEG_Var_SEL;                   //选择反向无功脉冲
*/				
} 
/********************************************************
* 复位电流旁路继电器
* 2011.5.19 zlz 修改 挂表状态下执行该指令
********************************************************/
void RST_IJDQ(void)                 
{
    u8 m;
    if(MTR_PLUG)                              //判断是否挂表
     {                                        //挂表
      I_JDQ=1;                                //设置原先继电器状态为吸合 保证动作
      I_In_Out_Pr(1);                         //电流接入 电流继电器断开
      TZZS_LED_OFF;                           //关闭跳闸指示灯
      NZTZ_FLAG=0;                            //跳闸恢复 清除
      GZ_FLAG=0;                              //清除故障状态标志
      m=Disp_Choose;                          //暂存显示模式
      Disp_Choose=DISP_TEST_DATA;             //更新显示模式
      memset(Disp_Buf,DISP_BLANK,8);          //显示同一设置 清显示
      Update_N_Buf();                         //更新圈数区 
      Update_Mtr_Num();                       //更新表位号区
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
      Disp_Choose=m;                          //更新显示方式 
     } 
} 
/********************************************************
* 测试命令显示8.8.8.8.8.8.8.8.
********************************************************/
void LED_TST(void)
{
    memset(Disp_Buf,DISP_ALL,8);
    Disp_Timer=(Timer_8ms-DISP_TIME);          //更新显示
//    Disp_Data(Disp_Code_Mode);                 //显示 按方式0译码(与原误差板译码方式相同 
}
/********************************************************
* 复位7279
********************************************************/
void RST_LED(void)
{
    Reset_HD7279();
    memset(Disp_Buf,DISP_BLANK,8);             //显示同一设置 清显示
    Update_N_Buf();                            //更新圈数区 
    Update_Mtr_Num();                          //更新表位号区
    Disp_Timer=(Timer_8ms-DISP_TIME);          //更新显示
//    Disp_Data(Disp_Code_Mode);                 //显示 按方式0译码(与原误差板译码方式相同 
}
/******************************************************
* 合闸检测使能命令
******************************************************/
void Set_TZEN(void)
{
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                     //不在设置范围 退出	
    if(CAN_MSG_IPtr->Data.BYTE[2]=='0')          
     TZEN=0;                                     //跳闸检测禁能
    else if(CAN_MSG_IPtr->Data.BYTE[2]=='1')
     TZEN=1;                                     //跳闸检测使能
}                           
/******************************************************
* 表报警信号检测使能 
******************************************************/
void Set_MBJEN(void)                   
{
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                     //不在设置范围 退出	 	
    if(CAN_MSG_IPtr->Data.BYTE[2]=='0')          
     MBJEN=0;                                    //表报警信号检测禁能
    else if(CAN_MSG_IPtr->Data.BYTE[2]=='1')
     MBJEN=1;                                    //表报警信号检测使能
}
/******************************************************
* 单三相台设置命令
******************************************************/
void Set_TS(void)
{
    u8 Sts=0;
    if(CAN_MSG_IPtr->Data.BYTE[0]=='3')          //是否设置为三相台
     {	         
      if(SINGLE_OR_THREE!=THREE)                 //单三相设置发生改变
       {
        SINGLE_OR_THREE=THREE;  
        Sts=0xFF;
       }
      DXTZ&=0x7F;                                //三相台                                  
     } 
    else if(CAN_MSG_IPtr->Data.BYTE[0]=='1')
     {	
      if(SINGLE_OR_THREE!=SINGLE)                //台体状态改变
       {
        SINGLE_OR_THREE=SINGLE;  
        Sts=0xFF;
       }
      DXTZ|=0x80;                                //单相台
     }
    else
     return;        
    if(Sts)                                      //状态改变 重置电压继电器状态
     {	  
      Sts=U_ESwitch[0];                          //A相状态
      ESwitch_Control_Pr(ALL_PHASE,              //三相电压设置
                         ON,                     //电子开关预置 吸合
                         Sts);                   //
      Sts=U_JDQ[0];                              //A相状态
      UJDQ_Control_Pr(ALL_PHASE,                 //三相电压设置
                      Sts);                      //	
     }                 			       
}
/******************************************************
* 设置单相跳闸
* 0 单相费控表跳闸继电器为常开触点
* 1 单相费控表跳闸继电器为常闭触点
* 2 不为单相费控表,跳闸继电器为双触点 单相表默认常开触点
* 3 强制为双触点
******************************************************/
void Set_DXTZ(void)
{
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                     //不在设置范围 退出	 	
    if(CAN_MSG_IPtr->Data.BYTE[2]=='0')          //          
     DXTZ=0x80;                                  //单相费控表跳闸继电器触点为常开
    else if(CAN_MSG_IPtr->Data.BYTE[2]=='1')
     DXTZ=0x81;                                  //单相费控表跳闸继电器触点为常闭
    else if(CAN_MSG_IPtr->Data.BYTE[2]=='2')     // 
     {	
      if(SINGLE_OR_THREE)
       DXTZ=0;                                   //双触点
      else
       DXTZ=0x80;                                //单相台默认为常开触点	 
     }
    else if(CAN_MSG_IPtr->Data.BYTE[2]=='3')     //非常规国网表，外置跳闸继电器为单触点，接常开点
     DXTZ=0x02; 
    else if(CAN_MSG_IPtr->Data.BYTE[2]=='4')     //非常规国网表，外置跳闸继电器为单触点，接常闭点
     DXTZ=0x04;  
}
/********************************************************
* 总控中心启动命令
********************************************************/
void Set_Master_Start(void)
{
    MASTER_START=1;                             	//总控中心已经启动标志
}
/********************************************************
* 设置多功能表(模拟表),负控终端串口通信参数
* 入口:Com 端口
********************************************************/
void Set_MTR_LCT_Com(u8 Com) 
{
    UART_SET Uart_Set;
    if(Com>IRECOM)                        //判断端口设置是否合法
     return;
    memcpy(&Uart_Set,
           &CAN_MSG_IPtr->Data.BYTE,
           4);
    if(Uart_Set.STOP>1)                   //判断停止位是否大于1
     Uart_Set.STOP=1;
    else
     Uart_Set.STOP=0;
    if(Uart_Set.LEN>UART_8_BIT)          //判断是否设置错误
     Uart_Set.LEN=UART_8_BIT;            //数据位数 默认8位
    if(Uart_Set.PARITY>UART_S_PARITY)
     Uart_Set.PARITY=UART_E_PARITY;      //设置错误按偶校验
    if((Uart_Set.BAUD<MIN_BAUD)||
       (Uart_Set.BAUD>MAX_BAUD))
     Uart_Set.BAUD=1200;                 //默认波特率	 	      	  
    memcpy(&Uart_Para[Com],
           &Uart_Set,
           4);
    Init_Com(Com);
}
/********************************************************
* 设置各表位时钟频率    
********************************************************/
void Set_Clk_Freq_Pr(void)
{
    float f;
    memcpy(&f,
           &CAN_MSG_IPtr->Data.BYTE,
           4);
//    if((f==0)||                                //判断是否合法
//       (f==CLK_FREQ_SET))                      //判断时钟频率是否改变
    if(f==0)
     return;
    CLK_FREQ_SET=f;                            //更新时钟频率
    CLK_FREQ_INT=f;                            //设置频率规格化值
    Cal_Clk_Reload(1);                         //重新计算时钟频率重装值和重装次数 重装    
}           
/********************************************************
* 设置该表位时钟频率测量时间   
********************************************************/
void Set_Clk_Time_Pr(void) 
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //时钟频率测量时间 
    if(m<6)                                    //判断测量时间设置是否太小 小于6s  
     m=6;                                                                         
    else if(m>250)                             //判断测量时间设置是否太大 大于250s
     m=250;
//    if(m==CLK_MEA_TIME)                        //判断测量时间是否改变
//     return;	     
    CLK_MEA_TIME=m;
    Cal_Clk_Reload(1);                         //重新计算时钟频率重装值和重装次数 重装    
}           
/********************************************************
* 时钟频率测量控制  
* MEA_STOP         '0'      //停止测量
* MEA_ORDER        '1'      //正常工作
********************************************************/
void Set_Clk_Ctl_Pr(void)            
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //时钟频率测量控制 
    if(m==CLK_MEA_CTL)                         //测量控制是否改变
     return;                                   //设置非法退出
    if((m==MEA_STOP)||                         //停止测量和上传
       (m==MEA_ORDER))                         //正常测量上传
     {	 
      CLK_MEA_CTL=m;                           //更新测量控制
      if(MFClk_Mode==UNION_PLS)                //是否为联合脉冲
       if(MFClk_Type!=SZCLK_PLS)               //是否设置不为测量时钟脉冲
        return;                                //是否为独立脉冲
      if(CLK_MEA_CTL==MEA_ORDER)               //判断是否设置为正常测量状态
       {	
        FIRST_CLK_PLS=1;                       //重新测量 
        if(WORK_MODE==MEA_ENG_DUTY_M)          //是否在脉冲周期测量状态
         return;
        TimerDisable(TIMER2_BASE,TIMER_A);     //Timer2-A禁能
        TimerIntClear(TIMER2_BASE,TIMER_CAPA_MATCH);//清溢出中断
        TimerLoadSet(TIMER2_BASE,TIMER_A,0xFFFF);//重置计数 0xFFFF
        TimerEnable(TIMER2_BASE,TIMER_A);      //Timer1-B启动
       }
     } 
}           
/********************************************************
* 需量周期测量控制   
* MEA_STOP         '0'      //停止测量
* MEA_ORDER        '1'      //正常工作
********************************************************/
void Set_XuL_Ctl_Pr(void)            
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //需量周期测量控制
    if(m==XUL_MEA_CTL)                         //测量控制是否改变
     return;                                   //设置非法退出
    if((m==MEA_STOP)||                         //停止测量和上传
       (m==MEA_ORDER))                         //正常测量上传
     {	 
      XUL_MEA_CTL=m;                           //更新测量控制
      if(MFClk_Mode==UNION_PLS)                //是否为联合脉冲
       if(MFClk_Type!=XULCLK_PLS)              //是否设置不为测量需量脉冲
        return;                                //是否为独立脉冲
      if(XUL_MEA_CTL==MEA_ORDER)               //判断是否设置为正常测量状态
       {	
        FIRST_XUL_PLS=1;                       //重新测量
        if(WORK_MODE==MEA_ENG_DUTY_M)          //是否在脉冲周期测量状态
         return;
        TimerDisable(TIMER2_BASE,TIMER_A);     //Timer2-A禁能
			    	TimerIntClear(TIMER2_BASE,TIMER_CAPA_MATCH);//清溢出中断
				    TimerLoadSet(TIMER2_BASE,TIMER_A,0xFFFF);//重置计数 0xFFFF
				    TimerEnable(TIMER2_BASE,TIMER_A);      //Timer2-A启动
       }
     } 
}           
/********************************************************
* 设置需量周期测量个数  
********************************************************/
void Set_XuL_Pls_Num(void)   
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //需量周期测量个数 
    if(m==0)                                   //需量周期测量脉冲个数不能为0
     m=1;	                                     
    else if(m>100)                             //需量周期测量脉冲个数不能大于100
     m=100;	 
    if(m==XUL_RELOAD_TIME)                     //判断是否改变
     return;	
    XUL_RELOAD_TIME=m;                         //更新需量设定脉冲数
    if(MFClk_Mode==UNION_PLS)                  //是否为联合脉冲
     if(MFClk_Type!=XULCLK_PLS)             	 //是否设置不为测量需量脉冲
      return;                                  //是否为独立脉冲
    FIRST_XUL_PLS=1;                           //重新测量 
}  
/********************************************************
* 多功能脉冲端口管脚初始化处理
********************************************************/
void MFuction_Clk_Cfg(void)
{
    u8  ucPins;                                //管脚定义
    u32 GPIOx;                                 //端口
    u32 TIMERx;                                //计数器
    u32 ulTimer,ulIntFlags;                    //TIMER_A TIMER_B 标志
    if(MTYPE==SOUTH)                           //判断是否为南网表
     {	
      ucPins=CYCLE_PIN_TAB[DZ_PLS-GDT_PLS];    //选择电子脉冲输入口作为 时钟脉冲输入 南网表
      GPIOx =CYCLE_PORT_TAB[DZ_PLS-GDT_PLS];   //选择电子脉冲输入口作为 时钟脉冲输入 南网表
      TIMERx=TIMER5_BASE;                      //默认的电子脉冲计数器 
      ulTimer=TIMER_A;                         //默认A
      ulIntFlags=TIMER_CAPA_MATCH;             //匹配溢出标志
     } 
    else                                       //国网表
     {	                                      
      ucPins=CYCLE_PIN_TAB[SZ_PLS-GDT_PLS];    //选择时钟脉冲输入口作为 时钟脉冲输入 国网表
      GPIOx =CYCLE_PORT_TAB[SZ_PLS-GDT_PLS];   //选择时钟脉冲输入口作为 时钟脉冲输入 国网表
      TIMERx=TIMER4_BASE;                      //默认的时钟脉冲计数器 
      ulTimer=TIMER_B;                         //默认B
      ulIntFlags=TIMER_CAPB_MATCH;             //中断溢出标志
     }
#ifdef PULSE                                   //是否为脉冲检测程序
    TimerDisable(TIMERx,
                 ulTimer); 
    TimerIntClear(TIMERx,                      //清溢出中断
                  ulIntFlags);
    GPIOIntClear(GPIOA,                        //端口 清除端口中断 按键中断
                 0xFF);	                       //管脚
    GPIOIntClear(GPIOM,                        //端口 清除端口中断 光电头脉冲 电子脉冲 时钟脉冲 需量 投切 合闸
                 0xFF);	                       //管脚
    GPIOIntDisable(XL_MC_GPIO,                 //端口 设置管脚中断禁能
                   XL_MC);
    GPIOIntDisable(TQ_MC_GPIO,                 //端口 设置管脚中断禁能
                   TQ_MC);                     //管脚	
    GPIOIntDisable(GPIOx,                      //端口 设置管脚中断禁能
                   ucPins);	                   //管脚	
    GPIOPinTypeGPIOInput(GPIOx,                //端口
                         ucPins);              //管脚PB.1 设为I/O口输入
    GPIOIntTypeSet(GPIOx,                      
                   ucPins,                      
                   GPIO_FALLING_EDGE);         //下降沿中断 先测量高电平时间
    SZ_INT_REEN=1;                             //时钟脉冲口中断重新使能
    SZ_Timer=Timer_8ms;                        //重开启中断定时
#else
    if(WORK_MODE==MEA_ENG_DUTY_M)              //在周期测量状态 不设置端口
     return;
    TimerDisable(TIMERx,
                 ulTimer); 
    TimerIntClear(TIMERx,                      //清溢出中断
                  ulIntFlags);
    GPIOIntClear(GPIOA,                        //端口 清除端口中断 按键中断
                 0xFF);	                       //管脚
    GPIOIntClear(GPIOM,                        //端口 清除端口中断 光电头脉冲 电子脉冲 时钟脉冲 需量 投切 合闸
                 0xFF);	                       //管脚
    if(MFClk_Mode==UNION_PLS)                  //判断多功能脉冲是否为联合脉冲
     {
      GPIOIntDisable(XL_MC_GPIO,               //端口 设置管脚中断禁能
                     XL_MC);	                  //管脚	
      GPIOIntDisable(TQ_MC_GPIO,               //端口 设置管脚中断禁能
                     TQ_MC);	                  //管脚	
      GPIOIntDisable(GPIOx,                    //端口 设置管脚中断禁能
                     ucPins);	                 //管脚	
      if(MFClk_Type==SZCLK_PLS)                //判断是否为测量时钟脉冲周期
       {                                       //用作定时器输入 	
        GPIOPinTypeTimer(GPIOx,                //端口
                         ucPins);              //管脚 设为计数器输入
        TimerLoadSet(TIMERx,                   //定时器
                     ulTimer,                  //通道
                     CLK_RELOAD_VAL_N);        //设置重装值(分频系数)
        TimerMatchSet(TIMERx,                  //定时器
                      ulTimer,                 //通道
                      0);
        TimerIntEnable(TIMERx,                 //定时器
                      ulIntFlags);             //
        TimerEnable(TIMERx,                    //定时器
                    ulTimer);                  //定时器启动
       }
      else 
       {
        GPIOPinTypeGPIOInput(GPIOx,            //端口
                             ucPins);          //管脚设为I/O口输入
        GPIOIntTypeSet(GPIOx,
                       ucPins,
                       GPIO_FALLING_EDGE);     //下降沿中断 先测量高电平时间
        SZ_INT_REEN=1;                         //时钟脉冲口中断重新使能
        SZ_Timer=Timer_8ms;                    //重开启中断定时
       }	                   
     }
    else                                       //多功能脉冲为独立脉冲
     {
      GPIOPinTypeTimer(GPIOx,                  //端口
                       ucPins);                //管脚 PB.1 设为计数器输入
      TimerLoadSet(TIMERx,                     //定时器
                   ulTimer,                    //通道
                   CLK_RELOAD_VAL_N);          //设置重装值(分频系数)
      TimerMatchSet(TIMERx,                    //定时器
                    ulTimer,                   //通道
                    0);                        
      TimerIntEnable(TIMERx,                   //定时器
                     ulIntFlags);              //
      TimerEnable(TIMERx,                      //定时器
                  ulTimer);                    //定时器启动
      GPIOIntTypeSet(XL_MC_GPIO,
                     XL_MC,            
                     GPIO_FALLING_EDGE);       //下降沿中断 
      GPIOIntTypeSet(TQ_MC_GPIO,
                     TQ_MC,            
                     GPIO_FALLING_EDGE);       //下降沿中断 
      XUL_INT_REEN=1;                          //开启需量中断
      TQ_INT_REEN=1;                           //开启投切中断
      XUL_Timer=Timer_8ms;                     //开启需量中断定时
      TQ_Timer=Timer_8ms;                      //开启投切中断定时
     }	 		
#endif
}           
/********************************************************
* 设置显示模式
* DISP_TEST       '0'  显示试验状态
* DISP_CFREQ      '1'  显示时钟频率
* DISP_CERR       '2'  显示日计时误差
* DISP_XULT       '3'  显示需量周期
* DISP_TQMC       '4'  显示投切脉冲
********************************************************/
void Set_Disp_Mode(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];              //设置显示模式
    if((m>DISP_HZMC)||                         //判断设置是否非法
       (m<DISP_TEST_DATA)||                    //
       (m==Disp_Choose))                       //判断显示方式是否改变    	
     return;
    Disp_Choose=DISP_TEST_DATA;	               //
    if(GZ_FLAG||NZTZ_FLAG)                     //是否处于故障和跳闸状态
     return;                                   //退出 不再显示其他内容	 
    memset(Disp_Buf,DISP_BLANK,8);             //显示同一设置 清显示
    Update_N_Buf();                            //更新圈数区 
    Update_Mtr_Num();                          //更新表位号区
    Disp_Timer=(Timer_8ms-DISP_TIME);          //更新显示
    Disp_Choose=m;                             //更新显示方式
//    Disp_Data(Disp_Code_Mode);                 //按方式0译码(与原误差板译码方式相同)
}    
/******************************************************
* 重新装载模拟表数据     上位机   -> 模拟表
* 误差单元命令 提高数据发送速度 命令误差单元重发上次数据->模拟表
******************************************************/
void Set_Reload_MtrD(void)
{
    if(Com_Tx_Sts[MTRCOM]!=COM_TX_NO)     //发送缓冲区不在空闲状态 退出
     return;
    if(Com_OHead[MTRCOM]!=0)              //发送缓冲区收到过有效数据
     {
      if(Com_Tx_Sts[MTRCOM]!=COM_TX_NO)   //是否空闲
       return;	
      Com_OTail[MTRCOM]=0;
      Com_Tx_Sts[MTRCOM]=COM_TX_EN;       //发送缓冲区有效 
     }	 
}
/******************************************************
* 重新装载负控终端数据   上位机   -> 负控终端
* 误差单元命令 提高数据发送速度 命令误差单元重发上次数据->负控终端
******************************************************/
void Set_Reload_LctD(void)
{
    if(Com_Tx_Sts[LCTCOM]!=COM_TX_NO)     //发送缓冲区不在空闲状态 退出
     return;
    if(Com_OHead[LCTCOM]!=0)              //发送缓冲区收到过有效数据
     {
      if(Com_Tx_Sts[LCTCOM]!=COM_TX_NO)   //是否空闲
       return;
      Com_OTail[LCTCOM]=0;
      Com_Tx_Sts[LCTCOM]=COM_TX_EN;       //发送缓冲区有效    
     }	 
}
/******************************************************
* 重新上传模拟表数据   模拟表   -> 上位机 
* 误差单元命令 提高数据发送速度 命令误差单元重新上传上次收到的模拟表数据
******************************************************/
void Set_ReTx_MtrD(void)
{
    if(Com_Rx_Sts[MTRCOM]!=COM_RX_NO)    //接收缓冲区不在空闲状态 退出
     return;                             //串口不在接收状态退出
    if(Com_IHead[MTRCOM]!=0)             //判断是否收到有效数据
     {	
      if(Com_Rx_Sts[MTRCOM]!=COM_RX_NO)  //是否为空闲
       return;      
      Com_ITail[MTRCOM]=0;               //发送处理指针清零
      Com_Rx_Sts[MTRCOM]=COM_RX_END;     //接收结束状态 收到有效数据等待处理
     } 
}
/******************************************************
* 重新上传负控终端数据   负控终端   -> 上位机     
* 误差单元命令 提高数据发送速度 命令误差单元重新上传上次收到的负控终端数据
******************************************************/
void Set_ReTx_LctD(void)
{
    if(Com_Rx_Sts[LCTCOM]!=COM_RX_NO)    //接收缓冲区不在空闲状态 退出
     return;                             //串口不在接收状态退出
    if(Com_IHead[LCTCOM]!=0)             //判断是否收到有效数据
     {	
      if(Com_Rx_Sts[LCTCOM]!=COM_RX_NO)  //是否为空闲
       return;      
      Com_ITail[LCTCOM]=0;               //发送处理指针清零
      Com_Rx_Sts[LCTCOM]=COM_RX_END;     //接收结束状态 收到有效数据等待处理
     } 
}
/******************************************************
* 接收时间基准     
* 格式 HH-MM-SS   时分秒  8字节
******************************************************/
void Set_Time_Base(void)
{
    u8 m;
    for(m=0;m<8;m++)
     TIME_ASC[m]=CAN_MSG_IPtr->Data.BYTE[m]; //拷贝当前时间	
}
/******************************************************
* 设置RS485 第二通道     
* '1' 接表485第二通道
* '2' 接表红外 
******************************************************/
void Set_Red485(void)
{
/*
    if(CAN_MSG_IPtr->Data.BYTE[0]==RED_485)  //判断是否选择红外485
     RED_485_EN;                             //切换到表红外485
    else
     RED_485_DN;                             //切换到表第二485通道 默认
*/
}
/******************************************************
* 读版本号    
******************************************************/
void Set_Read_Ver(void)             
{
    if(Mtr_Numb_ID==1)                       //是否为第1表位
     {
      u8 n,d;
      for(n=0;n<10;)
       {
        d=VER_TAB[n];
        if(d=='\0')
         break;
        TEMP_STR[n]=d; 
        n++;
       }
      if(n==0||n==10)                          //长度错误
       return;	    
      Send_Data(ERR_OCMD_VER,                  //命令
                n,
                TEMP_STR);
     }           
}
/******************************************************
* 读小显示工作状态    
******************************************************/
void Set_RSTS(void)
{
    u8 d;
    d=(u8)WORK_MODE;
    TEMP_STR[0]=((d/10)|'0');                 //状态高位
    TEMP_STR[1]=((d%10)|'0');                 //状态低位
    Send_Data(ERR_OCMD_STS,                   //命令
              2,
              TEMP_STR);
}
/******************************************************
* 根据表类型对脉冲选择进行处理
* 电子脉冲  时钟脉冲 输入互换   
******************************************************/
void Proc_MTYPE(void)
{
	
}
/******************************************************
* 选择表类型 国网表 'N'  南网表'S'  
* 电子脉冲  时钟脉冲 输入互换   
******************************************************/
void Set_MTYPE(void)
{
  u8 m;	
  m=CAN_MSG_IPtr->CTL.BIT.LEN;               //数据长度	
	if(m!=1)                                    //长度是否错误
	 return;
	m=CAN_MSG_IPtr->Data.BYTE[0];               //表类型
	if((m!=SOUTH)&&                             //是否不为南网表
		 (m!=NATIONAL))                            //是否不为国网表
   return;
  MTYPE=m;                                   //更新表类型
  Init_DEVICE_IO();                          //初始化电能脉冲 多功能脉冲 合闸脉冲 IO口
  Proc_MTYPE();  	                           //表类型处理	    	
}
/******************************************************
* 设置表位电压接入 断开     
******************************************************/
void Set_UCLOP(void)
{
    u8 m;
    if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
       (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1])) //判断是否在设置表位范围
     return;                                     //不在设置范围 退出	 	
    if(!MTR_PLUG)                                //判断是否挂表
     return;                                     //不挂表退出
    m=CAN_MSG_IPtr->Data.BYTE[2];                
    if(m==Set_Uclop)                             //电压接入状态是否改变
     return;                                     //没改变退出
    if(m==MTR_UOP)                               //表位继电器控制电压断开
     {                                            
      UJDQ_Open_ESwitch_Open();                  //电压继电器断开 电子开关断开
     }                                            
    else if(m==MTR_UCL)                          //表位继电器控制电压吸合
     {                                            
      UJDQ_Close_ESwitch_Open();                 //电压继电器接入 电子开关断开
     }                                            
    else if(m==MTR_UECL)                         //表位电子开关控制电压闭合
     {                                            
      UJDQ_Open_ESwitch_Close();                 //电压继电器断开 电子开关闭合
     }                                            
    else                                     
     return;                                     
    Set_Uclop=m;                                 //更新电压接入状态
}
/*****************************************************************************
* 载波试验
*****************************************************************************/
void Set_ZBTST(void)
{
    u8 m;
    if(!MTR_PLUG)                                //判断是否挂表
     return;                                     //不挂表退出
    if(SINGLE_OR_THREE)                          //判断是否为三相台
     return;                                     //三相台载表不需特殊处理
    m=CAN_MSG_IPtr->Data.BYTE[0];                //设置显示模式
    if(m=='0')                                   //退出载表通信试验
     {	                                      
      if(WORK_MODE==ZBTX_M)                      //当前是否在载表通信状态
       {	
        WORK_MODE=CAL_ENG_ERR_M;                 //设置为校验误差模式
        CURRENT_PLS_CNT=0;                       //清除脉冲计数
        CURRENT_ENG_KWh=0;                       //清除电能累计
        NEW_CMD=1;                               //新命令标志
        ABBA_HIGH;                               //RS485极性正常						
       } 
     }	
    else if(m=='1')                              //参数设置非法
     {
      if(WORK_MODE!=ZBTX_M)                      //当前是否不在载表通信状态 
       {	
        WORK_MODE=ZBTX_M;                        //进入载表通信状态
        NEW_CMD=1;                               //新命令标志
        WORK_Timer=(Mtr_Numb_ID*40);             //挂表延时 防止开关电源同时动作 电流太大 拉低电源
       } 
     }
    else
     return;
}
/*****************************************************************************
* 误差板状态指示灯控制
* D0 控制状态灯 D0='0' 灯灭 D0='1' 黄灯亮 D0='2' 绿灯亮 D0='3' 红灯亮 
* D1 控制耐压灯 D1='0' 灯灭 D1='1' 黄灯亮 D1='2' 绿灯亮 D1='3' 红灯亮 
* D0='5' D1='5' 计算机放弃控制 由程序控制
*****************************************************************************/
void Set_Light(void)
{
/*	
    u8 m,n,t;
    t=CAN_MSG_IPtr->CTL.BIT.LEN;                 //数据长度
    m=n=0;
    if(t==1)                                     //只有1组灯 1个数据 控制状态灯(误差 压接)
     m=CAN_MSG_IPtr->Data.BYTE[0];               //取指示灯数据
    else if(t==2)
     {	
      m=CAN_MSG_IPtr->Data.BYTE[0];              //取指示灯数据
      n=CAN_MSG_IPtr->Data.BYTE[1];              //取指示灯数据
     }
    else
     return;                                     //参数错误 退出     	     	
    if(m==LGT_OFF)                               //是否为灯灭
     {                                           
     	CD4094_DATA.E_RED=0;                       //误差状态指示红灯控制
     	CD4094_DATA.E_GRN=0;                       //误差状态指示绿灯控制
     }                                           
    else if(m==YLW_ON)                           //是否为黄灯亮
     {                                           
     	CD4094_DATA.E_RED=1;                       //误差状态指示红灯控制
     	CD4094_DATA.E_GRN=1;                       //误差状态指示绿灯控制
     }	 	       		                             
    else if(m==GRN_ON)                           //是否为绿灯亮
     {                                           
     	CD4094_DATA.E_RED=0;                       //误差状态指示红灯控制
     	CD4094_DATA.E_GRN=1;                       //误差状态指示绿灯控制
     }	 	       		                             
    else if(m==RED_ON)                           //是否为红灯亮
     {                                           
     	CD4094_DATA.E_RED=1;                       //误差状态指示红灯控制
     	CD4094_DATA.E_GRN=0;                       //误差状态指示绿灯控制
     }	 	       		
    if(n==LGT_OFF)                               //是否为灯灭
     {                                           
     	CD4094_DATA.N_RED=0;                       //耐压状态指示红灯控制
     	CD4094_DATA.N_GRN=0;                       //耐压状态指示绿灯控制
     }                                             
    else if(n==YLW_ON)                           //耐压为黄灯亮
     {                                             
     	CD4094_DATA.N_RED=1;                       //耐压状态指示红灯控制
     	CD4094_DATA.N_GRN=1;                       //耐压状态指示绿灯控制
     }	 	       		                               
    else if(n==GRN_ON)                           //耐压为绿灯亮
     {                                            
     	CD4094_DATA.N_RED=0;                       //耐压状态指示红灯控制
     	CD4094_DATA.N_GRN=1;                       //耐压状态指示绿灯控制
     }	 	       		                             
    else if(n==RED_ON)                           //是否为红灯亮
     {                                           
     	CD4094_DATA.N_RED=1;                       //误差状态指示红灯控制
     	CD4094_DATA.N_GRN=0;                       //误差状态指示绿灯控制
     }
    if((m=='5')&&(n=='5'))                       //是否为计算机放弃指示灯控制
     LIGHT_CTL_PC=FALSE;                         //计算机控制 CPU 放弃控制
    else
     LIGHT_CTL_PC=TRUE;                          //计算机控制 CPU 接管控制
    CD4094_FLAG=1;    	
    CD4094_Timer=(u8)Timer_1ms;                  //重启定时
*/				
}
/******************************************************
* 设置误差板LED位数 
* SLEDN:6  6位数码管
* SLEDN:8  8位数码管
******************************************************/
void Set_SLEDN(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];
    if(((m==LED_6)||(m==LED_8))&&               //位数是否设置正确
       (m!=SOLID_CFG.LED_NUM))                  //位数是否改变
     {
      SOLID_CFG.LED_NUM=m;
      SOLID_CFG.Flag=YES;                       //标志
      Solid_Save_Tab();                         //固化
      Update_N_Buf();                           //更新圈数区 
      Update_Mtr_Num();                         //更新表位号区
     }	 
}
/******************************************************
*  设置载波模块电压接入相别
*  PLCVSET:A(0dh)
*  A='1' 载波接入A相
*  A='2' 载波接入B相
*  A='3' 载波接入C相
******************************************************/
void Set_VSET(void)
{
	 	
}
/******************************************************
* 设置接入UART0的通讯方式
* COM0SEL:A(0dh)
* A='0' 第一路RS485接入
* A='1' 第三路RS485接入
******************************************************/						
void Set_COM0SEL(void)
{
/*	
  u8 m;
	 m=CAN_MSG_IPtr->Data.BYTE[0];
	 if((m=='0')||(m=='1'))
    {
     if(m=='0')
      {
       CD4094_DATA.COM0SEL=0;
      }
     if(m=='1')
      {
       CD4094_DATA.COM0SEL=1;
      }						
    }
		else
     CD4094_DATA.COM0SEL=0;
		CD4094_FLAG=1;
		CD4094_Timer=(u8)Timer_1ms;
*/	
}
/******************************************************
* 设置接入UART1的通讯方式
* COM1SEL:A(0dh)
* A='0' 第二路RS485接入
* A='1' RS232信道接入
******************************************************/
void Set_COM1SEL(void)
{
/*
  u8 m;
	 m=CAN_MSG_IPtr->Data.BYTE[0];
	 if((m=='0')||(m=='1'))
    {
     if(m=='0')
      {
       CD4094_DATA.COM1SEL=0;
      }
     if(m=='1')
      {
       CD4094_DATA.COM1SEL=1;
      }
    }
   else
    CD4094_DATA.COM1SEL=0;
   CD4094_FLAG=1;
   CD4094_Timer=(u8)Timer_1ms;
*/
}
/******************************************************
* 载波滤波电路接入选择
* ZBLBIN:A(0dh)
* A='0'载波滤波电路不接入
* A='1'载波滤波电路接入
******************************************************/
void Set_ZBLBIN(void)
{

}
/******************************************************
*  遥信信号控制命令
*  YXCTL:XXX,A(0dh)
*  XXX表位号,
*  A:十六进制数,每位代表一个遥信序号;0:代表闭合;1:代表断开
******************************************************/
void Set_YXCTL(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];                 //取出设置数据
    if(MTR_MOD==Smart_Meter)                      //正在检测的设备是智能电表
     return;			    
    GPIOPinWrite(YX_CTL_GPIO,YX_CTL,m);           //遥信信号按照设定值输出    
}
/******************************************************
*  门控信号输出状态控制
*  DOORCTL:XXX,A(0dh)
*  XXX表位号XXX=0，所有表位
*  A='0' 门控信号合
*  A='1' 门控信号开
******************************************************/
void Set_DOORCTL(void)
{
  if((Mtr_Numb_ID<CAN_MSG_IPtr->Data.BYTE[0])||
     (Mtr_Numb_ID>CAN_MSG_IPtr->Data.BYTE[1]))              //判断是否在设置表位范围
   return;                                                  //不在设置范围 退出	 	
  if(CAN_MSG_IPtr->Data.BYTE[2]=='0')                       //          
   DOOR_SIG_CLOSE;                                          //门控信号闭合
  else if(CAN_MSG_IPtr->Data.BYTE[2]=='1')                  
   DOOR_SIG_OPEN;                                           //门控信号断开
}
/******************************************************
*  检测设备类型设置
*  EQPMOD:A(0dh)
*  A='0' 检测设备为专变终端Ⅲ型(2013版)
*  A='1' 检测设备为集中器Ⅰ型(2013版)
*  A='2' 检测设备为电表
*  A='3' 检测设备为专变终端Ⅲ型(2009版)
*  A='4' 检测设备为集中器Ⅰ型(2009版)
*  A='5' 预留
*  A='6' 预留
******************************************************/
void Set_EQPMOD(void)
{
  u8 m;
	 m=CAN_MSG_IPtr->Data.BYTE[0];
	 if((m>='0')||(m<='6'))                     //设置数据是否合法
    {                                          
     MTR_MOD=m;                               //当前正在检测的设备类型 
    }
}
/******************************************************
*  输出脉冲参数设置(针对专变终端)
*  PULSET:XXX,A,BBBB,CCCC,D(0dh)
*  XXX表位号;XXX=0，所有表位
*  A=0全部遥信输出脉冲设置；A=1遥信1输出脉冲设置；A=2遥信2输出脉冲设置
*  BBBB：脉冲周期，单位毫秒，数据格式为ASCII码表示的十进制数
*  CCCC：脉冲宽度，
*  D：电平模式，D=0表示低电平脉冲；D=1表示高电平脉冲
*  重新设置参数后，要默认停止脉冲输出
******************************************************/
void Set_PULSET(void)
{
   PULSE_SET Pulse_Set;
   if(MTR_MOD==Smart_Meter)                   //当前不是在校验终端，退出；
    return;
   memcpy(&Pulse_Set,
          &CAN_MSG_IPtr->Data.BYTE,
          6);          
   if(Pulse_Set.MOD>1)                        //脉冲电平模式非法数据
    Pulse_Set.MOD=1;                          //默认高电平数据
   if(Pulse_Set.NUMB>8)                       //脉冲序号非法数据
    Pulse_Set.NUMB=0;                         //默认所有脉冲设置
   if(Pulse_Set.CYCLE>65535)                  //脉冲周期非法数据
    Pulse_Set.CYCLE=5000;                     //默认5000ms，12个脉冲/分钟
   if(Pulse_Set.WIDTH>65535)                  //脉冲宽度非法数据
    Pulse_Set.WIDTH=80;                       //默认脉冲宽度80ms
   if(Pulse_Set.NUMB==0)                      //全部脉冲设置
    { 
     Pulse1_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     Pulse2_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     Pulse3_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     Pulse4_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     Pulse5_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     Pulse6_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
				 Pulse7_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     Pulse8_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     PulOut1_Count=0;                         //脉冲输出个数清零；
     PulOut2_Count=0;                         //脉冲输出个数清零；
     PulOut3_Count=0;                         //脉冲输出个数清零；
     PulOut4_Count=0;                         //脉冲输出个数清零；
     PulOut5_Count=0;                         //脉冲输出个数清零；
     PulOut6_Count=0;                         //脉冲输出个数清零；
					PulOut7_Count=0;                         //脉冲输出个数清零；
					PulOut8_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin1_Set,&Pulse_Set,6);       //脉冲1设置参数保存
     memcpy(&YaoXin2_Set,&Pulse_Set,6);       //脉冲2设置参数保存
     memcpy(&YaoXin3_Set,&Pulse_Set,6);       //脉冲3设置参数保存
     memcpy(&YaoXin4_Set,&Pulse_Set,6);       //脉冲4设置参数保存
     memcpy(&YaoXin5_Set,&Pulse_Set,6);       //脉冲5设置参数保存
     memcpy(&YaoXin6_Set,&Pulse_Set,6);       //脉冲6设置参数保存
					memcpy(&YaoXin7_Set,&Pulse_Set,6);       //脉冲7设置参数保存
					memcpy(&YaoXin8_Set,&Pulse_Set,6);       //脉冲8设置参数保存
     if(YaoXin1_Set.MOD)                      //遥信1脉冲输出设置为高电平
					{
       MC_OUT1_LOW;                           //遥信1脉冲输出先输出低电平
						 YX_CTL1_LOW;                           //
					}
     else                                     //遥信1脉冲输出设置为低电平
					{
       MC_OUT1_HIGH;                          //遥信1脉冲输出先输出高电平
						 YX_CTL1_HIGH;
					}
     if(YaoXin2_Set.MOD)                      //遥信2脉冲设置为高电平
					{
       MC_OUT2_LOW;                            //遥信2脉冲输出先输出低电平
						 YX_CTL2_LOW;
					}
     else                                     //遥信2脉冲输出设置为低电平
					{
       MC_OUT2_HIGH;                           //遥信2脉冲输出先输出高电平		
						 YX_CTL2_HIGH;
					}
     if(YaoXin3_Set.MOD)                      //遥信3脉冲设置为高电平
					{
						 MC_OUT3_LOW;                            //遥信3脉冲输出先输出低电平
						 YX_CTL3_LOW;
					}
     else                                     //遥信3脉冲输出设置为低电平
					{
       MC_OUT3_HIGH;                           //遥信3脉冲输出先输出高电平
						 YX_CTL3_HIGH;
					}
     if(YaoXin4_Set.MOD)                      //遥信4脉冲设置为高电平
					{
       MC_OUT4_LOW;                            //遥信4脉冲输出首先输出低电平
						 YX_CTL4_LOW;
					}
     else                                     //遥信4脉冲输出设置为低电平
					{
       MC_OUT4_HIGH;                           //遥信4脉冲输出首先输出高电平
						 YX_CTL4_HIGH;
					}
     if(YaoXin5_Set.MOD)                      //遥信5脉冲输出设置为高电平
      MC_OUT5_LOW;                            //遥信5脉冲输出先输出低电平
     else                                     //遥信5脉冲输出设置为低电平
      MC_OUT5_HIGH;                           //遥信5脉冲输出先输出高电平
     if(YaoXin6_Set.MOD)                      //遥信6脉冲设置为高电平
      MC_OUT6_LOW;                            //遥信6脉冲输出先输出低电平
     else                                     //遥信6脉冲输出设置为低电平
      MC_OUT6_HIGH;                           //遥信6脉冲输出先输出高电平
     if(YaoXin7_Set.MOD)                      //遥信7脉冲设置为高电平
      MC_OUT7_LOW;                            //遥信7脉冲输出先输出低电平
     else                                     //遥信7脉冲输出设置为低电平
      MC_OUT7_HIGH;                           //遥信7脉冲输出先输出高电平					
     if(YaoXin8_Set.MOD)                      //遥信8脉冲设置为高电平
      MC_OUT8_LOW;                            //遥信8脉冲输出先输出低电平
     else                                     //遥信8脉冲输出设置为低电平
      MC_OUT8_HIGH;                           //遥信8脉冲输出先输出高电平					
    }                                          
   if(Pulse_Set.NUMB==1)                      //遥信1输出脉冲设置
    {                                           
     Pulse1_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；  
     PulOut1_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin1_Set,&Pulse_Set,6);       //遥信1输出脉冲设置参数保存
     if(YaoXin1_Set.MOD)                      //遥信1脉冲输出设置为高电平
					{
       MC_OUT1_LOW;                           //遥信1脉冲输出先输出低电平
						 YX_CTL1_LOW;                           //
					}
     else                                     //遥信1脉冲输出设置为低电平
					{
       MC_OUT1_HIGH;                          //遥信1脉冲输出先输出高电平
						 YX_CTL1_HIGH;
					}
    }                                         
   if(Pulse_Set.NUMB==2)                      //遥信2输出脉冲设置
    {                                           
     Pulse2_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出；
     PulOut2_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin2_Set,&Pulse_Set,6);       //遥信2输出脉冲设置参数保存
     if(YaoXin2_Set.MOD)                      //遥信2脉冲设置为高电平
					{
       MC_OUT2_LOW;                            //遥信2脉冲输出先输出低电平
						 YX_CTL2_LOW;
					}
     else                                     //遥信2脉冲输出设置为低电平
					{
       MC_OUT2_HIGH;                           //遥信2脉冲输出先输出高电平		
						 YX_CTL2_HIGH;
					}			
    }
   if(Pulse_Set.NUMB==3)                      //遥信3输出脉冲设置
    {
     Pulse3_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出
     PulOut3_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin3_Set,&Pulse_Set,6);	      //遥信3输出脉冲设置参数保存   
     if(YaoXin3_Set.MOD)                      //遥信3脉冲设置为高电平
					{
						 MC_OUT3_LOW;                            //遥信3脉冲输出先输出低电平
						 YX_CTL3_LOW;
					}
     else                                     //遥信3脉冲输出设置为低电平
					{
       MC_OUT3_HIGH;                           //遥信3脉冲输出先输出高电平
						 YX_CTL3_HIGH;
					}		
    }
   if(Pulse_Set.NUMB==4)                      //遥信4输出脉冲设置
    {                                          
     Pulse4_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出
     PulOut4_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin4_Set,&Pulse_Set,6);	      //遥信4输出脉冲设置参数保存  
     if(YaoXin4_Set.MOD)                      //遥信4脉冲设置为高电平
					{
       MC_OUT4_LOW;                           //遥信4脉冲输出首先输出低电平
						 YX_CTL4_LOW;
					}
     else                                     //遥信4脉冲输出设置为低电平
					{
       MC_OUT4_HIGH;                          //遥信4脉冲输出首先输出高电平
						 YX_CTL4_HIGH;
					}	
    }    
   if(Pulse_Set.NUMB==5)                      //遥信5输出脉冲设置
    {
     Pulse5_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出
     PulOut5_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin5_Set,&Pulse_Set,6);	      //遥信5输出脉冲设置参数保存  
     if(YaoXin5_Set.MOD)                      //遥信5脉冲输出设置为高电平
      MC_OUT5_LOW;                            //遥信5脉冲输出先输出低电平
     else                                     //遥信5脉冲输出设置为低电平
      MC_OUT5_HIGH;                           //遥信5脉冲输出先输出高电平	
    } 
   if(Pulse_Set.NUMB==6)                      //遥信6输出脉冲设置
    {
     Pulse6_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出
     PulOut6_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin6_Set,&Pulse_Set,6);	      //遥信6输出脉冲设置参数保存   
     if(YaoXin6_Set.MOD)                      //遥信6脉冲设置为高电平
      MC_OUT6_LOW;                            //遥信6脉冲输出先输出低电平
     else                                     //遥信6脉冲输出设置为低电平
      MC_OUT6_HIGH;                           //遥信6脉冲输出先输出高电平		
    }
   if(Pulse_Set.NUMB==7)                      //遥信7输出脉冲设置
    {
     Pulse7_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出
     PulOut7_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin7_Set,&Pulse_Set,6);	      //遥信7输出脉冲设置参数保存
     if(YaoXin7_Set.MOD)                      //遥信7脉冲设置为高电平
      MC_OUT7_LOW;                            //遥信7脉冲输出先输出低电平
     else                                     //遥信7脉冲输出设置为低电平
      MC_OUT7_HIGH;                           //遥信7脉冲输出先输出高电平		
    }
   if(Pulse_Set.NUMB==8)                      //遥信8输出脉冲设置
    {
     Pulse8_RUN_CTL=0;                        //设置参数时，要自动停止脉冲输出
     PulOut8_Count=0;                         //脉冲输出个数清零；
     memcpy(&YaoXin8_Set,&Pulse_Set,6);	      //遥信8输出脉冲设置参数保存
     if(YaoXin8_Set.MOD)                      //遥信8脉冲设置为高电平
      MC_OUT8_LOW;                            //遥信8脉冲输出先输出低电平
     else                                     //遥信8脉冲输出设置为低电平
      MC_OUT8_HIGH;                           //遥信8脉冲输出先输出高电平		
    }				
}
/******************************************************
*  输出脉冲运行设置(针对专变终端)
*  PULRUN:XXX,AB,CCC(0dh)
*  XXX表位号;XXX=0，所有表位
*  A='1'遥信1脉冲通道设置；A='2'遥信2脉冲通道设置；
*  A='3'遥信3脉冲1通道设置；A='4'遥信4脉冲2通道设置；
*  B='0'停止脉冲输出，B='1'时，开始脉冲输出
*  CCC:脉冲个数设置，ASCII码，根据脉冲的频率和输出脉冲的时间，计算出需要输出的脉冲总数；
******************************************************/ 
void Set_PULRUN(void)
{
    PULNUM_SET Pulnum_Set; 
    if(MTR_MOD==Smart_Meter)                   //当前不是在校验专变终端Ⅲ，退出；
     return;                                   
    memcpy(&Pulnum_Set,                        
           &CAN_MSG_IPtr->Data.BYTE,           
           4);                                 //取出设置数据
    if((Pulnum_Set.NUMB>8)||                   //脉冲序号非法数据
       (Pulnum_Set.QANTY>65535)||              //脉冲个数非法数据
       (Pulnum_Set.MOD>1))                     //脉冲输出控制为非法数据
     return;
    if(Pulnum_Set.NUMB==1)
     {
      if(Pulnum_Set.MOD==1)                    //遥信1脉冲输出控制                                                      
       Pulse1_RUN_CTL=1;                       //遥信1脉冲输出控制位置位
      else
       Pulse1_RUN_CTL=0;                       //遥信1脉冲输出控制位清零				
      PulOut1_RVS_Bit=1;                       //遥信1脉冲输出反转标志置位
      PulOut1_Timer=(u16)Timer_1ms;            //遥信1脉冲输出定时器赋初值
      PulOut1_Count=0;                         //脉冲输出个数清零；
      PulOut1_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；
     }		
    if(Pulnum_Set.NUMB==2)
     {
      if(Pulnum_Set.MOD==1)                    //遥信2脉冲输出控制                                                       
       Pulse2_RUN_CTL=1;                       //遥信2脉冲输出控制标志置位
      else                                     
       Pulse2_RUN_CTL=0;                       //遥信2脉冲输出控制标志清零
      PulOut2_RVS_Bit=1;                       //遥信2脉冲输出反转标志置位
      PulOut2_Timer=(u16)Timer_1ms;            //遥信2脉冲输出定时器赋初值
      PulOut2_Count=0;                         //脉冲输出个数清零；
      PulOut2_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；  	
     }
    if(Pulnum_Set.NUMB==3)
     {
      if(Pulnum_Set.MOD==1)                    //遥信3脉冲输出控制                                                  
       Pulse3_RUN_CTL=1;                       //遥信3脉冲输出控制标志置位
      else                                                                                       
       Pulse3_RUN_CTL=0;                       //遥信3脉冲输出控制标志清零
      PulOut3_RVS_Bit=1;                       //遥信3脉冲输出反转标志置位
      PulOut3_Timer=(u16)Timer_1ms;            //遥信3脉冲输出定时器赋初值
      PulOut3_Count=0;                         //脉冲输出个数清零；
      PulOut3_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；
     }		
    if(Pulnum_Set.NUMB==4)
     {
      if(Pulnum_Set.MOD==1)                    //遥信4脉冲输出控制
       Pulse4_RUN_CTL=1;                       //遥信4脉冲输出控制标志置位
      else                                     
       Pulse4_RUN_CTL=0;                       //遥信4脉冲输出控制标志清零
      PulOut4_RVS_Bit=1;                       //遥信4脉冲输出反转标志置位
      PulOut4_Timer=(u16)Timer_1ms;            //遥信4脉冲输出定时器赋初值
      PulOut4_Count=0;                         //脉冲输出个数清零；
      PulOut4_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；
     }
    if(Pulnum_Set.NUMB==5)
     {
      if(Pulnum_Set.MOD==1)                    //遥信5脉冲输出控制
       Pulse5_RUN_CTL=1;                       //遥信5脉冲输出控制标志置位
      else
       Pulse5_RUN_CTL=0;                       //遥信5脉冲输出控制标志清零
      PulOut5_RVS_Bit=1;                       //遥信5脉冲输出反转标志置位
      PulOut5_Timer=(u16)Timer_1ms;            //遥信5脉冲输出定时器赋初值
      PulOut5_Count=0;                         //脉冲输出个数清零；
      PulOut5_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；
     }
    if(Pulnum_Set.NUMB==6)
     {
      if(Pulnum_Set.MOD==1)                    //遥信6脉冲输出控制
       Pulse6_RUN_CTL=1;                       //遥信6脉冲输出控制标志置位
      else
       Pulse6_RUN_CTL=0;                       //遥信6脉冲输出控制标志清零
      PulOut6_RVS_Bit=1;                       //遥信6脉冲输出反转标志置位
      PulOut6_Timer=(u16)Timer_1ms;            //遥信6脉冲输出定时器赋初值
      PulOut6_Count=0;                         //脉冲输出个数清零；
      PulOut6_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；
     }
    if(Pulnum_Set.NUMB==7)
     {
      if(Pulnum_Set.MOD==1)                    //遥信7脉冲输出控制
       Pulse7_RUN_CTL=1;                       //遥信7脉冲输出控制标志置位
      else
       Pulse7_RUN_CTL=0;                       //遥信7脉冲输出控制标志清零
      PulOut7_RVS_Bit=1;                       //遥信7脉冲输出反转标志置位
      PulOut7_Timer=(u16)Timer_1ms;            //遥信7脉冲输出定时器赋初值
      PulOut7_Count=0;                         //脉冲输出个数清零；
      PulOut7_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；
     }
    if(Pulnum_Set.NUMB==8)
     {
      if(Pulnum_Set.MOD==1)                    //遥信8脉冲输出控制
       Pulse8_RUN_CTL=1;                       //遥信8脉冲输出控制标志置位
      else
       Pulse8_RUN_CTL=0;                       //遥信8脉冲输出控制标志清零
      PulOut8_RVS_Bit=1;                       //遥信8脉冲输出反转标志置位
      PulOut8_Timer=(u16)Timer_1ms;            //遥信8脉冲输出定时器赋初值
      PulOut8_Count=0;                         //脉冲输出个数清零；
      PulOut8_Count_Set=Pulnum_Set.QANTY;      //存储脉冲输出个数；
     }					
}
/******************************************************
*  设置终端遥控信号状态(针对专变终端)
*  YKSET:XXX,A(0dh)
*  XXX表位号;XXX=0,所有表位
*  A=0:停止对遥控信号的检测;A=1:遥控信号为电平模式;A=2:遥控信号为脉冲模式
******************************************************/
void Set_YKSET(void)
{
    u8 m;
    if((MTR_MOD!=Acquire_Terminal_13)&&
       (MTR_MOD!=Acquire_Terminal_09))          //当前不是在校验专变终端Ⅲ，退出；
      return;                                   
    m=CAN_MSG_IPtr->Data.BYTE[0];               //取出设置数据
    if((m<'0')||(m>'2'))                        
     return;                                    //非法数据退出
    if(m=='0')                                  //停止对遥控信号的检测
     YKSIG_EN=Disable_Bit;                      //遥控信号检测禁止
    if(m=='1')                                  //遥控信号检测为电平模式
     {                                          
      YKSIG_EN=Enable_Bit;                      //遥控信号检测使能
      YKSIG_MOD=LEVEL_MOD;                      //遥控信号为电平模式标志
     }                                          
    if(m=='2')                                  //遥控信号检测为脉冲模式
     {                                          
      YKSIG_EN=Enable_Bit;                      //遥控信号检测使能
      YKSIG_MOD=PULSE_MOD;                      //遥控信号检测为脉冲模式标志	
     }
}
/******************************************************
*  设置遥控信号脉冲触发方式
*  YKPULSE:XXX,A(0dh)
*  XXX表位号;XXX=0,所有表位  
*  A=0:遥控信号为脉冲模式时,低电平触发;A=1:遥控信号为脉冲模式时,高电平触发;
******************************************************/
void Set_YKPULSE(void)
{
    u8 m;
    if((MTR_MOD!=Acquire_Terminal_13)&&
       (MTR_MOD!=Acquire_Terminal_09))          //当前不是在校验专变终端Ⅲ，退出；
      return;                                    
    m=CAN_MSG_IPtr->Data.BYTE[0];               //取出设置数据
    if((m<'0')||m>'1')		                        
     return;                                    //非法数据退出
    if(m=='0')
     YKSIG_TRG_MOD=LOW_PUS_TRG;                 //脉冲模式时，低电平触发
    if(m=='1')
     YKSIG_TRG_MOD=HIG_PUS_TRG;                 //脉冲模式时，高电平触发
}
/******************************************************
*  测试无极性RS485
*  NP485SEL:XXX,A(0dh)
*  XXX表位号;XXX=0,所有表位  
*  A=0:正常RS485电平;A=1:翻转RS485电平;
******************************************************/
void Set_NPRS485(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];         //取出设置数据
    if((m<'0')||m>'1')                    //非法数据退出
     return;
    if(m=='0')
     {
       ABBA_HIGH;                         //正常RS485电平  
     }
    if(m=='1')
     {
       ABBA_LOW;                          //翻转RS485电平
     }
}
/******************************************************
*  直流12V电压测量控制
*  ADCONV:XXX,A(0dh)
*  XXX表位号;XXX=0,所有表位  
*  A='N ' 禁止直流12V测量;A='Y' 使能直流12V测量;
******************************************************/
void Set_ADCONV(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0];         //取出设置数据
	   if(((m!='N')&&(m!='Y'))||
					  (ADC_Start==m))
				  return;
				else
				{
					 ADC_Start=m;
     	ADC_Timer=(u16)Timer_1ms;                  //重置采样定时 
      ADC_Data.Trig=1;					
				}
}
/******************************************************
* 设置双回路校验台 电流接入主副回路
* TTAB:A(0DH) A=1 电流接入副回路 A=2 电流接入主回路
******************************************************/
void Set_TTAB(void)
{
    u8 m;
    m=CAN_MSG_IPtr->Data.BYTE[0]	;        //取出设置数据
	   if(m<'1')
					return;                              //非法数据
    TTAB_JDQ_STS=m;                       //双回路继电器状态寄存 
				TTAB_JDQ_DELY=1;                      //双回路继电器动作延时开始
				TTAB_JDQ_CHG=0;                       //双回路继电器动作标志
				TTAB_JDQ_DELY_Timer=(u16)Timer_1ms;   //初始化双回路继电器动作延时定时
				
}
/******************************************************
功耗测试命令 MEAPN:AAA,B(0dh)
  AAA 要测量功耗的表位 AAA=’000’ 表示广播地址
  B   功耗测量控制   B=’0’ 退出功耗测量 B=’1’ 进入功耗测量
******************************************************/
void Set_MEAPN(void)
{
	
}
/******************************************************
查询某表位功耗测试信息  CHKP:XXX,Y(0dh)
     XXX表位号，不支持广播指令查询；
     Y=H 查询A、B、C 及辅助电源的功耗信息，该命令发送后，功耗测试板依次返回MEAPA、MEAPB、MEAPC、MEAPF等功耗测试信息；此命令只适用于调试过程中。
     Y=A 查询A相功耗测试信息，该命令发送后，功耗测试板返回MEAPA
     Y=B 查询B相功耗测试信息，该命令发送后，功耗测试板返回MEAPB
     Y=C 查询C相功耗测试信息，该命令发送后，功耗测试板返回MEAPC
     Y=F 查询辅助电源功耗测试信息，该命令发送后，功耗测试板返回MEAPF
******************************************************/
void Set_CHKP(void)
{
	
}
/*****************************************************************************
* 调试使能 JTAG 口开放
*****************************************************************************/
void JTAG_EN_Pr(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //使能端口B时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //使能端口C时钟
    GPIODirModeSet(GPIOC,                        //端口GPIOC
                   TCK|TMS|TDI|TDO,              //管脚PC0-PC3
                   GPIO_DIR_MODE_HW);            //第二功能 TRST
    GPIOPadConfigSet(GPIOC,                      //端口GPIOB
                     TCK|TMS|TDI|TDO,            //管脚PC0-PC3   
                     GPIO_STRENGTH_8MA,          //2mA驱动
                     GPIO_PIN_TYPE_OD);          //若上拉
} 
/********************************************************
* 处理未检测到标准电能脉冲
* 显示 no bEP
********************************************************/
void No_BZENGMC_Pr(void)
{
    if(Disp_Choose==DISP_TEST_DATA)       //是否在显示当前试验数据状态
     {
      if(GZ_FLAG||NZTZ_FLAG)              //是否处于故障和跳闸状态
       return;                            //退出 不再显示其他内容	 
      if(SOLID_CFG.LED_NUM==LED_6)        //是否为6位数码管
       {	 
        if(NO_STD_ENG)                    //没有标准表脉冲
         {                                //显示 no bEP
          Disp_Buf[0]=DISP_n;
          Disp_Buf[1]=DISP_o;
          Disp_Buf[2]=DISP_BLANK;
          Disp_Buf[3]=DISP_b;
          Disp_Buf[4]=DISP_E;
          Disp_Buf[5]=DISP_P;
         }
        else
         {
         	memset(Disp_Buf+1,DISP_BLANK,5);//保留1位圈数位	
          Disp_Buf[5]=DISP_E;             //显示'E'
         } 
       }  
      else
       {	 
        if(NO_STD_ENG)                    //没有标准表脉冲
         {                                //显示 no bEP
          Disp_Buf[2]=DISP_n;
          Disp_Buf[3]=DISP_o;
          Disp_Buf[4]=DISP_BLANK;
          Disp_Buf[5]=DISP_b;
          Disp_Buf[6]=DISP_E;
          Disp_Buf[7]=DISP_P;
         }
        else
         {
         	memset(Disp_Buf+2,DISP_BLANK,6);  //保留2位圈数位	
          Disp_Buf[7]=DISP_E;               //显示'E'
         } 
       }  
     }  
}       
/********************************************************
* 被检表电能误差处理模式 计算误差
********************************************************/
void ENG_ERR_MODE_Pr(void)
{
    if(NEW_CMD)                                //判断是否为首次进入该工作状态
     {
      if(WORK_Timer)                           //定时是否到
       return;	
      NEW_CMD=0;                               //清除首次进入该实验标志
      PULSE_ZZ_END=0;		                        //清脉冲走字试验结束标志
      Init_DEVICE_IO();                        //初始化设备输入
      ReStart_Mea_Err();                       //误差清零
      Uin_Iin_Pr();                            //电压,电流接入
      return;                                  //退出
     }
    if(NEW_ENG_PLS)                            //判断是否有新脉冲
     {
      u32 Cnt;
      NEW_ENG_PLS=0;                           //清除新脉冲标志
      if(FIRST_ENG_PLS)                        //第一个脉冲
       {                                       
        FIRST_ENG_PLS=0;                       //清除第一个脉冲标志
        PRE_ENG_VAL=CUR_ENG_VAL;               //上次电能脉冲计数值 采样点值
        RAW_ENG_VAL=CUR_ENG_VAL;               //原始电能脉冲计数值 采样点值
        CURRENT_N=ACT_N;                       //重置圈数
        ENG_STB_CHK=0;                         //暂时不检查是否稳定
        return;
       }
      CURRENT_N--;
      if(CURRENT_N==0)                         //判断设定圈数是否到
       {                                       //计算误差
        CURRENT_N=ACT_N;                       //重置圈数
        Cnt=CUR_ENG_VAL-RAW_ENG_VAL;           //高频计数值
        PRE_ENG_VAL=CUR_ENG_VAL;               //上次电能脉冲计数值
        RAW_ENG_VAL=CUR_ENG_VAL;               //原始电能脉冲计数值
        if(Cnt>500)                            //10k 0.05S计数
         {                                     //认为有标准表脉冲
          ENG_ERR = STD_ENG_CNT_VAL;           //应计脉冲数
          ENG_ERR -=Cnt;                       //多(少)计脉冲数
          ENG_ERR *=100;                       //转换成百分数
          ENG_ERR /=Cnt;                       //计算被检表电能脉冲误差 
          NO_STD_ENG=0;                        //检测到标准表电能脉冲
          if(ENG_ERR>=10000)
           OVER_ERR_FLAG=1;                    //超差标志 
          else
           OVER_ERR_FLAG=0;                    //有效误差标志 
         }
        else                                   //没有标准表脉冲
         {
          OVER_ERR_FLAG=1;                     //超差标志
          NO_STD_ENG=1;                        //没检测到标准表电能脉冲
          ENG_ERR=0;
         } 
        if(!OVER_ERR_FLAG)                     //判断超差标志  
         {                                     //数据不超差
          if(ENG_ERR>=0)                       //判断是否为正误差 
           {
           	TEMP_STR[0]=' ';
            sprintf((char*)TEMP_STR+1,
                    "%7.5f",
                    ENG_ERR);                   //误差 总长度6位 4位小数点             
           } 
          else
           sprintf((char*)TEMP_STR,
                   "%8.5f",
                   ENG_ERR);                    //误差 总长度6位 4位小数点             
          ENG_ERR_ASC[0]=TEMP_STR[0];           //符号位
          Fill_Space_StrCpy(TEMP_STR+1,         //待拷贝的字符串
                            ENG_ERR_ASC+1,      //待写入的字符串
                            8);                 //字符串最大长度                 
          Send_Data(ERR_OCMD_CE,8,ENG_ERR_ASC); //回送误差数据
          if(Disp_Choose==DISP_TEST_DATA)       //判断是否显示当前试验数据
           {          	
            if(SOLID_CFG.LED_NUM==LED_6)        //是否为6位数码管
             Copy_Str_To_DSBUF(DISP_ENG_LEN,    //电能误差数据长度
                               (DISP_ENG_OFFSET-1),//电能误差显示偏移量
                               ENG_ERR_ASC);    //电能误差ASC地址
            else
             Copy_Str_To_DSBUF(DISP_ENG_LEN,    //电能误差数据长度
                               DISP_ENG_OFFSET, //电能误差显示偏移量
                               ENG_ERR_ASC);    //电能误差ASC地址
           }                    
         }                      
        else                                    //超差                           
         {
          ENG_ERR_ASC[0]='+';                   //回送+E
          ENG_ERR_ASC[1]='E';                   //
          Send_Data(ERR_OCMD_CE,2,ENG_ERR_ASC); //回送误差数据
          No_BZENGMC_Pr();
         }
       }
      else
       {
        OEND_Cnt_NSub=CUR_ENG_VAL-PRE_ENG_VAL; //高频计数值
        PRE_ENG_VAL=CUR_ENG_VAL;               //
        if(OEND_Cnt_NSub>OEND_Cnt_OSub)        //本次两次中断间计数值 与上次两次中断间计数值比较
         Cnt=OEND_Cnt_NSub-OEND_Cnt_OSub;      //跳变脉冲数
        else                                   
         Cnt=OEND_Cnt_OSub-OEND_Cnt_NSub;      //跳变脉冲数
        OEND_Cnt_OSub=OEND_Cnt_NSub;           //保存1圈内标准电能脉冲计数值         
        if(ENG_STB_CHK)                        //是否开始检查稳定
         {
          if(ENG_STB_RNG>500)                  //被检表单相脉冲频率高于2HZ 不再判断稳定
           {                                      
            if(Cnt>ENG_STB_RNG)                  //判断跳变是否超出范围
             {                                   
              CURRENT_N=ACT_N;                   //重置圈数
              FIRST_ENG_PLS=1;                   //重新计脉冲
              Update_N_Buf();                    //刷新圈数
             }
            else if(OEND_Cnt_NSub<200)           //标准脉冲跳变小与设定值 稳定
             {                                   //判断标准脉冲数是否太小
              NO_STD_ENG=1;                      //没检测到标准表电能脉冲
              No_BZENGMC_Pr();                   
             } 
           }  
         }
        else
         ENG_STB_CHK=1;	
       }             
      Update_N_Buf();                          //刷新圈数
      Send_Data(ERR_OCMD_CN,2,CURRENT_N_ASC);  //回送当前圈数2bytes
      if(Disp_Choose==DISP_TEST_DATA)          //判断是否显示当前试验数据
       {                                       
        Disp_Timer=(Timer_8ms-DISP_TIME);      //更新显示
//        Disp_Data(Disp_Code_Mode);               //复位并显示 按方式0译码(与原误差板译码方式相同     
//        Disp_Timer=Timer_8ms;                    //刷新显示定时器
       }
     }
}
/********************************************************
* 挂表模式处理
********************************************************/
void MTR_PLUG_MODE_Pr(void)     
{
    if(NEW_CMD)                                 //判断是否首次进入该状态
     {
      if(MTR_PLUG)                              //判断是否挂表
       {                                        //挂表
        WORK_MODE=CAL_ENG_ERR_M;                //进入误差模式
        NEW_CMD=1;                              //首次进入标志
				  		ABBA_HIGH;                              //RS485极性正常							
        CMD_DATA=GB;                            //单字节命令数据
        Send_Data(ERR_OCMD_CG,1,&CMD_DATA);     //发送挂表命令
        if(KEY_PC_PLUG)
         {
          KEY_PC_PLUG=0;                        //按键选择挂表 不延时
          WORK_Timer=20;                        //按键选择挂表 延时20ms
         }
        else 		
         WORK_Timer=(Mtr_Numb_ID*40);           //防止开关电源同时动作 电流太大 拉低电源
       }
      else
       {                                        //不挂表
        if(WORK_Timer)                          //判断定时是否到
         return;	
        NEW_CMD=0;                              //清除标志
        I_In_Out_Pr(0);                         //电流旁路 
        CMD_DATA=BGB;                           //单字节命令数据
        ESwitch_Control_Pr(ALL_PHASE,           //三相电压设置
                           ON,                  //电子开关预置 吸合
                           OFF);                //电子开关最终断开 
        if(SINGLE_OR_THREE)                     //判断是否为三相台
         UJDQ_Control_Pr(ALL_PHASE,             //三相电压设置
                         OFF);                  //电压继电器断开
        else                                    //单相设备
         {                                      
          UJDQ_Control_Pr(UA_PHASE,             //单相电压高端设置
                          OFF);                 //电压继电器断开    火线断开
          UJDQ_Control_Pr(UC_PHASE,             //控制公共端 断开
                          ON);                  //
#ifdef HENAN_AUTO_EQUP                          //是否为河南流水线
          UJDQ_Control_Pr(UB_PHASE,             //单相电压低端设置  零线接入
                          ON);                  //电压继电器断开
#else
          UJDQ_Control_Pr(UB_PHASE,             //单相电压低端设置  零线断开
                          OFF);                 //电压继电器断开
#endif
         }		                 
//        UJDQ_Open_ESwitch_Open();               //电压继电器断 电子开关断
        memset(Disp_Buf,DISP_MINUS,8);          //显示同一设置 清显示
        Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//        Disp_Data(Disp_Code_Mode);              //按方式0译码(与原误差板译码方式相同)
        Send_Data(ERR_OCMD_CG,1,&CMD_DATA);     //发送不挂表命令
        TZZS_LED_OFF;                           //关闭跳闸指示灯
       }
     }  
}
/********************************************************
* 对黑斑模式处理
* 对斑时电子脉冲为IO口输入
********************************************************/
void CATCH_HB_MODE_Pr(void)     
{
    if(NEW_CMD)                               //判断是否首次进入该状态
     {
      if(WORK_Timer)                          //延时是否到
       return;
      NEW_CMD=0;                              //清除首次进入标志
      HB_CATCHED=0;                           //清除黑斑对准标志
      ENG_PLS_IN_Pr();                        //电能脉冲输入处理
      Uin_Iin_Pr();                            //电压,电流接入
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);         //显示同一设置 清显示
      Update_Mtr_Num();                       //显示表位号 Disp_Buf[5] Disp_Buf[6] Disp_Buf[7]
      if(SOLID_CFG.LED_NUM==LED_6)            //是否为6位数码管
       {	
        Disp_Buf[0]=DISP_MINUS;               //显示'-' 显示对斑标志- -XXX
        Disp_Buf[2]=DISP_MINUS;               //显示'-'
       }
      else
       {	
        Disp_Buf[0]=DISP_MINUS;               //显示'-' 显示对斑标志-- -- XXX
        Disp_Buf[1]=DISP_MINUS;               //显示'-' 显示对斑标志-- -- XXX
        Disp_Buf[3]=DISP_MINUS;               //显示'-'
        Disp_Buf[4]=DISP_MINUS;               //显示'-' 显示对斑标志-  - XXX
       }
      CMD_DATA=NCATCH_HB;                     //单字节命令数据
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//      Disp_Data(Disp_Code_Mode);              //按方式0译码(与原误差板译码方式相同)
      Send_Data(ERR_OCMD_CS,1,&CMD_DATA);     //发送斑未对准命令
     }
    else if(!HB_CATCHED)                      //斑未对准处理
     {
      if(NEW_ENG_PLS)                         //收到脉冲 斑对准
       {
        NEW_ENG_PLS=0;                        //清除脉冲标志
        HB_CATCHED=1;                         //斑对准标志
        CMD_DATA=CATCH_HB;                    //单字节命令数据
        UJDQ_Open_ESwitch_Open();             //电压继电器断 电子开关断
        ENG_PLS_IN_Pr();                      //黑斑对准恢复电能脉冲输入
        if(SOLID_CFG.LED_NUM==LED_6)          //是否为6位数码管
         {	                                  //对准黑斑显示
          Disp_Buf[0]=DISP_MINUS;             //显示'-'
          Disp_Buf[1]=DISP_MINUS;             //显示'-' 显示斑准标志 ---XXX
          Disp_Buf[2]=DISP_MINUS;             //显示'-'
         } 
        else
         {                                    //对准黑斑显示	
          Disp_Buf[0]=DISP_MINUS;             //显示'-'
          Disp_Buf[1]=DISP_MINUS;             //显示'-' 显示斑准标志 ---- XXX
          Disp_Buf[2]=DISP_MINUS;             //显示'-'
          Disp_Buf[3]=DISP_MINUS;             //显示'-'
          Disp_Buf[4]=DISP_BLANK;             //显示' ' 
         } 
        Disp_Timer=(Timer_8ms-DISP_TIME);     //更新显示 
//        Disp_Data(Disp_Code_Mode);            //按方式0译码(与原误差板译码方式相同)
        Send_Data(ERR_OCMD_CS,1,&CMD_DATA);   //发送斑对准命令
       }
     } 
}
/********************************************************
* 启动 潜动模式下 监视电能脉冲计数
********************************************************/
void START_STOP_MODE_Pr(void)    
{
    if(NEW_CMD)                               //判断是否首次进入该状态
     {
      if(WORK_Timer)                          //延时是否到
       return;
      NEW_CMD=0;                              //清除首次进入标志
      CURRENT_N=0;                            //当前圈数
      NEW_ENG_PLS=0;                          //清除新脉冲标志
      ENG_PLS_IN_Pr();                        //电能脉冲输入处理
      Uin_Iin_Pr();                           //电压,电流接入
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);         //显示同一设置 清显示
      Update_Mtr_Num();                       //显示表位号 Disp_Buf[5] Disp_Buf[6] Disp_Buf[7]
      Update_N_Buf();
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//      Disp_Data(Disp_Code_Mode);              //按方式0译码(与原误差板译码方式相同)
      Send_Data(ERR_OCMD_CC,1,&CURRENT_N_ASC[1]);//发送当前圈数
      WORK_Timer=2000;                        //2秒后开始接收脉冲
     }
    else
     {
      if(WORK_Timer)                          //延时是否到
       {
       	NEW_ENG_PLS=0;                        //定时未到清除脉冲标志	
        return;
       } 
      if(NEW_ENG_PLS)
       {
        NEW_ENG_PLS=0;
        CURRENT_N++;
        Update_N_Buf();
        Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//        Disp_Data(Disp_Code_Mode);            //按方式0译码(与原误差板译码方式相同)
        Send_Data(ERR_OCMD_CC,1,&CURRENT_N_ASC[1]); //发送当前圈数
       }
     } 
}
/********************************************************
* 准备开始校核常数走字模式 继电器吸合
********************************************************/
void VERIFY_READY_MODE_Pr(void)     
{
    if(NEW_CMD)                               //判断是否首次进入该状态
     {
      if(WORK_Timer)                          //定时是否到
       return;	
      NEW_CMD=0;                              //清除首次进入标志
      VERIFY_END=0;                           //清除校核常数走字试验结束标志
      ENG_PLS_IN_Pr();                        //电能脉冲输入处理
      Uin_Iin_Pr();                           //电压,电流接入
      if(SOLID_CFG.LED_NUM==LED_6)            //是否为6位数码管
       Disp_Long_Data(6,                      //显示数据长度 
                      6,                      //显示起始地址 1---6 第一位到第六位
                      CURRENT_VERIFY_PLS,     //待显示数据
                      DISP_FILL_BLANK);       //显示走字圈数 数据长度 显示起始位 显示值  空位填充
      else                                   
       Disp_Long_Data(8,                      //显示数据长度 
                      8,                      //显示起始地址 1---8 第一位到第八位
                      CURRENT_VERIFY_PLS,     //待显示数据
                      DISP_FILL_BLANK);       //显示走字圈数 数据长度 显示起始位 显示值  空位填充
//      CMD_DATA=ZZ_STRT;                       //单字节命令数据
//      Send_Data(ERR_OCMD_CZ,1,&CMD_DATA);     //发送校核常数走字试验开始命令
     }
}
/********************************************************
* 校核常数走字试验模式
********************************************************/
void VERIFY_START_MODE_Pr(void)    
{
    if(NEW_CMD)                               //判断是否首次进入该状态
     {
      if(WORK_Timer)                          //进入工作状态延时 防止开关电源同时动作 电流太大 拉低电源
       return;
      NEW_CMD=0;                              //清除首次进入标志
      VERIFY_END=0;                           //清除校核常数走字试验结束标志
      CURRENT_VERIFY_PLS=VERIFY_PLS_Set;      //预置校核常数走字脉冲数
      if(SOLID_CFG.LED_NUM==LED_6)            //是否为6位数码管
       Disp_Long_Data(6,                      //显示数据长度 
                      6,                      //显示起始地址 1---6 第一位到第六位
                      CURRENT_VERIFY_PLS,     //待显示数据
                      DISP_FILL_BLANK);       //显示走字圈数 数据长度 显示起始位 显示值  空位填充
      else                                   
       Disp_Long_Data(8,                      //显示数据长度 
                      8,                      //显示起始地址 1---8 第一位到第八位
                      CURRENT_VERIFY_PLS,     //待显示数据
                      DISP_FILL_BLANK);       //显示走字圈数 数据长度 显示起始位 显示值  空位填充
      Uin_Iin_Pr();                           //电压,电流接入
      CMD_DATA=ZZ_STRT;                       //单字节命令数据
      Send_Data(ERR_OCMD_CZ,1,&CMD_DATA);     //发送校核常数走字试验开始命令
      return;
     }
    if(NEW_ENG_PLS)                           //判断是否有新脉冲
     {
      NEW_ENG_PLS=0;                          //清除新脉冲标志
      if(CURRENT_VERIFY_PLS==0)
       return;
      CURRENT_VERIFY_PLS--;                   //走字脉冲减一
      if(CURRENT_VERIFY_PLS==0)
       {
        VERIFY_END=1;                         //走字结束标志
        CMD_DATA=ZZ_END;                      //单字节命令数据
        UJDQ_Open_ESwitch_Open();             //断开电子开关和电压继电器
        if(Disp_Choose==DISP_TEST_DATA)
         {	
          memset(Disp_Buf,DISP_BLANK,8);      //显示同一设置 清显示
          if(SOLID_CFG.LED_NUM==LED_6)        //是否为6位数码管
           {
            Copy_Str_To_DSBUF(4,               //2012.11.7修改 原来为7               
                              0,               
                              VERIFY_ENG_ASC); //
            Disp_Buf[5]=0;                     //0圈   
           }
          else
           {	 		
            Copy_Str_To_DSBUF(6,               //2012.11.7修改 原来为7               
                              0,               
                              VERIFY_ENG_ASC); //
            Disp_Buf[7]=0;                     //0圈   
           }                       
         }                  
        Send_Data(ERR_OCMD_CZ,1,&CMD_DATA);   //发送走字试验完成命令
       }
      else  
       {	
        if(SOLID_CFG.LED_NUM==LED_6)          //是否为6位数码管
         Disp_Long_Data(6,                    //显示数据长度 
                        6,                    //显示起始地址 1---6 第一位到第六位
                        CURRENT_VERIFY_PLS,   //待显示数据
                        DISP_FILL_BLANK);     //显示走字圈数 数据长度 显示起始位 显示值  空位填充
        else                                 
         Disp_Long_Data(8,                    //显示数据长度 
                        8,                    //显示起始地址 1---8 第一位到第八位
                        CURRENT_VERIFY_PLS,   //待显示数据
                        DISP_FILL_BLANK);     //显示走字圈数 数据长度 显示起始位 显示值  空位填充
       }    
     }  
}
/********************************************************
* 跌落试验完成处理
********************************************************/
void SY_END(void)
{
    SY_START=0;                            
    SY_ACT=0;                              //清除已经动作标志
    SY_PROCESS=0;                          //清除进程标志 
    UJDQ_Close_ESwitch_Open();             //失压试验结束      
    if(Disp_Choose==DISP_TEST_DATA)        //是否在显示当前试验数据状态
     {	
      if(SOLID_CFG.LED_NUM==LED_6)         //是否为6位数码管
       {	
        Disp_Buf[0]=DISP_d;                //显示'd'
        Disp_Buf[1]=DISP_L;                //显示'L'
        Disp_Buf[2]=DISP_BLANK;            //显示空白
        Disp_Buf[3]=DISP_E;                //显示'E'
        Disp_Buf[4]=DISP_n;                //显示'n'
        Disp_Buf[5]=DISP_d;                //显示'd'
       } 
      else
       {	
        Disp_Buf[0]=DISP_d;                //显示'd'
        Disp_Buf[1]=DISP_L;                //显示'L'
        Disp_Buf[2]=DISP_BLANK;            //显示空白
        Disp_Buf[3]=DISP_BLANK;            //显示空白
        Disp_Buf[4]=DISP_BLANK;            //显示空白
        Disp_Buf[5]=DISP_E;                //显示'E'
        Disp_Buf[6]=DISP_n;                //显示'n'
        Disp_Buf[7]=DISP_d;                //显示'd'
       } 
     } 
}
/********************************************************
* 电压跌落试验模式
* 失压试验 方案1图形:
********************************************************/
void VOLTAGE_DOWN_MODE_Pr(void) 
{
    u16 t;
    t=(u16)(Timer_1ms);                      //启动失压定时器
    if(NEW_CMD)                              //判断是否首次进入该状态
     {
      if(WORK_Timer)                           //定时是否到
       return;
      SY_Timer=t;	                           //初始化定时器
      NEW_CMD=0;                             //清除首次进入标志
      NEW_ENG_PLS=0;                         //清除新脉冲标志
      CURRENT_N=0;                           //圈数清零
      SY_ACT=0;                              //失压未开始动作
      SY_PROCESS=0;                          //进入失压进程0
      SY_CNT=0;                              //失压计数
      if(SY_MODE==VLOSS_3)                   //电压跌落50%试验
       UJDQ_Close_ESwitch_Open();            //      
      else
       UJDQ_Open_ESwitch_Close();            //继电器断开,电子开关接入
      if(ENG_CLK_CH==DZ_PLS);                //电子脉冲
       { 
        u32 TIMERx;                          //计数器
        u32 ulTimer;                         //TIMER_A TIMER_B 标志
        if(MTYPE==SOUTH)                     //判断是否为南网表
         {	
          TIMERx=TIMER4_BASE;                //默认的时钟脉冲计数器 
          ulTimer=TIMER_B;                   //默认B
         } 
        else                                 
         {	                                  //国网表
          TIMERx=TIMER5_BASE;                //默认的电子脉冲计数器 
          ulTimer=TIMER_A;                   //默认A
         }
        TimerDisable(TIMERx,                 //Timer1-A禁能
                     ulTimer);               
        TimerLoadSet(TIMERx,                 //定时器
                     ulTimer,                //通道
                     1);                     //设置(分频系数)
        TimerMatchSet(TIMERx,                //定时器
                      ulTimer,               //通道
                      0);                    
        TimerEnable(TIMERx,                  //定时器
                  ulTimer);                  //定时器启动
       }
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);        //显示同一设置 清显示
      Update_N_Buf();                        //更新圈数区 
      Update_Mtr_Num();                      //更新表位号区
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//      Disp_Data(Disp_Code_Mode);             //按方式0译码(与原误差板译码方式相同)
      Send_Data(ERR_OCMD_CY,1,&CURRENT_N_ASC[1]);//发送失压试验时圈数
      WORK_Timer=18000;                      //18S后开始试验
      return;
     }
    if(SY_START)                               //判断失压试验是否启动 
     {
      if(WORK_Timer)                           //定时是否到
       return;	
      if(NEW_ENG_PLS)                          //判断是否收到脉冲
       {
        NEW_ENG_PLS=0;                         //清除新脉冲标志
        CURRENT_N++;
        CURRENT_N_ASC[0]=(CURRENT_N%10);       //只显示低位
        if(Disp_Choose==DISP_TEST_DATA)        //是否在显示当前试验数据状态
         {
          Disp_Buf[0]=CURRENT_N_ASC[0];
          Disp_Buf[1]=DISP_BLANK;              //
          Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//          Disp_Data(Disp_Code_Mode);           //按方式0译码(与原误差板译码方式相同)
         } 
        CURRENT_N_ASC[0]|='0';                 //ASC码             
        Send_Data(ERR_OCMD_CY,1,&CURRENT_N_ASC[1]);//发送失压试验时圈数
       } 
      if(UJDQ_FLAG||ESwitch_FLAG)              //继电器动作未完成 退出
       return;
      if(SY_MODE==VLOSS_1)                     //电压跌落方案1 断电1s 加电50ms 3次 
       {
        if(SY_PROCESS==0)                      //判断进入哪个进程
         {
          if(!SY_ACT)                          //判断失压是否动作
           {
            SY_ACT=1;                          //已经动作标志
            SY_Timer=t;	                       //初始化定时器
            ESwitch_Control_Pr(SY_PHASE,       //三相电压设置
                               OFF,            //电子开关预置 
                               OFF);           //电子开关最终断开 
            return;
           }                   
          if((u16)(t-SY_Timer)<1000)           //1s 时间未到 退出 
           return;                             
          SY_Timer=t;                          //初始化定时器
          SY_ACT=0;                            //清除已经动作标志
          SY_PROCESS=1;                        //进入下个进程 
          SY_CNT++;
          if(SY_CNT>=3)                        //判断电压跌落失压是否结束
           {                   
            SY_END();                          //失压完成处理
            return;
           }
         }
        else if(SY_PROCESS==1)
         {
          if(!SY_ACT)                          //判断失压是否动作
           {
            SY_ACT=1;                          //已经动作标志
            SY_Timer=t;	                       //初始化定时器
            ESwitch_Control_Pr(SY_PHASE,       //三相电压设置
                               ON,             //电子开关预置 
                               ON);            //电子开关最终状态 
           }                   
          if((u16)(t-SY_Timer)<50)             //判断50ms 时间是否到
           return; 
          SY_Timer=t;                          //初始化定时器
          SY_ACT=0;                            //清除已经动作标志
          SY_PROCESS=0;                        //进入下个进程 
         }                                     
       }                                       
      else if(SY_MODE==VLOSS_2)                //电压跌落方案2 断电20ms一次
       {
        if(!SY_ACT)                            //判断失压是否动作
         {                                     
          SY_ACT=1;                            //已经动作标志
          SY_Timer=t;                          //初始化定时器
          ESwitch_Control_Pr(SY_PHASE,         //三相电压设置
                             OFF,              //电子开关预置 
                             OFF);             //电子开关最终断开 
         }                                     
        if((u16)(t-SY_Timer)<20)               //判断20ms 时间是否到
         return;                               
        SY_END();
       }
      else
       SY_START=0;                             //跌落50%	 
     }
}
/********************************************************
* 进入计电能试验 计量被检表电能
********************************************************/
void MEASURE_ENG_MODE_Pr(void)        
{
    if(NEW_CMD)                             //判断是否首次进入该状态
     {
      if(WORK_Timer)                        //定时是否到
       return;	
      NEW_CMD=0;                            //清除首次进入标志
      NEW_ENG_PLS=0;                        //清除电能脉冲标志
      CURRENT_PLS_CNT=0;                    //脉冲数清零
      CURRENT_ENG_KWh=0;                    //电能清零
      Uin_Iin_Pr();                         //电压,电流接入
      return;
     }    
    if(NEW_ENG_PLS)                         //判断是否有新脉冲
     {
      NEW_ENG_PLS=0;                        //清除新脉冲标志
      CURRENT_PLS_CNT++;                    //
      CURRENT_ENG_KWh=CURRENT_PLS_CNT;      //脉冲数
      CURRENT_ENG_KWh/=MTR_ENG_CST;         //电能=(脉冲数/低频常数)*分频系数
      CURRENT_ENG_KWh*=DIVIDE_Coef;         //
      sprintf((char*)TEMP_STR,
              "%9.5f",
              CURRENT_ENG_KWh);             //显示当前电能       
      Fill_Space_StrCpy(TEMP_STR,           //待拷贝的字符串
                        CURRENT_ENG_ASC,    //待写入的字符串
                        9);                 //字符串最大长度                 
      if(Disp_Choose==DISP_TEST_DATA)       //判断是否显示当前试验数据
       Copy_Str_To_DSBUF(DISP_PWR_LEN,      //电能数据长度
                         DISP_PWR_OFFSET,   //电能数据显示偏移量
                         CURRENT_ENG_ASC);  //电能数据ASC地址   
      Send_Data(ERR_OCMD_CP,                //命令
                8,                          //数据长度
                CURRENT_ENG_ASC);           //数据地址
     }
}
/********************************************************
* 进入盘转误差试验
********************************************************/
void PANZHUAN_ERR_MODE_Pr(void)       
{
    if(NEW_CMD)                                //判断是否为首次进入该工作状态
     {
      if(WORK_Timer)                           //定时是否到
       return;	
      NEW_CMD=0;                               //清除首次进入该实验标志
      FIRST_ENG_PLS=1;                         //首次计电能脉冲标志
      ENG_PLS_IN_Pr();                         //电能脉冲输入处理
      Uin_Iin_Pr();                            //电压,电流接入
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);          //显示同一设置 清显示
      CURRENT_N=PZZS;                          //盘转数
      Update_N_Buf();                          //更新当前圈数显示缓冲区
      Update_Mtr_Num();                        //更新表位号区
      Send_Data(ERR_OCMD_CN,2,CURRENT_N_ASC);  //回送当前圈数2bytes
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//      Disp_Data(Disp_Code_Mode);               //按方式0译码(与原误差板译码方式相同)
      return;
     }
    if(NEW_ENG_PLS)                            //新电能脉冲
     {
      NEW_ENG_PLS=0;                           //清除新电能脉冲标志
      if(GZ_FLAG||NZTZ_FLAG)                   //是否处于故障和跳闸状态
       return;                                 //退出 不再显示其他内容	 
      if(FIRST_ENG_PLS)                        //第一个脉冲
       {                                       
        FIRST_ENG_PLS=0;                       //清除第一个脉冲标志
        CURRENT_N=PZZS;                        //重置圈数
        PRE_PZ_Cnt_Val=CUR_PZ_Cnt_Val;         //更新当前计数
        return;
       }
      CURRENT_N--;                             //盘转圈数减1
      if(CURRENT_N==0)                         //判断设定圈数是否到
       {	
        u16 Cnt;
        u8 Sign;                               //符号
        CURRENT_N=PZZS;                        //重置圈数
        Cnt=CUR_PZ_Cnt_Val-PRE_PZ_Cnt_Val;     //计数值
        PRE_PZ_Cnt_Val=CUR_PZ_Cnt_Val;         //更新采样点值
        if(Cnt>PZ_STD_CNT)                     //判断误差数
         {                               
          PZ_ERR_ASC[0]=' ';
          Sign=DISP_BLANK;                     //正误差
          Cnt-=PZ_STD_CNT;
         }	        
        else
         {
          PZ_ERR_ASC[0]='-';
          Sign=DISP_MINUS;                     //负误差
          Cnt=(PZ_STD_CNT-Cnt);                //          	 
         }		
        if(Cnt>99)                             //最大显示2位
         Cnt=99; 
        PZ_ERR_ASC[1]=(Cnt/10);                //误差圈数高位
        PZ_ERR_ASC[2]=(Cnt%10);                //误差圈数低位
        if(Disp_Choose==DISP_TEST_DATA)        //是否在显示当前试验数据状态
         {	
          if(SOLID_CFG.LED_NUM==LED_6)         //是否为6位数码管
           {	
            Disp_Buf[3]=Sign;
            Disp_Buf[4]=PZ_ERR_ASC[1];         //误差圈数高位ASC码
            Disp_Buf[5]=PZ_ERR_ASC[2];         //误差圈数低位ASC码
           } 
          else
           {	
            Disp_Buf[5]=Sign;
            Disp_Buf[6]=PZ_ERR_ASC[1];         //误差圈数高位ASC码
            Disp_Buf[7]=PZ_ERR_ASC[2];         //误差圈数低位ASC码
           } 
         } 
        PZ_ERR_ASC[1]|='0';                    //误差圈数高位
        PZ_ERR_ASC[2]|='0';                    //误差圈数低位
        Send_Data(ERR_OCMD_CF,                 //发送盘转误差圈数命令
                  3,                           //盘转误差长度
                  PZ_ERR_ASC);                 //盘转误差地址
       } 
      Send_Data(ERR_OCMD_CN,2,CURRENT_N_ASC);  //回送当前圈数2bytes
      Update_N_Buf();                          //刷新圈数
      if(Disp_Choose==DISP_TEST_DATA)          //判断是否显示当前试验数据
       {
        Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//        Disp_Data(Disp_Code_Mode);             //复位并显示 按方式0译码(与原误差板译码方式相同     
//        Disp_Timer=Timer_8ms;                  //刷新显示定时器
       }
     } 
}
/********************************************************
* 常数测试试验
********************************************************/
void MEA_CST_MODE_Pr(void)            
{
    if(NEW_CMD)                                //判断是否为首次进入该工作状态
     {
      if(WORK_Timer)                           //定时是否到
       return;	
      NEW_CMD=0;                               //清除首次进入该实验标志
      CURRENT_PLS_CNT=0;                       //脉冲计数清零
      ENG_PLS_IN_Pr();                         //初始化脉冲输入
      Uin_Iin_Pr();                            //电压,电流接入
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);          //显示同一设置 清显示
      Update_Mtr_Num();                        //更新表位号区
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//      Disp_Data(Disp_Code_Mode);               //按方式0译码(与原误差板译码方式相同)
      return;
     }
    if(NEW_ENG_PLS)                            //判断是否有新电能脉冲
     {
      NEW_ENG_PLS=0;                           //清除新电能脉冲标志
      CURRENT_PLS_CNT++;
      if(SOLID_CFG.LED_NUM==LED_6)             //是否为6位数码管 
       {	
        sprintf((char*)TEMP_STR,
                "%6d",
                CURRENT_PLS_CNT);              //脉冲数
        Fill_Space_StrCpy(TEMP_STR,            //待拷贝的字符串 拷贝低六位
                          CURRENT_PLS_ASC,     //待写入的字符串
                          7);                  //字符串最大长度  
       }                                 
      else
       {	
        sprintf((char*)TEMP_STR,
                "%8d",
                CURRENT_PLS_CNT);              //脉冲数
        Fill_Space_StrCpy(TEMP_STR,            //待拷贝的字符串
                          CURRENT_PLS_ASC,     //待写入的字符串
                          9);                  //字符串最大长度 
       }                                  
      if(Disp_Choose==DISP_TEST_DATA)          //判断是否显示当前试验数据
       Copy_Str_To_DSBUF(DISP_PLS_LEN,         //脉冲数据长度
                         DISP_PLS_OFFSET,      //脉冲显示偏移量
                         CURRENT_PLS_ASC);     //脉冲ASC地址   
      Send_Data(ERR_OCMD_CB,                   //回送脉冲累加值
                8,                             //数据长度4字节
                CURRENT_PLS_ASC);              //回送数据地址
     }                         
            
}
/********************************************************
* 功耗测量试验
********************************************************/
void MEA_POWER_D_MODE_Pr(void)        
{
    if(NEW_CMD)                                //判断是否为首次进入该工作状态
    {
      u8 C_Phase,O_Phase;                      //吸合相 断开相
      if(WORK_Timer)                           //定时是否到
       return;	
      NEW_CMD=0;                               //清除首次进入该实验标志
      if(!MTR_PLUG)                            //判断是否挂表
       return;                                 //不挂表退出
      if(Pwr_Phase=='1')                       //功耗测量相是否错误 是否为A相
       {
        C_Phase=(UB_PHASE|UC_PHASE);           //吸合B C相
        O_Phase=UA_PHASE;                      //断开A相 接入功耗测试仪
       }
      else if(Pwr_Phase=='2')                  //是否为B相 		
       {
        C_Phase=(UA_PHASE|UC_PHASE);           //吸合A C相
        O_Phase=UB_PHASE;                      //断开B相  接入功耗测试仪
       }
      else if(Pwr_Phase=='3')                  //是否为C相 
       {
        C_Phase=(UA_PHASE|UB_PHASE);           //吸合A B相
        O_Phase=UC_PHASE;                      //断开C相  接入功耗测试仪
       }
      else 
       {
       	Pwr_Phase='0';	 			
        WORK_MODE=CAL_ENG_ERR_M;               //设置为校验误差模式
        CURRENT_PLS_CNT=0;                     //清除脉冲计数
        CURRENT_ENG_KWh=0;                     //清除电能累计
        NEW_CMD=1;                             //新命令标志
	  					ABBA_HIGH;                             //RS485极性正常							
        WORK_Timer=(Mtr_Numb_ID*40);           //进入工作状态延时 防止开关电源同时动作 电流太大 拉低电源
        return;
       }
      if(!SINGLE_OR_THREE)                     //判断是否为单相台
       {                                       //单相台
        C_Phase=UB_PHASE|UC_PHASE;             //吸合A相零线 断开公共端 2012.4.14 为解决载表通信问题 用三相继电器板代替单相继电器板
        O_Phase=UA_PHASE;                      //断开A相火线
       }		 
      ESwitch_Control_Pr(ALL_PHASE,            //三相电压设置
                         ON,                   //电子开关预置 吸合
                         OFF);                 //电子开关最终断开  防止拉弧
      UJDQ_Control_Pr(C_Phase,                 //三相电压设置
                      ON);                     //电压继电器吸合 设置继电器吸合相
      UJDQ_Control_Pr(O_Phase,                 //三相电压设置
                      OFF);                    //电压继电器吸合 设置继电器断开相
    }							
}
/********************************************************
* 初始化设备测量IO口
* GDT_PLS          '1'      //光电头脉冲
* DZ_PLS           '2'      //电子脉冲
* SZ_PLS           '3'      //时钟脉冲
* XUL_PLS          '4'      //需量脉冲
* TQ_PLS           '5'      //投切脉冲
* HZ_PLS           '6'      //合闸脉冲
* 南网表只处理互换管脚  标志 定时器仍用原来的
********************************************************/
void Init_DEVICE_IO(void)
{
#ifdef PULSE                                         //脉冲检测程序处理该段
    GPIOPinTypeGPIOInput(CYCLE_PORT_TAB[DZ_PLS-GDT_PLS],  //端口
                         CYCLE_PIN_TAB[DZ_PLS-GDT_PLS]);  
    GPIOPinTypeGPIOInput(CYCLE_PORT_TAB[SZ_PLS-GDT_PLS],  //端口
                         CYCLE_PIN_TAB[SZ_PLS-GDT_PLS]);  
    GPIOPinTypeGPIOInput(CYCLE_PORT_TAB[GDT_PLS-GDT_PLS], //端口
                         CYCLE_PIN_TAB[GDT_PLS-GDT_PLS]);
    GPIOPinTypeGPIOInput(XL_MC_GPIO,                 //端口
                         XL_MC);                     //管脚 PH3 需量周期输入
    GPIOPinTypeGPIOInput(TQ_MC_GPIO,                 //端口
                         TQ_MC);                     //备用 PH4 时段投切输入
    GPIOIntDisable(CYCLE_PORT_TAB[DZ_PLS-GDT_PLS],   //端口
                   CYCLE_PIN_TAB[DZ_PLS-GDT_PLS]);   //先关闭电子脉冲中断	
    GPIOIntDisable(CYCLE_PORT_TAB[SZ_PLS-GDT_PLS],   //端口
                   CYCLE_PIN_TAB[SZ_PLS-GDT_PLS]);   //先关闭时钟 脉冲中断	
    GPIOIntDisable(CYCLE_PORT_TAB[GDT_PLS-GDT_PLS],  //端口
                   CYCLE_PIN_TAB[GDT_PLS-GDT_PLS]);  //先关闭光电 脉冲中断	
    GPIOIntDisable(XL_MC_GPIO,                       //端口 设置管脚中断禁能
                   XL_MC);	                          //管脚	
    GPIOIntDisable(TQ_MC_GPIO,                       //端口 设置管脚中断禁能
                   TQ_MC);	                          //管脚	
    TimerDisable(TIMER5_BASE,                        //禁止电子脉冲输入计数器 
                 TIMER_A);                           
    TimerDisable(TIMER4_BASE,                        //禁止时钟脉冲输入计数器 
                 TIMER_B); 
    GPIOIntTypeSet(CYCLE_PORT_TAB[DZ_PLS-GDT_PLS],   //端口
                   CYCLE_PIN_TAB[DZ_PLS-GDT_PLS],    //电子脉冲下降沿中断	
                   GPIO_FALLING_EDGE);               //
    GPIOIntTypeSet(CYCLE_PORT_TAB[SZ_PLS-GDT_PLS],   //端口
                   CYCLE_PIN_TAB[SZ_PLS-GDT_PLS],    //时钟脉冲下降沿中断	
                   GPIO_FALLING_EDGE);               //
    GPIOIntTypeSet(CYCLE_PORT_TAB[GDT_PLS-GDT_PLS],  //端口
                   CYCLE_PIN_TAB[GDT_PLS-GDT_PLS],   //光电脉冲下降沿中断	
                   GPIO_FALLING_EDGE);               //
    GPIOIntClear(GPIOM,                              //端口 清除端口中断 光电头脉冲 电子脉冲 时钟脉冲 需量 投切 合闸
                 0xFF);	                             //管脚
    GDT_INT_REEN=1;                                  //光电头中断重使能标志置0
    DZ_INT_REEN=1;                                   //清除重使能标志                                  
    SZ_INT_REEN=1;                                   //清除重使能标志  
    XUL_INT_REEN=0;                                  //清除重使能标志 
    TQ_INT_REEN=0;                                   //清除重使能标志 
    GDT_Timer=Timer_8ms;                             //重启光电脉冲中断使能定时
    DZ_Timer=Timer_8ms;                              //重启电子脉冲中断使能定时
    SZ_Timer=Timer_8ms;                              //重启时钟脉冲中断使能定时
#else
    if(WORK_MODE==MEA_ENG_DUTY_M)                    //测量脉冲周期状态
     {
      u32 GPIOx;
      u8 Pin;
      vu8_Ptr Timer;
      u32_Ptr FLAG;
      if(CYCLE_MEA_ID)                               //是否不是测量电能脉冲周期
       Pin=CYCLE_MEA_ID+1;
      else if(ENG_CLK_CH==GDT_PLS)                   //测量电能脉冲周期 判断是否为光电头脉冲
       Pin=(GDT_PLS-GDT_PLS);                        //光电头脉冲
      else
       Pin=(DZ_PLS-GDT_PLS);                         //电子脉冲
      Timer=IO_Timer_Tab[Pin];                       //            
      FLAG=IO_REEN_TAB[Pin];
      if(MTYPE==SOUTH)                               //判断是否为南网表
       {                                             //南网表处理
       	if(Pin==(DZ_PLS-GDT_PLS))                    //是否选择电子脉冲
       	 Pin=(SZ_PLS-GDT_PLS);                       //管脚换到时钟脉冲上	
       	else if(Pin==(SZ_PLS-GDT_PLS))               //是否选择时钟脉冲 
       	 Pin=(DZ_PLS-GDT_PLS);                       //管脚换到电子脉冲上			
       }
      GPIOx=CYCLE_PORT_TAB[Pin];                     //端口
      Pin=CYCLE_PIN_TAB[Pin];                        //对应管脚   
      GPIOPinTypeGPIOInput(GPIOx,                    //端口
                           Pin);                     //定义              
      GPIOPinTypeGPIOInput(XL_MC_GPIO,               //端口
                           XL_MC);                   //管脚 PH3 需量周期输入
      GPIOPinTypeGPIOInput(TQ_MC_GPIO,               //端口
                           TQ_MC);                   //备用 PH4 时段投切输入
      GPIOIntDisable(CYCLE_PORT_TAB[DZ_PLS-GDT_PLS],   //端口
                     CYCLE_PIN_TAB[DZ_PLS-GDT_PLS]);   //先关闭电子脉冲中断	
      GPIOIntDisable(CYCLE_PORT_TAB[SZ_PLS-GDT_PLS],   //端口
                     CYCLE_PIN_TAB[SZ_PLS-GDT_PLS]);   //先关闭时钟 脉冲中断	
      GPIOIntDisable(CYCLE_PORT_TAB[GDT_PLS-GDT_PLS],  //端口
                     CYCLE_PIN_TAB[GDT_PLS-GDT_PLS]);  //先关闭光电 脉冲中断	
      GPIOIntDisable(XL_MC_GPIO,                  //端口 设置管脚中断禁能
                     XL_MC);	                    //管脚	
      GPIOIntDisable(TQ_MC_GPIO,                  //端口 设置管脚中断禁能
                     TQ_MC);	                    //管脚	
      TimerDisable(TIMER5_BASE,                   //禁止电子脉冲输入计数器 
                   TIMER_A); 
      TimerDisable(TIMER4_BASE,                   //禁止时钟脉冲输入计数器 
                   TIMER_B); 
      RISE_FALL_LVL=0;                            //上升/下降沿中断标志 
      GPIOIntTypeSet(GPIOx,                       //端口
                     Pin,                         //管脚
                     GPIO_RISING_EDGE);           //下次中断改为上升沿中断 先测量高电平时间        
      GPIOIntClear(GPIOM,                         //端口 清除端口中断
                   0xFF);	                        //管脚
      GDT_INT_REEN=0;                             //光电头中断重使能标志置0
      DZ_INT_REEN=0;                              //清除重使能标志                                  
      SZ_INT_REEN=0;                              //清除重使能标志  
      XUL_INT_REEN=0;                             //清除重使能标志 
      TQ_INT_REEN=0;                              //清除重使能标志 
      *(Timer.Ptr)=Timer_8ms;   
      *(FLAG.Ptr)=1;                              //重使能标志
     }
    else
     {
      MFuction_Clk_Cfg();                         //初始化多功能测量 SZ XL TQ
      ENG_PLS_IN_Pr();                            //初始化电能脉冲输入
     }		 
#endif
}
/********************************************************
* 测量脉冲周期和占空比
* ENG_PLS          '1'      //电能脉冲
* SZ_PLS           '2'      //时钟脉冲
* XUL_PLS          '3'      //需量脉冲
* TQ_PLS           '4'      //投切脉冲
* HZ_PLS           '5'      //合闸脉冲
********************************************************/
void MEA_ENG_DUTY_MODE_Pr(void)       
{
    if(NEW_CMD)                                //判断是否为首次进入该工作状态
     {
      if(WORK_Timer)                           //定时是否到
       return;	
      NEW_CMD=0;                               //清除首次进入该实验标志
      Uin_Iin_Pr();                            //电压电流接入
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);          //显示同一设置 清显示
      CURRENT_N=ACT_N;                         //设定圈数
      Update_N_Buf();                          //更新圈数区 
      Update_Mtr_Num();                        //更新表位号区
      FIRST_ENG_PLS=1;                         //首个脉冲标志
      High_Lvl_Time_Tp=0;                      //高电平时间清零 
      Low_Lvl_Time_Tp=0;                       //低电平时间清零
      High_Lvl_CNT=0;                          //高电平计数清零
      Low_Lvl_CNT=0;                           //低电平计数清零
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//      Disp_Data(Disp_Code_Mode);               //按方式0译码(与原误差板译码方式相同)
      Init_DEVICE_IO();                        //初始化设备IO口
      return;
     }
    if(NEW_ENG_PLS)                            //收到新脉冲标志
     {
      NEW_ENG_PLS=0;                           //清除新脉冲标志
      if(RISE_FALL_LVL)                        //上次中断为下降沿
       {                                       //计算高电平时间
        High_Lvl_Time_Tp+=CYCLE_NEW_Timer;     //高电平时间累加
        High_Lvl_Time_Tp-=CYCLE_OLD_Timer;
        CYCLE_OLD_Timer=CYCLE_NEW_Timer;       //更新计数
        High_Lvl_CNT++;                        //高电平次数加1
       }
      else
       {
        Low_Lvl_Time_Tp+=CYCLE_NEW_Timer;      //低电平时间累加
        Low_Lvl_Time_Tp-=CYCLE_OLD_Timer;
        CYCLE_OLD_Timer=CYCLE_NEW_Timer;       //更新计数
        Low_Lvl_CNT++;                         //低电平次数
        if(Low_Lvl_CNT>=5)
         {
          if(High_Lvl_CNT==0)                  //没有高电平脉冲
           PLS_Lvl_Time[0]=0;
          else 
           PLS_Lvl_Time[0]=High_Lvl_Time_Tp/High_Lvl_CNT;
          if(Low_Lvl_CNT==0)
           PLS_Lvl_Time[1]=0;
          else           
           PLS_Lvl_Time[1]=Low_Lvl_Time_Tp/Low_Lvl_CNT;
          High_Lvl_CNT=0;
          Low_Lvl_CNT=0;
          High_Lvl_Time_Tp=0;
          Low_Lvl_Time_Tp=0;
          HIGH_LVL_TIME[0]='H';
          HIGH_LVL_TIME[1]=' ';
          if(SOLID_CFG.LED_NUM==LED_6)         //是否为6位数码管
           {	 		
            sprintf((char*)TEMP_STR,
                    "%5d",
                    PLS_Lvl_Time[0]);          //高电平时间 5位
            Fill_Space_StrCpy(TEMP_STR,        //待拷贝的字符串
                              HIGH_LVL_TIME+1, //待写入的字符串
                              6);              //字符串最大长度 
           }                                   
          else
           {	 		
            sprintf((char*)TEMP_STR,
                    "%6d",
                    PLS_Lvl_Time[0]);          //高电平时间
            Fill_Space_StrCpy(TEMP_STR,        //待拷贝的字符串
                              HIGH_LVL_TIME+2, //待写入的字符串
                              7);              //字符串最大长度 
           }                                   
          LOW_LVL_TIME[0]='L';
          LOW_LVL_TIME[1]=' ';
          if(SOLID_CFG.LED_NUM==LED_6)         //是否为6位数码管
           {	 		
            sprintf((char*)TEMP_STR,
                    "%5d",
                    PLS_Lvl_Time[1]);          //低电平时间
            Fill_Space_StrCpy(TEMP_STR,        //待拷贝的字符串
                              LOW_LVL_TIME+1,  //待写入的字符串
                              6);              //字符串最大长度                 
           }                                   
          else
           {	 		
            sprintf((char*)TEMP_STR,
                    "%6d",
                    PLS_Lvl_Time[1]);          //低电平时间
            Fill_Space_StrCpy(TEMP_STR,        //待拷贝的字符串
                              LOW_LVL_TIME+2,  //待写入的字符串
                              7);              //字符串最大长度                 
           }                                   
          memcpy(CANT_STR,
                 &PLS_Lvl_Time[0],
                 3);
          memcpy(CANT_STR+3,
                 &PLS_Lvl_Time[1],
                 3);
          Send_Data(ERR_OCMD_CM,               //回送脉冲周期命令
                    6,
                    CANT_STR);
          if(DISP_HL_LVL)                      //判断显示高电平时间还是低电平时间
           {                                   //显示高电平时间
            DISP_HL_LVL=0;                     //翻转
            if(Disp_Choose==DISP_TEST_DATA)     //判断是否显示当前试验数据
             Copy_Str_To_DSBUF(DISP_TIME_LEN,   //脉冲周期数据长度
                               DISP_TIME_OFFSET,//脉冲周期显示偏移量
                               HIGH_LVL_TIME);  //脉冲周期ASC地址   
           }
          else
           {                                   //显示低电平时间
            DISP_HL_LVL=1;                     //翻转
            if(Disp_Choose==DISP_TEST_DATA)    //判断是否显示当前试验数据
             Copy_Str_To_DSBUF(DISP_TIME_LEN,   //脉冲周期数据长度
                               DISP_TIME_OFFSET,//脉冲周期显示偏移量
                               LOW_LVL_TIME);   //脉冲周期ASC地址   
           }                     
         }
       } 
     } 																	  
             
}
/********************************************************
* 定脉冲走字试验
********************************************************/
void PULSE_ZZ_MODE_Pr(void)           
{
    if(NEW_CMD)                                //判断是否为首次进入该工作状态
     {
      if(WORK_Timer)                           //定时是否到
       return;	
      NEW_CMD=0;                               //清除首次进入该实验标志
      ENG_PLS_IN_Pr();                         //初始化脉冲输入
      Uin_Iin_Pr();                            //电压电流接入
      if(Disp_Choose==DISP_TEST_DATA)
       memset(Disp_Buf,DISP_BLANK,8);          //显示同一设置 清显示
      CURRENT_N=ACT_N;                         //设定圈数
      Update_N_Buf();                          //更新圈数区 
      Update_Mtr_Num();                        //更新表位号区
      FIRST_ENG_PLS=1;                         //首个脉冲标志
      PULSE_ZZ_END=0;
      if(!ZZ_PLS_LOADED)                       //判断脉冲是否已经预置
       {                                       //脉冲未预置 
        CURRENT_PLS_CNT=0;                     //脉冲清零
       }                                    
      Disp_Timer=(Timer_8ms-DISP_TIME);       //更新显示
//      Disp_Data(Disp_Code_Mode);               //按方式0译码(与原误差板译码方式相同)
      return;
     }     
    if(NEW_ENG_PLS)                            //收到新脉冲标志
     {
      NEW_ENG_PLS=0;                           //清除新脉冲标志
      if(PULSE_ZZ_END)
       return;
      if(GZ_FLAG||NZTZ_FLAG)                   //是否处于故障和跳闸状态
       return;                                 //退出 不再显示其他内容	 
      CURRENT_PLS_CNT++;                       //脉冲加1处理
      if(SOLID_CFG.LED_NUM==LED_6)             //是否为6位数码管
       {	
        sprintf((char*)TEMP_STR,
                "%6d",
                CURRENT_PLS_CNT);             //脉冲数ASC码表示
        Fill_Space_StrCpy(TEMP_STR,           //待拷贝的字符串
                          CURRENT_PLS_ASC,    //待写入的字符串
                          7);                 //字符串最大长度
       }                                    
      else
       {	
        sprintf((char*)TEMP_STR,
                "%8d",
                CURRENT_PLS_CNT);             //脉冲数ASC码表示
        Fill_Space_StrCpy(TEMP_STR,           //待拷贝的字符串
                          CURRENT_PLS_ASC,    //待写入的字符串
                          9);                 //字符串最大长度
       }                                    
      Send_Data(ERR_OCMD_CB,                   //回送脉冲累加值
                8,                             //数据长度8字节
                CURRENT_PLS_ASC);              //回送数据地址
      if(Disp_Choose==DISP_TEST_DATA)          //判断是否显示当前试验数据
       Copy_Str_To_DSBUF(DISP_PLS_LEN,         //脉冲数据长度
                         DISP_PLS_OFFSET,      //脉冲显示偏移量
                         CURRENT_PLS_ASC);     //脉冲ASC地址   
      if(CURRENT_PLS_CNT>=ZZ_PLS_Set)          //判断走字脉冲数是否到达设定值
       {
        UJDQ_Open_ESwitch_Open();              //电压断开
        CMD_DATA=ZZ_END;                       //走字试验完成
        PULSE_ZZ_END=1;                        //走字试验完成标志
        if(Disp_Choose==DISP_TEST_DATA)        //是否在显示当前试验数据状态
         {       
          Disp_Buf[0]=DISP_E;                  //显示'E'
          Disp_Buf[1]=DISP_n;                  //显示'n'
          Disp_Buf[2]=DISP_d;                  //显示'd'  
         } 
        Send_Data(ERR_OCMD_CK,                 //脉冲走字试验停止
                  1,                           //数据长度1字节
                  &CMD_DATA);                  //回送数据地址
       }                         
     } 
    
}
/********************************************************
* 耐压试验 并记录耐压状态下脉冲状态
* 回送 ED:XXX,X0 表位击穿
*      ED:XXX,X1 表位正常
********************************************************/
void NYSY_MODE_Pr(void)               
{
    if(NEW_ENG_PLS)                            //收到新脉冲标志
     {
      NEW_ENG_PLS=0;                           //清除新脉冲标志
      CURRENT_N++;                             //耐压过程中脉冲计数
      if(CURRENT_N>=10)                        //耐压过程中脉冲超过10个回校验状态
       {
        CURRENT_N=0;                           //清脉冲计数
        WORK_MODE=CAL_ENG_ERR_M;               //回校验状态
        NEW_CMD=1;                             //第一次进入标志
       } 
     } 
    if((u8)(Timer_8ms-NY_CHK_Timer)>=NY_CHK_TIME)//200ms查询一次
     {
      NY_CHK_Timer=Timer_8ms;                  //重启查询定时
      if(HC165_DATA.JCXH)                      //读耐压结果 GPIOPinRead(GPIOC,NY_IN) I/O检测击穿一直没有启用
       {//8391 系列误差板                      //没有击穿
        if(NY_RESULT==NY_UNKW)                 //耐压结果稳定
         NY_RESULT=NY_GOOD;                    //耐压结果稳定
        else if(NY_RESULT==NY_BAD)
         NY_RESULT=NY_UNKW;
        else
         {
          NY_RESULT=NY_GOOD;                   //好表
          NEW_ENG_DATA=1;
         } 	 	
       }
      else
       {
        if(NY_RESULT==NY_UNKW)                 //耐压结果稳定
         NY_RESULT=NY_BAD;                     //耐压结果稳定
        else if(NY_RESULT==NY_GOOD)
         NY_RESULT=NY_UNKW;	
        else
         {
          NY_RESULT=NY_BAD;                    //坏表
          NEW_ENG_DATA=1;
         } 	 	
       }  
     }
    if(NEW_ENG_DATA)
     {
      if((u16)(Timer_1ms-NY_SEND_Timer)>=NY_SEND_TIME)      //定时发送
       {
        NEW_ENG_DATA=0;                        //
        NY_SEND_Timer=(u16)Timer_1ms;          //	
        Send_Data(ERR_OCMD_CX,1,&NY_RESULT);   //回送耐压结果
       }  
     }  
}
/********************************************************
* 接地故障试验
********************************************************/
void GDGZ_MODE_Pr(void)
{
    if(NEW_CMD)
     {
      NEW_CMD=0;                              //清除第一次进入标志	
     }
    else
     {
      	
     }	 		
}
/********************************************************
* 载波通信试验 
********************************************************/

void ZBTX_MODE_Pr(void)
{
    if(NEW_CMD)                                //判断是否为首次进入该工作状态
     {
      if(WORK_Timer)                           //定时是否到
       return;	
      NEW_CMD=0;                               //清除首次进入该实验标志
      if(SINGLE_OR_THREE)                      //判断是否为三相台
       return;                                 //三相台载表不需特殊处理
      ESwitch_Control_Pr(ALL_PHASE,            //三相电压设置
                         ON,                   //电子开关预置 吸合
                         OFF);                 //电子开关最终断开 
      UJDQ_Control_Pr(ALL_PHASE,               //三相电压设置
                      OFF);                    //电压继电器断开
     }		                 
} 
/********************************************************
* 检测标准电能脉冲
********************************************************/
void Check_Std_Eclk(void)
{
    if((u16)(Timer_1ms-STD_ECLK_Timer)<STD_ECLK_OVTM)
     return;
    STD_ECLK_Timer=(u16)Timer_1ms;             //重置定时器
    NO_STD_ENG=1;                              //没有标准脉冲
    TimerDisable(TIMER1_BASE,TIMER_B);         //Timer1-B禁能
    TimerIntClear(TIMER1_BASE,TIMER_CAPB_MATCH);//清溢出中断
    TimerLoadSet(TIMER1_BASE,TIMER_B,0xFFFF);  //重置计数 0xFFFF
    TimerEnable(TIMER1_BASE,TIMER_B);          //Timer1-B启动
}
/********************************************************
* 误差计算板工作模式处理
********************************************************/
void Work_Mode_Pr(void) 
{
      switch(WORK_MODE)
       {
        case CAL_ENG_ERR_M:   ENG_ERR_MODE_Pr();     break; //00被检表电能脉冲误差模式处理
        case MTR_PLUG_M:      MTR_PLUG_MODE_Pr();    break; //01挂表模式处理
        case CATCH_HB_M:      CATCH_HB_MODE_Pr();    break; //02对黑斑模式处理
        case START_STOP_M:    START_STOP_MODE_Pr();  break; //03启动 潜动模式下 监视电能脉冲计数 
        case VERIFY_READY_M:  VERIFY_READY_MODE_Pr();break; //04准备开始校核常数走字模式 继电器吸合
        case VERIFY_START_M:  VERIFY_START_MODE_Pr();break; //05校核常数走字试验模式
        case VOLTAGE_DOWN_M:  VOLTAGE_DOWN_MODE_Pr();break; //06电压跌落试验模式
        case MEASURE_ENG_M:   MEASURE_ENG_MODE_Pr(); break; //07进入计电能试验 计量被检表电能
        case PANZHUAN_ERR_M:  PANZHUAN_ERR_MODE_Pr();break; //08进入盘转误差试验
        case MEA_CST_M:       MEA_CST_MODE_Pr();     break; //09常数测试试验
        case MEA_POWER_D_M:   MEA_POWER_D_MODE_Pr(); break; //10功耗测量试验
        case MEA_ENG_DUTY_M:  MEA_ENG_DUTY_MODE_Pr();break; //11测量电能脉冲周期和占空比
        case PULSE_ZZ_M:      PULSE_ZZ_MODE_Pr();    break; //12定脉冲走字试验
        case NYSY_M:          NYSY_MODE_Pr();        break; //13耐压试验 并记录耐压状态下脉冲状态
        case GDGZ_M:          GDGZ_MODE_Pr();        break; //14接地故障试验
        case ZBTX_M:          ZBTX_MODE_Pr();        break; //15载波通信试验        
       }
}        
/*****************************************************************************
* AD采样定时超时处理
*****************************************************************************/
void Proc_ADC_Timer(void)
{
    if(ADC_Data.Trig)                             //是否已经触发AD采样
     {
       if((u16)(Timer_1ms-ADC_Timer)<AD_OVER_TIME)//判断AD采样是否超时
        return;
       Init_Adc();                                //重新初始化ADC 
     }		
    else
     {	
       if((u16)(Timer_1ms-ADC_Timer)<AD_TRIG_TIME)//判断AD采样定时是否到
        return;
       ADCProcessorTrigger(ADC0_BASE,0);          //启动采样
     	 ADC_Timer=(u16)Timer_1ms;                  //重置采样定时 
       ADC_Data.Trig=1;
     }
}
/*****************************************************************************
* 处理AD采样值 
* 是否累加
*****************************************************************************/
void Proc_ADC_Data(void)
{
    if(ADC_Start!='Y')
    	return;
    if(ADC_Data.New_Data)            //判断是否有新数据
    {
       u16 Data;
       float f;
    
       ADC_Data.New_Data=0;          //清除新数据标志 
       Data=ADC_Data.Data;           //拷贝采样值
       f=Data*ADC0_Vref*ADC0_XiuZ/4096*100;
       f+=0.0005;                  // 
       Data=(u16)f;
       TEMP_STR[0]=(u8)((Data&0xff00)>>8);
       TEMP_STR[1]=(u8)(Data&0x00ff);
        
       Send_Data(ERR_OCMD_ADCONV,
					            2,
					            TEMP_STR);
       ADC_Data.Data=0;  
    }
}


