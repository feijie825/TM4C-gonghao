/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CLK_ERR.c
;* Author             : 张力阵
;* 时钟处理函数库
*******************************************************************************/
#include "Function.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "CLK_ERR.h"
#include "disp.h"
#include "ENG_ERR.h"
/********************************************************
* 读HC165数据子程序,两片HC165级联
* HC165最高位数据直接在输出数据线上，不需移位，直接读出
* 所以只需移位15次 就能得到16位数据
********************************************************/
u16 READ_HC165_DATA(void)
{
	   u16 d;
				d=0;                                                    //表位信息数据清零
				if(YK2NO_SIG_IN)                                        //遥控信号2常开采样
					d|=0x0004;                                               
				if(YK2NC_SIG_IN)                                        //遥控信号2常闭采样    
					d|=0x0008;                                               
				if(YK3NO_SIG_IN)                                        //遥控信号3常开采样
					d|=0x0010;    
				if(YK3NC_SIG_IN)                                        //遥控信号3常闭采样
					d|=0x0020;				
				if(YK4NO_SIG_IN)                                        //遥控信号4常开采样
					d|=0x0040;    
				if(YK4NC_SIG_IN)                                        //遥控信号4常闭采样
					d|=0x0080;				
				if(BJ_SIG_IN)                                           //报警信号采样
					d|=0x0100;
				if(YK1NO_SIG_IN)                                        //遥控信号1常开采样
					d|=0x0400;                                               
				if(YK1NC_SIG_IN)                                        //遥控信号1常闭采样    
					d|=0x0200; 				
    return (d); 	                                           //返回数据
}
/********************************************************
* 写CD4094数据子程序
********************************************************/
void WRITE_CD4094_DATA(void)
{
/*
    u8 m;
    u16 d;
    d=*(u16*)&CD4094_DATA;	
    d&=CD4094_MSK;                    //屏蔽无效位   
    for(m=0;m<16;m++)
     {
      if(d&0x8000)                    //高位为1
       CD4094_DIN_H;                  //高电平
      else
       CD4094_DIN_L;                  //低电平
      d<<=1;
      CD4094_CLK_H;                   //上升沿 数据移入
      SysCtlDelay(4);                 //延时1us
      CD4094_CLK_L;                   //低电平
     }	 
    CD4094_DIN_L;                     //低电平
    CD4094_STR_H;                     //
    SysCtlDelay(4);                   //延时1us
    CD4094_STR_L;                     //
*/
}
/********************************************************
* 计算设置时钟频率重装值和重装次数
* 入口:Sts是否重装计数器 1:重装计数器 
* 时钟频率=标准晶振频率(1000000)*时钟脉冲数/标准脉冲数
*         =标准晶振频率(1000000)*(分频值*重装次数)/标准脉冲数 
* 为了缩短计算时间 先根据设定频率和测量时间 算出 标准晶振频率(1000000)*(分频值*重装次数) 
* 由于频率 重新设定是以10为基数 要满足中断超时和中断过频 实现无缝对接 
* 过程如下:	(不考虑分频系数)
* 假设设定频率为F,则理论中断时间为(1/F)=T,实际频率为f 实际中断时间为(1/f)=t
* 现超时时间定为2倍理论时间 即 T1,  理论中断过频时间定为 T2, 
* 原则 就是 T1/T2>10(频率调整基数), 即T1>10T2,为简化设计 设T1=2T, T2<T1/10 即T2<T/5
********************************************************/
void Cal_Clk_Reload(u8 Sts)   
{
    u32 m;
    float f1,f2;
    u32 TIMERx;                                //计数器
    u32 ulTimer,ulIntFlags;                    //TIMER_A TIMER_B 标志
    if(MTYPE==SOUTH)                           //判断是否为南网表
     {	
      TIMERx=TIMER5_BASE;                      //默认的电子脉冲计数器 
      ulTimer=TIMER_A;                         //默认A
      ulIntFlags=TIMER_CAPA_MATCH;             //中断溢出标志
     } 
    else                                       
     {	                                        //国网表
      TIMERx=TIMER4_BASE;                      //默认的时钟脉冲计数器 
      ulTimer=TIMER_B;                         //默认B
      ulIntFlags=TIMER_CAPB_MATCH;             //中断溢出标志
     }
    f1=CLK_FREQ_INT;                           //设定频率
    f1*=CLK_MEA_TIME;                          //乘以测量时间 为需要计量的时钟脉冲个数
    m=(u32)(f1+0.5);                           //四舍五入
    if(m==0)                                   //判断在设定的时间内是否有一个时钟脉冲
     m=1;                                      
    if(CLK_FREQ_INT<1)                         //频率小于1HZ 一秒计数小于1个
     {                                         
      CLK_RELOAD_VAL_N=1;                      //设定时间内收到脉冲个数为1 
      CLK_RELOAD_TIME_N=m;                     //重装次数
     }                                         
    else if(CLK_FREQ_INT<60000)                //频率小于60KHz                                
     {                                   
      CLK_RELOAD_VAL_N=(u16)(CLK_FREQ_INT+0.5);//重装值(分频系数)
      CLK_RELOAD_TIME_N=CLK_MEA_TIME;          //重装次数        
     }                                         
    else                                       //每秒计数超过最大分频值
     {                                         
      CLK_RELOAD_VAL_N=60000;                  //重装值(分频系数)
      CLK_RELOAD_TIME_N=(m/60000);             //重装次数
     }                                         
    STD_CLK_VAL_ONE=STD_CLK_FREQ;              //标准时钟频率
    STD_CLK_VAL_ONE*=CLK_RELOAD_VAL_N;         //分频值
//计算中断间隔时间 超时时间                    
    f1=CLK_RELOAD_VAL_N;                       //时钟分频值
    f1/=CLK_FREQ_INT;                          //中断时间	 设定频率
    f2=STD_CLK_FREQ;                      
    f2*=f1;                               
    STD_CLK_CNT_ONE=(u32)(f2+0.5);             //单次中断应计的标准晶振脉冲数
    OCLK_STB_RNG=(f2/20+0.5);                  //5% 0.04% 判断时钟脉冲是否稳定用
    if(OCLK_STB_RNG<50)                        
     OCLK_STB_RNG=50;                          //最小15个脉冲 50/1000000=15us
    OCLK_Cnt_RSet_Max=(f2/10+0.5);             //10% 0.0125% 判断被检表时钟频率设置是否正常 与实际频率误差在0.0125%内     
    if(OCLK_Cnt_RSet_Max<25)                   //
     OCLK_Cnt_RSet_Max=25;                     
    if(CLK_RELOAD_TIME_N==1)                   //次数是否为1
     {	                                       
      STD_CLK_CNT_SUM=STD_CLK_CNT_ONE;         //标准脉冲数 用来判断稳定和频率设置是否错误 
      STD_CLK_VAL_SUM=STD_CLK_VAL_ONE;         //总标准脉冲数 用于计算被检表时钟频率
      SCLK_STB_RNG=OCLK_STB_RNG;               //时钟脉冲稳定误差限(多次)
      SCLK_Cnt_RSet_Max=OCLK_Cnt_RSet_Max;     //合计数改变触发重设时钟频率最大值
     }
    else
     {	  
      STD_CLK_VAL_SUM=STD_CLK_VAL_ONE;
      STD_CLK_VAL_SUM*=CLK_RELOAD_TIME_N;      //总标准脉冲数 用于计算被检表时钟频率
      f2*=CLK_RELOAD_TIME_N;                   
      STD_CLK_CNT_SUM=(u32)(f2+0.5);           //综合中断周期应计的标准晶振脉冲数
      SCLK_STB_RNG=(f2/20+0.5);                //5% 判断时钟脉冲是否稳定用
      if(SCLK_STB_RNG<50)                      
       SCLK_STB_RNG=50;                        //最小15个脉冲 50/1000000=15us
      SCLK_Cnt_RSet_Max=(f2/10+0.5);         //10% 0.0125 
      if(SCLK_Cnt_RSet_Max<25)                 //
       SCLK_Cnt_RSet_Max=25;     
	                 
     }                                         
    f1*=1000;                                  //转化为ms
    if(f1<20)                                  //中断不低于20ms 测量频率不大于3M
     f1=20;                                    
    CLK_Timer_Max=2*f1;                        //超过设定时间2倍 
    CLK_Timer_Min=(CLK_Timer_Max/16);          //中断时间小于设定中断时间值的0.125 1/8
    if(Sts)                                    //开机不需要重新装载       
     {
      Sts=0;  
       {
        CLK_RELOAD_TIME_O=CLK_RELOAD_TIME_N;   //当前重装计数初始化 
        CLK_RELOAD_Cnt=CLK_RELOAD_TIME_N;      //当前重装计数初始化 
        Sts=0xFF;                              //	
       }	
       {
       	Sts=0xFF;                              //
       	CLK_RELOAD_VAL_O=CLK_RELOAD_VAL_N;     //分频值保存	
        TimerDisable(TIMERx,                   //定时器
                      ulTimer);                //定时器启动
        TimerLoadSet(TIMERx,                   //定时器
                     ulTimer,                  //通道
                     CLK_RELOAD_VAL_N);        //设置(分频系数)
        TimerMatchSet(TIMERx,                  //定时器
                      ulTimer,                 //通道
                      0);
        TimerIntClear(TIMERx,                  //清溢出中断
                      ulIntFlags);
        if((MFClk_Mode!=UNION_PLS)||           //判断多功能脉冲是否为联合脉冲
           ((MFClk_Type==SZCLK_PLS)&&          //判断是否为测量时钟脉冲周期
            (WORK_MODE!=MEA_ENG_DUTY_M)))      //不在测量脉冲周期状态
         {   
          TimerEnable(TIMERx,                  //定时器
                      ulTimer);                //定时器启动
         }
       }
      if(Sts)
       {	   
        FIRST_CLK_PLS=1;                       //重新测量
        CLK_Timer=0;                           //重置定时器 
       } 
     }
}
/********************************************************
* 定时检测标准时钟脉冲是否存在
* 1S检测一次 不中断重新配置
********************************************************/
void Check_Std_Clk(void)      
{
    if((u8)(Timer_8ms-STD_CLK_Timer)<STD_CLK_OVTM)
     return;
    STD_CLK_Timer=Timer_8ms;
    NO_STD_CLK=1;                                 //标准时钟脉冲不存在标志 
    TimerDisable(TIMER2_BASE,TIMER_A);            //Timer2-A禁能
    TimerIntClear(TIMER2_BASE,TIMER_CAPA_MATCH);  //清溢出中断
    TimerLoadSet(TIMER2_BASE,TIMER_A,0xFFFF);     //重置计数 0xFFFF
    TimerEnable(TIMER2_BASE,TIMER_A);             //Timer1-B启动
}  
/********************************************************
* 根据实际频率推算设定频率
********************************************************/
void Act_Freq_to_Set(void)
{
    float f=1.0;
    double d;
    u32 m;
    d=CLK_ACT_FREQ;
    for(;;)
     {
      if(d>=10.0)
       {
        d/=10.0;                          //转化成1到10之间的数 
        f*=10.0;
       }
      else if(d<1.0)
       {
        d*=10.0;
        f/=10.0;
       } 
      else
       break; 
     }
    d*=1000;
    d+=0.5; 
    m=(u32)d;
    CLK_FREQ_INT=m;                      //设定频率 按实际频率 0.05%化整
    CLK_FREQ_INT/=1000;                  //还原设定频率  
    CLK_FREQ_INT*=f;                     //
    if(CLK_FREQ_INT==0)                  //规格化频率为0
     CLK_FREQ_INT=CLK_FREQ_SET;          //	
} 
/********************************************************
* 处理时钟脉冲中断超时
* 显示 no cLP
********************************************************/
void No_SZMC_Pr(void)
{
    CLK_Timer=0;                        //定时器清零
    strcpy((char*)CANT_STR,
           "No SZMC");
    Send_Data(ERR_OCMD_CLK_FRQ,         //回送没有晶振脉冲
              7,                        
              CANT_STR);                //回送时钟频率
    if(Disp_Choose==DISP_CLK_FREQ)      //判断是否显示时钟频率
     {
      if(GZ_FLAG||NZTZ_FLAG)            //是否处于故障和跳闸状态
       return;                          //退出 不再显示其他内容	 
      if(SOLID_CFG.LED_NUM==LED_6)      //是否为6位数码管 
       {                                //显示 
        Disp_Buf[0]=DISP_n;             //显示no cLP
        Disp_Buf[1]=DISP_o;
        Disp_Buf[2]=DISP_BLANK;         //显示空白
        Disp_Buf[3]=DISP_c;
        Disp_Buf[4]=DISP_L;
        Disp_Buf[5]=DISP_P;
       }		
      else
       { 		                              //显示F no cLP
        Disp_Buf[0]=DISP_F;               //显示'F'
        Disp_Buf[1]=DISP_BLANK;           //显示空白
        Disp_Buf[2]=DISP_n;               //显示F no cLP
        Disp_Buf[3]=DISP_o;
        Disp_Buf[4]=DISP_BLANK;           //显示空白
        Disp_Buf[5]=DISP_c;
        Disp_Buf[6]=DISP_L;
        Disp_Buf[7]=DISP_P;
       }
     }
}           
/********************************************************
* 处理未检测到标准时钟脉冲
* 显示 no bCP
********************************************************/
void No_BSZMC_Pr(void)
{
    strcpy((char*)CANT_STR,
           "No JZMC");
    Send_Data(ERR_OCMD_CLK_FRQ,         //回送没有晶振脉冲
              7,                        
              CANT_STR);                //回送时钟频率
    if((Disp_Choose==DISP_CLK_FREQ)||   //判断是否显示时钟频率
       (Disp_Choose==DISP_XUL_TIME))    //显示需量周期
     {
      if(GZ_FLAG||NZTZ_FLAG)            //是否处于故障和跳闸状态
       return;                          //退出 不再显示其他内容	 
      if(SOLID_CFG.LED_NUM==LED_6)      //是否为6位数码管 
       {                                //显示 
        Disp_Buf[0]=DISP_n;             //显示no bCP
        Disp_Buf[1]=DISP_o;
        Disp_Buf[2]=DISP_BLANK;         //显示空白
        Disp_Buf[3]=DISP_b;
        Disp_Buf[4]=DISP_C;
        Disp_Buf[5]=DISP_P;
       }		
      else
       { 		                            //显示F no cLP
        Disp_Buf[0]=DISP_F;             //显示'F'
        Disp_Buf[1]=DISP_BLANK;         //显示空白
        Disp_Buf[2]=DISP_n;             //显示F no bCP
        Disp_Buf[3]=DISP_o;
        Disp_Buf[4]=DISP_BLANK;         //显示空白
        Disp_Buf[5]=DISP_b;
        Disp_Buf[6]=DISP_C;
        Disp_Buf[7]=DISP_P;
       }
     }
}  
/********************************************************
* 处理时钟脉冲不稳
* 显示 no Stb
********************************************************/
void Clk_No_Stable(void)
{
    if(Disp_Choose==DISP_CLK_FREQ)      //判断是否显示时钟频率
     {
      if(GZ_FLAG||NZTZ_FLAG)            //是否处于故障和跳闸状态
       return;                          //退出 不再显示其他内容	 
      if(SOLID_CFG.LED_NUM==LED_6)      //是否为6位数码管 
       {                                //显示 
        Disp_Buf[0]=DISP_n;             //显示no stb
        Disp_Buf[1]=DISP_o;
        Disp_Buf[2]=DISP_BLANK;         //显示空白
        Disp_Buf[3]=DISP_5;
        Disp_Buf[4]=DISP_t;
        Disp_Buf[5]=DISP_b;
       }		
      else
       { 		                            //显示F no cLP
        Disp_Buf[0]=DISP_F;             //显示'F'
        Disp_Buf[1]=DISP_BLANK;         //显示空白
        Disp_Buf[2]=DISP_n;             //显示F no stb
        Disp_Buf[3]=DISP_o;
        Disp_Buf[4]=DISP_BLANK;         //显示空白
        Disp_Buf[5]=DISP_5;
        Disp_Buf[6]=DISP_t;
        Disp_Buf[7]=DISP_b;
       }
     }
}  
/********************************************************
* 测量时钟频率和日计时误差
* 时钟频率=标准晶振频率(1000000)*时钟脉冲数/标准脉冲数
* STD_CLK_CNT_ONE=标准晶振频率(1000000)*时钟脉冲数(重装值)
* STD_CLK_CNT_SUM=标准晶振频率(1000000)*总时钟脉冲数(重装值)
* 日计时误差=(86400*实测晶振频率)/标定晶振频率-86400
* 程序中判断 当前频率规格值是否太小 中断时间超短
* Proc_Clk_OvTm() 子程序判断时钟频率设定值是否太大 太大中断时间超时
********************************************************/
void MEA_CLK_FREQ(void)      
{
    if(NEW_CLK_PLS)                           //收到新时钟脉冲(分频后脉冲)
     {                                        
      u32 Cnt;	  
      NEW_CLK_PLS=0;                          //清除收到新时钟脉冲标志
      SZCLK_SET_TooM_T=0;                     //清除临时中断过频 标志  
      if((WORK_MODE==MEA_ENG_DUTY_M)||        //判断是否在测量脉冲周期
      	 ((MFClk_Mode==UNION_PLS)&&           //判断脉冲是否为联合脉冲
      	  (MFClk_Type!=SZCLK_PLS)))           //判断联合脉冲状态下 是否正在测量时钟脉冲
       {
        if(MTYPE==SOUTH)                      //判断是否为南网表
         {	
          TimerDisable(TIMER5_BASE,           //定时器
                       TIMER_A);              //定时器启动
         } 
        else                                  
         {	                                   //国网表
          TimerDisable(TIMER4_BASE,           //定时器
                       TIMER_B);              //定时器启动
         }
        return;                               //退出
       }
      if(SZCLK_SET_TooM)                      //判断时钟频率设置是否太小标志 处理
       {                                      
       	SZCLK_SET_TooM=0;                     //时钟频率设置太小标志
       	if(CLK_FREQ_INT<100000)               //频率小于100kHz 
       	 {	                                  
          CLK_FREQ_INT*=10;                   //频率扩大10倍
          Cal_Clk_Reload(1);                  //自动调整测量重装值和重装次数
         }
        if(MTYPE==SOUTH)                      //判断是否为南网表
         {	
          TimerEnable(TIMER5_BASE,            //定时器
                      TIMER_A);               //定时器启动
         } 
        else                                  
         {	                                   //国网表
          TimerEnable(TIMER4_BASE,            //定时器
                      TIMER_B);               //定时器启动
         }
       	return;                               
       }	                                    
      if(!MTR_PLUG)                           //是否挂表
       return;                                //不挂表退出	
      if(GZ_FLAG||NZTZ_FLAG)                  //是否处于故障和跳闸状态
       return;                                //退出 不再显示其他内容	 
      if(CLK_MEA_CTL!=MEA_ORDER)              //是否允许测量
       return;                                //不允许测量 退出	
      if(FIRST_CLK_PLS)                       //第一个脉冲
       {                                      
        FIRST_CLK_PLS=0;                      //清除第一个脉冲标志
        PRE_CLK_VAL=CUR_CLK_VAL;              //更新本次值
        RAW_CLK_VAL=CUR_CLK_VAL;              //更新原始值
        OCLK_STB_CHK=0;                       //暂时不检查单次中断是否稳定
        SCLK_STB_CHK=0;                       //暂时不检查综合中断是否稳定
        CLK_RELOAD_Cnt=CLK_RELOAD_TIME_N;     //重置圈数
        return;                               
       }                                      
      CLK_RELOAD_Cnt--;                       
      if(CLK_RELOAD_Cnt==0)                   //判断设定分频计数是否到
       { 
        float f;                                     
        CLK_RELOAD_Cnt=CLK_RELOAD_TIME_N;     //重装计数值
        if(NO_STD_CLK)                        //判断标准时钟脉冲是否存在
         { 
          No_BSZMC_Pr();                      //显示没有标准时钟脉冲
          FIRST_CLK_PLS=1;                    //重新开始计量
          return;                             //退出
         }
        SCLK_Cnt_NSub=CUR_CLK_VAL-RAW_CLK_VAL;//实计脉冲数
        RAW_CLK_VAL=CUR_CLK_VAL;              //更新原始值
        PRE_CLK_VAL=CUR_CLK_VAL;              //更新本次值
        if(SCLK_Cnt_NSub>SCLK_Cnt_OSub)       //本次计数值与上次计数值比较
         Cnt=SCLK_Cnt_NSub-SCLK_Cnt_OSub;     //计算跳变脉冲数
        else
         Cnt=SCLK_Cnt_OSub-SCLK_Cnt_NSub;     //计算跳变脉冲数 
        SCLK_Cnt_OSub=SCLK_Cnt_NSub;          //保存计数值
        if(SCLK_Cnt_NSub>50000)               //0.1S计数
         {	
          CLK_ACT_FREQ=STD_CLK_VAL_SUM;       //标准时钟脉冲数
          CLK_ACT_FREQ/=SCLK_Cnt_NSub;        //计算出时钟频率
         }
        else
         return;                              //标准计数值太小 退出	
        if(SCLK_STB_CHK)                      //是否开始检查稳定 
         {
          if(Cnt>SCLK_STB_RNG)                //判断跳变是否超出范围
           {
            Clk_No_Stable();                  //时钟脉冲 没有稳定显示		   
            return;                           //重新计脉冲
           }
          else                                
           {
           	if(SCLK_Cnt_NSub>STD_CLK_CNT_SUM)  //本次计数值 与 标准设定(应该计的数) 比较
             Cnt=SCLK_Cnt_NSub-STD_CLK_CNT_SUM;//标准差
            else                               
             Cnt=STD_CLK_CNT_SUM-SCLK_Cnt_NSub;//与标准脉冲数差值
            if(Cnt>SCLK_Cnt_RSet_Max)          //差值太大 超过0.0125%
             {
              Act_Freq_to_Set();               //推算设定频率
              Cal_Clk_Reload(1);               //自动调整测量重装值和重装次数
             }
           } 
         }                                    
        else 	
         {	                                
          SCLK_STB_CHK=1;                     //
         }
        sprintf((char*)TEMP_STR,
                "%8.6f",
                CLK_ACT_FREQ);                //实测时钟频率
        Fill_Space_StrCpy(TEMP_STR,           //待拷贝的字符串
                          CLK_FREQ_ASC,       //待写入的字符串
                          9);                 //字符串最大长度                 
        CLK_DAY_ERR=CLK_ACT_FREQ;             //实测频率 日计时误差 =(86400*实测晶振频率)/标定晶振频率-86400
        CLK_DAY_ERR/=CLK_FREQ_SET;            //按规格值算日计时误差=(实测频率/标定频率-1)*86400) 
        if((CLK_DAY_ERR>1.0006944)||          //日计时误差是否在60秒以内
           (CLK_DAY_ERR<0.9993056))           //在60秒以内按设定频率算日计时误差       
         {                                    //60秒以外按实际频率规格化算日计时误差
          if(CLK_ACT_FREQ<10)		          
           {
            f=((u32)(CLK_ACT_FREQ*10000+0.5));
            f/=10000;
           }
          else if(CLK_ACT_FREQ<100)
           {
            f=((u32)(CLK_ACT_FREQ*1000+0.5));
            f/=1000;
           }
          else if(CLK_ACT_FREQ<1000)  	 	
           {
            f=((u32)(CLK_ACT_FREQ*100+0.5));
            f/=100;
           }
          else if(CLK_ACT_FREQ<10000)  	 	
           {
            f=((u32)(CLK_ACT_FREQ*10+0.5));
            f/=10;
           }
          else
           f=((u32)(CLK_ACT_FREQ+0.5));
          if(f!=0)                              //规格频率太低
           {	
            CLK_DAY_ERR=CLK_ACT_FREQ;            //实测频率 日计时误差 =(86400*实测晶振频率)/标定晶振频率-86400
            CLK_DAY_ERR/=f;                      //按规格值算日计时误差=(实测频率/标定频率-1)*86400) 
           }
         }
/***************************************************************/         	 
        CLK_DAY_ERR-=1;                       //一切从应用出发 去掉	时钟频率自适应(规格化)
        CLK_DAY_ERR*=86400;                   //计算出日计时误差
        sprintf((char*)TEMP_STR,
                "%8.6f",
                CLK_DAY_ERR);                 //日计时误差
        Fill_Space_StrCpy(TEMP_STR,           //待拷贝的字符串
                          DAY_ERR_ASC,        //待写入的字符串
                          9);                 //字符串最大长度                 
        if(Disp_Choose==DISP_CLK_FREQ)        //判断是否显示时钟频率
         {
          if(SOLID_CFG.LED_NUM==LED_6)        //是否为6位数码管
           Copy_Str_To_DSBUF(DISP_FREQ_LEN,   //时钟频率数据长度
                             0,               //时钟频率显示偏移量
                             CLK_FREQ_ASC);    //时钟频率ASC地址   
          else
           {		
            Disp_Buf[0]=DISP_F;                 //显示'F'
            Copy_Str_To_DSBUF(DISP_FREQ_LEN,    //时钟频率数据长度
                              DISP_FREQ_OFFSET, //时钟频率显示偏移量
                              CLK_FREQ_ASC);    //时钟频率ASC地址   
           }                                    
         }                                    
        else if(Disp_Choose==DISP_DAY_ERR)    //判断是否显示日计时误差
         {                                    
          Disp_Buf[0]=DISP_d;                 //显示'd'
          Disp_Buf[1]=DISP_BLANK;             //显示空
          Copy_Str_To_DSBUF(DISP_DAY_LEN,     //日计时误差数据长度
                            DISP_DAY_OFFSET,  //日计时误差显示偏移量
                            DAY_ERR_ASC);     //日计时误差ASC地址 
         }
        if(CLK_FREQ_ASC[7]=='.')              //时钟频率>=1000000Hz 回送7位数据
         Send_Data(ERR_OCMD_CLK_FRQ,          //回送脉冲周期命令
                   7,                         
                   CLK_FREQ_ASC);             //回送时钟频率
        else        	                        
         Send_Data(ERR_OCMD_CLK_FRQ,          //回送时钟频率 8位数据
                   8,                         
                   CLK_FREQ_ASC);             
        if(DAY_ERR_ASC[7]=='.')               //日计时误差>=1000000S 回送7位数据
         Send_Data(ERR_OCMD_DAY_ERR,          //回送脉冲周期命令
                   7,                         
                   DAY_ERR_ASC);              //日计时误差
        else        	                        
         Send_Data(ERR_OCMD_DAY_ERR,          //日计时误差 8位数据
                   8,
                   DAY_ERR_ASC);
       }
      else
       {
       	if(NO_STD_CLK)                        //没有测到晶振脉冲
       	 return;
        OCLK_Cnt_NSub=CUR_CLK_VAL-PRE_CLK_VAL;//本次计数值
        PRE_CLK_VAL=CUR_CLK_VAL;              //更新上次采样值
        if(OCLK_Cnt_NSub>OCLK_Cnt_OSub)       //本次两次中断间计数值 与上次两次中断间计数值比较
         Cnt=OCLK_Cnt_NSub-OCLK_Cnt_OSub;     //跳变脉冲数
        else                                  
         Cnt=OCLK_Cnt_OSub-OCLK_Cnt_NSub;     //跳变脉冲数
        OCLK_Cnt_OSub=OCLK_Cnt_NSub;            
        if(OCLK_STB_CHK)                      //是否开始检查稳定
         {
          if(Cnt>OCLK_STB_RNG)                //判断跳变是否超出范围
           {
            Clk_No_Stable();                  //时钟脉冲 没有稳定显示		   
            return;                           //重新计脉冲
           }
          else
           {	
            if(OCLK_Cnt_NSub>STD_CLK_CNT_ONE)   //本次计数值 与 标准设定(应该计的数) 比较
             Cnt=OCLK_Cnt_NSub-STD_CLK_CNT_ONE; //标准差
            else                                
             Cnt=STD_CLK_CNT_ONE-OCLK_Cnt_NSub; //标准差
            if(Cnt>OCLK_Cnt_RSet_Max)           //实际频率与设定频率是否一致(在0.0125%范围内)
             {
              CLK_ACT_FREQ=STD_CLK_VAL_ONE;     //标准时钟脉冲数
              CLK_ACT_FREQ/=OCLK_Cnt_NSub;      //计算出实际的时钟频率
              Act_Freq_to_Set();                //推算设定频率
              Cal_Clk_Reload(1);                //自动调整测量重装值和重装次数
             }
           }
         }
        else 	
         OCLK_STB_CHK=1;                        //
       }  
     }
}
/********************************************************
* 处理时钟脉冲中断超时
* 判断时钟频率设定值是否太大 太大中断时间超时
********************************************************/
void Proc_Clk_OvTm(void)       
{
    if(WORK_MODE==MEA_ENG_DUTY_M)           //判断是否在测量脉冲周期
     return;                                //退出
    if(MFClk_Mode==UNION_PLS)               //判断脉冲是否为联合脉冲
     if(MFClk_Type!=SZCLK_PLS)              //判断联合脉冲状态下 是否正在测量需量周期
      return;                               //不在测量时钟脉冲 退出		
    if(NO_CLK_PLS)                          //判断是否检测到时钟脉冲标志
     {
      if(CLK_Timer>30000)                   //30S回送一次
       {
       	CLK_Timer=0;                        //	
        No_SZMC_Pr();                       //无时钟脉冲处理
       }  
      return;	    	
     }
    else if(CLK_Timer>=CLK_Timer_Max)       //超时未到 退出
     {
      CLK_Timer=0;                          //重置定时器
      SZCLK_SET_TooM_T=0;                   //超时 清除首次中断过频标志
      if(CLK_FREQ_INT<0.008)                //当前设定频率小于0.008HZ 停止测量
       {
       	NO_CLK_PLS=1;                       //置位没有时钟脉冲标志
       	CLK_FREQ_INT=CLK_FREQ_SET;          //置为默认值
        No_SZMC_Pr();                       //无时钟脉冲处理
       }	
      else                                  //未到最小设定频率
       {
       	NO_CLK_PLS=0;
       	CLK_FREQ_INT/=10;                   //设定频率
       } 
      Cal_Clk_Reload(1);                    //自动调整测量重装值和重装次数
      if(MTYPE==SOUTH)                      //判断是否为南网表
       {	
        TimerEnable(TIMER5_BASE,            //定时器
                    TIMER_A);               //定时器启动
       } 
      else                                  
       {	                                   //国网表
        TimerEnable(TIMER4_BASE,            //定时器
                    TIMER_B);               //定时器启动
       }
     }
}
/********************************************************
* 测量需量周期
********************************************************/
void MEA_XUL_TIME(void)        
{
    if(WORK_MODE==MEA_ENG_DUTY_M)           //判断是否在测量脉冲周期
     return;                                //退出
    if(MFClk_Mode==UNION_PLS)               //判断脉冲是否为联合脉冲
     if(MFClk_Type!=XULCLK_PLS)             //判断联合脉冲状态下 是否正在需量周期
      return;                               //不在测量时钟脉冲 退出		
    if(NEW_XUL_PLS)                         //收到新时钟脉冲
     {
      u32 Cnt;
      NEW_XUL_PLS=0;                        //清除收到新时钟脉冲标志
      if(!MTR_PLUG)                         //是否挂表
       return;                              //不挂表退出	
      if(GZ_FLAG||NZTZ_FLAG)                //是否处于故障和跳闸状态
       return;                              //退出 不再显示其他内容	 
      if(XUL_MEA_CTL!=MEA_ORDER)            //判断是否设置为正常测量状态
       return;	
      if(FIRST_XUL_PLS)                     //第一个需量脉冲
       {                                    
        FIRST_XUL_PLS=0;                    //清除第一个脉冲标志
        PRE_XUL_VAL=CUR_XUL_VAL;            //更新本次值
        RAW_XUL_VAL=CUR_XUL_VAL;            //更新原始值
        XUL_STB_CHK=0;                      //第一次不判断稳定
        XUL_RELOAD_Cnt=XUL_RELOAD_TIME;     //重置次数
        return;                           
       }                                  
      XUL_RELOAD_Cnt--;
      if(XUL_RELOAD_Cnt==0)                 //判断设定分频计数是否到
       {  
        if(NO_STD_CLK)                      //判断标准时钟脉冲是否存在
         { 
          No_BSZMC_Pr();                    //
          FIRST_XUL_PLS=1;                  //重新开始计量
          return;                           //退出
         }
        XUL_RELOAD_Cnt=XUL_RELOAD_TIME;     //重装计数值
        SXUL_Cnt_NSub=CUR_XUL_VAL-RAW_XUL_VAL;//实计脉冲数
        RAW_XUL_VAL=CUR_XUL_VAL;              //更新原始值
        PRE_XUL_VAL=CUR_XUL_VAL;              //更新本次值
        if(SXUL_Cnt_NSub<50000)             //标准时钟脉冲计数值是否太小 0.1S计数值
         return;                            //退出
        if(SXUL_Cnt_NSub>SXUL_Cnt_OSub)
         Cnt=SXUL_Cnt_NSub-SXUL_Cnt_OSub;   //跳变脉冲数
        else
         Cnt=SXUL_Cnt_OSub-SXUL_Cnt_NSub;
        SXUL_Cnt_OSub=SXUL_Cnt_NSub;
        if(XUL_RELOAD_TIME==1)              //单次 判断稳定	
         {		
          if(Cnt>(SXUL_Cnt_NSub>>6))        //跳变超过1.56% 认为不稳定
           return;	
         }  	 	
        XUL_TIME=SXUL_Cnt_NSub;             //标准时钟计数值  
        XUL_TIME/=STD_CLK_FREQ;             //除以标准时钟频率即为需量周期 单位:s
        XUL_TIME/=60;                       //转换为分钟
        memset(XUL_TIME_ASC,0,8);           //清除需量周期ASC
        sprintf((char*)TEMP_STR,
                "%8.6f",
                XUL_TIME);                  //实测需量周期
        Fill_Space_StrCpy(TEMP_STR,         //待拷贝的字符串
                          XUL_TIME_ASC,     //待写入的字符串
                          9);               //字符串最大长度                 
        if(Disp_Choose==DISP_XUL_TIME)      //判断是否在显示需量周期模式
         {
          Disp_Buf[0]=DISP_L;               //显示'L'
          Disp_Buf[1]=DISP_BLANK;           //显示空
          Copy_Str_To_DSBUF(DISP_XUL_LEN,   //需量周期数据长度
                            DISP_XUL_OFFSET,//需量周期显示偏移量
                            XUL_TIME_ASC);  //需量周期ASC地址   
         }
        if(XUL_TIME_ASC[7]=='.')            //需量周期>=1000000分钟 回送7位数据
         Send_Data(ERR_OCMD_XULZQ,          //回送脉冲周期命令
                   7,                         
                   XUL_TIME_ASC);           //回送需量周期
        else        	                        
         Send_Data(ERR_OCMD_XULZQ,          //回送需量周期 8位数据
                   8,                         
                   XUL_TIME_ASC);             
       }
      else
       {
       	if(NO_STD_CLK)                        //没有测到晶振脉冲
       	 return;
        OXUL_Cnt_NSub=CUR_XUL_VAL-PRE_XUL_VAL;//本次计数值
        PRE_XUL_VAL=CUR_XUL_VAL;              //更新上次采样值
        if(OXUL_Cnt_NSub>OXUL_Cnt_OSub)       //本次两次中断间计数值 与上次两次中断间计数值比较
         Cnt=OXUL_Cnt_NSub-OXUL_Cnt_OSub;     //跳变脉冲数
        else                                  
         Cnt=OXUL_Cnt_OSub-OXUL_Cnt_NSub;     //跳变脉冲数
        OXUL_Cnt_OSub=OXUL_Cnt_NSub;            
        if(XUL_STB_CHK)                       //
         {	
          if(Cnt>(OXUL_Cnt_NSub>>6))          //跳变超过1.56% 认为不稳定
           {                                  //不稳定
            RAW_XUL_VAL=CUR_XUL_VAL;          //更新上次采样值
            XUL_RELOAD_TIME=XUL_RELOAD_Cnt;   //重置计数值
           }	
         }
        else
         XUL_STB_CHK=1;                       //	   
       }  
     }
}
/*****************************************************************************
* 处理按键
*****************************************************************************/
void Key_Proc(void)
{
    if(PLUG_CHG_FLAG)                         //判断是否为挂表
     {
      PLUG_CHG_FLAG=0;                        //清除挂表设置改变标志
      NEW_KEY_FLAG=0;	                        //清除按键标志
      if(MTR_PLUG)                            //判断挂表状态是否改变
       MTR_PLUG=0;                            //设置为不挂表
      else
       MTR_PLUG=1;                            //挂表标志
      NEW_CMD=1;                              //新命令处理
      WORK_MODE=MTR_PLUG_M;                   //挂表状态
      KEY_PC_PLUG=1;                          //按键选择挂表 不延时
     }
    else if(NEW_KEY_FLAG)                     //判断是否有按键中断
     {
      if((u8)(Timer_8ms-Con_KEY_Timer)<
      	       KEY_PLUG_TIME)                  //是否
       return;	
      NEW_KEY_FLAG=0;                         //不为挂表设置改变 清除按键标志
      ELEC_HEAD_RESET_Pr();                   //光电头对光 误差清零	
     }		
}
/*****************************************************************************
* IO口中断重使能处理
*****************************************************************************/
void  IO_INT_REEN(void)        
{
    if(GDT_INT_REEN)
     if((u8)(Timer_8ms-GDT_Timer)>=GDT_REN_TIME)
      {                                     
       GDT_INT_REEN=0;
       if(WORK_MODE!=MEA_ENG_DUTY_M)       //判断是否在测量脉冲周期状态
        GPIOIntTypeSet(GDT_MC_GPIO,        //端口
                       GDT_MC,             //管脚
                       GPIO_FALLING_EDGE); //下降沿中断         
       GPIOIntClear(GDT_MC_GPIO,           //端口 清除端口中断
                    GDT_MC);               //管脚
       GPIOIntEnable(GDT_MC_GPIO,          //端口
                     GDT_MC);              //管脚 中断使能                   
      }
    if(DZ_INT_REEN)
     if((u8)(Timer_8ms-DZ_Timer)>=DZ_REEN_TIME)
      { 
       u8  ucPins;                         //管脚定义
       u32 GPIOx;                          //端口				
       if(MTYPE==SOUTH)                    //判断是否为南网表
				    {  
         ucPins=CYCLE_PIN_TAB[SZ_PLS-GDT_PLS]; //选择时钟脉冲输入口作为 电子脉冲输入 南网表
         GPIOx=CYCLE_PORT_TAB[SZ_PLS-GDT_PLS]; //选择时钟脉冲输入口作为 电子脉冲输入 南网表
        }
       else         
				    { 
         ucPins=CYCLE_PIN_TAB[DZ_PLS-GDT_PLS]; //选择电子脉冲输入口作为 电子脉冲输入 国网表
         GPIOx=CYCLE_PORT_TAB[DZ_PLS-GDT_PLS]; //选择电子脉冲输入口作为 电子脉冲输入 国网表
        }
       DZ_INT_REEN=0;                      //清除重使能标志                                  
       if(WORK_MODE!=MEA_ENG_DUTY_M)       //判断是否在测量脉冲周期状态
        GPIOIntTypeSet(GPIOx,               //端口
                       ucPins,              //管脚
                       GPIO_FALLING_EDGE);  //下降沿中断         
       GPIOIntClear(GPIOx,                 //端口 清除端口中断
                    ucPins);               //管脚
       GPIOIntEnable(GPIOx,                //端口
                     ucPins);              //管脚 中断使能                   
      }
    if(SZ_INT_REEN)
     if((u8)(Timer_8ms-SZ_Timer)>=SZ_REEN_TIME)
      {
       u8 ucPins;                          //管脚定义
       u32 GPIOx;                          //端口				
       if(MTYPE==SOUTH)                    //判断是否为南网表
        {
         ucPins=CYCLE_PIN_TAB[DZ_PLS-GDT_PLS]; //选择电子脉冲输入口作为 时钟脉冲输入 南网表
         GPIOx=CYCLE_PORT_TAB[DZ_PLS-GDT_PLS]; //选择电子脉冲输入口作为 时钟脉冲输入 南网表
        }
       else
        {         
         ucPins=CYCLE_PIN_TAB[SZ_PLS-GDT_PLS]; //选择时钟脉冲输入口作为 时钟脉冲输入 国网表
         GPIOx=CYCLE_PORT_TAB[SZ_PLS-GDT_PLS]; //选择时钟脉冲输入口作为 时钟脉冲输入 国网表
        }
       SZ_INT_REEN=0;                       //清除重使能标志  
       if(WORK_MODE!=MEA_ENG_DUTY_M)        //判断是否在测量脉冲周期状态
        GPIOIntTypeSet(GPIOx,               //端口
                       ucPins,              //管脚
                       GPIO_FALLING_EDGE);  //下降沿中断         
       GPIOIntClear(GPIOx,                  //端口 清除端口中断
                    ucPins);                //管脚
       GPIOIntEnable(GPIOx,                 //端口
                     ucPins);               //管脚 中断使能                   
      }	                                   
    if(XUL_INT_REEN)
     if((u8)(Timer_8ms-XUL_Timer)>=XUL_REEN_TIME)
      {                                     
       XUL_INT_REEN=0;                      //清除重使能标志 
       if(WORK_MODE!=MEA_ENG_DUTY_M)        //判断是否在测量脉冲周期状态
        GPIOIntTypeSet(XL_MC_GPIO,          //端口
                       XL_MC,               //管脚
                       GPIO_FALLING_EDGE);  //下降沿中断         
       GPIOIntClear(XL_MC_GPIO,             //端口 清除端口中断
                    XL_MC);	                //管脚
       GPIOIntEnable(XL_MC_GPIO,            //端口
                     XL_MC);                //管脚 中断使能                   
      }
    if(TQ_INT_REEN)
     if((u8)(Timer_8ms-TQ_Timer)>=TQ_REEN_TIME)
      {                                     
       TQ_INT_REEN=0;                      //清除重使能标志 
       if(WORK_MODE!=MEA_ENG_DUTY_M)       //判断是否在测量脉冲周期状态
        GPIOIntTypeSet(TQ_MC_GPIO,         //端口
                       TQ_MC,              //管脚
                       GPIO_FALLING_EDGE); //下降沿中断         
       GPIOIntClear(TQ_MC_GPIO,            //端口 清除端口中断
                    TQ_MC);	               //管脚
       GPIOIntEnable(TQ_MC_GPIO,           //端口
                     TQ_MC);               //管脚 中断使能                   
      }
/*						
    if(HZ_INT_REEN)
     if((u8)(Timer_8ms-HZ_Timer)>=HZ_REEN_TIME)
      {                                     
       HZ_INT_REEN=0;                      //清除重使能标志 
       if(WORK_MODE!=MEA_ENG_DUTY_M)       //判断是否在测量脉冲周期状态
        GPIOIntTypeSet(HZ_MC_GPIO,         //端口
                       HZ_MC,              //管脚
                       GPIO_FALLING_EDGE); //下降沿中断         
       GPIOIntClear(HZ_MC_GPIO,            //端口 清除端口中断
                    HZ_MC);	               //管脚
       GPIOIntEnable(HZ_MC_GPIO,           //端口
                     HZ_MC);               //管脚 中断使能                   
      }	
*/						
    if(KEY_INT_REEN)
     if((u8)(Timer_8ms-KEY_Timer)>=KEY_REEN_TIME)
      {                                     
       KEY_INT_REEN=0;                     //清除重使能标志 
       GPIOIntTypeSet(KEY_IN_GPIO,         //端口
                      KEY_IN,              //管脚
                      GPIO_FALLING_EDGE);  //下降沿中断         
       GPIOIntClear(KEY_IN_GPIO,           //端口 清除端口中断
                    KEY_IN);	              //管脚
       GPIOIntEnable(KEY_IN_GPIO,          //端口
                     KEY_IN);              //管脚 中断使能                   
      }	
}
/*****************************************************************************
* 定时处理HC165数据
* HC165 D0  负控遥控信号1
* HC165 D1  负控遥控信号2
* HC165 D2  负控遥控信号3
* HC165 D3  负控遥控信号4
* HC165 D4  负控遥控信号5
* HC165 D5  负控遥控信号6
* HC165 D6  负控遥控信号7
* HC165 D7  负控遥控信号8
* HC165 D8  电能表报警信号
* HC165 D9  电能表跳闸继电器常闭信号
* HC165 D10 电能表跳闸继电器常开信号
* HC165 D11 旁路继电器故障报警信号
* HC165 D12 旁路继电器接入(跳闸)状态信号
* HC165 D13 互感器报警
* HC165 D14 耐压击穿信号
* HC165 D15 表放好标志，1有效
*****************************************************************************/
void Proc_HC165(void)
{
	   u16 x,m,t,d;
    if(Timer_1ms<POWER_UP_TIME)                //没有上电稳定
     return;                                   //上电未稳定退出	
    if((u8)(Timer_1ms-HC165_Timer)<HC165_TIME) //判断采样定时是否到，20ms
     return;
    HC165_Timer=(u8)Timer_1ms;                 //重启定时 
    d=READ_HC165_DATA();                       //读HC165数据
    d&=HC165_MSK;                              //屏蔽无效位
    x=(d^HC165_TDATA);                         //数据是否改变	数据改变的位为1 即为1的位为不稳定
    HC165_TDATA=d;                             //保存临时数据
    d&=(~x);                                   //稳定的位 值为1的位
    m=*(u16*)&HC165_DATA;                       //状态发生变化的位
    HC165_XDATA=(m^d);                         //稳定的位是否发生变化
    HC165_XDATA&=(~x);                         //去掉不稳定的位的改变
    m&=x;                                      //与掉稳定的位
    m|=d;                                      //更新稳定的位的状态 
    *(u16*)&HC165_DATA=m;                      //更新数据
    t=0;
    if((u16)(Timer_1ms-MBJ_Send_Timer)>MBJ_SEND_TIME)
     t=0xFF;                                   //定时标志
    if((HC165_XDATA&MTR_BJ_BIT)||t)            //BIT0 电能表报警信号是否改变
     {                                         
      MBJ_Send_Timer=(u16)Timer_1ms;           //重启定时
      if(MTR_PLUG)
       {	
        NEW_MBJ_PLS=1;                          
        if(HC165_DATA.MTR_BJ==0)               //收到表报警信号
         MBJ_DATA='1';
        else
         MBJ_DATA='0';                         //表报警消失
       }  
     }
/*					
				if(MTR_MOD==Smart_Meter)                   //正在检测的设备是智能电表
				{	
      t=0;
      if((u16)(Timer_1ms-WZTZ_Send_Timer)>
      	        WZTZ_SEND_TIME)                 //定时检测跳闸继电器状态数据
       t=0xFF;                                   //定时标志
      if((HC165_XDATA&JDQ_OPN_BIT)||             //BIT2 继电器常开是否改变
         (HC165_XDATA&JDQ_CLS_BIT)||t)           //BIT1 继电器常闭 是否改变
      {                                         
        WZTZ_Send_Timer=(u16)Timer_1ms;          //重启定时
        if(MTR_PLUG)                             //是否挂表
        {	
          WZHZ_DATA[0]='0';                      //常开未跳闸
          WZHZ_DATA[1]='0';                      //常闭无效
          NEW_WZHZ_PLS=1;                        //   
          WZTZ_FLAG=0;                           //外置跳闸标志
          if(DXTZ&0x80)                          //是否设置为单相费控表
           {                                     
            if(HC165_DATA.JDQ_OPN==(DXTZ&0x01))  //状态与设定状态是否一致
             {	
              WZHZ_DATA[0]='1';                  //常开跳闸
              WZHZ_DATA[1]='1';                  //常闭跳闸
             }
           }                                     
          else 
           {
            if(DXTZ&0x02)                            //河北三相单触点,接13端子或者15端子
             {
              if(HC165_DATA.JDQ_OPN==1)
               {
                 WZHZ_DATA[0]='1';                  //13号端子跳闸信号动作
                 WZHZ_DATA[1]='1';
               }
             }
            else if(DXTZ&0x04)
             {
              if(HC165_DATA.JDQ_CLS==0)
               {
                WZHZ_DATA[0]='1';                  
                WZHZ_DATA[1]='1';																		//15号端子跳闸信号动作
               }
             }
            else                                   //常规国网表双触点
             {	
              if(HC165_DATA.JDQ_OPN==0)            //为低电平
               WZHZ_DATA[0]='1';                   //常开跳闸
              if(HC165_DATA.JDQ_CLS==0)            //为低电平
               WZHZ_DATA[1]='1';                   //常闭跳闸
             }
          }									
          if((WZHZ_DATA[0]=='1')&&
             (WZHZ_DATA[1]=='1'))
           {
            WZTZ_FLAG=1;                           //外置跳闸标志
            TZZS_LED_ON;                           //点亮跳闸指示灯
           }                                                 
          else                                               
           {                                                 
            WZTZ_FLAG=0;                                     
            if(!NZTZ_FLAG)                         //判断外置是否跳闸	                          
             TZZS_LED_OFF;                         //关闭跳闸指示灯
           }                                                 
        }                                                    
      }                                                      
			 }                                                        
*/
				if(MTR_MOD==Smart_Meter)                   //正在检测的设备是智能电表
				{	
      t=0;
      if((u16)(Timer_1ms-WZTZ_Send_Timer)>
      	        WZTZ_SEND_TIME)                 //定时检测跳闸继电器状态数据
       t=0xFF;                                   //定时标志
      if((HC165_XDATA&JDQ_OPN_BIT)||             //BIT2 继电器常开是否改变
         (HC165_XDATA&JDQ_CLS_BIT)||t)           //BIT1 继电器常闭是否改变
      {                                         
        WZTZ_Send_Timer=(u16)Timer_1ms;          //重启定时
        if(MTR_PLUG)                             //是否挂表
        {	
          WZHZ_DATA[0]='0';                      //常开未跳闸
          WZHZ_DATA[1]='0';                      //常闭无效
          NEW_WZHZ_PLS=1;                        //   
          WZTZ_FLAG=0;                           //外置跳闸标志
          if(DXTZ&0x80)                          //是否设置为单相费控表
           {                                     
            if(HC165_DATA.JDQ_OPN==(DXTZ&0x01))  //状态与设定状态是否一致
             {	
              WZHZ_DATA[0]='1';                  //常开跳闸
              WZHZ_DATA[1]='1';                  //常闭跳闸
             }
           }                                     
          else 
           {
            if(DXTZ&0x02)                            //河北三相单触点,接13端子或者15端子
             {
              if(HC165_DATA.JDQ_OPN==1)
               {
                 WZHZ_DATA[0]='1';                  //13号端子跳闸信号动作
                 WZHZ_DATA[1]='1';
               }
             }
            else if(DXTZ&0x04)
             {
              if(HC165_DATA.JDQ_CLS==1)
               {
                WZHZ_DATA[0]='1';                  
                WZHZ_DATA[1]='1';																		//15号端子跳闸信号动作
               }
             }
            else                                   //常规国网表双触点
             {	
              if(HC165_DATA.JDQ_OPN==0)            //为低电平
               WZHZ_DATA[0]='1';                   //常开跳闸
              if(HC165_DATA.JDQ_CLS==1)            //为低电平
               WZHZ_DATA[1]='1';                   //常闭跳闸
             }
          }									
          if((WZHZ_DATA[0]=='1')&&
             (WZHZ_DATA[1]=='1'))
           {
            WZTZ_FLAG=1;                           //外置跳闸标志
            TZZS_LED_ON;                           //点亮跳闸指示灯
           }                                                 
          else                                               
           {                                                 
            WZTZ_FLAG=0;                                     
            if(!NZTZ_FLAG)                         //判断外置是否跳闸	                          
             TZZS_LED_OFF;                         //关闭跳闸指示灯
           }                                                 
        }                                                    
      }                                                      
			 }                                                        
			 else                                                     //当前正在校验的设备非智能电表
			 {                                                        
      if((HC165_XDATA&JDQ_OPN_BIT)||                         //BIT2 遥控信号1是否改变
         (HC165_XDATA&JDQ_CLS_BIT))                          //BIT1 遥控信号2是否改变
      {                                                      
        if(MTR_PLUG)                                         //是否挂表
        {	 
          NEW_LUNCI1_PLS=1;                                  //轮次1事件发生标志
          LUNCI1_SEND_COUNT=0;                               //轮次1信号发送次数清零
          LUNCI1_Send_Timer=(u16)(Timer_1ms-LUNCI1_SEND_TIME);    //轮次1信号发送定时器初始化
          if(HC165_XDATA&JDQ_OPN_BIT)                        //轮次1常开动作
          	LUNCI1_DATA[0]='1';                               //轮次1常开动作标志置位
          if(HC165_XDATA&JDQ_CLS_BIT)                        //轮次1常闭动作
          	LUNCI1_DATA[1]='1';                               //轮次1常闭动作标志置位
        }  
      }
      if((HC165_XDATA&YK_SIG3_BIT)||                         //BIT10 遥控信号3是否改变
      	  (HC165_XDATA&YK_SIG4_BIT))			 		                    //BIT11 遥控信号4是否改变
      {
							 if(MTR_PLUG)
								{						
									 NEW_LUNCI2_PLS=1;                                  //轮次2事件发生标志
									 LUNCI2_SEND_COUNT=0;                               //轮次2信号发送次数清零
									 LUNCI2_Send_Timer=(u16)(Timer_1ms-LUNCI2_SEND_TIME);    //轮次2信号发送定时器初始化
									 if(HC165_XDATA&YK_SIG3_BIT)                        //轮次2常开动作
											LUNCI2_DATA[0]='1';                               //轮次2常开动作标志置位
										if(HC165_XDATA&YK_SIG4_BIT)                        //轮次2常闭动作
											LUNCI2_DATA[1]='1';                               //轮次2常闭动作标志置位
								}
      }
						if((HC165_XDATA&YK_SIG5_BIT)||                         //BIT12 遥控信号5是否改变
      	  (HC165_XDATA&YK_SIG6_BIT))			 		                    //BIT13 遥控信号6是否改变
      {
							 if(MTR_PLUG)
								{							
									 NEW_LUNCI3_PLS=1;                                  //轮次3事件发生标志
									 LUNCI3_SEND_COUNT=0;                               //轮次3信号发送次数清零
									 LUNCI3_Send_Timer=(u16)(Timer_1ms-LUNCI3_SEND_TIME);    //轮次3信号发送定时器初始化
									 if(HC165_XDATA&YK_SIG5_BIT)                           //轮次3常开动作
											LUNCI3_DATA[0]='1';                               //轮次3常开动作标志置位
										if(HC165_XDATA&YK_SIG6_BIT)                           //轮次3常闭动作
											LUNCI3_DATA[1]='1';                               //轮次3常闭动作标志置位
								}      	
      }
						if((HC165_XDATA&YK_SIG7_BIT)||                         //BIT14 遥控信号7是否改变
      	  (HC165_XDATA&YK_SIG8_BIT))			 		                    //BIT15 遥控信号8是否改变
      {
      	 if(MTR_PLUG)
								{
									 NEW_LUNCI4_PLS=1;                                  //轮次4事件发生标志
									 LUNCI4_SEND_COUNT=0;                               //轮次4信号发送次数清零
									 LUNCI4_Send_Timer=(u16)(Timer_1ms-LUNCI4_SEND_TIME);    //轮次4信号发送定时器初始化
									if(HC165_XDATA&YK_SIG7_BIT)                         //轮次4常开动作
										LUNCI4_DATA[0]='1';                                //轮次4常开动作标志置位
									if(HC165_XDATA&YK_SIG8_BIT)                         //轮次4常闭动作
										LUNCI4_DATA[1]='1';                                //轮次4常闭动作标志置位
								}
      }
			 }
/*
    t=0;
    if((u16)(Timer_1ms-XLJDQ_ERR_Timer)>GZ_SEND_TIME)//定时检测是否到
     t=0xFF;	
    if((HC165_XDATA&GZ_STS_BIT)||t)            //BIT4 续流继电器故障状态是否改变
     {	 
      NEW_JBJ_PLS=1;                           //续流继电器接入状态标志 定时或改变发送
      XLJDQ_ERR_Timer=(u16)Timer_1ms;          //1s后回送故障恢复数据
      if(HC165_DATA.GZ_STS)                    //是否发生故障             
       {
       	GZ_DATA='0';                           //继电器动作结束
        if(!(HC165_XDATA&GZ_STS_BIT))          //是否为脉冲状态改变引起
         GZS_FLAG=1;                           //不为改变引起(定时引起)
       }                                       
      else                                     //故障位
       {
       	GZ_DATA='1';                           //继电器动作
       }
     }      
    t=0;                                   
    if((u16)(Timer_1ms-NZTZ_Send_Timer)>NZTZ_SEND_TIME)//定时检测是否到内置跳闸
     t=0xFF;
    if((HC165_XDATA&TZ_STS_BIT)||t)            //BIT4 续流继电器状态
     {
      NEW_NZHZ_PLS=1;                          //判断是否有内置合闸脉冲信号
      NZTZ_Send_Timer=(u16)Timer_1ms;          //重置定时
      NZTZ_FLAG=0;	 	
      if(HC165_DATA.TZ_STS==0)                 //判断是否跳闸
       {	
        TZZS_LED_ON;                           //点亮跳闸指示灯
        NZTZ_FLAG=1;                           //内置跳闸标志
        NZHZ_DATA='1';                         //收到内置合闸信号
       }
      else
       {	
        NZTZ_FLAG=0;                           //内置跳闸标志
        if(!WZTZ_FLAG)                         //判断外置跳闸标志
         TZZS_LED_OFF;                         //关闭跳闸指示灯
        NZHZ_DATA='0';                         //收到内置合闸信号
       }
     } 
*/                                                   
    HC165_XDATA=0;                             //清除数据改变标志
}
/*****************************************************************************
* 定时处理CD4094数据
*****************************************************************************/
/*
void Proc_CD4094(void)
{
    if((u8)(Timer_1ms-CD4094_Timer)<CD4094_TIME) //判断CD4094数据输出定时是否到；延时20ms输出；
     return;
    CD4094_Timer=(u8)Timer_1ms;                  //重启定时
    if(CD4094_FLAG)                              //CD4094是否需要更新数据
     {
      CD4094_FLAG=0;                             //清除标志
      WRITE_CD4094_DATA();                       //更新数据
     }	 
}
*/
/********************************************************
* 处理多功能脉冲信号
********************************************************/
void Proc_MFuction_PLS(void)
{
	   u8 m;
    if(GZ_FLAG)                                  //续流继电器报警标志
     {
      if((u16)(Timer_1ms-OPEN_IN_Timer)>
							  GZ_SEND_TIME)                           //续流继电器报警信号发送时间3s
       {	                      
         OPEN_IN_Timer=(u16)Timer_1ms;           //重启定时器
         GZ_DATA='1';                            //是否为故障状态
      	
          if(SOLID_CFG.LED_NUM==LED_6)           //是否为6位数码管
           {
            Disp_Buf[0]=DISP_E;                  //显示'E'
            Disp_Buf[1]=DISP_R;                  //显示'R'
            Disp_Buf[2]=DISP_R;                  //显示'R'
           if(Mtr_Numb_Str[0]==0)
            Disp_Buf[3]=DISP_BLANK;              //显示空白
           else 	     	
            Disp_Buf[3]=Mtr_Numb_Str[0];         //高位
            Disp_Buf[4]=Mtr_Numb_Str[1];         //中位
            Disp_Buf[5]=Mtr_Numb_Str[2];         //低位
           }	
          else
           {	
            Disp_Buf[0]=DISP_E;                  //显示'E'
            Disp_Buf[1]=DISP_R;                  //显示'R'
            Disp_Buf[2]=DISP_R;                  //显示'R'
            Disp_Buf[3]=DISP_BLANK;              //显示空白
            Disp_Buf[4]=DISP_BLANK;              //显示空白
            Disp_Buf[5]=Mtr_Numb_Str[0];         //高位
            Disp_Buf[6]=Mtr_Numb_Str[1];         //中位
            Disp_Buf[7]=Mtr_Numb_Str[2];         //低位
           }  
          Send_Data(ERR_OCMD_JDQGZ,              //续流继电器报警信号
                    1,                           
                    &GZ_DATA);                   //回送故障状态 
											return;
								}											        	                                                                     
     }                                         
    if(NEW_MBJ_PLS)                            //表报警脉冲标志
     {                                         
      NEW_MBJ_PLS=0;                           //清除表报警脉冲标志
      if(MBJEN)
       {	
        Send_Data(ERR_OCMD_MBJ,                //表报警脉冲信号
                  1,                             
                  &MBJ_DATA);                  //
        return;	
       } 
     }		 
    if(NEW_WZHZ_PLS)                           //判断是否有合闸脉冲信号
     {
      if((u16)(Timer_1ms-WZTZ_Send_Timer)>100) //延时100ms 发数据 防止常开常闭连续发送
       {	
        NEW_WZHZ_PLS=0;                        //清除合闸标志
        if(DXTZ&0x80)                          //判断是否为单相费控表
         WZHZ_DATA[1]='0';                     //屏蔽常闭跳闸	
        if(TZEN)                               //是否允许检测
         {	
          Send_Data(ERR_OCMD_WZTZ,             //回送收到合闸脉冲信号
                    2,                         
                    WZHZ_DATA);                //常开 常闭数据   
         }            
        if(Disp_Choose==DISP_HZMC)             //当前状态为显示合闸状态
         {
          if(GZ_FLAG||NZTZ_FLAG)               //是否处于故障和跳闸状态
           return;                             //退出 不再显示其他内容	 
          if(SOLID_CFG.LED_NUM==LED_6)         //是否为6位数码管
           {
            Disp_Buf[0]=DISP_H;                //显示'H'
            Disp_Buf[1]=DISP_2;                //显示'Z'
            Disp_Buf[2]=DISP_BLANK;            //显示空白
            if(WZHZ_DATA[0]=='1')              //常开触点状态
             Disp_Buf[3]=DISP_H;               //H
            else
             Disp_Buf[3]=DISP_L;               //L
            Disp_Buf[4]=DISP_BLANK;
            if(WZHZ_DATA[1]=='1')              //常闭触点状态
             Disp_Buf[5]=DISP_H;               //H
            else
             Disp_Buf[5]=DISP_L;               //L
           }	
          else
           {	
            Disp_Buf[0]=DISP_H;                //显示'H'
            Disp_Buf[1]=DISP_2;                //显示'Z'
            Disp_Buf[2]=DISP_BLANK;            //显示空白
            Disp_Buf[3]=DISP_BLANK;            //显示空白
            if(WZHZ_DATA[0]=='1')              //常开触点状态
             Disp_Buf[4]=DISP_H;               //H
            else
             Disp_Buf[4]=DISP_L;               //L
            Disp_Buf[5]=DISP_BLANK;
            Disp_Buf[6]=DISP_BLANK;
            if(WZHZ_DATA[1]=='1')              //常闭触点状态
             Disp_Buf[7]=DISP_H;               //H
            else
             Disp_Buf[7]=DISP_L;               //L
           }  
         }
        return;
       } 	
     }
					
    if((u16)(Timer_1ms-NZTZ_Send_Timer)>
					   NZTZ_SEND_TIME)                        //是否到发送续流继电器状态时间5s				
     {
      NZTZ_Send_Timer=(u16)Timer_1ms;          //重置定时		
      NZHZ_DATA='0';                          //						
      if(NZTZ_FLAG)                            //内置跳闸信号产生
      {
        NZHZ_DATA='1';                  							//内置跳闸信号数据
								if((WORK_MODE!=NYSY_M))                //不在耐压状态
								{
         if(!GZ_FLAG)                          //不在故障状态
         {
          if(SOLID_CFG.LED_NUM==LED_6)         //是否为6位数码管
           {	
            Disp_Buf[0]=DISP_0;                //显示'0'
            Disp_Buf[1]=DISP_P;                //显示'P'
            Disp_Buf[2]=DISP_N;                //显示'N'
            if(Mtr_Numb_Str[0]==0)
             Disp_Buf[3]=DISP_BLANK;           //显示空白
            else 	     	
             Disp_Buf[3]=Mtr_Numb_Str[0];      //高位
            Disp_Buf[4]=Mtr_Numb_Str[1];       //中位
            Disp_Buf[5]=Mtr_Numb_Str[2];       //低位
           } 
          else
           {	
            Disp_Buf[0]=DISP_0;                //显示'0'
            Disp_Buf[1]=DISP_P;                //显示'P'
            Disp_Buf[2]=DISP_E;                //显示'E'
            Disp_Buf[3]=DISP_N;                //显示'N'
            Disp_Buf[4]=DISP_BLANK;            //显示空白
            Disp_Buf[5]=Mtr_Numb_Str[0];       //高位
            Disp_Buf[6]=Mtr_Numb_Str[1];       //中位
            Disp_Buf[7]=Mtr_Numb_Str[2];       //低位
           } 
         }
								}
								else                                   //正在耐压状态,复位跳闸信号
									RST_IJDQ();
      }

						if(TZEN)                                 //是否允许检测
       {	                                     
        Send_Data(ERR_OCMD_NZTZ,               //回送收到合闸脉冲信号
                  1,                         
                  &NZHZ_DATA);                 //   
       } 
     }
    if(NEW_TQ_PLS)                             //投切脉冲
     {                                         
      NEW_TQ_PLS=0;                            //清除投切脉冲标志
      if(Timer_1ms<POWER_UP_TIME)              //没有上电稳定
       return;	
      Send_Data(ERR_OCMD_TQMC,                 //回送收到投切脉冲信号
                0,                             
                CANT_STR);                     //
      if(Disp_Choose==DISP_TQMC)               //当前状态为显示投切状态
       {                                       
        if(GZ_FLAG||NZTZ_FLAG)                 //是否处于故障和跳闸状态
         return;                               //退出 不再显示其他内容	 
        if(SOLID_CFG.LED_NUM==LED_6)           //是否为6位数码管
         {		
          Disp_Buf[0]=DISP_5;                    //显示'S'
          Disp_Buf[1]=DISP_BLANK;                //显示空白
          Disp_Buf[2]=DISP_R;                    //R
          Disp_Buf[3]=DISP_BLANK;
          Disp_Buf[4]=DISP_5;
          Disp_Buf[5]=DISP_d;
         } 
        else
         {		
          Disp_Buf[0]=DISP_5;                    //显示'S'
          Disp_Buf[1]=DISP_BLANK;                //显示空白
          Disp_Buf[2]=DISP_BLANK;                //显示空白
          Disp_Buf[3]=DISP_R;                    //R
          Disp_Buf[4]=DISP_BLANK;                //显示空白
          Disp_Buf[5]=DISP_BLANK;
          Disp_Buf[6]=DISP_5;
          Disp_Buf[7]=DISP_d;
         } 
       }	
     }	
    if(ENT_XY_PLS)                             //小显示退出试验结论显示
     {
      ENT_XY_PLS=0;
      if(GZ_FLAG||NZTZ_FLAG)                   //是否处于故障和跳闸状态
       return;
      m=Disp_Choose;                           //暂存显示模式
      Disp_Choose=DISP_TEST_DATA;              //更新显示模式
      memset(Disp_Buf,DISP_BLANK,8);           //显示同一设置 清显示
      Update_N_Buf();                          //更新圈数区 
      Update_Mtr_Num();                        //更新表位号区
      Disp_Timer=(Timer_8ms-DISP_TIME);        //更新显示
      Disp_Choose=m;                           //更新显示方式 
     }
    if(DIS_STD_PLS)                           //小显示显示校验合格
     {
      DIS_STD_PLS=0;
      if(GZ_FLAG||NZTZ_FLAG)                  //是否处于故障和跳闸状态
       return;
      if(SOLID_CFG.LED_NUM==LED_6)            //是否为6位数码管
       {	
        Disp_Buf[0]=DISP_h;                   //显示'h'
        Disp_Buf[1]=DISP_G;                   //显示'g'
        Disp_Buf[2]=DISP_BLANK;               //显示空白
        if(Mtr_Numb_Str[0]==0)
         Disp_Buf[3]=DISP_BLANK;              //显示空白
        else 	     	                         
         Disp_Buf[3]=Mtr_Numb_Str[0];         //高位
        Disp_Buf[4]=Mtr_Numb_Str[1];          //中位
        Disp_Buf[5]=Mtr_Numb_Str[2];          //低位  
       }
      else
       {	
        Disp_Buf[0]=DISP_h;                   //显示'h'
        Disp_Buf[1]=DISP_G;                   //显示'g'
        Disp_Buf[2]=DISP_BLANK;               //显示空白
        Disp_Buf[3]=DISP_BLANK;               //显示空白
        Disp_Buf[4]=DISP_BLANK;               //显示空白
        Disp_Buf[5]=Mtr_Numb_Str[0];          //高位
        Disp_Buf[6]=Mtr_Numb_Str[1];          //中位
        Disp_Buf[7]=Mtr_Numb_Str[2];          //低位  
       }
     }                                        
    if(DIS_UNSTD_PLS)                         //小显示显示校验不合格
     {                                        
      DIS_UNSTD_PLS=0;                        
      if(GZ_FLAG||NZTZ_FLAG)                  //是否处于故障和跳闸状态
       return;                                
      if(SOLID_CFG.LED_NUM==LED_6)            //是否为6位数码管
       {	
        Disp_Buf[0]=DISP_b;                   //显示'b'
        Disp_Buf[1]=DISP_h;                   //显示'h'
        Disp_Buf[2]=DISP_G;                   //显示'g'
        if(Mtr_Numb_Str[0]==0)
         Disp_Buf[3]=DISP_BLANK;              //显示空白
        else 	     	                          
         Disp_Buf[3]=Mtr_Numb_Str[0];         //高位
        Disp_Buf[4]=Mtr_Numb_Str[1];          //中位
        Disp_Buf[5]=Mtr_Numb_Str[2];          //低位
       }      
      else
       {	
        Disp_Buf[0]=DISP_b;                   //显示'b'
        Disp_Buf[1]=DISP_h;                   //显示'h'
        Disp_Buf[2]=DISP_G;                   //显示'g'
        Disp_Buf[3]=DISP_BLANK;               //显示空白
        Disp_Buf[4]=DISP_BLANK;               //显示空白
        Disp_Buf[5]=Mtr_Numb_Str[0];          //高位
        Disp_Buf[6]=Mtr_Numb_Str[1];          //中位
        Disp_Buf[7]=Mtr_Numb_Str[2];          //低位
       } 
     } 
}
/********************************************************
* 处理自动流水线脉冲信号检测
* 电子脉冲 光电脉冲 时钟脉冲
********************************************************/
void Proc_AUTO_PLS(void)
{
#ifdef PULSE
    u8 m,n;
    if(DZ_NEW_PLS)                                          //判断是否收到电子脉冲
     {
      if((u8)(Timer_1ms-DZ_PLS_Timer)>=DZ_PLS_TIME)         //消抖延时是否到
       {
        DZ_NEW_PLS=0;                                       //清除电子脉冲标志位
        DZ_PLS_Timer=(u8)Timer_1ms;                         
        n=0;                                                //
        for(m=0;m<10;m++)                                   //连续读10次
         {                                                  
          if(!DZMC_IN)                                      //判断电子脉冲是否仍为低电平
           n++;                                             
         }                                                  
        if(n>=5)                                            //超过50%  	
         {		                                                
          Send_Data(ERR_OCMD_DZPLS,                         //回送收到电子脉冲信号
                    0,                                           
                    CANT_STR);                              // 
         }                                                  
       }                                                    
     }                                                      
    if(GD_NEW_PLS)                                          //是否收到光电脉冲
     {
      if((u8)(Timer_1ms-GDT_PLS_Timer)>=GDT_PLS_TIME)       //消抖延时是否到
       {
        GD_NEW_PLS=0;                                       //清除光电脉冲标志位	
        GDT_PLS_Timer=(u8)Timer_1ms;                        
        n=0;                                                //
        for(m=0;m<10;m++)                                   //连续读10次
         {                                                  
          if(!GDTMC_IN)                                     //判断光电脉冲是否仍为低电平
           n++;                                             
         }                                                  
        if(n>=5)                                            //超过50% 	
         {		                                                
          Send_Data(ERR_OCMD_GDPLS,                         //回送收到光电头脉冲信号
                    0,                                           
                    CANT_STR);                              //
         }                                                  
       }                                                    
     }	                                                     
    if(SZ_NEW_PLS)                                          //是否收到时钟脉冲
     {
      if((u8)(Timer_1ms-SZ_PLS_Timer)>=SZ_PLS_TIME)         //消抖延时是否到
       {
        SZ_NEW_PLS=0;                                       //清除时钟脉冲标志位	
        SZ_PLS_Timer=(u8)Timer_1ms;                         
        n=0;                                                //
        for(m=0;m<10;m++)                                   //连续读10次
         {                                                  
          if(!SZMC_IN)                                      //判断时钟脉冲是否仍为低电平
           n++;                                             
         }                                                  
        if(n>=5)                                            //超过50% 	
         {		                                                
          Send_Data(ERR_OCMD_SZPLS,                         //回送收到电子脉冲信号
                    0,                                           
                    CANT_STR);                              //
         }                                                  
       }                                                    
     }	                                                     
#else                                                       
					u8 m,n;                                                
    if(NZTZ_NEW_PLS)                                        //是否收到开路信号
     {                                                      
      if((u8)(Timer_1ms-NZTZ_PLS_Timer)>=NZTZ_PLS_TIME)     //消抖延时是否到
       {
        NZTZ_NEW_PLS=0;                                     //清除收到开路信号标志位	
        NZTZ_PLS_Timer=(u8)Timer_1ms;                       
        n=0;                                                //
        for(m=0;m<10;m++)                                   //连续读10次
         {                                                  
//          if(!OPEN_IN_STS)                                  //判断脉冲是否仍为低电平
           n++;                                             
         }                                                  
        if(n>=5)                                            //超过50%,确定开路信号产生	
        {                                                   
          NZTZ_FLAG=1;                                      //开路信号标志
									 GZ_FLAG=0;                                        //
									 GZ_CHECK_FLAG=1;                                  //续流继电器故障检测开始标志  
          OPEN_IN_Timer=(u16)Timer_1ms;                       //重启定时器									
        }
         else                                               //没有超过50%,开路信号产系干扰
        {                                                   
          NZTZ_FLAG=0;                                      //开路信号标志
									 GZ_FLAG=0;                                        //
									 GZ_CHECK_FLAG=0;                                  //续流继电器故障检测开始标志              
									 OPEN_IN_Timer=(u16)Timer_1ms;                       //重启定时器
        }								
       }                                                    
     }                                                      
    if(GZ_CHECK_FLAG)                                       //续流继电器故障检测开始标志 
     {
      if((u16)(Timer_1ms-OPEN_IN_Timer)>=GZ_STB_TIME)       //开路信号产生后持续时间计时
       {	
        GZ_CHECK_FLAG=0;                      
        OPEN_IN_Timer=(u16)Timer_1ms;                       //重启定时器
        n=0;                                                //
        for(m=0;m<10;m++)                                   //连续读10次
         {                                                  
//          if(!OPEN_IN_STS)                                  //判断光电脉冲是否仍为低电平
           n++;                                             
         }                                                  
        if(n>=5)                                            //超过50%,确定开路信号产生	
        {                                                   
									 GZ_FLAG=1;                                        //          
        }         
       }           
     }    					
#endif    	 	
}
/********************************************************
* 检测PLL时钟 200ms 检测一次寄存器
* 实时检测 标准晶振定时(131ms一次)
* 默认不使用 RCC2
* 有标准晶振时 STD_CLK_Cnt 相当于 131ms定时器
* 没有标准晶振 不影响检测(无法通过标准晶振检测PLL错误)
* 但是标准晶振频率超过1000k时可能有问题
* 标准晶振频率改变时 要调整比较值 即if(t<76)中的76
* 经测试 正常计数值为131 PLL异常失败 计数值约为42
* 即 CPU速度变慢(8mHz) Timer_1ms 计数速度为PLL正常时的 8/25=0.32(32%)
* 标准晶振中断时间不变 仍为131ms Timer_1ms 计数值减少 时间时间为3.125ms
* 检测失锁标志
********************************************************/
void Check_PLL(void)
{
    if(REF_JZ_INT)                                //标准时钟中断标志
     {
      u16 t;
      REF_JZ_INT=0;                               //清除标准时钟中断
      t=((u16)(NEW_PLL_Ref_Timer-OLD_PLL_Ref_Timer)); 
      OLD_PLL_Ref_Timer=NEW_PLL_Ref_Timer;        //刷新定时器
      if(t<76)                                    //75/131=0.572 PLL工作于25M 工作频率小于14M认为失败
       {
        PLL_ERR_Cnt++;                            //	
        if(PLL_ERR_Cnt>4)                         //连续5次错误 约0.6s
         {
          PLL_ERR_Cnt=0;                          //	
          Init_Pll();                             //初始化锁相环	 
          PLL_CHK_Timer=Timer_8ms;                //定时到重启定时
          return;	
         }
       }  		
      else
       PLL_ERR_Cnt=0;                             //	
    }
    if(((u8)(Timer_8ms-PLL_CHK_Timer))>=PLL_CHK_TIME)//判断显示定时是否到,200ms检测一次
     {
      u32  ulRCC,ulRCC2;
      PLL_CHK_Timer=Timer_8ms;                    //定时到重启定时
      if(!(HWREG(SYSCTL_RIS) & SYSCTL_INT_PLL_LOCK))    //PLL失锁标志
       {
        PLL_ERR_Cnt=0;                            //	
        Init_Pll();                               //初始化锁相环	 
        return;	
       }
      ulRCC = HWREG(SYSCTL_RCC);                  //读取RCC  运行模式时钟配置寄存器
      ulRCC2 = HWREG(SYSCTL_RCC2);                //读取RCC2 运行模式时钟配置寄存器2
      ulRCC&=((SYSCTL_RCC_ACG|                    //时钟门控域 
               SYSCTL_RCC_SYSDIV_M|               //系统分频域
               SYSCTL_RCC_USESYSDIV|              //使用分频域
               SYSCTL_RCC_USEPWMDIV|              //新增 有别于LM3S2139
               SYSCTL_RCC_PWMDIV_M|               //新增 有别于LM3S2139
               SYSCTL_RCC_PWRDN|                  //掉电域
               SYSCTL_RCC_BYPASS|                 //PLL旁路域
               SYSCTL_RCC_XTAL_M|                 //晶振域
               SYSCTL_RCC_OSCSRC_M|               //振荡器选择域
               SYSCTL_RCC_MOSCDIS));              //主振荡器禁能域
      if((ulRCC2&SYSCTL_RCC2_USERCC2)||           //不使用RCC2
         (ulRCC!=SysRCC_CFG))                     
       {   
        PLL_ERR_Cnt=0;                            //清除错误计数
        Init_Pll();                               //初始化锁相环	 
       }  
     }  
}
/********************************************************
* 外部看门狗喂狗
* 2013.4.8 原喂狗程序放在Proc_CD4094(); 中 每40ms喂狗1次
* 开机看门狗喂狗定时60ms 喂5次后变为 800ms 
********************************************************/
void  Ext_WDTFeed(void)         //
{
    u8 m;
    m=(u8)(Timer_8ms-EXT_WD_Timer);
    if(EXT_WDFEED_CNT<5)
     {
      if(m<(32/8))
     	 return;
     	EXT_WDFEED_CNT++; 
     }	
    else
     {
     	if(m<(800/8))
     	 return;	
     }	 
    EXT_WD_Timer=Timer_8ms;
    GPIOPinWrite(WDI_GPIO,WDI,~GPIOPinRead(WDI_GPIO,WDI)); //复位外部硬件看门狗
}

/********************************************************
* 专变终端脉冲输出控制(针对专变终端)
* 
********************************************************/
void PULSE_CTL_Pro(void)
{
		if((MTR_MOD!=Acquire_Terminal_13)&&
     (MTR_MOD!=Acquire_Terminal_09))                       	//当前不是在校验专变终端Ⅲ，退出；
		  return; 
		if(Pulse1_RUN_CTL)                                        //遥信信号1脉冲输出开始
		{                                                         
    if(PulOut1_RVS_Bit)                                     //遥信信号1脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut1_Timer)>=                   
							  (YaoXin1_Set.CYCLE-YaoXin1_Set.WIDTH))             //非脉冲时间到
						{                                                     
							 MC_OUT1_Reverse;                                    //遥信信号1脉冲反转
							 YX_CTL1_Reverse;
							 PulOut1_RVS_Bit=0;                                  //遥信信号1脉冲反转标志更换
							 PulOut1_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
      }                                                     
    }                                                       
				else                                                    //                                                    
				{                                                       
      if((u16)(Timer_1ms-PulOut1_Timer)>=                   
							  YaoXin1_Set.WIDTH)                                 //脉冲时间到
						{                                                     
						  MC_OUT1_Reverse;                                    //遥信信号1脉冲电平反转
							 YX_CTL1_Reverse;
        PulOut1_RVS_Bit=1;                                  //遥信信号1脉冲反转标志更换
        PulOut1_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
							 PulOut1_Count++;                                    //遥信1脉冲输出计数累加
							 if(PulOut1_Count>=PulOut1_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut1_Count=0;                                  //遥信1脉冲输出计数清零
									 Pulse1_RUN_CTL=0;                                 //遥信1脉冲输出控制位清零
        }
      }                                                     
    }                                                       
  }                                                         
		if(Pulse2_RUN_CTL)                                        //遥信信号2脉冲输出开始
		{                                                         
    if(PulOut2_RVS_Bit)                                     //遥信信号2脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut2_Timer)>=                   
							  (YaoXin2_Set.CYCLE-YaoXin2_Set.WIDTH))             //非脉冲时间到
						{                                                     
        MC_OUT2_Reverse;                                    //遥信信号2脉冲反转         
							 YX_CTL2_Reverse;
							 PulOut2_RVS_Bit=0;                                  //遥信信号2脉冲反转标志变换
							 PulOut2_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
						}                                                     
    }                                                       
				else                                                    
				{                                                       
					 if((u16)(Timer_1ms-PulOut2_Timer)>=                   
							  YaoXin2_Set.WIDTH)                                 //脉冲时间到
						{                                                     
        MC_OUT2_Reverse;                                    //遥信信号2脉冲反转
							 YX_CTL2_Reverse;
        PulOut2_RVS_Bit=1;                                  //遥信信号2脉冲反转标志变换
							 PulOut2_Timer=(u16)Timer_1ms;							                //脉冲输出定时器初始化
							 PulOut2_Count++;                                    //遥信2脉冲输出计数累加
							 if(PulOut2_Count>=PulOut2_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut2_Count=0;                                  //遥信2脉冲输出计数清零
									 Pulse2_RUN_CTL=0;                                 //遥信2脉冲输出控制位清零
        }							
						}
				}
  }
  if(Pulse3_RUN_CTL)                                        //遥信信号3脉冲输出开始
		{                                                         
    if(PulOut3_RVS_Bit)                                     //遥信信号3脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut3_Timer)>=                   
							  (YaoXin3_Set.CYCLE-YaoXin3_Set.WIDTH))             //非脉冲时间到
						{                                                     
							 MC_OUT3_Reverse;                                    //遥信信号3脉冲反转
							 YX_CTL3_Reverse;
							 PulOut3_RVS_Bit=0;                                  //遥信信号3脉冲反转标志变换
							 PulOut3_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
						}                                                     
    }                                                       
				else                                                    
				{                                                       
					 if((u16)(Timer_1ms-PulOut3_Timer)>=                   
							  YaoXin3_Set.WIDTH)                                 //脉冲时间到
						{                                                     
							 MC_OUT3_Reverse;                                    //遥信信号3脉冲反转
							 YX_CTL3_Reverse;
							 PulOut3_RVS_Bit=1;                                  //遥信信号3脉冲反转标志变换
							 PulOut3_Timer=(u16)Timer_1ms;							                //脉冲输出定时器初始化
							 PulOut3_Count++;                                    //遥信3脉冲输出计数累加
							 if(PulOut3_Count>=PulOut3_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut3_Count=0;                                  //遥信3脉冲输出计数清零
									 Pulse3_RUN_CTL=0;                                 //遥信3脉冲输出控制位清零
        }									
						}
				}
  }		
  if(Pulse4_RUN_CTL)                                        //遥信信号4脉冲输出开始
		{                                                         
    if(PulOut4_RVS_Bit)                                     //遥信信号4脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut4_Timer)>=                   
							  (YaoXin4_Set.CYCLE-YaoXin4_Set.WIDTH))             //非脉冲时间到
						{                                                     
							 MC_OUT4_Reverse;                                    //遥信信号4脉冲反转
							 YX_CTL4_Reverse;
							 PulOut4_RVS_Bit=0;                                  //遥信信号4脉冲反转标志变换
							 PulOut4_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
						}                                                     
    }                                                       
				else                                                    
				{                                                       
					 if((u16)(Timer_1ms-PulOut4_Timer)>=                   
							  YaoXin4_Set.WIDTH)                                 //脉冲时间到
						{                                                     
							 MC_OUT4_Reverse;                                    //遥信信号4脉冲反转
							 YX_CTL4_Reverse;
							 PulOut4_RVS_Bit=1;                                  //遥信信号4脉冲反转标志变换
							 PulOut4_Timer=(u16)Timer_1ms;							                //脉冲输出定时器初始化
							 PulOut4_Count++;                                    //遥信4脉冲输出计数累加
							 if(PulOut4_Count>=PulOut4_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut4_Count=0;                                  //遥信4脉冲输出计数清零
									 Pulse4_RUN_CTL=0;                                 //遥信4脉冲输出控制位清零
        }								
						}
				}
  }		
		if(Pulse5_RUN_CTL)                                        //遥信信号5脉冲输出开始
		{                                                         
    if(PulOut5_RVS_Bit)                                     //遥信信号5脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut5_Timer)>=                   
							  (YaoXin5_Set.CYCLE-YaoXin5_Set.WIDTH))             //非脉冲时间到
						{                                                     
							 MC_OUT5_Reverse;                                    //遥信信号5脉冲反转
							 PulOut5_RVS_Bit=0;                                  //遥信信号5脉冲反转标志更换
							 PulOut5_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化							 
      }                                                     
    }                                                       
				else                                                    
				{                                                       
      if((u16)(Timer_1ms-PulOut5_Timer)>=                   
							  YaoXin5_Set.WIDTH)                                 //脉冲时间到
						{                                                     
						  MC_OUT5_Reverse;                                    //遥信信号5脉冲电平反转
        PulOut5_RVS_Bit=1;                                  //遥信信号5脉冲反转标志更换
        PulOut5_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
							 PulOut5_Count++;                                    //遥信5脉冲输出计数累加
							 if(PulOut5_Count>=PulOut5_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut5_Count=0;                                  //遥信5脉冲输出计数清零
									 Pulse5_RUN_CTL=0;                                 //遥信5脉冲输出控制位清零
        }								
      }                                                     
    }                                                       
  } 
		if(Pulse6_RUN_CTL)                                        //遥信信号6脉冲输出开始
		{                                                         
    if(PulOut6_RVS_Bit)                                     //遥信信号6脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut6_Timer)>=                   
							  (YaoXin6_Set.CYCLE-YaoXin6_Set.WIDTH))             //非脉冲时间到
						{                                                     
							 MC_OUT6_Reverse;                                    //遥信信号6脉冲反转
							 PulOut6_RVS_Bit=0;                                  //遥信信号6脉冲反转标志更换
							 PulOut6_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
      }                                                     
    }                                                       
				else                                                    
				{                                                       
      if((u16)(Timer_1ms-PulOut6_Timer)>=                   
							  YaoXin6_Set.WIDTH)                                 //脉冲时间到
						{                                                     
						  MC_OUT6_Reverse;                                    //遥信信号6脉冲电平反转
        PulOut6_RVS_Bit=1;                                  //遥信信号脉冲6反转标志更换
        PulOut6_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
							 PulOut6_Count++;                                    //遥信6脉冲输出计数累加
							 if(PulOut6_Count>=PulOut6_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut6_Count=0;                                  //遥信6脉冲输出计数清零
									 Pulse6_RUN_CTL=0;                                 //遥信6脉冲输出控制位清零
        }								
      }                                                     
    }                                                       
  }
  if(Pulse7_RUN_CTL)                                        //遥信信号7脉冲输出开始
		{                                                         
    if(PulOut7_RVS_Bit)                                     //遥信信号7脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut7_Timer)>=                   
							  (YaoXin7_Set.CYCLE-YaoXin7_Set.WIDTH))             //非脉冲时间到
						{                                                     
							 MC_OUT7_Reverse;                                    //遥信信号7脉冲反转
							 PulOut7_RVS_Bit=0;                                  //遥信信号7脉冲反转标志更换
							 PulOut7_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
      }                                                     
    }                                                       
				else                                                    
				{                                                       
      if((u16)(Timer_1ms-PulOut7_Timer)>=                   
							  YaoXin7_Set.WIDTH)                                 //脉冲时间到
						{                                                     
						  MC_OUT7_Reverse;                                    //遥信信号7脉冲电平反转
        PulOut7_RVS_Bit=1;                                  //遥信信号脉冲7反转标志更换
        PulOut7_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
							 PulOut7_Count++;                                    //遥信7脉冲输出计数累加
							 if(PulOut7_Count>=PulOut7_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut7_Count=0;                                  //遥信7脉冲输出计数清零
									 Pulse7_RUN_CTL=0;                                 //遥信7脉冲输出控制位清零
        }								
      }                                                     
    }                                                       
  }
		if(Pulse8_RUN_CTL)                                        //遥信信号8脉冲输出开始
		{                                                         
    if(PulOut8_RVS_Bit)                                     //遥信信号8脉冲反转标志
				{                                                       
      if((u16)(Timer_1ms-PulOut8_Timer)>=                   
							  (YaoXin8_Set.CYCLE-YaoXin8_Set.WIDTH))             //非脉冲时间到
						{                                                     
							 MC_OUT8_Reverse;                                    //遥信信号8脉冲反转
							 PulOut8_RVS_Bit=0;                                  //遥信信号8脉冲反转标志更换
							 PulOut8_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
      }                                                     
    }                                                       
				else                                                    
				{                                                       
      if((u16)(Timer_1ms-PulOut8_Timer)>=                   
							  YaoXin8_Set.WIDTH)                                 //脉冲时间到
						{                                                     
						  MC_OUT8_Reverse;                                    //遥信信号8脉冲电平反转
        PulOut8_RVS_Bit=1;                                  //遥信信号脉冲8反转标志更换
        PulOut8_Timer=(u16)Timer_1ms;                       //脉冲输出定时器初始化
							 PulOut8_Count++;                                    //遥信8脉冲输出计数累加
							 if(PulOut8_Count>=PulOut8_Count_Set)                //判断设置的脉冲个数是否已经输出完毕
								{
          PulOut8_Count=0;                                  //遥信8脉冲输出计数清零
									 Pulse8_RUN_CTL=0;                                 //遥信8脉冲输出控制位清零
        }								
      }                                                     
    }                                                       
  }		
}
/********************************************************
* 处理轮次脉冲信号
********************************************************/
void Proc_LUNCI_PLS(void)
{
  if(YKSIG_EN)
		{
    if(NEW_LUNCI1_PLS)		                                    //轮次1控制事件发生标志 		
    {
      if((u8)(Timer_1ms-LUNCI1_Send_Timer)>=LUNCI1_SEND_TIME)//轮次1发送延时到？
						{						
							 if(LUNCI1_SEND_COUNT>=3)                            //轮次1控制信号发送次数是否已经到3次
								{
									 NEW_LUNCI1_PLS=0;                                 //轮次1控制事件发生标志清零
          LUNCI1_DATA[0]='0';                               //轮次1常开信号无改变标志
          LUNCI1_DATA[1]='0';                               //轮次1常闭信号无改变标志
        }
								else                                                //发送次数没到3次，继续发送数据
								{
									 LUNCI1_Send_Timer=(u8)Timer_1ms;                  //初始化定时器
          Send_Data(ERR_OCMD_LUNCI1,                        //发送数据
							             2,
							             &LUNCI1_DATA[0]);
									 LUNCI1_SEND_COUNT++;							                       //发送次数累加
								}
						}		      
				}		
    if(NEW_LUNCI2_PLS)                                      //轮次2控制事件发生标志				
				{
					 if((u8)(Timer_1ms-LUNCI2_Send_Timer)>=LUNCI2_SEND_TIME)//轮次2发送延时时间到？
						{     					
							 if(LUNCI2_SEND_COUNT>=3)                            //轮次2控制信号发送次数已达3次
								{
									 NEW_LUNCI2_PLS=0;                                 //轮次2控制事件发生标志清零
									 LUNCI2_DATA[0]='0';                               //轮次2常开信号无改变标志
									 LUNCI2_DATA[1]='0';                               //轮次2常闭信号无改变标志
								}
								else                                                //发送次数没到3次，继续发送数据
								{
							   LUNCI2_Send_Timer=(u8)Timer_1ms;                  //发送定时初始化
									 Send_Data(ERR_OCMD_LUNCI2,                        //发送数据
							             2,
							             &LUNCI2_DATA[0]);
									 LUNCI2_SEND_COUNT++;                              //发送次数累加
								}
				  }
    }
    if(NEW_LUNCI3_PLS)                                      //轮次3控制事件发生标志				
				{
					 if((u8)(Timer_1ms-LUNCI3_Send_Timer)>=LUNCI3_SEND_TIME)//轮次3发送延时时间到？
						{     					
							 if(LUNCI3_SEND_COUNT>=3)                            //轮次3控制信号发送次数已达3次
								{
									 NEW_LUNCI3_PLS=0;                                 //轮次3控制事件发生标志清零
									 LUNCI3_DATA[0]='0';                               //轮次3常开信号无改变标志
									 LUNCI3_DATA[1]='0';                               //轮次3常闭信号无改变标志
								}
								else                                                //发送次数没到3次，继续发送数据
								{
							   LUNCI3_Send_Timer=(u8)Timer_1ms;                  //发送定时初始化
									 Send_Data(ERR_OCMD_LUNCI3,                        //发送数据
							             2,
							             &LUNCI3_DATA[0]);
									 LUNCI3_SEND_COUNT++;                              //发送次数累加
								}
				  }
    }
    if(NEW_LUNCI4_PLS)                                      //轮次4控制事件发生标志				
				{
					 if((u8)(Timer_1ms-LUNCI4_Send_Timer)>=LUNCI4_SEND_TIME)//轮次4发送延时时间到？
						{     					
							 if(LUNCI4_SEND_COUNT>=3)                            //轮次4控制信号发送次数已达3次
								{
									 NEW_LUNCI4_PLS=0;                                 //轮次4控制事件发生标志清零
									 LUNCI4_DATA[0]='0';                               //轮次4常开信号无改变标志
									 LUNCI4_DATA[1]='0';                               //轮次4常闭信号无改变标志
								}
								else                                                //发送次数没到3次，继续发送数据
								{
							   LUNCI4_Send_Timer=(u8)Timer_1ms;                  //发送定时初始化
									 Send_Data(ERR_OCMD_LUNCI4,                        //发送数据
							             2,
							             &LUNCI4_DATA[0]);
									 LUNCI4_SEND_COUNT++;                              //发送次数累加
								}
				  }
    }				
		}				
}    
