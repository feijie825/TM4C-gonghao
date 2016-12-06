/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : CLK_ERR.h
;* Author             : 张力阵
;* 时钟处理函数库头文件和宏定义
*******************************************************************************/
#define DEFAULT_HC165  0x1F00            //HC165默认状态
//HC165数据屏蔽码
#define HC165_MSK      0xFFFF            //HC165屏蔽码
#define YK_SIG1_BIT   (1<<0x00)          //BIT0  负控遥控信号1
#define YK_SIG2_BIT   (1<<0x01)          //BIT1  负控遥控信号2
#define YK_SIG3_BIT   (1<<0x02)          //BIT2  负控遥控信号3
#define YK_SIG4_BIT   (1<<0x03)          //BIT3  负控遥控信号4
#define YK_SIG5_BIT   (1<<0x04)          //BIT4  负控遥控信号5
#define YK_SIG6_BIT   (1<<0x05)          //BIT5  负控遥控信号6
#define YK_SIG7_BIT   (1<<0x06)          //BIT6  负控遥控信号7
#define YK_SIG8_BIT   (1<<0x07)          //BIT7  负控遥控信号7
#define MTR_BJ_BIT    (1<<0x08)          //BIT8  电能表报警信号
#define JDQ_CLS_BIT   (1<<0x09)          //BIT9  继电器常闭
#define JDQ_OPN_BIT   (1<<0x0a)          //BIT10 继电器常开
#define GZ_STS_BIT    (1<<0x0b)          //BIT11 续流继电器状态 	
#define TZ_STS_BIT    (1<<0x0c)          //BIT12 表跳闸指示
#define HGQ_BJ_BIT    (1<<0x0d)          //BIT13 三相并线台用
#define JCXH_BIT      (1<<0x0e)          //BIT14 耐压漏电流击穿信号 
#define MTR_ON_BIT    (1<<0x0f)          //BIT15 表放好标志
//CD4094数据屏蔽码
#define  CD4094_MSK  0xFFFF        
 
/********************************************************
* 读HC165数据子程序
********************************************************/
u16 READ_HC165_DATA(void);
/********************************************************
* 写CD4094数据子程序
********************************************************/
void WRITE_CD4094_DATA(void);
/********************************************************
* 处理需量脉冲超时
********************************************************/
void Proc_Xul_OvTm(void);       
/********************************************************
* 测量需量周期
********************************************************/
void MEA_XUL_TIME(void);        
/********************************************************
* 处理时钟脉冲中断超时
********************************************************/
void Proc_Clk_OvTm(void);       
/********************************************************
* 测量时钟频率和日计时误差
********************************************************/
void MEA_CLK_FREQ(void);      
/********************************************************
* 定时检测标准时钟脉冲是否存在
********************************************************/
void Check_Std_Clk(void);      
/********************************************************
* 计算设置频率重装值和重装次数
********************************************************/
void Cal_Clk_Reload(u8 Sts);   
/********************************************************
* 计算被检表高频重装值和重装次数
********************************************************/
void Cal_Gp_Relaod(void);
/********************************************************
* 设置多功能表(模拟表),负控终端串口通信参数
* 入口:Com 端口
********************************************************/
void Set_MTR_LCT_Com(u8 Com); 
/********************************************************
* 设置各表位时钟频率    
********************************************************/
void Set_Clk_Freq_Pr(void);           
/********************************************************
* 设置该表位时钟频率测量时间   
********************************************************/
void Set_Clk_Time_Pr(void); 
/********************************************************
* 时钟频率测量控制  
********************************************************/
void Set_Clk_Ctl_Pr(void);            
/********************************************************
* 需量周期测量控制   
********************************************************/
void Set_XuL_Ctl_Pr(void);            
/********************************************************
* 设置需量周期测量个数  
********************************************************/
void Set_XuL_Pls_Num(void);  
/********************************************************
* 多功能脉冲端口管脚初始化处理
********************************************************/
void MFuction_Clk_Cfg(void);
/********************************************************
* 设置显示模式
********************************************************/
void Set_Disp_Mode(void);
/********************************************************
* 时钟频率测量模式处理
********************************************************/
void CLK_FREQ_Pr(void);    
/******************************************************
* 重新装载模拟表数据     上位机   -> 模拟表
* 误差单元命令 提高数据发送速度 命令误差单元重发上次数据->模拟表
******************************************************/
void Set_Reload_MtrD(void);
/******************************************************
* 重新装载负控终端数据   上位机   -> 负控终端
* 误差单元命令 提高数据发送速度 命令误差单元重发上次数据->负控终端
******************************************************/
void Set_Reload_LctD(void);
/******************************************************
* 重新上传模拟表数据   模拟表   -> 上位机 
* 误差单元命令 提高数据发送速度 命令误差单元重新上传上次收到的模拟表数据
******************************************************/
void Set_ReTx_MtrD(void);
/******************************************************
* 重新上传负控终端数据   负控终端   -> 上位机     
* 误差单元命令 提高数据发送速度 命令误差单元重新上传上次收到的负控终端数据
******************************************************/
void Set_ReTx_LctD(void);
/******************************************************
* 接收时间基准     
* 格式 HH:MM:SS   时分秒  8字节
******************************************************/
void Set_Time_Base(void);
/********************************************************
* 处理HZ信号
********************************************************/
void Proc_MFuction_PLS(void);
/*****************************************************************************
* 处理需量周期脉冲
*****************************************************************************/
void Proc_Xul_Pls(u16 m);
/*****************************************************************************
* 处理按键
*****************************************************************************/
void Key_Proc(void);
/*****************************************************************************
* IO口中断重使能处理
*****************************************************************************/
void  IO_INT_REEN(void);        
/*****************************************************************************
* 定时处理HC165数据
*****************************************************************************/
void Proc_HC165(void);
/*****************************************************************************
* 定时处理CD4094数据
*****************************************************************************/
//void Proc_CD4094(void);
/********************************************************
* 处理自动流水线脉冲信号检测
* 电子脉冲 光电脉冲 时钟脉冲
********************************************************/
void Proc_AUTO_PLS(void);
/********************************************************
* 检测PLL时钟
********************************************************/
void Check_PLL(void);
/********************************************************
* 外部看门狗喂狗
* 2013.4.8 原喂狗程序放在Proc_CD4094(); 中 每40ms喂狗1次
* 开机看门狗喂狗定时60ms 喂3次后变为 800ms 
********************************************************/
void  Ext_WDTFeed(void);
/********************************************************
* 专变终端脉冲输出控制
********************************************************/
void PULSE_CTL_Pro(void);
/********************************************************
* 处理轮次脉冲信号
********************************************************/
void Proc_LUNCI_PLS(void);
