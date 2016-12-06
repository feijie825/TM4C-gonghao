/* Copyright (c) 2005-2013 Texas Instruments Incorporated.  All rights reserved.
* File Name          : main.c
* Author             : 张力阵
* 主函数
*******************************************************************************/
#include "Function.h"
#include "ENG_ERR.h"
#include "CLK_ERR.h"
#include "string.h"
#include "Disp.h"
#include "math.h"
int main(void)
{
      SysCtlDelay(500);     //延时80uS
      SysCtlDelay(3125000); //延时100mS
      IntMasterDisable();   //开机禁能中断
      Init_Pll();           //初始化锁相环
      Init_Gpio();          //初始化I/O口
      SysCtlDelay(6250000); //延时100mS,为了等待24V开关电源上电，增加延时
      Init_SysTick();       //初始化系统节拍定时器 并使能中断
      Init_Ssi();           //初始化同步串口 控制数码管显示
      Init_Uart();          //初始化串口
      Init_Timer();         //初始化定时器
                            //    Init_Adc();                        //初始化ADC
      Init_Ram();           //初始化RAM区和变量
      Init_CAN0();          //初始化CAN0接口
      Init_CAN1();          //初始化CAN1接口
      Init_Wdt();           //初始化看门狗并启动
                            //     Reset_HD7279();                    //复位HD7279
      Init_Int();           //初始化NVIC中断使能和优先级配置
      IntMasterEnable();    //CPU中断允许

      for (;;)
      { //主程序循环一圈41us

            WDTFeed();
            Ext_WDTFeed(); //外部看门狗喂狗
#if 0
      Proc_Mst_Check();                //处理主机查询 
      Proc_LDATA_MSG_IBUF();           //处理CAN0长数据帧接收缓冲区处理
      Proc_SDATA_IBUF();               //处理CAN0短数据接收指令缓冲区
      Proc_SDATA_OBUF();               //处理CAN0短数据发送指令缓冲区
      Proc_CAN_OvTm();                 //CAN0 总线超时处理
      Proc_CAN_STS();                  //处理CAN0总线状态
						Proc_CNA1_SDATA_IBUF();          //处理CAN1短数据接收指令缓冲区
						Proc_CAN1_SDATA_OBUF();          //处理CAN1短数据发送指令缓冲区
						Proc_CAN1_OvTm();                //CAN1总线超时处理
						Proc_CAN1_STS();                 //处理CAN1总线状态
      Proc_Com_Rx_OvTm(MTRCOM);        //MTRCOM接收超时处理
      Proc_Com_Rx_OvTm(LCTCOM);        //LCTCOM接收超时处理
      Proc_Com_Rx_OvTm(RTECOM);        //RTECOM接收超时处理
      Proc_Com_Rx_OvTm(ATECOM);        //ATECOM接收超时处理
      Proc_Com_Rx_OvTm(IRECOM);        //IRECOM接收超时处理
      Proc_Com_OBuf(MTRCOM);           //MTRCOM 发送缓冲区处理
      Proc_Com_IBuf(MTRCOM);           //MTRCOM 接收缓冲区处理
      Proc_Com_OBuf(LCTCOM);           //LCTCOM 发送缓冲区处理
      Proc_Com_IBuf(LCTCOM);           //LCTCOM 接收缓冲区处理
						Proc_Com_OBuf(RTECOM);           //RTECOM 发送缓冲区处理
						Proc_Com_IBuf(RTECOM);           //RTECOM 接收缓冲区处理
						Proc_Com_OBuf(ATECOM);           //ATECOM 发送缓冲区处理
						Proc_Com_IBuf(ATECOM);           //ATECOM 接收缓冲区处理
						Proc_Com_OBuf(IRECOM);           //IRECOM 发送缓冲区处理
						Proc_Com_IBuf(IRECOM);           //IRECOM 接收缓冲区处理
      I_JDQ_Time_Pr();                 //电流继电器定时处理
      UJDQ_ESwitch_Time_Pr();          //电压继电器,电子开关定时处理
      GDT_RST_Pr();                    //光电头复位处理
      Work_Mode_Pr();                  //工作模式处理
      Disp_Time_Pr();                  //显示定时处理
      Check_Std_Clk();                 //定时检测标准时钟脉冲是否存在
//      Check_Std_Eclk();              //检测标准电能脉冲是否存在
      MEA_CLK_FREQ();                  //测量时钟频率和日计时误差 
      Proc_Clk_OvTm();                 //处理时钟脉冲超时
      MEA_XUL_TIME();                  //测量需量周期
//      Proc_Xul_OvTm();               //处理需量脉冲超时
      Key_Proc();                      //处理按键
      IO_INT_REEN();                   //IO口中断重使能处理
      Proc_HC165();                    //定时处理HC165数据
//      Proc_CD4094();                   //定时处理CD4094数据
//      Proc_ADC_Timer();                //AD采样定时超时处理
      Proc_MFuction_PLS();             //处理多功能脉冲信号
      Proc_AUTO_PLS();                 //处理自动流水线脉冲信号检测
      Check_PLL();                     //检测PLL时钟
      PULSE_CTL_Pro();                 //专变终端脉冲输出控制
      Proc_LUNCI_PLS();                //处理轮次脉冲信号
#endif
      }
}
