/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
* File Name          : Init.c
* Author             : 张力阵
* 初始化
*******************************************************************************/
#include "Function.h"
#include "string.h"
#include "stdio.h"
#include "CLK_ERR.h"   //时钟处理声明文件
#include "ENG_ERR.h"   //电能处理声明文件
#include "Disp.h"
#include "pin_map.h"
/*****************************************************************************
* 程序版本号
*****************************************************************************/
const u8 VER_TAB[]={"M4V1.0.0\r"};
/*****************************************************************************
* 校验设置对应列表
*****************************************************************************/
const u8 Parity_Set_TAB[5]=
{
    0x10,           //无校验位 'N' FIFO 使能
    0x12,           //奇校验   'O' FIFO 使能
    0x16,           //偶校验   'E' FIFO 使能
    0x92,           //Mark校验 'M' FIFO 使能 校验位固定为1
    0x96            //Space校验'S' FIFO 使能 校验位固定为0
};
/*****************************************************************************
* 串口默认参数列表 MTRCOM LCTCOM
*****************************************************************************/
const UART_SET UART_PARA_TAB[UART_NUMB]=
{           
    {
     UART0_BAUD,  //波特率
     UART_8_BIT,  //数据位数
     UART_1_STOP, //停止位数
					UART_E_PARITY//校验位
    },
    {
     UART1_BAUD,  //波特率
     UART_8_BIT,  //数据位数
     UART_1_STOP, //停止位数
					UART_E_PARITY//校验位
    },
    {
     UART3_BAUD,  //波特率
     UART_8_BIT,  //数据位数
     UART_1_STOP, //停止位数
					UART_E_PARITY//校验位
    },				
				{
     UART5_BAUD,  //波特率
     UART_8_BIT,  //数据位数
     UART_1_STOP, //停止位数
					UART_E_PARITY//校验位
    },				
				{
     UART6_BAUD,  //波特率
     UART_8_BIT,  //数据位数
     UART_1_STOP, //停止位数
					UART_E_PARITY//校验位
    },				
};
/*****************************************************************************
* CAN0 报文对象处理设置表格 MSG RAM 
* 接收报文对象需要设置滤波 
* 发送报文对象不需设置滤波
* 各域位定义见 vari.h 中报文结构体定义 
* 增加报文对象时 依次添加
*****************************************************************************/
const CAN_MSG_SET  CAN_MSG_SET_TAB[]=
{    //控制域     仲裁域     过滤屏蔽
    {0x000103A0,0x00000000,0x1FFFFF00},  //标准帧 主机广播查询帧       MST_CHK_BCAST   MSG OBJ ID1  接收
    {0x000203A0,0x00000000,0x1FFFFF00},  //标准帧 主机单播查询帧       MST_CHK_SCAST   MSG OBJ ID2  接收
    {0x00030010,0x00010000,0x1FFFFFFF},  //标准帧 从机回应查询         SLV_CHK_ECHO    MSG OBJ ID3  发送
    {0x000403E0,0x10000000,0x1FFFFF0F},  //扩展帧 主机批准帧           MST_LDATA_ACK   MSG OBJ ID4  接收
    {0x000503E0,0x10020000,0x1FFFFF00},  //扩展帧 主机发送长广播数据帧 MST_LCDATA_TX   MSG OBJ ID5  接收
    {0x000603E0,0x10020000,0x1FFFFF00},  //扩展帧 主机发送长单播数据帧 MST_LSDATA_TX   MSG OBJ ID6  接收
    {0x000703E0,0x10040000,0x1FFFFF0F},  //扩展帧 主机请求从机重发     MST_LDATA_REQRT MSG OBJ ID7  接收
    {0x000803E0,0x10080008,0x1001FF08},  //扩展帧 主机发送短广播数据帧 MST_SCDATA_TX   MSG OBJ ID8  接收
    {0x000903E0,0x10080008,0x1001FF08},  //扩展帧 主机发送短单播数据帧 MST_SSDATA_TX   MSG OBJ ID9  接收
    {0x000A0040,0x10010000,0x1FFFFFFF},  //扩展帧 从机发送请求帧       SLV_LDATA_REQTX MSG OBJ ID10 发送
    {0x000B0050,0x10030000,0x1FFFFFFF},  //扩展帧 从机发送长数据帧     SLV_LDATA_TX    MSG OBJ ID11 发送
    {0x000C0040,0x10050000,0x1FFFFFFF},  //扩展帧 从机请求主机重发     SLV_LDATA_REQRT MSG OBJ ID12 发送
    {0x000D0050,0x10090000,0x1FFFFFFF},  //扩展帧 从机发送短数据帧     SLV_SDATA_TX    MSG OBJ ID13 发送
};
/*****************************************************************************
* CAN1 报文对象处理设置表格 MSG RAM 
* 接收报文对象需要设置滤波 
* 发送报文对象不需设置滤波
* 各域位定义见 vari.h 中报文结构体定义 
* 增加报文对象时 依次添加
*****************************************************************************/
const CAN1_MSG_SET  CAN1_MSG_SET_TAB[]=
{
    {0x000103E0,0x10000002,0x1800FFFF},  //扩展帧 主机发送短广播数据帧 MST_SCDATA_TX   MSG OBJ ID1  接收
    {0x000203E0,0x10000002,0x1800FFFF},  //扩展帧 主机发送短单播数据帧 MST_SSDATA_TX   MSG OBJ ID2  接收
    {0x00030050,0x10000002,0x1FFFFFFF},  //扩展帧 从机发送短数据帧     SLV_SDATA_TX    MSG OBJ ID3  发送	
};
/*****************************************************************************
* 检测标准表编号保存表格状态
* 返回Current_Save_Tab当前使用的表位号表格ID
* Save_Tab_Sts 标准表编号状态表格状态列表
*****************************************************************************/
void Check_SMtr_Tab_Sts(void)
{
    u16 m,n;
    u16 Len;
    const u8 *Str;
    u8    c;
    Len=sizeof(SAVE_S);
    Len+=0x03;
    Len&=0xFFFC;			                            //4字节对齐
    Current_Save_Tab=0;				                    //默任无保存值
    for(n=0;n<SAVE_TAB_NUMB;n++)
     {
      Str=(u8 *)CFG_BASE_ADDR;
      Str+=n*Len;
      c=*Str;
      if(c==YES)                                 //有修正值
       {
        memcpy(&SOLID_CFG,Str,sizeof(SAVE_S));   //拷贝保存值
        Current_Save_Tab=(n+1);		               //表位号标志为DATA_YES 该修正表格有效
        Save_Tab_Sts[n]=VALIDE;                  
       }                                         
      else if(c!=0xff)                           
       Save_Tab_Sts[n]=NOT_BLACK;	               //表位号标志不为0xff 直接置不空标志
      else                                       
       {                                         
        Save_Tab_Sts[n]=BLANK;                   //先设置表位号表格为空 可以写入表位号数据
        for(m=0;m<(Len/4);m++)
         {                                       //检查后续字节
          if(*((u32 *)Str)!=0xFFFFFFFF)
           {
            Save_Tab_Sts[n]=NOT_BLACK;
            break;
           }
          Str+=4; 
         }
       }
     }
    if(Current_Save_Tab)                         //当前保存有修正数据
     {                                           //判断修正值数据是否可用
      if((SOLID_CFG.LED_NUM!=LED_6)&&            //判断是否为6位数码管
         (SOLID_CFG.LED_NUM!=LED_8))             //判断是否为8位数码管
      	SOLID_CFG.LED_NUM=LED_8;                  //默认8位数码管
       return;                                   //有保存值退出
     } 
    SOLID_CFG.Flag=YES;                          //标志
    SOLID_CFG.LED_NUM=LED_8;                     //默认8位数码管
    for(m=0;m<SAVE_TAB_NUMB;m++)                 //查看是否有空位置
     {
      if(Save_Tab_Sts[m]==BLANK)                 //是否空(可写)
       return;                                   //有空的表位号表格退出
     }                                           
    FlashErase(CFG_BASE_ADDR);                   //没有表位值 且没有空表位位置 擦除表位BANK
}
//固化参数值 即RAM->FLASH
void Solid_Save_Tab(void)
{
    u32 *Ptr,Addr,Data=0;
    u16 Len;
    u8  n;
    Len=sizeof(SAVE_S);                          //保存值长度
    Len+=0x03;
    Len&=0xFFFC;                                 //4字节对齐
    for(n=0;n<SAVE_TAB_NUMB;n++)
     {
      if(Save_Tab_Sts[n]==BLANK)
       break;                                    //有空的修正表格跳出循环
     }
    if(n==SAVE_TAB_NUMB)                         //判断是否没有空表格
     {
      FlashErase(CFG_BASE_ADDR);                 //没有表位值 且没有空表位位置 擦除表位BANK
      for(n=0;n<SAVE_TAB_NUMB;n++)
       Save_Tab_Sts[n]=BLANK;
      Current_Save_Tab=0;
      n=0;
     } 
    if(Current_Save_Tab!=0)                        
     {                                             //当前有修正值 
      Addr=CFG_BASE_ADDR;    
      Addr+=(Current_Save_Tab-1)*Len;              //当前保存值地址(需要擦除) 
      FlashProgram(&Data,Addr,4);                  //写入0 表示表位号无效
      Save_Tab_Sts[Current_Save_Tab-1]=NOT_BLACK;  //not blank
     }
    Ptr=(u32 *)&SOLID_CFG;
    Addr=CFG_BASE_ADDR;
    Addr+=n*Len;
    Save_Tab_Sts[n]=VALIDE;                        //数据有效
    Current_Save_Tab=n+1;                          //更新当前保存表格ID号
    FlashProgram(Ptr,Addr,Len);                    //编程 烧写当前值
}
//读表位号
//表地址编码和专变终端的脉冲输出共用I/O口
void Read_Mtr_Numb(void)
{
    u8 m,n;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);//使能端口G时钟
    SysCtlDelay(500);                       //延时0.08ms
//初始化GPIOG口                             
/*    
	   GPIODirModeSet(GPIOG,
                   BW,
                   GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIOG,                 //端口 设置管脚类型    
                     BW,                    //管脚                 
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPD);//下拉      
*/	
	   GPIOPinTypeGPIOInput(BW_GPIO,BW);         //PORTG设置为输入
    SysCtlDelay(500);                       //延时0.08ms
    for(;;)                                 
     {                                      
      m=GPIOPinRead(GPIOG,BW);              //读表位号
      SysCtlDelay(5000);                    //延时0.5ms
      n=GPIOPinRead(GPIOG,BW);              //读表位号  
      if(m==n)                              //稳定
       break;	   
     }
    if(m!=0xFF)
     m++;	                                 //
    Mtr_Numb_ID=m;                         //
    Mtr_Numb_Str[2]=m%10;                  //低位
    m/=10;                               
    Mtr_Numb_Str[1]=m%10;                  //低位
    m/=10;                               
    Mtr_Numb_Str[0]=m%10;                  //高位
    if(m==0)                             
     Mtr_Numb_Str[0]=DISP_BLANK;           //表位号最高位
				
//    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOG);//禁能端口G时钟
//    Check_Mtr_Tab_Sts();        //检测表位号表格状态 并读出表位号
//    Solid_Mtr_Tab();
				GPIOPinTypeGPIOOutput(MC_OUT_GPIO,      //端口，表位地址设置完成后，PORTG口设置为输出
                          MC_OUT);          //脉冲输出控制
			 GPIOPinWrite(MC_OUT_GPIO,MC_OUT,MC_OUT);           //遥信信号按照设定值输出
}
/****************************************************************************
* 字符串拷贝 将Str1 拷贝到Str2中
* Str1 待拷贝的字符串
* Str2 待写入的字符串
* Len Str2 最大长度
* 待拷贝字符串长度不够 前面填充空格 ' ' 为了显示后对齐
****************************************************************************/
void Fill_Space_StrCpy(u8 *Str1,u8 *Str2,u8 Len)
{
    u8 m,n=0;
    if(Len<2)                        //数据太短
     {	
      *Str2='\0';	
      return;
     } 
    Len--;                           //字符串包含结束符 '\0' 
    for(n=0;n<Len;n++)               //循环剔除Str1中开头的空字符' '
     {
      if(*Str1!=' ')
       break;	
      Str1++;
     }
    if(n==Len)                       //判断是否没有有效数据
     {
      for(m=0;m<Len;m++)
       {
        *Str2=' ';                   //填充空格
        Str2++;
       }
      *Str2='\0';  	 	
      return; 
     }
    else 	
     m=n=strlen((const char*)Str1);  //检查字符串Str1 实际长度
    if(m<Len)
     {
      for(;m<Len;m++)
       {
        *Str2=' ';                   //填充空格
        Str2++;
       }
      Len=n;                         //数据实际长度  	
     }
    if(Str1[Len-1]=='.')             //判断要拷贝的最后一个字符是否为'.' 
     {
      *Str2=' ';                     //多填充一个空格
      Str2++;
      Len--;	
     }
    for(m=0;m<Len;m++)
     {
      *Str2=*Str1;
      Str1++;
      Str2++;
     } 	
    *Str2='\0';  	 	
}
/*****************************************************************************
* 初始化系统频率子程序
*****************************************************************************/
void Init_Pll(void)
{
    SysCtlDelay(62500);               //延时10mS 等待稳定
    SysCtlClockSet(SYSCTL_SYSDIV_5 |  //系统5分频 系统频率=400/5=80MHz 
                   SYSCTL_USE_PLL |   //使用锁相环PLL
                   SYSCTL_OSC_MAIN |  //使用主振荡器时钟
                   SYSCTL_INT_OSC_DIS|//禁能内部振荡器
                   SYSCTL_RCC2_DIV400|//
                   SYSCTL_XTAL_8MHZ); //系统晶振频率8MHz
    Sysclk=SysCtlClockGet();          //获取系统频率
}
#if 0
/*****************************************************************************
* 初始化GPIO口 
*****************************************************************************/
void Init_Gpio(void)
{
//设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能端口A时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能端口B时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能端口C时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能端口D时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能端口E时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能端口F时钟
//	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);//使能端口G时钟，在读地址子程序中使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//使能端口H时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);//使能端口J时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);//使能端口K时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);//使能端口L时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);//使能端口M时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);//使能端口N时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);//使能端口P时钟
//初始化GPIOA口
	   GPIOPinConfigure(GPIO_PA0_U0RX);
	   GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(UART0_GPIO,             //端口
                    U0RX|                   //管脚 初始化UART0接口管脚	 PA.0 PA.1
                    U0TX);                  //管脚 初始化UART0接口管脚	 PA.0 PA.1

	   GPIOPinTypeGPIOOutput(DISP_RST_GPIO,    //端口
                          DISP_RST);        //管脚 PA6 HD7219复位输出
																										
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
				GPIOPinConfigure(GPIO_PA3_SSI0FSS);
				GPIOPinConfigure(GPIO_PA4_SSI0RX);
				GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(SPI0_GPIO,               //端口
                   SSICLK|SSIFSS|           //管脚 SSICLK SSIFSS SSIRX SSITX设置为SSI管脚
																			SSIRX|SSITX);            //PA2 PA3 PA4 PA5

    GPIOPinTypeGPIOInput(KEY_IN_GPIO,       //端口
                         KEY_IN);           //按键输入口PA7
//初始化GPIOB口   
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
				GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(CAN0_GPIO,               //端口，初始化CAN0接口管脚
                   CAN0RX|                  //管脚PB4
                   CAN0TX);                 //PB5
																			
    GPIOPinTypeGPIOInput(YK1NO_GPIO,        //端口
                         YK1NO);            //遥控信号1常开
    GPIOPinTypeGPIOInput(YK1NC_GPIO,        //端口
                         YK1NC);            //遥控信号1常闭
				GPIOPinTypeGPIOInput(YK2NO_GPIO,        //端口
                         YK2NO);            //遥控信号2常开	
    GPIOPinTypeGPIOInput(YK2NC_GPIO,        //端口
                         YK2NC);            //遥控信号2常闭																									
//初始化GPIOC口																									
				GPIOPinConfigure(GPIO_PC4_U1RX);
				GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(UART1_GPIO,             //端口
                    U1RX|                   //管脚PC4，PC5
                    U1TX);                  //初始化UART1接口管脚
				GPIOPinConfigure(GPIO_PC6_U3RX);																
				GPIOPinConfigure(GPIO_PC7_U3TX);
				GPIOPinTypeUART(UART3_GPIO,             //端口
				                U3RX|                   //管脚PC6，PC7
																				U3TX);                  //初始化UART3接口管脚
//初始化GPIOD口
				GPIOPinConfigure(GPIO_PD4_U6RX);
				GPIOPinConfigure(GPIO_PD5_U6TX);
    GPIOPinTypeUART(UART6_GPIO,             //端口
                    U6RX|                   //管脚PD4，PD5
                    U6TX);                  //初始化UART6接口管脚    

//初始化GPIOE口
				GPIOPinConfigure(GPIO_PE0_U7RX);
				GPIOPinConfigure(GPIO_PE1_U7TX);
    GPIOPinTypeUART(UART7_GPIO,             //端口
                    U7RX|                   //管脚
                    U7TX);                  //初始化UART7接口管脚
    GPIOPinTypeADC(AIN0_GPIO,AIN0);         //初始化AIN0引脚
    GPIOPinTypeGPIOOutput(TZ_ZS_GPIO,       //端口
                          TZ_ZS);           //跳闸指示灯控制
				GPIOPinConfigure(GPIO_PE4_U5RX);
				GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(UART5_GPIO,             //端口
                    U5RX|                   //管脚
                    U5TX);                  //初始化UART5接口管脚
    GPIOPinConfigure(GPIO_PE6_CAN1RX);
				GPIOPinConfigure(GPIO_PE7_CAN1TX);
    GPIOPinTypeCAN(CAN1_GPIO,               //端口
                   CAN1RX|                  //管脚
                   CAN1TX);                 //初始化CAN1接口管脚
//初始化GPIOF口
/* 解锁IO口,PF0*/
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0xFF;               /* bits can be written */
    GPIOPinTypeGPIOOutput(UC_ESWC_GPIO,     //端口
                          UC_ESWC);         //C相电压电子开关控制
    GPIOPinTypeGPIOOutput(UB_ESWC_GPIO,     //端口
                          UB_ESWC);         //B相电压电子开关控制
    GPIOPinTypeGPIOOutput(UA_ESWC_GPIO,     //端口
                          UA_ESWC);         //A相电压电子开关控制 
    GPIOPinTypeGPIOOutput(GBC_C_GPIO,       //端口
                          GBC_C);           //C相续流继电器断开控制 		
    GPIOPinTypeGPIOOutput(TEST_LAMP_GPIO,   //端口
                          TEST_LAMP);	      //管脚PN5 检定指示灯控制引脚																										
    GPIOPinTypeGPIOOutput(UC_JC_GPIO,       //端口
                          UC_JC);           //C相电压继电器控制 
    GPIOPinTypeGPIOOutput(UA_JC_GPIO,       //端口
                          UA_JC);	          //管脚PN6 A相电压继电器控制
    GPIOPinTypeGPIOOutput(UB_JC_GPIO,       //端口
                          UB_JC);	          //管脚PN7 B相电压继电器控制 																										
   
//初始化GPIOG口     
//初始化GPIOH口   
    GPIOPinTypeGPIOOutput(ABBA_GPIO,        //端口
                          ABBA);            //RS485极性翻转控制引脚
//初始化GPIOJ口

    GPIOPinTypeGPIOOutput(YX_CTL_GPIO,      //端口
                          YX_CTL);          //遥信控制，设置为输出  
//初始化GPIOK口       
    GPIOPinTypeGPIOInput(YK4NO_GPIO,        //端口
                         YK4NO);            //遥控信号4常开,PK4
    GPIOPinTypeGPIOInput(YK4NC_GPIO,        //端口
                         YK4NC);            //遥控信号4常闭,PK5
				GPIOPinTypeGPIOInput(YK3NO_GPIO,        //端口
                         YK3NO);            //遥控信号3常开,PK6	
    GPIOPinTypeGPIOInput(YK3NC_GPIO,        //端口
                         YK3NC);            //遥控信号3常闭,PK7
//初始化GPIOL口    
    GPIOPinConfigure(GPIO_PL0_T0CCP0);      //配置I/O口复用功能；PL0
				GPIOPinConfigure(GPIO_PL1_T0CCP1);      //配置I/O口复用功能；PL1
				GPIOPinConfigure(GPIO_PL2_T1CCP0);      //配置I/O口复用功能；PL2
				GPIOPinConfigure(GPIO_PL3_T1CCP1);      //配置I/O口复用功能；PL3
				GPIOPinConfigure(GPIO_PL4_T2CCP0);      //配置I/O口复用功能；PL4
				GPIOPinConfigure(GPIO_PL5_T2CCP1);      //配置I/O口复用功能；PL5
				GPIOPinConfigure(GPIO_PL6_T3CCP0);      //配置I/O口复用功能；PL6
				GPIOPinConfigure(GPIO_PL7_T3CCP1);      //配置I/O口复用功能；PL7				
    GPIOPinTypeTimer(HFIN1_GPIO,            //端口
				                 HFIN1);                //管脚
    GPIOPinTypeTimer(HFIN2_GPIO,            //端口
				                 HFIN2);                //管脚
    GPIOPinTypeTimer(HFIN3_GPIO,            //端口
				                 HFIN3);                //管脚		
    GPIOPinTypeTimer(FH_IN_GPIO,            //端口
                     FH_IN);                //管脚
    GPIOPinTypeTimer(JZ_IN_GPIO,            //端口
                     JZ_IN);                //管脚
    GPIOPinTypeTimer(QFH_IN_GPIO,
				                 QFH_IN);																			
    GPIOPinTypeTimer(WGFMC_GPIO,            //端口 
                     WGFMC);                //管脚		
    GPIOPinTypeTimer(YGFMC_GPIO,            //端口
                     YGFMC);                //管脚  																					
//初始化GPIOM口				    
    GPIOPinConfigure(GPIO_PM0_T4CCP0);      //配置I/O口复用功能；PM0
				GPIOPinConfigure(GPIO_PM1_T4CCP1);      //配置I/O口复用功能；PM1
				GPIOPinConfigure(GPIO_PM2_T5CCP0);      //配置I/O口复用功能；PM2
				
    GPIOPinTypeTimer(WGMC_GPIO,             //端口 
                     WGMC);                 //管脚 																								
				GPIOPinTypeTimer(YGMC_GPIO,             //端口 
                     YGMC);                 //管脚
    GPIOPinTypeTimer(SZ_MC_GPIO,            //端口 
                     SZ_MC);                //管脚 																					
																						
    GPIOPinTypeGPIOInput(GDT_MC_GPIO,       //端口
                         GDT_MC);           //管脚PM3 光电头脉冲输入
    GPIOPinTypeGPIOOutput(GDT_RST_GPIO,     //端口
                          GDT_RST);         //管脚PM4 光电头复位
    GPIOPinTypeGPIOInput(XL_MC_GPIO,        //端口
                         XL_MC);            //管脚PM5 需量周期输入
    GPIOPinTypeGPIOInput(TQ_MC_GPIO,        //端口
                         TQ_MC);            //备用PM6 时段投切输入    
				GPIOPinTypeGPIOOutput(GOG_KZ_GPIO,      //端口
                          GOG_KZ);          //管脚PM7 被检表脉冲共高共低选择控制 																										
//初始化GPION口															
    GPIOPinTypeGPIOOutput(TTAB1_GPIO,       //端口
				                      TTAB1);           //管脚PN0 双回路切换1
    GPIOPinTypeGPIOOutput(TTAB2_GPIO,       //端口
				                      TTAB2);           //管脚PN1 双回路切换2
    GPIOPinTypeGPIOOutput(WDI_GPIO,         //端口
				                      WDI);             //管脚PN3 外置看门狗喂狗引脚
    GPIOPinTypeGPIOInput(OPEN_IN_GPIO,      //端口
				                     OPEN_IN);          //管脚PN4 续流保护继电器状态输入引脚
    GPIOPinTypeGPIOOutput(BGB_C_GPIO,       //端口
                          BGB_C);           //管脚PN5 三相续流继电器闭合控制
    GPIOPinTypeGPIOOutput(GBA_C_GPIO,       //端口
                          GBA_C);           //管脚PN6 A相续流继电器断开控制
    GPIOPinTypeGPIOOutput(GBB_C_GPIO,       //端口
                          GBB_C);           //管脚PN7 B相续流继电器断开控制																									
              
//初始化GPIOP口  			
    GPIOPinTypeGPIOOutput(DOOR_GPIO,        //端口
                          DOOR);            //管脚PP0 门控信号
    GPIOPinTypeGPIOInput(BJ_SIG_GPIO,       //端口
                         BJ_SIG);           //管脚PP1 电表报警信号																									
       
//初始化端口中断 GPIOA
    GPIOPadConfigSet(KEY_IN_GPIO,           //端口 设置管脚类型    
                     KEY_IN,                //管脚             
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(KEY_IN_GPIO,             //端口 设置管脚中断发送
                   KEY_IN,                  //管脚             
                   GPIO_FALLING_EDGE);      //中断方式             
//初始化端口中断 GPIOM
    GPIOPadConfigSet(GDT_MC_GPIO,           //端口 设置管脚类型    
                     GDT_MC,                //管脚                 
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉            
    GPIOIntTypeSet(GDT_MC_GPIO,             //端口 设置管脚中断发送
                   GDT_MC,                  //管脚                 
                   GPIO_FALLING_EDGE);      //中断方式
    GPIOPadConfigSet(XL_MC_GPIO,            //端口 设置管脚类型    
                     XL_MC,                 //管脚 需量输入 时段投切 合闸脉冲
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(XL_MC_GPIO,              //端口 设置管脚中断方式
                   XL_MC,                   //管脚 需量输入 时段投切 合闸脉冲
                   GPIO_FALLING_EDGE);      //中断方式             
    GPIOPadConfigSet(TQ_MC_GPIO,            //端口 设置管脚类型    
                     TQ_MC,                 //管脚 需量输入 时段投切 合闸脉冲
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(TQ_MC_GPIO,              //端口 设置管脚中断方式
                   TQ_MC,                   //管脚 需量输入 时段投切 合闸脉冲
                   GPIO_FALLING_EDGE);      //中断方式             
//初始化端口中断 GPION
    GPIOPadConfigSet(OPEN_IN_GPIO,          //端口 设置管脚类型    
                     OPEN_IN,               //管脚             
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉 
    GPIOIntTypeSet(OPEN_IN_GPIO,            //端口 设置管脚中断方式
				               OPEN_IN,                 //管脚 续流继电器状态信号输入
																			GPIO_FALLING_EDGE);      //中断方式
    GPIOIntEnable(OPEN_IN_GPIO,             //端口 设置管脚中断使能
                  OPEN_IN);	                //管脚																				
                     
//端口初步设置
    DOWN_JOIN;                              //脉冲输出共E
    I_JDQ=1;                                //默认电流继电器接入 电流旁路
    DISP_RST_EN;                            //显示复位
    GDT_RST_DN;                             //
    TEST_LAMP_OFF;                          //校验指示灯灭
    Uin_Iin_Pr();                           //电压电流接入标志
    WDI_HIGH;                               //
    TTAB_JDQ_STS='2';                       //双回路继电器状态寄存 
				TTAB_JDQ_DELY=1;                        //双回路继电器动作延时开始
				TTAB_JDQ_CHG=0;                         //双回路继电器动作标志
				TTAB_JDQ_DELY_Timer=(u16)Timer_1ms;     //初始化双回路继电器动作延时定时
				GPIOPinWrite(YX_CTL_GPIO,YX_CTL,YX_CTL);           //遥信信号按照设定值输出
}
#endif
/*****************************************************************************
* 初始化GPIO口 
*****************************************************************************/
void Init_Gpio(void)
{
//设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能端口A时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能端口B时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能端口C时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能端口D时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能端口E时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能端口F时钟
//	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);//使能端口G时钟，在读地址子程序中使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//使能端口H时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);//使能端口J时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);//使能端口K时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);//使能端口L时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);//使能端口M时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);//使能端口N时钟
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);//使能端口P时钟
//初始化GPIOA口
	   GPIOPinTypeGPIOOutput(DISP_RST_GPIO,    //端口
                          DISP_RST);        //管脚 PA6 HD7219复位输出
																										
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(SPI0_GPIO,               //端口
                   SSICLK|SSIFSS|           //管脚 SSICLK SSIFSS SSIRX SSITX设置为SSI管脚
				   SSIRX|SSITX);            //PA2 PA3 PA4 PA5

    GPIOPinTypeGPIOInput(KEY_IN_GPIO,       //端口
                         KEY_IN);           //按键输入口PA7
//初始化GPIOB口   							
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(I2C0_SDA_GPIO,
                    I2C0_SDA);
    GPIOPinTypeI2CSCL(I2C0_SCL_GPIO,
                        I2C0_SCL);
//初始化GPIOC口		
	//    GPIOPinConfigure(GPIO_PC6_U3RX);
	//    GPIOPinConfigure(GPIO_PC7_U3TX);
    // GPIOPinTypeUART(UART0_GPIO,             //端口
    //                 U3RX|                   //管脚 初始化UART0接口管脚	 PC.6 PC.7
    //                 U3TX);                  //管脚 初始化UART0接口管脚	 PC.6 PC.7																							
				GPIOPinConfigure(GPIO_PC4_U1RX);
				GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(UART1_GPIO,             //端口
                    U1RX|                   //管脚PC4，PC5
                    U1TX);                  //初始化UART1接口管脚
				GPIOPinConfigure(GPIO_PC6_U3RX);																
				GPIOPinConfigure(GPIO_PC7_U3TX);
				GPIOPinTypeUART(UART3_GPIO,             //端口
				                U3RX|                   //管脚PC6，PC7
																				U3TX);                  //初始化UART3接口管脚
//初始化GPIOD口
GPIOPinConfigure(GPIO_PD4_U6RX);
GPIOPinConfigure(GPIO_PD5_U6TX);
GPIOPinTypeUART(UART6_GPIO,
                    U6RX|
                    U6TX);
GPIOPinTypeGPIOOutput(U_5460A_SCLK_GPIO,
                          U_5460A_SCLK|
                          U_5460A_CS|
                          U_5460A_SDI); 
// GPIOPinTypeGPIOOutput(U_5460A_CS_GPIO,
//                             U_5460A_CS);
// GPIOPinTypeGPIOOutput(U_5460A_SDO_GPIO,
//                             U_5460A_SDO);
GPIOPinTypeGPIOInput(U_5460A_SDO_GPIO,
                            U_5460A_SDO);                                                      

//初始化GPIOE口
GPIOPinTypeGPIOOutput(U_5460A_RST_GPIO,
                        U_5460A_RST);
	// 			GPIOPinConfigure(GPIO_PE0_U7RX);
	// 			GPIOPinConfigure(GPIO_PE1_U7TX);
    // GPIOPinTypeUART(UART7_GPIO,             //端口
    //                 U7RX|                   //管脚
    //                 U7TX);                  //初始化UART7接口管脚
    // GPIOPinTypeADC(AIN0_GPIO,AIN0);         //初始化AIN0引脚
    // GPIOPinTypeGPIOOutput(TZ_ZS_GPIO,       //端口
    //                       TZ_ZS);           //跳闸指示灯控制
	// 			GPIOPinConfigure(GPIO_PE4_U5RX);
	// 			GPIOPinConfigure(GPIO_PE5_U5TX);
    // GPIOPinTypeUART(UART5_GPIO,             //端口
    //                 U5RX|                   //管脚
    //                 U5TX);                  //初始化UART5接口管脚
    // GPIOPinConfigure(GPIO_PE6_CAN1RX);
	// 			GPIOPinConfigure(GPIO_PE7_CAN1TX);
    // GPIOPinTypeCAN(CAN1_GPIO,               //端口
    //                CAN1RX|                  //管脚
    //                CAN1TX);                 //初始化CAN1接口管脚
//初始化GPIOF口
/* 解锁IO口,PF0*/
    GPIOPinConfigure(GPIO_PF0_CAN0RX);
				GPIOPinConfigure(GPIO_PF3_CAN0TX);
    GPIOPinTypeCAN(CAN0_GPIO,               //端口，初始化CAN0接口管脚
                   CAN0RX|                  //管脚PF0
                   CAN0TX);                 //PF3
												
    // HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    // HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0xFF;               /* bits can be written */
    // GPIOPinTypeGPIOOutput(UC_ESWC_GPIO,     //端口
    //                       UC_ESWC);         //C相电压电子开关控制
    // GPIOPinTypeGPIOOutput(UB_ESWC_GPIO,     //端口
    //                       UB_ESWC);         //B相电压电子开关控制
    // GPIOPinTypeGPIOOutput(UA_ESWC_GPIO,     //端口
    //                       UA_ESWC);         //A相电压电子开关控制 
    // GPIOPinTypeGPIOOutput(GBC_C_GPIO,       //端口
    //                       GBC_C);           //C相续流继电器断开控制 		
    // GPIOPinTypeGPIOOutput(TEST_LAMP_GPIO,   //端口
    //                       TEST_LAMP);	      //管脚PN5 检定指示灯控制引脚																										
    // GPIOPinTypeGPIOOutput(UC_JC_GPIO,       //端口
    //                       UC_JC);           //C相电压继电器控制 
    // GPIOPinTypeGPIOOutput(UA_JC_GPIO,       //端口
    //                       UA_JC);	          //管脚PN6 A相电压继电器控制
    // GPIOPinTypeGPIOOutput(UB_JC_GPIO,       //端口
    //                       UB_JC);	          //管脚PN7 B相电压继电器控制 																										
   
//初始化GPIOG口     
                                                                                        
//初始化GPIOH口   
    GPIOPinTypeGPIOOutput(ABBA_GPIO,        //端口
                          ABBA);            //RS485极性翻转控制引脚
    GPIOPinTypeGPIOInput(I_5460A_INT_GPIO,
                            I_5460A_INT);
    GPIOPinTypeGPIOOutput(I_5460A_INT_GPIO,
                            I_5460A_SCLK|
                            I_5460A_CS|
                            I_5460A_SDI);
    // GPIOPinTypeGPIOOutput(I_5460A_SCLK_GPIO,
    //                         I_5460A_SCLK);
    // GPIOPinTypeGPIOOutput(I_5460A_CS_GPIO,
    //                         I_5460A_CS);
    // GPIOPinTypeGPIOOutput(I_5460A_SDO_GPIO,
    //                         I_5460A_SDO);
    GPIOPinTypeGPIOInput(I_5460A_SDO_GPIO,
                            I_5460A_SDO);                                  
//初始化GPIOJ口

    // GPIOPinTypeGPIOOutput(YX_CTL_GPIO,      //端口
    //                       YX_CTL);          //遥信控制，设置为输出  
    GPIOPinTypeGPIOOutput(NO_SELT3_GPIO,
                            NO_SELT3|
                            P3P4SELOUT3|
                            NO_SELT2|
                            P3P4SELOUT2|
                            NO_SELT1|
                            P3P4SELOUT1|
                            K2|
                            K1);
// GPIOPinTypeGPIOOutput(P3P4SELOUT3_GPIO,
//                         P3P4SELOUT3);
// GPIOPinTypeGPIOOutput(NO_SELT2_GPIO,
//                         NO_SELT2);
// GPIOPinTypeGPIOOutput(P3P4SELOUT2_GPIO,
//                         P3P4SELOUT2);
// GPIOPinTypeGPIOOutput(NO_SELT1_GPIO,
//                         NO_SELT1);
// GPIOPinTypeGPIOOutput(P3P4SELOUT1_GPIO,
//                         P3P4SELOUT1);
// GPIOPinTypeGPIOOutput(K2_GPIO,
//                         K2);
// GPIOPinTypeGPIOOutput(K1_GPIO,
//                         K1);                                                                                                                                                                            
//初始化GPIOK口       
    GPIOPinTypeGPIOInput(YK4NO_GPIO,        //端口
                         YK4NO);            //遥控信号4常开,PK4
    GPIOPinTypeGPIOInput(YK4NC_GPIO,        //端口
                         YK4NC);            //遥控信号4常闭,PK5
				GPIOPinTypeGPIOInput(YK3NO_GPIO,        //端口
                         YK3NO);            //遥控信号3常开,PK6	
    GPIOPinTypeGPIOInput(YK3NC_GPIO,        //端口
                         YK3NC);            //遥控信号3常闭,PK7
//初始化GPIOL口    
    GPIOPinConfigure(GPIO_PL0_T0CCP0);      //配置I/O口复用功能；PL0
				GPIOPinConfigure(GPIO_PL1_T0CCP1);      //配置I/O口复用功能；PL1
				GPIOPinConfigure(GPIO_PL2_T1CCP0);      //配置I/O口复用功能；PL2
				GPIOPinConfigure(GPIO_PL3_T1CCP1);      //配置I/O口复用功能；PL3
				GPIOPinConfigure(GPIO_PL4_T2CCP0);      //配置I/O口复用功能；PL4
				GPIOPinConfigure(GPIO_PL5_T2CCP1);      //配置I/O口复用功能；PL5
				GPIOPinConfigure(GPIO_PL6_T3CCP0);      //配置I/O口复用功能；PL6
				GPIOPinConfigure(GPIO_PL7_T3CCP1);      //配置I/O口复用功能；PL7				
    GPIOPinTypeTimer(HFIN1_GPIO,            //端口
				                 HFIN1);                //管脚
    GPIOPinTypeTimer(HFIN2_GPIO,            //端口
				                 HFIN2);                //管脚
    GPIOPinTypeTimer(HFIN3_GPIO,            //端口
				                 HFIN3);                //管脚		
    GPIOPinTypeTimer(FH_IN_GPIO,            //端口
                     FH_IN);                //管脚
    GPIOPinTypeTimer(JZ_IN_GPIO,            //端口
                     JZ_IN);                //管脚
    GPIOPinTypeTimer(QFH_IN_GPIO,
				                 QFH_IN);																			
    GPIOPinTypeTimer(WGFMC_GPIO,            //端口 
                     WGFMC);                //管脚		
    GPIOPinTypeTimer(YGFMC_GPIO,            //端口
                     YGFMC);                //管脚  																					
//初始化GPIOM口				    
    GPIOPinConfigure(GPIO_PM0_T4CCP0);      //配置I/O口复用功能；PM0
				GPIOPinConfigure(GPIO_PM1_T4CCP1);      //配置I/O口复用功能；PM1
				GPIOPinConfigure(GPIO_PM2_T5CCP0);      //配置I/O口复用功能；PM2
				
    GPIOPinTypeTimer(WGMC_GPIO,             //端口 
                     WGMC);                 //管脚 																								
				GPIOPinTypeTimer(YGMC_GPIO,             //端口 
                     YGMC);                 //管脚
    GPIOPinTypeTimer(SZ_MC_GPIO,            //端口 
                     SZ_MC);                //管脚 																					
																						
    GPIOPinTypeGPIOInput(GDT_MC_GPIO,       //端口
                         GDT_MC);           //管脚PM3 光电头脉冲输入
    GPIOPinTypeGPIOOutput(GDT_RST_GPIO,     //端口
                          GDT_RST);         //管脚PM4 光电头复位
    GPIOPinTypeGPIOInput(XL_MC_GPIO,        //端口
                         XL_MC);            //管脚PM5 需量周期输入
    GPIOPinTypeGPIOInput(TQ_MC_GPIO,        //端口
                         TQ_MC);            //备用PM6 时段投切输入    
				GPIOPinTypeGPIOOutput(GOG_KZ_GPIO,      //端口
                          GOG_KZ);          //管脚PM7 被检表脉冲共高共低选择控制 																										
//初始化GPION口				
    GPIOPinTypeGPIOOutput(I_5460A_RST_GPIO,
                            I_5460A_RST);
    GPIOPinTypeGPIOOutput(TTAB1_GPIO,       //端口
				                      TTAB1);           //管脚PN0 双回路切换1
    GPIOPinTypeGPIOOutput(TTAB2_GPIO,       //端口
				                      TTAB2);           //管脚PN1 双回路切换2
    GPIOPinTypeGPIOOutput(WDI_GPIO,         //端口
				                      WDI);             //管脚PN3 外置看门狗喂狗引脚
    GPIOPinTypeGPIOInput(OPEN_IN_GPIO,      //端口
				                     OPEN_IN);          //管脚PN4 续流保护继电器状态输入引脚
    GPIOPinTypeGPIOOutput(BGB_C_GPIO,       //端口
                          BGB_C);           //管脚PN5 三相续流继电器闭合控制
    GPIOPinTypeGPIOOutput(GBA_C_GPIO,       //端口
                          GBA_C);           //管脚PN6 A相续流继电器断开控制
    GPIOPinTypeGPIOOutput(GBB_C_GPIO,       //端口
                          GBB_C);           //管脚PN7 B相续流继电器断开控制																									
              
//初始化GPIOP口  			
    GPIOPinTypeGPIOOutput(DOOR_GPIO,        //端口
                          DOOR);            //管脚PP0 门控信号
    GPIOPinTypeGPIOInput(BJ_SIG_GPIO,       //端口
                         BJ_SIG);           //管脚PP1 电表报警信号	
    GPIOPinTypeGPIOInput(U_5460A_INT_GPIO,
                            U_5460A_INT);                         																								
       
//初始化端口中断 GPIOA
    GPIOPadConfigSet(KEY_IN_GPIO,           //端口 设置管脚类型    
                     KEY_IN,                //管脚             
                     GPIO_STRENGTH_8MA,     //驱动能力             
                     GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
    GPIOIntTypeSet(KEY_IN_GPIO,             //端口 设置管脚中断发送
                   KEY_IN,                  //管脚             
                   GPIO_FALLING_EDGE);      //中断方式        
    U_5460A_CS_H;
    U_5460A_SCLK_H;
    U_5460A_SDI_H;
    U_5460A_RST_H;
    I_5460A_CS_H;
    I_5460A_SCLK_H;
    I_5460A_SDI_H;
    I_5460A_RST_H;

// //初始化端口中断 GPIOM
//     GPIOPadConfigSet(GDT_MC_GPIO,           //端口 设置管脚类型    
//                      GDT_MC,                //管脚                 
//                      GPIO_STRENGTH_8MA,     //驱动能力             
//                      GPIO_PIN_TYPE_STD_WPU);//上拉/下拉            
//     GPIOIntTypeSet(GDT_MC_GPIO,             //端口 设置管脚中断发送
//                    GDT_MC,                  //管脚                 
//                    GPIO_FALLING_EDGE);      //中断方式
//     GPIOPadConfigSet(XL_MC_GPIO,            //端口 设置管脚类型    
//                      XL_MC,                 //管脚 需量输入 时段投切 合闸脉冲
//                      GPIO_STRENGTH_8MA,     //驱动能力             
//                      GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
//     GPIOIntTypeSet(XL_MC_GPIO,              //端口 设置管脚中断方式
//                    XL_MC,                   //管脚 需量输入 时段投切 合闸脉冲
//                    GPIO_FALLING_EDGE);      //中断方式             
//     GPIOPadConfigSet(TQ_MC_GPIO,            //端口 设置管脚类型    
//                      TQ_MC,                 //管脚 需量输入 时段投切 合闸脉冲
//                      GPIO_STRENGTH_8MA,     //驱动能力             
//                      GPIO_PIN_TYPE_STD_WPU);//上拉/下拉               
//     GPIOIntTypeSet(TQ_MC_GPIO,              //端口 设置管脚中断方式
//                    TQ_MC,                   //管脚 需量输入 时段投切 合闸脉冲
//                    GPIO_FALLING_EDGE);      //中断方式             
// //初始化端口中断 GPION
//     GPIOPadConfigSet(OPEN_IN_GPIO,          //端口 设置管脚类型    
//                      OPEN_IN,               //管脚             
//                      GPIO_STRENGTH_8MA,     //驱动能力             
//                      GPIO_PIN_TYPE_STD_WPU);//上拉/下拉 
//     GPIOIntTypeSet(OPEN_IN_GPIO,            //端口 设置管脚中断方式
// 				               OPEN_IN,                 //管脚 续流继电器状态信号输入
// 							   GPIO_FALLING_EDGE);      //中断方式
//     GPIOIntEnable(OPEN_IN_GPIO,             //端口 设置管脚中断使能
//                   OPEN_IN);	                //管脚																				
                     
//端口初步设置
    // DOWN_JOIN;                              //脉冲输出共E
    // I_JDQ=1;                                //默认电流继电器接入 电流旁路
    // DISP_RST_EN;                            //显示复位
    // GDT_RST_DN;                             //
    // TEST_LAMP_OFF;                          //校验指示灯灭
    // Uin_Iin_Pr();                           //电压电流接入标志
    // WDI_HIGH;                               //
    // TTAB_JDQ_STS='2';                       //双回路继电器状态寄存 
	// 			TTAB_JDQ_DELY=1;                        //双回路继电器动作延时开始
	// 			TTAB_JDQ_CHG=0;                         //双回路继电器动作标志
	// 			TTAB_JDQ_DELY_Timer=(u16)Timer_1ms;     //初始化双回路继电器动作延时定时
	// 			GPIOPinWrite(YX_CTL_GPIO,YX_CTL,YX_CTL);           //遥信信号按照设定值输出
}
/*****************************************************************************
* 初始化系统节拍定时器 
*****************************************************************************/
void Init_SysTick(void)
{
    SysTickPeriodSet((Sysclk/1000000)*SYS_TIME); //设置系统节拍定时周期
    SysTickEnable();			                  //使能系统节拍定时器
    SysTickIntEnable();			                //使能系统节拍定时器中断
}
/*****************************************************************************
* 初始化SSI
*****************************************************************************/
void Init_Ssi(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);//使能SSI0时钟
    SSIConfigSetExpClk(SSI0,Sysclk,\
                            SSI_FRF_MOTO_MODE_0,\
                            SSI_MODE_MASTER,\
                            SSI_BITRATE,\
                            SSI_CR0_DSS_8);    //配置SSI CR0
    SSIEnable(SSI0);                           //使能SSI0
}
/*****************************************************************************
* 串口初始化
* 人口 Com:端口号
*****************************************************************************/
void Init_Com(u8 Com)
{
    float f;
    UART_PARA UART_P;
    u32 UARTx;
    UARTx = UART_PORT[Com];
    *((u32*)&UART_P) = Parity_Set_TAB[Uart_Para[Com].PARITY];    //校验位
    UART_P.Data_Len=Uart_Para[Com].LEN;              //数据位数
    UART_P.Stop2=Uart_Para[Com].STOP;                //停止位	 
    f=40000;                                         //
    f/=(Uart_Para[Com].BAUD);                        //4个字符接收时间
    f/=TIMER_8MS;
    if(f<10)
     Com_Rx_OvTm[Com]=10;                            //最少等待80ms
    else if(f>31)
     Com_Rx_OvTm[Com]=31;                            //最多等待248ms
    else
     Com_Rx_OvTm[Com]=(s8)(f+0.5);                   //接收超时设定 等待4字节时间
    UARTConfigSetExpClk(UARTx,Sysclk,Uart_Para[Com].BAUD,*((u32*)&UART_P));
    UARTFIFOLevelSet(UARTx,UART_FIFO_TX1_8,          //发送FIFO 中断触发深度1/8
                           UART_FIFO_RX7_8);         //接收FIFO 中断触发深度7/8
    UARTIntEnable(UARTx,UART_INT_RX |                //开启接收中断 
                        UART_INT_RT|                 //开启接收超时中断
                        UART_INT_TX );	              //开启发送中断
}
/*****************************************************************************
* 串口初始化
* 管脚已在Init_Gpio()中设置
*****************************************************************************/
void Init_Uart(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);       //UART0设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);       //UART1设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);       //UART3设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);       //UART5设备时钟使能
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);       //UART6设备时钟使能	
    memcpy(&Uart_Para[MTRCOM],                         //MTRCOM 参数
           &UART_PARA_TAB[MTRCOM],
           4);
    memcpy(&Uart_Para[LCTCOM],                         //LCTCOM参数
           &UART_PARA_TAB[LCTCOM],
           4);
    Init_Com(MTRCOM);                                  //初始化UART0 
    Init_Com(LCTCOM);                                  //初始化UART1
	   Init_Com(RTECOM);                                  //初始化UART3
	   Init_Com(ATECOM);                                  //初始化UART5
	   Init_Com(IRECOM);                                  //初始化UART6
}
/*****************************************************************************
* 初始化定时器
* Timer0 Timer1 Timer2 被分成六个计数器
* Timer0-A Timer0-B Timer1-A Timer1-B Timer2-A Timer2-B 
*****************************************************************************/
void Init_Timer(void)
{
	   	
//初始化Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	//使能设备时钟	   
    TimerConfigure(TIMER0,TIMER_CFG_SPLIT_PAIR|   //TIMER0分成两个16bit 计数器
                          TIMER_CFG_A_CAP_COUNT|  //Timer0-A 边沿计数模式	CCP0输入
                          TIMER_CFG_B_CAP_COUNT); //Timer0-B 边沿计数模式	CCP1输入
//初始化Timer0-A 备用高频信号1输入
    TimerControlStall(TIMER0_BASE, 
	                     TIMER_A,
                      true);	
    TimerControlEvent(TIMER0_BASE,                //定时器
                      TIMER_A,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式,只捕捉下降沿
    TimerLoadSet(TIMER0_BASE,                     //定时器
                 TIMER_A,                         //通道
                 0XFFFF);                         //设置重装值(分频系数)
    TimerMatchSet(TIMER0_BASE,                    //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER0_BASE,                   //定时器
                   TIMER_CAPA_MATCH);             //中断类型
    TimerEnable(TIMER0_BASE,                      //定时器
                TIMER_A);                         //定时器启动

//初始化Timer0-B 备用高频信号2输入                //设置CCP1触发沿为下降沿
    TimerControlStall(TIMER0_BASE, 
				                  TIMER_B,
                      true);
    TimerControlEvent(TIMER0_BASE,                //定时器
                      TIMER_B,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式，只捕捉下降沿
    TimerLoadSet(TIMER0_BASE,                     //定时器
                 TIMER_B,                         //通道
                 0xFFFF);                         //设置重装值(分频系数)
    TimerMatchSet(TIMER0_BASE,                    //定时器
                  TIMER_B,                        //通道
                  0);                             //设置比较值																		
    TimerIntEnable(TIMER0_BASE,                   //定时器
                   TIMER_CAPB_MATCH);             //中断类型           
    TimerEnable(TIMER0_BASE,                      //定时器
                TIMER_B);                         //定时器启动
					
//初始化Timer1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	//使能设备时钟
    TimerConfigure(TIMER1_BASE,
                   TIMER_CFG_SPLIT_PAIR|          //TIMER1分成两个16bit 计数器
                   TIMER_CFG_A_CAP_COUNT|         //Timer1-A 边沿计数模式	CCP0输入
                   TIMER_CFG_B_CAP_COUNT);        //Timer1-B 边沿计数模式	CCP1输入
//初始化Timer1-A 备用高频信号3输入                //设置触发沿为下降沿
    TimerControlStall(TIMER1_BASE,
				                  TIMER_A,
																						true); 
    TimerControlEvent(TIMER1_BASE,                //定时器
                      TIMER_A,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER1_BASE,                     //定时器
                 TIMER_A,                         //通道
                 0XFFFF);                         //设置重装值(分频系数)CLK_RELOAD_VAL_N
    TimerMatchSet(TIMER1_BASE,                    //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER1_BASE,                   //定时器
                   TIMER_CAPA_MATCH);             //中断类型
    TimerEnable(TIMER1_BASE,                      //定时器
                TIMER_A);                         //定时器启动
//初始化Timer1-B 500kHz标准晶振脉冲输入           //设置触发沿为下降沿
    TimerControlStall(TIMER1_BASE,
                      TIMER_B,
                      true);
    TimerControlEvent(TIMER1_BASE,                //定时器
                      TIMER_B,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER1_BASE,                     //定时器
                 TIMER_B,                         //通道
                 0XFFFF);                         //设置重装值(分频系数)
    TimerMatchSet(TIMER1_BASE,                    //定时器
                  TIMER_B,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER1_BASE,                   //定时器
                   TIMER_CAPB_MATCH);             //中断类型
    TimerEnable(TIMER1_BASE,                      //定时器
                TIMER_B);                         //定时器启动
//初始化Timer2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); //使能设备时钟
    TimerConfigure(TIMER2_BASE,
                   TIMER_CFG_SPLIT_PAIR|          //TIMER2分成两个16bit 计数器
                   TIMER_CFG_A_CAP_COUNT|         //Timer1-A 边沿计数模式
                   TIMER_CFG_B_CAP_COUNT);        //Timer1-B 边沿计数模式
//初始化Timer2-A 标准表有功高频输入
    TimerControlStall(TIMER2_BASE,
                      TIMER_A,
                      true);
    TimerControlEvent(TIMER2_BASE,                //定时器
                      TIMER_A,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER2_BASE,                     //定时器
                 TIMER_A,                         //通道
                 0xFFFF);                         //设置重装值(分频系数)
    TimerMatchSet(TIMER2_BASE,                    //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER2_BASE,                   //定时器
                   TIMER_CAPA_MATCH);             //中断类型
    TimerEnable(TIMER2_BASE,                      //定时器
                TIMER_A);                         //定时器启动
//初始化Timer2-B 标准表无功高频输入
    TimerControlStall(TIMER2_BASE,
                      TIMER_B,
                      true);
    TimerControlEvent(TIMER2_BASE,                //定时器
                      TIMER_B,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER2_BASE,                     //定时器
                 TIMER_B,                         //通道
                 0xFFFF);                         //设置重装值(分频系数)
    TimerMatchSet(TIMER2_BASE,                    //定时器
                  TIMER_B,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER2_BASE,                   //定时器
                   TIMER_CAPB_MATCH);             //中断类型
    TimerEnable(TIMER2_BASE,                      //定时器
                TIMER_B);                         //定时器启动
//初始化Timer3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);	//使能设备时钟
    TimerConfigure(TIMER3_BASE,
                   TIMER_CFG_SPLIT_PAIR|          //TIMER3分成两个16bit 计数器
                   TIMER_CFG_A_CAP_COUNT|         //Timer3-A 边沿计数模式	CCP0输入
                   TIMER_CFG_B_CAP_COUNT);        //Timer3-B 边沿计数模式	CCP1输入
//初始化Timer3-A 被检表时钟脉冲                   //设置触发沿为下降沿
    TimerControlStall(TIMER3_BASE,
				                  TIMER_A,
																						true); 
    TimerControlEvent(TIMER3_BASE,                //定时器
                      TIMER_A,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER3_BASE,                     //定时器
                 TIMER_A,                         //通道
                 1);                              //设置重装值(分频系数)
    TimerMatchSet(TIMER3_BASE,                    //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER3_BASE,                   //定时器
                   TIMER_CAPA_MATCH);             //中断类型
    TimerEnable(TIMER3_BASE,                      //定时器
                TIMER_A);                         //定时器启动 
//初始化Timer3-B 被检表正向有功脉冲               //设置触发沿为下降沿
    TimerControlStall(TIMER3_BASE, 
				                  TIMER_B,
																						true); 
    TimerControlEvent(TIMER3_BASE,                //定时器
                      TIMER_B,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER3_BASE,                     //定时器
                 TIMER_B,                         //通道
                 1);                              //设置重装值(分频系数)
    TimerMatchSet(TIMER3_BASE,                    //定时器
                  TIMER_B,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER3_BASE,                   //定时器
                   TIMER_CAPB_MATCH);             //中断类型
    TimerEnable(TIMER3_BASE,                      //定时器
                TIMER_B);                         //定时器启动                                   
//初始化Timer4
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4); //使能设备时钟	
    TimerConfigure(TIMER4_BASE,
                   TIMER_CFG_SPLIT_PAIR|          //TIMER4分成两个16bit 计数器
                   TIMER_CFG_A_CAP_COUNT|         //Timer4-A 边沿计数模式	CCP0输入
                   TIMER_CFG_B_CAP_COUNT);        //Timer4-B 边沿计数模式	CCP1输入   
//初始化Timer4-A 被检表正向无功脉冲               //设置触发沿为下降沿
    TimerControlStall(TIMER4_BASE,
				                  TIMER_A,
																						true); 
    TimerControlEvent(TIMER4_BASE,                //定时器
                      TIMER_A,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER4_BASE,                     //定时器
                 TIMER_A,                         //通道
                 1);                              //设置重装值(分频系数)
    TimerMatchSet(TIMER4_BASE,                    //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER4_BASE,                   //定时器
                   TIMER_CAPA_MATCH);             //中断类型
    TimerEnable(TIMER4_BASE,                      //定时器
                TIMER_A);                         //定时器启动
//初始化Timer4-B 被检表反向有功脉冲               //设置触发沿为下降沿
    TimerControlStall(TIMER4_BASE,
				                  TIMER_B,
																						true); 
    TimerControlEvent(TIMER4_BASE,                //定时器
                      TIMER_B,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER4_BASE,                     //定时器
                 TIMER_B,                         //通道
                 1);                              //设置重装值(分频系数)
    TimerMatchSet(TIMER4_BASE,                    //定时器
                  TIMER_B,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER4_BASE,                   //定时器
                   TIMER_CAPB_MATCH);             //中断类型
    TimerEnable(TIMER4_BASE,                      //定时器
                TIMER_B);                         //定时器启动                    
//初始化Timer5-A 被检表反向无功脉冲    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5); //使能设备时钟
    TimerConfigure(TIMER5_BASE,
                   TIMER_CFG_SPLIT_PAIR|          //TIMER5分成两个16bit 计数器
                   TIMER_CFG_A_CAP_COUNT);        //Timer5-A 边沿计数模式	CCP0输入   
//初始化Timer5-A 被检表反向无功脉冲               //设置触发沿为下降沿
    TimerControlStall(TIMER5_BASE,
				                  TIMER_A,
																						true); 
    TimerControlEvent(TIMER5_BASE,                //定时器
                      TIMER_A,                    //通道 
                      TIMER_EVENT_NEG_EDGE);      //脉冲输入方式
    TimerLoadSet(TIMER5_BASE,                     //定时器
                 TIMER_A,                         //通道
                 1);                              //设置重装值(分频系数)
    TimerMatchSet(TIMER5_BASE,                    //定时器
                  TIMER_A,                        //通道
                  0);                             //设置比较值
    TimerIntEnable(TIMER5_BASE,                   //定时器
                   TIMER_CAPA_MATCH);             //中断类型
    TimerEnable(TIMER5_BASE,                      //定时器
                TIMER_A);                         //定时器启动    
}
/*****************************************************************************
* 初始化CAN0接口
*****************************************************************************/

void Init_CAN0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);  //使能CAN设备SYCTL_RCGC(使能时钟) 
    CANDisable(CAN0);                            //进入初始化模式
    CANInit(CAN0);                               //清除CAN MSG RAM
    CANBitRateSet(CAN0,Sysclk,CANBAUD);          //设置波特率
    CANIntEnable(CAN0, CAN_INT_MASTER |	         //使能中断
                       CAN_INT_ERROR );          //使能错误计数中断 Boff Ewarn 
    Set_MsgRam(CAN0);                            //初始化CAN网络 设置接收报文对象过滤
    CANEnable(CAN0);                             
    CANStatusGet(CAN0, CAN_STS_CONTROL);         //清除状态中断
}
/*****************************************************************************
* 初始化CAN1接口
*****************************************************************************/

void Init_CAN1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);  //使能CAN设备SYCTL_RCGC(使能时钟) 
    CANDisable(CAN1);                            //进入初始化模式
    CANInit(CAN1);                               //清除CAN MSG RAM
    CANBitRateSet(CAN1,Sysclk,CANBAUD);          //设置波特率
    CANIntEnable(CAN1, CAN_INT_MASTER |	         //使能中断
                       CAN_INT_ERROR );          //使能错误计数中断 Boff Ewarn 
    Set_CAN1MsgRam(CAN1);                        //初始化CAN网络 设置接收报文对象过滤
	   CANEnable(CAN1);
    CANStatusGet(CAN1, CAN_STS_CONTROL);         //清除状态中断
}
/*****************************************************************************
* 初始化看门狗 
*****************************************************************************/
void Init_Wdt(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);             //使能看门狗设备SYCTL_RCGC(使能时钟) .
    WatchdogUnlock(WATCHDOG0);                               //解锁
    WatchdogIntClear(WATCHDOG0);                             //清除看门狗中断
    WatchdogReloadSet(WATCHDOG0,(Sysclk/1000)*WATCH_TIME);   //设置看门狗周期
    WatchdogResetEnable(WATCHDOG0);                          //看门狗溢出复位使能
    WatchdogStallEnable(WATCHDOG0);
    WatchdogEnable(WATCHDOG0);                               //看门狗中断启动
}
/*****************************************************************************
* 初始化ADC
*****************************************************************************/
void Init_Adc(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);               //使能ADC设备SYCTL_RCGC(使能时钟) 
	   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能端口E时钟
	   GPIOPinTypeADC(AIN0_GPIO,AIN0);         //初始化AIN0引脚
	   SysCtlADCSpeedSet(SYSCTL_ADCSPEED_125KSPS);		             //设置ADC采样速率
//    ADCSequenceDisable(ADC0_BASE,0);                          //禁能采样序列0
    ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);//设置为软件触发 最高优先级    
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_CH0|       //采样序列0 采样ADC2   ADC_CTL_CH2
                                           ADC_CTL_IE|        //采样序列0 中断使能
                                           ADC_CTL_END);      //采样序列0 最后一个采样
                                 
    ADCHardwareOversampleConfigure(ADC0_BASE,64);             //硬件平均64次采样取平均
	   ADCIntEnable(ADC0_BASE,0);								                        //使能ADC sequence0中断
    ADCSequenceEnable(ADC0_BASE,0);							                    //使能采样序列0
	   ADCProcessorTrigger(ADC0_BASE,0);	
}

/*****************************************************************************
* 初始化中断 
* 中断优先级 0最高 255最低
* 标准晶振频率较高 中断有效级最高 防止丢脉冲
*****************************************************************************/
void Init_Int(void)
{  
    IntPrioritySet(FAULT_SYSTICK,INT_PRIORITY_5);   //系统节拍发生器 中断优先级  5 系统定时器中断
    IntPrioritySet(INT_UART0,INT_PRIORITY_6);       //UART0          中断优先级  6 UART0中断
    IntPrioritySet(INT_UART1,INT_PRIORITY_6);       //UART1          中断优先级  6 UART1中断
	   IntPrioritySet(INT_UART3,INT_PRIORITY_6);       //UART3          中断优先级  6 UART3中断
	   IntPrioritySet(INT_UART5,INT_PRIORITY_6);       //UART5          中断优先级  6 UART5中断
	   IntPrioritySet(INT_UART6,INT_PRIORITY_6);       //UART6          中断优先级  6 UART6中断
    IntPrioritySet(INT_TIMER0A,INT_PRIORITY_2);     //TIMER0-A       中断优先级  2 备用高频信号1计数中断
    IntPrioritySet(INT_TIMER0B,INT_PRIORITY_2);     //TIMER0-B       中断优先级  2 备用高频信号2计数中断
    IntPrioritySet(INT_TIMER1A,INT_PRIORITY_2);     //TIMER1-A       中断优先级  2 备用高频信号3计数中断	
    IntPrioritySet(INT_TIMER1B,INT_PRIORITY_0);     //TIMER1-B       中断优先级  0 标准晶振高频计数溢出中断
    IntPrioritySet(INT_TIMER2A,INT_PRIORITY_2);     //TIMER2-A       中断优先级  2 标准表有功高频计数溢出中断
    IntPrioritySet(INT_TIMER2B,INT_PRIORITY_2);     //TIMER2-B       中断优先级  2 标准表无功高频计数溢出中断	
    IntPrioritySet(INT_TIMER3A,INT_PRIORITY_1);     //TIMER3-A       中断优先级  1 被检表时钟脉冲计数溢出中断
	   IntPrioritySet(INT_TIMER3B,INT_PRIORITY_3);     //TIMER3-B       中断优先级  3 被检表有功电能脉冲计数中断
    IntPrioritySet(INT_TIMER4A,INT_PRIORITY_3);     //TIMER4-A       中断优先级  3 被检表无功电能脉冲计数中断
    IntPrioritySet(INT_TIMER4B,INT_PRIORITY_3);     //TIMER4-B       中断优先级  3 被检表反向有功电能脉冲计数中断
    IntPrioritySet(INT_TIMER5A,INT_PRIORITY_3);     //TIMER5-A       中断优先级  3 被检表反向无功电能脉冲计数中断
	   IntPrioritySet(INT_CAN0,INT_PRIORITY_4);        //CAN            中断优先级  4 CAN 中断
//    IntPrioritySet(INT_WATCHDOG,INT_PRIORITY_0);  //看门狗         中断优先级  0 看门狗 中断
    IntPrioritySet(INT_GPIOA,INT_PRIORITY_5);       //GPIOA          中断优先级  5 按键中断
    IntPrioritySet(INT_GPIOM,INT_PRIORITY_3);       //GPIOM          中断优先级  3 光电头脉冲 需量脉冲 时段投切脉冲中断 
	   IntPrioritySet(INT_GPION,INT_PRIORITY_5);       //GPION          中断优先级  5 内置跳闸信号中断
	   IntPrioritySet(INT_ADC0SS0,INT_PRIORITY_7);     //ADC0SS0        中断优先级  7 ADC0转换中断
	
    IntEnable(FAULT_SYSTICK);   //系统节拍发生器NVIC中断使能  系统定时器中断
    IntEnable(INT_UART0);       //UART0         NVIC中断使能  UART0中断
    IntEnable(INT_UART1);       //UART1         NVIC中断使能  UART1中断
				IntEnable(INT_UART3);       //UART3         NVIC中断使能  UART3中断
				IntEnable(INT_UART5);       //UART5         NVIC中断使能  UART5中断
				IntEnable(INT_UART6);       //UART6         NVIC中断使能  UART6中断
    IntEnable(INT_TIMER0A);     //TIMER0-A      NVIC中断使能  备用高频信号1计数中断
    IntEnable(INT_TIMER0B);     //TIMER0-B      NVIC中断使能  备用高频信号2计数中断
    IntEnable(INT_TIMER1A);     //TIMER1-A      NVIC中断使能  备用高频信号3计数中断
    IntEnable(INT_TIMER1B);     //TIMER1-B      NVIC中断使能  标准晶振高频计数溢出中断
    IntEnable(INT_TIMER2A);     //TIMER2-A      NVIC中断使能  标准表有功高频计数溢出中断
    IntEnable(INT_TIMER2B);     //TIMER2-B      NVIC中断使能  标准表无功高频计数溢出中断
				IntEnable(INT_TIMER3A);     //TIMER3-A      NVIC中断使能  时钟脉冲计数溢出中断
				IntEnable(INT_TIMER3B);     //TIMER3-B      NVIC中断使能  被检表有功电能脉冲计数中断
				IntEnable(INT_TIMER4A);     //TIMER4-A      NVIC中断使能  被检表无功电能脉冲计数中断
				IntEnable(INT_TIMER4B);     //TIMER4-B      NVIC中断使能  被检表反向有功电能脉冲计数中断
				IntEnable(INT_TIMER5A);     //TIMER5-A      NVIC中断使能  被检表反向无功电能脉冲计数中断
    IntEnable(INT_CAN0);        //CAN           NVIC中断使能  CAN 中断
//    IntEnable(INT_WATCHDOG);   //看门狗        NVIC中断使能
    IntEnable(INT_GPIOA);       //GPIOA         NVIC中断使能  GPIOA中断 按键输入
    IntEnable(INT_GPIOM);       //GPIOM         NVIC中断使能  GPIOM中断 光电头脉冲 需量脉冲 时段投切 合闸脉冲
				IntEnable(INT_GPION);       //GPION         NVIC中断使能  GPION中断 内置跳闸信号输入
				IntEnable(INT_ADC0SS0);     //ADC0SSS0      NVIC中断使能  ADC0转换中断
}
/*****************************************************************************
* RAM和变量初始化
*****************************************************************************/
void Init_Ram(void)
{   
    Read_Mtr_Numb();                   //读表号
    Check_SMtr_Tab_Sts();              //读配置
    memset(Disp_Buf,DISP_BLANK,8);     //显示同一设置 清显示
    Disp_Choose=DISP_TEST_DATA;        //默认显示试验数据
    SINGLE_OR_THREE=SINGLE;            //默认单相台
    DISP_HL_LVL=1;                     //显示高电平时间
    CLK_MEA_TIME =10;                  //设置时钟频率测量时间 默认20秒 单位:s
    CLK_FREQ_SET =1.0;                 //默认时钟频率为1HZ
    CLK_FREQ_INT =1.0;                 //默认规格化时钟频率为1HZ
    CLK_ACT_FREQ=1.0;                  //时钟频率默认实际值1.0
    CLK_DAY_ERR=0.0;                   //日计时误差
    VERIFY_ENG_Kwh=0.01;               //校核常数走字度数 默认0.1kwh
    sprintf((char*)TEMP_STR,           //校核常数走字度数
            "%6.2f",
            VERIFY_ENG_Kwh);           //校核常数走字电能ASC表示
    Fill_Space_StrCpy(TEMP_STR,        //待拷贝的字符串
                      VERIFY_ENG_ASC,  //待写入的字符串
                      8);              //字符串最大长度                 
    VERIFY_PLS_Set=360;                //校核常数走字脉冲数
    ZZ_PLS_Set=360;                    //定脉冲走字脉冲数
    PZBZ=4;                            //默认的盘转比值
    PZZS=3;                            //默认的盘转转数
    PZ_STD_CNT=PZZS*PZBZ;              //标准脉冲数
    CURRENT_VERIFY_PLS=360;            //当前走字脉冲数
    XUL_RELOAD_TIME=1;                 //需量脉冲测量次数
    XUL_RELOAD_Cnt=1;                  //需量脉冲计数
    WORK_MODE = CAL_ENG_ERR_M;         //开机默认为误差校验状态
    ENG_CLK_CH = DZ_PLS;               //开机默认电子脉冲
    CYCLE_MEA_SEL=EPLS_T;              //默认测量电能脉冲周期
    CYCLE_MEA_ID=0;	                   //默认测量电能脉冲周期
    FIRST_ENG_PLS=1;                   //首个电能脉冲标志
    FIRST_CLK_PLS=1;                   //首个时钟脉冲标志
    FIRST_XUL_PLS=1;                   //首个需量脉冲标志
    NEW_CMD=1;                         //首次进入试验标志
    SY_MODE=VLOSS_1;                   //失压模式
    SY_PHASE=ALL_PHASE;                //默认三相电压失压
    DIVIDE_Coef=1;                     //默认不分频
    GP_CLK_SET=10000;                  //设置检高频脉冲数
    MTR_ENG_CST=3600.0;                //默认被检表低频常数
    MTR_MIN_CST=3600.0;                //默认最小电表常数
    STD_ENG_CST=36000000.0;            //标准表高频常数
    SET_N=3;                           //默认校验圈数 1圈
    Cal_Clk_Reload(0);                 //计算时钟频率重装值和重装次数 不重装 226us
    CLK_RELOAD_VAL_O=CLK_RELOAD_VAL_N; //分频值保存
    CLK_RELOAD_TIME_O=CLK_RELOAD_TIME_N;//次数保存
    Cal_Gp_Relaod();                   //计算高频校验脉冲 重装值和重装次数
    Cal_STD_ENG_CNT_VAL();             //计算标准值和圈数归一化
    CAN_STS.BYTE =0;                   //清除错误状态标志
    CAN_MSG_IPtr=&CAN_SDATA_MSG_IBUF[0];//初始化接收报文处理指针
    MTR_PLUG=1;                        //挂表标志 0:不挂表 1:挂表 
    Disp_Code_Mode=LED_SEND_DATA_CODE2;//显示模式 方式0译码
    CAN_MSG_OPtr=&CAN_SDATA_MSG_OBUF[0];//初始化发送报文处理指针
    Update_N_Buf();                    //更新圈数区 
    Update_Mtr_Num();                  //更新表位号区
    MFClk_Mode=ALONE_PLS;              //多功能脉冲输入方式 联合or独立
    MFClk_Type=SZCLK_PLS;              //多功能脉冲输入类型 
    NO_STD_CLK=1;                      //默认没有标准脉冲
    NY_RESULT=NY_UNKW;                 //耐压结果未知 
    PLSGD=GD_E;                        //判断脉冲输出类型是否改变
    PLSHC_MODE=HC_2;                   //脉冲合成方式 默认有功合成 无功合成
    PLS_QUAD=PA_PLS;                   //脉冲象限     默认输入有功
    CAN_RX_OVTimer=(u16)Timer_1ms;     //初始化CAN超时定时器
    CAN_ERR=0;                         //
    CAN_SEND_DTIME=(Mtr_Numb_ID%10);   //10个为一组
    CAN_SEND_DELAY=CAN_SEND_DTIME;     //延时 避开
    WORK_Timer=0;                      //进入工作模式定时器
    Init_DEVICE_IO();                  //初始化电能脉冲 多功能脉冲 合闸脉冲 IO口
    GPIOIntClear(GPIOA,                //端口 清除端口中断
                 0xFF);	
    GPIOIntClear(GPIOE,                //端口 清除端口中断
                 0xFF);	
    GPIOIntClear(GPIOF,                //端口 清除端口中断
                 0xFF);	
    GPIOIntClear(GPIOH,                //端口 清除端口中断
                 0xFF);	
    KEY_INT_REEN=1;                    //按键中断使能
    KEY_Timer=(u8)Timer_1ms;           //按键中断延时 200mS后开按键中断
    Disp_Timer=(Timer_8ms-DISP_TIME);  //更新显示
    PLL_CHK_Timer=0;                   //锁相环检查定时
    Disp_En_Timer=0;                   //显示刷新
    XUL_MEA_CTL=MEA_ORDER;             //开机允许测量需量周期
    CLK_MEA_CTL=MEA_ORDER;             //开机允许测量时钟频率
    DIS_UNSTD_PLS=0;                   //小显示显示试验结论“bhg”标志
    DIS_STD_PLS=0;                     //小显示显示试验结论“hg”标志
    ENT_XY_PLS=0;                      //小显示退出显示试验结论标志
    HC165_TDATA=DEFAULT_HC165;         //HC165默认数据
    *(u16*)&HC165_DATA=DEFAULT_HC165;  //HC165默认数据
//    *(u16*)&CD4094_DATA=0;             //CD4094数据	    需要初始化

    Pulse1_RUN_CTL=0;                  //脉冲1输出控制；0表示停止输出；1表示开始输出
    Pulse2_RUN_CTL=0;                  //脉冲2输出控制；0表示停止输出；1表示开始输出
    Pulse3_RUN_CTL=0;                  //脉冲3输出控制；0表示停止输出；1表示开始输出
    Pulse4_RUN_CTL=0;                  //脉冲4输出控制；0表示停止输出；1表示开始输出
    Pulse5_RUN_CTL=0;                  //脉冲5输出控制；0表示停止输出；1表示开始输出
    Pulse6_RUN_CTL=0;                  //脉冲6输出控制；0表示停止输出；1表示开始输出
				Pulse7_RUN_CTL=0;                  //脉冲7输出控制；0表示停止输出；1表示开始输出
    Pulse8_RUN_CTL=0;                  //脉冲8输出控制；0表示停止输出；1表示开始输出
    PulOut1_Count=0;                   //脉冲1输出计数
    PulOut2_Count=0;                   //脉冲2输出计数
    PulOut3_Count=0;                   //脉冲3输出计数
    PulOut4_Count=0;                   //脉冲4输出计数
    PulOut5_Count=0;                   //脉冲5输出计数
    PulOut6_Count=0;                   //脉冲6输出计数				
				PulOut7_Count=0;                   //脉冲7输出计数
    PulOut8_Count=0;                   //脉冲8输出计数				
    PulOut1_Count_Set=30000;           //默认脉冲输出个数；
    PulOut2_Count_Set=30000;           //默认脉冲输出个数；
    PulOut3_Count_Set=30000;           //默认脉冲输出个数；
    PulOut4_Count_Set=30000;           //默认脉冲输出个数；
    PulOut5_Count_Set=30000;           //默认脉冲输出个数；				
    PulOut6_Count_Set=30000;           //默认脉冲输出个数；
    PulOut7_Count_Set=30000;           //默认脉冲输出个数；				
    PulOut8_Count_Set=30000;           //默认脉冲输出个数；

    WZHZ_DATA[0]='0';                  //合闸常开
    WZHZ_DATA[1]='0';                  //合闸常闭
    DXTZ=0;                            //费控表默认双触点
    TZEN=1;                            //跳闸发送使能
    MBJEN=1;                           //表报警回送使能
    Pwr_Phase='0';                     //默认不测功耗
    PLL_CHK_Timer=Timer_8ms;           //定时到重启定时
    NEW_PLL_Ref_Timer=0;               //锁相环参考定时器
    OLD_PLL_Ref_Timer=0;               //锁相环参考定时器
    PLL_ERR_Cnt=0;                     //锁相环错误计数
    REF_JZ_INT=0;                      //标准晶振中断标志
    Set_Uclop=MTR_UCL;                 //默认电压接入
    RST_FLAG=1;                        //开机复位标志
    WIRE_TYPE=WIRE_P4;                 //默认接线方式为三相四线
    MTR_RDY_DATA=MTR_NO;               //默认无表
    LIGHT_CTL_PC=FALSE;                //默认CPU直接控制 
    EXT_WDFEED_CNT=0;                  //外部看门狗喂狗计数
    EXT_WD_Timer=Timer_8ms;            //定时到重启定时
    MTR_MOD=Smart_Meter;               //默认校验电表
    MTYPE=NATIONAL;                    //默认国网表		
			//	Uin_Iin_Pr();                           //电压电流接入标志
}
