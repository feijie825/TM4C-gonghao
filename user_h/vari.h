/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : vari.h
;* Author             : 张力阵
;* 变量声明
*******************************************************************************/
#include "define.h"
//CAN错误状态结构体定义	与 CANStatus定义一致
typedef struct
{
    u32  LEC  :3; //最新的总线错误代码
    u32  TxOK :1; //报文成功发送标志
    u32  RxOK :1; //报文成功接收标志 与过滤结果无关
    u32  EPass:1; //错误认可状态 1 发送或接收错误>127(认可门限)
    u32  EWarn:1; //错误警告状态 1 发送或接收错误>96(警告门限)
    u32  Boff :1; //总线脱离状态 1 模块CAN总线脱离
}CAN_STS_S;
//CAN错误状态共用体定义 方便操作
typedef union
{
    u8        BYTE;
    CAN_STS_S BIT;
}CAN_STS_U;
//控制域结构体定义
typedef struct
{
    u32	 LEN          :4;     //数据长度 0-8 4bit
    u32  TX_INT_EN    :1;     //发送中断允许位
    u32  RX_INT_EN    :1;     //接收中断允许位
    u32  EXD_ID       :1;     //扩展帧标志
    u32  ID_FLT_EN    :1;     //ID滤波使能位
    u32  DIR_FLT_EN   :1;     //方向位滤波使能位
    u32  EXT_FLT_EN   :1;     //扩展位滤波使能位
    u32  RMT_FRM      :1;     //远程帧标志 
    u32  NEW_DATA     :1;     //新数据标志
    u32  DATA_LOST    :1;     //数据丢失标志
    u32  RESV         :3;     //保留
    u32  IDx          :5;     //报文对象编号 MSG RAM ID
}CTL_S;
//控制域联合体定义 方便操作
typedef union
{
    u32  WORD;
    CTL_S BIT;       
}CTL_U;
//CAN0 ID结构体定义
typedef struct
{
    u32  IDX   :3  ;//索引长数据串编号
    u32  END   :1  ;//长数据结束标志
    u32  TYPE  :4  ;//长数据类型标志
    u32  MNUM  :8  ;//表编号 0 广播命令 1~255 表位号
    u32  DIR   :1  ;//方向位 DIR=0 总控->单元 DIR=1 单元->总控
    u32  CMD   :11 ;//命令区 11bit 0~2031 b0~b11111101111
    u32  EXD   :1  ;//ID中帧标识 0 标准帧 1 扩展帧 用于控制发送优先级
    u32  RESV  :2  ;//保留
}ID_S;
//CAN1 ID结构体定义
typedef struct
{
    u32  DEV   :8  ;//设备类型号
    u32  MNUM  :8  ;//表编号 0 广播命令 1~255 表位号
    u32  CMD   :11 ;//命令区 11bit 0~2031 b0~b11111101111
	   u32  DIR   :1  ;//方向位 DIR=0 总控->单元 DIR=1 单元->总控
    u32  EXD   :1  ;//ID中帧标识 0 标准帧 1 扩展帧 用于控制发送优先级
    u32  RESV  :3  ;//保留	
}CAN1_ID_S;
//CAN0 ID联合体定义
typedef union
{
    u32  WORD;
    ID_S BIT;       
}ID_U;
//CAN1 ID联合体定义
typedef union
{
    u32       WORD;
    CAN1_ID_S BIT;	
}CAN1_ID_U;
//CAN0
typedef struct
{
    u32  IDX   :3  ;//索引长数据串编号
    u32  END   :1  ;//长数据结束标志
    u32  TYPE  :4  ;//长数据类型标志
    u32  MNUM  :8  ;//表编号 0 广播命令 1~255 表位号
    u32  DIR   :1  ;//方向位 DIR=0 总控->单元 DIR=1 单元->总控
    u32  CMD   :11 ;//命令区 11bit 0~2031 b0~b11111101111
    u32  EXD   :1  ;//ID中帧标识 0 标准帧 1 扩展帧 用于控制发送优先级
    u32  RESV  :3  ;//保留
}MASK_S;
//CAN1
typedef struct
{
    u32  DEV   :8  ;//设备类型号
    u32  MNUM  :8  ;//表编号 0 广播命令 1~255 表位号
    u32  CMD   :11 ;//命令区 11bit 0~2031 b0~b11111101111
	   u32  DIR   :1  ;//方向位 DIR=0 总控->单元 DIR=1 单元->总控
    u32  EXD   :1  ;//ID中帧标识 0 标准帧 1 扩展帧 用于控制发送优先级
    u32  RESV  :3  ;//保留	  
}CAN1_MASK_S;
//CAN0
typedef union
{
    u32    WORD;
    MASK_S BIT;
}MASK_U;
//CAN1
typedef union
{
    u32         WORD;
    CAN1_MASK_S BIT;
}CAN1_MASK_U;
//报文数据结构体
typedef struct
{
    u16  DA1;
    u16  DA2;
    u16  DB1;
    u16  DB2;
}DATA_S;
//CAN报文数据共用体 方便访问
typedef union
{
    DATA_S WORD;        //
    u8     BYTE[8];	    //发送顺序 Byte[0](1) ... Byte[7] DA1.L DA1.H ... DB2.L DB2.H
}DATA_U;
//CAN0 报文接收屏蔽设置
typedef struct
{
    CTL_U  CTL;         //控制域 包含数据长度
    ID_U   ID;          //报文ID ID.31=0 扩展报文 ID.31=0 标准报文
    MASK_U IDMask;      //报文ID屏蔽
}CAN_MSG_SET;
//CAN1 报文接收屏蔽设置
typedef struct
{
    CTL_U       CTL;          //控制域 包含数据长度
    CAN1_ID_U   ID;       //报文ID ID.31=0 扩展报文 ID.31=0 标准报文
    CAN1_MASK_U IDMask; //报文ID屏蔽	
}CAN1_MSG_SET;
//CAN0 发送和接收到的报文结构体定义 短帧直接存入接收报文区
typedef struct
{
    CTL_U  CTL;         //控制域 包含数据长度
    ID_U   ID;          //报文ID ID.31=0 扩展报文 ID.31=0 标准报文
    DATA_U Data;        //报文数据
}CAN_MSG;
//CAN1 发送和接收到的报文结构体定义 短帧直接存入接收报文区
typedef struct
{
	   CTL_U     CTL;          //控制域 包含数据长度
	   CAN1_ID_U ID;       //报文ID
	   DATA_U    Data;        //报文数据
}CAN1_MSG;
//CAN长数据处理结构体
typedef struct
{
    u8   *BUF;    //指向长数据存放位置  如:指向Com0_Obuf
    u16  *HEAD;   //指向数据头  指针	如:指向Com0_OHead
    u16  *TAIL;	  //指向数据尾  尾指针	如:指向Com0_OTail
    u8   *STS;    //指向缓冲区状态 指针	如:指向Com0_Tx_Sts
}CAN_LMSG_PR;


typedef struct
{
    u16   Flag;          //保存标志
    u16   LED_NUM;       //LED 数码管位数
}SAVE_S;
typedef struct
{
	   u16 YK_SIG1   :1;           //BIT0  负控遥控信号1 
	   u16 YK_SIG2   :1;           //BIT1  负控遥控信号2 
	   u16 YK_SIG3   :1;           //BIT2  负控遥控信号2 
	   u16 YK_SIG4   :1;           //BIT3  负控遥控信号2 
	   u16 YK_SIG5   :1;           //BIT4  负控遥控信号2 
	   u16 YK_SIG6   :1;           //BIT5  负控遥控信号2 
	   u16 YK_SIG7   :1;           //BIT6  负控遥控信号2 
	   u16 YK_SIG8   :1;           //BIT7  负控遥控信号2	
    u16 MTR_BJ    :1;           //BIT8  电能表报警信号
    u16 JDQ_CLS   :1;           //BIT9  继电器常闭
    u16 JDQ_OPN   :1;           //BIT10 继电器常开
    u16 GZ_STS    :1;           //BIT11 续流继电器故障状态 	
    u16 TZ_STS    :1;           //BIT12 跳闸状态
    u16 HGQ_BJ    :1;           //BIT13 三相并线台用
    u16 JCXH      :1;           //BIT14 耐压漏电流击穿信号
    u16 MTR_ON    :1;           //BIT15 表放好标志  1有效    	 

}HC165_DT;
typedef struct
{
    u16  TZZS      :1;          //BIT0 跳闸指示信号
    u16  XLJDQ_OPN :1;          //BIT1 续流继电器控制信号 续流继电器断开
    u16  XLJDQ_CLS :1;          //BIT2 续流继电器控制信号 续流继电器吸合 电流旁路
    u16  P3_P4     :1;          //BIT3 功耗测试时P3/P4选择
    u16  E_RED     :1;          //BIT4 误差状态指示红灯控制
    u16  E_GRN     :1;          //BIT5 误差状态指示绿灯控制
    u16  N_GRN     :1; 	        //BIT6 耐压状态指示绿灯控制
    u16  N_RED     :1;          //BIT7 耐压状态指示红灯控制
    u16  YX_SIG7   :1;          //BIT8 负控遥信信号7
    u16  COM0SEL   :1;          //BIT9 单片机USART0通讯接口选择
    u16  COM1SEL   :1;          //BIT10单片机USART1通讯接口选择
    u16  DOOR_SIG  :1;          //BIT11门接点信号
    u16  YX_SIG6   :1;          //BIT12遥信信号5
    u16  YX_SIG5   :1;          //BIT13遥信信号6
    u16	 YX_SIG2   :1;          //BIT14遥信信号2
    u16  YX_SIG1   :1;          //BIT15遥信信号1
}CD4094_DT;
typedef struct
{
    vu32 Flag;                //标志
    vu32 Numb;                //表位号
}Mtr_Numb;

typedef struct 
{
    u32 Break     :1;        //发送终止位
    u32 Parity_En :1;        //校验允许 
    u32 Even      :1;        //偶校验选择
    u32 Stop2     :1;        //2 stop bit
    u32 Fifo_En   :1;        //Fifo 使能
    u32 Data_Len  :2;        //数据位长度 0~5 3~8
    u32 Parity_01 :1;        //奇偶校验固定为0 或1 parity_En=1 (Even=1 校验位为1,Even=0 校验位为0)
    u32 Baud      :18;       //波特率
}UART_PARA;
typedef struct
{
    u32 BAUD     :18;        //波特率
    u32 LEN      :4;         //数据长度 5 6 7 8
    u32 STOP     :2;         //停止位 1 2 3=1.5   
    u32 PARITY   :3;         //校验位 0:'N' 1:'O' 2:'E' 3:'M' 4:'S'
}UART_SET;
typedef struct
{
    u8  NUMB;       //终端输出脉冲的序号，0代表所有脉冲
    u8  MOD;        //终端输出脉冲的模式，0代表低电平脉冲
	   u16 CYCLE;      //终端输出脉冲的周期
	   u16 WIDTH;      //终端输出脉冲的宽度
}PULSE_SET;
typedef struct
{
	   u8  NUMB;        //终端输出脉冲的序号,0代表所有脉冲
	   u8  MOD;         //终端输出脉冲控制
	   u16 QANTY;       //终端输出脉冲的数量,最大65535
}PULNUM_SET;
//工作模式定义
typedef enum
{
    CAL_ENG_ERR_M   = 0,       //计算被检表电能脉冲误差
    MTR_PLUG_M      = 1,       //选择挂表状态
    CATCH_HB_M      = 2,       //对黑斑
    START_STOP_M    = 3,       //监视电能脉冲计数 启动潜动模式下脉冲计数
    VERIFY_READY_M  = 4,       //准备开始走字 继电器吸合
    VERIFY_START_M  = 5,       //进入走字试验
    VOLTAGE_DOWN_M  = 6,       //进入电压跌落试验
    MEASURE_ENG_M   = 7,       //进入计电能试验 计量被检表电能
    PANZHUAN_ERR_M  = 8,       //进入盘转误差试验
    MEA_CST_M       = 9,       //常数测试试验
    MEA_POWER_D_M   =10,       //功耗测量试验
    MEA_ENG_DUTY_M  =11,       //测量设定脉冲周期和占空比
    PULSE_ZZ_M      =12,       //定脉冲走字试验
    NYSY_M          =13,       //耐压试验 并记录耐压状态下脉冲状态
    GDGZ_M          =14,       //接地故障试验
    ZBTX_M          =15,       //载波通信试验
}MODE;
//CAN长数据处理结构体
typedef struct
{
    vu8 *Ptr;                  //指向缓冲区状态 指针	如:指向Com0_Tx_Sts
}vu8_Ptr;

//CAN长数据处理结构体
typedef struct
{
    u32 *Ptr;                  //指向缓冲区状态 指针	如:指向Com0_Tx_Sts
}u32_Ptr;
//ADC数据
typedef struct
{
    u16 Data          :12;	  //数据
    u16 New_Data      :1;    //有新数据
    u16 Trig          :1;    //触发标志
}Adc_Dat;

//位定义 定义字变量 包含32个位变量	4字节对齐 在LM3S2139.h 中定义
extern u32         SysRCC_CFG;                     //系统RCC配置

extern u8          MTYPE;                          //表类型 'N' 国网表  'S' 南网表
extern u8          EXT_WDFEED_CNT;                 //外部看门狗喂狗计数

extern SAVE_S      SOLID_CFG;                      //固化的配置结构体

extern u8          MTR_RDY_DATA;                   //表放置状态数据 '0' 没表 '1'有表
extern u8          Current_Save_Tab;               //当前保存值所在表格
extern u8          Save_Tab_Sts[SAVE_TAB_NUMB];    //表位号状态表格
extern u8          PLL_ERR_Cnt;                    //锁相环错误计数
extern u8          Set_Uclop;                      //设置电压接入
extern u8          Pwr_Phase;                      //功耗相
extern u8          DXTZ;                           //单相费控表跳闸继电器状态
extern u8          MBJ_DATA;                       //表报警数据
extern u8          LUNCI1_DATA[2];                 //轮次1信号 [0]常开 [1]常闭 
extern u8          LUNCI2_DATA[2];                 //轮次2信号 [0]常开 [1]常闭 
extern u8          LUNCI3_DATA[2];                 //轮次3信号 [0]常开 [1]常闭 
extern u8          LUNCI4_DATA[2];                 //轮次4信号 [0]常开 [1]常闭 
extern u8          LUNCI1_SEND_COUNT;              //轮次1信号发送次数计数
extern u8          LUNCI2_SEND_COUNT;              //轮次2信号发送次数计数
extern u8          LUNCI3_SEND_COUNT;              //轮次3信号发送次数计数
extern u8          LUNCI4_SEND_COUNT;              //轮次4信号发送次数计数
extern u8          WZHZ_DATA[2];                   //外置合闸信号 [0]常闭 [1]常开 
extern u8          GZ_DATA;                        //故障数据
extern u8          NZHZ_DATA;                      //内置合闸信号(电流旁路继电器信号)
extern HC165_DT    HC165_DATA;                     //HC165数据
extern u16         HC165_TDATA;                    //HC165临时数据
extern u16         HC165_XDATA;                    //HC165数据变化
//extern CD4094_DT   CD4094_DATA;                    //CD4094数据
extern u8          NY_RESULT;                      //耐压结果
extern u8          TIME_ASC[8];                    //时间ASC HH-MM-SS  时分秒
extern u8          PLSGD;                          //脉冲共高共低
extern u8          MFClk_Mode;                     //多功能脉冲输入方式
extern u8          MFClk_Type;                     //多功能脉冲输入类型
extern u8          PLSHC_MODE;                     //脉冲合成方式
extern u8          PLS_QUAD;                       //脉冲象限
extern u8          Disp_Choose;                    //显示选择
extern u8          Disp_Code_Mode;                 //显示译码模式
extern u16         ELEC_PLS_CNT;                   //当前电子脉冲计数
extern u8          ENG_ERR_ASC[9];                 //电能误差ASC码 最多带四位小数点 符号位 +5位数据
extern u8          CLK_FREQ_ASC[9];                //时钟频率
extern u8          DAY_ERR_ASC[9];                 //日计时误差 最多3个小数点
extern u8          XUL_TIME_ASC[9];                //需量周期ASC码 
extern u8          VERIFY_ENG_ASC[9];              //校核超时走字电能ASC表示
extern u8          CURRENT_N_ASC[2];               //当前圈数ASC码
extern u8          CURRENT_ENG_ASC[9];             //当前电能ASC码
extern u8          CURRENT_PLS_ASC[9];             //当前脉冲数ASC码
extern u8          HIGH_LVL_TIME[9];               //高电平时间 第一个字节 'H'
extern u8          LOW_LVL_TIME[9];                //低电平时间 第一个字节 'L'
extern u32         PLS_Lvl_Time[2];                //脉冲电平时间
extern u32         High_Lvl_Time_Tp;               //临时存放脉冲高电平时间
extern u32         Low_Lvl_Time_Tp;                //临时存放脉冲低电平时间
extern u8          High_Lvl_CNT;                   //高电平中断测量计数
extern u8          Low_Lvl_CNT;                    //低电平中断测量计数
extern u8          CMD_DATA;                       //单字节命令数据
extern u8          WIRE_TYPE;                      //接线方式
extern u16         DIVIDE_Coef;                    //分频系数
extern u8          CYCLE_MEA_SEL;                  //周期测量选择
extern u8          CYCLE_MEA_ID;                   //周期测量选择ID编号
extern u8          SY_CNT;                         //失压计数
extern u8          SY_PROCESS;                     //失压进程
extern u8          SY_MODE;                        //失压模式 
extern u8          SY_PHASE;                       //失压相 0:三相同时 1:UA 2:UB 3:UC
extern u8          PZZS;                           //计算盘转误差 多少圈算次误差 
extern u8          PZBZ;                           //盘转和脉冲比值
extern u8          PZ_STD_CNT;                     //盘转试验标准计数

extern u16         CUR_PZ_Cnt_Val;                 //盘转采样点   电子脉冲计数值
extern u16         PRE_PZ_Cnt_Val;                 //上一个采样点 电子脉冲计数值

extern u8          PZ_ERR_ASC[3];                  //盘转误差圈数
extern float       VERIFY_ENG_Kwh;                 //校核常数走字度数
extern u32         VERIFY_PLS_Set;                 //校核常数走字脉冲数
extern u32         CURRENT_VERIFY_PLS;             //当前校核常数走字脉冲数
extern u32         ZZ_PLS_Set;                     //走字脉冲数设置
extern u32         CURRENT_ZZ_PLS;                 //当前走字脉冲数    

extern PULSE_SET   YaoXin1_Set;                    //专变终端脉冲1设置参数
extern PULSE_SET   YaoXin2_Set;                    //专变终端脉冲2设置参数
extern PULSE_SET   YaoXin3_Set;                    //专变终端脉冲3设置参数
extern PULSE_SET   YaoXin4_Set;                    //专变终端脉冲4设置参数
extern PULSE_SET   YaoXin5_Set;                    //专变终端脉冲5设置参数
extern PULSE_SET   YaoXin6_Set;                    //专变终端脉冲6设置参数
extern PULSE_SET   YaoXin7_Set;                    //专变终端脉冲7设置参数
extern PULSE_SET   YaoXin8_Set;                    //专变终端脉冲8设置参数
//电能脉冲计数累加和总电能                                
extern u32         CURRENT_PLS_CNT;                //脉冲计数 开机后一直累加
extern float       CURRENT_ENG_KWh;                //当前电能
//检高频变量定义                                   
extern float       GP_ENG_CST;                     //被检表高频常数
extern u32         GP_CLK_SET;                     //检高频脉冲数设置
extern u32         GP_CLK_ACT;                     //实际高频计数值
extern u16         GP_RELOAD_TIME;                 //时钟频率重装次数
extern u16         GP_RELOAD_VAL;                  //时钟频率重装值
extern u16         GP_RELOAD_Cnt;                  //高频脉冲重装计数
//标准时钟脉冲计数高位
extern u32         STD_CLK_Cnt;                    //标准时钟计数高位 
//时钟脉冲频率和日计时误差计算变量定义             
extern u8          CLK_MEA_CTL;                    //时钟频率测量控制
extern u16         CLK_STB_RNG;                    //时钟脉冲稳定误差限
extern u16         CLK_RELOAD_TIME_N;              //时钟频率重装次数新值
extern u16         CLK_RELOAD_TIME_O;              //时钟频率重装次数旧值
extern u16         CLK_RELOAD_VAL_N;               //时钟频率重装值新值
extern u16         CLK_RELOAD_VAL_O;               //时钟频率重装值旧值
extern u16         CLK_RELOAD_Cnt;                 //时钟脉冲重装计数
extern u8          CLK_MEA_TIME;                   //时钟频率测量定时 单位:s 秒  10---100s之间 默认20s
extern float       CLK_FREQ_SET;                   //时钟频率设定值 默认1HZ
extern float       CLK_FREQ_INT;                   //时钟频率规格化值 默认1HZ
extern double      CLK_DAY_ERR;                    //日计时误差 
extern double      CLK_ACT_FREQ;                   //实际频率 化整后 2009.3.13 为提高精度 改为双精度浮点数
extern double      STD_CLK_VAL_ONE;                //标准时钟计算值 用于计算频率  单次中断             计算时钟频率 标准值
extern double      STD_CLK_VAL_SUM;                //标准时钟计算值 用于计算频率 CLK_RELOAD_TIME次中断 计算时钟频率 标准值
extern u32         STD_CLK_CNT_ONE;                //标准时钟计数值 每次中断                 应计的标准脉冲数
extern u32         STD_CLK_CNT_SUM;                //标准时钟计数值 总 CLK_RELOAD_TIME次中断 应计的标准脉冲数
extern u32         RAW_CLK_VAL;                    //原始时钟脉冲计数值
extern u32         CUR_CLK_VAL;                    //当前时钟脉冲计数值
extern u32         PRE_CLK_VAL;                    //上次时钟脉冲计数值

extern u32         SCLK_Cnt_OSub;                  //本次计数值 
extern u32         SCLK_Cnt_NSub;                  //上次计数值
extern u32         SCLK_Cnt_RSet_Max;              //合计数改变触发重设时钟频率最大值
extern u32         SCLK_STB_RNG;                   //时钟脉冲稳定误差限(多次)

extern u32         OCLK_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OCLK_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OCLK_Cnt_RSet_Max;              //单次计数改变触发重设时钟频率最大值
extern u32         OCLK_STB_RNG;                   //时钟脉冲稳定误差限(单次)
//需量周期变量定义
extern u8          XUL_MEA_CTL;                    //需量周期测量控制
extern float       XUL_TIME;                       //需量周期
extern u8          XUL_RELOAD_TIME;                //需量脉冲重装次数
extern u8          XUL_RELOAD_Cnt;                 //需量脉冲计数

extern u32         RAW_XUL_VAL;                    //原始需量中断标准时钟脉冲计数值
extern u32         CUR_XUL_VAL;                    //当前需量中断标准时钟脉冲计数值
extern u32         PRE_XUL_VAL;                    //上次需量中断标准时钟脉冲计数值

extern u32         SXUL_Cnt_OSub;                  //本次计数值 
extern u32         SXUL_Cnt_NSub;                  //上次计数值

extern u32         OXUL_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OXUL_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用

//被检表误差计算变量定义                           
extern u8          ENG_CLK_CH;                     //脉冲选择 0:光点头脉冲 1:电子脉冲 2:标准表脉冲
extern MODE        WORK_MODE;                      //工作模式   
extern u16         ENG_STB_RNG;                    //电能稳定误差限   
extern float       STD_ENG_CNT_VAL;                //标准电能脉冲应计个数 = STD_ENG_CST*ACT_N/MTR_ENG_CST
extern float       STD_ENG_CST;                    //标准表高频常数 用于计算误差
extern float       MTR_ENG_CST;                    //本表位脉冲低频常数
extern float       MTR_MIN_CST;                    //所有表位最小脉冲低频常数
extern u8          CURRENT_N;                      //当前圈数
extern u8          ACT_N;                          //实际校验圈数 ACT_N=MTR_ENG_CST*SET_N/MTR_MIN_CST
extern u8          SET_N;                          //设定校验圈数
extern u32         STD_ENG_Cnt;                    //标准时钟计数高位

extern u32         CUR_ENG_VAL;                    //当前电能脉冲计数值
extern u32         PRE_ENG_VAL;                    //上次电能脉冲计数值
extern u32         RAW_ENG_VAL;                    //原始电能脉冲计数值

extern u16         SENG_STB_RNG;                   //时钟脉冲稳定误差限(多次)

extern u32         OEND_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
extern u32         OEND_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用
extern u16         OEND_STB_RNG;                   //时钟脉冲稳定误差限(单次)

extern float       ENG_ERR;                        //电能误差
                                                   
extern u8          Mtr_Numb_ID;                    //表位号
extern u8          Mtr_Numb_Str[3];                //表位号ASC码

extern u8          MTR_MOD;                        //当前正在校验的设备的类型
extern u8          TTAB_JDQ_STS;                   //双回路继电器状态寄存
//extern u8          Current_Mtr_Tab;                //当前表位号所在表格
//extern u8          Mtr_Tab_Sts[MTR_TAB_NUMB];      //表位号状态表格

extern u32         Sysclk;                         //系统频率

extern u8          ADC_Start;                      //ADC转换开始标志
extern u32         ADC_SEQ_Data[8];                //AD 采样序列
extern Adc_Dat     ADC_Data;                       //AD 采样数据
extern u16         ADC_Timer;                      //AD采样定时
extern float       ADC0_Vref;                      //ADC0参考电压
extern float       ADC0_XiuZ;                      //ADC0测量修正值
//中断中用到的定时器 用volatile 定义
extern vu32        Timer_1ms;                      //1ms定时器
extern vu32        CLK_Timer;                      //时钟测量定时
extern vu8         Timer_8ms;                      //8ms定时器 扩展定时时间 单字节扩展到2.048s
extern vu8         GDT_Timer;                      //光电头口中断重启定时器
extern vu8         DZ_Timer;                       //电子脉冲口中断重启定时器
extern vu8         SZ_Timer;                       //时钟脉冲管脚重开中断定时
extern vu8         XUL_Timer;                      //需量口中断重启定时器
extern vu8         TQ_Timer;                       //投切口中断重启定时器
extern vu8         HZ_Timer;                       //HZ脉冲重开中断定时
extern vu8         KEY_Timer;                      //按键重启中断定时器
extern vu8         HC165_Timer;                    //HC165采样定时
//extern vu8         CD4094_Timer;                   //CD4094输出定时
extern vu8         PLL_CHK_Timer;                  //PLL检查定时
extern vu8         GDT_PLS_Timer;                  //光电脉冲消抖延时
extern vu8         DZ_PLS_Timer;                   //电子脉冲消抖延时
extern vu8         SZ_PLS_Timer;                   //时钟脉冲消抖延时
extern vu8         EXT_WD_Timer;                   //外部看门狗喂狗定时
extern vu8         NZTZ_PLS_Timer;                 //开路信号消抖延时
//以下定时用Timer_1ms 计时 精度较高 或时间长于2.048s
extern u32         CYCLE_OLD_Timer;                //周期测量定时器(老)
extern u32         CYCLE_NEW_Timer;                //周期测量定时器(新)
extern u32         CLK_Timer_Max;                  //时钟测量脉冲最大值
extern u32         CLK_Timer_Min;                  //时钟测量脉冲最小值
extern u16         SY_Timer;                       //失压计时
extern u16         NY_SEND_Timer;                  //命令回送定时
extern u16         CAN_STX_OVTimer;                //CAN 短数据发送超时定时器
extern u16         CAN1_STX_OVTimer;               //CAN1短数据发送超时定时器
extern u16         CAN_LTX_OVTimer;                //CAN 长数据发送超时定时器
extern u16         CAN_RX_OVTimer;                 //CAN 总线接收超时定时器
extern u16         CAN1_RX_OVTimer;                //CAN1总线接收超时定时器
extern u16         WORK_Timer;                     //进入工作模式定时器 单位:ms 最多65.535S
extern u16         STD_ECLK_Timer;                 //标准电能脉冲检测定时
extern u16         MBJ_Send_Timer;                 //表报警数据回送定时
extern u16         WZTZ_Send_Timer;                //外置跳闸数据回送定时
extern u16         NZTZ_Send_Timer;                //内置跳闸数据回送定时
extern u16         OPEN_IN_Timer;                  //开路信号产生后持续时间计时
extern u16         NEW_PLL_Ref_Timer;              //锁相环参考定时器
extern u16         OLD_PLL_Ref_Timer;              //锁相环参考定时器
extern u16         MTR_ON_Timer;                   //表位存在标志
extern u16         TTAB_JDQ_DELY_Timer;            //双回路继电器动作延时定时
extern u16         LUNCI1_Send_Timer;              //轮次1信号数据回送定时 
extern u16         LUNCI2_Send_Timer;              //轮次2信号数据回送定时
extern u16         LUNCI3_Send_Timer;              //轮次3信号数据回送定时
extern u16         LUNCI4_Send_Timer;              //轮次4信号数据回送定时
extern u16         PulOut1_Timer;                  //脉冲1输出定时
extern u16         PulOut2_Timer;                  //脉冲2输出定时
extern u16         PulOut3_Timer;                  //脉冲3输出定时
extern u16         PulOut4_Timer;                  //脉冲4输出定时
extern u16         PulOut5_Timer;                  //脉冲5输出定时
extern u16         PulOut6_Timer;                  //脉冲6输出定时
extern u16         PulOut7_Timer;                  //脉冲7输出定时
extern u16         PulOut8_Timer;                  //脉冲8输出定时
extern u16         PulOut1_Count;                  //脉冲1输出计数
extern u16         PulOut2_Count;                  //脉冲2输出计数
extern u16         PulOut3_Count;                  //脉冲3输出计数
extern u16         PulOut4_Count;                  //脉冲4输出计数
extern u16         PulOut5_Count;                  //脉冲5输出计数
extern u16         PulOut6_Count;                  //脉冲6输出计数
extern u16         PulOut7_Count;                  //脉冲7输出计数
extern u16         PulOut8_Count;                  //脉冲8输出计数
extern u16         PulOut1_Count_Set;              //脉冲1输出计数
extern u16         PulOut2_Count_Set;              //脉冲2输出计数
extern u16         PulOut3_Count_Set;              //脉冲3输出计数
extern u16         PulOut4_Count_Set;              //脉冲4输出计数
extern u16         PulOut5_Count_Set;              //脉冲5输出计数
extern u16         PulOut6_Count_Set;              //脉冲6输出计数
extern u16         PulOut7_Count_Set;              //脉冲7输出计数
extern u16         PulOut8_Count_Set;              //脉冲8输出计数
//以下定时器 用Timer_8ms 计时
extern u8          Disp_Timer;                     //显示定时处理
extern u8          GDT_RST_Timer;                  //光电头复位延时
extern u8          Com_Rx_Time[UART_NUMB];         //COM 接收定时
extern u8          Com_Rx_OvTm[UART_NUMB];         //UART接收超时处理
extern u8          UJDQ_Timer;                     //电压继电器定时
extern u8          IJDQ_Timer;                     //电流继电器定时
extern u8          ESwitch_Timer;                  //电子开关定时
extern u8          NY_CHK_Timer;                   //耐压查询定时
extern u8          Con_KEY_Timer;                  //连续按键定时
extern u8          Disp_En_Timer;                  //显示使能定时 防止连续写入SSI
extern u8          STD_CLK_Timer;                  //标准时钟脉冲定时
extern u8          TTAB_JDQ_CHG_Timer;             //双回路继电器动作定时

extern u8          Disp_Buf[8];                    //显示缓冲区
extern UART_SET    Uart_Para[UART_NUMB] ;          //UART 参数
//CAN通信定义                                      
extern u32         CANERR_RxCNT;                   //CAN接收错误计数  
extern u32         CANERR_TxCNT;                   //CAN发送错误计数  
extern u8          CAN_LMSG_TX_TYPE;               //当前发送CAN长数据类型
extern u8          CAN_LMSG_RX_TYPE;               //读取接收CAN长数据类型
extern CAN_LMSG_PR CAN_LMSG_RX_Ptr;                //CAN长数据接收结构体指针
extern CAN_LMSG_PR CAN_LMSG_TX_Ptr;                //CAN长数据发送结构体指针
extern CAN_STS_U   CAN_STS;                        //总线状态
extern CAN_STS_U   CAN1_STS;                       //CAN1总线状态
extern u8          CAN_LDATA_TX_STS;               //CAN长数据发送状态 0 无定义 'R':已经发出请求 'S' 正在发送数据 'E' 发送结束
extern u8          CAN_NEXT_MSG_IDx;               //CAN下一帧报文的索引号 根据该号判断是否丢失报文
extern u8          CAN_LMSG_TX_STS;                //CAN长帧发送状态
extern u8          CAN_SMSG_TX_STS;                //CAN短帧发送状态
extern u8          CAN1_SMSG_TX_STS;               //CAN1短帧发送状态
extern CAN_MSG     CAN_MSG_Rx;                     //用于临时接收CAN报文
extern CAN_MSG     CAN_SMSG_Tx;                    //用于临时发送CAN短数据报文
extern CAN_MSG     CAN_LMSG_Tx;                    //用于临时发送CAN长数据报文
extern u8          Echo_Sts;                       //响应状态  0 无定义 'C':收到查询 'S':已经发送回应命令 'A':已经响应
extern u8          CAN_SEND_DTIME;                 //CAN帧发送延时时间
extern u8          CAN_SEND_DELAY;                 //CAN帧发送延时 防止同时竞争总线

extern CAN_MSG     CAN_SDATA_MSG_IBUF[CAN_SDILEN]; //CAN短数据接收指令缓冲区
extern u8          CAN_SDATA_MSG_IHead;            //CAN短数据接收指令缓冲区头指针 接收指针
extern u8          CAN_SDATA_MSG_ITail;            //CAN短数据接收指令缓冲区尾指针 处理指针
extern CAN_MSG     *CAN_MSG_IPtr;                  //CAN短数据帧接收处理指针
extern CAN_MSG     CAN_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN短数据发送指令缓冲区
extern u8          CAN_SDATA_MSG_OHead;            //CAN短数据发送指令缓冲区头指针 接收指针
extern u8          CAN_SDATA_MSG_OTail;            //CAN短数据发送指令缓冲区尾指针 处理指针
extern CAN_MSG     *CAN_MSG_OPtr;                  //CAN短数据帧发送处理指针

extern CAN1_MSG    CAN1_SDATA_MSG_IBUF[CAN_SDILEN];//CAN1短数据接收指令缓冲区
extern u8          CAN1_SDATA_MSG_IHead;           //CAN1短数据接收指令缓冲区头指针 接收指针
extern u8          CAN1_SDATA_MSG_ITail;           //CAN1短数据接收指令缓冲区尾指针 处理指针
extern CAN1_MSG    *CAN1_MSG_IPtr;                 //CAN1短数据帧接收处理指针
extern CAN1_MSG     CAN1_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN1短数据发送指令缓冲区
extern u8          CAN1_SDATA_MSG_OHead;            //CAN1短数据发送指令缓冲区头指针 接收指针
extern u8          CAN1_SDATA_MSG_OTail;            //CAN1短数据发送指令缓冲区尾指针 处理指针
extern CAN1_MSG    *CAN1_MSG_OPtr;                  //CAN1短数据帧发送处理指针

extern CAN_MSG     CAN_LDATA_MSG_IBUF[CAN_LDILEN]; //CAN长数据接收指令缓冲区               
extern u8          CAN_LDATA_MSG_IHead;            //CAN长数据接收指令缓冲区头指针 接收指针
extern u8          CAN_LDATA_MSG_ITail;            //CAN长数据接收指令缓冲区尾指针 处理指针
extern u8          CAN_LDATA_SERR_Cnt;             //CAN长数据发送错误计数

//COM 缓冲区定义                                 
extern u8          Com0_IBuf[MTRCOM_ILEN];         //MTRCOM接收缓冲区 接收多功能表数据
extern u8          Com0_OBuf[MTRCOM_OLEN];         //MTRCOM发送缓冲区 发送多功能表数据
extern u8          Com1_IBuf[LCTCOM_ILEN];         //LCTCOM接收缓冲区 接收多功能表数据
extern u8          Com1_OBuf[LCTCOM_OLEN];         //LCTCOM发送缓冲区 发送多功能表数据
extern u8          Com3_IBuf[RTECOM_ILEN];         //RTECOM接收缓冲区
extern u8          Com3_OBuf[RTECOM_OLEN];         //RTECOM发送缓冲区
extern u8          Com5_IBuf[ATECOM_ILEN];         //ATECOM接收缓冲区
extern u8          Com5_OBuf[ATECOM_OLEN];         //ATECOM发送缓冲区
extern u8          Com6_IBuf[IRECOM_ILEN];         //IRECOM接收缓冲区
extern u8          Com6_OBuf[IRECOM_OLEN];         //IRECOM发送缓冲区
//COM 处理定义
extern u16         Com_IHead[UART_NUMB];           //COM接收缓冲区头指针 接收指针                                  
extern u16         Com_ITail[UART_NUMB];           //COM接收缓冲区尾指针 处理指针                                  
extern u8          Com_Rx_Sts[UART_NUMB];          //COM接收状态 'R' 正在接收多功能表数据 'E' 多功能表数据接收结束 
extern u16         Com_OHead[UART_NUMB];           //COM发送缓冲区头指针接收指针                                   
extern u16         Com_OTail[UART_NUMB];           //COM发送缓冲区尾指针处理指针                                   
extern u8          Com_Tx_Sts[UART_NUMB];          //COM数据发送状态 'Y'正在发送数据 其它 不在发送数据             

extern u8          CANT_STR[8];                    //CAN帧字符串临时缓冲区
extern u8          TEMP_STR[20];                   //临时用字符串
//位变量定义
extern u32         SINGLE_OR_THREE ;               //单相台 三相台标志 0: 单相台 1:三相台
extern u32         NEW_ENG_PLS     ;               //收到新电能脉冲标志
extern u32         NEW_CLK_PLS     ;               //收到新时钟脉冲标志
extern u32         NEW_XUL_PLS     ;               //收到新需量脉冲标志 
extern u32         FIRST_ENG_PLS   ;               //首次测量电能脉冲标志
extern u32         FIRST_CLK_PLS   ;               //首次测量时钟脉冲标志
extern u32         FIRST_XUL_PLS   ;               //首次测量需量脉冲标志
extern u32         OVER_ERR_FLAG   ;               //超差标志
extern u32         NEW_ENG_DATA    ;               //电能误差新数据
extern u32         BEEP_EN         ;               //蜂鸣器使能标志
extern u32         MTR_PLUG        ;               //挂表标志 0:不挂表 1:挂表
extern u32         GDT_RST_FLAG    ;               //光电头复位标志
extern u32         NEW_CMD         ;               //新命令标志
extern u32         HB_BACK_EDGE    ;               //后延对斑标志
extern u32         SY_START        ;               //失压启动标志
extern u32         ZZ_PLS_LOADED   ;               //脉冲是否已经预置
extern u32         HB_CATCHED      ;               //黑斑对准标志
extern u32         VERIFY_END      ;               //校核常数走字试验结束标志 
extern u32         SY_ACT          ;               //失压已动作标志 
extern u32         RISE_FALL_LVL   ;               //上升沿/下降沿中断
extern u32         ENG_STB_CHK     ;               //电能脉冲稳定检查标志
extern u32         SCLK_STB_CHK    ;               //综合时钟脉冲稳定检查标志
extern u32         OCLK_STB_CHK    ;               //单次时钟脉冲稳定检查标志
extern u32         XUL_STB_CHK     ;               //需量脉冲稳定检测标志
extern u32         NO_STD_CLK      ;               //标准时钟脉冲存在标志 0:存在 1:不存在
extern u32         NO_CLK_PLS      ;               //没有检测到时钟脉冲标志
extern u32         NO_STD_ENG      ;               //没有检测到标准电能脉冲标志
extern u32         DISP_HL_LVL     ;               //显示脉冲周期标志 0: 显示低电平时间 1:显示高电平时间
extern u32         PULSE_ZZ_END    ;               //脉冲走字试验结束标志
extern u32         NEW_WZHZ_PLS    ;               //外置合闸脉冲标志
extern u32         NEW_NZHZ_PLS    ;               //内置合闸脉冲标志
extern u32         NEW_TQ_PLS      ;               //新时段投切脉冲标志
extern u32         NEW_MBJ_PLS     ;               //表报警脉冲标志
extern u32         NEW_JBJ_PLS     ;               //续流继电器报警标志
extern u32         REF_JZ_INT      ;               //标准晶振中断标志
extern u32         RST_FLAG        ;               //复位标志
extern u32         NEW_LUNCI1_PLS  ;               //轮次1动作发生标志
extern u32         NEW_LUNCI2_PLS  ;               //轮次2动作发生标志
extern u32         NEW_LUNCI3_PLS  ;               //轮次3动作发生标志
extern u32         NEW_LUNCI4_PLS  ;               //轮次4动作发生标志
extern u32         TX_ZS_BIT       ;               //通信指示等标志
extern u32         SZCLK_SET_TooM_T;               //时钟频率设置太小标志(临时 第一次检测到设置太小)
extern u32         SZCLK_SET_TooM  ;               //时钟频率设置太小标志
extern u32         NEW_KEY_FLAG    ;               //新按键标志
extern u32         PLUG_CHG_FLAG   ;               //挂表改变标志
extern u32         CD4094_FLAG     ;               //4094改变输出标志                                                   
extern u32         KEY_PC_PLUG     ;               //按键选择挂表 还是 上位机选择挂表
extern u32         TZEN            ;               //合闸检测使能命令   
extern u32         MBJEN           ;               //表报警信号检测使能
extern u32         GZ_FLAG         ;               //电流旁路继电器故障标志
extern u32         GZS_FLAG        ;               //临时用故障标志
extern u32         NZTZ_FLAG       ;               //内置跳闸标志(表位续流继电器) 
extern u32         WZTZ_FLAG       ;               //外置跳闸标志(表跳闸触点) 
extern u32         MASTER_START    ;               //总控中心启动标志                                                   
extern u32         GDT_INT_REEN    ;               //光电头口中断重启定时器
extern u32         DZ_INT_REEN     ;               //电子脉冲口中断重启定时器
extern u32         SZ_INT_REEN     ;               //时钟脉冲管脚重开中断定时
extern u32         XUL_INT_REEN    ;               //需量口中断重启定时器
extern u32         TQ_INT_REEN     ;               //投切口中断重启定时器
extern u32         HZ_INT_REEN     ;               //合闸口中断重启定时器
extern u32         KEY_INT_REEN    ;               //按键口中断重启定时器
extern u32         CAN_ERR         ;               //CAN总线错误 
extern u32         CAN1_ERR        ;               //CAN1总线错误
                                                   
extern u32         I_JDQ           ;               //电流继电器状态 0: 断开(电流接入) 1: 吸合(电流旁路)
extern u32         I_JDQ_CHG       ;               //电流继电器状态改变
extern u32         I_JDQ_EN        ;               //电流继电器使能信号状态
extern u32         UJDQ_FLAG       ;               //电压继电器状态发生改变标志
extern u32         ESwitch_FLAG    ;               //电子开关状态发生改变标志
extern u32         U_JDQ[3]        ;               //电压继电器状态
extern u32         U_ESwitch[3]    ;               //电压电子开关状态
extern u32         DZ_NEW_PLS      ;               //电子脉冲中断标志   用于电子脉冲信号检测
extern u32         GD_NEW_PLS      ;               //光电头脉冲中断标志 用于光电头信号检测
extern u32         SZ_NEW_PLS      ;               //时钟脉冲信号检测   用于电子脉冲信号检测
extern u32         NZTZ_NEW_PLS    ;               //开路信号中断标志

extern u32         ENT_XY_PLS      ;               //小显示内容进入当前试验项目
extern u32         DIS_UNSTD_PLS   ;               //小显示显示试验结论不合格
extern u32         DIS_STD_PLS     ;               //小显示显示试验结论合格
extern u32         LIGHT_CTL_PC    ;               //表状态指示灯控制  TRUE=PC  FLASE=CPU(TST)

extern u32         Pulse1_RUN_CTL  ;	  						      //脉冲1输出控制。0表示停止输出；1表示开始输出
extern u32         Pulse2_RUN_CTL  ;					          //脉冲2输出控制。0表示停止输出；1表示开始输出
extern u32         Pulse3_RUN_CTL  ;	  						      //脉冲3输出控制。0表示停止输出；1表示开始输出
extern u32         Pulse4_RUN_CTL  ;					          //脉冲4输出控制。0表示停止输出；1表示开始输出
extern u32         Pulse5_RUN_CTL  ;	  						      //脉冲5输出控制。0表示停止输出；1表示开始输出
extern u32         Pulse6_RUN_CTL  ;					          //脉冲6输出控制。0表示停止输出；1表示开始输出
extern u32         Pulse7_RUN_CTL  ;					          //脉冲7输出控制。0表示停止输出；1表示开始输出
extern u32         Pulse8_RUN_CTL  ;					          //脉冲8输出控制。0表示停止输出；1表示开始输出

extern u32         PulOut1_RVS_Bit ;               //脉冲1反转标志位
extern u32         PulOut2_RVS_Bit ;               //脉冲2反转标志位
extern u32         PulOut3_RVS_Bit ;               //脉冲3反转标志位
extern u32         PulOut4_RVS_Bit ;               //脉冲4反转标志位
extern u32         PulOut5_RVS_Bit ;               //脉冲5反转标志位
extern u32         PulOut6_RVS_Bit ;               //脉冲6反转标志位
extern u32         PulOut7_RVS_Bit ;               //脉冲6反转标志位
extern u32         PulOut8_RVS_Bit ;               //脉冲6反转标志位

extern u32         YKSIG_EN        ;               //遥控信号检测使能
extern u32         YKSIG_MOD       ;               //遥控信号检测模式
extern u32         YKSIG_TRG_MOD   ;               //遥控信号检测模式是脉冲时，触发方式
extern u32         GZ_CHECK_FLAG   ;               //续流继电器故障检测开始标志
extern u32         TTAB_JDQ_CHG    ;               //双回路继电器动作标志
extern u32         TTAB_JDQ_DELY   ;               //双回路继电器动作延时标志
//内存0x20000004 单元 4字节 位定义
/****************************************************************************
* CAN短数据帧发送
****************************************************************************/
extern const CAN_MSG CAN_TX_SMSG;

/********************************************************
* IO口基址定义
********************************************************/
extern const u32 PORT_BASE_ADDR_TAB[];
/********************************************************
* 周期测量脉冲输入管脚对应端口表格 
* GDT_PLS   '1'      //光电头脉冲 GDT_MC
* DZ_PLS    '2'      //电子脉冲   DZ_MC
* SZ_PLS    '3'      //时钟脉冲   SZ_MC
* XUL_PLS   '4'      //需量脉冲   XL_MC
* TQ_PLS    '5'      //投切脉冲   TQ_MC
* HZ_PLS    '6'      //合闸脉冲   HZ_MC
********************************************************/
extern const u32 CYCLE_PORT_TAB[];
/********************************************************
* 周期测量脉冲输入管脚表格 
* GDT_PLS   '1'      //光电头脉冲 GDT_MC
* DZ_PLS    '2'      //电子脉冲   DZ_MC
* SZ_PLS    '3'      //时钟脉冲   SZ_MC
* XUL_PLS   '4'      //需量脉冲   XL_MC
* TQ_PLS    '5'      //投切脉冲   TQ_MC
* HZ_PLS    '6'      //合闸脉冲   HZ_MC
********************************************************/
extern const u8 CYCLE_PIN_TAB[];
/********************************************************
* 管脚重开定时器 
* 光电头脉冲重开定时器 GDT_MC
* 电子脉冲重开定时器   DZ_MC
* 时钟脉冲重开定时器   SZ_MC
* 需量脉冲重开定时器   XL_MC
* 投切脉冲重开定时器   TQ_MC
* 合闸脉冲重开定时器   HZ_MC
********************************************************/
extern const vu8_Ptr IO_Timer_Tab[];
/********************************************************
* 管脚重开中断标志 
* 光电头脉冲重开定时器 GDT_MC
* 电子脉冲重开定时器   DZ_MC
* 时钟脉冲重开定时器   SZ_MC
* 需量脉冲重开定时器   XL_MC
* 投切脉冲重开定时器   TQ_MC
* 合闸脉冲重开定时器   HZ_MC
********************************************************/
extern const u32_Ptr IO_REEN_TAB[];
/*****************************************************************************
* 程序版本号
*****************************************************************************/
extern const u8 VER_TAB[];
extern const SAVE_S Save_Tab;

/*****************************************************************************
* CAN长数据发送处理结构体表格
*****************************************************************************/
extern const CAN_LMSG_PR CAN_LMSG_TX_TAB[];
/*****************************************************************************
* CAN长数据接收处理结构体表格
*****************************************************************************/
extern const CAN_LMSG_PR CAN_LMSG_RX_TAB[];
/******************************************************************************
* CAN波特率设置表
******************************************************************************/
extern const u32 CANBitClkSettings[];
/*****************************************************************************
* CAN0 报文对象处理设置表格 MSG RAM 
*****************************************************************************/
extern const CAN_MSG_SET CAN_MSG_SET_TAB[];
/*****************************************************************************
* CAN1 报文对象处理设置表格 MSG RAM 
*****************************************************************************/
extern const CAN1_MSG_SET CAN1_MSG_SET_TAB[];

//CAN 接口地址表格
extern const u32 UART_PORT[8];

