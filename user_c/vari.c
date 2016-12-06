/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : vari.c
;* Author             : 张力阵
;* 变量定义
*******************************************************************************/
#include "TM4C123_Lib.h"
#include "Function.h"
//位定义 定义字变量 包含32个位变量	4字节对齐 在LM3S2139.h 中定义
//变量定义
u32         SysRCC_CFG;                     //系统RCC配置

__align(4)                                  //地址分配 4的倍数
SAVE_S      SOLID_CFG;                      //固化的配置结构体

u8          MTYPE;                          //表类型 'N' 国网表  'S' 南网表
u8          EXT_WDFEED_CNT;                 //外部看门狗喂狗计数

u8          MTR_RDY_DATA;                   //表放置状态数据 '0' 没表 '1'有表

u8          Current_Save_Tab;               //当前保存值所在表格
u8          Save_Tab_Sts[SAVE_TAB_NUMB];    //表位号状态表格

u8          PLL_ERR_Cnt;                    //锁相环错误计数
u8          Set_Uclop;                      //设置电压接入
u8          Pwr_Phase;                      //功耗相
u8          DXTZ;                           //单相费控表跳闸继电器状态
u8          MBJ_DATA;                       //表报警数据
u8          LUNCI1_DATA[2];                 //轮次1信号 [0]常开 [1]常闭 
u8          LUNCI2_DATA[2];                 //轮次1信号 [0]常开 [1]常闭 
u8          LUNCI3_DATA[2];                 //轮次1信号 [0]常开 [1]常闭 
u8          LUNCI4_DATA[2];                 //轮次1信号 [0]常开 [1]常闭 
u8          LUNCI1_SEND_COUNT;              //轮次1信号发送次数计数
u8          LUNCI2_SEND_COUNT;              //轮次2信号发送次数计数
u8          LUNCI3_SEND_COUNT;              //轮次3信号发送次数计数
u8          LUNCI4_SEND_COUNT;              //轮次4信号发送次数计数
u8          WZHZ_DATA[2];                   //外置合闸信号 [0]常开 [1]常闭 
u8          GZ_DATA;                        //故障数据
u8          NZHZ_DATA;                      //内置合闸信号(电流旁路继电器信号)
HC165_DT    HC165_DATA;                     //HC165数据
u16         HC165_TDATA;                    //HC165临时数据
u16         HC165_XDATA;                    //HC165数据变化
//CD4094_DT   CD4094_DATA;                    //CD4094数据
u8          NY_RESULT;                      //耐压结果
u8          TIME_ASC[8];                    //时间ASC HH-MM-SS  时分秒
u8          PLSGD;                          //脉冲共高共低
u8          MFClk_Mode;                     //多功能脉冲输入方式
u8          MFClk_Type;                     //多功能脉冲输入类型
u8          PLSHC_MODE;                     //脉冲合成方式
u8          PLS_QUAD;                       //脉冲象限
u8          Disp_Choose;                    //显示选择
u8          Disp_Code_Mode;                 //显示译码模式
u16         ELEC_PLS_CNT;                   //当前电子脉冲计数
u8          ENG_ERR_ASC[9];                 //电能误差ASC码 最多带四位小数点 符号位 +5位数据
u8          CLK_FREQ_ASC[9];                //时钟频率 
u8          DAY_ERR_ASC[9];                 //日计时误差 最多3个小数点
u8          XUL_TIME_ASC[9];                //需量周期ASC码 
u8          VERIFY_ENG_ASC[9];              //校核超时走字电能ASC表示
u8          CURRENT_N_ASC[2];               //当前圈数ASC码
u8          CURRENT_ENG_ASC[9];             //当前电能ASC码
u8          CURRENT_PLS_ASC[9];             //当前脉冲数ASC码
u8          HIGH_LVL_TIME[9];               //高电平时间 第一个字节 'H'
u8          LOW_LVL_TIME[9];                //低电平时间 第一个字节 'L'
u32         PLS_Lvl_Time[2];                //脉冲电平时间
u32         High_Lvl_Time_Tp;               //临时存放脉冲高电平时间
u32         Low_Lvl_Time_Tp;                //临时存放脉冲低电平时间
u8          High_Lvl_CNT;                   //高电平中断测量计数
u8          Low_Lvl_CNT;                    //低电平中断测量计数
u8          CMD_DATA;                       //单字节命令数据
u8          WIRE_TYPE;                      //接线方式
u16         DIVIDE_Coef;                    //分频系数
u8          CYCLE_MEA_SEL;                  //周期测量选择
u8          CYCLE_MEA_ID;                   //周期测量选择ID编号
u8          SY_CNT;                         //失压计数
u8          SY_PROCESS;                     //失压进程
u8          SY_MODE;                        //失压模式 
u8          SY_PHASE;                       //失压相 7:三相同时 1:UA 2:UB 4:UC 可以组合
u8          PZZS;                           //计算盘转误差 多少圈算次误差 
u8          PZBZ;                           //盘转和脉冲比值
u8          PZ_STD_CNT;                     //盘转试验标准计数
u16         CUR_PZ_Cnt_Val;                 //盘转采样点   电子脉冲计数值
u16         PRE_PZ_Cnt_Val;                 //上一个采样点 电子脉冲计数值
u8          PZ_ERR_ASC[3];                  //盘转误差圈数
float       VERIFY_ENG_Kwh;                 //校核常数走字度数
u32         VERIFY_PLS_Set;                 //校核常数走字脉冲数
u32         CURRENT_VERIFY_PLS;             //当前校核常数走字脉冲数    
u32         ZZ_PLS_Set;                     //走字脉冲数设置

//电能脉冲计数累加                          
u32         CURRENT_PLS_CNT;                //脉冲计数 开机后一直累加
float       CURRENT_ENG_KWh;                //当前电能
//检高频变量定义                            
float       GP_ENG_CST;                     //被检表高频常数
u32         GP_CLK_SET;                     //检高频脉冲数设置
u32         GP_CLK_ACT;                     //实际高频计数值
u16         GP_RELOAD_TIME;                 //时钟频率重装次数
u16         GP_RELOAD_VAL;                  //时钟频率重装值
u16         GP_RELOAD_Cnt;                  //高频脉冲重装计数
//标准时钟脉冲计数高位
u32         STD_CLK_Cnt;                    //标准时钟计数高位 
//时钟脉冲频率和日计时误差计算变量定义      
u8          CLK_MEA_CTL;                    //时钟频率测量控制
u16         CLK_RELOAD_TIME_N;              //时钟频率重装次数新值
u16         CLK_RELOAD_TIME_O;              //时钟频率重装次数旧值
u16         CLK_RELOAD_VAL_N;               //时钟频率重装值新值
u16         CLK_RELOAD_VAL_O;               //时钟频率重装值旧值
u16         CLK_RELOAD_Cnt;                 //当前时钟脉冲重装计数
u8          CLK_MEA_TIME;                   //时钟频率测量定时 单位:s 秒  10---100s之间 默认20s
float       CLK_FREQ_SET;                   //时钟频率设定值 默认1HZ
float       CLK_FREQ_INT;                   //时钟频率规格化值 默认1HZ
double      CLK_DAY_ERR;                    //日计时误差 
double      CLK_ACT_FREQ;                   //实际频率 化整后 2009.3.13 为提高精度 改为双精度浮点数
double      STD_CLK_VAL_ONE;                //标准时钟计算值 用于计算频率  单次中断             计算时钟频率 标准值
double      STD_CLK_VAL_SUM;                //标准时钟计算值 用于计算频率 CLK_RELOAD_TIME次中断 计算时钟频率 标准值
u32         STD_CLK_CNT_ONE;                //标准时钟计数值 每次中断                 应计的标准脉冲数
u32         STD_CLK_CNT_SUM;                //标准时钟计数值 总 CLK_RELOAD_TIME次中断 应计的标准脉冲数

u32         RAW_CLK_VAL;                    //原始时钟脉冲计数值
u32         CUR_CLK_VAL;                    //当前时钟脉冲计数值
u32         PRE_CLK_VAL;                    //上次时钟脉冲计数值
    
u32         SCLK_Cnt_OSub;                  //本次计数值 
u32         SCLK_Cnt_NSub;                  //上次计数值
u32         SCLK_Cnt_RSet_Max;              //合计数改变触发重设时钟频率最大值
u32         SCLK_STB_RNG;                   //时钟脉冲稳定误差限(多次)

u32         OCLK_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
u32         OCLK_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用
u32         OCLK_Cnt_RSet_Max;              //单次计数改变触发重设时钟频率最大值
u32         OCLK_STB_RNG;                   //时钟脉冲稳定误差限(单次)

//需量周期变量定义
u8          XUL_MEA_CTL;                    //需量周期测量控制
float       XUL_TIME;                       //需量周期
u8          XUL_RELOAD_TIME;                //需量脉冲重装次数 设定值
u8          XUL_RELOAD_Cnt;                 //需量脉冲计数
double      XUL_Cnt_Sum;                    //需量计算 标准时钟脉冲累加

u32         RAW_XUL_VAL;                    //原始需量中断标准时钟脉冲计数值
u32         CUR_XUL_VAL;                    //当前需量中断标准时钟脉冲计数值
u32         PRE_XUL_VAL;                    //上次需量中断标准时钟脉冲计数值

u32         SXUL_Cnt_OSub;                  //本次计数值 
u32         SXUL_Cnt_NSub;                  //上次计数值

u32         OXUL_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
u32         OXUL_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用

//被检表误差计算变量定义                    
u8          ENG_CLK_CH;                     //脉冲选择 0:光电头脉冲 1:电子脉冲 
MODE        WORK_MODE;                      //工作模式
u16         ENG_STB_RNG;                    //电能稳定误差限   
float       STD_ENG_CNT_VAL;                //标准电能脉冲应计个数 = STD_ENG_CST*ACT_N/MTR_ENG_CST
float       STD_ENG_CST;                    //标准表高频常数 用于计算误差
float       MTR_ENG_CST;                    //本表位脉冲低频常数
float       MTR_MIN_CST;                    //所有表位最小脉冲低频常数
u8          CURRENT_N;                      //当前圈数
u8          ACT_N;                          //实际校验圈数 ACT_N=本表位低频常数*SET_N/最小低频常数
u8          SET_N;                          //设定校验圈数
u32         STD_ENG_Cnt;                    //标准时钟计数高位

u32         CUR_ENG_VAL;                    //当前电能脉冲计数值
u32         PRE_ENG_VAL;                    //上次电能脉冲计数值
u32         RAW_ENG_VAL;                    //原始电能脉冲计数值

u16         SENG_STB_RNG;                   //时钟脉冲稳定误差限(多次)

u32         OEND_Cnt_OSub;                  //上次单次时钟脉冲计数差值 用于判断是否稳定用
u32         OEND_Cnt_NSub;                  //本次单次时钟脉冲计数差值 用于判断是否稳定用
u16         OEND_STB_RNG;                   //时钟脉冲稳定误差限(单次)

float       ENG_ERR;                        //电能误差
                                            
u8          Mtr_Numb_ID;                    //表位号
u8          Mtr_Numb_Str[3];                //表位号ASC码

u8          MTR_MOD;                        //当前正在校验的设备的类型
u8          TTAB_JDQ_STS;                   //双回路继电器状态寄存
//u8          Current_Mtr_Tab;                //当前表位号所在表格
//u8          Mtr_Tab_Sts[MTR_TAB_NUMB];      //表位号状态表格

u32         Sysclk;                         //系统频率

u8          ADC_Start;                      //ADC转换开始标志
u32         ADC_SEQ_Data[8];                //AD 采样序列
Adc_Dat     ADC_Data;                       //AD 采样数据
u16         ADC_Timer;                      //AD采样定时
float       ADC0_Vref;                      //ADC0参考电压
float       ADC0_XiuZ;                      //ADC0测量修正值
//中断中用到的定时器 用volatile 定义 仿真程序循环等待 不取当前定时值问题
vu32        Timer_1ms;                      //1ms定时器
vu32        CLK_Timer;                      //时钟测量定时
vu8         Timer_8ms;                      //8ms定时器 扩展定时时间 单字节扩展到2.048s
vu8         GDT_Timer;                      //光电头口中断重启定时器
vu8         DZ_Timer;                       //电子脉冲口中断重启定时器
vu8         SZ_Timer;                       //时钟脉冲管脚重开中断定时
vu8         XUL_Timer;                      //需量口中断重启定时器
vu8         TQ_Timer;                       //投切口中断重启定时器
vu8         HZ_Timer;                       //合闸口中断重启定时器
vu8         KEY_Timer;                      //按键口中断重启定时器
vu8         HC165_Timer;                    //HC165采样定时
//vu8         CD4094_Timer;                   //CD4094输出定时
vu8         PLL_CHK_Timer;                  //PLL检查定时
vu8         GDT_PLS_Timer;                  //光电脉冲消抖延时
vu8         DZ_PLS_Timer;                   //电子脉冲消抖延时
vu8         SZ_PLS_Timer;                   //时钟脉冲消抖延时
vu8         EXT_WD_Timer;                   //外部看门狗喂狗定时
vu8         NZTZ_PLS_Timer;                 //开路信号消抖延时
//以下定时用Timer_1ms 计时 精度较高 或时间长于2.048s
u32         CYCLE_OLD_Timer;                //周期测量定时器(老)
u32         CYCLE_NEW_Timer;                //周期测量定时器(新)
u32         CLK_Timer_Max;                  //时钟测量脉冲最大值
u32         CLK_Timer_Min;                  //时钟测量脉冲最小值
u16         SY_Timer;                       //失压计时
u16         NY_SEND_Timer;                  //命令回送定时
u16         CAN_STX_OVTimer;                //CAN 短数据发送超时定时器
u16         CAN1_STX_OVTimer;               //CAN1短数据发送超时定时器
u16         CAN_LTX_OVTimer;                //CAN 长数据发送超时定时器
u16         CAN_RX_OVTimer;                 //CAN 总线接收超时定时器 用于处理总线错误
u16         CAN1_RX_OVTimer;                //CAN1总线接收超时定时器
u16         WORK_Timer;                     //进入工作模式定时器 单位:ms 最多65.535S
u16         STD_ECLK_Timer;                 //标准电能脉冲检测定时
u16         MBJ_Send_Timer;                 //表报警数据回送定时
u16         WZTZ_Send_Timer;                //外置跳闸数据回送定时
u16         NZTZ_Send_Timer;                //内置跳闸数据回送定时
u16         OPEN_IN_Timer;                  //开路信号产生后持续时间计时
u16         NEW_PLL_Ref_Timer;              //锁相环参考定时器
u16         OLD_PLL_Ref_Timer;              //锁相环参考定时器
u16         MTR_ON_Timer;                   //表位存在标志
u16         TTAB_JDQ_DELY_Timer;            //双回路继电器动作延时定时
u16         LUNCI1_Send_Timer;              //轮次1信号数据回送定时 
u16         LUNCI2_Send_Timer;              //轮次2信号数据回送定时
u16         LUNCI3_Send_Timer;              //轮次3信号数据回送定时
u16         LUNCI4_Send_Timer;              //轮次4信号数据回送定时
u16         PulOut1_Timer;                  //脉冲1输出定时
u16         PulOut2_Timer;                  //脉冲2输出定时
u16         PulOut3_Timer;                  //脉冲3输出定时
u16         PulOut4_Timer;                  //脉冲4输出定时
u16         PulOut5_Timer;                  //脉冲5输出定时
u16         PulOut6_Timer;                  //脉冲6输出定时
u16         PulOut7_Timer;                  //脉冲7输出定时
u16         PulOut8_Timer;                  //脉冲8输出定时
u16         PulOut1_Count;                  //脉冲1输出计数
u16         PulOut2_Count;                  //脉冲2输出计数
u16         PulOut3_Count;                  //脉冲3输出计数
u16         PulOut4_Count;                  //脉冲4输出计数
u16         PulOut5_Count;                  //脉冲5输出计数
u16         PulOut6_Count;                  //脉冲6输出计数
u16         PulOut7_Count;                  //脉冲7输出计数
u16         PulOut8_Count;                  //脉冲8输出计数
u16         PulOut1_Count_Set;              //脉冲1输出计数
u16         PulOut2_Count_Set;              //脉冲2输出计数
u16         PulOut3_Count_Set;              //脉冲3输出计数
u16         PulOut4_Count_Set;              //脉冲4输出计数
u16         PulOut5_Count_Set;              //脉冲5输出计数
u16         PulOut6_Count_Set;              //脉冲6输出计数
u16         PulOut7_Count_Set;              //脉冲7输出计数
u16         PulOut8_Count_Set;              //脉冲8输出计数
//以下定时器 用Timer_8ms 计时
u8          Disp_Timer;                     //显示定时处理
u8          GDT_RST_Timer;                  //光电头复位延时
u8          Com_Rx_Time[UART_NUMB];         //COM 接收定时
u8          Com_Rx_OvTm[UART_NUMB];         //UART接收超时处理
u8          UJDQ_Timer;                     //电压继电器定时
u8          IJDQ_Timer;                     //电流继电器定时
u8          ESwitch_Timer;                  //电子开关定时
u8          NY_CHK_Timer;                   //耐压查询定时
u8          Con_KEY_Timer;                  //连续按键定时
u8          Disp_En_Timer;                  //显示使能定时 防止连续写入SSI
u8          STD_CLK_Timer;                  //标准时钟脉冲检测定时
u8          TTAB_JDQ_CHG_Timer;             //双回路继电器动作定时
u8          Disp_Buf[8];                    //显示缓冲区
UART_SET    Uart_Para[UART_NUMB];           //UART 参数
//CAN通信定义                               
u32         CANERR_RxCNT;                   //CAN接收错误计数  
u32         CANERR_TxCNT;                   //CAN发送错误计数  
u8          CAN_LMSG_TX_TYPE;               //当前发送CAN长数据类型
u8          CAN_LMSG_RX_TYPE;               //读取接收CAN长数据类型
CAN_LMSG_PR CAN_LMSG_RX_Ptr;                //CAN长数据接收结构体指针
CAN_LMSG_PR CAN_LMSG_TX_Ptr;                //CAN长数据发送结构体指针
CAN_STS_U   CAN_STS;                        //CAN0总线状态
CAN_STS_U   CAN1_STS;                       //CAN1总线状态
u8          CAN_LDATA_TX_STS;               //CAN长数据发送状态 见define.h CAN长数据发送状态定义
u8          CAN_NEXT_MSG_IDx;               //CAN下一帧报文的索引号 根据该号判断是否丢失报文
u8          CAN_LMSG_TX_STS;                //CAN长帧发送状态
u8          CAN_SMSG_TX_STS;                //CAN短帧发送状态
u8          CAN1_SMSG_TX_STS;               //CAN1短帧发送状态
CAN_MSG     CAN_MSG_Rx;                     //用于临时接收CAN报文
CAN_MSG     CAN_SMSG_Tx;                    //用于临时发送CAN短数据报文
CAN_MSG     CAN_LMSG_Tx;                    //用于临时发送CAN长数据报文
u8          Echo_Sts;                       //响应状态  0 无定义 'C':收到查询 'S':已经发送回应命令 'A':已经响应
u8          CAN_SEND_DTIME;                 //CAN帧发送延时时间
u8          CAN_SEND_DELAY;                 //CAN帧发送延时定时器 防止同时竞争总线
CAN_MSG     CAN_SDATA_MSG_IBUF[CAN_SDILEN]; //CAN短数据接收指令缓冲区
u8          CAN_SDATA_MSG_IHead;            //CAN短数据接收指令缓冲区头指针 接收指针
u8          CAN_SDATA_MSG_ITail;            //CAN短数据接收指令缓冲区尾指针 处理指针
CAN_MSG     *CAN_MSG_IPtr;                  //CAN短数据帧接收处理指针
CAN_MSG     CAN_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN短数据发送指令缓冲区
u8          CAN_SDATA_MSG_OHead;            //CAN短数据发送指令缓冲区头指针 接收指针
u8          CAN_SDATA_MSG_OTail;            //CAN短数据发送指令缓冲区尾指针 处理指针
CAN_MSG     *CAN_MSG_OPtr;                  //CAN短数据帧发送处理指针
CAN1_MSG    CAN1_SDATA_MSG_IBUF[CAN_SDILEN];//CAN1短数据接收指令缓冲区
u8          CAN1_SDATA_MSG_IHead;           //CAN1短数据接收指令缓冲区头指针 接收指针
u8          CAN1_SDATA_MSG_ITail;           //CAN1短数据接收指令缓冲区尾指针 处理指针
CAN1_MSG    *CAN1_MSG_IPtr;                 //CAN1短数据帧接收处理指针
CAN1_MSG     CAN1_SDATA_MSG_OBUF[CAN_SDOLEN]; //CAN1短数据发送指令缓冲区
u8          CAN1_SDATA_MSG_OHead;            //CAN1短数据发送指令缓冲区头指针 接收指针
u8          CAN1_SDATA_MSG_OTail;            //CAN1短数据发送指令缓冲区尾指针 处理指针
CAN1_MSG    *CAN1_MSG_OPtr;                  //CAN1短数据帧发送处理指针

CAN_MSG     CAN_LDATA_MSG_IBUF[CAN_LDILEN]; //CAN长数据接收指令缓冲区               
u8          CAN_LDATA_MSG_IHead;            //CAN长数据接收指令缓冲区头指针 接收指针
u8          CAN_LDATA_MSG_ITail;            //CAN长数据接收指令缓冲区尾指针 处理指针
u8          CAN_LDATA_SERR_Cnt;             //CAN长数据发送错误计数

//COM 缓冲区定义                                 
u8          Com0_IBuf[MTRCOM_ILEN];         //MTRCOM接收缓冲区 接收多功能表数据
u8          Com0_OBuf[MTRCOM_OLEN];         //MTRCOM发送缓冲区 发送多功能表数据
u8          Com1_IBuf[LCTCOM_ILEN];         //LCTCOM接收缓冲区 接收多功能表数据
u8          Com1_OBuf[LCTCOM_OLEN];         //LCTCOM发送缓冲区 发送多功能表数据
u8          Com3_IBuf[RTECOM_ILEN];         //RTECOM接收缓冲区
u8          Com3_OBuf[RTECOM_OLEN];         //RTECOM发送缓冲区
u8          Com5_IBuf[ATECOM_ILEN];         //ATECOM接收缓冲区
u8          Com5_OBuf[ATECOM_OLEN];         //ATECOM发送缓冲区
u8          Com6_IBuf[IRECOM_ILEN];         //IRECOM接收缓冲区
u8          Com6_OBuf[IRECOM_OLEN];         //IRECOM发送缓冲区
//COM 处理定义
u16         Com_IHead[UART_NUMB];           //COM接收缓冲区头指针 接收指针
u16         Com_ITail[UART_NUMB];           //COM接收缓冲区尾指针 处理指针
u8          Com_Rx_Sts[UART_NUMB];          //COM接收状态 'R' 正在接收多功能表数据 'E' 多功能表数据接收结束 
u16         Com_OHead[UART_NUMB];           //COM发送缓冲区头指针接收指针
u16         Com_OTail[UART_NUMB];           //COM发送缓冲区尾指针处理指针
u8          Com_Tx_Sts[UART_NUMB];          //COM数据发送状态 'Y'正在发送数据 其它 不在发送数据

u8          CANT_STR[8];                    //CAN帧字符串临时缓冲区
u8          TEMP_STR[20];                   //临时用字符串
PULSE_SET   YaoXin1_Set;                    //专变终端脉冲1设置参数
PULSE_SET   YaoXin2_Set;                    //专变终端脉冲2设置参数
PULSE_SET   YaoXin3_Set;                    //专变终端脉冲3设置参数
PULSE_SET   YaoXin4_Set;                    //专变终端脉冲4设置参数
PULSE_SET   YaoXin5_Set;                    //专变终端脉冲5设置参数
PULSE_SET   YaoXin6_Set;                    //专变终端脉冲6设置参数
PULSE_SET   YaoXin7_Set;                    //专变终端脉冲7设置参数
PULSE_SET   YaoXin8_Set;                    //专变终端脉冲8设置参数

