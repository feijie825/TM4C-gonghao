/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : define.h
;* Author             : 张力阵
;* 用户程序声明和预定义
;* 优先级规则 1.主机帧高于从机帧(主机D=0 从机D=1 ID中其他位相同情况下 )
;*            2.标准帧高于扩展帧
;*            3.长数据帧高于短数据帧
*******************************************************************************/
#include "Data_types.h"
#include "TM4C123_Lib.h"

#define CFG_BASE_ADDR 0xFC00                                         //最后一页 用于存放配置字 1K BYTES 大于1K时应增加扇区 最大空间32K BYTES
#define SAVE_TAB_NUMB ((1024 / ((sizeof(SAVE_S) + 3) & 0xFFFC)) - 1) //最多保存条数 4字节对齐 保留1

//CAN 长数据种类个数
#define MSG_OBJ_NUMB 13      //报文对象使用个数  要和LM3S21xx_CAN.C CAN_MSG_SET_TAB 表格长度一帧
#define CAN_LDT_TYPE_NUMB 10 //长数据总类最大个数
#define CAN_LDT_ACT_NUMB 5   //长数据总类实际个数 不大于CAN_LDT_TYPE_NUMB
//CAN ID 参与过滤码定义
#define IDX_MASK_CODE (0x07 << 0)      //帧编号参与过滤代码 3BIT
#define END_MASK_CODE (0x01 << 3)      //结束位参与过滤代码 1BIT
#define TYPE_MSAK_CODE (0x0F << 4)     //类型域参与过滤代码	4BIT
#define MNUM_MASK_CODE (0xff << 8)     //表位域参与过滤代码 8BIT
#define DIR_MASK_CODE (0x01 << 16)     //方向位参与过滤代码 1BIT
#define CMD_MASK_CODE (0x7ff << 17)    //命令域参与过滤代码 11BIT
#define EXD_MASK_CODE (0x01 << 28)     //ID中扩展帧标识符 	1 BIT 为了让标准帧优先级高于扩展帧 增加
#define H6BCMD_MASK_CODE (0x7E0 << 17) //前6位命令域参与过滤代码
//CAN 主机查询和回应状态定义
#define MST_CHK_RCVD 'C'  //已收到主机查询命令
#define SLV_ECHO_SEND 'S' //正在发送从机响应命令
#define SLV_ECHO_ACK 'A'  //从机响应命令成功发送
//从机 CAN长数据发送状态定义
#define SLV_LDATA_TX_NO 0     //从机长数据空闲状态
#define SLV_LDATA_TX_REQ 'R'  //从机长数据发送请求已发出 REQUEST
#define SLV_LDATA_TX_ACK 'A'  //从机长数据发送请求被批准 ACK
#define SLV_LDATA_RETX 'R'    //主机请求从机重发标志 Retransmit
#define SLV_LDATA_TX_IS 'S'   //从机正在发送长数据
#define SLV_LDATA_TX_FI 'F'   //单帧发送完成
#define SLV_LDATA_TX_LAST 'L' //从机正在发送最后一帧数据
#define SLV_LDATA_TX_END 'E'  //从机长数据发送结束
//从机 CAN长数据接收状态定义
#define SLV_LDATA_RX_NO 0    //空闲状态
#define SLV_LDATA_RX_IS 'S'  //从机长数据正在接收状态
#define SLV_LDATA_RX_END 'E' //从机长数据接收结束
#define SLV_LDATA_RX_OUT 'O' //接收到的数据正在处理
//CAN 短帧数据发送状态
#define SLV_SMSG_TX_IS 'S' //正在发送短帧
//串口接收状态定义
#define COM_RX_NO 0    //空闲状态
#define COM_RX_IS 'R'  //串口正在接收数据
#define COM_RX_END 'E' //串口数据接收结束
//串口发送状态定义
#define COM_TX_NO 0   //发送缓冲区空闲状态
#define COM_TX_IS 'O' //串口正在发送数据
#define COM_TX_EN 'E' //串口数据有效
//数据有效定义
#define DATA_YES 'Y'       //存储区有数据标志 表位号
#define DATA_VALIDE 'E'    //数据有效标志
#define DATA_BLANK 'B'     //数据空标志
#define DATA_NOT_BLACK 'N' //数据不空标志

//定义MSG RAM ID对应命令
// 11bit ID 和IDmask 格式 0CDXXXXXXXX 	 C=0~1	 D=0 主->从 D=1 从->主 XXXXXXXX=0~255 表位号
// 29bit ID 和IDmask 格式 1CCCCCCCCCCDXXXXXXXXTTTTEIII	CCCCCCCCCCC =0~2015 D同上 TTTT 长数据 类型 E 长数据结束 III 帧编号
//                         CCCCCCCCCC 命令 D 方向 XXXXXXXX 表位号 TTTT 长数据 类型 E 长数据结束 III 帧编号
//MST 表示主机发送 从机接收 SLV 表示从机发送 主机接收
/**************************************************************
* 帧类型    : 标准帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=0 广播        
* 命令功能  : 主机广播查询  
* 参与滤波位:      MM M MMMMMMMM
* 滤波屏蔽  : 开   0C D XXXXXXXX
* 命令格式  :      00 0 00000000 
* 命令优先级: 最高
* 命令号    : 0
* 广播/单播 : 广播
**************************************************************/
#define MST_CHK_BCAST 1 //在MSG RAM 中位置
#define MST_CHK_CMD 0   //主机查询命令编号
/**************************************************************
* 帧类型    : 标准帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=本机表号        
* 命令功能  : 主机单播查询  
* 参与滤波位:      MM M MMMMMMMM
* 滤波屏蔽  : 开   0C D XXXXXXXX
* 命令格式  :      00 0 XXXXXXXX 
* 命令优先级: 次高
* 命令号    : 0
* 广播/单播 : 单播
**************************************************************/
#define MST_CHK_SCAST 2
/**************************************************************
* 帧类型    : 标准帧
* 命令方向  : 主机<-从机  D=1
* 表位号    : XXXXXXXX=本机表号        
* 参与滤波位: 发送命令 无需设置滤波     
* 命令功能  : 从机回应主机查询  
* 滤波屏蔽  : 关   0C D XXXXXXXX
* 命令格式  :      00 1 XXXXXXXX 
* 命令优先级: 次高
* 命令号    : 0
* 广播/单播 : 单播
**************************************************************/
#define SLV_CHK_ECHO 3
#define SLV_ECHO_CMD 0 //从机响应命令
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=本机表号        
* 命令功能  : 主机批准从机发送长数据请求  
* 参与滤波位:          MMMMMMMMMMMM M MMMMMMMM			 M	MMM
* 滤波屏蔽  : 开       1CCCCCCCCCCC D XXXXXXXX   TTTT    E  III
* 命令格式  :          100000000000 0 XXXXXXXX 0000~1111 0	000
* 命令优先级: 扩展帧中最高 低于CHK_BCAST CHK_SCAST CHK_ECHO 标准帧
* 命令号    : 0
* 长数据类型: TTTT 0~15
* 长数据结束: E   0 该命令无意义
* 数据编号  : III 0 该命令无意义
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define MST_LDATA_ACK 4
#define MST_LDATA_ACK_CMD 0 //主机批准长数据发送命令
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=0        
* 命令功能  : 主机长广播数据发送 
* 参与滤波位:          MMMMMMMMMMMM M MMMMMMMM
* 滤波屏蔽  : 开       1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          100000000001 0 00000000 0000~1111 X XXX
* 命令优先级: 扩展帧中次高 
* 命令号    : 1
* 长数据类型: TTTT 0~15
* 长数据结束: E   0 长数据发送未结束 1长数据结束(最后一帧)
* 数据编号  : III 0 数据开始 1~7 数据编号 
* 广播/单播 : 广播
**************************************************************/
#define MST_LCDATA_TX 5
#define MST_LDATA_TX_CMD 1 //主机长数据发送命令
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=本机表号        
* 命令功能  : 主机长单播数据发送 
* 参与滤波位:          MMMMMMMMMMMM M MMMMMMMM
* 滤波屏蔽  : 开 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          100000000001 0 XXXXXXXX 0000~1111 X XXX
* 命令优先级: 扩展帧中次高 
* 命令号    : 1
* 长数据类型: TTTT 0~15
* 长数据结束: E   0 长数据发送未结束 1长数据结束(最后一帧)
* 数据编号  : III 0 数据开始 1~7 数据编号 
* 广播/单播 : 单播
**************************************************************/
#define MST_LSDATA_TX 6
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=本机表号        
* 命令功能  : 主机请求从机重发数据 当主机(总控中心)接收长数据块出错时执行
* 参与滤波位:          MMMMMMMMMMMM M MMMMMMMM			 M MMM
* 滤波屏蔽  : 开 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          100000000010 0 XXXXXXXX 0000~1111 0 000
* 命令优先级: 扩展帧中依次 
* 命令号    : 2
* 长数据类型: TTTT 0~15
* 长数据结束: E   0 该命令无意义
* 数据编号  : III 0 该命令无意义
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define MST_LDATA_REQRT 7
#define MST_LDATA_REQRT_CMD 2 //主机请求重发长数据命令
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=0        
* 命令功能  : 主机正在发送短广播数据串
* 参与滤波位:          M            M MMMMMMMM			 M
* 滤波屏蔽  : 开 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          1CCCCCCCCCCC 0 00000000 0000~1111 0 XXX
* 命令优先级: 扩展帧中依次 
* 命令号    : 4~2015 分组 分为7个256条指令组和一个220条指令组 每个指令组对应一个功能单元 
* 长数据类型: TTTT 0~15	 该命令预留
* 长数据结束: E   0      该命令无意义
* 数据编号  : III 0      该命令预留
* 广播/单播 : 广播
**************************************************************/
#define MST_SCDATA_TX 8    //LONG CAST DATA 单播数据串
#define MST_SDATA_TX_CMD 4 //主机发送短数据命令
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  D=0
* 表位号    : XXXXXXXX=本机表号         
* 命令功能  : 主机正在发送短单播数据串
* 参与滤波位:          M            M MMMMMMMM			 M
* 滤波屏蔽  : 开 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          1CCCCCCCCCCC 0 XXXXXXXX 0000~1111 0 XXX
* 命令优先级: 扩展帧中依次 
* 命令号    : 4~2015 分组 分为7个256条指令组和一个220条指令组 每个指令组对应一个功能单元 
* 长数据类型: TTTT 0~15	 该命令预留
* 长数据结束: E   0      该命令无意义
* 数据编号  : III 0      该命令预留
* 广播/单播 : 单播
**************************************************************/
#define MST_SSDATA_TX 9 //LONG SINGLE DATA 单播数据串

/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 从机-> 主机 D=1
* 表位号    : XXXXXXXX=本机表号        
* 命令功能  : 从机请求向主机发送长数据  
* 参与滤波位: 无
* 滤波屏蔽  : 关	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          100000000000 1 XXXXXXXX 0000~1111 0 000
* 命令优先级: 扩展帧中最高 低于CHK_BCAST CHK_SCAST CHK_ECHO 标准帧 
* 命令号    : 0
* 长数据类型: TTTT 0~15
* 长数据结束: E   0 该命令无意义
* 数据编号  : III 0 该命令无意义
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define SLV_LDATA_REQTX 10
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 从机-> 主机 D=1
* 表位号    : XXXXXXXX=本机表号        
* 命令功能  : 从机长数据发送 
* 参与滤波位: 无         
* 滤波屏蔽  : 关 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          100000000001 1 XXXXXXXX 0000~1111 X XXX
* 命令优先级: 扩展帧中次高 
* 命令号    : 1
* 长数据类型: TTTT 0~15
* 长数据结束: E   0 长数据发送未结束 1长数据结束(最后一帧)
* 数据编号  : III 0 数据开始 1~7 数据编号 
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define SLV_LDATA_TX 11
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 从机-> 主机 D=1
* 表位号    : XXXXXXXX=本机表号        
* 命令功能  : 从机请求主机重发数据 当从机接收长数据块出错时执行 
* 参与滤波位: 无         
* 滤波屏蔽  : 关 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          100000000010 1 XXXXXXXX 0000~1111 0 000
* 命令优先级: 扩展帧中次高 
* 命令号    : 2
* 长数据类型: TTTT 0~15
* 长数据结束: E   0 该命令无意义
* 数据编号  : III 0 该命令无意义
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define SLV_LDATA_REQRT 12
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 从机->主机  D=1
* 表位号    : XXXXXXXX=本机表号         
* 命令功能  : 从机正在发送短数据串
* 参与滤波位: 无
* 滤波屏蔽  : 关 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :          1CCCCCCCCCCC 1 XXXXXXXX 0000~1111 X XXX
* 命令优先级: 扩展帧中依次 
* 命令号    : 4~2015 分组 分为7个256条指令组和一个220条指令组 每个指令组对应一个功能单元 
* 长数据类型: TTTT 0~15	 该命令预留
* 长数据结束: E   0      该命令无意义
* 数据编号  : III 0      该命令预留
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define SLV_SDATA_TX 13 //SHORT  DATA 短数据串

//定义CAN1 MSG RAM ID对应命令
// 29bit ID 和IDmask 格式 1DCCCCCCCCCCXXXXXXXXDDDDDDDD	CCCCCCCCCCC =0~1983 D DDDDDDDD 设备类型号
//                         D 方向 CCCCCCCCCC 命令  XXXXXXXX 表位号
//MST 表示主机发送 从机接收 SLV 表示从机发送 主机接收
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  R=0
* 设备类型号: DDDDDDDD 电机单元 1 功耗单元 2
* 表位号    : XXXXXXXX=0        
* 命令功能  : 主机正在发送短广播数据串
* 参与滤波位:          MM             MMMMMMMM  DDDDDDDD               
* 滤波屏蔽  : 开       1DCCCCCCCCCCC  XXXXXXXX  11111111              
* 命令格式  :          10CCCCCCCCCCC  00000000  00000002              
* 命令优先级: 扩展帧中依次 
* 命令号    : 0~1983 
* 广播/单播 : 广播
**************************************************************/
#define CAN1_MST_SCDATA_TX 1 //广播数据串
/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 主机->从机  R=0
* 设备类型号: DDDDDDDD 电机单元 1 功耗单元 2
* 表位号    : XXXXXXXX=本机表号         
* 命令功能  : 主机正在发送短单播数据串
* 参与滤波位:          MM             MMMMMMMM DDDDDDDD
* 滤波屏蔽  : 开       1R CCCCCCCCCCC XXXXXXXX 11111111
* 命令格式  :          10 CCCCCCCCCCC XXXXXXXX 00000002
* 命令优先级: 扩展帧中依次 
* 命令号    : 0~1983 
* 广播/单播 : 单播
**************************************************************/
#define CAN1_MST_SSDATA_TX 2 //单播数据串

/**************************************************************
* 帧类型    : 扩展帧
* 命令方向  : 从机->主机  D=1
* 表位号    : XXXXXXXX=本机表号         
* 命令功能  : 从机正在发送短数据串
* 参与滤波位: 无
* 滤波屏蔽  : 关 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* 命令格式  :        1CCCCCCCCCCC 1 XXXXXXXX 0000~1111 X XXX
* 命令优先级: 扩展帧中依次 
* 命令号    : 4~2015 分组 分为7个256条指令组和一个220条指令组 每个指令组对应一个功能单元 
* 长数据类型: TTTT 0~15	 该命令预留
* 长数据结束: E   0      该命令无意义
* 数据编号  : III 0      该命令预留
* 广播/单播 : 单播(只有单播)
**************************************************************/
#define CAN1_SLV_SDATA_TX 3 //SHORT  DATA 短数据串
//定义SSI位速率
#define SSI_BITRATE 16667 //16.667K

#define NATIONAL 'N' //国网表
#define SOUTH 'S'    //南网表
#define Enable_Bit 1
#define Disable_Bit 0
#define LEVEL_MOD 0
#define PULSE_MOD 1
#define LOW_PUS_TRG 0
#define HIG_PUS_TRG 1

#define MTR_NO '0'  //没表
#define MTR_RDY '1' //表放好

#define LGT_OFF '0' //灯灭
#define YLW_ON '1'  //黄灯亮
#define GRN_ON '2'  //绿灯亮
#define RED_ON '3'  //红灯亮

#define LED_6 '6' //6位数码管
#define LED_8 '8' //8位数码管

//数据有效定义
#define YES 'Y'       //存储区有数据标志 表位号
#define VALIDE 'E'    //数据有效标志
#define BLANK 'B'     //数据空标志
#define NOT_BLACK 'N' //数据不空标志

#define LED_6 '6' //六位数码管
#define LED_8 '8' //八位数码管

#define UART_NUMB 5 //串口个数
//误差计算单元多功能端口
#define MTRCOM 0 //模拟表(多功能表)端口
#define LCTCOM 1 //负控终端端口
#define RTECOM 2 //负控终端RS485维护口
#define ATECOM 3 //负控终端RS232维护口
#define IRECOM 4 //红外头通讯口
//波特率定义 数据位
#define UART_5_BIT 0 //数据位长度 5bit
#define UART_6_BIT 1 //数据位长度 6bit
#define UART_7_BIT 2 //数据位长度 7bit
#define UART_8_BIT 3 //数据位长度 8bit
//波特率定义 停止位
#define UART_1_STOP 0 //1   stop bit
#define UART_2_STOP 1 //2   stop bit
//波特率定义 校验位
#define UART_N_PARITY 0 //无校验位
#define UART_O_PARITY 1 //ODD 奇校验
#define UART_E_PARITY 2 //EVEN 偶校验
#define UART_M_PARITY 3 //Mark 校验(1)
#define UART_S_PARITY 4 //Space校验(0)

#define UART0_BAUD 2400 //UART0默认波特率
#define UART1_BAUD 2400 //UART1默认波特率
#define UART3_BAUD 2400 //UART3默认波特率
#define UART5_BAUD 2400 //UART5默认波特率
#define UART6_BAUD 2400 //UART6默认波特率

#define MIN_BAUD 110    //最小波特率
#define MAX_BAUD 115200 //最大波特率

#define Wh_Wire '0'  //有功接线
#define Var_Wire '1' //无功接线

#define VLOSS_1 '1' //ΔU=100% 1S3次    与原装置协议相同
#define VLOSS_2 '2' //ΔU=100% 1个周期
#define VLOSS_3 '3' //ΔU=50%  1分钟

#define GB '1'  //挂表             与原装置协议不同
#define BGB '2' //不挂表

#define GD_E '1' //共发射集
#define GD_C '2' //共集电极

#define HC_2 '1' //合成两路
#define HC_3 '2' //合成三路
#define HC_4 '3' //合成四路

#define GDT_PLS '1' //光电头脉冲
#define DZ_PLS '2'  //电子脉冲
#define SZ_PLS '3'  //时钟脉冲
#define XUL_PLS '4' //需量脉冲
#define TQ_PLS '5'  //投切脉冲
#define HZ_PLS '6'  //合闸脉冲

#define EPLS_T '1' //测量电能脉冲周期
#define SZ_T '2'   //测量时钟脉冲周期
#define XUL_T '3'  //测量需量脉冲周期
#define TQ_T '4'   //投切脉冲
#define HZ_T '5'   //合闸脉冲

#define UNION_PLS '1' //联合多功能脉冲 时钟 需量 投切 等共用
#define ALONE_PLS '2' //独立多功能脉冲 时钟 需量 投切 分开输入
//只有在联合脉冲下有效
#define NO_PLS '0'     //默认无脉冲输入
#define SZCLK_PLS '1'  //当前输入为时钟脉冲
#define XULCLK_PLS '2' //当前输入为需量脉冲
#define TQCLK_PLS '3'  //当前输入为投切脉冲

#define PA_PLS '0' //正向有功
#define QA_PLS '1' //正向无功
#define PR_PLS '2' //反向有功
#define QR_PLS '3' //反向无功

#define NCATCH_HB '0' //黑斑未捕捉
#define CATCH_HB '1'  //黑斑已捕捉

#define ZZ_STRT '0' //走字开始
#define ZZ_END '1'  //走字结束

#define NY_BAD '0'  //耐压击穿
#define NY_GOOD '1' //耐压合格
#define NY_UNKW '2' //未知

#define MEA_STOP '0'  //停止测量
#define MEA_ORDER '1' //正常工作

#define UA_PHASE (1 << 0)                          //A相电压
#define UB_PHASE (1 << 1)                          //B相电压
#define UC_PHASE (1 << 2)                          //C相电压
#define ALL_PHASE (UA_PHASE | UB_PHASE | UC_PHASE) //三相电压

#define OFF 0 //断开
#define ON 1  //接入

#define DISP_TEST '0'  //显示试验状态
#define DISP_CFREQ '1' //显示时钟频率
#define DISP_CERR '2'  //显示日计时误差
#define DISP_XULT '3'  //显示需量周期
#define DISP_TQMC '4'  //显示投切脉冲

#define SUB_485 '1' //表第二485通道
#define RED_485 '2' //表红外485

#define MTR_UOP '0'  //表位电压断开
#define MTR_UCL '1'  //表位继电器控制电压吸合
#define MTR_UECL '2' //表位电子开关控制电压吸合

#define SINGLE 0 //单相台
#define THREE 1  //三相台
//接线方式定义
#define WIRE_P1 '0'    //接线方式 单相有功
#define WIRE_P4 '1'    //接线方式 三相四线有功
#define WIRE_P3_2 '2'  //接线方式 三相三线有功
#define WIRE_Q4_3 '3'  //接线方式 三相四线无功90度三元件无功
#define WIRE_Q3_60 '4' //接线方式 三相三线移相60度无功 60度两元件无功
#define WIRE_Q3_90 '5' //接线方式 三相三线跨线无功90度两元件无功
#define WIRE_Q4_R '6'  //接线方式 三相四线真无功
#define WIRE_Q3_R '7'  //接线方式 三相三线真无功
#define WIRE_P3_3 '8'  //接线方式 三相三线有功 UA UB UC 按三相四线输出UB对地不为0
#define WIRE_Q3_2 '9'  //接线方式 三相三线无功 输出按三相三线输出 UB接U0
#define WIRE_Q3_CT ':' //接线方式 三相三线人工中心点无功
#define WIRE_Q1 ';'    //单相方式 无功

#define MEA_PWR_P3 0 //测功耗 P3
#define MEA_PWR_P4 1 //测功耗 P4
//定义CAN命令表
#define MULTI_DATA_CMD 0   //0-3     长数据帧命令   4条
#define MTR_DATA_CMD 4     //4-259   原误差单元命令 256条
#define CLOCK_DATA_CMD 260 //260-515	时钟校验仪命令 256条
#define FKZD_DATA_CMD 516  //516-771 负控终端命令   256条
                           //772-2015 1244条指令预留
#define MTR_TAB_NUMB 128

#define WATCH_TIME 400 //1000    //看门狗周期单位:mS 毫秒
#define SYS_TIME 1000  //系统节拍定时周期 单位:uS 微秒

#define CAN_TX_OVTM 5000     //CAN发送超时5S
#define CAN_RX_OVTM 12000    //CAN接收超时18S
#define NY_SEND_TIME 2000    //命令发送定时
#define ZK2009_OVTM 10000    //ZK2009启动超时
#define CAN_LDATA_SERR_MAX 3 //CAN长数据最大发送错误计数 超过该计数值 清除有数据标志 可以接收下一帧

#define TIMER_8MS 8
#define NY_CHK_TIME (200 / TIMER_8MS)  //耐压检测定时
#define GDT_RST_TIME (200 / TIMER_8MS) //光电头复位延时 单位:ms
#ifdef PULSE
#define GDT_REN_TIME (400 / TIMER_8MS) //光电脉冲中断重新使能定时 400ms
#define DZ_REEN_TIME (400 / TIMER_8MS) //电子脉冲中断重新使能定时 400ms
#else
#define GDT_REN_TIME (40 / TIMER_8MS) //光电脉冲中断重新使能定时 40ms
#define DZ_REEN_TIME (40 / TIMER_8MS) //电子脉冲中断重新使能定时 40ms
#endif
#define SZ_REEN_TIME (40 / TIMER_8MS)  //时钟脉冲中断重新使能定时 200ms
#define XUL_REEN_TIME (40 / TIMER_8MS) //需量脉冲中断重新使能定时 200ms
#define TQ_REEN_TIME (40 / TIMER_8MS)  //投切脉冲中断重新使能定时 200ms
#define HZ_REEN_TIME (40 / TIMER_8MS)  //合闸脉冲中断重新使能定时 200ms

#define GDT_PLS_TIME 20  //光电脉冲消抖延时
#define DZ_PLS_TIME 20   //电子脉冲消抖延时
#define SZ_PLS_TIME 20   //时钟脉冲消抖延时
#define NZTZ_PLS_TIME 20 //开路信号消抖延时

#define KEY_REEN_TIME (200 / TIMER_8MS) //按键重启使能中断定时
#define KEY_PLUG_TIME (600 / TIMER_8MS) //按键挂表定时
#define PLL_CHK_TIME (200 / TIMER_8MS)  //锁相环PLL检查定时

#define MBJ_SEND_TIME 21000  //表报警回送定时
#define WZTZ_SEND_TIME 19000 //表跳闸回送定时
#define NZTZ_SEND_TIME 5000  //外置跳闸回送定时
#define MTR_ON_TIME 10000    //表位放好定时
#define GZ_SEND_TIME 3000    //定时回送故障
#define GZ_STB_TIME 1500     //故障稳定定时
#define HC165_TIME 20        //HC165采样定时
#define LUNCI1_SEND_TIME 100 //轮次1信息回送定时
#define LUNCI2_SEND_TIME 150 //轮次2信息回送定时
#define LUNCI3_SEND_TIME 200 //轮次3信息回送定时
#define LUNCI4_SEND_TIME 250 //轮次4信息回送定时
//#define CD4094_TIME           20               //CD4094延时处理

#define DISP_EN_TIME 15 //显示使能定时

#define POWER_UP_TIME 4000 //上电定时 上电稳定

#define STD_CLK_FREQ 500000 //标准时钟频率
//标准时钟脉冲计数器理论中断时间=0xFFFF*1000/STD_CLK_FREQ=131ms
#define STD_CLK_OVTM (1000 / 8) //单位:8ms 约1s
#define STD_ECLK_OVTM 20000     //单位:1ms 约20s

#define BACK_MEA_ERR_D '0' //回误差校验状态
#define CATCH_HB_FWEG '1'  //前沿对斑 ForWard  EDGE
#define CATCH_HB_BWEG '2'  //后沿对斑 BackWard EDGE

#define BEEP_ONN '1' //喇叭开
//口编号定义

#define SSI0 SSI0_BASE // SSI0
#define SSI1 SSI1_BASE // SSI1
#define SSI2 SSI2_BASE // SSI2
#define SSI3 SSI3_BASE // SSI3

#define WATCHDOG0 WATCHDOG0_BASE // Watchdog0
#define WATCHDOG1 WATCHDOG1_BASE // Watchdog1

#define CAN0 CAN0_BASE // CAN0  地址
#define CAN1 CAN1_BASE // CAN1  地址

#define UART0 UART0_BASE // UART0 地址
#define UART1 UART1_BASE // UART1 地址
#define UART2 UART2_BASE // UART2 地址
#define UART3 UART3_BASE // UART3 地址
#define UART4 UART4_BASE // UART4 地址
#define UART5 UART5_BASE // UART5 地址
#define UART6 UART6_BASE // UART6 地址
#define UART7 UART7_BASE // UART7 地址

#define TIMER0 TIMER0_BASE // Timer0
#define TIMER1 TIMER1_BASE // Timer1
#define TIMER2 TIMER2_BASE // Timer2
#define TIMER3 TIMER3_BASE // Timer3
#define TIMER4 TIMER4_BASE // Timer4
#define TIMER5 TIMER5_BASE // Timer5

#define PORTA 0  //PORTA 口编号定义
#define PORTB 1  //PORTB 口编号定义
#define PORTC 2  //PORTC 口编号定义
#define PORTD 3  //PORTD 口编号定义
#define PORTE 4  //PORTE 口编号定义
#define PORTF 5  //PORTF 口编号定义
#define PORTG 6  //PORTG 口编号定义
#define PORTH 7  //PORTH 口编号定义
#define PORTJ 8  //PORTJ 口编号定义
#define PORTK 9  //PORTK 口编号定义
#define PORTL 10 //PORTL 口编号定义
#define PORTM 11 //PORTM 口编号定义
#define PORTN 12 //PORTN 口编号定义
#define PORTP 13 //PORTP 口编号定义

#define GPIOA GPIO_PORTA_BASE //PORTA 口地址定义
#define GPIOB GPIO_PORTB_BASE //PORTB 口地址定义
#define GPIOC GPIO_PORTC_BASE //PORTC 口地址定义
#define GPIOD GPIO_PORTD_BASE //PORTD 口地址定义
#define GPIOE GPIO_PORTE_BASE //PORTE 口地址定义
#define GPIOF GPIO_PORTF_BASE //PORTF 口地址定义
#define GPIOG GPIO_PORTG_BASE //PORTG 口地址定义
#define GPIOH GPIO_PORTH_BASE //PORTH 口地址定义
#define GPIOJ GPIO_PORTJ_BASE //PORTJ 口地址定义
#define GPIOK GPIO_PORTK_BASE //PORTK 口地址定义
#define GPIOL GPIO_PORTL_BASE //PORTL 口地址定义
#define GPIOM GPIO_PORTM_BASE //PORTM 口地址定义
#define GPION GPIO_PORTN_BASE //PORTN 口地址定义
#define GPIOP GPIO_PORTP_BASE //PORTP 口地址定义

//DEFINE 管道对应端口
//GPIOA
#define UART0_GPIO GPIOA
#define SPI0_GPIO GPIOA
#define DISP_RST_GPIO GPIOA
#define KEY_IN_GPIO GPIOA

//GPIOB
#define YK1NO_GPIO GPIOB //遥控信号1常开
#define YK1NC_GPIO GPIOB //遥控信号1常闭
#define YK2NO_GPIO GPIOB //遥控信号2常开
#define YK2NC_GPIO GPIOB //遥控信号2常闭
#define I2C0_SCL_GPIO GPIOB
#define I2C0_SDA_GPIO GPIOB
#define I2C0_WP_GPIO GPIOB
// #define CAN0_GPIO             GPIOB    //CAN0
//GPIOC
#define JTAG_GPIO GPIOC  //调试JTAG/SWD
#define UART1_GPIO GPIOC //UART1
#define UART3_GPIO GPIOC //UART3
//GPIOD
#define UART6_GPIO GPIOD //UART6
/*TM4C_GH*/
#define U_5460A_SCLK_GPIO GPIOD //电压回路5460SCLK
#define U_5460A_CS_GPIO GPIOD   //电压回路5460CS
#define U_5460A_SDO_GPIO GPIOD  //电压回路5460SDO
#define U_5460A_SDI_GPIO GPIOD  //电压回路5460SDI

//GPIOE
#define UART7_GPIO GPIOE //UART7
#define TZ_ZS_GPIO GPIOE //跳闸指示灯控制
#define AIN0_GPIO GPIOE  //AD0转换引脚
#define UART5_GPIO GPIOE //UART5
#define CAN1_GPIO GPIOE  //CAN1
/*TM4C_GH*/
#define U_5460A_RST_GPIO GPIOE //电压回路5460RST

//GPIOF
#define UC_ESWC_GPIO GPIOF   //C相电压电子开关控制 三相台定义  UC_ESWC
#define UA_ESWC_GPIO GPIOF   //A相电压电子开关控制 三相台定义  UA_ESWC
#define UB_ESWC_GPIO GPIOF   //B相电压电子开关控制 三相台定义  UB_ESWC
#define GBC_C_GPIO GPIOF     //C相续流继电器断开控制
#define TEST_LAMP_GPIO GPIOF //装置校验指示灯控制
#define UC_JC_GPIO GPIOF     //C相电压继电器控制   三相台定义  UC_JC
#define UB_JC_GPIO GPIOF     //B相电压继电器控制   三相台定义 UB_JC   U2K_C	单相控制零线
#define UA_JC_GPIO GPIOF     //A相电压继电器控制   三相台定义 UA_JC   U1K_C	单相控制火线
/*TM4C_GH*/
#define CAN0_GPIO GPIOF //CAN0

//GPIOG
#define BW_GPIO GPIOG      //表位号
#define MC_OUT_GPIO GPIOG  //专变终端(2013版)脉冲输出
#define MC_OUT1_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出1
#define MC_OUT2_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出2
#define MC_OUT3_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出3
#define MC_OUT4_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出4
#define MC_OUT5_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出5
#define MC_OUT6_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出6
#define MC_OUT7_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出7
#define MC_OUT8_GPIO GPIOG //专变终端Ⅲ型(2013版)脉冲输出8
//PORTH
#define ABBA_GPIO GPIOH         //
#define I_5460A_INT_GPIO GPIOH  //电流回路5460INT
#define I_5460A_SCLK_GPIO GPIOH //电流回路5460SCLK
#define I_5460A_CS_GPIO GPIOH   //电流回路5460CS
#define I_5460A_SDO_GPIO GPIOH  //电流回路5460SDO
#define I_5460A_SDI_GPIO GPIOH  //电流回路5460SDI

//PORTJ
#define YX_CTL_GPIO GPIOJ  //专变终端(2013版)遥信控制端口
#define YX_CTL1_GPIO GPIOJ //专变终端(2013版)遥信控制端口1
#define YX_CTL2_GPIO GPIOJ //专变终端(2013版)遥信控制端口2
#define YX_CTL3_GPIO GPIOJ //专变终端(2013版)遥信控制端口3
#define YX_CTL4_GPIO GPIOJ //专变终端(2013版)遥信控制端口4
#define YX_CTL5_GPIO GPIOJ //专变终端(2013版)遥信控制端口5
#define YX_CTL6_GPIO GPIOJ //专变终端(2013版)遥信控制端口6
#define YX_CTL7_GPIO GPIOJ //专变终端(2013版)遥信控制端口7
#define YX_CTL8_GPIO GPIOJ //专变终端(2013版)遥信控制端口8
/*TM4C_GH*/
#define NO_SELT3_GPIO GPIOJ
#define P3P4SELOUT3_GPIO GPIOJ
#define NO_SELT2_GPIO GPIOJ
#define P3P4SELOUT2_GPIO GPIOJ
#define NO_SELT1_GPIO GPIOJ
#define P3P4SELOUT1_GPIO GPIOJ
#define K2_GPIO GPIOJ
#define K1_GPIO GPIOJ

//PORTK
#define YK4NO_GPIO GPIOK //遥控信号4常开
#define YK4NC_GPIO GPIOK //遥控信号4常闭
#define YK3NO_GPIO GPIOK //遥控信号3常开
#define YK3NC_GPIO GPIOK //遥控信号3常闭

//PORTL
#define HFIN1_GPIO GPIOL  //备用高频信号输入端口1 Timer0-A
#define HFIN2_GPIO GPIOL  //备用高频信号输入端口2 Timer0-B
#define HFIN3_GPIO GPIOL  //备用高频信号输入端口3 Timer1-A
#define FH_IN_GPIO GPIOL  //标准表高频输入 Timer1-B
#define JZ_IN_GPIO GPIOL  //高稳晶振输入接口 Timer1-B
#define QFH_IN_GPIO GPIOL //标准表无功高频脉冲 Timer2-B，谐波标准表预留
#define WGFMC_GPIO GPIOL  //被检表反向无功脉冲 Timer3-A
#define YGFMC_GPIO GPIOL  //被检表反向有功脉冲 Timer3-B

//PORTM
#define WGMC_GPIO GPIOM    //被检表正向无功脉冲 Timer4-A
#define SZ_MC_GPIO GPIOM   //被检表时钟脉冲 Timer4-B
#define YGMC_GPIO GPIOM    //被检表正向有功脉冲 Timer5-A
#define GDT_MC_GPIO GPIOM  //被检表光电头脉冲 Timer5-B
#define GDT_RST_GPIO GPIOM //光电头复位控制引脚
#define XL_MC_GPIO GPIOM   //被检表需量周期输入
#define TQ_MC_GPIO GPIOM   //被检表时段投切脉冲
#define GOG_KZ_GPIO GPIOM  //被检表脉冲共高共低选择控制

//PORTN
#define TTAB1_GPIO GPION   //双回路切换1
#define TTAB2_GPIO GPION   //双回路切换2
#define WDI_GPIO GPION     //外置看门狗控制引脚
#define OPEN_IN_GPIO GPION //续流保护继电器状态输入引脚
#define BGB_C_GPIO GPION   //三相续流继电器闭合控制
#define GBA_C_GPIO GPION   //A相续流继电器断开控制
#define GBB_C_GPIO GPION   //B相续流继电器断开控制
/*TM4C_GH*/
#define I_5460A_RST_GPIO GPION //电流回路5460RST
//PORTP
#define DOOR_GPIO GPIOP   //门控信号输出
#define BJ_SIG_GPIO GPIOP //报警信号输入
/*TM4C_GH*/
#define U_5460A_INT_GPIO GPIOP //电压回路5460INT

//管脚定义
//PORTA
#define U0RX GPIO_PIN_0     //UART0接收
#define U0TX GPIO_PIN_1     //UART0发送
#define SSICLK GPIO_PIN_2   //SSICLK HD7279 时钟
#define SSIFSS GPIO_PIN_3   //SSIFSS HD7279 片选
#define SSIRX GPIO_PIN_4    //SSIRX  HD7279 发送
#define SSITX GPIO_PIN_5    //SSITX  HD7279 接收
#define DISP_RST GPIO_PIN_6 //       HD7279 复位
#define KEY_IN GPIO_PIN_7   //按键输入

//PORTB
#define YK1NO GPIO_PIN_0 //遥控信号1常开
#define YK1NC GPIO_PIN_1 //遥控信号1常闭
#define YK2NO GPIO_PIN_2 //遥控信号2常开
#define YK2NC GPIO_PIN_3 //遥控信号2常闭
// #define CAN0RX                GPIO_PIN_4   //CAN0接收
// #define CAN0TX                GPIO_PIN_5   //CAN0发送
/*TM4C_GH*/
#define I2C0_WP GPIO_PIN_1
#define I2C0_SCL GPIO_PIN_2
#define I2C0_SDA GPIO_PIN_3

//PORTC
#define TCK GPIO_PIN_0  //调试TCK SWCLK SW调试
#define TMS GPIO_PIN_1  //调试TMS SWDIO SW调试
#define TDI GPIO_PIN_2  //调试TDI
#define TDO GPIO_PIN_3  //调试TDO SWO   SW调试
#define U1RX GPIO_PIN_4 //UART1接收
#define U1TX GPIO_PIN_5 //UART1发送
#define U3RX GPIO_PIN_6 //UART3接收
#define U3TX GPIO_PIN_7 //UART3发送
//PORTD
#define U_5460A_SCLK GPIO_PIN_0 //电压回路5460SCLK
#define U_5460A_CS GPIO_PIN_1   //电压回路5460CS
#define U_5460A_SDO GPIO_PIN_2  //电压回路5460SDO
#define U_5460A_SDI GPIO_PIN_3  //电压回路5460SDI
#define U6RX GPIO_PIN_4         //UART6接收
#define U6TX GPIO_PIN_5         //UART6发送

//单相台定义
#define ION_JC GPIO_PIN_1  //电流继电器通 单相台用 UB电压继电器控制信号
#define IOFF_JC GPIO_PIN_2 //电流继电器断 单相台用 UC电压继电器控制信号

//PORTE
#define U7RX GPIO_PIN_0   //UART7接收
#define U7TX GPIO_PIN_1   //UART7发送
#define TZ_ZS GPIO_PIN_2  //跳闸指示灯控制引脚
#define AIN0 GPIO_PIN_3   //AD0转换引脚
#define U5RX GPIO_PIN_4   //UART5接收
#define U5TX GPIO_PIN_5   //UART5发送
#define CAN1RX GPIO_PIN_6 //CAN1接收
#define CAN1TX GPIO_PIN_7 //CAN1发
/*TM4C_GH*/
#define U_5460A_RST GPIO_PIN_3 //电压回路5460RST

//PORTF
#define UC_ESWC GPIO_PIN_0   //C相电压电子开关控制 三相台定义  UC_ESWC
#define UA_ESWC GPIO_PIN_1   //A相电压电子开关控制 三相台定义  UA_ESWC
#define UB_ESWC GPIO_PIN_2   //B相电压电子开关控制 三相台定义  UB_ESWC
#define GBC_C GPIO_PIN_3     //C相续流继电器断开控制引脚
#define TEST_LAMP GPIO_PIN_4 //校验指示灯控制引脚
#define UC_JC GPIO_PIN_5     //C相电压继电器控制   三相台定义 UC_JC   U3K_C
#define UB_JC GPIO_PIN_6     //B相电压继电器控制   三相台定义 UB_JC   U2K_C	单相控制零线
#define UA_JC GPIO_PIN_7     //A相电压继电器控制   三相台定义 UA_JC   U1K_C	单相控制火线
/*TM4C_GH*/
#define CAN0RX GPIO_PIN_0 //CAN0接收
#define CAN0TX GPIO_PIN_3 //CAN0发送

//PORTG
#define BW (GPIO_PIN_0 | GPIO_PIN_1 | \
            GPIO_PIN_2 | GPIO_PIN_3 | \
            GPIO_PIN_4 | GPIO_PIN_5 | \
            GPIO_PIN_6 | GPIO_PIN_7)
#define MC_OUT (GPIO_PIN_0 | GPIO_PIN_1 | \
                GPIO_PIN_2 | GPIO_PIN_3 | \
                GPIO_PIN_4 | GPIO_PIN_5 | \
                GPIO_PIN_6 | GPIO_PIN_7)
#define MC_OUT1 GPIO_PIN_0 //脉冲输出1
#define MC_OUT2 GPIO_PIN_1 //脉冲输出2
#define MC_OUT3 GPIO_PIN_2 //脉冲输出3
#define MC_OUT4 GPIO_PIN_3 //脉冲输出4
#define MC_OUT5 GPIO_PIN_4 //脉冲输出5
#define MC_OUT6 GPIO_PIN_5 //脉冲输出6
#define MC_OUT7 GPIO_PIN_6 //脉冲输出7
#define MC_OUT8 GPIO_PIN_7 //脉冲输出8
//PORTH
#define ABBA GPIO_PIN_0 //RS485极性翻转控制引脚
/*TM4C_GH*/
#define I_5460A_INT GPIO_PIN_3  //电流回路5460INT
#define I_5460A_SCLK GPIO_PIN_4 //电流回路5460SCLK
#define I_5460A_CS GPIO_PIN_5   //电流回路5460CS
#define I_5460A_SDO GPIO_PIN_6  //电流回路5460SDO
#define I_5460A_SDI GPIO_PIN_7  //电流回路5460SDI

//PORTJ
#define YX_CTL (GPIO_PIN_0 | GPIO_PIN_1 | \
                GPIO_PIN_2 | GPIO_PIN_3 | \
                GPIO_PIN_4 | GPIO_PIN_5 | \
                GPIO_PIN_6 | GPIO_PIN_7)
#define YX_CTL1 GPIO_PIN_0 //专变终端(2013版)遥信控制端口1
#define YX_CTL2 GPIO_PIN_1 //专变终端(2013版)遥信控制端口2
#define YX_CTL3 GPIO_PIN_2 //专变终端(2013版)遥信控制端口3
#define YX_CTL4 GPIO_PIN_3 //专变终端(2013版)遥信控制端口4
#define YX_CTL5 GPIO_PIN_4 //专变终端(2013版)遥信控制端口5
#define YX_CTL6 GPIO_PIN_5 //专变终端(2013版)遥信控制端口6
#define YX_CTL7 GPIO_PIN_6 //专变终端(2013版)遥信控制端口7
#define YX_CTL8 GPIO_PIN_7 //专变终端(2013版)遥信控制端口8
/*TM4C_GH*/
#define NO_SELT3 GPIO_PIN_0
#define P3P4SELOUT3 GPIO_PIN_1
#define NO_SELT2 GPIO_PIN_2
#define P3P4SELOUT2 GPIO_PIN_3
#define NO_SELT1 GPIO_PIN_4
#define P3P4SELOUT1 GPIO_PIN_5
#define K2 GPIO_PIN_6
#define K1 GPIO_PIN_7

//PORTK
#define YK4NO GPIO_PIN_4 //遥控信号4常开
#define YK4NC GPIO_PIN_5 //遥控信号5常闭
#define YK3NC GPIO_PIN_6 //遥控信号6常闭
#define YK3NO GPIO_PIN_7 //遥控信号7常开

//PORTL
#define HFIN1 GPIO_PIN_0  //备用高频信号输入端口1 Timer0-A
#define HFIN2 GPIO_PIN_1  //备用高频信号输入端口2 Timer0-B
#define HFIN3 GPIO_PIN_2  //备用高频信号输入端口3 Timer1-A
#define FH_IN GPIO_PIN_3  //标准表有功高频输入 Timer1-B
#define JZ_IN GPIO_PIN_4  //高稳晶振输入接口 Timer2-A
#define QFH_IN GPIO_PIN_5 //标准表无功高频输入 Timer2-B
#define WGFMC GPIO_PIN_6  //被检表反向无功脉冲 Timer3-A
#define YGFMC GPIO_PIN_7  //被检表反向有功脉冲 Timer3-B

//PORTM
#define WGMC GPIO_PIN_0    //被检表正向无功脉冲 Timer4-A
#define SZ_MC GPIO_PIN_1   //被检表时钟脉冲 Timer4-B
#define YGMC GPIO_PIN_2    //被检表正向有功脉冲 Timer5-A
#define GDT_MC GPIO_PIN_3  //被检表光电头脉冲 Timer5-B
#define GDT_RST GPIO_PIN_4 //光电头复位控制引脚
#define XL_MC GPIO_PIN_5   //改变* 需量周期输入
#define TQ_MC GPIO_PIN_6   //备用 改变* 时段投切脉冲
#define GOG_KZ GPIO_PIN_7  //被检表脉冲共高共低选择控制

//PORTN
#define TTAB1 GPIO_PIN_1   //双回路切换1
#define TTAB2 GPIO_PIN_0   //双回路切换2
#define WDI GPIO_PIN_3     //外置看门狗控制引脚
#define OPEN_IN GPIO_PIN_4 //续流保护继电器状态输入引脚
#define BGB_C GPIO_PIN_5   //三相续流继电器闭合控制引脚
#define GBA_C GPIO_PIN_6   //A相续流继电器断开控制引脚
#define GBB_C GPIO_PIN_7   //B相续流继电器断开控制引脚
/*TM4C_GH*/
#define I_5460A_RST GPIO_PIN_2 //电流回路5460RST
//PORTP
#define DOOR GPIO_PIN_0    //门控信号
#define BJ_SIG GPIO_PIN_1  //报警信号
#define PWM_DAC GPIO_PIN_2 //PWM模拟输出

#define U_IN_CTL GPIO_PIN_7 //电压接入控制 1 接入1号端子(默认) 0:接入3号端子 某些单相台使用 与B相电子开关控制复用
/*TM4C_GH*/
#define U_5460A_INT GPIO_PIN_2 //电压回路5460INT

//I/O口宏定义
//PORTA
#define DISP_RST_EN GPIOPinWrite(DISP_RST_GPIO, DISP_RST, 0);       //复位管脚置0
#define DISP_RST_DN GPIOPinWrite(DISP_RST_GPIO, DISP_RST, DISP_RST) //复位管脚置1

//PORTB
#define YK1NO_SIG_IN GPIOPinRead(YK1NO_GPIO, YK1NO) //遥控信号1常开(电表外置跳闸信号常开)
#define YK1NC_SIG_IN GPIOPinRead(YK1NC_GPIO, YK1NC) //遥控信号1常闭(电表外置跳闸信号常闭)
#define YK2NO_SIG_IN GPIOPinRead(YK2NO_GPIO, YK2NO) //遥控信号2常开
#define YK2NC_SIG_IN GPIOPinRead(YK2NC_GPIO, YK2NC) //遥控信号2常闭

//PORTC

//PORTD
#define U_5460A_SCLK_H GPIOPinWrite(U_5460A_SCLK_GPIO, U_5460A_SCLK, U_5460A_SCLK)
#define U_5460A_SCLK_L GPIOPinWrite(U_5460A_SCLK_GPIO, U_5460A_SCLK, 0)
#define U_5460A_CS_H GPIOPinWrite(U_5460A_CS_GPIO, U_5460A_CS, U_5460A_CS)
#define U_5460A_CS_L GPIOPinWrite(U_5460A_CS_GPIO, U_5460A_CS, 0)
#define U_5460A_SDI_H GPIOPinWrite(U_5460A_SDI_GPIO, U_5460A_SDI, U_5460A_SDI)
#define U_5460A_SDI_L GPIOPinWrite(U_5460A_SDI_GPIO, U_5460A_SDI, 0)
//PORTE
#define TZZS_LED_ON GPIOPinWrite(TZ_ZS_GPIO, TZ_ZS, TZ_ZS) //跳闸指示灯打开
#define TZZS_LED_OFF GPIOPinWrite(TZ_ZS_GPIO, TZ_ZS, 0)    //跳闸指示灯关闭
#define U_5460A_RST_H GPIOPinWrite(U_5460A_RST_GPIO, U_5460A_RST, U_5460A_RST)
#define U_5460A_RST_L GPIOPinWrite(U_5460A_RST_GPIO, U_5460A_RST, 0)
//PORTF
#define UC_ESW_OFF GPIOPinWrite(UC_ESWC_GPIO, UC_ESWC, 0)               //C相电压电子开关断开
#define UC_ESW_ON GPIOPinWrite(UC_ESWC_GPIO, UC_ESWC, UC_ESWC)          //C相电压电子开关接入
#define UB_ESW_OFF GPIOPinWrite(UB_ESWC_GPIO, UB_ESWC, 0)               //B相电压电子开关断开
#define UB_ESW_ON GPIOPinWrite(UB_ESWC_GPIO, UB_ESWC, UB_ESWC)          //B相电压电子开关接入
#define UA_ESW_OFF GPIOPinWrite(UA_ESWC_GPIO, UA_ESWC, 0)               //A相电压电子开关断开
#define UA_ESW_ON GPIOPinWrite(UA_ESWC_GPIO, UA_ESWC, UA_ESWC)          //A相电压电子开关接入
#define ICJDQ_OPEN GPIOPinWrite(GBC_C_GPIO, GBC_C, GBC_C)               //C相续流继电器断开控制
#define ICJDQ_OPEN_CANCEL GPIOPinWrite(GBC_C_GPIO, GBC_C, 0)            //C相续流继电器断开控制撤销
#define TEST_LAMP_ON GPIOPinWrite(TEST_LAMP_GPIO, TEST_LAMP, TEST_LAMP) //校验指示灯亮
#define TEST_LAMP_OFF GPIOPinWrite(TEST_LAMP_GPIO, TEST_LAMP, 0)        //校验指示灯灭
#define UC_JDQ_ON GPIOPinWrite(UC_JC_GPIO, UC_JC, 0)                    //C相电压继电器接入
#define UC_JDQ_OFF GPIOPinWrite(UC_JC_GPIO, UC_JC, UC_JC)               //C相电压继电器断开
#define UB_JDQ_ON GPIOPinWrite(UB_JC_GPIO, UB_JC, 0)                    //B相电压继电器接入
#define UB_JDQ_OFF GPIOPinWrite(UB_JC_GPIO, UB_JC, UB_JC)               //B相电压继电器断开
#define UA_JDQ_ON GPIOPinWrite(UA_JC_GPIO, UA_JC, 0)                    //A相电压继电器接入
#define UA_JDQ_OFF GPIOPinWrite(UA_JC_GPIO, UA_JC, UA_JC)               //A相电压继电器断开
//PORTG
#define MC_OUT1_LOW GPIOPinWrite(MC_OUT1_GPIO, MC_OUT1, 0)                                       //脉冲信号1输出低
#define MC_OUT1_HIGH GPIOPinWrite(MC_OUT1_GPIO, MC_OUT1, MC_OUT1)                                //脉冲信号1输出高
#define MC_OUT2_LOW GPIOPinWrite(MC_OUT2_GPIO, MC_OUT2, 0)                                       //脉冲信号2输出低
#define MC_OUT2_HIGH GPIOPinWrite(MC_OUT2_GPIO, MC_OUT2, MC_OUT2)                                //脉冲信号2输出高
#define MC_OUT3_LOW GPIOPinWrite(MC_OUT3_GPIO, MC_OUT3, 0)                                       //脉冲信号3输出低
#define MC_OUT3_HIGH GPIOPinWrite(MC_OUT3_GPIO, MC_OUT3, MC_OUT3)                                //脉冲信号3输出高
#define MC_OUT4_LOW GPIOPinWrite(MC_OUT4_GPIO, MC_OUT4, 0)                                       //脉冲信号4输出低
#define MC_OUT4_HIGH GPIOPinWrite(MC_OUT4_GPIO, MC_OUT4, MC_OUT4)                                //脉冲信号4输出高
#define MC_OUT5_LOW GPIOPinWrite(MC_OUT5_GPIO, MC_OUT5, 0)                                       //脉冲信号5输出低
#define MC_OUT5_HIGH GPIOPinWrite(MC_OUT5_GPIO, MC_OUT5, MC_OUT5)                                //脉冲信号5输出高
#define MC_OUT6_LOW GPIOPinWrite(MC_OUT6_GPIO, MC_OUT6, 0)                                       //脉冲信号6输出低
#define MC_OUT6_HIGH GPIOPinWrite(MC_OUT6_GPIO, MC_OUT6, MC_OUT6)                                //脉冲信号6输出高
#define MC_OUT7_LOW GPIOPinWrite(MC_OUT7_GPIO, MC_OUT7, 0)                                       //脉冲信号7输出低
#define MC_OUT7_HIGH GPIOPinWrite(MC_OUT7_GPIO, MC_OUT7, MC_OUT7)                                //脉冲信号7输出高
#define MC_OUT8_LOW GPIOPinWrite(MC_OUT8_GPIO, MC_OUT8, 0)                                       //脉冲信号8输出低
#define MC_OUT8_HIGH GPIOPinWrite(MC_OUT8_GPIO, MC_OUT8, MC_OUT8)                                //脉冲信号8输出高
#define MC_OUT1_Reverse GPIOPinWrite(MC_OUT1_GPIO, MC_OUT1, ~GPIOPinRead(MC_OUT1_GPIO, MC_OUT1)) //脉冲信号1反转
#define MC_OUT2_Reverse GPIOPinWrite(MC_OUT2_GPIO, MC_OUT2, ~GPIOPinRead(MC_OUT2_GPIO, MC_OUT2)) //脉冲信号2反转
#define MC_OUT3_Reverse GPIOPinWrite(MC_OUT3_GPIO, MC_OUT3, ~GPIOPinRead(MC_OUT3_GPIO, MC_OUT3)) //脉冲信号3反转
#define MC_OUT4_Reverse GPIOPinWrite(MC_OUT4_GPIO, MC_OUT4, ~GPIOPinRead(MC_OUT4_GPIO, MC_OUT4)) //脉冲信号4反转
#define MC_OUT5_Reverse GPIOPinWrite(MC_OUT5_GPIO, MC_OUT5, ~GPIOPinRead(MC_OUT5_GPIO, MC_OUT5)) //脉冲信号5反转
#define MC_OUT6_Reverse GPIOPinWrite(MC_OUT6_GPIO, MC_OUT6, ~GPIOPinRead(MC_OUT6_GPIO, MC_OUT6)) //脉冲信号6反转
#define MC_OUT7_Reverse GPIOPinWrite(MC_OUT7_GPIO, MC_OUT7, ~GPIOPinRead(MC_OUT7_GPIO, MC_OUT7)) //脉冲信号7反转
#define MC_OUT8_Reverse GPIOPinWrite(MC_OUT8_GPIO, MC_OUT8, ~GPIOPinRead(MC_OUT8_GPIO, MC_OUT8)) //脉冲信号8反转
//PORTH
#define ABBA_HIGH GPIOPinWrite(ABBA_GPIO, ABBA, ABBA) //默认RS485极性
#define ABBA_LOW GPIOPinWrite(ABBA_GPIO, ABBA, 0)     //翻转RS485极性
#define I_5460A_SCLK_H GPIOPinWrite(I_5460A_SCLK_GPIO, I_5460A_SCLK, I_5460A_SCLK)
#define I_5460A_SCLK_L GPIOPinWrite(I_5460A_SCLK_GPIO, I_5460A_SCLK, 0)
#define I_5460A_CS_H GPIOPinWrite(I_5460A_CS_GPIO, I_5460A_CS, I_5460A_CS)
#define I_5460A_CS_L GPIOPinWrite(I_5460A_CS_GPIO, I_5460A_CS, 0)
#define I_5460A_SDI_H GPIOPinWrite(I_5460A_SDI_GPIO, I_5460A_SDI, I_5460A_SDI)
#define I_5460A_SDI_L GPIOPinWrite(I_5460A_SDI_GPIO, I_5460A_SDI, 0)
//PORTJ
#define YX_CTL1_LOW GPIOPinWrite(YX_CTL1_GPIO, YX_CTL1, 0)
#define MMMMMMMMMM mmmmmmmmm                                                                     //不加此行后面部分代码不按颜色主题显示（VSCODE中）                                      //遥信信号1输出低
#define YX_CTL1_HIGH GPIOPinWrite(YX_CTL1_GPIO, YX_CTL1, YX_CTL1)                                //遥信信号1输出高
#define YX_CTL2_LOW GPIOPinWrite(YX_CTL2_GPIO, YX_CTL2, 0)                                       //遥信信号2输出低
#define YX_CTL2_HIGH GPIOPinWrite(YX_CTL2_GPIO, YX_CTL2, YX_CTL2)                                //遥信信号2输出高
#define YX_CTL3_LOW GPIOPinWrite(YX_CTL3_GPIO, YX_CTL3, 0)                                       //遥信信号3输出低
#define YX_CTL3_HIGH GPIOPinWrite(YX_CTL3_GPIO, YX_CTL3, YX_CTL3)                                //遥信信号3输出高
#define YX_CTL4_LOW GPIOPinWrite(YX_CTL4_GPIO, YX_CTL4, 0)                                       //遥信信号4输出低
#define YX_CTL4_HIGH GPIOPinWrite(YX_CTL4_GPIO, YX_CTL4, YX_CTL4)                                //遥信信号4输出高
#define YX_CTL1_Reverse GPIOPinWrite(YX_CTL1_GPIO, YX_CTL1, ~GPIOPinRead(YX_CTL1_GPIO, YX_CTL1)) //遥信信号1反转
#define YX_CTL2_Reverse GPIOPinWrite(YX_CTL2_GPIO, YX_CTL2, ~GPIOPinRead(YX_CTL2_GPIO, YX_CTL2)) //遥信信号2反转
#define YX_CTL3_Reverse GPIOPinWrite(YX_CTL3_GPIO, YX_CTL3, ~GPIOPinRead(YX_CTL3_GPIO, YX_CTL3)) //遥信信号3反转
#define YX_CTL4_Reverse GPIOPinWrite(YX_CTL4_GPIO, YX_CTL4, ~GPIOPinRead(YX_CTL4_GPIO, YX_CTL4)) //遥信信号4反转
#define NO_SELT3_H GPIOPinWrite(NO_SELT3_GPIO, NO_SELT3, NO_SELT3)
#define NO_SELT3_L GPIOPinWrite(NO_SELT3_GPIO, NO_SELT3, 0)
#define P3P4SELOUT3_H GPIOPinWrite(P3P4SELOUT3_GPIO, P3P4SELOUT3, P3P4SELOUT3)
#define P3P4SELOUT3_L GPIOPinWrite(P3P4SELOUT3_GPIO, P3P4SELOUT3, 0)
#define NO_SELT2_H GPIOPinWrite(NO_SELT2_GPIO, NO_SELT2, NO_SELT2)
#define NO_SELT2_L GPIOPinWrite(NO_SELT2_GPIO, NO_SELT2, 0)
#define P3P4SELOUT2_H GPIOPinWrite(P3P4SELOUT2_GPIO, P3P4SELOUT2, P3P4SELOUT2)
#define P3P4SELOUT2_L GPIOPinWrite(P3P4SELOUT2_GPIO, P3P4SELOUT2, 0)
#define NO_SELT1_H GPIOPinWrite(NO_SELT1_GPIO, NO_SELT31 NO_SELT1)
#define NO_SELT1_L GPIOPinWrite(NO_SELT1_GPIO, NO_SELT1, 0)
#define P3P4SELOUT1_H GPIOPinWrite(P3P4SELOUT1_GPIO, P3P4SELOUT1, P3P4SELOUT1)
#define P3P4SELOUT1_L GPIOPinWrite(P3P4SELOUT1_GPIO, P3P4SELOUT1, 0)
#define K2_H GPIOPINWRITE(K2_GPIO, K2, K2)
#define K2_L GPIOPINWRITE(K2_GPIO, K2, 0)
#define K1_H GPIOPINWRITE(K1_GPIO, K1, K1)
#define K1_L GPIOPINWRITE(K1_GPIO, K1, 0)
//PORTK
#define YK3NO_SIG_IN GPIOPinRead(YK3NO_GPIO, YK3NO) //遥控信号3常开
#define YK3NC_SIG_IN GPIOPinRead(YK3NC_GPIO, YK3NC) //遥控信号3常闭
#define YK4NO_SIG_IN GPIOPinRead(YK4NO_GPIO, YK4NO) //遥控信号4常开
#define YK4NC_SIG_IN GPIOPinRead(YK4NC_GPIO, YK4NC) //遥控信号4常闭
//PORTM
#define GDT_RST_EN GPIOPinWrite(GDT_RST_GPIO, GDT_RST, GDT_RST) //光电头复位使能 高电平使能
#define GDT_RST_DN GPIOPinWrite(GDT_RST_GPIO, GDT_RST, 0)       //光电头复位禁能 低电平禁能
#define DOWN_JOIN GPIOPinWrite(GOG_KZ_GPIO, GOG_KZ, GOG_KZ)     //共低端 发射极连在一起 E
#define UP_JOIN GPIOPinWrite(GOG_KZ_GPIO, GOG_KZ, 0)            //共高端 集电极连在一起 C
#define DZMC_IN GPIOPinRead(DZ_MC_GPIO, DZ_MC)                  //电子脉冲数据输入
#define SZMC_IN GPIOPinRead(SZ_MC_GPIO, SZ_MC)                  //时钟脉冲数据输入
#define GDTMC_IN GPIOPinRead(GDT_MC_GPIO, GDT_MC)               //光电头脉冲数据输入
//PORTN
#define I_5460A_RST_H GPIOPinWrite(I_5460A_RST_GPIO, I_5460A_RST, I_5460A_RST)
#define I_5460A_RST_L GPIOPinWrite(I_5460A_RST_GPIO, I_5460A_RST, 0)
#define TTAB1JDQ_CLOSE GPIOPinWrite(TTAB1_GPIO, TTAB1, TTAB1) //电流回路1继电器接入
#define TTAB1JDQ_CANCEL GPIOPinWrite(TTAB1_GPIO, TTAB1, 0)    //电流回路1继电器撤销
#define TTAB2JDQ_CLOSE GPIOPinWrite(TTAB2_GPIO, TTAB2, TTAB2) //电流回路2继电器接入
#define TTAB2JDQ_CANCEL GPIOPinWrite(TTAB2_GPIO, TTAB2, 0)    //电流回路2继电器撤销
#define WDI_HIGH GPIOPinWrite(WDI_GPIO, WDI, WDI)             //输出 外部看门狗复位
#define WDI_LOW GPIOPinWrite(WDI_GPIO, WDI, 0)                //输出
#define IJDQ_CLOSE GPIOPinWrite(BGB_C_GPIO, BGB_C, BGB_C)     //续流继电器闭合控制
#define IJDQ_CLOSE_CANCEL GPIOPinWrite(BGB_C_GPIO, BGB_C, 0)  //续流继电器闭合控制撤销
#define IAJDQ_OPEN GPIOPinWrite(GBA_C_GPIO, GBA_C, GBA_C)     //A相续流继电器断开控制
#define IAJDQ_OPEN_CANCEL GPIOPinWrite(GBA_C_GPIO, GBA_C, 0)  //A相续流继电器断开控制撤销
#define IBJDQ_OPEN GPIOPinWrite(GBB_C_GPIO, GBB_C, GBB_C)     //B相续流继电器断开控制
#define I_5460A_RST_H GPIOPinWrite(I_5460A_RST_GPIO, I_5460A_RST, I_5460A_RST)
#define I_5460A_RST_L GPIOPinWrite(I_5460A_RST_GPIO, I_5460A_RST, 0)
//PORTP
#define DOOR_SIG_OPEN GPIOPinWrite(DOOR_GPIO, DOOR, DOOR) //门控信号开
#define DOOR_SIG_CLOSE GPIOPinWrite(DOOR_GPIO, DOOR, 0)   //门控信号闭
#define BJ_SIG_IN GPIOPinRead(BJ_SIG_GPIO, BJ_SIG)        //表报警信号输入
//CAN缓冲区长度定义
#define CAN_SDILEN 50 //CAN短数据接收缓冲区长度	每个结构体长16字节 8字节有效数据
#define CAN_SDOLEN 50 //CAN短数据发送缓冲区长度	每个结构体长16字节 8字节有效数据

#define CAN_LDILEN 20 //CAN长数据接收缓冲区长度	每个结构体长16字节 8字节有效数据
#define CAN_LDOLEN 10 //CAN长数据发送缓冲区长度	每个结构体长16字节 8字节有效数据

//多功能表
#define MTRCOM_ILEN 2048 //COM0接收缓冲区长度
#define MTRCOM_OLEN 1024 //COM0发送缓冲区长度
#define LCTCOM_ILEN 2048 //COM1接收缓冲区长度
#define LCTCOM_OLEN 1024 //COM1发送缓冲区长度
#define RTECOM_ILEN 2048 //COM3接收缓冲区长度
#define RTECOM_OLEN 1024 //COM3发送缓冲区长度
#define ATECOM_ILEN 2048 //COM5接收缓冲区长度
#define ATECOM_OLEN 1024 //COM5发送缓冲区长度
#define IRECOM_ILEN 2048 //COM6接收缓冲区长度
#define IRECOM_OLEN 1024 //COM6发送缓冲区长度

//CAN 波特率设置 与CANBitClkSettings[]中对应
#define CANBAUD_100K 100000
#define CANBAUD_125K 125000
#define CANBAUD_250K 250000
#define CANBAUD_500K 500000
#define CANBAUD_1M 1000000
#define CANBAUD CANBAUD_500K
//误差单元短CAN帧指令分块 每256个命令为一组
#define ERR_CMD_ID 0 //误差计算单元原有命令 CAN 命令编号(0x004~0x103)
#define CLK_CMD_ID 1 //时钟校验仪命令       CAN 命令编号(0x104~0x203)
#define LCT_CMD_ID 2 //负控终端指令         CAN 命令编号(0x204~0x303)
//MTR_MOD被检设备类型定义
#define Acquire_Terminal_13 '0' //专变终端Ⅲ型(2013版)标识
#define Reading_Terminal_13 '1' //集中器Ⅰ型(2013版)标识
#define Smart_Meter '2'         //智能电表
#define Acquire_Terminal_09 '3' //专变终端Ⅲ型(2009版)标识
#define Reading_Terminal_09 '4' //集中器Ⅰ型(2009版)标识
//原误差板命令								   //0 1 2 3  预留给长数据处理
#define ERR_ICMD_GD (ERR_CMD_ID * 0x100 + 4) //接收标准值     XX 表号 XXXX 数据 0---3为长数据命令
#define ERR_ICMD_GE (ERR_ICMD_GD + 1)        //接收校验圈数
#define ERR_ICMD_GF (ERR_ICMD_GE + 1)        //接收误差清零
#define ERR_ICMD_GG (ERR_ICMD_GF + 1)        //接收光电头对光
#define ERR_ICMD_GH (ERR_ICMD_GG + 1)        //选表位 挂表选择
#define ERR_ICMD_GI (ERR_ICMD_GH + 1)        //对黑斑         ;0退去其他试验，回到校验状态
#define ERR_ICMD_GJ (ERR_ICMD_GI + 1)        //监视光电头脉冲 ;1起动/潜动时记数 0退去其他试验，回到校验状态
#define ERR_ICMD_GK (ERR_ICMD_GJ + 1)        //接收当前脉冲象限
#define ERR_ICMD_GL (ERR_ICMD_GK + 1)        //喇叭开关
#define ERR_ICMD_GM (ERR_ICMD_GL + 1)        //准备开始走字试验
#define ERR_ICMD_GN (ERR_ICMD_GM + 1)        //开始进行走字试验
#define ERR_ICMD_GO (ERR_ICMD_GN + 1)        //接收被校表常数
#define ERR_ICMD_GP (ERR_ICMD_GO + 1)        //脉冲选择  电子脉冲还是光电头脉冲
#define ERR_ICMD_GQ (ERR_ICMD_GP + 1)        //计电能试验      1:开始  0:终止
#define ERR_ICMD_GR (ERR_ICMD_GQ + 1)        //清除电能计数
#define ERR_ICMD_GS (ERR_ICMD_GR + 1)        //脉冲比较  电子脉冲和光电头脉冲比较
#define ERR_ICMD_GT (ERR_ICMD_GS + 1)        //接收脉冲和盘转比值
#define ERR_ICMD_GU (ERR_ICMD_GT + 1)        //接收脉冲和盘转设定比较圈数
#define ERR_ICMD_GV (ERR_ICMD_GU + 1)        //进入失压状态,接收失压方案
#define ERR_ICMD_GW (ERR_ICMD_GV + 1)        //失压试验开始
#define ERR_ICMD_GX (ERR_ICMD_GW + 1)        //功耗试验或控制时钟脉冲和需量周期脉冲转换
#define ERR_ICMD_GY (ERR_ICMD_GX + 1)        //送分频系数
#define ERR_ICMD_GZ (ERR_ICMD_GY + 1)        //计读脉冲法常数测试试验
#define ERR_ICMD_Gd (ERR_ICMD_GZ + 1)        //功耗测试 表位和相别选择
#define ERR_ICMD_Ge (ERR_ICMD_Gd + 1)        //功耗测试仪 电压线圈有功修正值
#define ERR_ICMD_Gf (ERR_ICMD_Ge + 1)        //电压线圈视在功率修正值
#define ERR_ICMD_Gg (ERR_ICMD_Gf + 1)        //接收电流线圈视在功率修正值
#define ERR_ICMD_Gh (ERR_ICMD_Gg + 1)        //设置功耗测量单元单元号
#define ERR_ICMD_Gi (ERR_ICMD_Gh + 1)        //脉冲波形测试
#define ERR_ICMD_Gj (ERR_ICMD_Gi + 1)        //接收接线方式
#define ERR_ICMD_Gk (ERR_ICMD_Gj + 1)        //4059被校表脉冲输出类型选择
#define ERR_ICMD_Gl (ERR_ICMD_Gk + 1)        //设置走字脉冲数
#define ERR_ICMD_Gm (ERR_ICMD_Gl + 1)        //脉冲走字试验
#define ERR_ICMD_Gn (ERR_ICMD_Gm + 1)        //预置脉冲数
//误差单元接收命令(多功能走字耐压台通讯命令)
#define ERR_ICMD_Go (ERR_ICMD_Gn + 1)           //开始耐压试验
#define ERR_ICMD_Gp (ERR_ICMD_Go + 1)           //耐压时间到
#define ERR_ICMD_Gq (ERR_ICMD_Gp + 1)           //切换表尾电压端子
#define ERR_ICMD_Gr (ERR_ICMD_Gq + 1)           //误差校验指示灯控制命令
#define ERR_ICMD_Gs (ERR_ICMD_Gr + 1)           //继电器吸合
#define ERR_ICMD_Gt (ERR_ICMD_Gs + 1)           //继电器断开
#define ERR_ICMD_Gu (ERR_ICMD_Gt + 1)           //电流回路开路检测
#define ERR_ICMD_SYPH (ERR_ICMD_Gu + 1)         //电压跌落相别
#define ERR_ICMD_MIN_CST (ERR_ICMD_SYPH + 1)    //最小电表常数
#define ERR_ICMD_ZZDS (ERR_ICMD_MIN_CST + 1)    //接收校核常数度数
#define ERR_ICMD_MCHC (ERR_ICMD_ZZDS + 1)       //接收脉冲合成方式
#define ERR_ICMD_SYFAN (ERR_ICMD_MCHC + 1)      //接收失压方案
#define ERR_ICMD_MFCLKMD (ERR_ICMD_SYFAN + 1)   //设置多功能脉冲方式  共用还是分开
#define ERR_ICMD_MFCLKTY (ERR_ICMD_MFCLKMD + 1) //当前多功能脉冲类型  时钟脉冲 需量脉冲 投切脉冲 ...
#define ERR_ICMD_ELEPLS (ERR_ICMD_MFCLKTY + 1)  //设置电子脉冲输入 调试用 PA PR QA QR '1' '2' '3' '4'
#define ERR_ICMD_IJDQR (ERR_ICMD_ELEPLS + 1)    //电流旁路继电器复位
#define ERR_ICMD_LEDTST (ERR_ICMD_IJDQR + 1)    //测试命令显示8.8.8.8.8.8.8.8.
#define ERR_ICMD_RSTLED (ERR_ICMD_LEDTST + 1)   //复位7279
#define ERR_ICMD_TZEN (ERR_ICMD_RSTLED + 1)     //合闸检测使能命令
#define ERR_ICMD_MBJEN (ERR_ICMD_TZEN + 1)      //表报警信号检测使能
#define ERR_ICMD_TS (ERR_ICMD_MBJEN + 1)        //单三相台发送命令
#define ERR_ICMD_DXTZ (ERR_ICMD_TS + 1)         //设置单相跳闸
#define ERR_ICMD_START (ERR_ICMD_DXTZ + 1)      //总控中心启动命令
#define ERR_ICMD_SETCK (ERR_ICMD_START + 1)     //设置插卡
#define ERR_ICMD_UCLOP (ERR_ICMD_SETCK + 1)     //设置表位电压接入 断开
#define ERR_ICMD_ZBTST (ERR_ICMD_UCLOP + 1)     //载表试验
#define ERR_ICMD_LIGHT (ERR_ICMD_ZBTST + 1)     //误差板状态指示灯控制
#define ERR_ICMD_SLEDN (ERR_ICMD_LIGHT + 1)     //设置误差板LED数码管显示位数
#define ERR_ICMD_VSET (ERR_ICMD_SLEDN + 1)      //设置载波虚拟表电压接入相别
#define ERR_ICMD_COM0SEL (ERR_ICMD_VSET + 1)    //设置接入UART0的通讯方式
#define ERR_ICMD_COM1SEL (ERR_ICMD_COM0SEL + 1) //设置接入UART1的通讯方式
#define ERR_ICMD_ZBLBIN (ERR_ICMD_COM1SEL + 1)  //载波滤波电路接入选择
#define ERR_ICMD_YXCTL (ERR_ICMD_ZBLBIN + 1)    //遥信信号控制命令
#define ERR_ICMD_DOORCTL (ERR_ICMD_YXCTL + 1)   //门控信号控制命令
#define ERR_ICMD_EQPMOD (ERR_ICMD_DOORCTL + 1)  //检测设备类型设置
#define ERR_ICMD_PULSET (ERR_ICMD_EQPMOD + 1)   //脉冲参数设置命令(针对专变终端)
#define ERR_ICMD_PULRUN (ERR_ICMD_PULSET + 1)   //脉冲输出设置命令(针对专变终端)
#define ERR_ICMD_YKSET (ERR_ICMD_PULRUN + 1)    //终端遥控信号设置
#define ERR_ICMD_YKPULSE (ERR_ICMD_YKSET + 1)   //设置遥控信号脉冲触发方式
#define ERR_ICMD_NPRS485 (ERR_ICMD_YKPULSE + 1) //测试无极性RS485
#define ERR_ICMD_MTYPE (ERR_ICMD_NPRS485 + 1)   //表类型 国网表 'N'  南网表'S'
#define ERR_ICMD_ADCONV (ERR_ICMD_MTYPE + 1)    //12V直流电AD转换控制
#define ERR_ICMD_ULVL (ERR_ICMD_ADCONV + 1)     //设置电压选择70%or100%
#define ERR_ICMD_WZTZCTL (ERR_ICMD_ULVL + 1)    //外置跳闸信号端子继电器控制‘O’断开‘C’闭合
#define ERR_ICMD_TTAB (ERR_ICMD_WZTZCTL + 1)    //单相装置电流双回路切换
#define ERR_ICMD_MEAPN (ERR_ICMD_TTAB + 1)      //并行功耗测试控制
#define ERR_ICMD_CHKP (ERR_ICMD_MEAPN + 1)      //并行功耗各项数据查询
//误差单元接收命令(时钟校验仪部分命令)
#define ERR_ICMD_MBAUD (CLK_CMD_ID * 0x100 + 4) //设置多功能表(模拟表)串口通信参数
#define ERR_ICMD_LBAUD (ERR_ICMD_MBAUD + 1)     //设置负控终端串口通信参数
#define ERR_ICMD_CLKFRQ (ERR_ICMD_LBAUD + 1)    //设置各表位时钟频率
#define ERR_ICMD_CLKTIM (ERR_ICMD_CLKFRQ + 1)   //设置该表位时钟频率测量时间
#define ERR_ICMD_CLKCTL (ERR_ICMD_CLKTIM + 1)   //时钟频率测量控制
#define ERR_ICMD_XULCTL (ERR_ICMD_CLKCTL + 1)   //需量周期测量控制
#define ERR_ICMD_XULPLS (ERR_ICMD_XULCTL + 1)   //设置需量周期测量个数
#define ERR_ICMD_EDIS (ERR_ICMD_XULPLS + 1)     //设置显示方式
#define ERR_ICMD_RLMDT (ERR_ICMD_EDIS + 1)      //重新装载模拟表数据     上位机   -> 模拟表
#define ERR_ICMD_RLLDT (ERR_ICMD_RLMDT + 1)     //重新装载负控终端数据   上位机   -> 负控终端
#define ERR_ICMD_RTMDT (ERR_ICMD_RLLDT + 1)     //重发收到的模拟表数据   模拟表   -> 上位机
#define ERR_ICMD_RTLDT (ERR_ICMD_RTMDT + 1)     //重发收到的负控终端数据 负控终端 -> 上位机
#define ERR_ICMD_SEC (ERR_ICMD_RTLDT + 1)       //时间基准 秒信号
#define ERR_ICMD_RED485 (ERR_ICMD_SEC + 1)      //设置RS485第二通道 '1' 接表485第二通道 '2' 接表红外
#define ERR_ICMD_READ_VER (ERR_ICMD_RED485 + 1) //读版本号命令
#define ERR_ICMD_RSTS (ERR_ICMD_READ_VER + 1)   //读小显示工作状态
#define ERR_ICMD_SMTR (ERR_ICMD_RSTS + 1)       //设置监视仪表标准表 兼容协议 被检表误差板未用
#define ERR_ICMD_RBAUD (ERR_ICMD_SMTR + 1)      //RTECOM串口通信参数
#define ERR_ICMD_ABAUD (ERR_ICMD_RBAUD + 1)     //ATECOM串口通信参数
#define ERR_ICMD_IBAUD (ERR_ICMD_ABAUD + 1)     //IRECOM串口通信参数
//负控终端命令 LCT_CMD_ID*0x100+4
#define TX_MTR_DATA (LCT_CMD_ID * 0x100 + 4) //发送模拟表数据到负控终端
#define TX_LCT_DATA (TX_MTR_DATA + 1)        //发送数据到负控终端

//保留命令
#define JTAG_EN 0x7DF //2015 最后一个命令
//回送原误差板数据命令
#define ERR_OCMD_CB (ERR_CMD_ID * 0x100 + 4) //  XXXXXX(0DH)  回送被校表脉冲数(累加值)
#define ERR_OCMD_CC (ERR_OCMD_CB + 1)        //  X(0DH)       起动、潜动试验时圈数
#define ERR_OCMD_CE (ERR_OCMD_CC + 1)        //  ±XXXX(0DH)   误差
#define ERR_OCMD_CF (ERR_OCMD_CE + 1)        //  ±XXX(0DH)    盘转误差圈数
#define ERR_OCMD_CG (ERR_OCMD_CF + 1)        //  X(0DH)       X:0未挂表X:1 挂表
#define ERR_OCMD_CH (ERR_OCMD_CG + 1)        //  :XXX(0DH)    湿度
#define ERR_OCMD_CI (ERR_OCMD_CH + 1)        //  ±XXXX(0DH)   XXXX误差
#define ERR_OCMD_CK (ERR_OCMD_CI + 1)        //  1(0DH)       累积脉冲试验停止
#define ERR_OCMD_CM (ERR_OCMD_CK + 1)        //  XXXXXX,XXXXXX(0DH) 送出被校表脉冲周期, 占空比
                                             //  前面的数据是：被校表脉冲高电平占用时间
                                             //  后面的数据是：被校表脉冲低电平占用时间
#define ERR_OCMD_CN (ERR_OCMD_CM + 1)        //  XX(0DH)      XX当前圈数
#define ERR_OCMD_CP (ERR_OCMD_CN + 1)        //  XXXXXX(0DH)  XXX.XXX被校表电能
#define ERR_OCMD_CS (ERR_OCMD_CP + 1)        //  X(0DH)       X:0,未对好斑,X:1斑已对好
#define ERR_OCMD_CT (ERR_OCMD_CS + 1)        //  :XXX(0DH)    温度
#define ERR_OCMD_CV (ERR_OCMD_CT + 1)        //  X（0DH）     X=0：未知 X=1：开路X=2：不开路
#define ERR_OCMD_CW (ERR_OCMD_CV + 1)        //  :XXXXXX,XXXXXXX,XXXXXX   功耗测试数据
#define ERR_OCMD_CX (ERR_OCMD_CW + 1)        //  X(0DH)       耐压结果
#define ERR_OCMD_CY (ERR_OCMD_CX + 1)        //  X(0DH)       回送失压试验接收脉冲个数
#define ERR_OCMD_CZ (ERR_OCMD_CY + 1)        //  X(0DH)       X=0: 走字试验开始,X=1: 走字试验结束

//回送时钟数据命令
#define ERR_OCMD_CLK_FRQ (CLK_CMD_ID * 0x100 + 4) //  回送被检表时钟频率
#define ERR_OCMD_DAY_ERR (ERR_OCMD_CLK_FRQ + 1)   //  回送日计时误差
#define ERR_OCMD_XULZQ (ERR_OCMD_DAY_ERR + 1)     //  回送需量周期
#define ERR_OCMD_TQMC (ERR_OCMD_XULZQ + 1)        //  收到时段投切脉冲
#define ERR_OCMD_NZTZ (ERR_OCMD_TQMC + 1)         //  内置跳闸信号状态
#define ERR_OCMD_WZTZ (ERR_OCMD_NZTZ + 1)         //  外置跳闸信号状态
#define ERR_OCMD_JDQGZ (ERR_OCMD_WZTZ + 1)        //  续流继电器故障状态
#define ERR_OCMD_MBJ (ERR_OCMD_JDQGZ + 1)         //  表报警输出状态
#define ERR_OCMD_VER (ERR_OCMD_MBJ + 1)           //  回送误差板版本号
#define ERR_OCMD_STS (ERR_OCMD_VER + 1)           //  回送误差板工作状态
#define ERR_OCMD_BERR (ERR_OCMD_STS + 1)          //  回送标准表误差(给副标准表 误差计算预留)
#define ERR_OCMD_DZPLS (ERR_OCMD_BERR + 1)        //  检测到电子脉冲
#define ERR_OCMD_GDPLS (ERR_OCMD_DZPLS + 1)       //  检测到光电脉冲
#define ERR_OCMD_SZPLS (ERR_OCMD_GDPLS + 1)       //  检测到时钟脉冲
#define ERR_OCMD_MTRDY (ERR_OCMD_SZPLS + 1)       //  有无表回送命令
#define ERR_OCMD_LUNCI1 (ERR_OCMD_MTRDY + 1)      //  轮次1控制状态回送名令
#define ERR_OCMD_LUNCI2 (ERR_OCMD_LUNCI1 + 1)     //  轮次2控制状态回送名令
#define ERR_OCMD_LUNCI3 (ERR_OCMD_LUNCI2 + 1)     //  轮次3控制状态回送名令
#define ERR_OCMD_LUNCI4 (ERR_OCMD_LUNCI3 + 1)     //  轮次4控制状态回送名令
#define ERR_OCMD_ADCONV (ERR_OCMD_LUNCI4 + 1)     //  直流12VAD转换结果上传
//定义AD采样超时时间
#define AD_OVER_TIME 100  //单位:1ms  100ms 正常125k 64次平均512us
#define AD_TRIG_TIME 1000 //1秒采样1次
