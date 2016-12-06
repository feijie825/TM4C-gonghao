/******************** (C) COPYRIGHT 2014 TI TM4C123xx ********************
;* File Name          : Usr_Lib.c
;* Author             : 张力阵
;* 用户函数库 官方库 扩展
*******************************************************/			   
#include "Function.h"
#include "string.h"
/*****************************************************************************
* 设置CAN MSG 接收RAM区过滤设置
* 入口:CANx CAN结构体
* 长数据 命令 CMD=3预留
*****************************************************************************/
void Set_MsgRam(u32 CANx)
{
    CAN_MSG_SET RX_MSG_SET;    //接收报文
/*********设置主机广播查询命令过滤 MSG RAM 编号MST_CHK_BCAST(1) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_CHK_BCAST-1],sizeof(CAN_MSG_SET));
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机单播查询命令过滤 MSG RAM 编号MST_CHK_SCAST(2) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_CHK_SCAST-1],sizeof(CAN_MSG_SET));
    RX_MSG_SET.ID.BIT.MNUM = Mtr_Numb_ID;               //仲裁域	表位号
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机批准从机发送长数据请求过滤 MSG RAM 编号MST_LDATA_ACK(4)   *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_LDATA_ACK-1],sizeof(CAN_MSG_SET));
    RX_MSG_SET.ID.BIT.MNUM = Mtr_Numb_ID;               //仲裁域	表位号	CMD=0
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机发送长广播数据串 过滤  MSG RAM 编号MST_LCDATA_TX(5) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_LCDATA_TX-1],sizeof(CAN_MSG_SET));
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机发送长单播数据串 过滤  MSG RAM 编号MST_LSDATA_TX(6) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_LSDATA_TX-1],sizeof(CAN_MSG_SET));
    RX_MSG_SET.ID.BIT.MNUM = Mtr_Numb_ID;               //仲裁域	表位号
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机请求从机重发数据 过滤  MSG RAM 编号MST_LDATA_REQ(7) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_LDATA_REQRT-1],sizeof(CAN_MSG_SET));
    RX_MSG_SET.ID.BIT.MNUM = Mtr_Numb_ID;               //仲裁域	表位号
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机发送短广播数据串 过滤  MSG RAM 编号MST_SCDATA_TX(8) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_SCDATA_TX-1],sizeof(CAN_MSG_SET));
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机正在发送短单播数据串 过滤  MSG RAM 编号MST_SSDATA_TX(9) *********/
    memcpy(&RX_MSG_SET,&CAN_MSG_SET_TAB[MST_SSDATA_TX-1],sizeof(CAN_MSG_SET));
    RX_MSG_SET.ID.BIT.MNUM = Mtr_Numb_ID;               //仲裁域	表位号
    CAN_RxMsg_Set(CANx, &RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
}
/*****************************************************************************
* 设置CAN MSG 接收RAM区过滤设置
* 入口:CANx CAN结构体
* 长数据 命令 CMD=3预留
*****************************************************************************/
void Set_CAN1MsgRam(u32 CANx)
{
    CAN1_MSG_SET CAN1_RX_MSG_SET;    //接收报文
/*********设置主机发送短广播数据串 过滤  MSG RAM 编号MST_SCDATA_TX(8) *********/
    memcpy(&CAN1_RX_MSG_SET,&CAN1_MSG_SET_TAB[CAN1_MST_SCDATA_TX-1],sizeof(CAN1_MSG_SET));
    CAN1_RxMsg_Set(CANx, &CAN1_RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)
/*********设置主机正在发送短单播数据串 过滤  MSG RAM 编号MST_SSDATA_TX(9) *********/
    memcpy(&CAN1_RX_MSG_SET,&CAN1_MSG_SET_TAB[MST_SSDATA_TX-1],sizeof(CAN1_MSG_SET));
    CAN1_RX_MSG_SET.ID.BIT.MNUM = Mtr_Numb_ID;               //仲裁域	表位号
    CAN1_RxMsg_Set(CANx, &CAN1_RX_MSG_SET,MSG_OBJ_TYPE_RX);   //入口(CAN接口基址,报文对象编号,报文结构体地址,报文对象类型)	
}
/*****************************************************************************
* 喂狗程序 
*****************************************************************************/
void WDTFeed(void)
{
    WatchdogReloadSet(WATCHDOG0,
	                    WatchdogReloadGet(WATCHDOG0));    //写入重装值 更新看门狗当前计时器
}
/*****************************************************************************
* 写CAN数据寄存器
* 入口:pucData 待发送数据指针
* 入口:pulRegister 数据寄存器地址
* 入口:iSize 数据长度
* 实际发送数据长度受发送DLC控制位控制
* 
* ! \internal
* ! Copies data from a buffer to the CAN Data registers.
* !
* ! \param pui8Data is a pointer to the data to be written out to the CAN
* ! controller's data registers.
* ! \param pui32Register is an uint32_t pointer to the first register of the
* ! CAN controller's data registers.  For example, in order to use the IF1
* ! register set on CAN controller 0, the value would be: \b CAN0_BASE \b +
* ! \b CAN_O_IF1DA1.
* ! \param iSize is the number of bytes to copy into the CAN controller.
* !
* ! This function takes the steps necessary to copy data from a contiguous
* ! buffer in memory into the non-contiguous data registers used by the CAN
* ! controller.  This function is rarely used outside of the CANMessageSet()
* ! function.
* !
* ! \return None.
*****************************************************************************/
void CANDataRegWrite(uint8_t *pui8Data, uint32_t *pui32Register, uint32_t ui32Size)
{
    uint32_t ui32Idx, ui32Value;

    //
    // Loop always copies 1 or 2 bytes per iteration.
    //
    for(ui32Idx = 0; ui32Idx < ui32Size; )
    {
        //
        // Write out the data 16 bits at a time since this is how the registers
        // are aligned in memory.
        //
        ui32Value = pui8Data[ui32Idx++];

        //
        // Only write the second byte if needed otherwise it will be zero.
        //
        if(ui32Idx < ui32Size)
        {
            ui32Value |= (pui8Data[ui32Idx++] << 8);
        }

        HWREG(pui32Register++) = ui32Value;
    }
}
/*****************************************************************************
* 读CAN数据寄存器
* 入口:IntNumber CAN中断号
* 入口:pucData 待返回数据的指针
* 入口:pulRegister 数据寄存器地址指针
* 入口:iSize 数据长度
* 实际接收数据长度受DLC控制位控制
* 
* ! \internal
* ! Copies data from a buffer to the CAN Data registers.
* !
* ! \param pui8Data is a pointer to the location to store the data read from
* ! the CAN controller's data registers.
* ! \param pui32Register is an uint32_t pointer to the first register of the
* ! CAN controller's data registers.  For example, in order to use the IF1
* ! register set on CAN controller 1, the value would be: \b CAN0_BASE \b +
* ! \b CAN_O_IF1DA1.
* ! \param iSize is the number of bytes to copy from the CAN controller.
* !
* ! This function takes the steps necessary to copy data to a contiguous buffer
* ! in memory from the non-contiguous data registers used by the CAN
* ! controller.  This function is rarely used outside of the CANMessageGet()
* ! function.
* !
* ! \return None.
* 
*****************************************************************************/
void CANDataRegRead(uint8_t *pui8Data, uint32_t *pui32Register, uint32_t ui32Size)
{
    uint32_t ui32Idx, ui32Value;

    //
    // Loop always copies 1 or 2 bytes per iteration.
    //
    for(ui32Idx = 0; ui32Idx < ui32Size; )
    {
        //
        // Read out the data 16 bits at a time since this is how the registers
        // are aligned in memory.
        //
        ui32Value = HWREG(pui32Register++);

        //
        // Store the first byte.
        //
        pui8Data[ui32Idx++] = (uint8_t)ui32Value;

        //
        // Only read the second byte if needed.
        //
        if(ui32Idx < ui32Size)
        {
            pui8Data[ui32Idx++] = (uint8_t)(ui32Value >> 8);
        }
    }
}

/*****************************************************************************
* 设置 CAN0 接收报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针 
* 入口: eMsgType 报文类型
* 关闭接收到远程帧自动回送功能 ZLZ 2008.11.27
*****************************************************************************/
void CAN_RxMsg_Set(u32 CANx, CAN_MSG_SET *CAN_MSG, tMsgObjType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usMaskReg[2]={0,0};
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_RX) ||
           (eMsgType == MSG_OBJ_TYPE_RX_REMOTE) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE) ||)
    while(HWREG(CANx + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY)
     {                                    //等待接口寄存器空闲
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //方向 INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //允许访问MSG 数据A
                    CAN_IF1CMSK_DATAB |   //允许访问MSG 数据B
                    CAN_IF1CMSK_CONTROL|  //允许访问MSG 控制位
					CAN_IF1CMSK_ARB);     //允许访问MSG 仲裁位
    switch(eMsgType)
     {                    //根据报文类型跳转
         case MSG_OBJ_TYPE_RX:
          {                //接收数据帧
           usArbReg[1] = 0;//设置方向位为接收数据帧
           break;
          }
         case MSG_OBJ_TYPE_RX_REMOTE:
          {                //接收远程帧
           usArbReg[1] = CAN_IF1ARB2_DIR;    //设置方向位为接收远程帧
           usMsgCtrl   = CAN_IF1MCTL_UMASK;  //访问屏蔽位
           usMaskReg[0] = 0xffff;            //远程帧默认ID过滤使能
           usMaskReg[1] = 0x1fff;            //远程帧默认ID过滤使能
           usCmdMaskReg |= CAN_IF1CMSK_MASK; //访问仲裁位
           break;
          }
         case MSG_OBJ_TYPE_RXTX_REMOTE:
          {                //接收远程帧后自动回送
           usArbReg[1] = CAN_IF1ARB2_DIR;
           usMsgCtrl = CAN_IF1MCTL_RMTEN | CAN_IF1MCTL_UMASK;
           break;
          }
         default:
          return;              //非法 退出
     }
     if(CAN_MSG->CTL.BIT.ID_FLT_EN )//判断是否使用ID过滤屏蔽
      {                        //使用ID过滤屏蔽
       if(CAN_MSG->CTL.BIT.EXD_ID)
        {                      //扩展帧	29bit ID
          usMaskReg[0] = CAN_MSG->IDMask.WORD & CAN_IF1MSK1_IDMSK_M;
          usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 16) & CAN_IF1MSK2_IDMSK_M);
        }
       else
        {                      //标准帧  
         usMaskReg[0] = 0;
         usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 6 ) & CAN_IF1MSK2_IDMSK_M);
        }
      }
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.EXT_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MXTD;  //使用扩展ID过滤
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.DIR_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MDIR;  //传输方向用于过滤
    if(CAN_MSG->CTL.BIT.ID_FLT_EN  ||
       CAN_MSG->CTL.BIT.EXT_FLT_EN ||
       CAN_MSG->CTL.BIT.DIR_FLT_EN)
     {
      usMsgCtrl |= CAN_IF1MCTL_UMASK;
      usCmdMaskReg |= CAN_IF1CMSK_MASK;
     }
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //扩展帧 29BIT ID
      usArbReg[0] |= CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD;
     }
    else
     {                      //标准帧
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL;
     }
    usMsgCtrl |= (CAN_MSG->CTL.BIT.LEN  | CAN_IF1MCTL_EOB); //设置控制位 数据长度 单次发送
    if(CAN_MSG->CTL.BIT.RX_INT_EN )                         //判断接收中断是否使能
     usMsgCtrl |= CAN_IF1MCTL_RXIE;                         //接收中断使能        
    HWREG(CANx + CAN_O_IF1CMSK) = usCmdMaskReg;             //写命令屏蔽寄存器
    HWREG(CANx + CAN_O_IF1MSK1) = usMaskReg[0];             //写屏蔽寄存器1
    HWREG(CANx + CAN_O_IF1MSK2) = usMaskReg[1];             //写屏蔽寄存器2
    HWREG(CANx + CAN_O_IF1ARB1) = usArbReg[0];              //写仲裁寄存器1
    HWREG(CANx + CAN_O_IF1ARB2) = usArbReg[1];              //写仲裁寄存器2
    HWREG(CANx + CAN_O_IF1MCTL) = usMsgCtrl;                //写报文控制寄存器
    HWREG(CANx + CAN_O_IF1CRQ)  = (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M);//INF->MSG RAM
}

/*****************************************************************************
* 设置 CAN1 接收报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针 
* 入口: eMsgType 报文类型
* 关闭接收到远程帧自动回送功能 ZLZ 2008.11.27
*****************************************************************************/
void CAN1_RxMsg_Set(u32 CANx, CAN1_MSG_SET *CAN_MSG, tMsgObjType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usMaskReg[2]={0,0};
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_RX) ||
           (eMsgType == MSG_OBJ_TYPE_RX_REMOTE) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE) ||)
    while(HWREG(CANx + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY)
     {                                    //等待接口寄存器空闲
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //方向 INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //允许访问MSG 数据A
                    CAN_IF1CMSK_DATAB |   //允许访问MSG 数据B
                    CAN_IF1CMSK_CONTROL|  //允许访问MSG 控制位
					CAN_IF1CMSK_ARB);     //允许访问MSG 仲裁位
    switch(eMsgType)
     {                    //根据报文类型跳转
         case MSG_OBJ_TYPE_RX:
          {                //接收数据帧
           usArbReg[1] = 0;//设置方向位为接收数据帧
           break;
          }
         case MSG_OBJ_TYPE_RX_REMOTE:
          {                //接收远程帧
           usArbReg[1] = CAN_IF1ARB2_DIR;    //设置方向位为接收远程帧
           usMsgCtrl   = CAN_IF1MCTL_UMASK;  //访问屏蔽位
           usMaskReg[0] = 0xffff;            //远程帧默认ID过滤使能
           usMaskReg[1] = 0x1fff;            //远程帧默认ID过滤使能
           usCmdMaskReg |= CAN_IF1CMSK_MASK; //访问仲裁位
           break;
          }
         case MSG_OBJ_TYPE_RXTX_REMOTE:
          {                //接收远程帧后自动回送
           usArbReg[1] = CAN_IF1ARB2_DIR;
           usMsgCtrl = CAN_IF1MCTL_RMTEN | CAN_IF1MCTL_UMASK;
           break;
          }
         default:
          return;              //非法 退出
     }
     if(CAN_MSG->CTL.BIT.ID_FLT_EN )//判断是否使用ID过滤屏蔽
      {                        //使用ID过滤屏蔽
       if(CAN_MSG->CTL.BIT.EXD_ID)
        {                      //扩展帧	29bit ID
          usMaskReg[0] = CAN_MSG->IDMask.WORD & CAN_IF1MSK1_IDMSK_M;
          usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 16) & CAN_IF1MSK2_IDMSK_M);
        }
       else
        {                      //标准帧  
         usMaskReg[0] = 0;
         usMaskReg[1] = ((CAN_MSG->IDMask.WORD >> 6 ) & CAN_IF1MSK2_IDMSK_M);
        }
      }
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.EXT_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MXTD;  //使用扩展ID过滤
    if(CAN_MSG->CTL.BIT.ID_FLT_EN && CAN_MSG->CTL.BIT.DIR_FLT_EN)
     usMaskReg[1] |= CAN_IF1MSK2_MDIR;  //传输方向用于过滤
    if(CAN_MSG->CTL.BIT.ID_FLT_EN  ||
       CAN_MSG->CTL.BIT.EXT_FLT_EN ||
       CAN_MSG->CTL.BIT.DIR_FLT_EN)
     {
      usMsgCtrl |= CAN_IF1MCTL_UMASK;
      usCmdMaskReg |= CAN_IF1CMSK_MASK;
     }
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //扩展帧 29BIT ID
      usArbReg[0] |= CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD;
     }
    else
     {                      //标准帧
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_MSGVAL;
     }
    usMsgCtrl |= (CAN_MSG->CTL.BIT.LEN  | CAN_IF1MCTL_EOB); //设置控制位 数据长度 单次发送
    if(CAN_MSG->CTL.BIT.RX_INT_EN )                         //判断接收中断是否使能
     usMsgCtrl |= CAN_IF1MCTL_RXIE;                         //接收中断使能        
    HWREG(CANx + CAN_O_IF1CMSK) = usCmdMaskReg;             //写命令屏蔽寄存器
    HWREG(CANx + CAN_O_IF1MSK1) = usMaskReg[0];             //写屏蔽寄存器1
    HWREG(CANx + CAN_O_IF1MSK2) = usMaskReg[1];             //写屏蔽寄存器2
    HWREG(CANx + CAN_O_IF1ARB1) = usArbReg[0];              //写仲裁寄存器1
    HWREG(CANx + CAN_O_IF1ARB2) = usArbReg[1];              //写仲裁寄存器2
    HWREG(CANx + CAN_O_IF1MCTL) = usMsgCtrl;                //写报文控制寄存器
    HWREG(CANx + CAN_O_IF1CRQ)  = (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M);//INF->MSG RAM
}
/*****************************************************************************
* 设置 CAN0报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针 
* 入口: eMsgType 报文类型 MSG_OBJ_TYPE_TX  MSG_OBJ_TYPE_TX_REMOTE
* 2008.12.5 为提高处理速度 去掉远程帧发送 如果需要发送远程帧 要修改程序
*****************************************************************************/
void CAN_Tx_Msg(u32 CANx, CAN_MSG *CAN_MSG, tMsgObjType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_TX) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE))
    
    while(HWREG(CANx + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY)
     {        //等待接口寄存器空闲
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //方向 INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //允许访问MSG 数据A
                    CAN_IF1CMSK_DATAB |   //允许访问MSG 数据B
                    CAN_IF1CMSK_CONTROL|  //允许访问MSG 控制位
                    CAN_IF1CMSK_ARB);     //允许访问MSG 仲裁位 
    if(eMsgType != MSG_OBJ_TYPE_TX)
     return;
    usArbReg[1] = (CAN_IF1ARB2_DIR|CAN_IF1ARB2_MSGVAL);    //设置方向位为发送 发送数据帧
    usMsgCtrl = (CAN_IF1MCTL_TXRQST| CAN_IF1MCTL_EOB);     //置位发送请求  发送数据帧
    usMsgCtrl |= CAN_MSG->CTL.BIT.LEN  ;                   //设置控制位 数据长度 单次发送
    if(CAN_MSG->CTL.BIT.TX_INT_EN )                        //判断发送中断是否使能
     usMsgCtrl |= CAN_IF1MCTL_TXIE;                        //发送中断使能
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //扩展帧 29BIT ID
      usArbReg[0]  = CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_XTD;
     }
    else
     {                      //标准帧
      usArbReg[0] = 0;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
     }
    if(CAN_MSG->CTL.BIT.LEN!=0)
     {                                              //数据帧 数据长度不为0导入数据
      CANDataRegWrite(&CAN_MSG->Data.BYTE[0],       //待发送数据指针
                      (u32*)(CANx + CAN_O_IF1DA1),  //CAN数据寄存器地址
                      CAN_MSG->CTL.BIT.LEN);        //数据长度)
     }
    HWREG(CANx + CAN_O_IF1CMSK) = usCmdMaskReg;     //写命令屏蔽寄存器
    HWREG(CANx + CAN_O_IF1ARB1) = usArbReg[0];      //写仲裁寄存器1
    HWREG(CANx + CAN_O_IF1ARB2) = usArbReg[1];      //写仲裁寄存器2
    HWREG(CANx + CAN_O_IF1MCTL) = usMsgCtrl;        //写报文控制寄存器
    HWREG(CANx + CAN_O_IF1CRQ)  = (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M);//INF->MSG RAM
}
/*****************************************************************************
* 设置 CAN1报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针 
* 入口: eMsgType 报文类型 MSG_OBJ_TYPE_TX  MSG_OBJ_TYPE_TX_REMOTE
* 2008.12.5 为提高处理速度 去掉远程帧发送 如果需要发送远程帧 要修改程序
*****************************************************************************/
void CAN1_Tx_Msg(u32 CANx, CAN1_MSG *CAN_MSG, tMsgObjType eMsgType)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    ASSERT((eMsgType == MSG_OBJ_TYPE_TX) ||
           (eMsgType == MSG_OBJ_TYPE_TX_REMOTE))
    
    while(HWREG(CANx + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY)
     {        //等待接口寄存器空闲
     }

    usCmdMaskReg = (CAN_IF1CMSK_WRNRD |   //方向 INF->MSG RAM
                    CAN_IF1CMSK_DATAA |   //允许访问MSG 数据A
                    CAN_IF1CMSK_DATAB |   //允许访问MSG 数据B
                    CAN_IF1CMSK_CONTROL|  //允许访问MSG 控制位
                    CAN_IF1CMSK_ARB);     //允许访问MSG 仲裁位 
    if(eMsgType != MSG_OBJ_TYPE_TX)
     return;
    usArbReg[1] = (CAN_IF1ARB2_DIR|CAN_IF1ARB2_MSGVAL);    //设置方向位为发送 发送数据帧
    usMsgCtrl = (CAN_IF1MCTL_TXRQST| CAN_IF1MCTL_EOB);     //置位发送请求  发送数据帧
    usMsgCtrl |= CAN_MSG->CTL.BIT.LEN  ;                   //设置控制位 数据长度 单次发送
    if(CAN_MSG->CTL.BIT.TX_INT_EN )                        //判断发送中断是否使能
     usMsgCtrl |= CAN_IF1MCTL_TXIE;                        //发送中断使能
    if(CAN_MSG->CTL.BIT.EXD_ID)
     {                     //扩展帧 29BIT ID
      usArbReg[0]  = CAN_MSG->ID.WORD & CAN_IF1ARB1_ID_M;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 16) & CAN_IF1ARB2_ID_M;
      usArbReg[1] |= CAN_IF1ARB2_XTD;
     }
    else
     {                      //标准帧
      usArbReg[0] = 0;
      usArbReg[1] |= (CAN_MSG->ID.WORD >> 6) & CAN_IF1ARB2_ID_M;
     }
    if(CAN_MSG->CTL.BIT.LEN!=0)
     {                                              //数据帧 数据长度不为0导入数据
      CANDataRegWrite(&CAN_MSG->Data.BYTE[0],       //待发送数据指针
                      (u32*)(CANx + CAN_O_IF1DA1),  //CAN数据寄存器地址
                      CAN_MSG->CTL.BIT.LEN);        //数据长度)
     }
    HWREG(CANx + CAN_O_IF1CMSK) = usCmdMaskReg;     //写命令屏蔽寄存器
    HWREG(CANx + CAN_O_IF1ARB1) = usArbReg[0];      //写仲裁寄存器1
    HWREG(CANx + CAN_O_IF1ARB2) = usArbReg[1];      //写仲裁寄存器2
    HWREG(CANx + CAN_O_IF1MCTL) = usMsgCtrl;        //写报文控制寄存器
    HWREG(CANx + CAN_O_IF1CRQ)  = (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M);//INF->MSG RAM
}
/*****************************************************************************
* 读取 CAN 报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针
* 入口: bClrPendingInt 是否清除挂起位
* 2008.11.27 zlz 取消了屏蔽码读取 不接收远程帧 
*****************************************************************************/
void CAN_Rx_Msg(u32 CANx, CAN_MSG *CAN_MSG, u8 bClrPendingInt)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    usCmdMaskReg = (CAN_IF1CMSK_DATAA |   //允许访问DA
                    CAN_IF1CMSK_DATAB |   //允许访问DB
                    CAN_IF1CMSK_CONTROL | //访问控制位
                    CAN_IF1CMSK_MASK |    //访问屏蔽位
                    CAN_IF1CMSK_ARB|      //访问仲裁位
                    CAN_IF1CMSK_NEWDAT);  //清除新数据标志
    if(bClrPendingInt)                    //判断是否清除pending位
     usCmdMaskReg |= CAN_IF1CMSK_CLRINTPND;//清除
    HWREG(CANx + CAN_O_IF2CMSK) = usCmdMaskReg; //写命令屏蔽寄存器
    HWREG(CANx + CAN_O_IF2CRQ) = (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M); //MSG RAM->INF
    while(HWREG(CANx + CAN_O_IF2CRQ) & CAN_IF1CRQ_BUSY)
     {                                     //等待 MSG RAM->INF 完成
     }
    usArbReg[0]  = HWREG(CANx + CAN_O_IF2ARB1); //读仲裁寄存器1
    usArbReg[1]  = HWREG(CANx + CAN_O_IF2ARB2); //读仲裁寄存器2
    usMsgCtrl    = HWREG(CANx + CAN_O_IF2MCTL); //读报文控制寄存器
/*
    CAN_MSG->CTL.WORD = MSG_OBJ_NO_FLAGS;//清除报文标志 
    if((!(usMsgCtrl & CAN_IF1MCTL_TXRQST) && (usArbReg[1] & CAN_IF1ARB2_DIR))
     || ((usMsgCtrl & CAN_IF1MCTL_TXRQST) && (!(usArbReg[1] & CAN_IF1ARB2_DIR))))
     CAN_MSG->CTL.BIT.RMT_FRM = 1;       //远程帧标志
*/
    if(usArbReg[1] & CAN_IF1ARB2_XTD)
     {                                 //扩展帧
      CAN_MSG->ID.WORD = (((usArbReg[1] & CAN_IF1ARB2_ID_M) << 16) | usArbReg[0]);//ID
      CAN_MSG->CTL.BIT.EXD_ID = 1;  //控制域中扩展帧标志
     }
    else							   //标准帧
     CAN_MSG->ID.WORD = (usArbReg[1] & CAN_IF1ARB2_ID_M) << 6;//ID
/* 2008.12.5 zlz 为提高处理速度去掉
    if(usMsgCtrl & CAN_IF1MCTL_MSGLST)             //判断是否有报文丢失 类似Overrun 
     CAN_MSG->CTL.BIT.DATA_LOST = 1;               //报文丢失标志 
*/
    if(usMsgCtrl & CAN_IF1MCTL_NEWDAT)              //判断是否有新数据
     {                                              //有新数据
      CAN_MSG->CTL.BIT.LEN = (usMsgCtrl & CAN_IF1MCTL_DLC_M);//数据长度
//      if(CAN_MSG->CTL.BIT.RMT_FRM == 0)
       {                                            //不为远程帧 
        CANDataRegRead(&CAN_MSG->Data.BYTE[0],	    //保存数据指针
                       (u32*)(CANx + CAN_O_IF2DA1), //CAN数据寄存器地址
                        CAN_MSG->CTL.BIT.LEN);      //数据长度
       }
/*       
      CANRegWrite((u32)&CANx->INF[1].CMSK , CAN_IF1CMSK_NEWDAT);//清除新数据标志
      CANRegWrite((u32)&CANx->INF[1].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {              //等待写操作完成
       }
      CAN_MSG->CTL.BIT.NEW_DATA = 1;              //置位有新数据标志  
*/
     }
    else                                             //没有数据
     CAN_MSG->CTL.BIT.LEN = 0;                    //数据长度0 
}

/*****************************************************************************
* 读取 CAN1 报文RAM 区 报文对象
* 入口:CANx CAN结构体
* 入口: CAN_MSG 报文指针
* 入口: bClrPendingInt 是否清除挂起位
* 2008.11.27 zlz 取消了屏蔽码读取 不接收远程帧 
*****************************************************************************/
void CAN1_Rx_Msg(u32 CANx, CAN1_MSG *CAN_MSG, u8 bClrPendingInt)
{
    u16 usCmdMaskReg;
    u16 usArbReg[2]={0,0};
    u16 usMsgCtrl=0;
    ASSERT(CANBaseValid(CANx));
    ASSERT((CAN_MSG->CTL.BIT.IDx <= 32) && (CAN_MSG->CTL.BIT.IDx != 0));
    usCmdMaskReg = (CAN_IF1CMSK_DATAA |   //允许访问DA
                    CAN_IF1CMSK_DATAB |   //允许访问DB
                    CAN_IF1CMSK_CONTROL | //访问控制位
                    CAN_IF1CMSK_MASK |    //访问屏蔽位
                    CAN_IF1CMSK_ARB|      //访问仲裁位
                    CAN_IF1CMSK_NEWDAT);  //清除新数据标志
    if(bClrPendingInt)                    //判断是否清除pending位
     usCmdMaskReg |= CAN_IF1CMSK_CLRINTPND;//清除
    HWREG(CANx + CAN_O_IF2CMSK) = usCmdMaskReg; //写命令屏蔽寄存器
    HWREG(CANx + CAN_O_IF2CRQ) = (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M); //MSG RAM->INF
    while(HWREG(CANx + CAN_O_IF2CRQ) & CAN_IF1CRQ_BUSY)
     {                                     //等待 MSG RAM->INF 完成
     }
    usArbReg[0]  = HWREG(CANx + CAN_O_IF2ARB1); //读仲裁寄存器1
    usArbReg[1]  = HWREG(CANx + CAN_O_IF2ARB2); //读仲裁寄存器2
    usMsgCtrl    = HWREG(CANx + CAN_O_IF2MCTL); //读报文控制寄存器
/*
    CAN_MSG->CTL.WORD = MSG_OBJ_NO_FLAGS;//清除报文标志 
    if((!(usMsgCtrl & CAN_IF1MCTL_TXRQST) && (usArbReg[1] & CAN_IF1ARB2_DIR))
     || ((usMsgCtrl & CAN_IF1MCTL_TXRQST) && (!(usArbReg[1] & CAN_IF1ARB2_DIR))))
     CAN_MSG->CTL.BIT.RMT_FRM = 1;       //远程帧标志
*/
    if(usArbReg[1] & CAN_IF1ARB2_XTD)
     {                                 //扩展帧
      CAN_MSG->ID.WORD = (((usArbReg[1] & CAN_IF1ARB2_ID_M) << 16) | usArbReg[0]);//ID
      CAN_MSG->CTL.BIT.EXD_ID = 1;  //控制域中扩展帧标志
     }
    else							   //标准帧
     CAN_MSG->ID.WORD = (usArbReg[1] & CAN_IF1ARB2_ID_M) << 6;//ID
/* 2008.12.5 zlz 为提高处理速度去掉
    if(usMsgCtrl & CAN_IF1MCTL_MSGLST)             //判断是否有报文丢失 类似Overrun 
     CAN_MSG->CTL.BIT.DATA_LOST = 1;               //报文丢失标志 
*/
    if(usMsgCtrl & CAN_IF1MCTL_NEWDAT)              //判断是否有新数据
     {                                              //有新数据
      CAN_MSG->CTL.BIT.LEN = (usMsgCtrl & CAN_IF1MCTL_DLC_M);//数据长度
//      if(CAN_MSG->CTL.BIT.RMT_FRM == 0)
       {                                            //不为远程帧 
        CANDataRegRead(&CAN_MSG->Data.BYTE[0],	    //保存数据指针
                       (u32*)(CANx + CAN_O_IF2DA1), //CAN数据寄存器地址
                        CAN_MSG->CTL.BIT.LEN);      //数据长度
       }
/*       
      CANRegWrite((u32)&CANx->INF[1].CMSK , CAN_IF1CMSK_NEWDAT);//清除新数据标志
      CANRegWrite((u32)&CANx->INF[1].CRQ , (CAN_MSG->CTL.BIT.IDx & CAN_IF1CRQ_MNUM_M));
      while(CANRegRead(IntNumber,(u32)&CANx->INF[1].CRQ) & CAN_IF1CRQ_BUSY)
       {              //等待写操作完成
       }
      CAN_MSG->CTL.BIT.NEW_DATA = 1;              //置位有新数据标志  
*/
     }
    else                                             //没有数据
     CAN_MSG->CTL.BIT.LEN = 0;                    //数据长度0 
}


