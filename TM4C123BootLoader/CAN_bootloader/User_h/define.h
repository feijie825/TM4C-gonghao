/******************** (C) COPYRIGHT 2008 LUMINARY LM3S21xx ********************
;* File Name          : define.h
;* Author             : ������
;* �û�����������Ԥ����
;* ���ȼ����� 1.����֡���ڴӻ�֡(����D=0 �ӻ�D=1 ID������λ��ͬ����� )
;*            2.��׼֡������չ֡
;*            3.������֡���ڶ�����֡
*******************************************************************************/
#include "Data_types.h"
#include "../inc/gpio.h"
//#include "TM4C123_Lib.h"

#define  CFG_BASE_ADDR    0x28000          //���һҳ ���ڴ�������� 1K BYTES ����1KʱӦ�������� ���ռ�32K BYTES
#define  SAVE_TAB_NUMB    ((1024/((sizeof(SAVE_S)+3)&0xFFFC))-1)  //��ౣ������ 4�ֽڶ��� ����1

//CAN �������������
#define  MSG_OBJ_NUMB      13              //���Ķ���ʹ�ø���  Ҫ��LM3S21xx_CAN.C CAN_MSG_SET_TAB ��񳤶�һ֡
#define  CAN_LDT_TYPE_NUMB 10              //����������������
#define  CAN_LDT_ACT_NUMB   2              //����������ʵ�ʸ��� ������CAN_LDT_TYPE_NUMB
//CAN ID ��������붨��
#define  IDX_MASK_CODE    (0x07<<0  )      //֡��Ų�����˴��� 3BIT
#define  END_MASK_CODE    (0x01<<3  )      //����λ������˴��� 1BIT
#define  TYPE_MSAK_CODE   (0x0F<<4  )      //�����������˴���	4BIT
#define  MNUM_MASK_CODE   (0xff<<8  )      //��λ�������˴��� 8BIT
#define  DIR_MASK_CODE    (0x01<<16 )      //����λ������˴��� 1BIT
#define  CMD_MASK_CODE    (0x7ff<<17)      //�����������˴��� 11BIT
#define  EXD_MASK_CODE	  (0x01<<28 )      //ID����չ֡��ʶ�� 	1 BIT Ϊ���ñ�׼֡���ȼ�������չ֡ ����
#define  H6BCMD_MASK_CODE (0x7E0<<17)      //ǰ6λ�����������˴��� 
//CAN ������ѯ�ͻ�Ӧ״̬����
#define  MST_CHK_RCVD     'C'	           //���յ�������ѯ����
#define  SLV_ECHO_SEND    'S'	           //���ڷ��ʹӻ���Ӧ����
#define  SLV_ECHO_ACK     'A'	           //�ӻ���Ӧ����ɹ�����
//�ӻ� CAN�����ݷ���״̬����
#define  SLV_LDATA_TX_NO   0               //�ӻ������ݿ���״̬
#define  SLV_LDATA_TX_REQ 'R'              //�ӻ������ݷ��������ѷ��� REQUEST
#define  SLV_LDATA_TX_ACK 'A'              //�ӻ������ݷ���������׼ ACK
#define  SLV_LDATA_RETX   'R'              //��������ӻ��ط���־ Retransmit
#define  SLV_LDATA_TX_IS  'S'              //�ӻ����ڷ��ͳ�����
#define  SLV_LDATA_TX_FI  'F'              //��֡�������
#define  SLV_LDATA_TX_LAST 'L'             //�ӻ����ڷ������һ֡����
#define  SLV_LDATA_TX_END 'E'              //�ӻ������ݷ��ͽ���
//�ӻ� CAN�����ݽ���״̬����
#define  SLV_LDATA_RX_NO   0               //����״̬
#define  SLV_LDATA_RX_IS  'S'              //�ӻ����������ڽ���״̬
#define  SLV_LDATA_RX_END 'E'              //�ӻ������ݽ��ս���
#define  SLV_LDATA_RX_OUT 'O'              //���յ����������ڴ���
//CAN ��֡���ݷ���״̬
#define  SLV_SMSG_TX_IS   'S'              //���ڷ��Ͷ�֡
//���ڽ���״̬����
#define  COM_RX_NO         0               //����״̬
#define  COM_RX_IS        'R'              //�������ڽ�������
#define  COM_RX_END       'E'              //�������ݽ��ս���
//���ڷ���״̬����
#define  COM_TX_NO         0               //���ͻ���������״̬
#define  COM_TX_IS        'O'              //�������ڷ�������
#define  COM_TX_EN        'E'              //����������Ч
//������Ч����
#define  DATA_YES         'Y'              //�洢�������ݱ�־ ��λ��
#define  DATA_VALIDE      'E'              //������Ч��־ 
#define  DATA_BLANK       'B'              //���ݿձ�־
#define  DATA_NOT_BLACK   'N'              //���ݲ��ձ�־

//����MSG RAM ID��Ӧ����
// 11bit ID ��IDmask ��ʽ 0CDXXXXXXXX 	 C=0~1	 D=0 ��->�� D=1 ��->�� XXXXXXXX=0~255 ��λ��
// 29bit ID ��IDmask ��ʽ 1CCCCCCCCCCDXXXXXXXXTTTTEIII	CCCCCCCCCCC =0~2015 Dͬ�� TTTT ������ ���� E �����ݽ��� III ֡���
//                         CCCCCCCCCC ���� D ���� XXXXXXXX ��λ�� TTTT ������ ���� E �����ݽ��� III ֡���
//MST ��ʾ�������� �ӻ����� SLV ��ʾ�ӻ����� ��������
/**************************************************************
* ֡����    : ��׼֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=0 �㲥        
* �����  : �����㲥��ѯ  
* �����˲�λ:      MM M MMMMMMMM
* �˲�����  : ��   0C D XXXXXXXX
* �����ʽ  :      00 0 00000000 
* �������ȼ�: ���
* �����    : 0
* �㲥/���� : �㲥
**************************************************************/
#define MST_CHK_BCAST    1 	  //��MSG RAM ��λ��
#define MST_CHK_CMD      0    //������ѯ������
/**************************************************************
* ֡����    : ��׼֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=�������        
* �����  : ����������ѯ  
* �����˲�λ:      MM M MMMMMMMM
* �˲�����  : ��   0C D XXXXXXXX
* �����ʽ  :      00 0 XXXXXXXX 
* �������ȼ�: �θ�
* �����    : 0
* �㲥/���� : ����
**************************************************************/
#define MST_CHK_SCAST    2  
/**************************************************************
* ֡����    : ��׼֡
* �����  : ����<-�ӻ�  D=1
* ��λ��    : XXXXXXXX=�������        
* �����˲�λ: �������� ���������˲�     
* �����  : �ӻ���Ӧ������ѯ  
* �˲�����  : ��   0C D XXXXXXXX
* �����ʽ  :      00 1 XXXXXXXX 
* �������ȼ�: �θ�
* �����    : 0
* �㲥/���� : ����
**************************************************************/
#define SLV_CHK_ECHO     3 
#define SLV_ECHO_CMD     0   //�ӻ���Ӧ���� 
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=�������        
* �����  : ������׼�ӻ����ͳ���������  
* �����˲�λ:          MMMMMMMMMMMM M MMMMMMMM			 M	MMM
* �˲�����  : ��       1CCCCCCCCCCC D XXXXXXXX   TTTT    E  III
* �����ʽ  :          100000000000 0 XXXXXXXX 0000~1111 0	000
* �������ȼ�: ��չ֡����� ����CHK_BCAST CHK_SCAST CHK_ECHO ��׼֡
* �����    : 0
* ����������: TTTT 0~15
* �����ݽ���: E   0 ������������
* ���ݱ��  : III 0 ������������
* �㲥/���� : ����(ֻ�е���)
**************************************************************/
#define MST_LDATA_ACK	  4  
#define MST_LDATA_ACK_CMD 0 //������׼�����ݷ�������
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=0        
* �����  : �������㲥���ݷ��� 
* �����˲�λ:          MMMMMMMMMMMM M MMMMMMMM
* �˲�����  : ��       1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          100000000001 0 00000000 0000~1111 X XXX
* �������ȼ�: ��չ֡�дθ� 
* �����    : 1
* ����������: TTTT 0~15
* �����ݽ���: E   0 �����ݷ���δ���� 1�����ݽ���(���һ֡)
* ���ݱ��  : III 0 ���ݿ�ʼ 1~7 ���ݱ�� 
* �㲥/���� : �㲥
**************************************************************/
#define MST_LCDATA_TX    5
#define MST_LDATA_TX_CMD 1  //���������ݷ�������
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=�������        
* �����  : �������������ݷ��� 
* �����˲�λ:          MMMMMMMMMMMM M MMMMMMMM
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          100000000001 0 XXXXXXXX 0000~1111 X XXX
* �������ȼ�: ��չ֡�дθ� 
* �����    : 1
* ����������: TTTT 0~15
* �����ݽ���: E   0 �����ݷ���δ���� 1�����ݽ���(���һ֡)
* ���ݱ��  : III 0 ���ݿ�ʼ 1~7 ���ݱ�� 
* �㲥/���� : ����
**************************************************************/
#define MST_LSDATA_TX    6
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=�������        
* �����  : ��������ӻ��ط����� ������(�ܿ�����)���ճ����ݿ����ʱִ��
* �����˲�λ:          MMMMMMMMMMMM M MMMMMMMM			 M MMM
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          100000000010 0 XXXXXXXX 0000~1111 0 000
* �������ȼ�: ��չ֡������ 
* �����    : 2
* ����������: TTTT 0~15
* �����ݽ���: E   0 ������������
* ���ݱ��  : III 0 ������������
* �㲥/���� : ����(ֻ�е���)
**************************************************************/
#define MST_LDATA_REQRT    7 
#define MST_LDATA_REQRT_CMD 2   //���������ط����������� 
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=0        
* �����  : �������ڷ��Ͷ̹㲥���ݴ�
* �����˲�λ:          M            M MMMMMMMM			 M
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          1CCCCCCCCCCC 0 00000000 0000~1111 0 XXX
* �������ȼ�: ��չ֡������ 
* �����    : 4~2015 ���� ��Ϊ7��256��ָ�����һ��220��ָ���� ÿ��ָ�����Ӧһ�����ܵ�Ԫ 
* ����������: TTTT 0~15	 ������Ԥ��
* �����ݽ���: E   0      ������������
* ���ݱ��  : III 0      ������Ԥ��
* �㲥/���� : �㲥
**************************************************************/
#define MST_SCDATA_TX    8    //LONG CAST DATA �������ݴ�
#define MST_SDATA_TX_CMD 4    //�������Ͷ���������
/**************************************************************
* ֡����    : ��չ֡
* �����  : ����->�ӻ�  D=0
* ��λ��    : XXXXXXXX=�������         
* �����  : �������ڷ��Ͷ̵������ݴ�
* �����˲�λ:          M            M MMMMMMMM			 M
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          1CCCCCCCCCCC 0 XXXXXXXX 0000~1111 0 XXX
* �������ȼ�: ��չ֡������ 
* �����    : 4~2015 ���� ��Ϊ7��256��ָ�����һ��220��ָ���� ÿ��ָ�����Ӧһ�����ܵ�Ԫ 
* ����������: TTTT 0~15	 ������Ԥ��
* �����ݽ���: E   0      ������������
* ���ݱ��  : III 0      ������Ԥ��
* �㲥/���� : ����
**************************************************************/
#define MST_SSDATA_TX    9   //LONG SINGLE DATA �������ݴ�

/**************************************************************
* ֡����    : ��չ֡
* �����  : �ӻ�-> ���� D=1
* ��λ��    : XXXXXXXX=�������        
* �����  : �ӻ��������������ͳ�����  
* �����˲�λ: ��
* �˲�����  : ��	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          100000000000 1 XXXXXXXX 0000~1111 0 000
* �������ȼ�: ��չ֡����� ����CHK_BCAST CHK_SCAST CHK_ECHO ��׼֡ 
* �����    : 0
* ����������: TTTT 0~15
* �����ݽ���: E   0 ������������
* ���ݱ��  : III 0 ������������
* �㲥/���� : ����(ֻ�е���)
**************************************************************/
#define SLV_LDATA_REQTX	 10  
/**************************************************************
* ֡����    : ��չ֡
* �����  : �ӻ�-> ���� D=1
* ��λ��    : XXXXXXXX=�������        
* �����  : �ӻ������ݷ��� 
* �����˲�λ: ��         
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          100000000001 1 XXXXXXXX 0000~1111 X XXX
* �������ȼ�: ��չ֡�дθ� 
* �����    : 1
* ����������: TTTT 0~15
* �����ݽ���: E   0 �����ݷ���δ���� 1�����ݽ���(���һ֡)
* ���ݱ��  : III 0 ���ݿ�ʼ 1~7 ���ݱ�� 
* �㲥/���� : ����(ֻ�е���)
**************************************************************/
#define SLV_LDATA_TX    11
/**************************************************************
* ֡����    : ��չ֡
* �����  : �ӻ�-> ���� D=1
* ��λ��    : XXXXXXXX=�������        
* �����  : �ӻ����������ط����� ���ӻ����ճ����ݿ����ʱִ�� 
* �����˲�λ: ��         
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          100000000010 1 XXXXXXXX 0000~1111 0 000
* �������ȼ�: ��չ֡�дθ� 
* �����    : 2
* ����������: TTTT 0~15
* �����ݽ���: E   0 ������������
* ���ݱ��  : III 0 ������������
* �㲥/���� : ����(ֻ�е���)
**************************************************************/
#define SLV_LDATA_REQRT    12
/**************************************************************
* ֡����    : ��չ֡
* �����  : �ӻ�->����  D=1
* ��λ��    : XXXXXXXX=�������         
* �����  : �ӻ����ڷ��Ͷ����ݴ�
* �����˲�λ: ��
* �˲�����  : �� 	   1CCCCCCCCCCC D XXXXXXXX   TTTT    E III
* �����ʽ  :          1CCCCCCCCCCC 1 XXXXXXXX 0000~1111 X XXX
* �������ȼ�: ��չ֡������ 
* �����    : 4~2015 ���� ��Ϊ7��256��ָ�����һ��220��ָ���� ÿ��ָ�����Ӧһ�����ܵ�Ԫ 
* ����������: TTTT 0~15	 ������Ԥ��
* �����ݽ���: E   0      ������������
* ���ݱ��  : III 0      ������Ԥ��
* �㲥/���� : ����(ֻ�е���)
**************************************************************/
#define SLV_SDATA_TX    13   //SHORT  DATA �����ݴ�

//����SSIλ����
#define SSI_BITRATE      16667        //16.667K


#define NATIONAL         'N' //������
#define SOUTH            'S' //������
#define Enable_Bit       1
#define Disable_Bit      0
#define LEVEL_MOD        0
#define PULSE_MOD        1
#define LOW_PUS_TRG      0
#define HIG_PUS_TRG      1

#define  MTR_NO         '0'        //û��
#define  MTR_RDY        '1'        //��ź�

#define  LGT_OFF        '0'        //����
#define  YLW_ON         '1'        //�Ƶ���
#define  GRN_ON         '2'        //�̵���
#define  RED_ON         '3'        //�����


#define  LED_6            '6'          //6λ�����
#define  LED_8            '8'          //8λ�����

//������Ч����
#define YES               'Y'          //�洢�������ݱ�־ ��λ��
#define VALIDE            'E'          //������Ч��־ 
#define BLANK             'B'          //���ݿձ�־
#define NOT_BLACK         'N'          //���ݲ��ձ�־

#define LED_6              '6'  //��λ�����
#define LED_8              '8'  //��λ�����

#define UART_NUMB           2            //���ڸ���
//�����㵥Ԫ�๦�ܶ˿�                 
#define MTRCOM              0            //ģ���(�๦�ܱ�)�˿�
#define LCTCOM              1            //�����ն˶˿�
//�����ʶ��� ����λ                        
#define UART_5_BIT          0            //����λ���� 5bit
#define UART_6_BIT          1            //����λ���� 6bit
#define UART_7_BIT          2            //����λ���� 7bit
#define UART_8_BIT          3            //����λ���� 8bit
//�����ʶ��� ֹͣλ                      
#define UART_1_STOP         0            //1   stop bit      
#define UART_2_STOP         1            //2   stop bit      
//�����ʶ��� У��λ                      
#define UART_N_PARITY       0            //��У��λ
#define UART_O_PARITY       1            //ODD ��У��
#define UART_E_PARITY       2            //EVEN żУ��
#define UART_M_PARITY       3            //Mark У��(1)
#define UART_S_PARITY       4            //SpaceУ��(0)

#define UART0_BAUD          2400         //UART0Ĭ�β�����
#define UART1_BAUD          2400         //UART1Ĭ�β�����

#define MIN_BAUD            110          //��С������
#define MAX_BAUD            115200       //�������

#define Wh_Wire          '0'       //�й�����
#define Var_Wire         '1'       //�޹�����

#define VLOSS_1          '1'      //��U=100% 1S3��    ��ԭװ��Э����ͬ
#define VLOSS_2          '2'      //��U=100% 1������ 
#define VLOSS_3          '3'      //��U=50%  1����   

#define GB               '1'      //�ұ�             ��ԭװ��Э�鲻ͬ 
#define BGB              '2'      //���ұ�

#define GD_E             '1'      //�����伯         
#define GD_C             '2'      //�����缫

#define HC_2             '1'      //�ϳ���· 
#define HC_3             '2'      //�ϳ���·
#define HC_4             '3'      //�ϳ���·

#define GDT_PLS          '1'      //���ͷ����
#define DZ_PLS           '2'      //��������
#define SZ_PLS           '3'      //ʱ������
#define XUL_PLS          '4'      //��������
#define TQ_PLS           '5'      //Ͷ������
#define HZ_PLS           '6'      //��բ����

#define EPLS_T           '1'      //����������������
#define SZ_T             '2'      //����ʱ����������
#define XUL_T            '3'      //����������������        
#define TQ_T             '4'      //Ͷ������
#define HZ_T             '5'      //��բ����

#define UNION_PLS        '1'      //���϶๦������ ʱ�� ���� Ͷ�� �ȹ���
#define ALONE_PLS        '2'      //�����๦������ ʱ�� ���� Ͷ�� �ֿ�����         
//ֻ����������������Ч
#define NO_PLS           '0'      //Ĭ������������
#define SZCLK_PLS        '1'      //��ǰ����Ϊʱ������
#define XULCLK_PLS       '2'      //��ǰ����Ϊ��������
#define TQCLK_PLS        '3'      //��ǰ����ΪͶ������

#define PA_PLS           '0'      //�����й�
#define QA_PLS           '1'      //�����޹�
#define PR_PLS           '2'      //�����й�
#define QR_PLS           '3'      //�����޹�

#define NCATCH_HB        '0'      //�ڰ�δ��׽
#define CATCH_HB         '1'      //�ڰ��Ѳ�׽

#define ZZ_STRT          '0'      //���ֿ�ʼ
#define ZZ_END           '1'      //���ֽ���

#define NY_BAD           '0'      //��ѹ����
#define NY_GOOD          '1'      //��ѹ�ϸ�
#define NY_UNKW          '2'      //δ֪

#define MEA_STOP         '0'      //ֹͣ����
#define MEA_ORDER        '1'      //��������
 
#define UA_PHASE        (1<<0)    //A���ѹ
#define UB_PHASE        (1<<1)    //B���ѹ
#define UC_PHASE        (1<<2)    //C���ѹ
#define ALL_PHASE       (UA_PHASE|UB_PHASE|UC_PHASE)//�����ѹ

#define OFF             0    //�Ͽ�
#define ON              1    //����

#define DISP_TEST       '0'  //��ʾ����״̬
#define DISP_CFREQ      '1'  //��ʾʱ��Ƶ��
#define DISP_CERR       '2'  //��ʾ�ռ�ʱ���
#define DISP_XULT       '3'  //��ʾ��������
#define DISP_TQMC       '4'  //��ʾͶ������

#define SUB_485         '1'  //��ڶ�485ͨ��
#define RED_485         '2'  //�����485

#define MTR_UOP         '0'  //��λ��ѹ�Ͽ�
#define MTR_UCL         '1'  //��λ�̵������Ƶ�ѹ����
#define MTR_UECL        '2'  //��λ���ӿ��ؿ��Ƶ�ѹ����

#define SINGLE           0   //����̨
#define THREE            1   //����̨
//���߷�ʽ����
#define WIRE_P1          '0'      //���߷�ʽ �����й�
#define WIRE_P4          '1'      //���߷�ʽ ���������й�
#define WIRE_P3_2        '2'      //���߷�ʽ ���������й�
#define WIRE_Q4_3        '3'      //���߷�ʽ ���������޹�90����Ԫ���޹�
#define WIRE_Q3_60       '4'      //���߷�ʽ ������������60���޹� 60����Ԫ���޹�
#define WIRE_Q3_90       '5'      //���߷�ʽ �������߿����޹�90����Ԫ���޹�
#define WIRE_Q4_R        '6'      //���߷�ʽ �����������޹�
#define WIRE_Q3_R        '7'      //���߷�ʽ �����������޹�
#define WIRE_P3_3        '8'      //���߷�ʽ ���������й� UA UB UC �������������UB�Եز�Ϊ0
#define WIRE_Q3_2        '9'      //���߷�ʽ ���������޹� ���������������� UB��U0
#define WIRE_Q3_CT       ':'      //���߷�ʽ ���������˹����ĵ��޹�
#define WIRE_Q1          ';'      //���෽ʽ �޹�

#define MEA_PWR_P3        0       //�⹦�� P3
#define MEA_PWR_P4        1       //�⹦�� P4
//����CAN�����
#define MULTI_DATA_CMD    0       //0-3     ������֡����   4��
#define MTR_DATA_CMD      4       //4-259   ԭ��Ԫ���� 256��
#define CLOCK_DATA_CMD    260     //260-515	ʱ��У�������� 256��
#define FKZD_DATA_CMD     516     //516-771 �����ն�����   256��
							  //772-2015 1244��ָ��Ԥ��
#define MTR_TAB_NUMB      128    
                          
#define WATCH_TIME 	      1000    //���Ź����ڵ�λ:mS ����
#define SYS_TIME          1000    //ϵͳ���Ķ�ʱ���� ��λ:uS ΢��
                                  
#define CAN_TX_OVTM       5000    //CAN���ͳ�ʱ5S
#define CAN_RX_OVTM       12000   //CAN���ճ�ʱ18S
#define NY_SEND_TIME      2000    //����Ͷ�ʱ
#define ZK2009_OVTM       10000   //ZK2009������ʱ
#define CAN_LDATA_SERR_MAX 3      //CAN����������ʹ������ �����ü���ֵ ��������ݱ�־ ���Խ�����һ֡

#define TIMER_8MS         8
#define NY_CHK_TIME       (200/TIMER_8MS)  //��ѹ��ⶨʱ
#define GDT_RST_TIME      (200/TIMER_8MS)  //���ͷ��λ��ʱ ��λ:ms
#ifdef PULSE
#define GDT_REN_TIME      (400/TIMER_8MS)  //��������ж�����ʹ�ܶ�ʱ 400ms
#define DZ_REEN_TIME      (400/TIMER_8MS)  //���������ж�����ʹ�ܶ�ʱ 400ms
#else
#define GDT_REN_TIME      (40/TIMER_8MS)   //��������ж�����ʹ�ܶ�ʱ 40ms
#define DZ_REEN_TIME      (40/TIMER_8MS)   //���������ж�����ʹ�ܶ�ʱ 40ms
#endif
#define SZ_REEN_TIME      (40/TIMER_8MS)   //ʱ�������ж�����ʹ�ܶ�ʱ 200ms          
#define XUL_REEN_TIME     (40/TIMER_8MS)   //���������ж�����ʹ�ܶ�ʱ 200ms  
#define TQ_REEN_TIME      (40/TIMER_8MS)   //Ͷ�������ж�����ʹ�ܶ�ʱ 200ms  
#define HZ_REEN_TIME      (40/TIMER_8MS)   //��բ�����ж�����ʹ�ܶ�ʱ 200ms  

#define GDT_PLS_TIME      20               //�������������ʱ
#define DZ_PLS_TIME       20               //��������������ʱ
#define SZ_PLS_TIME       20               //ʱ������������ʱ

#define KEY_REEN_TIME     (200/TIMER_8MS)  //��������ʹ���ж϶�ʱ
#define KEY_PLUG_TIME     (600/TIMER_8MS)  //�����ұ�ʱ
#define PLL_CHK_TIME      (200/TIMER_8MS)  //���໷PLL��鶨ʱ

#define MBJ_SEND_TIME     21000            //�������Ͷ�ʱ
#define WZTZ_SEND_TIME    19000            //����բ���Ͷ�ʱ
#define NZTZ_SEND_TIME    17000            //������բ���Ͷ�ʱ
#define MTR_ON_TIME       10000            //��λ�źö�ʱ
#define GZ_SEND_TIME      15000            //��ʱ���͹���
#define GZ_STB_TIME       1000             //�����ȶ���ʱ
#define HC165_TIME        20               //HC165������ʱ
#define LUNCI1_SEND_TIME  100              //�ִ�1��Ϣ���Ͷ�ʱ
#define LUNCI2_SEND_TIME  150              //�ִ�2��Ϣ���Ͷ�ʱ
#define LUNCI3_SEND_TIME  200              //�ִ�3��Ϣ���Ͷ�ʱ
#define LUNCI4_SEND_TIME  250              //�ִ�4��Ϣ���Ͷ�ʱ
#define CD4094_TIME       20               //CD4094��ʱ����
  
#define DISP_EN_TIME      15               //��ʾʹ�ܶ�ʱ

#define POWER_UP_TIME     4000             //�ϵ綨ʱ �ϵ��ȶ�
                          
#define STD_CLK_FREQ      500000           //��׼ʱ��Ƶ��
//��׼ʱ����������������ж�ʱ��=0xFFFF*1000/STD_CLK_FREQ=131ms
#define STD_CLK_OVTM     (1000/8)          //��λ:8ms Լ1s 
#define STD_ECLK_OVTM     20000            //��λ:1ms Լ20s 

#define BACK_MEA_ERR_D    '0'  //�����У��״̬
#define CATCH_HB_FWEG     '1'  //ǰ�ض԰� ForWard  EDGE
#define CATCH_HB_BWEG     '2'  //���ض԰� BackWard EDGE
                          
#define BEEP_ONN          '1'  //���ȿ�
//�ڱ�Ŷ���

#define SSI0              SSI0_BASE       // SSI0   
#define SSI1              SSI1_BASE       // SSI1   
#define SSI2              SSI2_BASE       // SSI2   
#define SSI3              SSI3_BASE       // SSI3   

#define WATCHDOG0         WATCHDOG0_BASE  // Watchdog0
#define WATCHDOG1         WATCHDOG1_BASE  // Watchdog1

#define CAN0              CAN0_BASE   // CAN0  ��ַ
#define CAN1              CAN1_BASE   // CAN1  ��ַ
 
#define UART0             UART0_BASE  // UART0 ��ַ
#define UART1             UART1_BASE  // UART1 ��ַ
#define UART2             UART2_BASE  // UART2 ��ַ
#define UART3             UART3_BASE  // UART3 ��ַ
#define UART4             UART4_BASE  // UART4 ��ַ
#define UART5             UART5_BASE  // UART5 ��ַ
#define UART6             UART6_BASE  // UART6 ��ַ
#define UART7             UART7_BASE  // UART7 ��ַ

#define TIMER0            TIMER0_BASE // Timer0
#define TIMER1            TIMER1_BASE // Timer1
#define TIMER2            TIMER2_BASE // Timer2
#define TIMER3            TIMER3_BASE // Timer3
#define TIMER4            TIMER4_BASE // Timer4
#define TIMER5            TIMER5_BASE // Timer5

#define PORTA             0   //PORTA �ڱ�Ŷ���
#define PORTB             1   //PORTB �ڱ�Ŷ���
#define PORTC             2   //PORTC �ڱ�Ŷ���
#define PORTD             3   //PORTD �ڱ�Ŷ���
#define PORTE             4   //PORTE �ڱ�Ŷ���
#define PORTF             5   //PORTF �ڱ�Ŷ���
#define PORTG             6   //PORTG �ڱ�Ŷ���
#define PORTH             7   //PORTH �ڱ�Ŷ���

#define GPIOA             GPIO_PORTA_BASE //PORTA �ڵ�ַ����             
#define GPIOB             GPIO_PORTB_BASE //PORTB �ڵ�ַ����             
#define GPIOC             GPIO_PORTC_BASE //PORTC �ڵ�ַ����             
#define GPIOD             GPIO_PORTD_BASE //PORTD �ڵ�ַ����             
#define GPIOE             GPIO_PORTE_BASE //PORTE �ڵ�ַ����             
#define GPIOF             GPIO_PORTF_BASE //PORTF �ڵ�ַ����             
#define GPIOG             GPIO_PORTG_BASE //PORTG �ڵ�ַ����             
#define GPIOH             GPIO_PORTH_BASE //PORTH �ڵ�ַ����             
#define GPIOK             GPIO_PORTK_BASE //PORTK �ڵ�ַ����             
//define �ܵ���Ӧ�˿�
//GPIOA
#define  UART0_GPIO      GPIOA
#define  SPI0_GPIO       GPIOA
#define  DISP_RST_GPIO   GPIOA
#define  GDT_MC_GPIO     GPIOA    //���ͷ����
#define  GDT_RST_GPIO    GPIOA
//GPIOB
#define  UART1_GPIO      GPIOB    //UART1
#define  TEST_LAMP_GPIO  GPIOB    //У��ָʾ�ƿ��� ��һ��λ��
#define  CAN0_GPIO       GPIOB    //CAN0
//GPIOC
#define  JTAG_GPIO       GPIOC    //����JTAG/SWD
#define  BEEP_GPIO       GPIOC    //  ���������� �ֹ�����               ����
#define  P3_OR_P4_GPIO   GPIOC    // ���Ĳ���ʱ �������� �������߿���  
//GPIOD
#define UA_JC_GPIO       GPIOD    // A���ѹ�̵�������   ����̨���� UA_JC   U1K_C	������ƻ���
#define UB_JC_GPIO       GPIOD    // B���ѹ�̵�������   ����̨���� UB_JC   U2K_C	�����������
#define UC_JC_GPIO       GPIOD    // C���ѹ�̵�������   ����̨���� UC_JC   U3K_C
#define UA_ESWC_GPIO     GPIOD    // A���ѹ���ӿ��ؿ��� ����̨���� UA_ESWC U4K_C
#define CD4094_DIN_GPIO  GPIOD    //4094 �������� 595
#define CD4094_STR_GPIO  GPIOD    //4094 �������� 595
//����̨����    
#define ION_JC_GPIO      GPIOD    //�����̵���ͨ ����̨�� UB��ѹ�̵��������ź�
#define IOFF_JC_GPIO     GPIOD    //�����̵����� ����̨�� UC��ѹ�̵��������ź�
//GPIOE
#define HC165_SL_GPIO    GPIOE    //HC165��λ/���� �ߵ�ƽ��λʹ�� �͵�ƽ����ʹ��
#define CD4094_CLK_GPIO  GPIOE    //595ʱ�� 
#define HC165_PIN_GPIO   GPIOE    //74HC165 ����Qh PB4
#define MC_OUT1_GPIO     GPIOE    //ר���նˢ���(2013��)�������1
#define MC_OUT2_GPIO     GPIOE    //ר���նˢ���(2013��)�������2
#define KEY_IN_GPIO      GPIOE    //��������
//GPIOF
#define DZ_MC_GPIO       GPIOF    //CCP0��������
#define JZ_IN_GPIO       GPIOF    //CCP1���Ⱦ�������ӿ�
#define SZ_MC_GPIO       GPIOF    //CCP2ʱ������
#define GP_BK_GPIO       GPIOF    //CCP3 �����׼���Ƶ���� GP 
#define FH_IN_GPIO       GPIOF  	//CCP4��׼���Ƶ����
#define PWM_DAC_GPIO     GPIOF    //CCP5 PWMģ�����
#define UC_ESWC_GPIO     GPIOF    //���� * �ı� C���ѹ���ӿ��ؿ��� ����̨����  UC_ESWC
#define UB_ESWC_GPIO     GPIOF    //���� * �ı� B���ѹ���ӿ��ؿ��� ����̨����  UB_ESWC

#define U_IN_CTL_GPIO    GPIOF    //��ѹ������� 1 ����1�Ŷ���(Ĭ��) 0:����3�Ŷ��� ĳЩ����̨ʹ�� ��B����ӿ��ؿ��Ƹ���
//GPIOG
#define BW_GPIO          GPIOG    //��λ��

//PORTH   
#define GOG_KZ_GPIO      GPIOH    //��������干�߹���ѡ�����
#define MC_PN_KZ_GPIO    GPIOH    //�ı䱻����������������� ���������� Positive(0) or Negative(1)
#define MC_WV_KZ_GPIO    GPIOH    //�ı䱻����������������� �������޹� Watt(0) or Var(1)
#define XL_MC_GPIO       GPIOH    //     �ı�* ������������
#define TQ_MC_GPIO       GPIOH    //���� �ı�* ʱ��Ͷ������
#define HZ_MC_GPIO       GPIOH    //���� �ı�* ��բ����
#define TX_MC_GPIO       GPIOH    //���� �ı�* ͨ��ָʾ
#define TXXZ_MC_GPIO     GPIOH    //��� ͨ��ѡ�� ����ͨ�� AB�ڶ�ͨ���л�

//PORTK                 
#define HC165_PDN_GPIO   GPIOK    //HC165ʱ�ӽ��� �ߵ�ƽ���� �͵�ƽʱ����Ч
#define HC165_CLK_GPIO   GPIOK    //HC165ʱ������ �������������

//�ܽŶ���
//PORTA
#define U0RX     GPIO_PIN_0	  //UART0����
#define U0TX     GPIO_PIN_1	  //UART0����
#define SSICLK   GPIO_PIN_2   //SSICLK HD7279 ʱ��
#define SSIFSS   GPIO_PIN_3   //SSIFSS HD7279 Ƭѡ
#define DISP_RST GPIO_PIN_4   //SSIRX  HD7279 ��λ
#define SSITX    GPIO_PIN_5   //SSITX  HD7279 ����
#define GDT_MC  GPIO_PIN_6  //���ͷ����
#define GDT_RST GPIO_PIN_7  //���ͷ��λ�ź�

#define GDTMC_IN    GPIOPinRead(GPIOA,GDT_MC)  //���������������

//PORTB
#define U1RX      GPIO_PIN_0   //UART1����
#define U1TX      GPIO_PIN_1   //UART1����
#define TEST_LAMP GPIO_PIN_2   //* У��ָʾ�ƿ��� ��һ��λ��

#define CANRX     GPIO_PIN_4   //CAN����
#define CANTX     GPIO_PIN_5   //CAN����

//PORTC
#define TCK      GPIO_PIN_0   //����TCK SWCLK SW����
#define TMS      GPIO_PIN_1   //����TMS SWDIO SW����
#define TDI      GPIO_PIN_2   //����TDI
#define TDO      GPIO_PIN_3   //����TDO SWO   SW����

#define BEEP     GPIO_PIN_7   //  ���������� �ֹ�����               ����
#define P3_OR_P4 GPIO_PIN_7   //* ���Ĳ���ʱ �������� �������߿���  

//PORTD                       
#define UA_JC    GPIO_PIN_0   // A���ѹ�̵�������   ����̨���� UA_JC   U1K_C	������ƻ���
#define UB_JC    GPIO_PIN_1   // B���ѹ�̵�������   ����̨���� UB_JC   U2K_C	�����������
#define UC_JC    GPIO_PIN_2   // C���ѹ�̵�������   ����̨���� UC_JC   U3K_C
#define UA_ESWC  GPIO_PIN_3   // A���ѹ���ӿ��ؿ��� ����̨���� UA_ESWC U4K_C
#define CD4094_DIN GPIO_PIN_4   //4094 �������� 595
#define CD4094_STR GPIO_PIN_5   //4094 �������� 595

//����̨����    
#define ION_JC   GPIO_PIN_1   //�����̵���ͨ ����̨�� UB��ѹ�̵��������ź�
#define IOFF_JC  GPIO_PIN_2   //�����̵����� ����̨�� UC��ѹ�̵��������ź�

//PORTE                       
#define HC165_SL    GPIO_PIN_0  //HC165��λ/���� �ߵ�ƽ��λʹ�� �͵�ƽ����ʹ��
#define CD4094_CLK  GPIO_PIN_1  //595ʱ�� 
#define HC165_PIN   GPIO_PIN_2  //74HC165 ����Qh PB4
#define MC_OUT1     GPIO_PIN_3  //ר���նˢ���(2013��)�������1
#define MC_OUT2     GPIO_PIN_4  //ר���նˢ���(2013��)�������2
#define KEY_IN      GPIO_PIN_5  //��������

//PORTF
#define DZ_MC       GPIO_PIN_0  //CCP0��������
#define JZ_IN       GPIO_PIN_1  //CCP1���Ⱦ�������ӿ�
#define SZ_MC       GPIO_PIN_2  //CCP2ʱ������
#define GP_BK       GPIO_PIN_3  //CCP3 �����׼���Ƶ���� GP 
#define FH_IN       GPIO_PIN_4	//CCP4��׼���Ƶ����
#define PWM_DAC     GPIO_PIN_5  //CCP5 PWMģ�����
#define UC_ESWC     GPIO_PIN_6  //���� * �ı� C���ѹ���ӿ��ؿ��� ����̨����  UC_ESWC
#define UB_ESWC     GPIO_PIN_7  //���� * �ı� B���ѹ���ӿ��ؿ��� ����̨����  UB_ESWC

#define U_IN_CTL      GPIO_PIN_7   //��ѹ������� 1 ����1�Ŷ���(Ĭ��) 0:����3�Ŷ��� ĳЩ����̨ʹ�� ��B����ӿ��ؿ��Ƹ���

#define DZMC_IN     GPIOPinRead(GPIOF,DZ_MC)   //����������������
#define SZMC_IN     GPIOPinRead(GPIOF,SZ_MC)   //ʱ��������������

//PORTG                       
#define BW       (GPIO_PIN_0|GPIO_PIN_1| \
                  GPIO_PIN_2|GPIO_PIN_3| \
                  GPIO_PIN_4|GPIO_PIN_5| \
                  GPIO_PIN_6|GPIO_PIN_7)

//PORTH   
#define GOG_KZ   GPIO_PIN_0   //��������干�߹���ѡ�����
#define MC_PN_KZ GPIO_PIN_1   //�ı䱻����������������� ���������� Positive(0) or Negative(1)
#define MC_WV_KZ GPIO_PIN_2   //�ı䱻����������������� �������޹� Watt(0) or Var(1)
#define XL_MC    GPIO_PIN_3   //     �ı�* ������������
#define TQ_MC    GPIO_PIN_4   //���� �ı�* ʱ��Ͷ������
#define HZ_MC    GPIO_PIN_5   //���� �ı�* ��բ����
#define TX_MC    GPIO_PIN_6   //���� �ı�* ͨ��ָʾ
#define TXXZ_MC  GPIO_PIN_7   //��� ͨ��ѡ�� ����ͨ�� AB�ڶ�ͨ���л�

//PORTK                 
#define HC165_PDN GPIO_PIN_2   //HC165ʱ�ӽ��� �ߵ�ƽ���� �͵�ƽʱ����Ч
#define HC165_CLK GPIO_PIN_3   //HC165ʱ������ �������������

//I/O�ں궨��
//PORTA
#define DISP_RST_EN    GPIOPinWrite(GPIOA,DISP_RST,0);       //��λ�ܽ���0
#define DISP_RST_DN    GPIOPinWrite(GPIOA,DISP_RST,DISP_RST) //��λ�ܽ���1
#define GDT_RST_EN     GPIOPinWrite(GPIOA,GDT_RST,GDT_RST)   //���ͷ��λʹ�� �ߵ�ƽʹ��
#define GDT_RST_DN     GPIOPinWrite(GPIOA,GDT_RST,0)         //���ͷ��λ���� �͵�ƽ����

//PORTB
#define TEST_LAMP_ON   GPIOPinWrite(GPIOB,TEST_LAMP,TEST_LAMP) //У��ָʾ����
#define TEST_LAMP_OFF  GPIOPinWrite(GPIOB,TEST_LAMP,0)       //У��ָʾ����

//PORTC
//#define BEEP_ON        GPIOPinWrite(GPIOC,BEEP_ON,BEEP_ON)   //������ ��
//#define BEEP_OFF       GPIOPinWrite(GPIOC,BEEP_ON,0)         //������ ��
#define WIRE_P3_CTL      GPIOPinWrite(GPIOC,P3_OR_P4,0)        //��������
#define WIRE_P4_CTL      GPIOPinWrite(GPIOC,P3_OR_P4,P3_OR_P4) //�������߻���

//PORTD 
#define UA_JDQ_ON      GPIOPinWrite(GPIOD,UA_JC,0)           //A���ѹ�̵�������
#define UA_JDQ_OFF     GPIOPinWrite(GPIOD,UA_JC,UA_JC)     //A���ѹ�̵����Ͽ�
#define UB_JDQ_ON      GPIOPinWrite(GPIOD,UB_JC,0)           //B���ѹ�̵�������
#define UB_JDQ_OFF     GPIOPinWrite(GPIOD,UB_JC,UB_JC)     //B���ѹ�̵����Ͽ�
#define UC_JDQ_ON      GPIOPinWrite(GPIOD,UC_JC,0)           //C���ѹ�̵�������
#define UC_JDQ_OFF     GPIOPinWrite(GPIOD,UC_JC,UC_JC)     //C���ѹ�̵����Ͽ�
#define UA_ESW_ON      GPIOPinWrite(GPIOD,UA_ESWC,0)         //A���ѹ���ӿ��ؽ���
#define UA_ESW_OFF     GPIOPinWrite(GPIOD,UA_ESWC,UA_ESWC) //A���ѹ���ӿ��ضϿ�
//����̨����
#define I_IN_EN        GPIOPinWrite(GPIOD,ION_JC|IOFF_JC,ION_JC)        //�������� ����̨ʹ�� �͵�ƽ �̵����Ͽ�
#define I_BYPASS_EN    GPIOPinWrite(GPIOD,ION_JC|IOFF_JC,ION_JC|IOFF_JC)//������· ����̨ʹ�� �ߵ�ƽ �̵������� ������·
#define I_JDQ_EN_CNCL  GPIOPinWrite(GPIOD,ION_JC,0)          //�̵���ʹ���źų���
#define I_JDQ_CTL_CNCL GPIOPinWrite(GPIOD,IOFF_JC,0)         //�̵��������źų���   

#define CD4094_DIN_H   GPIOPinWrite(GPIOD,CD4094_DIN,CD4094_DIN)//4094 ��������1
#define CD4094_DIN_L   GPIOPinWrite(GPIOD,CD4094_DIN,0)         //4094 ��������0

#define CD4094_STR_L   GPIOPinWrite(GPIOD,CD4094_STR,0)         //4094 �����������ʹ��
#define CD4094_STR_H   GPIOPinWrite(GPIOD,CD4094_STR,CD4094_STR)//4094 ��������������

//PORTE
#define HC165_IN       GPIOPinRead(GPIOE,HC165_PIN)            //HC165��������
#define CD4094_CLK_H   GPIOPinWrite(GPIOE,CD4094_CLK,CD4094_CLK)//CD4094 ʱ������ߵ�ƽ
#define CD4094_CLK_L   GPIOPinWrite(GPIOE,CD4094_CLK,0)         //CD4094 ʱ������͵�ƽ
#define YX3_OUT_LOW      GPIOPinWrite(GPIOE,MC_OUT1,0)         //�����ź�1�����
#define YX3_OUT_HIGH     GPIOPinWrite(GPIOE,MC_OUT1,MC_OUT1)   //�����ź�1�����
#define YX4_OUT_LOW      GPIOPinWrite(GPIOE,MC_OUT2,0)           //�����ź�2�����
#define YX4_OUT_HIGH     GPIOPinWrite(GPIOE,MC_OUT2,MC_OUT2)   //�����ź�2�����
#define YX3_OUT_Reverse  GPIOPinWrite(GPIOE,MC_OUT1,~GPIOPinRead(GPIOC,MC_OUT1))//�����ź�1��ת
#define YX4_OUT_Reverse  GPIOPinWrite(GPIOE,MC_OUT2,~GPIOPinRead(GPIOC,MC_OUT2))//�����ź�2��ת
#define HC165_SHIFT    GPIOPinWrite(GPIOE,HC165_SL,HC165_SL)  //HC165��λʹ��
#define HC165_LOAD     GPIOPinWrite(GPIOE,HC165_SL,0)           //HC165����ʹ��

//PORTF
#define UB_ESW_ON      GPIOPinWrite(GPIOF,UB_ESWC,0)         //B���ѹ���ӿ��ؽ���
#define UB_ESW_OFF     GPIOPinWrite(GPIOF,UB_ESWC,UB_ESWC)   //B���ѹ���ӿ��ضϿ�
#define UC_ESW_ON      GPIOPinWrite(GPIOF,UC_ESWC,0)         //C���ѹ���ӿ��ؽ���
#define UC_ESW_OFF     GPIOPinWrite(GPIOF,UC_ESWC,UC_ESWC)   //C���ѹ���ӿ��ضϿ�

//PORTH  
#define POS_Watt_SEL   GPIOPinWrite(GPIOH,MC_PN_KZ|MC_WV_KZ,0)                //ѡ�������й�����
#define NEG_Watt_SEL   GPIOPinWrite(GPIOH,MC_PN_KZ|MC_WV_KZ,MC_PN_KZ)         //ѡ�����й�����
#define POS_Var_SEL    GPIOPinWrite(GPIOH,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ)         //ѡ�������޹�����
#define NEG_Var_SEL    GPIOPinWrite(GPIOH,MC_PN_KZ|MC_WV_KZ,MC_WV_KZ|MC_PN_KZ)//ѡ�����޹�����
#define DOWN_JOIN      GPIOPinWrite(GPIOH,GOG_KZ,GOG_KZ)     //���Ͷ� ���伫����һ�� E
#define UP_JOIN        GPIOPinWrite(GPIOH,GOG_KZ,0)            //���߶� ���缫����һ�� C
#define TX_MC_ON       GPIOPinWrite(GPIOH,TX_MC,TX_MC)       //ͨ��ָʾ����
#define TX_MC_OFF      GPIOPinWrite(GPIOH,TX_MC,0)             //ͨ��ָʾ����
/*
#define RED_485_EN     GPIOPinWrite(GPIOF,TXXZ_MC,TXXZ_MC)     //��� ͨ��ѡ�� ����ͨ�� AB�ڶ�ͨ���л�
#define RED_485_DN     GPIOPinWrite(GPIOF,TXXZ_MC,0)           //��� ͨ��ѡ�� AB�ڶ�ͨ���л� Ĭ�ϵڶ�ͨ��
*/
#define WDI_HIGH       GPIOPinWrite(GPIOH,TXXZ_MC,TXXZ_MC)   //��� �ⲿ���Ź���λ         
#define WDI_LOW        GPIOPinWrite(GPIOH,TXXZ_MC,0)           //���           

//PORTK
#define HC165_DN       GPIOPinWrite(GPIOD,HC165_PDN,HC165_PDN)//HC165ʱ�ӽ���
#define HC165_EN       GPIOPinWrite(GPIOD,HC165_PDN,0)          //HC165ʱ��ʹ��
#define HC165_CLK_H    GPIOPinWrite(GPIOD,HC165_CLK,HC165_CLK)//HC165ʱ�ӹܽŸߵ�ƽ
#define HC165_CLK_L    GPIOPinWrite(GPIOD,HC165_CLK,0)        //HC165ʱ�ӹܽŵ͵�ƽ

//#define U_PORT1_IN     GPIOPinWrite(GPIOH,U_IN_CTL,U_IN_CTL) //��ѹ����1�Ŷ���
//#define U_PORT3_IN     GPIOPinWrite(GPIOH,U_IN_CTL,0)        //��ѹ����3�Ŷ���

//CAN���������ȶ���
#define CAN_SDILEN		 50      //CAN�����ݽ��ջ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����
#define CAN_SDOLEN		 50      //CAN�����ݷ��ͻ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����
                       
#define CAN_LDILEN		 20      //CAN�����ݽ��ջ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����
#define CAN_LDOLEN		 10      //CAN�����ݷ��ͻ���������	ÿ���ṹ�峤16�ֽ� 8�ֽ���Ч����

//�๦�ܱ�
#define MTRCOM_ILEN    2048    //COM0���ջ���������
#define MTRCOM_OLEN    1024    //COM0���ͻ��������� 
//����
#define LCTCOM_ILEN    2048    //COM1���ջ���������
#define LCTCOM_OLEN    1024    //COM1���ͻ��������� 


//CAN ���������� ��CANBitClkSettings[]�ж�Ӧ
#define CANBAUD_100K   100000
#define CANBAUD_125K   125000
#define CANBAUD_250K   250000
#define CANBAUD_500K   500000
#define CANBAUD_1M    1000000
#define CANBAUD	       CANBAUD_500K
//��Ԫ��CANָ֡��ֿ� ÿ256������Ϊһ��
#define ERR_CMD_ID     0              //�����㵥Ԫԭ������ CAN ������(0x004~0x103)
#define CLK_CMD_ID     1              //ʱ��У��������       CAN ������(0x104~0x203)
#define LCT_CMD_ID     2              //�����ն�ָ��         CAN ������(0x204~0x303)
//MTR_MOD�����豸���Ͷ���
#define Acquire_Terminal_13  '0'      //ר���նˢ���(2013��)��ʶ
#define Reading_Terminal_13  '1'      //����������(2013��)��ʶ
#define Smart_Meter          '2'      //���ܵ��
#define Acquire_Terminal_09  '3'      //ר���նˢ���(2009��)��ʶ
#define Reading_Terminal_09  '4'      //����������(2009��)��ʶ
//ԭ��������								   //0 1 2 3  Ԥ���������ݴ���
#define ERR_ICMD_GD       (ERR_CMD_ID*0x100+4) //���ձ�׼ֵ     XX ��� XXXX ���� 0---3Ϊ����������
#define ERR_ICMD_GE       (ERR_ICMD_GD+1)        //����У��Ȧ��
#define ERR_ICMD_GF       (ERR_ICMD_GE+1)        //�����������
#define ERR_ICMD_GG       (ERR_ICMD_GF+1)        //���չ��ͷ�Թ�
#define ERR_ICMD_GH       (ERR_ICMD_GG+1)        //ѡ��λ �ұ�ѡ��
#define ERR_ICMD_GI       (ERR_ICMD_GH+1)        //�Ժڰ�         ;0��ȥ�������飬�ص�У��״̬
#define ERR_ICMD_GJ       (ERR_ICMD_GI+1)        //���ӹ��ͷ���� ;1��/Ǳ��ʱ���� 0��ȥ�������飬�ص�У��״̬
#define ERR_ICMD_GK       (ERR_ICMD_GJ+1)        //���յ�ǰ�������� 
#define ERR_ICMD_GL       (ERR_ICMD_GK+1)        //���ȿ���
#define ERR_ICMD_GM       (ERR_ICMD_GL+1)        //׼����ʼ��������
#define ERR_ICMD_GN       (ERR_ICMD_GM+1)        //��ʼ������������
#define ERR_ICMD_GO       (ERR_ICMD_GN+1)        //���ձ�У����
#define ERR_ICMD_GP       (ERR_ICMD_GO+1)        //����ѡ��  �������廹�ǹ��ͷ����
#define ERR_ICMD_GQ       (ERR_ICMD_GP+1)        //�Ƶ�������      1:��ʼ  0:��ֹ
#define ERR_ICMD_GR       (ERR_ICMD_GQ+1)        //������ܼ���
#define ERR_ICMD_GS       (ERR_ICMD_GR+1)        //����Ƚ�  ��������͹��ͷ����Ƚ�
#define ERR_ICMD_GT       (ERR_ICMD_GS+1)        //�����������ת��ֵ
#define ERR_ICMD_GU       (ERR_ICMD_GT+1)        //�����������ת�趨�Ƚ�Ȧ��
#define ERR_ICMD_GV       (ERR_ICMD_GU+1)        //����ʧѹ״̬,����ʧѹ����
#define ERR_ICMD_GW       (ERR_ICMD_GV+1)        //ʧѹ���鿪ʼ
#define ERR_ICMD_GX       (ERR_ICMD_GW+1)        //������������ʱ�������������������ת��
#define ERR_ICMD_GY       (ERR_ICMD_GX+1)        //�ͷ�Ƶϵ��
#define ERR_ICMD_GZ       (ERR_ICMD_GY+1)        //�ƶ����巨������������
#define ERR_ICMD_Gd       (ERR_ICMD_GZ+1)        //���Ĳ��� ��λ�����ѡ��
#define ERR_ICMD_Ge       (ERR_ICMD_Gd+1)        //���Ĳ����� ��ѹ��Ȧ�й�����ֵ
#define ERR_ICMD_Gf       (ERR_ICMD_Ge+1)        //��ѹ��Ȧ���ڹ�������ֵ
#define ERR_ICMD_Gg       (ERR_ICMD_Gf+1)        //���յ�����Ȧ���ڹ�������ֵ
#define ERR_ICMD_Gh       (ERR_ICMD_Gg+1)        //���ù��Ĳ�����Ԫ��Ԫ��
#define ERR_ICMD_Gi       (ERR_ICMD_Gh+1)        //���岨�β���
#define ERR_ICMD_Gj       (ERR_ICMD_Gi+1)        //���ս��߷�ʽ
#define ERR_ICMD_Gk       (ERR_ICMD_Gj+1)        //4059��У�������������ѡ��
#define ERR_ICMD_Gl       (ERR_ICMD_Gk+1)        //��������������
#define ERR_ICMD_Gm       (ERR_ICMD_Gl+1)        //������������
#define ERR_ICMD_Gn       (ERR_ICMD_Gm+1)        //Ԥ��������
//��Ԫ��������(�๦��������ѹ̨ͨѶ����)
#define ERR_ICMD_Go       (ERR_ICMD_Gn+1)        //��ʼ��ѹ����
#define ERR_ICMD_Gp       (ERR_ICMD_Go+1)        //��ѹʱ�䵽
#define ERR_ICMD_Gq       (ERR_ICMD_Gp+1)        //�л���β��ѹ����
#define ERR_ICMD_Gr       (ERR_ICMD_Gq+1)        //���У��ָʾ�ƿ�������
#define ERR_ICMD_Gs       (ERR_ICMD_Gr+1)        //�̵�������        
#define ERR_ICMD_Gt       (ERR_ICMD_Gs+1)        //�̵����Ͽ�
#define ERR_ICMD_Gu       (ERR_ICMD_Gt+1)        //������·��·���
#define ERR_ICMD_SYPH     (ERR_ICMD_Gu+1)        //��ѹ�������
#define ERR_ICMD_MIN_CST  (ERR_ICMD_SYPH+1)      //��С�����
#define ERR_ICMD_ZZDS     (ERR_ICMD_MIN_CST+1)   //����У�˳�������
#define ERR_ICMD_MCHC     (ERR_ICMD_ZZDS+1)      //��������ϳɷ�ʽ
#define ERR_ICMD_SYFAN    (ERR_ICMD_MCHC+1)      //����ʧѹ����
#define ERR_ICMD_MFCLKMD  (ERR_ICMD_SYFAN+1)     //���ö๦�����巽ʽ  ���û��Ƿֿ�
#define ERR_ICMD_MFCLKTY  (ERR_ICMD_MFCLKMD+1)   //��ǰ�๦����������  ʱ������ �������� Ͷ������ ...
#define ERR_ICMD_ELEPLS   (ERR_ICMD_MFCLKTY+1)   //���õ����������� ������ PA PR QA QR '1' '2' '3' '4'
#define ERR_ICMD_IJDQR    (ERR_ICMD_ELEPLS+1)    //������·�̵�����λ
#define ERR_ICMD_LEDTST   (ERR_ICMD_IJDQR+1)     //����������ʾ8.8.8.8.8.8.8.8.
#define ERR_ICMD_RSTLED   (ERR_ICMD_LEDTST+1)    //��λ7279
#define ERR_ICMD_TZEN     (ERR_ICMD_RSTLED+1)    //��բ���ʹ������       
#define ERR_ICMD_MBJEN	  (ERR_ICMD_TZEN+1)      //�����źż��ʹ�� 
#define ERR_ICMD_TS       (ERR_ICMD_MBJEN+1)     //������̨��������
#define ERR_ICMD_DXTZ     (ERR_ICMD_TS+1)        //���õ�����բ
#define ERR_ICMD_START    (ERR_ICMD_DXTZ+1)      //�ܿ������������� 
#define ERR_ICMD_SETCK    (ERR_ICMD_START+1)     //���ò忨
#define ERR_ICMD_UCLOP    (ERR_ICMD_SETCK+1)     //���ñ�λ��ѹ���� �Ͽ� 
#define ERR_ICMD_ZBTST	  (ERR_ICMD_UCLOP+1)     //�ر�����
#define ERR_ICMD_LIGHT    (ERR_ICMD_ZBTST+1)     //����״ָ̬ʾ�ƿ���
#define ERR_ICMD_SLEDN    (ERR_ICMD_LIGHT+1)     //��������LED�������ʾλ��
#define ERR_ICMD_VSET     (ERR_ICMD_SLEDN+1)     //�����ز�������ѹ�������
#define ERR_ICMD_COM0SEL  (ERR_ICMD_VSET+1)      //���ý���UART0��ͨѶ��ʽ
#define ERR_ICMD_COM1SEL  (ERR_ICMD_COM0SEL+1)   //���ý���UART1��ͨѶ��ʽ       
#define ERR_ICMD_ZBLBIN   (ERR_ICMD_COM1SEL+1)   //�ز��˲���·����ѡ��          
#define ERR_ICMD_YXCTL    (ERR_ICMD_ZBLBIN+1)    //ң���źſ�������              
#define ERR_ICMD_DOORCTL  (ERR_ICMD_YXCTL+1)     //�ſ��źſ�������                  
#define ERR_ICMD_EQPMOD   (ERR_ICMD_DOORCTL+1)   //����豸��������              
#define ERR_ICMD_PULSET   (ERR_ICMD_EQPMOD+1)    //���������������(���ר���ն�)
#define ERR_ICMD_PULRUN   (ERR_ICMD_PULSET+1)    //���������������(���ר���ն�)
#define ERR_ICMD_YKSET    (ERR_ICMD_PULRUN+1)    //�ն�ң���ź�����              
#define ERR_ICMD_YKPULSE  (ERR_ICMD_YKSET+1)     //����ң���ź����崥����ʽ
#define ERR_ICMD_NPRS485  (ERR_ICMD_YKPULSE+1)   //�����޼���RS485
#define ERR_ICMD_MTYPE    (ERR_ICMD_NPRS485+1)   //������ ������ 'N'  ������'S' 
//��Ԫ��������(ʱ��У���ǲ�������)
#define ERR_ICMD_MBAUD    (CLK_CMD_ID*0x100+4)   //���ö๦�ܱ�(ģ���)����ͨ�Ų���
#define ERR_ICMD_LBAUD    (ERR_ICMD_MBAUD+1)     //���ø����ն˴���ͨ�Ų���
#define ERR_ICMD_CLKFRQ   (ERR_ICMD_LBAUD+1)     //���ø���λʱ��Ƶ��
#define ERR_ICMD_CLKTIM   (ERR_ICMD_CLKFRQ+1)    //���øñ�λʱ��Ƶ�ʲ���ʱ��
#define ERR_ICMD_CLKCTL   (ERR_ICMD_CLKTIM+1)    //ʱ��Ƶ�ʲ�������
#define ERR_ICMD_XULCTL   (ERR_ICMD_CLKCTL+1)    //�������ڲ�������
#define ERR_ICMD_XULPLS   (ERR_ICMD_XULCTL+1)    //�����������ڲ�������
#define ERR_ICMD_EDIS     (ERR_ICMD_XULPLS+1)    //������ʾ��ʽ
#define ERR_ICMD_RLMDT    (ERR_ICMD_EDIS+1)      //����װ��ģ�������     ��λ��   -> ģ���   
#define ERR_ICMD_RLLDT    (ERR_ICMD_RLMDT+1)     //����װ�ظ����ն�����   ��λ��   -> �����ն� 
#define ERR_ICMD_RTMDT    (ERR_ICMD_RLLDT+1)     //�ط��յ���ģ�������   ģ���   -> ��λ��   
#define ERR_ICMD_RTLDT    (ERR_ICMD_RTMDT+1)     //�ط��յ��ĸ����ն����� �����ն� -> ��λ��   
#define ERR_ICMD_SEC      (ERR_ICMD_RTLDT+1)     //ʱ���׼ ���ź�
#define ERR_ICMD_RED485	  (ERR_ICMD_SEC+1)       //����RS485�ڶ�ͨ�� '1' �ӱ�485�ڶ�ͨ�� '2' �ӱ���� 
#define ERR_ICMD_READ_VER (ERR_ICMD_RED485+1)	   //���汾������
#define ERR_ICMD_RSTS     (ERR_ICMD_READ_VER+1)  //��С��ʾ����״̬
#define ERR_ICMD_SMTR     (ERR_ICMD_RSTS+1)      //���ü����Ǳ��׼�� ����Э�� ���������δ�� 
//�����ն����� LCT_CMD_ID*0x100+4 
#define TX_MTR_DATA       (LCT_CMD_ID*0x100+4)   //����ģ������ݵ������ն�
#define TX_LCT_DATA       (TX_MTR_DATA+1)        //�������ݵ������ն�

//��������
#define JTAG_EN           0x7DF                  //2015 ���һ������ 
//����ԭ������������
#define ERR_OCMD_CB       (ERR_CMD_ID*0x100+4)   //  XXXXXX(0DH)  ���ͱ�У��������(�ۼ�ֵ)
#define ERR_OCMD_CC       (ERR_OCMD_CB+1)        //  X(0DH)       �𶯡�Ǳ������ʱȦ�� 
#define ERR_OCMD_CE       (ERR_OCMD_CC+1)        //  ��XXXX(0DH)   ���
#define ERR_OCMD_CF       (ERR_OCMD_CE+1)        //  ��XXX(0DH)    ��ת���Ȧ��
#define ERR_OCMD_CG       (ERR_OCMD_CF+1)        //  X(0DH)       X:0δ�ұ�X:1 �ұ�
#define ERR_OCMD_CH       (ERR_OCMD_CG+1)        //  :XXX(0DH)    ʪ��  
#define ERR_OCMD_CI       (ERR_OCMD_CH+1)        //  ��XXXX(0DH)   XXXX���
#define ERR_OCMD_CK       (ERR_OCMD_CI+1)        //  1(0DH)       �ۻ���������ֹͣ
#define ERR_OCMD_CM       (ERR_OCMD_CK+1)        //  XXXXXX,XXXXXX(0DH) �ͳ���У����������, ռ�ձ�
                                               //  ǰ��������ǣ���У������ߵ�ƽռ��ʱ��
                                               //  ����������ǣ���У������͵�ƽռ��ʱ��
#define ERR_OCMD_CN       (ERR_OCMD_CM+1)        //  XX(0DH)      XX��ǰȦ��
#define ERR_OCMD_CP       (ERR_OCMD_CN+1)        //  XXXXXX(0DH)  XXX.XXX��У�����
#define ERR_OCMD_CS       (ERR_OCMD_CP+1)        //  X(0DH)       X:0,δ�Ժð�,X:1���ѶԺ�
#define ERR_OCMD_CT       (ERR_OCMD_CS+1)        //  :XXX(0DH)    �¶�  
#define ERR_OCMD_CV       (ERR_OCMD_CT+1)        //  X��0DH��     X=0��δ֪ X=1����·X=2������·
#define ERR_OCMD_CW       (ERR_OCMD_CV+1)        //  :XXXXXX,XXXXXXX,XXXXXX   ���Ĳ�������
#define ERR_OCMD_CX       (ERR_OCMD_CW+1)        //  X(0DH)       ��ѹ���
#define ERR_OCMD_CY       (ERR_OCMD_CX+1)        //  X(0DH)       ����ʧѹ��������������
#define ERR_OCMD_CZ       (ERR_OCMD_CY+1)        //  X(0DH)       X=0: �������鿪ʼ,X=1: �����������

//����ʱ����������
#define ERR_OCMD_CLK_FRQ  (CLK_CMD_ID*0x100+4)   //  ���ͱ����ʱ��Ƶ��
#define ERR_OCMD_DAY_ERR  (ERR_OCMD_CLK_FRQ+1)   //  �����ռ�ʱ���
#define ERR_OCMD_XULZQ    (ERR_OCMD_DAY_ERR+1)   //  ������������
#define ERR_OCMD_TQMC     (ERR_OCMD_XULZQ+1)     //  �յ�ʱ��Ͷ������
#define ERR_OCMD_NZTZ     (ERR_OCMD_TQMC+1)      //  ������բ�ź�״̬
#define ERR_OCMD_WZTZ     (ERR_OCMD_NZTZ+1)      //  ������բ�ź�״̬ 
#define ERR_OCMD_JDQGZ    (ERR_OCMD_WZTZ+1)      //  �����̵�������״̬
#define ERR_OCMD_MBJ      (ERR_OCMD_JDQGZ+1)     //  �������״̬
#define ERR_OCMD_VER      (ERR_OCMD_MBJ+1)       //  ��������汾��
#define ERR_OCMD_STS      (ERR_OCMD_VER+1)       //  �������幤��״̬
#define ERR_OCMD_BERR     (ERR_OCMD_STS+1)       //  ���ͱ�׼�����(������׼�� ������Ԥ��)
#define ERR_OCMD_DZPLS    (ERR_OCMD_BERR+1)      //  ��⵽��������
#define ERR_OCMD_GDPLS    (ERR_OCMD_DZPLS+1)     //  ��⵽�������
#define ERR_OCMD_SZPLS    (ERR_OCMD_GDPLS+1)     //  ��⵽ʱ������
#define ERR_OCMD_MTRDY    (ERR_OCMD_SZPLS+1)     //  ���ޱ�������� 
#define ERR_OCMD_LUNCI1   (ERR_OCMD_MTRDY+1)     //  �ִ�1����״̬��������
#define ERR_OCMD_LUNCI2   (ERR_OCMD_LUNCI1+1)    //  �ִ�2����״̬��������
#define ERR_OCMD_LUNCI3   (ERR_OCMD_LUNCI2+1)    //  �ִ�3����״̬��������
#define ERR_OCMD_LUNCI4   (ERR_OCMD_LUNCI3+1)    //  �ִ�4����״̬��������
