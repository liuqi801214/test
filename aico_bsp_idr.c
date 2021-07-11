#include "string.h"
#include "System_rtl876x.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "rtl876x_ir.h"
#include "trace.h"
#include "app_section.h"
#include "platform_types.h"
#include "platform_diagnose.h"
#include "platform_os.h"
#include "aico_bsp_idr.h"
#include "aico_bsp_irl.h"
#include "board.h"
#include <os_msg.h>
#include <app_msg.h>



#if RTK_EVB
#define IR_TX_PIN                   P2_5
#else
#define IR_TX_PIN                   P1_0//
#endif
#define IR_TX_FIFO_THR_LEVEL        2


#define PULSE_HIGH                  ((uint32_t)0x80000000)
#define PULSE_LOW                   0x0
#define LOG_HIGH                    1
#define LOG_LOW                     0
#define MAX_CODE_SIZE               12

#define AICO_REMOTE_COMMAND_HEAD_TYPE_1 0xA8
#define AICO_REMOTE_COMMAND_HEAD_TYPE_2 0xA9
#define AICO_REMOTE_COMMAND_HEAD_TYPE_3 0xAA
#define AICO_REMOTE_COMMAND_HEAD_TYPE_4 0xAB
#define AICO_REMOTE_COMMAND_HEAD_TYPE_5 0xAC
#define AICO_REMOTE_COMMAND_HEAD_TYPE_6 0xAE
#define AICO_REMOTE_COMMAND_HEAD_TYPE_7 0xA6


#define  KEY_LEN 12


//数据头 8bit
struct Aico_Remote_Command_Head
{
    u8 aico_remote_command_type;
};

//载波周期类型
#define AICO_REMOTE_COMMAND_CARRIER_CYCLE_1 0x11,
#define AICO_REMOTE_COMMAND_CARRIER_CYCLE_2 0x1a,
#define AICO_REMOTE_COMMAND_CARRIER_CYCLE_3 0x64,

//载波
struct Aico_Remote_Command_Carrier
{
    u16 aico_remote_command_carrier_cycle;  //载波周期 wwwwwwww 8bit
    u16 aico_remote_command_carriet_enable; //有无载波 type6-7有效 0x80有 其他无 8bit
};

//高低位 多帧数据 连续发码 GXXX 4bit
struct Aico_Remote_Command_Framecfg
{
    u8 aico_remote_command_framecfg_value;
    u8 aico_remote_command_framecfg_g;
    u8 aico_remote_command_framecfg_w;
    u8 aico_remote_command_framecfg_v;
};

//重复帧 NNNN 4bit
struct Aico_Remote_Command_Reframe
{
    u8 aico_remote_command_reframe_value;
};

//数据格式 YYYY 8-16bit
struct Aico_Remote_Command_Dataformat
{
    u16 aico_remote_command_dataformat_value;
};

//数据位 MMMMMMMM type1-3有效 8bit
struct Aico_Remote_Command_Databits
{
    u8 aico_remote_command_databits_value;
};

//数据周期 TTTTTTTT
struct Aico_Remote_Command_Datacycle
{
    u16 aico_remote_command_datacycle_start_addr;
    u8 aico_remote_command_datacycle_len;
};

//数据 DDDDDDDD
struct Aico_Remote_Command_Data
{
    u16 aico_remote_command_data_start_addr;
    u16 aico_remote_command_data_len;
};


//数据格式
struct Aico_Remote_Command
{
    struct Aico_Remote_Command_Head aico_remote_command_head;
    struct Aico_Remote_Command_Framecfg aico_remote_command_framecfg;
    struct Aico_Remote_Command_Carrier aico_remote_command_carrier;
    struct Aico_Remote_Command_Reframe aico_remote_command_reframe;
    struct Aico_Remote_Command_Dataformat aico_remote_command_dataformat;
    struct Aico_Remote_Command_Databits aico_remote_command_databits;
    struct Aico_Remote_Command_Datacycle aico_remote_command_datacycle;
    struct Aico_Remote_Command_Data aico_remote_command_data;
};

struct Aico_Remote_Command aico_remote_command;

uint16_t p_send_buf_len = 0;

/*-----------------------CMD_TYPE1---------------------------*/
//TTTT
struct Aico_Remote_Command_Datacycle_Type1
{
    u16 pilot_code_high_time;
    u16 pilot_code_low_time;
    u16 data0_high_time;
    u16 data0_low_time;
    u16 data1_high_time;
    u16 data1_low_time;
    u16 clearance0_time;
    u16 clearance1_time;
};
//YYYY
struct Aico_Remote_Command_Dataformat_Type1
{
    u8 pilot_code_enable;      // 11:有引导码 10:仅引导码高电平 00:无引导码
    u8 data_encoding_format;   // 00:正常数据 01:双向位编码 10:高为1 低为0
    u8 synchronous_bit_ebalbe; // 1:有同步位 0:无同步位
    u8 tail_code_enable;       // 0:无尾码 0:有尾码
    u8 code_sending_mode;      // 00:单次发码 01:有间隙连续发码 11:有间隙 超过65536US，连续发码
};
struct Aico_Remote_Command_Data_Type1
{
    struct Aico_Remote_Command_Datacycle_Type1 aico_remote_command_datacycle_type1;
    struct Aico_Remote_Command_Dataformat_Type1 aico_remote_command_dataformat_type1;
};

struct Aico_Remote_Command_Data_Type1 aico_remote_command_data_type1;

typedef enum
{
    IR_SEND_IDLE,
    IR_SEND_CAMMAND = 1,
    IR_SEND_REPEAT_CODE,

    IR_SEND_NOT_COMPLETED,
    IR_SEND_COMPLETED,
} _IR_SEND_STATE_;


//IRDA_BUF  gIrBuf = {0};
IRDA_BUF  *gIrBuf =0;

static uint16_t IR_TX_Count = 0;

static uint32_t HL_Time2TxBufCount(uint32_t a, uint32_t b)
{
    //return ((a & 0x80000000) | ((a & 0x7FFFFFFF) / b));
    return ((a & 0x80000000) | ((((a & 0x7FFFFFFF) * 100) / b) / 10));

}

void ir_send_int_handler(void) DATA_RAM_FUNCTION;

extern void *evt_queue_handle;  //!< Event queue handle
extern void *io_queue_handle;

struct Aico_Remote_Receive_Buf aico_remote_receive_buf;
#if  RTK_EVB_IR_TEST
struct Aico_Remote_Receive_Buf aico_remote_receive_buf_type1 =
{
    /*
      {AICO_REMOTE_COMMAND_HEAD_TYPE_1,0x00,0xc3,38,0x23,0x28,0x11,0x94,0x02,0x30,0x02,0x30,0x02,0x30,0x06,0x9a,0x00,0x00,0x00,0x00,96,0x30,0x30,0x30,0x30,0x30,0x30, 0x30,0x30,0x30, 0x30,0x30,0x30 },    //uint32_t HeaderContext[MAX_HEADDER_LEN];
    12,
    1,
        1,
      */

    {0xFF, 0xE9, 0x62, 0x2D, 0x77, 0x38, 0x30, 0x41, 0x43, 0x41, 0x33, 0x32, 0x60, 0x43, 0x15, 0x3A, 0x55, 0x0D, 0xDD, 0x41, 0x43, 0x51, 0xFE, 0xEF, 0xC7},
    25,
    1,
    1,

};
struct Aico_Remote_Receive_Buf aico_remote_receive_buf_type2 =
{

    /*
    {
     0xFF,0xE8,0xC2,0xB0,0x88,0x54,0xE1,0xB9,0x52,0xBB,
     0x61,0xF1,0xCC,0x47,0x57,0xF9,0xC4,0x32,0x24,0x51,
     0x82,0xB7,0xC7,0x24,0xE9,0xB9,0x52,0xC0,0xF1,0xCC,
     0x05,0xA9,0xA7,0x6F,0xC9,0xFA,0xF9,0xB5,0xC3,0x3B,
     0x3B,0xD0,0x70,0x0B,0x03,0x09},    //uint32_t HeaderContext[MAX_HEADDER_LEN];
    46,
    1,
     1,
    */
    {
        0xFF, 0xE8, 0xC2, 0xB0, 0x08, 0x50, 0xD3, 0x69, 0x52, 0xD7,
        0x61, 0xF2, 0x08, 0x43, 0x73, 0xFA, 0x00, 0x3E, 0xA0, 0x83,
        0x73, 0xC3, 0x43, 0x2C, 0xCC, 0xA2, 0x6B, 0xB0, 0xFA, 0xFA,
        0x00, 0xC1, 0x46, 0x74, 0xFF, 0xB0, 0x47, 0x5F, 0xA2
    },
    39,
    1,
    1,

};


struct Aico_Remote_Receive_Buf aico_remote_receive_buf_type3 =
{
    {
        0xFF, 0xEB, 0xC3, 0xB0, 0x58, 0x18, 0xC8, 0x51, 0x26, 0x41,
        0x56, 0x32, 0x32, 0x43, 0x54, 0x3E, 0x1B, 0x38, 0x30, 0x41,
        0x43, 0x0B, 0x2B, 0x5A, 0x4F, 0x41, 0x63, 0x7B, 0x30, 0x38, 0x07
    },    //uint32_t HeaderContext[MAX_HEADDER_LEN];
    31,
    1,
    1,
};

struct Aico_Remote_Receive_Buf aico_remote_receive_buf_type4 =
{
    {
        0xFF, 0xEA, 0xC3, 0x22, 0x33, 0x50, 0xE3, 0x65, 0x44, 0x93, 0x80, 0x6E, 0xB8, 0x40, 0xCD, 0x18, 0xE3, 0x1C, 0x37, 0x91, \
        0x02, 0x1D, 0x80, 0x6E, 0x3A, 0xFD, 0x82, 0x66, 0x36, 0x9C, 0x25, 0x41, 0x7B, 0x66, 0x79, 0x58, 0x08, 0x50, 0x99, 0x1F, \
        0xC0, 0x35, 0x30, 0x4E, 0x52, 0x99, 0x66, 0x30, 0xEB, 0x71, 0xB2, 0x38, 0x30, 0x28, 0x35, 0x49, 0x60
    },    //uint32_t HeaderContext[MAX_HEADDER_LEN];
    57,
    1,
    1,
};


struct Aico_Remote_Receive_Buf aico_remote_receive_buf_type5 =
{
    {
        0xFF, 0xED, 0x52, 0x13, 0x58, 0x38, 0x30, 0x41, 0x43, 0x43,
        0x89, 0x33, 0x34, 0x41, 0x8B, 0x3B, 0xA4, 0x0D, 0xD8, 0x79,
        0xC2, 0x57, 0x4E, 0x74, 0x09, 0x51, 0x43, 0x37, 0x30
    },    //uint32_t HeaderContext[MAX_HEADDER_LEN];
    29,
    1,
    1,
};


struct Aico_Remote_Receive_Buf aico_remote_receive_buf_type6 =
{
    {
        0xFF, 0xEF, 0x43, 0xB8, 0x04, 0xB4, 0x21, 0x42, 0xAB, 0xC1, 0xA2, 0x35, 0xF3, 0xC7, 0x54, 0x3A, 0xE4, 0xBA, 0xD3, 0x44,
        0x88, 0xC1, 0xA2, 0x35, 0xF3, 0xC7, 0x54, 0x3A, 0xE4, 0xBA, 0xD3, 0x44, 0x88, 0xC5, 0x56, 0x32, 0xEC, 0xC3, 0xA0, 0x2F,
        0xF4, 0xB4, 0x21, 0x42, 0xAB, 0xC1, 0xA2, 0x35, 0xF3, 0xC7, 0x54, 0x3A, 0xE4, 0xBA, 0xD3, 0x44, 0x88, 0xC1, 0xA2, 0x35,
        0xF3, 0xC7, 0x54, 0x3A, 0xE4, 0xBA, 0xD3, 0x44, 0x88, 0xC5, 0x56, 0x32, 0xEC, 0xC3, 0xA0, 0x2F, 0xF4, 0xB4, 0x21, 0x42,
        0xAB, 0xC1, 0xA2, 0x35, 0xF3, 0xC7, 0x54, 0x3A, 0xE4, 0xBA, 0xD3, 0x44, 0x88, 0xC1, 0xA2, 0x35, 0xF3, 0xC7, 0x54, 0x3A,
        0xE4, 0xBA, 0xD3, 0x44, 0x88, 0xC5, 0x56, 0x32, 0xEC, 0xC3, 0xA0, 0x45, 0x30
    },    //uint32_t HeaderContext[MAX_HEADDER_LEN];
    113,
    1,
    1,
};

struct Aico_Remote_Receive_Buf aico_remote_receive_buf_type7 =
{
    {
        0xFF, 0xE7, 0x43, 0xB8, 0x03, 0xEC, 0x63, 0xCB, 0x5B, 0x24, 0xC2, 0xFA, 0x5F, 0xC2, 0x8F, 0x5F, 0xB3, 0xF4, 0x57, 0x68,
        0x2B, 0x6A, 0x26, 0xB3, 0xF4, 0x26, 0xC0, 0xF4, 0x57, 0xBB, 0xFF, 0x29, 0x6B, 0x24, 0x68, 0x57, 0xBB, 0x8A, 0x24, 0xBB,
        0xFD, 0x5F, 0x19, 0x26, 0x6A, 0x2B, 0x68, 0x57, 0x11, 0x29, 0xC0, 0xFE, 0x57, 0xBB, 0xFD, 0x26, 0xC0, 0x89, 0x26, 0x19,
        0x5F, 0xC2, 0x89, 0x5F, 0x19, 0x5F, 0x19, 0x29, 0x6A, 0x24, 0x68, 0x57, 0x11, 0x26, 0x6A, 0x5F, 0xB3, 0xF5, 0x57, 0x68,
        0x2B, 0xC0, 0x8B, 0x58, 0xBB, 0x8A, 0x24, 0xBB, 0xFA, 0x51, 0x8F, 0xBE, 0xDA, 0x15, 0x95, 0x73, 0xBD, 0x4B, 0x2A, 0x86,
        0xB0
    },    //uint32_t HeaderContext[MAX_HEADDER_LEN];
    101,
    1,
    1,
};
#endif


uint8_t  aico_i2c_key[KEY_LEN] = {0x41, 0x43, 0x38, 0x30, 0x38, 0x30, 0x41, 0x43, 0x43, 0x41, 0x30, 0x38};


void aico_remote_command_decrypt(void)
{
    int i = 0;
    u8 first_bit = 0;

    first_bit = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[0];
    if (first_bit == 0xff)
    {
        for (i = 0; i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i++)
        {
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] =
                aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1] ^ aico_i2c_key[i % KEY_LEN];
        }
    }
    else
    {
        for (i = 0; i < first_bit; i++)
        {
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] =
                aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1] ^ aico_i2c_key[i % KEY_LEN];
        }

        for (i = first_bit; i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i++)
        {
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] =
                aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1];
        }
    }
}

#if  RTK_EVB_IR_TEST
DATA_RAM_FUNCTION void ir_send_cpy_type(uint8_t type)
{
    switch (type)
    {
    case 1: memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
                       aico_remote_receive_buf_type1.aico_remote_iic_reg_data_buf,
                       aico_remote_receive_buf_type1.aico_remote_iic_reg_data_len);
        aico_remote_receive_buf.aico_remote_iic_reg_data_len =
            aico_remote_receive_buf_type1.aico_remote_iic_reg_data_len;
        aico_remote_command_decrypt();
        break;
    case 2: memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
                       aico_remote_receive_buf_type2.aico_remote_iic_reg_data_buf,
                       aico_remote_receive_buf_type2.aico_remote_iic_reg_data_len);
        aico_remote_receive_buf.aico_remote_iic_reg_data_len =
            aico_remote_receive_buf_type2.aico_remote_iic_reg_data_len;
        aico_remote_command_decrypt();
        break;
    case 3:
        memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type3.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type3.aico_remote_iic_reg_data_len);
        aico_remote_receive_buf.aico_remote_iic_reg_data_len =
            aico_remote_receive_buf_type3.aico_remote_iic_reg_data_len;
        aico_remote_command_decrypt();
        break;
    case 4:
        memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type4.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type4.aico_remote_iic_reg_data_len);
        aico_remote_receive_buf.aico_remote_iic_reg_data_len =
            aico_remote_receive_buf_type4.aico_remote_iic_reg_data_len;
        aico_remote_command_decrypt();
        break;
    case 5:
        memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type5.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type5.aico_remote_iic_reg_data_len);
        aico_remote_receive_buf.aico_remote_iic_reg_data_len =
            aico_remote_receive_buf_type5.aico_remote_iic_reg_data_len;
        aico_remote_command_decrypt();

        break;
    case 6:
        memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type6.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type6.aico_remote_iic_reg_data_len);
        aico_remote_receive_buf.aico_remote_iic_reg_data_len =
            aico_remote_receive_buf_type6.aico_remote_iic_reg_data_len;
        aico_remote_command_decrypt();
        break;
    case 7:
        memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type7.aico_remote_iic_reg_data_buf,
               aico_remote_receive_buf_type7.aico_remote_iic_reg_data_len);
        aico_remote_receive_buf.aico_remote_iic_reg_data_len =
            aico_remote_receive_buf_type7.aico_remote_iic_reg_data_len;
        aico_remote_command_decrypt();
        break;
    default: break;
    }

}
#endif

void ir_send_driver_init(uint8_t freq)
{
    /* Enable IR clock */
    RCC_PeriphClockCmd(APBPeriph_IR, APBPeriph_IR_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_IR, APBPeriph_IR_CLOCK, ENABLE);

    Pad_Config(IR_TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pinmux_Config(IR_TX_PIN, IRDA_TX);

    /* Initialize IR */
    IR_InitTypeDef IR_InitStruct;
    IR_StructInit(&IR_InitStruct);
    IR_InitStruct.IR_Freq           = freq;//IR_LearnPacket.freq;
    IR_InitStruct.IR_DutyCycle      = 3;//IR_LearnPacket.duty_cycle;
    IR_InitStruct.IR_Mode           = IR_MODE_TX;
    IR_InitStruct.IR_TxInverse      = IR_TX_DATA_NORMAL;
    IR_InitStruct.IR_TxFIFOThrLevel = IR_TX_FIFO_THR_LEVEL;
    IR_Init(&IR_InitStruct);
    IR_Cmd(IR_MODE_TX, ENABLE);
    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = IR_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* Modify IR interrupt handle */
    //RamVectorTableUpdate(IR_VECTORn, ir_send_int_handler);
    RamVectorTableUpdate(IR_VECTORn, ir_send_int_handler);
}

bool ir_send_start(void)
{
    /* Start to send first bytes data of encoded data */
    //IR_SendBuf(gIrBuf->pBuf/*IR_LearnPacket.ir_buf*/, IR_TX_FIFO_SIZE, DISABLE);
	IR_SendBuf(gIrBuf->pBuf/*IR_LearnPacket.ir_buf*/, IR_TX_FIFO_SIZE, DISABLE);
    /* Record number which has been sent */
    IR_TX_Count = IR_TX_FIFO_SIZE;

    /* Enable IR threshold interrupt. when TX FIFO offset <= threshold value, trigger interrupt*/
    IR_INTConfig(IR_INT_TF_LEVEL, ENABLE);

    /*encode repeat code data*/

    return true;
}

void ir_send_exit(void)
{
    gIrBuf->irSendStage = IR_SEND_IDLE;
    IR_DeInit();
    IR_INTConfig(IR_INT_TF_LEVEL, DISABLE);
}



#if DLPS_EN   
#include "dlps_crtl.h"
    //void dlps_ctrl_set(uint8_t enable);
#endif
void ir_receive_data_exec(uint8_t *p_data, uint16_t len)
{
#if DLPS_EN 
    //dlps_ctrl_set(0);
	 egg_ir_exec_dlps_set(0);
#endif 
    if(gIrBuf!=NULL)
    {
        //ir_send_exit();
        DBG_DIRECT("ir_receive_data_exec gIrBuf!=NULL plt_free");
        IR_INTConfig(IR_INT_TF_LEVEL, DISABLE);
        IR_TX_Count = 0;
        plt_free(gIrBuf,RAM_TYPE_DATA_ON);
        gIrBuf =NULL;
    }    
    memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf, p_data, len);
    aico_remote_receive_buf.aico_remote_iic_reg_data_len = len;
    aico_remote_command_decrypt();
    printi("aico_remote_receive_buf len = %d", len);
    dprintt(aico_remote_receive_buf.aico_remote_iic_reg_data_buf, 20);
    aico_remote_command_send();
	ir_send_check_time_start();
		
}

void ir_learn_data_exec(uint8_t *p_data_learn, uint16_t learn_len)
{
#if DLPS_EN 
    //dlps_ctrl_set(0);
	  egg_ir_study_set(0);
#endif 
	  if(gIrBuf!=NULL)
    {
        //ir_send_exit();
        DBG_DIRECT("ir_receive_data_exec gIrBuf!=NULL plt_free");
        IR_INTConfig(IR_INT_TF_LEVEL, DISABLE);
        IR_TX_Count = 0;
        plt_free(gIrBuf,RAM_TYPE_DATA_ON);
        gIrBuf =NULL;
    }    
    memcpy(aico_remote_receive_buf.aico_remote_iic_reg_data_buf, p_data_learn, learn_len);
    aico_remote_receive_buf.aico_remote_iic_reg_data_len = learn_len;
    aico_remote_receive_buf.aico_remote_iic_reg_data_len++;
    printi("ir_learn_data_exec len = %d", learn_len);
    dprinti(&aico_remote_receive_buf.aico_remote_iic_reg_data_buf[learn_len - 20], 20);
    aico_remote_command_send();
		ir_send_check_time_start();
}



u8 aico_remote_command_analysis(u8 *cmd, u16 cmdlen)
{
    if (cmd[0] == AICO_REMOTE_COMMAND_HEAD_TYPE_1)
    {
        //HEAD
        aico_remote_command.aico_remote_command_head.aico_remote_command_type = cmd[0];
        //GGGG
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_value =
            (cmd[1] & 0xf0) >> 4;
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g = cmd[1] >> 7;
        //NNNN
        aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value = cmd[1] & 0x0f;
        //YYYY YYYY
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value = cmd[2];
        //WWWW WWWW
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = cmd[3];
        //TTTT TTTT *16
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_len = 16;
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr = 4;
        //00MM MMMM
        aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value = cmd[20];
        //DDDD DDDD*n
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len = cmdlen - 21;
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr = 21;
        return TRUE;
    }
    else if (cmd[0] == AICO_REMOTE_COMMAND_HEAD_TYPE_2)
    {
        //HEAD
        aico_remote_command.aico_remote_command_head.aico_remote_command_type = cmd[0];
        //GGGG
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_value =
            (cmd[1] & 0xf0) >> 4;
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g = cmd[1] >> 7;
        //NNNN
        aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value = cmd[1] & 0x0f;
        //YYYY YYYY*2
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value =
            (cmd[2] << 8) + cmd[3];
        //WWWW WWWW
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = cmd[4];
        //TTTT TTTT*n
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr = 5;
        //DDDD DDDD*n

        return TRUE;
    }
    else if (cmd[0] == AICO_REMOTE_COMMAND_HEAD_TYPE_3)
    {
        //HEAD
        aico_remote_command.aico_remote_command_head.aico_remote_command_type = cmd[0];
        //GGGG
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_value =
            (cmd[1] & 0xf0) >> 4;
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g = cmd[1] >> 7;
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_w =
            (cmd[1] >> 6) & 0x01;
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_v =
            (cmd[1] >> 5) & 0x01;
        //NNNN
        aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value = cmd[1] & 0x0f;
        //YYYY YYYY
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value = cmd[2];
        //WWWW WWWW
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = cmd[3];
        //TTTT TTTT *16
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_len = 16;
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr = 4;
        //00MM MMMM
        aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value = cmd[20];
        //DDDD DDDD*n
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len = cmdlen - 21;
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr = 21;

        return TRUE;
    }
    else if (cmd[0] == AICO_REMOTE_COMMAND_HEAD_TYPE_4)
    {
        //HEAD
        aico_remote_command.aico_remote_command_head.aico_remote_command_type = cmd[0];
        //GGGG
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_value =
            (cmd[1] & 0xf0) >> 4;
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g = cmd[1] >> 7;
        //NNNN
        aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value = cmd[1] & 0x0f;
        //YYYY YYYY*2
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value =
            (cmd[2] << 8) + cmd[3];
        //WWWW WWWW
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = cmd[4];
        //TTTT TTTT*n
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr = 5;
        //DDDD DDDD*n


        return TRUE;
    }
    else if (cmd[0] == AICO_REMOTE_COMMAND_HEAD_TYPE_5)
    {
        //HEAD
        aico_remote_command.aico_remote_command_head.aico_remote_command_type = cmd[0];
        //GGGG val:0001
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_value =
            (cmd[1] & 0xf0) >> 4;
        //NNNN
        aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value = cmd[1] & 0x0f;
        //YYYY YYYY val:0010 1011
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value = cmd[2];
        //WWWW WWWW val:0110 1000
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = cmd[3];
        //TTTT TTTT *16 val:0000000000C8030C00C8039435E83881
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_len = 16;
        aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr = 4;
        //DDDD DDDD*n
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len = cmdlen - 20;
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr = 20;

        return TRUE;
    }
    else if (cmd[0] == AICO_REMOTE_COMMAND_HEAD_TYPE_6)
    {
        //HEAD
        aico_remote_command.aico_remote_command_head.aico_remote_command_type = cmd[0];
        //GGGG
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_value = cmd[1];
        //NNNN
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carriet_enable = cmd[2];
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = cmd[3];
        //DDDD DDDD*n
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len = cmdlen - 4;
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr = 4;

        return TRUE;
    }
    else if (cmd[0] == AICO_REMOTE_COMMAND_HEAD_TYPE_7)
    {
        //HEAD
        aico_remote_command.aico_remote_command_head.aico_remote_command_type = cmd[0];
        //GGGG
        aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_value = cmd[1];
        //NNNN
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carriet_enable = cmd[2];
        aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = cmd[3];
        //DDDD DDDD*n
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len = cmdlen - 4;
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr = 4;

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}





void aico_remote_ldr_command_type1_analysis(void)
{

    /*
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_high_time =9000;
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_low_time = 4500;
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time = 560;
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time = 560;
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time = 560;
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time = 1690;
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time = 0;
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time = 0;

    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.pilot_code_enable = 3;//aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 6;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format =0; //(aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x30) >> 4;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.synchronous_bit_ebalbe =0;// (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x08) >> 3;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.tail_code_enable =0; //(aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x04) >> 2;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode =1;// aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x03;
    */
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_high_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 0] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 1];
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_low_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 2] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 3];
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 4] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 5];
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 6] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 7];
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 8] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 9];
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 10] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 11];
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 12] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 13];
    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 14] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 15];

    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.pilot_code_enable =
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 6;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x30) >>
        4;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.synchronous_bit_ebalbe =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x08) >>
        3;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.tail_code_enable =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x04) >>
        2;
    aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode =
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x03;


}

void aico_remote_ldr_command_type1_transfer(void)
{
    u8 reframe = 0;

    u16 data_frame_len = 0;
    uint8_t BaseTime1 = 0;
    aico_remote_ldr_command_type1_analysis();
	  

    //初始化载波
    /*
    aico_bsp_timer_carrier_pwm_stop();
    aico_bsp_timer_carrier_pwm_init(&period_t,(4000/(aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle+1)));
    aico_bsp_timer_carrier_pwm_duty_set(period_t/3);
    aico_bsp_timer_carrier_pwm_start();
    */
    float freq = 0;
    aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = 40000 /
            (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle + 1);
    freq = aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    freq = freq / 10;
    BaseTime1 = 100000 /
                aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    if (aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value % 8 == 0)
    {
        data_frame_len = aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value
                         / 8;
    }
    else
    {
        data_frame_len = aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value
                         / 8 + 1;
    }
    if ((data_frame_len *
         (aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value + 1)) >
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len)
    {
        data_frame_len = 0;
    }
    gIrBuf->pBufLen = 0;
    for (reframe = 0;
         reframe <= aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value;
         reframe++)
    {

        //引导码
        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.pilot_code_enable ==
            3) //有引导码
        {

            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_high_time, BaseTime1);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_low_time, BaseTime1);
            /*
            AICO_LDR_DATA_PIN(1);
            delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_high_time);
            AICO_LDR_DATA_PIN(0);
            delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_low_time);
            */

        }
        else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.pilot_code_enable ==
                 2) //仅引导码高电平
        {
            /*AICO_LDR_DATA_PIN(1);
            delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_high_time);
            */
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                aico_remote_command_data_type1.aico_remote_command_datacycle_type1.pilot_code_high_time, BaseTime1);
        }
        //数据
        for (uint8_t i = 0;
             i < aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value; i++)
        {
            if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format ==
                0) //正常数据
            {
                if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 0)
                {
                    if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                             + data_frame_len * reframe + (i / 8)] & (0x80 >> (i % 8)))
                    {

                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time, BaseTime1);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time, BaseTime1);
                        /*
                        AICO_LDR_DATA_PIN(1);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time);
                        AICO_LDR_DATA_PIN(0);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time);
                        */
                    }
                    else
                    {
                        /*
                        AICO_LDR_DATA_PIN(1);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time);
                        AICO_LDR_DATA_PIN(0);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time);
                        */
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time, BaseTime1);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time, BaseTime1);

                    }
                }
                else if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 1)
                {
                    if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                             + data_frame_len * reframe + (i / 8)] & (0x01 << (i % 8)))
                    {
                        /*
                        AICO_LDR_DATA_PIN(1);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time);
                        AICO_LDR_DATA_PIN(0);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time);
                        */
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time, BaseTime1);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time, BaseTime1);
                    }
                    else
                    {
                        /*
                        AICO_LDR_DATA_PIN(1);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time);
                        AICO_LDR_DATA_PIN(0);
                        delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time);
                        */
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time, BaseTime1);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time, BaseTime1);
                    }
                }
            }
            else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format ==
                     1)  //双相位编码
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                         + data_frame_len * reframe + (i / 8)] & (0x80 >> (i % 8)))
                {
                    /*
                    AICO_LDR_DATA_PIN(0);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time);
                    AICO_LDR_DATA_PIN(1);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_low_time, BaseTime1);
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time, BaseTime1);

                }
                else
                {
                    /*
                    AICO_LDR_DATA_PIN(1);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time);
                    AICO_LDR_DATA_PIN(0);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_high_time, BaseTime1);
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time, BaseTime1);
                }

            }
            else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format ==
                     2)  //高为1低为0
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                         + data_frame_len * reframe + (i / 8)] & (0x80 >> (i % 8)))
                {
                    /*
                    AICO_LDR_DATA_PIN(1);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time, BaseTime1);
                }
                else
                {
                    /*
                    AICO_LDR_DATA_PIN(0);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data0_low_time, BaseTime1);
                }
            }
        }
        //尾码
        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.tail_code_enable == 1)
        {
            /*
            AICO_LDR_DATA_PIN(1);
            delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time);
            AICO_LDR_DATA_PIN(0);
            delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time);
            */
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time, BaseTime1);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time, BaseTime1);
        }
        //同步位
        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.synchronous_bit_ebalbe == 1)
        {
            /*
            AICO_LDR_DATA_PIN(1);
            delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time);
            */
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                aico_remote_command_data_type1.aico_remote_command_datacycle_type1.data1_high_time, BaseTime1);
        }
        //间隙
        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.tail_code_enable == 1)
        {
            if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 1)
            {
                /*
                AICO_LDR_DATA_PIN(0);
                delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time);
                */
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time, BaseTime1);
            }
            else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 3)
            {
                /*
                AICO_LDR_DATA_PIN(0);
                delay_us(65535);
                delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time);
                */
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 65535, BaseTime1);
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                    aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time, BaseTime1);
            }
        }
        else
        {
            if (reframe == 0) //第一帧间隙0
            {
                if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 1)
                {
                    /*
                    AICO_LDR_DATA_PIN(0);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time, BaseTime1);
                }
                else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 3)
                {
                    /*
                    AICO_LDR_DATA_PIN(0);
                    delay_us(65535);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 65535, BaseTime1);
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance0_time, BaseTime1);
                }
            }
            else //第二帧以后间隙1
            {
                if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 1)
                {
                    /*
                    AICO_LDR_DATA_PIN(0);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time, BaseTime1);
                }
                else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 3)
                {
                    /*
                    AICO_LDR_DATA_PIN(0);
                    delay_us(65535);
                    delay_us(aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time);
                    */
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 65535, BaseTime1);
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type1.aico_remote_command_datacycle_type1.clearance1_time, BaseTime1);

                }
            }
        }
    }
    //gIrBuf.pBufLen=gIrBuf.pBufLen-1;
    ir_send_driver_init(freq);
    ir_send_start();
    //aico_bsp_timer_carrier_pwm_stop();
}
//uint16_t aico_remote_ldr_command_type1_signle_num(void)
//{
//    u8 reframe = 0;

//    u16 data_frame_len = 0;
//    uint16_t signle_num =0;
//    aico_remote_ldr_command_type1_analysis();
//    if (aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value % 8 == 0)
//    {
//        data_frame_len = aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value / 8;
//                        
//    }
//    else
//    {
//        data_frame_len = aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value/ 8 + 1;
//                         
//    }
//    if ((data_frame_len *(aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value + 1)) >
//        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len)
//    {
//        data_frame_len = 0;
//    }
//  
//    for (reframe = 0;
//         reframe <= aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value;
//         reframe++)
//    {

//        //引导码
//        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.pilot_code_enable ==
//            3) //有引导码
//        {

//            signle_num++ ;
//            signle_num++ ;
//           
//        }
//        else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.pilot_code_enable ==
//                 2) //仅引导码高电平
//        {
//            
//            signle_num++;
//        }
//        //数据
//        for (uint8_t i = 0;
//             i < aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value; i++)
//        {
//            if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format ==
//                0) //正常数据
//            {
//                if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 0)
//                {
//                    signle_num++;
//                    signle_num++;
//                }
//                else if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 1)
//                {
//                   signle_num++;
//                   signle_num++;
//                }
//            }
//            else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format ==
//                     1)  //双相位编码
//            {
//               signle_num++;
//               signle_num++;

//            }
//            else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.data_encoding_format ==
//                     2)  //高为1低为0
//            {
//                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
//                                                                         + data_frame_len * reframe + (i / 8)] & (0x80 >> (i % 8)))
//                {
//                   signle_num++;
//                   
//                }
//                else
//                {
//                   signle_num++;
//                }
//            }
//        }
//        //尾码
//        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.tail_code_enable == 1)
//        {
//          
//            signle_num++;
//            signle_num++;
//        }
//       
//        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.synchronous_bit_ebalbe == 1)
//        {
//           
//            signle_num++;
//        }
//        //间隙
//        if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.tail_code_enable == 1)
//        {
//            if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 1)
//            {
//               signle_num++;
//            }
//            else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 3)
//            {
//                
//               signle_num++;
//               signle_num++;
//            }
//        }
//        else
//        {
//            if (reframe == 0) //第一帧间隙0
//            {
//                if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 1)
//                {
//                   signle_num++;
//                }
//                else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 3)
//                {
//                   signle_num++;
//                   signle_num++;
//                }
//            }
//            else //第二帧以后间隙1
//            {
//                if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 1)
//                {
//                    signle_num++;
//                                                                      
//                }
//                else if (aico_remote_command_data_type1.aico_remote_command_dataformat_type1.code_sending_mode == 3)
//                {
//                    signle_num++;
//                    signle_num++;

//                }
//            }
//        }
//    }
//   return  signle_num;
//}

/*-----------------------CMD_TYPE2---------------------------*/
//YYYY
struct Aico_Remote_Command_Dataformat_Type2
{
    u8 pilot_code_enable;      // 1bit 1:有引导码 0:无引导码
    u8 data_units_num;         // 2bit 数据单元位数 00:1 01:2 10:3 11:4
    u8 data_units_val_num;     // 3bit 数据单元值个数 010-111
    u8 segmentation_code_num;  // 3bit 分割码个数
    u8 flip_code_enable;       // 1bit 0:无翻转码 1:有翻转码
    u8 tail_code_enable;       // 1bit 0:无尾码 0:有尾码
    u8 repeat_mode;            // 1bit 0:普通模式 1:特殊模式
    u8 repeat_frame_pilot_code;// 2bit 00:重复帧无引导码  01:重复帧有引导码，使用第一帧的引导码 10:重复帧有引导码，使用重复帧引导码
    u8 repeat_frame_data_code; // 1bit 0:重复帧无数据码 1:重复帧有数据码
    u8 repeat_frame_tail_code; // 1bit 0:重复帧无尾码 1:重复帧有尾码
};

//TTTT
struct Aico_Remote_Command_Datacycle_Type2
{
    u16 pilot_code_addr;               //引导码起始地址
    u16 pilot_code_len;                //引导码长度

    u8 data_code_len;                  //数据码长度
    u8 data_code_unit_val[20][2];      //数据码值 0:码值位置 1:码值长度

    u8 segmentation_code_data[20];     //分割码插入位置 max20
    u8 segmentation_code_addr;         //分割码位置
    u8 segmentation_code_len;          //分割码长度

    u8 flip_code_num;                  //翻转码数量
    u8 flip_code_val[20][5];           //翻转码 0:位置 1:可翻转数据个数 2：可翻转长度-位数 3：翻转值 4：电平是否和数据码一致
    u16 flip_code_ttl[20][2][2];       //翻转码电平时间 翻转码序号 翻转码码值（0,1） 翻转码码值位置和长度

    u8 tail_code_addr;                 //尾码数据地址
    u8 tail_code_len;                  //尾码长度

    u8 repeat_frame_pilot_code_addr;   //重复帧引导码起始地址
    u8 repeat_frame_pilot_code_len;    //重复帧引导码长度

    u8 repeat_frame_data_code_addr;    //重复帧数据码起始位置
    u8 repeat_frame_data_code_len;     //重复帧数据码长度

    u8 repeat_frame_tail_code_addr;    //尾码数据起始位置
    u8 repeat_frame_tail_code_len;     //尾码数据长度

};

struct Aico_Remote_Command_Data_Type2
{
    struct Aico_Remote_Command_Datacycle_Type2 aico_remote_command_datacycle_type2;
    struct Aico_Remote_Command_Dataformat_Type2 aico_remote_command_dataformat_type2;
};

struct Aico_Remote_Command_Data_Type2 aico_remote_command_data_type2;

void aico_remote_ldr_command_type2_analysis(void)
{
    int i = 0;
    u16 segmentation_code_start_addr = 0;
    u16 flip_code_start_addr = 0;
    u16 flip_code_len = 0;
    u16 repeat_frame_data_code_start_addr = 0;
    u16 repeat_frame_data_code_start_len = 0;

    //INIT
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_addr = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_len = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_addr = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_len = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_addr = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_len = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_addr = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_len = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_addr = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_len = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_addr = 0;
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_len = 0;

    for (i = 0; i < 20; i++)
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[i][0] = 0;
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[i][1] = 0;

        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_data[i] = 0;

        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[i][0] = 0;
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[i][1] = 0;
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[i][2] = 0;
    }

    //YYYY
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.pilot_code_enable =
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 15;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num = ((
                aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 13) &
            0x03) + 1;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_val_num =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 10) &
        0x07;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.segmentation_code_num =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 7) &
        0x07;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.flip_code_enable =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 6) &
        0x01;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.tail_code_enable =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 5) &
        0x01;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_mode =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 4) &
        0x01;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_pilot_code =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 2) &
        0x03;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_data_code =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 1) &
        0x01;
    aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_tail_code =
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x01;

    //TTTT
    //引导码
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_addr = 5;
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.pilot_code_enable)
    {
        for (i = 5; i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i += 2)
        {
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len += 2;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                break;
            }
        }
    }
    else
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len = 0;
    }

    //数据码
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_len =
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_addr
                                                             + aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len];

    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[0][0] =
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_addr +
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len + 1;
    for (i = 0;
         i < aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_val_num; i++)
    {
        int j = 0;

        for (j = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[i][0];
             ;
             j += 2)
        {
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[i][1] += 2;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[j] & 0x80) == 0)
            {
                break;
            }
        }
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[i + 1][0] =
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[i][0] +
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[i][1];
    }
    //分割码
    segmentation_code_start_addr =
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_val_num
                - 1][0] + aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_val_num
                        - 1][1];
    for (i = 0;
         i < aico_remote_command_data_type2.aico_remote_command_dataformat_type2.segmentation_code_num; i++)
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_data[i] =
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[segmentation_code_start_addr + i];
    }

    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_addr =
        segmentation_code_start_addr +
        aico_remote_command_data_type2.aico_remote_command_dataformat_type2.segmentation_code_num;
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.segmentation_code_num == 0)
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_len = 0;
    }
    else
    {
        for (i = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_addr;
             i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i += 2)
        {
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_len += 2;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                break;
            }
        }
    }

    //翻转码
    flip_code_start_addr =
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_addr +
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_len;
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.flip_code_enable == 0)
    {
        flip_code_len = 0;
    }
    else
    {
        for (i = flip_code_start_addr; i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i += 2)
        {
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num][0]
                = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x7f;
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num][1]
                = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1] >> 4;
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num][2]
                = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1] & 0x03;
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num][4]
                = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1] & 0x08;
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num ++;
            flip_code_len += 2; //翻转码定位

            if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num > 1)
            {
                //aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num-1][0] = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num-1][0];
            }


            //存在特殊翻转码
            if (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                    - 1][4] != 0)
            {
                int j = 0;

                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                        - 1][0][0] = flip_code_start_addr + flip_code_len;
                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                        - 1][0][1] = 0; //初始化

                for (j = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                         - 1][0][0];; j += 2)
                {
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                            - 1][0][1] += 2;
                    if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[j] & 0x80) == 0)
                    {
                        break;
                    }
                }
                flip_code_len +=
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                            - 1][0][1];

                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                        - 1][1][0] = flip_code_start_addr + flip_code_len;
                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                        - 1][1][1] = 0; //初始化

                for (j = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                         - 1][1][0];; j += 2)
                {
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                            - 1][1][1] += 2;
                    if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[j] & 0x80) == 0)
                    {
                        break;
                    }
                }
                flip_code_len +=
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num
                            - 1][1][1];

            }

            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                break;
            }
        }
        //flip_code_len =  2*aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num;
    }

    //尾码
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_addr =
        flip_code_start_addr + flip_code_len;

    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.tail_code_enable == 0)
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_len = 0;
    }
    else
    {
        for (i = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_addr;
             i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i += 2)
        {
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_len += 2;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                break;
            }
        }
    }

    //重复帧引导码
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_addr =
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_addr +
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_len;
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_pilot_code ==
        2)
    {
        for (i = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_addr;
             i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i += 2)
        {
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_len += 2;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                break;
            }
        }
    }
    else
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_len = 0;
    }

    //重复帧数据码
    repeat_frame_data_code_start_addr =
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_addr +
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_len;
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_data_code == 1
        && aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_mode == 1)
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_addr =
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[repeat_frame_data_code_start_addr];
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_len =
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[repeat_frame_data_code_start_addr + 1];
        repeat_frame_data_code_start_len = 2;
    }
    else
    {
        repeat_frame_data_code_start_len = 0;
    }

    //重复帧尾码
    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_addr =
        repeat_frame_data_code_start_addr + repeat_frame_data_code_start_len;
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_tail_code == 1
        && aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_mode == 1)
    {
        for (i = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_addr;
             i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i += 2)
        {
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_len += 2;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                break;
            }
        }
    }
    else
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_len = 0;
    }
    //数据码
    aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr =
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_addr +
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_len;
    aico_remote_command.aico_remote_command_data.aico_remote_command_data_len =
        aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1 -
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr;

    //修正重复帧数据码位置
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_data_code == 1
        && aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_mode == 1)
    {
        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_addr =
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[repeat_frame_data_code_start_addr] / 8 +
            aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr;
    }
}

void aico_remote_ldr_ttl_send(u16 addr, u16 len, uint16_t BaseTime)
{
    int i = 0;
    //uint8_t flag =0;
    for (i = 0; i < len;)
    {

        if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr] & 0x40)
        {
            //AICO_LDR_DATA_PIN(1);
            //flag =1;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 2 + addr] & 0x40) && (i + 2 < len))
            {
                u32 t1 = 0, t2 = 0;
                t1 = ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr] & 0x3f) << 8) +
                     aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr + 1];
                t2 = ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 2 + addr] & 0x3f) << 8) +
                     aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 2 + addr + 1];
                //delay_us((t1<<14)+t2);

                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | ((t1 << 14) + t2), BaseTime);
                i += 4;
            }
            else
            {
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | (((
                                                                                       aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr] & 0x3f) << 8) +
                                                                                  aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr + 1]), BaseTime);
                //delay_us(((aico_remote_iic_buf.aico_remote_iic_reg_data_buf[i+addr]&0x3f)<<8)+aico_remote_iic_buf.aico_remote_iic_reg_data_buf[i+addr+1]);
                i += 2;
            }

        }
        else
        {
            //AICO_LDR_DATA_PIN(0);

            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 2 + addr] & 0x40) == 0 &&
                (i + 2 < len))
            {
                u32 t1 = 0, t2 = 0;
                t1 = ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr] & 0x3f) << 8) +
                     aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr + 1];
                t2 = ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 2 + addr] & 0x3f) << 8) +
                     aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 2 + addr + 1];
                //delay_us((t1<<14)+t2);
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | ((t1 << 14) + t2), BaseTime);
                i += 4;
            }
            else
            {
                //delay_us(((aico_remote_iic_buf.aico_remote_iic_reg_data_buf[i+addr]&0x3f)<<8)+aico_remote_iic_buf.aico_remote_iic_reg_data_buf[i+addr+1]);
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | (((
                                                                                      aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr] & 0x3f) << 8) +
                                                                                 aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr + 1]), BaseTime);
                i += 2;
            }

        }
        /*
        if(flag==1)
         gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH|(((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i+addr]&0x3f)<<8)+aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i+addr+1]), BaseTime);
        else
         gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW|(((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i+addr]&0x3f)<<8)+aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i+addr+1]), BaseTime);
        */
        //delay_us(((aico_remote_iic_buf.aico_remote_iic_reg_data_buf[i+addr]&0x3f)<<8)+aico_remote_iic_buf.aico_remote_iic_reg_data_buf[i+addr+1]);

    }
}

void aico_remote_ldr_command_type2_transfer(void)
{
    int i = 0;
    u8 send_list[128][2];//数据队列
    u8 send_list_len_1 = 0;
    u8 send_list_len_2 = 0;
    u8 reframe = 0;


    aico_remote_ldr_command_type2_analysis();


    //数据预处理
    gIrBuf->pBufLen = 0;
    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num > 1)
    {
        for (i = 0; i < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_len;
             i += aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num)
        {
            int j = 0;
            u8 val = 0;

            for (j = 0; j < aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num;
                 j++)
            {
                val += ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                              + i / 8] & (0x80 >> ((i + j) % 8))) >> (7 - (i + j) % 8) <<
                        (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num - 1 - j));
            }
            send_list[send_list_len_1][0] = val;
            send_list_len_1++;
        }

        //存在翻转码
        //翻转码 0:位置 1:可翻转数据个数 2：可翻转长度-位数
        if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.flip_code_enable)
        {
            int j = 0;
            u8 send_list_flip[128];
            u8 send_list_len_flip = 0;

            //重置数据
            for (i = 0; i < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_len;
                 i++)
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                         + i / 8] & (0x80 >> (i % 8)))
                {
                    send_list_flip[i] = 1;
                }
                else
                {
                    send_list_flip[i] = 0;
                }
                send_list_len_flip++;
            }

            for (j = 0; j < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num;
                 j++)
            {
                int k = 0;
                u8 val = 0;
                if (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3] >=
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][1])
                {
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3] = 0;
                }
                val = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3];
                for (k = 0;
                     k < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][2]; k++)
                {
                    send_list_flip[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][0]
                                   + aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][2] - 1 - k] =
                                       val % 2;
                    val = val / 2;
                }

                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3]++;
            }

            send_list_len_1 = 0;
            for (i = 0; i < send_list_len_flip;
                 i += aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num)
            {
                int j = 0;
                u8 val_f = 0;
                for (j = 0; j < aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num;
                     j++)
                {
                    val_f += (send_list_flip[i + j] <<
                              (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num - 1 - j));
                }
                send_list[send_list_len_1][0] = val_f;
                send_list_len_1++;
            }
        }
    }
    else
    {
        for (i = 0; i < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_len;
             i += aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num)
        {
            if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 0)
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                         + i / 8] & (0x80 >> (i % 8)))
                {
                    send_list[i][0] = 1;
                }
                else
                {
                    send_list[i][0] = 0;
                }
                send_list_len_1++;
            }
            else
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                         + i / 8] & (0x01 << (i % 8)))
                {
                    send_list[i][0] = 1;
                }
                else
                {
                    send_list[i][0] = 0;
                }
                send_list_len_1++;
            }
        }
        send_list_len_1 = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_len;

        //存在翻转码
        //翻转码 0:位置 1:可翻转数据个数 2：可翻转长度-位数
        if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.flip_code_enable)
        {
            int j = 0;
            for (j = 0; j < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num;
                 j++)
            {
                int k = 0;
                u8 val = 0;
                if (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3] >=
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][1])
                {
                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3] = 0;
                }
                val = aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3];
                for (k = 0;
                     k < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][2]; k++)
                {
                    send_list[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][0] +
                              aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][2] - 1 - k][0] =
                                  val % 2;
                    val = val / 2;
                }

                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[j][3]++;
            }
        }
    }

    if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num > 1)
    {
        for (i = 0;
             i < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_len;
             i += aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num)
        {
            int j = 0;
            u8 val = 0;

            for (j = 0; j < aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num;
                 j++)
            {
                val += ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                              + aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_addr + i
                                                                              / 8] & (0x80 >> ((i + j) % 8))) >> (7 - (i + j) % 8) <<
                        (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num - 1 - j));
            }
            send_list[send_list_len_2][1] = val;
            send_list_len_2++;
        }
    }
    else
    {
        for (i = 0;
             i < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_len;
             i += aico_remote_command_data_type2.aico_remote_command_dataformat_type2.data_units_num)
        {
            if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 0)
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_addr
                                                                         + i / 8] & (0x80 >> (i % 8)))
                {
                    send_list[i][1] = 1;
                }
                else
                {
                    send_list[i][1] = 0;
                }
                send_list_len_2++;
            }
            else
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_addr
                                                                         + i / 8] & (0x01 << (i % 8)))
                {
                    send_list[i][1] = 1;
                }
                else
                {
                    send_list[i][1] = 0;
                }
                send_list_len_2++;
            }
        }
        send_list_len_2 =
            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_data_code_len;
    }

    //初始化载波
    //aico_bsp_timer_carrier_pwm_stop();
    //aico_bsp_timer_carrier_pwm_init(&period_t,(4000/(aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle+1)));
    //aico_bsp_timer_carrier_pwm_duty_set(period_t/3);
    //aico_bsp_timer_carrier_pwm_start();
    float freq1 = 0;
    aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = 40000 /
            (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle + 1);
    freq1 = aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    freq1 = freq1 / 10;
    uint16_t BaseTime2 = 100000 /
                         aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    for (reframe = 0;
         reframe <= aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value;
         reframe++)
    {
        if (reframe == 0) //第一帧
        {
            //引导码
            aico_remote_ldr_ttl_send(
                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_addr,
                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len, BaseTime2);

            //数据
            for (i = 0; i < send_list_len_1; i++)
            {
                int j = 0, k = 0, send_flip_special = 0, send_flip_num = 0;
                for (j = 0;
                     j < aico_remote_command_data_type2.aico_remote_command_dataformat_type2.segmentation_code_num; j++)
                {
                    if (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_data[j] ==
                        i)
                    {
                        //分割码
                        aico_remote_ldr_ttl_send(
                            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_addr,
                            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_len,
                            BaseTime2);
                    }
                }

                //特殊翻转码
                //aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num-1][4] != 0
                for (k = 0; k < aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_num;
                     k++)
                {
                    if (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[k][4] != 0)
                    {
                        if (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[k][0] <= i &&
                            (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[k][0] +
                             aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_val[k][2] - 1) >= i)
                        {
                            send_flip_num = k;
                            send_flip_special = 1;
                        }
                    }
                }

                if (send_flip_special == 0)
                {
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[send_list[i][0]][0],
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[send_list[i][0]][1],
                        BaseTime2);
                }
                else
                {
                    //特殊翻转码
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[send_flip_num][send_list[i][0]][0],
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.flip_code_ttl[send_flip_num][send_list[i][0]][1],
                        BaseTime2);
                }
            }
            //尾码
            aico_remote_ldr_ttl_send(
                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_addr,
                aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_len, BaseTime2);
        }
        else
        {
            if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_mode ==
                0) //普通重复模式
            {
                //引导码
                if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_pilot_code ==
                    1)
                {
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_addr,
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len, BaseTime2);
                }
                else if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_pilot_code
                         == 2)
                {
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_addr,
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_len,
                        BaseTime2);
                }

                //数据码
                if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_data_code)
                {
                    for (i = 0; i < send_list_len_1; i++)
                    {
                        int j = 0;
                        for (j = 0;
                             j < aico_remote_command_data_type2.aico_remote_command_dataformat_type2.segmentation_code_num; j++)
                        {
                            if (aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_data[j] ==
                                i)
                            {
                                aico_remote_ldr_ttl_send(
                                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_addr,
                                    aico_remote_command_data_type2.aico_remote_command_datacycle_type2.segmentation_code_len,
                                    BaseTime2);
                            }
                        }
                        aico_remote_ldr_ttl_send(
                            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[send_list[i][0]][0],
                            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[send_list[i][0]][1],
                            BaseTime2);
                    }
                }

                //尾码
                if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_tail_code)
                {
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_addr,
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.tail_code_len, BaseTime2);
                }
            }
            else //特殊重复模式
            {
                //引导码
                if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_pilot_code ==
                    1)
                {
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_addr,
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.pilot_code_len, BaseTime2);
                }
                else if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_pilot_code
                         == 2)
                {
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_addr,
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_pilot_code_len,
                        BaseTime2);
                }

                //数据码
                if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_data_code)
                {
                    for (i = 0; i < send_list_len_2; i++)
                    {
                        aico_remote_ldr_ttl_send(
                            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[send_list[i][1]][0],
                            aico_remote_command_data_type2.aico_remote_command_datacycle_type2.data_code_unit_val[send_list[i][1]][1],
                            BaseTime2);
                    }
                }

                //尾码
                if (aico_remote_command_data_type2.aico_remote_command_dataformat_type2.repeat_frame_tail_code)
                {
                    aico_remote_ldr_ttl_send(
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_addr,
                        aico_remote_command_data_type2.aico_remote_command_datacycle_type2.repeat_frame_tail_code_len,
                        BaseTime2);
                }
            }
        }
    }
    printi("type2_transfer ,freq=%f", freq1);
    dprintt(gIrBuf->pBuf, 20);
    ir_send_driver_init(freq1);
    ir_send_start();
    //aico_bsp_timer_carrier_pwm_stop();
}

/*-----------------------CMD_TYPE3---------------------------*/
//TTTT
struct Aico_Remote_Command_Datacycle_Type3
{
    u16 pilot_code_high_time;  //引导码高电平时间
    u16 pilot_code_low_time;   //引导码低电平时间
    u16 data0_high_time;       //数据0高电平时间
    u16 data0_low_time;        //数据0低电平时间
    u16 data1_high_time;       //数据1高电平时间
    u16 data1_low_time;        //数据1低电平时间
    u16 tail_code_low_time;    //尾码低电平时间
    u16 tail_code_high_time;   //尾码高电平时间
};
//YYYY
struct Aico_Remote_Command_Dataformat_Type3
{
    u8 pilot_code_enable;             // 1:有引导码 0:无引导码
    u8 tail_code_time_format;         // 0:尾码与数据位高电平宽度一致 1:尾码与同步位高电平宽度一致（B0=1时有效）
    u8 data_encoding_format;          // 0:正常数据 1:双向位编码
    u8 multi_frame_gap_enable;        // 0:多帧之间有间隙 1:多帧之间无间隙（NNNN <>0&&B5=0时间隙大于65.5MS小于131MS）
    u8 synchronous_bit_enalbe;        // 1:有同步位 0:无同步位
    u8 tail_code_enable;              // 0:无尾码 1:有尾码
    u8 synchronous_bit_width_format;  // 0:同步位与数据位高电平宽度一致 1:同步位与数据位高电平宽度不一致 （T13 T14=高电平宽度 ）
    u8 tail_code_width_format;        // 0:尾码与数据位高电平宽度一致 01:尾码与数据位高电平宽度不一致
};

struct Aico_Remote_Command_Data_Type3
{
    struct Aico_Remote_Command_Datacycle_Type3 aico_remote_command_datacycle_type3;
    struct Aico_Remote_Command_Dataformat_Type3 aico_remote_command_dataformat_type3;
};

struct Aico_Remote_Command_Data_Type3 aico_remote_command_data_type3;

void aico_remote_ldr_command_type3_analysis(void)
{
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.pilot_code_high_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 0] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 1];
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.pilot_code_low_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 2] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 3];
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 4] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 5];
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_low_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 6] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 7];
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_high_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 8] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 9];
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_low_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 10] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 11];
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_low_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 12] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 13];
    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_high_time =
        (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                              + 14] << 8) +
        aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_datacycle.aico_remote_command_datacycle_start_addr
                                                             + 15];

    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.pilot_code_enable =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x80) >>
        7;
    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.tail_code_time_format =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x40) >>
        6;
    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.data_encoding_format =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x20) >>
        5;
    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.multi_frame_gap_enable =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x10) >>
        4;
    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.synchronous_bit_enalbe =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x08) >>
        3;
    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.tail_code_enable =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x04) >>
        2;
    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.synchronous_bit_width_format =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x02) >>
        1;
    aico_remote_command_data_type3.aico_remote_command_dataformat_type3.tail_code_width_format =
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0x01;
}

void aico_remote_ldr_command_type3_transfer(void)
{
    u8 reframe = 0;

    u16 data_frame_len = 0;

    aico_remote_ldr_command_type3_analysis();
    uint16_t BaseTime3 = 0;
    //初始化载波
    //aico_bsp_timer_carrier_pwm_stop();
    //aico_bsp_timer_carrier_pwm_init(&period_t,(4000/(aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle+1)));
    //aico_bsp_timer_carrier_pwm_duty_set(period_t/3);
    //aico_bsp_timer_carrier_pwm_start();

    //多帧写码内容不一致

    gIrBuf->pBufLen = 0;
    float freq3 = 0;
    aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = 40000 /
            (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle + 1);
    freq3 = aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    freq3 = freq3 / 10;
    BaseTime3 = 100000 /
                aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;

    if (aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value % 8 == 0)
    {
        data_frame_len = aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value
                         / 8;
    }
    else
    {
        data_frame_len = aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value
                         / 8 + 1;
    }
    if ((data_frame_len *
         (aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value + 1)) >
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len)
    {
        data_frame_len = 0;
    }
    p_send_buf_len = 0;
    for (reframe = 0;
         reframe <= aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value;
         reframe++)
    {
        int i = 0;
        //引导码
        if (aico_remote_command_data_type3.aico_remote_command_dataformat_type3.pilot_code_enable ==
            1) //有引导码
        {


            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                aico_remote_command_data_type3.aico_remote_command_datacycle_type3.pilot_code_high_time, BaseTime3);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                aico_remote_command_data_type3.aico_remote_command_datacycle_type3.pilot_code_low_time, BaseTime3);
            //AICO_LDR_DATA_PIN(1);
            //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.pilot_code_high_time);
            //AICO_LDR_DATA_PIN(0);
            //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.pilot_code_low_time);
        }

        //数据
        for (i = 0; i < aico_remote_command.aico_remote_command_databits.aico_remote_command_databits_value;
             i++)
        {
            if (aico_remote_command_data_type3.aico_remote_command_dataformat_type3.data_encoding_format ==
                0) //正常数据
            {
                if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 0)
                {
                    if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                             + data_frame_len * reframe + (i / 8)] & (0x80 >> (i % 8)))
                    {

                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_high_time, BaseTime3);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_low_time, BaseTime3);

                        //AICO_LDR_DATA_PIN(1);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_high_time);
                        //AICO_LDR_DATA_PIN(0);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_low_time);
                    }
                    else
                    {
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time, BaseTime3);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_low_time, BaseTime3);

                        //AICO_LDR_DATA_PIN(1);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time);
                        //AICO_LDR_DATA_PIN(0);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_low_time);
                    }
                }
                else if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 1)
                {
                    if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                             + data_frame_len * reframe + (i / 8)] & (0x01 << (i % 8)))
                    {

                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_high_time, BaseTime3);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_low_time, BaseTime3);
                        //AICO_LDR_DATA_PIN(1);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_high_time);
                        //AICO_LDR_DATA_PIN(0);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_low_time);
                    }
                    else
                    {
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time, BaseTime3);
                        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                            aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_low_time, BaseTime3);
                        //AICO_LDR_DATA_PIN(1);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time);
                        //AICO_LDR_DATA_PIN(0);
                        //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_low_time);
                    }
                }
            }
            else if (aico_remote_command_data_type3.aico_remote_command_dataformat_type3.data_encoding_format ==
                     1)  //双相位编码
            {
                if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                         + data_frame_len * reframe + (i / 8)] & (0x80 >> (i % 8)))
                {

                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_high_time, BaseTime3);
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                        aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_low_time, BaseTime3);

                    //AICO_LDR_DATA_PIN(0);
                    //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_low_time);
                    //AICO_LDR_DATA_PIN(1);
                    //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data1_high_time);
                }
                else
                {

                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                        aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time, BaseTime3);
                    gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                        aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_low_time, BaseTime3);
                    //AICO_LDR_DATA_PIN(1);
                    //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time);
                    //AICO_LDR_DATA_PIN(0);
                    //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_low_time);
                }
            }
        }

        //尾码
        if (aico_remote_command_data_type3.aico_remote_command_dataformat_type3.tail_code_enable == 1)
        {


            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_high_time, BaseTime3);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_low_time, BaseTime3);
            //AICO_LDR_DATA_PIN(1);
            //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_high_time);
            //AICO_LDR_DATA_PIN(0);
            //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_low_time);
        }

        //同步码
        if (aico_remote_command_data_type3.aico_remote_command_dataformat_type3.synchronous_bit_enalbe == 1)
        {
            if (aico_remote_command_data_type3.aico_remote_command_dataformat_type3.synchronous_bit_width_format
                == 0)
            {

                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time, BaseTime3);
                //AICO_LDR_DATA_PIN(1);
                //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.data0_high_time);

            }
            else
            {
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH |
                                                                    aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_low_time, BaseTime3);
                //AICO_LDR_DATA_PIN(1);
                //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_low_time);
            }
        }

        //间隙（无尾码）
        if (aico_remote_command_data_type3.aico_remote_command_dataformat_type3.tail_code_enable == 0)
        {
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW |
                                                                aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_low_time, BaseTime3);
            //AICO_LDR_DATA_PIN(0);
            //delay_us(aico_remote_command_data_type3.aico_remote_command_datacycle_type3.tail_code_low_time);
        }
    }
    ir_send_driver_init(freq3);
    ir_send_start();
    //aico_bsp_timer_carrier_pwm_stop();
}

/*-----------------------CMD_TYPE4---------------------------*/
//YYYY
struct Aico_Remote_Command_Dataformat_Type4
{
    u8 prefix_code_num;          //前缀码数量
    u8 data_code_type_num;       //数据码值种类数量
    u8 combination_code_num;     //组合码数量
};

//TTTT
struct Aico_Remote_Command_Datacycle_Type4
{
    u16 prefix_code_addr[20][2];        //前缀码n起始地址
    u16 data_code_addr[8][2];           //数据码n起始地址
    u16 combination_code_addr;          //组合码起始地址
    u16 combination_code_val[20][3];    //组合码组成值 0:前缀码序号 1:按键码起始位 2:按键码长度
};

struct Aico_Remote_Command_Data_Type4
{
    struct Aico_Remote_Command_Dataformat_Type4 aico_remote_command_dataformat_type4;
    struct Aico_Remote_Command_Datacycle_Type4 aico_remote_command_datacycle_type4;
};

void aico_remote_ldr_command_type4_transfer(void)
{
    int i = 0;
    u8 reframe = 0;
    u8 prefix_code_num = 1;
    u8 data_code_num = 0;
    u8 combination_code_num = 0;
    struct Aico_Remote_Command_Data_Type4 aico_remote_command_data_type4;

    uint16_t BaseTime4 = 0;
    //解析YYYY
    aico_remote_command_data_type4.aico_remote_command_dataformat_type4.prefix_code_num =
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 11;
    aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num =
        (aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value >> 8) &
        0x07;
    aico_remote_command_data_type4.aico_remote_command_dataformat_type4.combination_code_num =
        aico_remote_command.aico_remote_command_dataformat.aico_remote_command_dataformat_value & 0xff;

    //解析TTTT
    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[0][0] = 5;
    for (i = 5; i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1; i = i + 2)
    {
        if (data_code_num ==
            aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num)
        {
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_addr = i + 2;
                data_code_num++;
            }
        }

        if (data_code_num > 0 &&
            data_code_num <
            aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num)
        {
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[data_code_num][0]
                    = i + 2;
                data_code_num++;
            }
        }

        if (prefix_code_num ==
            aico_remote_command_data_type4.aico_remote_command_dataformat_type4.prefix_code_num)
        {
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[0][0] = i + 2;
                prefix_code_num++;
                data_code_num = 1;
            }
        }

        if (prefix_code_num <
            aico_remote_command_data_type4.aico_remote_command_dataformat_type4.prefix_code_num)
        {
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x80) == 0)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[prefix_code_num][0]
                    = i + 2;
                prefix_code_num++;
            }
        }
    }

    for (i = 0;
         i < aico_remote_command_data_type4.aico_remote_command_dataformat_type4.prefix_code_num - 1; i++)
    {
        aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[i][1] =
            aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[i + 1][0] -
            aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[i][0];
    }
    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[aico_remote_command_data_type4.aico_remote_command_dataformat_type4.prefix_code_num
            - 1][1] = aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[0][0] -
                      aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[aico_remote_command_data_type4.aico_remote_command_dataformat_type4.prefix_code_num
                              - 1][0];

    for (i = 0;
         i < aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num - 1; i++)
    {
        aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[i][1] =
            aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[i + 1][0] -
            aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[i][0];
    }
    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num
            - 1][1] = aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_addr -
                      aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num
                              - 1][0];

    for (i = 0; i < aico_remote_command_data_type4.aico_remote_command_dataformat_type4.prefix_code_num;
         i++)
    {
        //printf("addr:%d len:%d\r\n",aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[i][0],aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[i][1]);
    }
    for (i = 0;
         i < aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num; i++)
    {
        //printf("addr:%d len:%d\r\n",aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[i][0],aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[i][1]);
    }

    for (i = aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_addr;
         i < aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1;)
    {
        int addr_data, len_data;
        if (combination_code_num <
            aico_remote_command_data_type4.aico_remote_command_dataformat_type4.combination_code_num)
        {
            aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][0]
                = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] >> 4;
            addr_data = ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] >> 2) & 0x03);
            len_data = (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i] & 0x03);

            if (addr_data == 0)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][1]
                    = 0;
            }
            else if (addr_data == 1)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][1]
                    = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1];
            }
            else if (addr_data == 2)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][1]
                    = (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 1] << 8) +
                      aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + 2];
            }
            else
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][1]
                    = 0;
            }

            if (len_data == 0)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][2]
                    = 0;
            }
            else if (len_data == 1)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][2]
                    = aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr_data + 1];
            }
            else if (len_data == 2)
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][2]
                    = (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr_data + 1] << 8) +
                      aico_remote_receive_buf.aico_remote_iic_reg_data_buf[i + addr_data + 2];
            }
            else
            {
                aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[combination_code_num][2]
                    = 0;
            }

            i = i + addr_data + len_data + 1;

            combination_code_num++;

            //计算数据位的起始地址以及长度
            if (combination_code_num ==
                aico_remote_command_data_type4.aico_remote_command_dataformat_type4.combination_code_num)
            {
                aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr = i;
                aico_remote_command.aico_remote_command_data.aico_remote_command_data_len =
                    aico_remote_receive_buf.aico_remote_iic_reg_data_len - 1 - i;
            }
        }
        else
        {
            i++;
        }
    }

    //初始化载波
    //aico_bsp_timer_carrier_pwm_stop();
    //aico_bsp_timer_carrier_pwm_init(&period_t,(4000/(aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle+1)));
    //aico_bsp_timer_carrier_pwm_duty_set(period_t/3);
    //aico_bsp_timer_carrier_pwm_start();
    gIrBuf->pBufLen = 0;
    float freq4 = 0;
    aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = 40000 /
            (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle + 1);
    freq4 = aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    freq4 = freq4 / 10;
    BaseTime4 = 100000 /
                aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    for (reframe = 0;
         reframe <= aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value;
         reframe++)
    {
        for (i = 0;
             i < aico_remote_command_data_type4.aico_remote_command_dataformat_type4.combination_code_num; i++)
        {
            if (aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[i][0] >
                0)
            {
                aico_remote_ldr_ttl_send(
                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[i][0]
                            - 1][0], aico_remote_command_data_type4.aico_remote_command_datacycle_type4.prefix_code_addr[aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[i][0]
                                    - 1][1], BaseTime4);
            }
            //存在数据位
            if (aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[i][2] >
                0)
            {
                if (aico_remote_command_data_type4.aico_remote_command_dataformat_type4.data_code_type_num == 2)
                {
                    int j = 0;

                    for (j = 0;
                         j < aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[i][2];
                         j++)
                    {
                        //高位在前
                        if (aico_remote_command.aico_remote_command_framecfg.aico_remote_command_framecfg_g == 0)
                        {
                            if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                                     + aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[i][1] / 8
                                                                                     + j / 8] & (0x80 >> (j % 8)))
                            {
                                aico_remote_ldr_ttl_send(
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[1][0],
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[1][1], BaseTime4);
                            }
                            else
                            {
                                aico_remote_ldr_ttl_send(
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[0][0],
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[0][1], BaseTime4);
                            }
                        }
                        else //低位在前
                        {
                            if (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                                     + aico_remote_command_data_type4.aico_remote_command_datacycle_type4.combination_code_val[i][1] / 8
                                                                                     + j / 8] & (0x01 << (j % 8)))
                            {
                                aico_remote_ldr_ttl_send(
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[1][0],
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[1][1], BaseTime4);
                            }
                            else
                            {
                                aico_remote_ldr_ttl_send(
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[0][0],
                                    aico_remote_command_data_type4.aico_remote_command_datacycle_type4.data_code_addr[0][1], BaseTime4);
                            }
                        }
                    }
                }
            }
        }
    }
    ir_send_driver_init(freq4);
    ir_send_start();
    //aico_bsp_timer_carrier_pwm_stop();
}

/*-----------------------CMD_TYPE5---------------------------*/
void aico_remote_ldr_command_type5_transfer(void)
{
    u8 reframe = 0;


    //初始化载波
    //aico_bsp_timer_carrier_pwm_stop();
    //aico_bsp_timer_carrier_pwm_init(&period_t,(4000/(aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle+1)));
    //aico_bsp_timer_carrier_pwm_duty_set(period_t/3);
    //aico_bsp_timer_carrier_pwm_start();
    uint16_t BaseTime5 = 0;
    gIrBuf->pBufLen = 0;
    float freq5 = 0;
    aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = 40000 /
            (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle + 1);
    freq5 = aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    freq5 = freq5 / 10;
    BaseTime5 = 100000 /
                aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    for (reframe = 0;
         reframe <= aico_remote_command.aico_remote_command_reframe.aico_remote_command_reframe_value;
         reframe++)
    {
        int i = 0;

        for (i = 0; i < 4; i++)
        {

            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | 200, BaseTime5);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 780 + 135 *
                                                                (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                        + i] >> 4), BaseTime5);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | 200, BaseTime5);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 780 + 135 *
                                                                (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                        + i] & 0x0f), BaseTime5);
            //AICO_LDR_DATA_PIN(1);
            //delay_us(200);
            //AICO_LDR_DATA_PIN(0);
            //delay_us(780+135*(aico_remote_iic_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr+i]>>4));
            //AICO_LDR_DATA_PIN(1);
            //delay_us(200);
            //AICO_LDR_DATA_PIN(0);
            //delay_us(780+135*(aico_remote_iic_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr+i]&0x0f));
        }
        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | 200, BaseTime5);
        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 13800, BaseTime5);
        //AICO_LDR_DATA_PIN(1);
        //delay_us(200);
        //AICO_LDR_DATA_PIN(0);
        //delay_us(13800);

        for (i = 4; i < aico_remote_command.aico_remote_command_data.aico_remote_command_data_len - 1; i++)
        {

            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | 200, BaseTime5);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 780 + 135 *
                                                                (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                        + i] >> 4), BaseTime5);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | 200, BaseTime5);
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 780 + 135 *
                                                                (aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                        + i] & 0x0f), BaseTime5);

            //AICO_LDR_DATA_PIN(1);
            //delay_us(200);
            //AICO_LDR_DATA_PIN(0);
            //delay_us(780+135*(aico_remote_iic_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr+i]>>4));
            //AICO_LDR_DATA_PIN(1);
            //delay_us(200);
            //AICO_LDR_DATA_PIN(0);
            //delay_us(780+135*(aico_remote_iic_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr+i]&0x0f));
        }
        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | 200, BaseTime5);
        gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | 80000, BaseTime5);
        //AICO_LDR_DATA_PIN(1);
        //delay_us(200);
        //AICO_LDR_DATA_PIN(0);
        //delay_ms(80);
    }
    ir_send_driver_init(freq5);
    ir_send_start();
    //aico_bsp_timer_carrier_pwm_stop();
}

/*-----------------------CMD_TYPE6---------------------------*/
void aico_remote_ldr_command_type6_transfer(void)
{
    int i = 0;

    uint16_t BaseTime6 = 0;
    gIrBuf->pBufLen = 0;
    float freq6 = 0;
    aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = 40000 /
            (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle + 1);
    freq6 = aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    freq6 = freq6 / 10;
    BaseTime6 = 100000 /
                aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    aico_remote_command.aico_remote_command_data.aico_remote_command_data_len =
        aico_remote_command.aico_remote_command_data.aico_remote_command_data_len - 1;

    if (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carriet_enable == 0x80)
    {
        //aico_bsp_timer_carrier_pwm_stop();
        //aico_bsp_timer_carrier_pwm_init(&period_t,(4000/(aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle+1)));
        //aico_bsp_timer_carrier_pwm_duty_set(period_t/3);
        //aico_bsp_timer_carrier_pwm_start();
    }
    else
    {
        //aico_bsp_timer_carrier_pwm_stop();
    }

    if ((aico_remote_command.aico_remote_command_data.aico_remote_command_data_len % 2) == 0)
    {
        for (i = 0; i < aico_remote_command.aico_remote_command_data.aico_remote_command_data_len; i += 2)
        {
            u16 data_time = 0;
            if ((aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                      + i] >> 7) == 1)
            {

                data_time = ((
                                 aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                                      + i] & 0x7f) << 8) +
                            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                                 + i + 1];
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | data_time, BaseTime6);

                //AICO_LDR_DATA_PIN(1);
            }
            else
            {
                data_time = ((
                                 aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                                      + i] & 0x7f) << 8) +
                            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                                 + i + 1];
                gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | data_time, BaseTime6);

                //AICO_LDR_DATA_PIN(0);
            }


            //delay_us(data_time);
        }
    }
    ir_send_driver_init(freq6);
    ir_send_start();
//  aico_bsp_timer_carrier_pwm_stop();
}

/*-----------------------CMD_TYPE7---------------------------*/
void aico_remote_ldr_command_type7_transfer(void)
{
    int i = 0;
    uint8_t flag = 0;
    u8 isCarrier = FALSE;
    uint16_t BaseTime7 = 0;
    gIrBuf->pBufLen = 0;
    float freq7 = 0;
    aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle = 40000 /
            (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle + 1);
    freq7 = aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    freq7 = freq7 / 10;
    BaseTime7 = 100000 /
                aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle;
    if (aico_remote_command.aico_remote_command_carrier.aico_remote_command_carriet_enable == 0x80)
    {
        //aico_bsp_timer_carrier_pwm_stop();
        //aico_bsp_timer_carrier_pwm_init(&period_t,(4000/(aico_remote_command.aico_remote_command_carrier.aico_remote_command_carrier_cycle+1)));
        //aico_bsp_timer_carrier_pwm_duty_set(period_t/3);
        //aico_bsp_timer_carrier_pwm_start();
        isCarrier = TRUE;
    }
    else
    {
        //aico_bsp_timer_carrier_pwm_stop();
        isCarrier = FALSE;
    }

    for (i = 0; i < aico_remote_command.aico_remote_command_data.aico_remote_command_data_len;)
    {
        u16 remoteValue = 0;
        u8 cspValue;

        cspValue =
            aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                 + i];
        if (cspValue & 0x40)
        {
            //AICO_LDR_DATA_PIN(1);
            flag = 1;
        }
        else
        {
            //AICO_LDR_DATA_PIN(0);
            flag = 0;
        }

        if (cspValue & 0x80)
        {
            if (cspValue & 0x20)
            {
                remoteValue += 16384;
            }
            if (cspValue & 0x10)
            {
                remoteValue += 8192;
            }
            if (cspValue & 0x08)
            {
                remoteValue += 4096;
            }
            if (cspValue & 0x04)
            {
                remoteValue += 2048;
            }
            if (cspValue & 0x02)
            {
                remoteValue += 1024;
            }
            if (cspValue & 0x01)
            {
                remoteValue += 512;
            }

            cspValue =
                aico_remote_receive_buf.aico_remote_iic_reg_data_buf[aico_remote_command.aico_remote_command_data.aico_remote_command_data_start_addr
                                                                     + i + 1];
            if (cspValue & 0x80)
            {
                remoteValue += 256;
            }
            if (cspValue & 0x40)
            {
                remoteValue += 128;
            }
            if (cspValue & 0x20)
            {
                remoteValue += 64;
            }
            if (cspValue & 0x10)
            {
                remoteValue += 32;
            }
            if (cspValue & 0x08)
            {
                remoteValue += 16;
            }
            if (cspValue & 0x04)
            {
                remoteValue += 8;
            }
            if (cspValue & 0x02)
            {
                remoteValue += 4;
            }
            if (cspValue & 0x01)
            {
                remoteValue += 2;
            }

            i = i + 2;
        }
        else
        {
            if (isCarrier == TRUE)
            {
                if (cspValue & 0x20)
                {
                    remoteValue += 512;
                }
                if (cspValue & 0x10)
                {
                    remoteValue += 256;
                }
                if (cspValue & 0x08)
                {
                    remoteValue += 128;
                }
                if (cspValue & 0x04)
                {
                    remoteValue += 64;
                }
                if (cspValue & 0x02)
                {
                    remoteValue += 32;
                }
                if (cspValue & 0x01)
                {
                    remoteValue += 16;
                }
            }
            else
            {
                if (cspValue & 0x20)
                {
                    remoteValue += 256;
                }
                if (cspValue & 0x10)
                {
                    remoteValue += 128;
                }
                if (cspValue & 0x08)
                {
                    remoteValue += 64;
                }
                if (cspValue & 0x04)
                {
                    remoteValue += 32;
                }
                switch (cspValue & 0x03)
                {
                case 0x00 : remoteValue += 4;  break;
                case 0x01 : remoteValue += 10; break;
                case 0x02 : remoteValue += 16; break;
                case 0x03 : remoteValue += 24; break;
                default : break;
                }
            }
            i = i + 1;
        }
        if (flag == 1)
        {
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_HIGH | remoteValue, BaseTime7);

        }
        else
        {
            gIrBuf->pBuf[gIrBuf->pBufLen++] =  HL_Time2TxBufCount(PULSE_LOW | remoteValue, BaseTime7);
        }
        //delay_us(remoteValue);
    }
    ir_send_driver_init(freq7);
    ir_send_start();
    //aico_bsp_timer_carrier_pwm_stop();
}

void aico_remote_ldr_signal_transfer(void)
{

    gIrBuf = plt_malloc(4000, RAM_TYPE_DATA_ON);
	  if (NULL == gIrBuf)
    {
       printe("aico_remote_ldr_signal_transfer: allocate id buffer failed!");
         
    }
	  else
		{	
			switch (aico_remote_command.aico_remote_command_head.aico_remote_command_type)
			{
			case AICO_REMOTE_COMMAND_HEAD_TYPE_1: aico_remote_ldr_command_type1_transfer(); break;
			case AICO_REMOTE_COMMAND_HEAD_TYPE_2: aico_remote_ldr_command_type2_transfer(); break;
			case AICO_REMOTE_COMMAND_HEAD_TYPE_3: aico_remote_ldr_command_type3_transfer(); break;
			case AICO_REMOTE_COMMAND_HEAD_TYPE_4: aico_remote_ldr_command_type4_transfer(); break;
			case AICO_REMOTE_COMMAND_HEAD_TYPE_5: aico_remote_ldr_command_type5_transfer(); break;
			case AICO_REMOTE_COMMAND_HEAD_TYPE_6: aico_remote_ldr_command_type6_transfer(); break;
			case AICO_REMOTE_COMMAND_HEAD_TYPE_7: aico_remote_ldr_command_type7_transfer(); break;
			default: break;
			}
		}
}



void aico_remote_command_send(void)
{
    if (aico_remote_command_analysis(aico_remote_receive_buf.aico_remote_iic_reg_data_buf,
                                     aico_remote_receive_buf.aico_remote_iic_reg_data_len))
    {
        aico_remote_ldr_signal_transfer();
    }
}

void app_ir_send_msg(void *pargs);
DATA_RAM_FUNCTION void ir_send_int_handler(void)
{
    IR_MaskINTConfig(IR_INT_TF_LEVEL, ENABLE);

    /* Continue to send by interrupt */
    if (IR_GetINTStatus(IR_INT_TF_LEVEL) == SET)
    {
        /* The remaining data is larger than the TX FIFO length */
        if ((gIrBuf->pBufLen - IR_TX_Count) >= IR_TX_FIFO_SIZE)
        {
            IR_SendBuf(gIrBuf->pBuf + IR_TX_Count, (IR_TX_FIFO_SIZE - IR_TX_FIFO_THR_LEVEL), DISABLE);
            IR_TX_Count += (IR_TX_FIFO_SIZE - IR_TX_FIFO_THR_LEVEL);

            /* Clear threshold interrupt */
            IR_ClearINTPendingBit(IR_INT_TF_LEVEL_CLR);
        }
        else if ((gIrBuf->pBufLen - IR_TX_Count) > 0)
        {
            /* The remaining data is less than the TX FIFO length */

            /*  Configure TX threshold level to zero and trigger interrupt when TX FIFO is empty */
            IR_SetTxThreshold(0);
            IR_SendBuf(gIrBuf->pBuf + IR_TX_Count, gIrBuf->pBufLen - IR_TX_Count, DISABLE);
            IR_TX_Count += (gIrBuf->pBufLen - IR_TX_Count);

            /* Clear threshold interrupt */
            IR_ClearINTPendingBit(IR_INT_TF_LEVEL_CLR);
        }
        else
        {
            /* Tx completed */
            /* Disable IR tx empty interrupt */
            app_ir_send_msg(0);
            DBG_DIRECT("app_ir_send_msg");
            IR_INTConfig(IR_INT_TF_LEVEL, DISABLE);
            IR_TX_Count = 0;
            /* Clear threshold interrupt */
            IR_ClearINTPendingBit(IR_INT_TF_LEVEL_CLR);
					  //plt_free(gIrBuf, RAM_TYPE_DATA_ON);
            IR_INTConfig(IR_INT_TF_LEVEL, DISABLE);
			
        }
    }

    /* Unmask IR interrupt */
    IR_MaskINTConfig(IR_INT_TF_LEVEL, DISABLE);
}

DATA_RAM_FUNCTION void app_ir_send_msg(void *pargs)
{
    uint8_t event = EVENT_IO_TO_APP;
    T_IO_MSG msg;
    msg.type = IO_MSG_TYPE_IR_SEND_COMPLETE;
    msg.subtype = 0;
    msg.u.buf = pargs;
    if (os_msg_send(io_queue_handle, &msg, 0) == false)
    {
    }
    else if (os_msg_send(evt_queue_handle, &event, 0) == false)
    {
    }
}


void ir_send_girbuf_free(void)
{
   plt_free(gIrBuf, RAM_TYPE_DATA_ON);
   gIrBuf = NULL;
       
}

static plt_timer_t ir_send_timeout_check =0;


void ir_send_check_time_stop(void)
{
    if (ir_send_timeout_check != NULL)
    {
        plt_timer_stop(ir_send_timeout_check, 0);
        //plt_timer_delete(ir_send_timeout_check, 0);
        //ir_send_timeout_check = NULL;
    }
}

void ir_send_timeout(void *pargs)
{   
     app_ir_send_msg(0);
     IR_INTConfig(IR_INT_TF_LEVEL, DISABLE);
      IR_TX_Count = 0;
            /* Clear threshold interrupt */
     IR_ClearINTPendingBit(IR_INT_TF_LEVEL_CLR);
					  //plt_free(gIrBuf, RAM_TYPE_DATA_ON);
     IR_INTConfig(IR_INT_TF_LEVEL, DISABLE);
	 DBG_DIRECT("ir_send_timeout:app_ir_send_msg");    
   
}


void ir_send_check_time_start(void)
{

   if (ir_send_timeout_check == NULL)
	 {
        ir_send_timeout_check = plt_timer_create("ir_send_timeout_check", 1000, 0, 0, ir_send_timeout);

        if (ir_send_timeout_check == NULL)
        {
            printe("ir_learn_timeout_check failed");
        }
	      else
	      plt_timer_start(ir_send_timeout_check, 0);
	 }
	 else
	 {
        plt_timer_start(ir_send_timeout_check, 0);
   }
}


