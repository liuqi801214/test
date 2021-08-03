/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      infrared.h
* @brief     Smart mesh demo application
* @details
* @author    hector_huang
* @date      2021-01-28
* @version   v0.1
* *********************************************************************************************************
*/
#include "mesh_api.h"
#include "infrared.h"
#include "app_task.h"
#include "aico_bsp_idr.h"
#include "aico_bsp_irl.h"
#include "infrared_flash.h"
#include "infrared_temper.h"
#include "infrared_ext_flash.h"
#include "flash_map.h"
#include "infrared_light.h"
#include "infrared_light_controller.h"
#include "exfun.h"
#include "infrared_model.h"
#include "dfu_server.h"
#include "button_app.h"
#include "mp_test.h"
#define INFRARED_TIMEOUT_PERIODIC_IR               0
#define INFRARED_TIMEOUT_STUDY                     1
#define INFRARED_TIMEOUT_EXEC_SCENE                2
#define INFRARED_TIMEOUT_OTA                       3

extern uint32_t *LoopQueue;

//add test


typedef struct _ir_periodic_entry_t
{
    struct _ir_periodic_entry_t *pnext;
    plt_timer_t id_tmr;
    uint32_t interval;
    uint32_t total_time;
    uint16_t exec_id;
    uint8_t ir_code_len;
    uint8_t ir_code[0];
} ir_periodic_entry_t;

typedef struct
{
    bool studying;
    plt_timer_t study_tmr;
    uint16_t study_id;
    //uint8_t study_data[1316];
    uint8_t study_data[2000];
    uint16_t study_data_len;
    ir_study_result_t study_result;
} ir_study_t;

typedef struct _ir_exec_scene_entry_t
{
    struct _ir_exec_scene_entry_t *pnext;
    ir_scene_label_t scene;
    ir_scene_key_t keys[IR_SCENE_KEY_MAX_SIZE];
    plt_timer_t delay_timer;
    ir_scene_key_t *pcur_key;
} ir_exec_scene_entry_t;

typedef struct _ir_thermostat_entry_t
{
    struct _ir_thermostat_entry_t *pnext;
    ir_thermostat_label_t label;
    ir_thermostat_setting_t setting;
    uint8_t state;
} ir_thermostat_entry_t;

typedef struct
{
    plt_list_t ir_periodic_list;
    ir_study_t study;
    plt_list_t ir_exec_scene_list;
    uint8_t temper_param;
    uint8_t user_id[6];
    uint8_t region_id[6];
    uint8_t common_flag[6];
    ir_blue_light_t blue_light;
    ir_red_light_t red_light;
    ir_beeper_t beeper;
    plt_timer_t ota_timer;
} ir_ctx_t;

ir_ctx_t ir_ctx;

void ir_init(void)
{
    ir_ctx.blue_light.func_enable = true;
    ir_ctx.blue_light.work_enable = true;
    ir_ctx.blue_light.interval = 2;
    ir_ctx.blue_light.active_time = 300;
    ir_ctx.red_light.func_enable = true;
    ir_ctx.beeper.conn_enable = true;
    ir_ctx.beeper.func_enable = true;
}

ir_err_code_t ir_add(ir_rcu_id_t rcu_id, ir_id_label_t ir_id_label, uint8_t *pir_code,
                     uint16_t ir_code_len)
{
    printi("ir_add: rcu id 0x%02x%02x%02x%02x%02x%02x, ir id 0x%02x%02x%02x%02x%02x%02x, ir label %d, ir code len %d",
           rcu_id.id[0], rcu_id.id[1], rcu_id.id[2], rcu_id.id[3], rcu_id.id[4], rcu_id.id[5],
           ir_id_label.id.id[0], ir_id_label.id.id[1], ir_id_label.id.id[2], ir_id_label.id.id[3],
           ir_id_label.id.id[4], ir_id_label.id.id[5],
           ir_id_label.label, ir_code_len);
    return ir_ext_flash_add_ir(rcu_id, ir_id_label, pir_code, ir_code_len);
}

bool ir_get_label(ir_id_t ir_id, uint32_t *plabel)
{
    return fatfs_get_ir_label_by_id(ir_id, plabel);;
}

bool ir_get_by_id(ir_id_t ir_id, ir_code_t *pir)
{
    return ir_ext_flash_get_by_id(ir_id, pir);
}

bool ir_get_by_id_by_no_pocde(ir_id_t ir_id, ir_code_t *pir)
{
    return ir_ext_flash_get_by_id_no_pcode(ir_id, pir);
}

uint16_t ir_rcu_id_ir_count(ir_rcu_id_t rcu_id)
{
    return ir_ext_flash_rcu_id_ir_count(rcu_id);
}

uint16_t ir_inquire_by_rcu_id(ir_rcu_id_t rcu_id, ir_id_label_t *pid_labels)
{
    return ir_ext_flash_inquire_by_rcu_id(rcu_id, pid_labels);
}

uint16_t ir_inquire_count(void)
{
    return ir_ext_flash_inquire_ir_count();
}

ir_err_code_t ir_delete_by_id(ir_id_t *pir_ids, uint8_t id_counts)
{
    return ir_ext_flash_delete_ir_by_id(pir_ids, id_counts);
}

ir_err_code_t ir_delete_by_rcu_id(ir_rcu_id_t rcu_id)
{
    printi("ir_delete_by_rcu_id: delete rcu %02x-%02x-%02x-%02x-%02x-%02x",
           rcu_id.id[5], rcu_id.id[4], rcu_id.id[3], rcu_id.id[2],
           rcu_id.id[1], rcu_id.id[0]);
    return ir_ext_flash_delete_ir_by_rcu_id(rcu_id);
}

ir_err_code_t ir_delete_by_rcu_id_by_egg_task(ir_rcu_id_t rcu_id,uint16_t dst, uint16_t tid)
                                
{
    printi("ir_delete_by_rcu_id: delete rcu %02x-%02x-%02x-%02x-%02x-%02x",
           rcu_id.id[5], rcu_id.id[4], rcu_id.id[3], rcu_id.id[2],
           rcu_id.id[1], rcu_id.id[0]);
    return ir_ext_flash_delete_ir_by_rcu_id_by_egg_task(rcu_id,dst,tid);//ir_ext_flash_delete_ir_by_rcu_id(rcu_id,dst,tid);
}




ir_err_code_t ir_delete_all(void)
{
    printi("ir_delete_all: delete all ir");
    return ir_ext_flash_delete_all_ir();
}

#if DLPS_EN    
    #include "dlps_crtl.h"
    bool is_all_light_idle(void);
#endif



DATA_RAM_FUNCTION void ir_receive_data_exec(uint8_t *p_data, uint16_t len);
DATA_RAM_FUNCTION ir_err_code_t ir_exec(uint8_t *pir_code, uint16_t ir_code_len,uint16_t *ir_study_id)
{
#if DLPS_EN 
    //dlps_ctrl_set(0);
	  egg_ir_exec_dlps_set(0);
#endif
	  //uint16_t ir_study_id;
    if(ir_study_id != NULL)
    {
        if(ir_get_study_status(ir_study_id) == 1)
        {
            printi("ir_study_stop");   
            ir_study_stop(*ir_study_id);
            
        }
        else
        *ir_study_id =0xff;
    }
    ir_receive_data_exec(pir_code, ir_code_len);
    printi("ir_exec: ir code len %d", ir_code_len);
    dprintt(pir_code, ir_code_len);
    
    light_cwrgb_turn_off();
    //light_breath_red_blue_stop();
		light_breath_red_stop();
    light_blink_breath_time_stop();
    if(light_controller_get_period() ==5000)
	  {								
	    light_controller_period_change(20);							
	  }
    //light_set_blue_lightness(0);
    
    light_blink(light_get_red(), 0, 60000, 50, 50, 1, NULL);
#if DLPS_EN 
    if(is_all_light_idle())
    //dlps_ctrl_set(1);
	 egg_ir_exec_dlps_set(1);
#endif
    return IR_SUCCESS;
}

void ir_exec_periodic_timeout(void *pargs)
{
    T_IO_MSG msg;
    msg.type = INFRARED_MSG;
    msg.subtype = INFRARED_TIMEOUT_PERIODIC_IR;
    msg.u.buf = pargs;
    app_send_msg_to_apptask(&msg);
}

static void ir_exec_periodic_timeout_handle(plt_timer_t timer)
{
    ir_periodic_entry_t *pentry = (ir_periodic_entry_t *)ir_ctx.ir_periodic_list.pfirst;
    ir_periodic_entry_t *pprev_node = NULL;
    while (pentry != NULL)
    {
        if (pentry->id_tmr == timer)
        {
            if (pentry->total_time < pentry->interval)
            {
                /* exec finish */
                plt_list_delete(&ir_ctx.ir_periodic_list, pprev_node, pentry);
                plt_timer_delete(pentry->id_tmr, 0);
                plt_free(pentry, RAM_TYPE_DATA_ON);
                printi("ir_exec_periodic_timeout_handle: exec id %d timeout", pentry->exec_id);
            }
            else
            {
                pentry->total_time -= pentry->interval;
                
                ir_exec(pentry->ir_code, pentry->ir_code_len,NULL);
            }
            return ;
        }
        pprev_node = pentry;
        pentry = pentry->pnext;
    }
}

ir_err_code_t ir_exec_periodic(uint16_t exec_id, uint8_t *pir_code, uint16_t ir_code_len,
                               uint8_t interval, uint8_t total_time)
{
    ir_periodic_entry_t *pentry = plt_malloc(sizeof(ir_periodic_entry_t) + ir_code_len,
                                             RAM_TYPE_DATA_ON);
    if (NULL == pentry)
    {
        printe("ir_exec_periodic: failed, out of memory");
        return IR_ERR_UNKNOWN_ERR;
    }

    pentry->interval = interval;
    pentry->interval *= 100;
    pentry->id_tmr = plt_timer_create("pid", pentry->interval, true, 0, ir_exec_periodic_timeout);
    if (NULL == pentry->id_tmr)
    {
        plt_free(pentry, RAM_TYPE_DATA_ON);
        printe("ir_exec_periodic: create timer failed");
        return IR_ERR_UNKNOWN_ERR;
    }
    pentry->total_time = total_time;
    pentry->total_time *= 1000;
    pentry->exec_id = exec_id;
    pentry->ir_code_len = ir_code_len;
    memcpy(pentry->ir_code, pir_code, ir_code_len);

    /* start timer */
    plt_timer_start(pentry->id_tmr, 0);

    plt_list_push(&ir_ctx.ir_periodic_list, pentry);
    printi("ir_exec_periodic: exec id %d, interval %d, total_time %d", exec_id, pentry->interval,
           pentry->total_time);
    return IR_SUCCESS;
}

ir_err_code_t ir_exec_periodic_stop(uint16_t exec_id)
{
    ir_periodic_entry_t *pentry = (ir_periodic_entry_t *)ir_ctx.ir_periodic_list.pfirst;
    ir_periodic_entry_t *pprev_node = NULL;
    while (pentry != NULL)
    {
        if (pentry->exec_id == exec_id)
        {
            printi("ir_exec_periodic_stop: exec id %d", exec_id);
            plt_list_delete(&ir_ctx.ir_periodic_list, pprev_node, pentry);
            plt_timer_delete(pentry->id_tmr, 0);
            plt_free(pentry, RAM_TYPE_DATA_ON);
            return IR_SUCCESS;
        }
        pprev_node = pentry;
        pentry = pentry->pnext;
    }

    printw("ir_exec_periodic_stop: failed, not found exec id %d", exec_id);
    return IR_ERR_TIMER_NOT_EXIST;
}

uint16_t ir_inquire_by_id(ir_id_t *pir_ids, uint8_t id_counts, ir_id_label_t *pid_labels)
{
    uint16_t id_out_count = 0;
    ir_code_t *pcode = plt_malloc(sizeof(ir_code_t)*id_counts, RAM_TYPE_DATA_ON);
    if (NULL == pcode)
    {
        printe("ir_inquire_by_id: failed to allocate ir code memory!");
        return 0;
    }

    for (uint8_t i = 0; i < id_counts; ++i)
    {
        //if (ir_get_by_id(pir_ids[i], pcode))
        if(ir_get_by_id_by_no_pocde(pir_ids[i], pcode))
        {

            pid_labels[id_out_count]=pcode->ir_id_label;
           // dprinti(pid_labels[id_out_count].label,4);
           
            id_out_count ++;
        }
    }
    printi("id_out_count=%d",id_out_count);
		if(pcode != NULL)
    plt_free(pcode, RAM_TYPE_DATA_ON);

    return id_out_count;
}

void ir_study_timeout(void *pargs)
{
    T_IO_MSG msg;
    msg.type = INFRARED_MSG;
    msg.subtype = INFRARED_TIMEOUT_STUDY;
    msg.u.buf = pargs;
    app_send_msg_to_apptask(&msg);
}

ir_err_code_t ir_study_start(uint16_t study_id, uint8_t timeout)
{
    if (ir_ctx.study.studying)
    {
        printw("ir_study_start: already studying");
        return IR_ERR_STUDYING;
    }
 #if DLPS_EN 
    //dlps_ctrl_set(0);
      egg_ir_study_set(0);
 #endif  
    light_cwrgb_turn_off();
    light_breath_red_blue_stop();
    light_lighten_red(60000);
    
    printi("light_lighten_red 60000");
    uint32_t study_time = timeout;
    study_time *= 1000;
    if (NULL == ir_ctx.study.study_tmr)
    {
        ir_ctx.study.study_tmr = plt_timer_create("study", study_time, false, 0, ir_study_timeout);
        if (NULL == ir_ctx.study.study_tmr)
        {
            printe("ir_study_start: create study timer failed");
            return IR_ERR_START_STUDY_FAILED;
        }
        plt_timer_start(ir_ctx.study.study_tmr, 0);
    }
    else
    {
        plt_timer_change_period(ir_ctx.study.study_tmr, study_time, 0);
    }
    aico_bsp_irl_enable(1);
    ir_ctx.study.studying = true;
    ir_ctx.study.study_id = study_id;
    ir_ctx.study.study_result = IR_STUDY_SUCCESS;
    ir_ctx.study.study_data_len = 0;
    
    printi("ir_study_start: study id %d, timeout %d", study_id, study_time);

    return IR_SUCCESS;
}

#if DLPS_EN    
    void dlps_ctrl_set(uint8_t enable);
    bool is_all_light_idle(void);
#endif

ir_err_code_t ir_study_stop(uint16_t study_id)
{
    light_lighten_red(0);
    
    if (ir_ctx.study.studying &&
        (ir_ctx.study.study_id == study_id))
    {
        printi("ir_study_stop: study stop, id %d", study_id);
        plt_timer_stop(ir_ctx.study.study_tmr, 0);
        ir_ctx.study.studying = false;
        aico_bsp_irl_enable(0);
    }
    #if DLPS_EN  
    if(is_all_light_idle())
    {//dlps_ctrl_set(1);
              
      egg_ir_study_set(1);
    }
    #endif
    return IR_SUCCESS;
}

bool ir_get_study_status(uint16_t *ir_study_id)
{    
	 if(ir_ctx.study.studying)
	 {
	     if(ir_study_id!= NULL)
         *ir_study_id  = ir_ctx.study.study_id;
		  return 1;
	 }
	 else 
	  	return 0; 
	 
}

void driver_init(void);
#include "rtl876x_rcc.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_gpio.h"
#include "button.h"
#include "infrared_power.h"
void light_breath_restore(void);
ir_err_code_t ir_study_timeout_check(void)
{
   
   	DBG_DIRECT("ir_study_timeout_check len =%d", ir_ctx.study.study_data_len);
	light_lighten_red(0);
    plt_timer_stop(ir_ctx.study.study_tmr, 0);
    aico_bsp_irl_auto_stop();
    ir_study_data_process(ir_ctx.study.study_data, &ir_ctx.study.study_data_len);
   
    if (ir_ctx.study.study_data_len != 0)
    {
       //ir_learn_data_exec(ir_ctx.study.study_data, ir_ctx.study.study_data_len);
       btn_ir_learn_state = BTN_IR_LEARN_STATE_LEARNED;
        
    }
    else
    {
       btn_ir_learn_state = BTN_IR_LEARN_STATE_IDLE;
        
    }        
    ir_ctx.study.studying = false;
    
    aico_bsp_irl_enable(0);
    ir_study_result_notify(MESH_BEARER_ALL,ir_ctx.study.study_id,IR_STUDY_SUCCESS);
    light_cwrgb_turn_off();
    //light_breath_restore();
    if(get_power_light_charging()==true)
    {
      light_breath_red_restore();
    
    }
    else
    {
      light_breath_blue_restore();
    
    }
    //GPIO_ClearINTPendingBit(BUTTON_GPIO);
    GPIO_MaskINTConfig(BUTTON_GPIO, DISABLE);
    GPIO_INTConfig(BUTTON_GPIO, ENABLE);
    
    #if DLPS_EN 
    //if(is_all_light_idle())
     //dlps_ctrl_set(1);
    egg_ir_study_set(1);
    #endif
    return IR_SUCCESS;
}


ir_err_code_t ir_learn_data_send(void)
{
    printi("ir_study_data len =%d", ir_ctx.study.study_data_len);
    if (ir_ctx.study.study_data_len != 0)
    {
        light_cwrgb_turn_off();
        light_breath_red_blue_stop();
        light_blink_breath_time_stop();
        if(light_controller_get_period() ==5000)
	      {								
	        light_controller_period_change(20);							
	      }
    //light_set_blue_lightness(0);
    
        light_blink(light_get_red(), 0, 60000, 500, 50, 1, NULL);
				
			  ir_learn_data_exec(ir_ctx.study.study_data, ir_ctx.study.study_data_len);
        
    }
    #if DLPS_EN 
    if(is_all_light_idle())
     //dlps_ctrl_set(1);
		  egg_ir_study_set(1);
    #endif
    return IR_SUCCESS;

}



ir_study_result_t ir_study_result_get(uint16_t study_id)
{
    ir_study_result_t result = IR_STUDY_FAILED;
    ir_ctx.study.study_result = get_study_result();
    if (ir_ctx.study.study_id == study_id)
    {
        result = ir_ctx.study.study_result;
    }

    return result;
}

uint8_t *ir_study_data_get(uint16_t study_id, uint16_t *pstudy_data_len)
{
    uint8_t *pstudy_data = NULL;
    *pstudy_data_len = 0;
    if (ir_ctx.study.study_id == study_id)
    {
        //ir_study_data_process(ir_ctx.study.study_data, &ir_ctx.study.study_data_len);
        *pstudy_data_len = ir_ctx.study.study_data_len;
        pstudy_data = ir_ctx.study.study_data;
    }

    return pstudy_data;
}

ir_err_code_t ir_add_scene(ir_scene_label_t scene, ir_scene_key_t *pkeys, uint8_t key_size)
{
    ir_err_code_t ret = IR_SUCCESS;
    /* find exists */
    if (!ir_ext_flash_add_scene(scene, pkeys, key_size))
    {
        ret = IR_ERR_STORAGE_FULL;
    }

    printi("ir_add_scene: scene id(%02x-%02x-%02x-%02x-%02x-%02x), label 0x%04x, key size %d",
           scene.id.id[0], scene.id.id[1], scene.id.id[2], scene.id.id[3], scene.id.id[4], scene.id.id[5],
           scene.label, key_size);

    for (uint8_t i = 0; i < key_size; ++i)
    {
        printi("ir_add_scene: key_id %02x-%02x-%02x-%02x-%02x-%02x, key_group 0x%04x, delay_time %ds",
               pkeys[i].key_id.id[0], pkeys[i].key_id.id[1], pkeys[i].key_id.id[2], pkeys[i].key_id.id[3],
               pkeys[i].key_id.id[4], pkeys[i].key_id.id[5], pkeys[i].key_group, pkeys[i].delay_time);
    }

    return ret;
}

bool ir_get_scene_label(ir_scene_id_t scene_id, uint32_t *plabel)
{
    return fatfs_get_scene_label(scene_id, plabel);;
}

void ir_exec_scene_timeout(void *pargs)
{
    T_IO_MSG msg;
    msg.type = INFRARED_MSG;
    msg.subtype = INFRARED_TIMEOUT_EXEC_SCENE;
    msg.u.buf = pargs;
    app_send_msg_to_apptask(&msg);
}
bool ir_send_queue_push(uint8_t *ir_code,uint16_t ir_code_len,uint16_t ack_falg,uint32_t msg_opcode,uint16_t src,uint16_t tid,ir_id_label_t msg_label);
static void ir_exec_scene_timeout_handle(plt_timer_t timer)
{
    ir_exec_scene_entry_t *pentry = (ir_exec_scene_entry_t *)ir_ctx.ir_exec_scene_list.pfirst;
    ir_exec_scene_entry_t *pprev_node = NULL;
    //printi("ir_exec_scene_timeout_handle");
    //dprinti(&pentry->keys[0],27);
    //dprinti(&pentry->pcur_key[0],27);
    while (pentry != NULL)
    {
        if (pentry->delay_timer == timer)
        {
//            ir_code_t *pcode = plt_malloc(sizeof(ir_code_t) + 1400, RAM_TYPE_DATA_ON);
//            if (NULL != pcode)
//            {
//                /* execute key */
//                ir_id_t ir_id = ir_key_id_to_ir_id(pentry->pcur_key->key_id);
//                if(ir_get_by_id(ir_id, pcode))
//                ir_exec(pcode->ir_code, pcode->ir_code_len);
//                printi("ir_exec_scene_timeout_handle: exec key id(%02x-%02x-%02x-%02x-%02x-%02x), ir id(%02x-%02x-%02x-%02x-%02x-%02x)",
//                       pentry->pcur_key->key_id.id[0], pentry->pcur_key->key_id.id[1], pentry->pcur_key->key_id.id[2],
//                       pentry->pcur_key->key_id.id[3], pentry->pcur_key->key_id.id[4], pentry->pcur_key->key_id.id[5],
//                       ir_id.id[0], ir_id.id[1], ir_id.id[2], ir_id.id[3], ir_id.id[4], ir_id.id[5]);
//                plt_free(pcode, RAM_TYPE_DATA_ON);
//            }
            ir_code_t *pcode =NULL;
            ir_id_t ir_id = ir_key_id_to_ir_id(pentry->pcur_key->key_id);
            if( fatfs_get_ir_code_by_id(ir_id,&pcode)==FR_OK)
            {
                if(ir_send_queue_push(pcode->ir_code+4,pcode->ir_code_len,0,0,0,0,pcode->ir_id_label)==0)
                {
                    printi("ir_send_queue_push = NULL,ir exec");
                    ir_exec(pcode->ir_code+4, pcode->ir_code_len,NULL);
                    printi("ir_exec_scene_timeout_handle: exec key id(%02x-%02x-%02x-%02x-%02x-%02x), ir id(%02x-%02x-%02x-%02x-%02x-%02x)",
                    pentry->pcur_key->key_id.id[0], pentry->pcur_key->key_id.id[1], pentry->pcur_key->key_id.id[2],
                    pentry->pcur_key->key_id.id[3], pentry->pcur_key->key_id.id[4], pentry->pcur_key->key_id.id[5],
                    ir_id.id[0], ir_id.id[1], ir_id.id[2], ir_id.id[3], ir_id.id[4], ir_id.id[5]);
                } 
                
            
            }
            if (NULL != pcode)
            {
                plt_free(pcode, RAM_TYPE_DATA_ON);
            }
            
            /* find next key */
            uint8_t start_idx = 0;
            for (uint8_t i = 0; i < IR_SCENE_KEY_MAX_SIZE; i++)
            {
                if (pentry->pcur_key == &pentry->keys[i])
                {
                    start_idx = i + 1;
                    printi("start_idx =%d",start_idx);
                    //dprinti(pentry->pcur_key,9);
                    break;
                }
            }

            uint32_t delay_time = 0;
            if (start_idx < IR_SCENE_KEY_MAX_SIZE)
            {
                for (uint8_t i = start_idx; i < IR_SCENE_KEY_MAX_SIZE; i++)
                {
                    delay_time += pentry->keys[i].delay_time;
                    if (mesh_sub_addr_check(pentry->keys[i].key_group) ||
                        MESH_IS_BROADCAST_ADDR(pentry->keys[i].key_group))
                    {
                        pentry->pcur_key = &pentry->keys[i];
                        dprinti(pentry->pcur_key,9);
                        delay_time *= 1000;
                        if (delay_time == 0)
                        {
                           delay_time = 100;
													 printi("delay_time =100");
                        }
                        plt_timer_change_period(pentry->delay_timer, delay_time, 0);
                        printi("ir_exec_scene_timeout_handle: scene(%02x-%02x-%02x-%02x-%02x-%02x) switch to key(%02x-%02x-%02x-%02x-%02x-%02x), delay time %d",
                               pentry->scene.id.id[0], pentry->scene.id.id[1], pentry->scene.id.id[2],
                               pentry->scene.id.id[3], pentry->scene.id.id[4], pentry->scene.id.id[5],
                               pentry->keys[i].key_id.id[0], pentry->keys[i].key_id.id[1], pentry->keys[i].key_id.id[2],
                               pentry->keys[i].key_id.id[3], pentry->keys[i].key_id.id[4], pentry->keys[i].key_id.id[5],
                               delay_time);
                        return ;
                    }
                }
            }

            /* scene execute finish */
            printi("ir_exec_scene_timeout_handle: scene(%02x-%02x-%02x-%02x-%02x-%02x) execute finished",
                   pentry->scene.id.id[0], pentry->scene.id.id[1], pentry->scene.id.id[2],
                   pentry->scene.id.id[3], pentry->scene.id.id[4], pentry->scene.id.id[5]);
            plt_list_delete(&ir_ctx.ir_exec_scene_list, pprev_node, pentry);
            plt_timer_delete(pentry->delay_timer, 0);
            plt_free(pentry, RAM_TYPE_DATA_ON);
            ir_ctx.ir_exec_scene_list.pfirst = NULL;
            //dprinti(&pentry->keys[0],27);
            break;
        }
        pprev_node = pentry;
        pentry = pentry->pnext;
    }
}

ir_err_code_t ir_exec_scene(ir_scene_id_t scene_id, uint32_t *plabel)
{
    ir_err_code_t ret = IR_SUCCESS;
    ir_scene_key_t keys[IR_SCENE_KEY_MAX_SIZE];
    uint8_t key_size = 0;
    uint32_t label = ir_ext_flash_get_scene_keys(scene_id, keys, &key_size);
    printi("key_size = %d",key_size);
    if (key_size == 0)
    {
        printe("ir_exec_scene: scene(%02x-%02x-%02x-%02x-%02x-%02x) not exist",
               scene_id.id[0], scene_id.id[1], scene_id.id[2], scene_id.id[3], scene_id.id[4], scene_id.id[5]);
        ret = IR_ERR_SCENE_NOT_EXIST;
    }
    else
    {
        ir_exec_scene_entry_t *pentry = plt_zalloc(sizeof(ir_exec_scene_entry_t), RAM_TYPE_DATA_ON);
       
        //printi("scene malloc");
        //dprinti(&pentry->keys[0],27);
        if (NULL == pentry)
        {
            printe("ir_exec_scene: failed, out of memory");
            return IR_ERR_UNKNOWN_ERR;
        }
        pentry->delay_timer = plt_timer_create("es", 1000, false, 0, ir_exec_scene_timeout);
        if (NULL == pentry->delay_timer)
        {
            plt_free(pentry, RAM_TYPE_DATA_ON);
            printe("ir_exec_scene: create timer failed");
            return IR_ERR_UNKNOWN_ERR;
        }

        pentry->scene.id = scene_id;
        pentry->scene.label = label;
        pentry->pcur_key = NULL;
        for (uint8_t i = 0; i < key_size; i++)
        {
            pentry->keys[i] = keys[i];
            //printi("keys[i] i=%d",i);
            //dprinti(&pentry->keys[i],9);
        }

        if (NULL != plabel)
        {
            *plabel = label;
        }

        /* start timer */
        uint32_t delay_time = 0;
        for (uint8_t i = 0; i < key_size; i++)
        {
            delay_time += pentry->keys[i].delay_time;
            if (mesh_sub_addr_check(pentry->keys[i].key_group) ||
                MESH_IS_BROADCAST_ADDR(pentry->keys[i].key_group))
            {
                delay_time *= 1000;
                if (delay_time == 0)
                {
                    delay_time = 10;
                }
                pentry->pcur_key = &pentry->keys[i];
                
                plt_timer_change_period(pentry->delay_timer, delay_time, 0);
                plt_list_push(&ir_ctx.ir_exec_scene_list, pentry);
                //dprinti(&pentry->keys[0],27);
                //printi("pentry->keys[i] i=%d",i);
                printi("ir_exec_scene_timeout_handle: scene(%02x-%02x-%02x-%02x-%02x-%02x) switch to key(%02x-%02x-%02x-%02x-%02x-%02x), delay time %d",
                       pentry->scene.id.id[0], pentry->scene.id.id[1], pentry->scene.id.id[2],
                       pentry->scene.id.id[3], pentry->scene.id.id[4], pentry->scene.id.id[5],
                       pentry->keys[i].key_id.id[0], pentry->keys[i].key_id.id[1], pentry->keys[i].key_id.id[2],
                       pentry->keys[i].key_id.id[3], pentry->keys[i].key_id.id[4],
                       pentry->keys[i].key_id.id[5], delay_time);
                return IR_SUCCESS;
            }
        }

        /* scene execute finish */
        printw("ir_exec_scene: scene(%02x-%02x-%02x-%02x-%02x-%02x) has no key to execute",
               pentry->scene.id.id[0], pentry->scene.id.id[1], pentry->scene.id.id[2], pentry->scene.id.id[3],
               pentry->scene.id.id[4], pentry->scene.id.id[5]);
        plt_timer_delete(pentry->delay_timer, 0);
        plt_free(pentry, RAM_TYPE_DATA_ON);
        ret = IR_ERR_UNKNOWN_ERR;
    }
    return ret;
}

static void ir_del_scene(ir_scene_id_t scene_id)
{
    ir_exec_scene_entry_t *pentry = (ir_exec_scene_entry_t *)ir_ctx.ir_exec_scene_list.pfirst;
    ir_exec_scene_entry_t *pprev_node = NULL;
    while (pentry != NULL)
    {
        if (0 == memcmp(scene_id.id, pentry->scene.id.id, 6))
        {
            printi("ir_del_scene: scene(%02x-%02x-%02x-%02x-%02x-%02x)",
                   scene_id.id[0], scene_id.id[1], scene_id.id[2], scene_id.id[3], scene_id.id[4], scene_id.id[5]);
            plt_list_delete(&ir_ctx.ir_exec_scene_list, pprev_node, pentry);
            plt_timer_delete(pentry->delay_timer, 0);
            plt_free(pentry, RAM_TYPE_DATA_ON);
            break;
        }
        pprev_node = pentry;
        pentry = pentry->pnext;
    }
}

void ir_del_scenes(ir_scene_id_t *pscene_ids, uint8_t scene_id_cnt)
{
    for (uint8_t i = 0; i < scene_id_cnt; ++i)
    {
        ir_del_scene(pscene_ids[i]);
    }

    ir_ext_flash_delete_scene(pscene_ids, scene_id_cnt);
}

void ir_del_all_scene(void)
{
    ir_exec_scene_entry_t *pentry = NULL;
    while ((pentry = plt_list_pop(&ir_ctx.ir_exec_scene_list)) != NULL)
    {
        printi("ir_del_scene: scene(%02x-%02x-%02x-%02x-%02x-%02x)",
               pentry->scene.id.id[0], pentry->scene.id.id[1], pentry->scene.id.id[2],
               pentry->scene.id.id[3], pentry->scene.id.id[4], pentry->scene.id.id[5]);
        plt_timer_delete(pentry->delay_timer, 0);
        plt_free(pentry, RAM_TYPE_DATA_ON);
    }

    ir_ext_flash_clear_scene();
}

void ir_get_scene_labels(ir_scene_id_t *pscene_ids, uint8_t scene_id_cnt, uint32_t *plabels)
{
    ir_ext_flash_get_scene_labels(pscene_ids, scene_id_cnt, plabels);
}

void ir_get_all_scene(ir_scene_label_t *pscenes, uint8_t *pscene_cnt)
{
    ir_ext_flash_get_all_scene(pscenes, pscene_cnt);
}
extern uint32_t *LoopQueue;
void driver_init(void);
void ir_study_timeout_process(void)
{
     printi("ir_study_timeout_process");
     aico_bsp_irl_auto_stop();
     ir_study_data_process(ir_ctx.study.study_data, &ir_ctx.study.study_data_len);
     aico_bsp_irl_enable(0);
     ir_study_result_notify(MESH_BEARER_ALL, ir_ctx.study.study_id, IR_STUDY_TIMEOUT);
     driver_init();
     btn_ir_learn_state = BTN_IR_LEARN_STATE_IDLE;
}


void ir_study_msg_timeout(void *pargs)
{
    T_IO_MSG msg;
    msg.type = INFRARED_IR_STUDY_TIMEOUT_MSG;
    msg.subtype = 0;
    msg.u.buf = pargs;
    app_send_msg_to_apptask(&msg);
}

void dfu_server_timer_stop(void);
void light_breath_restore(void);
void ir_timeout_handle(uint16_t type, plt_timer_t timer)
{
    switch (type)
    {
    case INFRARED_TIMEOUT_PERIODIC_IR:
        ir_exec_periodic_timeout_handle(timer);
        break;
    case INFRARED_TIMEOUT_STUDY:
        ir_learn_check_time_stop();
        DBG_DIRECT("ir_study_timeout: study stop, id %d, data len %d", ir_ctx.study.study_id,
               ir_ctx.study.study_data_len);       
       light_cwrgb_turn_off();
       if(get_power_light_charging()==true)
       {
           light_breath_red_restore(); 
       }
       else
       {
           light_breath_blue_restore();
    
       }        
        //light_breath_restore();
        
        plt_timer_stop(ir_ctx.study.study_tmr, 0);
        ir_ctx.study.studying = false;
        ir_study_msg_timeout(0); 
        #if DLPS_EN  
        //if( is_all_light_idle())    
        //dlps_ctrl_set(1);
		egg_ir_study_set(1);
        #endif
        break;
    case INFRARED_TIMEOUT_EXEC_SCENE:
        ir_exec_scene_timeout_handle(timer);
        break;
    case INFRARED_TIMEOUT_OTA:
        mesh_service_adv_start();
        dfu_server_timer_stop();
        break;
    default:
        break;
    }
}

ir_id_t ir_key_id_to_ir_id(ir_key_id_t key_id)
{
    ir_id_t ir_id;
    memset(ir_id.id, 0, 6);
    if ((key_id.id[5] & 0x80) == 0)
    {
        printi("exec ir id key_id.id[5] =0x%02x",key_id.id[5]);
        memcpy(ir_id.id, key_id.id, 6);
    }
    else
    {
       
        uint8_t temper_flag = key_id.id[1];
        int8_t temper = 0;
        if (temper_flag == 0)
        {
            temper = ir_get_eco_temper();
            printi("eco_temper = 0x%04x",temper );
        }
        else
        {
            temper = temper_flag + 15;
            printi("temper = 0x%04x",temper);
        }
        uint8_t temper_param = ir_get_temper_param();
        float cur_temper = ir_temper_get() / 10;
        if (temper_param & 0x80)
        {
            cur_temper -= (temper_param & ~0x80);
          
        }
        else
        {
            cur_temper += temper_param;
        }
        printi("cur_temper = 0x%04x,temper == 0x%04x",cur_temper,temper);
        if (temper >= cur_temper)
        {
            /* hot */
            ir_id.id[5] = key_id.id[5];
            ir_id.id[5] &= 0x7f;
            ir_id.id[4] = key_id.id[4];
            //uint16_t temper_set =0x1000|((temper - 15)<<8);
           // ir_id.id[1] = temper - 15;
           // ir_id.id[1] = temper_set>>8;
            //ir_id.id[0] = temper_set&0xff;
            ir_id.id[1] =0x10|(temper - 15);
        }
        else
        {
            /* cold */
            ir_id.id[5] = key_id.id[5];
            ir_id.id[5] &= 0x7f;
            ir_id.id[4] = key_id.id[4];
            ir_id.id[1] = temper - 15;
        }
        dprinti(&ir_id.id[1],5);
    }

    return ir_id;
}

ir_id_t ir_rcu_id_to_ir_id(ir_key_id_t key_id)
{
    ir_id_t id;

    return id;
}

bool ir_write_sn(uint8_t sn[15])
{
    flash_erase_locked(FLASH_ERASE_SECTOR, BKP_DATA1_ADDR);
    flash_write_locked(BKP_DATA1_ADDR, 15, sn);

    return true;
}
bool mp_ftl_data_load_sn(uint8_t sn[15]);
//void ir_read_sn(uint8_t sn[15])
//{
//    //flash_read_locked(BKP_DATA1_ADDR, 15, sn);
//   // mp_ftl_data_load_sn(sn);
//    
//}
void ir_read_sn(uint8_t sn[15])
{
     uint8_t default_sn_prefix[7] = {0x41, 0x43, 0x45, 0x47, 0x01, 0x15, 0x06};
     memset(sn, 0, 15);
     mp_ftl_data_load_sn(sn);
     if (mp_sn_verify(sn, default_sn_prefix))
     {
        mp_ftl_data_save_sn(sn);
     }
}



void ir_set_user_id(uint8_t user_id[6])
{
    memcpy(ir_ctx.user_id, user_id, 6);
}

void ir_get_user_id(uint8_t user_id[6])
{
    memcpy(user_id, ir_ctx.user_id, 6);
}

void ir_set_region_id(uint8_t region_id[6])
{
    memcpy(ir_ctx.region_id, region_id, 6);
}

void ir_get_region_id(uint8_t region_id[6])
{
    memcpy(region_id, ir_ctx.region_id, 6);
}

void ir_set_common_flag(uint8_t common_flag[6])
{
    memcpy(ir_ctx.common_flag, common_flag, 6);
}

void ir_get_common_flag(uint8_t common_flag[6])
{
    memcpy(common_flag, ir_ctx.common_flag, 6);
}

void ir_set_blue_light(ir_blue_light_t light)
{
    ir_ctx.blue_light = light;
}

ir_blue_light_t ir_get_blue_light(void)
{
    return ir_ctx.blue_light;
}

void ir_set_red_light(ir_red_light_t light)
{
    ir_ctx.red_light = light;
}

ir_red_light_t ir_get_red_light(void)
{
    return ir_ctx.red_light;
}

void ir_set_beeper(ir_beeper_t beeper)
{
    ir_ctx.beeper = beeper;
}

ir_beeper_t ir_get_beeper(void)
{
    return ir_ctx.beeper;
}

void ir_ota_timeout(void *pargs)
{
    T_IO_MSG msg;
    msg.type = INFRARED_MSG;
    msg.subtype = INFRARED_TIMEOUT_OTA;
    msg.u.buf = pargs;
    app_send_msg_to_apptask(&msg);
}

void dfu_server_timer_start(void);
void ir_ota_start(uint32_t time)
{
    if (NULL == ir_ctx.ota_timer)
    {
        //ir_ctx.ota_timer = plt_timer_create("ota_t", time * 3600000, false, 0, ir_ota_timeout);
       // plt_timer_start(ir_ctx.ota_timer, time * 3600000);
        ir_ctx.ota_timer = plt_timer_create("ota_t", time * 60000, false, 0, ir_ota_timeout);
        plt_timer_start(ir_ctx.ota_timer,0);
        mesh_service_adv_stop();
        dfu_server_timer_start();
        transport_ctx_disconnect_all(1000);
    }
    else
    {
        //plt_timer_change_period(ir_ctx.ota_timer, time * 3600000, 0);
         plt_timer_start(ir_ctx.ota_timer, time * 60000);
    }
}

void ir_ota_stop(void)
{
    if (NULL != ir_ctx.ota_timer)
    {
        plt_timer_delete(ir_ctx.ota_timer, 0);
        ir_ctx.ota_timer = NULL;
    }
}



