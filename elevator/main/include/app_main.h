#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "app_speech.h"



#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
 */
//#define SUPPORT_HEARTBEAT
//#define SPP_DEBUG_MODE

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

    SPP_IDX_NB,
};



#define GPIO_LED_RED    21
#define GPIO_LED_WHITE  22
#define GPIO_BUTTON     15

typedef enum
{
	BG = 0,
	KW01,
	KW02,
	KW03,
	KW04,
	KW05,
	KW06,
	KW07,
    KW08,
    KW09,
    KW10,
    KW11,
    KW12,
    KW13,
    KW14,
    KW15,
} en_fsm_state;

extern en_fsm_state g_state;

#endif
