#ifndef LCD_MANAGER_H
#define LCD_MANAGER_H

//=========================== header ==========================
#include <stdint.h>
#include "esp_log.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define BOARD_LCD_MOSI 47
#define BOARD_LCD_MISO -1
#define BOARD_LCD_SCK 21
#define BOARD_LCD_CS 44
#define BOARD_LCD_DC 43
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL 48
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 240
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
#define LCD_HOST SPI3_HOST


//=========================== define ===========================
#define LCD_MANAGER_TAG "lcd_manager"

//=========================== typedef ===========================


//=========================== variables ===========================


//=========================== prototypes ===========================
//lcd 초기화
esp_err_t init_lcd();
//lcd task로 들어오는 프레임을 받는 queue
QueueHandle_t get_lcd_frame_in_queue();
//데이터 전송 목적으로 프레임을 전송할때 사용하는 queue
void set_lcd_frame_out_queue(QueueHandle_t handler);

//=========================== tasks ===========================
//camera버퍼를 lcd에 띄우는 task
void vision_display_task(void *arg);
//부가 정보등을 lcd에 띄우는 task
void info_display_task(void *arg);







#endif