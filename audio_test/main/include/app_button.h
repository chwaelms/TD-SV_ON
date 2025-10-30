#ifndef BUTTON_APP_H__
#define BUTTON_APP_H__

#include <stdint.h>

#include "esp_event.h"
#include "esp_adc/adc_oneshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//=========================== define ===========================
#define BUTTON_APP_RUNTIME_LOG         (false)

#define GPIO_BUTTON         (1)

#define BUTTON_APP_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#define BUTTON_APP_ADC_ATTEN    ADC_ATTEN_DB_11

#define LONG_PRESS_INTERVAL      (1 * 1000000) // 1sec

#define BUTTON_NUM_MAX      (4)
/*     4개의 Button Array가 GPIO_BUTTON 핀에 연결됨.
bit width default & atten_DB_11 설정에서 V_max = 4095

Button Layout:
    [A - 27K]       [X - 3.3K]

    [B - 15K]       [Y - 1.3K]

Circuit:
                        +---- GPIO_BUTTON ADC pin
                        |
    VCC -----R(10K)-----+---- A ----R(27K)---- GND     // 3000 (aprox.)
    (MAX :              |
        4095)           +---- B ----R(15K)---- GND     // 2500
                        |
                        +---- X ----R(3.3K)--- GND     // 1000
                        |
                        +---- Y ----R(1.3K)--- GND     // 470
*/

// Declare an event base
ESP_EVENT_DECLARE_BASE(BUTTON_EVENTS);      // declaration of the button events family

//=========================== typedef ==========================
enum{
    ADC_DIV_MAX = 4000,
    ADC_DIV_AB  = 2700,
    ADC_DIV_BX  = 1700,
    ADC_DIV_XY  = 700,
    ADC_DIV_MIN = 0,
};

typedef enum{
    BTN_TYPE_NONE   = 0,
    BTN_TYPE_A      = 10,
    BTN_TYPE_B      = 20,
    BTN_TYPE_X      = 30,
    BTN_TYPE_Y      = 40,
} button_type_t;

enum {
    B_PRESSED   = 1,
    B_RELEASED  = 2,
    B_LONG_PRESS = 3
};

typedef enum {
    BUTTON_EVENT_A_PRESSED      = BTN_TYPE_A + B_PRESSED,
    BUTTON_EVENT_A_RELEASED     = BTN_TYPE_A + B_RELEASED,
    BUTTON_EVENT_A_LONG_PRESS   = BTN_TYPE_A + B_LONG_PRESS,

    BUTTON_EVENT_B_PRESSED      = BTN_TYPE_B + B_PRESSED,
    BUTTON_EVENT_B_RELEASED     = BTN_TYPE_B + B_RELEASED,
    BUTTON_EVENT_B_LONG_PRESS   = BTN_TYPE_B + B_LONG_PRESS,

    BUTTON_EVENT_X_PRESSED      = BTN_TYPE_X + B_PRESSED,
    BUTTON_EVENT_X_RELEASED     = BTN_TYPE_X + B_RELEASED,
    BUTTON_EVENT_X_LONG_PRESS   = BTN_TYPE_X + B_LONG_PRESS,

    BUTTON_EVENT_Y_PRESSED      = BTN_TYPE_Y + B_PRESSED,
    BUTTON_EVENT_Y_RELEASED     = BTN_TYPE_Y + B_RELEASED,
    BUTTON_EVENT_Y_LONG_PRESS   = BTN_TYPE_Y + B_LONG_PRESS,
} button_event_t;



typedef struct{
    button_type_t type;
    uint16_t max;
    uint16_t min;
    bool pressed;
} button_conf_t;

//=========================== variables ========================

typedef struct{
    adc_unit_t      adc_unit;
    adc_channel_t   adc_channel;

    adc_oneshot_unit_handle_t adc_unit_handle;

    button_conf_t button_info[BUTTON_NUM_MAX];
} buttonApp_var_t;

//=========================== prototypes =======================
// MUST!!! - Default event loop must be created before calling this init function.
// #include "esp_event.h"
// esp_event_loop_create_default();
void init_button(void);

#endif // BUTTON_APP_H__