#include "app_button.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"

#include "esp_adc/adc_oneshot.h"

//=========================== constants ===========================
static const char* TASK_TAG = "ButtonAppTask";

//=========================== variables ===========================
buttonApp_var_t buttonApp_var = {
    .adc_unit       = -1,
    .adc_channel    = -1,
    .button_info    ={
        { // info A --------------------
            .type = BTN_TYPE_A,
            .max  = ADC_DIV_MAX,
            .min  = ADC_DIV_AB,
            .pressed = false,
        },
        { // info B --------------------
            .type = BTN_TYPE_B,
            .max  = ADC_DIV_AB,
            .min  = ADC_DIV_BX,
            .pressed = false,
        },
        { // info X --------------------
            .type = BTN_TYPE_X,
            .max  = ADC_DIV_BX,
            .min  = ADC_DIV_XY,
            .pressed = false,
        },
        { // info Y --------------------
            .type = BTN_TYPE_Y,
            .max  = ADC_DIV_XY,
            .min  = ADC_DIV_MIN,
            .pressed = false,
        }
    }
};

ESP_EVENT_DEFINE_BASE(BUTTON_EVENTS);

//=========================== prototypes ==========================
char _getCharFromBtnId(button_type_t t);

//=========================== tasks ===============================
static void buttonAppTask (void *args)
{
    static int adc_raw;

    int64_t cur_time = esp_timer_get_time();
    int64_t rec_time = esp_timer_get_time();

    ESP_LOGI(TASK_TAG, "Launch Button App");
    while(1){
        // Step 1. Get ADC value
        ESP_ERROR_CHECK(adc_oneshot_read(buttonApp_var.adc_unit_handle,
                                        buttonApp_var.adc_channel, 
                                        &adc_raw));

        // Step 2. Record backup time
        cur_time = esp_timer_get_time();

        // Step 3. Check ADC value for each button configuration
        for(int i = 0; i < BUTTON_NUM_MAX; i++){
            if(
                (buttonApp_var.button_info[i].min <= adc_raw) &&
                (adc_raw <= buttonApp_var.button_info[i].max)
            ){
                if ( ! buttonApp_var.button_info[i].pressed ){
                    // Press Detected
                    ESP_LOGI(TASK_TAG, "[%lld] Button %c is pressed", 
                        cur_time, 
                        _getCharFromBtnId( buttonApp_var.button_info[i].type ) );
                    rec_time = cur_time;
                    ESP_ERROR_CHECK(esp_event_post(BUTTON_EVENTS, 
                                                buttonApp_var.button_info[i].type + B_PRESSED,
                                                 NULL, 0, portMAX_DELAY));
                }
                buttonApp_var.button_info[i].pressed = true;
            }
            else{
                if ( buttonApp_var.button_info[i].pressed ){
                    // Release Detected
                    
                    // Release event is useless in this implementation.
                    // notify(releaseEvent);

                    if ( (cur_time - rec_time) > LONG_PRESS_INTERVAL ){
                        // Long Press Event
                        ESP_LOGI(TASK_TAG, "[%lld] Button %c is pressed long time", 
                                cur_time,
                                 _getCharFromBtnId( buttonApp_var.button_info[i].type ) );
                                 
                        ESP_ERROR_CHECK(esp_event_post(BUTTON_EVENTS, 
                                                buttonApp_var.button_info[i].type + B_LONG_PRESS,
                                                 NULL, 0, portMAX_DELAY));
                    }
                    else{
                        // Release Event
                        ESP_LOGI(TASK_TAG, "[%lld] Button %c is released", 
                                cur_time,
                                _getCharFromBtnId( buttonApp_var.button_info[i].type ) );
                        
                        
                        ESP_ERROR_CHECK(esp_event_post(BUTTON_EVENTS, 
                                                buttonApp_var.button_info[i].type + B_RELEASED,
                                                 NULL, 0, portMAX_DELAY));
                    }
                }

                buttonApp_var.button_info[i].pressed = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//=========================== call backs ===========================

//=========================== public ==============================
void init_button(void)
{
    //get adc unit & adc channel using io number
    adc_oneshot_io_to_channel(GPIO_BUTTON, &(buttonApp_var.adc_unit), &(buttonApp_var.adc_channel));

    //ESP_LOGI("Button App Init", "GPIO %d is connected ADC Unit [%d], Channel [%d]", GPIO_BUTTON, buttonApp_var.adc_unit + 1, buttonApp_var.adc_channel);

    //ADC Unit Init
    adc_oneshot_unit_init_cfg_t init_conf = {
        .unit_id = buttonApp_var.adc_unit
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_conf, &(buttonApp_var.adc_unit_handle)));

    //ADC Channel configuration for initialized unit
    adc_oneshot_chan_cfg_t conf = {
        .bitwidth = BUTTON_APP_ADC_BITWIDTH,
        .atten = BUTTON_APP_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(buttonApp_var.adc_unit_handle,
                                            buttonApp_var.adc_channel,
                                            &conf));

    ESP_LOGI("Button App Init", 
                "Button App & ADC%d.%d Init Complete. RunTime Log [%d]",
                buttonApp_var.adc_unit + 1, buttonApp_var.adc_channel, BUTTON_APP_RUNTIME_LOG);
    
    if (BUTTON_APP_RUNTIME_LOG) esp_log_level_set(TASK_TAG, ESP_LOG_VERBOSE);
    else esp_log_level_set(TASK_TAG, ESP_LOG_WARN);
    
    xTaskCreatePinnedToCore(buttonAppTask, "buttonAppTask", 3 * 1024, NULL,
                            5, NULL, 
                            tskNO_AFFINITY);
}

//=========================== private ==============================
char _getCharFromBtnId(button_type_t t)
{
    switch(t){
        case BTN_TYPE_A:
            return 'A';
        case BTN_TYPE_B:
            return 'B';
        case BTN_TYPE_X:
            return 'X';
        case BTN_TYPE_Y:
            return 'Y';
        default:
            return ' ';
    }
}
