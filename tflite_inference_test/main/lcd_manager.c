//=========================== header ==========================
#include "lcd_manager.h"
#include "esp_camera.h"
#include <string.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "state_controller.h"

// #include "logo_en_240x240_lcd.h"

//=========================== variables ===========================
static esp_lcd_panel_handle_t panel_handle = NULL;

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;

static send_data_t vision_frame;

//=========================== prototypes ==========================
//void _app_lcd_draw_wallpaper();

//=========================== public ==============================
esp_err_t init_lcd()
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BOARD_LCD_BL
    };
    // Initialize the GPIO of backlight
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(LCD_MANAGER_TAG, "Initialize SPI bus");
    spi_bus_config_t bus_conf = {
        .sclk_io_num = BOARD_LCD_SCK,
        .mosi_io_num = BOARD_LCD_MOSI,
        .miso_io_num = BOARD_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_conf, SPI_DMA_CH_AUTO));

    ESP_LOGI(LCD_MANAGER_TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BOARD_LCD_DC,
        .cs_gpio_num = BOARD_LCD_CS,
        .pclk_hz = BOARD_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = BOARD_LCD_CMD_BITS,
        .lcd_param_bits = BOARD_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    //ESP_LOGI(LCD_MANAGER_TAG, "Install ST7789 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BOARD_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    
    ESP_ERROR_CHECK(gpio_set_level(BOARD_LCD_BL, BOARD_LCD_BK_LIGHT_OFF_LEVEL));
    
    // Reset the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    
    // Initialize LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // Turn on the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));// Set inversion for esp32s3eye
    
    // Turn on backlight (Different LCD screens may need different levels)
    ESP_ERROR_CHECK(gpio_set_level(BOARD_LCD_BL, BOARD_LCD_BK_LIGHT_ON_LEVEL));
    
    //_app_lcd_draw_wallpaper();
    vTaskDelay(pdMS_TO_TICKS(200));

    //camera로 부터 전송받을 queue 생성
    xQueueFrameI = xQueueCreate(2, sizeof(camera_fb_t* ));

    // Turn off the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));

    return ESP_OK;
}

QueueHandle_t get_lcd_frame_in_queue()
{
    return xQueueFrameI;
}

void set_lcd_frame_out_queue(QueueHandle_t handler)
{
    xQueueFrameO = handler;
}

//=========================== tasks ===============================
void vision_display_task(void *arg)
{
    // Turn on the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    camera_fb_t *frame = NULL;
    vision_frame.type = VISION_DATA;
    ESP_LOGI(VISION_PROVIDER_TAG, "LCDTask Start!");
    while (true)
    {   
        if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, frame->width, frame->height, (uint16_t *)frame->buf);
            if (xQueueFrameO)
            {
                memcpy(&vision_frame.data.vision_data.frame, &frame, sizeof(frame));
                xQueueSend(xQueueFrameO, &vision_frame, portMAX_DELAY);
                xQueueFrameO = NULL;
            }
            else
            {
                camera_fb_return(frame);
            }
        }
    }
}

void info_display_task(void *arg)
{
}



//=========================== private ==============================
// void _app_lcd_draw_wallpaper()
// {
//     uint16_t *pixels = (uint16_t *)heap_caps_malloc((logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
//     if (NULL == pixels)
//     {
//         ESP_LOGE(LCD_MANAGER_TAG, "Memory for bitmap is not enough");
//         return;
//     }
//     memcpy(pixels, logo_en_240x240_lcd, (logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t));
//     esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, logo_en_240x240_lcd_width, logo_en_240x240_lcd_height, (uint16_t *)pixels);
//     heap_caps_free(pixels);
// }

