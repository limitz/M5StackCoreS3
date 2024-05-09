/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_camera.h"
#include "model_zoo/human_face_detect_msr01.hpp"

static const char *TAG = "example";

extern "C" void app_main(void)
{
    HumanFaceDetectMSR01 model(0.3f, 0.5f, 1, 0.5f);

    bsp_i2c_init();
    bsp_display_start();
    bsp_display_backlight_on(); // Set display brightness to 100%

    // Initialize the camera
    camera_config_t camera_config = BSP_CAMERA_DEFAULT_CONFIG;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    ESP_LOGI(TAG, "Camera Init done");

    // Create LVGL canvas for camera image
    bsp_display_lock(0);
    lv_obj_t *camera_canvas = lv_canvas_create(lv_scr_act());
    assert(camera_canvas);
    lv_obj_center(camera_canvas);
    bsp_display_unlock();

    camera_fb_t *pic;
    lv_point_t pts[] = {
        { 10, 10 },
        { 200, 20 },
        { 180, 170 },
        { 30, 150 },
        { 10, 10 }
    };

    while (1) {
        pic = esp_camera_fb_get();
        if (pic) {
            int i=0;
            std::list<dl::detect::result_t>& results = model.infer((uint16_t*)pic->buf, {(int)pic->height, (int)pic->width, 3 });
            for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); 
                    prediction != results.end(); 
                    prediction++, i++)
            {
                printf("[%d] score: %f, box: [%d, %d, %d, %d]\n", i, 
                        prediction->score, 
                        prediction->box[0], prediction->box[1], prediction->box[2], prediction->box[3]);
                pts[0].x = prediction->box[0];
                pts[0].y = prediction->box[1];
                pts[1].x = prediction->box[2];
                pts[1].y = prediction->box[1];
                pts[2].x = prediction->box[2];
                pts[2].y = prediction->box[3];
                pts[3].x = prediction->box[0];
                pts[3].y = prediction->box[3];
                pts[4].x = prediction->box[0];
                pts[4].y = prediction->box[1];
            }
            if (0==i)
            {
                memset(pts, 0, sizeof(pts));
            }
            bsp_display_lock(0);
            lv_canvas_set_buffer(camera_canvas, pic->buf, pic->width, pic->height, LV_IMG_CF_TRUE_COLOR);
            lv_draw_line_dsc_t dsc;
            lv_draw_line_dsc_init(&dsc);
            dsc.color = lv_color_make(240,0,60);
            dsc.width = 2;
            lv_canvas_draw_line(camera_canvas, pts, 5, &dsc);
            bsp_display_unlock();
            esp_camera_fb_return(pic);
        } else {
            ESP_LOGE(TAG, "Get frame failed");
        }
        vTaskDelay(1);
    }
}
