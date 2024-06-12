/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c_master.h"

#include "i2c_sgp30.h"


#define SCL_IO_PIN (CONFIG_I2C_MASTER_SCL) // default is 4
#define SDA_IO_PIN (CONFIG_I2C_MASTER_SDA) // default is 5

#define PORT_NUMBER -1

static i2c_master_dev_handle_t g_i2c_dev = NULL;

static int s_esp_i2c_write(uint8_t addr, const uint8_t *data, size_t data_len){
    (void)addr;
    if (ESP_OK != i2c_master_transmit(g_i2c_dev, data, data_len, -1)){
        return 1;
    } // 无限等待？如果不是无限等待的话，data不能使用栈上的变量?

    return 0;
}

static int s_esp_i2c_read(uint8_t addr, uint8_t * data, size_t buf_len){
    (void)addr;
    if (ESP_OK != i2c_master_receive(g_i2c_dev, data, buf_len, -1)){
        return 1;
    }

    return 0;
}

static void s_esp_sleep_msec(uint32_t mseconds){
    vTaskDelay(mseconds / portTICK_PERIOD_MS);   
}

static esp_err_t s_app_i2c_bus_init(i2c_master_dev_handle_t * i2c_handle){
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = 100000, // or fast mode 400000,SGP30支持I2c fast mode模式
        .device_address = 0x58,
    };

    i2c_master_dev_handle_t i2c_dev = NULL;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &i2c_dev));

    
    *i2c_handle = i2c_dev;

    return ESP_OK;

}

void s_app_i2c_bus_deinit(void){
    i2c_master_bus_rm_device(g_i2c_dev);
}

void app_main(void)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    if (ESP_OK != s_app_i2c_bus_init(&g_i2c_dev)){
        printf("i2c bus init failed\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    }

    struct i2c_sgp30_t sgp30_config = {
        .i2c_write = s_esp_i2c_write,
        .i2c_read = s_esp_i2c_read,
        .msleep = s_esp_sleep_msec,
        .i2c_addr = 0x58,
    };
    i2c_sgp30_handle_t sgp30_handle = &sgp30_config;

    if (SGP30_OK != i2c_sgp30_init(sgp30_handle)){
        printf("sgp30 init failed\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    }

    uint8_t version[2];
    if (SGP30_OK != i2c_sgp30_get_feature_set_version(sgp30_handle, version)){
        printf("sgp30 get fearture set failed\n");
    }
    printf("sgp30 feature set: 0x%02x 0x%02x\n", version[0], version[1]);

  //  uint16_t test_data;
  //  i2c_sgp30_measure_test(sgp30_handle, &test_data);
   // printf("test_data: 0x%x\n", test_data);

    uint16_t base_co2eq, base_tvoc;
    uint16_t h2_sig, eth_sig;
    i2c_sgp30_get_baseline(sgp30_handle ,&base_co2eq, &base_tvoc); // should wait 20s to get after init air quality.
    printf("tvoc_base: %d, co2_base: %dppm\n", base_tvoc, base_co2eq);

    uint16_t co2eq, tvoc;
    while(1){
        i2c_sgp30_measure_air_quality(sgp30_handle, &co2eq, &tvoc);
        printf("tvoc: %dppb, co2: %dppm\n", tvoc, co2eq);

        i2c_sgp30_measure_raw_signals(sgp30_handle, &h2_sig, &eth_sig);
        printf("RAW data: H2_signal: %d, Ethanol_signal: %d\n", h2_sig, eth_sig);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
