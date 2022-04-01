/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file test_tsl2561_smbus.c
 * @brief Test application for the TSL2561 Light-to-Digital Converter.
 * This program performs a series of I2C and SMBus transactions on a connected
 * TSL2561 device, and reports back the results for manual verification.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "emc1072.h"
#include "smbus.h"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

// The following values are bitwise ORed with register addresses to create a command value
#define SMB_BLOCK           0x10  // Transaction to use Block Write/Read protocol
#define SMB_WORD            0x20  // Transaction to use Word Write/Read protocol
#define SMB_CLEAR           0x40  // Clear any pending interrupt (self-clearing)
#define SMB_COMMAND         0x80  // Select command register

const char * TEST = "test";

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

void test_smbus_task(void * pvParameter)
{
    i2c_master_init();

    i2c_port_t i2c_num = I2C_MASTER_NUM;
    i2c_address_t address = EMC1072_SMBUS_ADDRESS;

    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, EMC1072_SMBUS_TIMEOUT / portTICK_RATE_MS);

    // SMBus Quick Commands:
//    ESP_LOGI(TEST, "smbus_quick:");
//    ESP_ERROR_CHECK(smbus_quick(smbus_info, true));
//    ESP_ERROR_CHECK(smbus_quick(smbus_info, false));

    // Send Byte
    ESP_LOGI(TEST, "smbus_send_byte:");
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, SMB_COMMAND));
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, CHIP_WAKE_UP));  // power up

    // Receive Byte
    ESP_LOGI(TEST, "smbus_receive_byte:");
    uint8_t status = 0;
    ESP_ERROR_CHECK(smbus_send_byte(smbus_info, SMB_COMMAND));
    ESP_ERROR_CHECK(smbus_receive_byte(smbus_info, &status));
    ESP_LOGI(TEST, "status 0x%02x (expect 0x03)", status);

    // Write Byte
    ESP_LOGI(TEST, "smbus_write_byte:");
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONFIGURATION | SMB_COMMAND, CHIP_STANDBY));  // power down
    ESP_ERROR_CHECK(smbus_receive_byte(smbus_info, &status));
    ESP_LOGI(TEST, "status 0x%02x (expect 0x00)", status);
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONFIGURATION | SMB_COMMAND, CHIP_WAKE_UP));  // power up

    // Read Byte
    ESP_LOGI(TEST, "smbus_read_byte:");
    uint8_t temp_r = 0;
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONVERSION_RATE | SMB_COMMAND, RATE_1X1));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_CONVERSION_RATE | SMB_COMMAND, &temp_r));
    ESP_LOGI(TEST, "temp_range 0x%01x (expect 0x00)", temp_r);

    // tune settings
    uint8_t config = 0;
    vTaskDelay(500 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONFIGURATION | SMB_COMMAND, 
                    DEFAULT_CONFIGURATION | TEMP_RANGE_EXTEND));  
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_CONFIGURATION | SMB_COMMAND, &config));
    ESP_LOGI(TEST, "config 0x%01x (expect 0x0d)", config);
   
    // Read Byte
    ESP_LOGI(TEST, "smbus_read_byte:");
    uint8_t temp_range = 0;
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONVERSION_RATE | SMB_COMMAND, RATE_1X1));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_CONVERSION_RATE | SMB_COMMAND, &temp_range));
    ESP_LOGI(TEST, "temp_range 0x%01x (expect 0x01)", temp_range);

    uint8_t data_i[2] = { 0 };
    uint8_t data_e[2] = { 0 };
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_DIODEDATA_INT_L | SMB_COMMAND | SMB_WORD, &data_i[0]));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_DIODEDATA_INT_H | SMB_COMMAND | SMB_WORD, &data_i[1]));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_DIODEDATA_EXT_L | SMB_COMMAND | SMB_WORD, &data_e[0]));
    ESP_ERROR_CHECK(smbus_read_byte(smbus_info, REG_DIODEDATA_EXT_H | SMB_COMMAND | SMB_WORD, &data_e[1]));
    ESP_LOGI(TEST, "interior data: low block 0x%01x, high block 0x%01x", data_i[0] & 0xf, data_i[1] & 0xf);
    ESP_LOGI(TEST, "interior data: word 0x%02x", (data_i[1] << 8) & data_i[0]);
    ESP_LOGI(TEST, "exterior data: low block 0x%01x, high block 0x%01x", data_e[0] & 0xf, data_e[1] & 0xf);
    ESP_LOGI(TEST, "exterior data: word 0x%02x", (data_e[1] << 8) & data_e[0]);
    
    ESP_ERROR_CHECK(smbus_write_byte(smbus_info, REG_CONFIGURATION | SMB_COMMAND, CHIP_STANDBY));  // power down

    ESP_LOGW(TEST, "Test complete.");

    //while(1)
    //    ;
    vTaskDelete(NULL);
}
