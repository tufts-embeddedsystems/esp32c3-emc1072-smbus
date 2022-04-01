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
 * @file emc1072.c
 *
 * Acknowledgements to Kevin Townsend for the Adafruit EMC1072 driver: https://github.com/adafruit/Adafruit_EMC1072
 * Acknowledgements to https://github.com/lexruee/tsl2561 for a working reference.
 */

#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "emc1072.h"

static const char * TAG = "emc1072"; 

static bool _is_init(const emc1072_info_t * emc1072_info)
{
    bool ok = false;
    if (emc1072_info != NULL)
    {
        if (emc1072_info->init)
        {
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "emc1072_info is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "emc1072_info is NULL");
    }
    return ok;
}

static bool _check_device_id(reg_default_t device_type, reg_default_t device_mfr, reg_default_t device_rev)
{
    const char * name = NULL;
    switch (device_type)
    {
        case VALID_DEVICE_ID: name = "1-ACZL-TR"; break;
        default: break;
    }
    if (name)
    {
        ESP_LOGI(TAG, "Device ID is EMC1072-%s", name);
    }
    else
    {
        ESP_LOGW(TAG, "Device is not recognised");
    }

    const char * mfr = NULL;
    switch (device_mfr)
    {
        case VALID_DEVICE_MFR: mfr = "SMSC"; break;
        default: break;
    }
    if (mfr)
    {
        ESP_LOGI(TAG, "Device manufacturer is %s", mfr);
    }
    else
    {
        ESP_LOGW(TAG, "Device manufacturer is not recognised");
    }

    const char * rev = NULL;
    switch (device_rev)
    {
        case VALID_DEVICE_REV: rev = "48"; break;
        default: break;
    }
    if (rev)
    {
        ESP_LOGI(TAG, "Device revision is %s", rev);
    }
    else
    {
        ESP_LOGW(TAG, "Device revision is not recognised");
    }
    return ((name != NULL) & (mfr != NULL) & (rev != NULL));
}

static esp_err_t _wake_up(emc1072_info_t * emc1072_info)
{
    esp_err_t err = ESP_FAIL;
    if (emc1072_info != NULL)
    {
        if (!emc1072_info->awake)
        {
            if ((err = smbus_write_byte(emc1072_info->smbus_info, REG_CONFIGURATION | SMB_COMMAND, CHIP_WAKE_UP)) == ESP_OK)
            {
                emc1072_info->awake = true;
            }
        }
        else
        {
            ESP_LOGW(TAG, "Device already awake");
            err = ESP_OK;  // not an error
        }
    }
    return err;
}

static esp_err_t _power_down(emc1072_info_t * emc1072_info)
{
    esp_err_t err = ESP_FAIL;
    if (emc1072_info != NULL)
    {
        if (emc1072_info->awake)
        {
            if ((err = smbus_write_byte(emc1072_info->smbus_info, REG_CONFIGURATION | SMB_COMMAND, CHIP_STANDBY)) == ESP_OK)
            {
                emc1072_info->awake = false;
            }
        }
        else
        {
            ESP_LOGW(TAG, "Device not awake");
            err = ESP_OK;  // not an error
        }
    }
    return err;
}

// Public API

emc1072_info_t * emc1072_malloc(void)
{
    emc1072_info_t * emc1072_info = malloc(sizeof(*emc1072_info));
    if (emc1072_info != NULL)
    {
        memset(emc1072_info, 0, sizeof(*emc1072_info));
        ESP_LOGD(TAG, "malloc emc1072_info_t %p", emc1072_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc emc1072_info_t failed");
    }
    return emc1072_info;
}

void emc1072_free(emc1072_info_t ** emc1072_info)
{
    if (emc1072_info != NULL && (*emc1072_info != NULL))
    {
        ESP_LOGD(TAG, "free emc1072_info_t %p", *emc1072_info);
        free(*emc1072_info);
        *emc1072_info = NULL;
    }
    else
    {
        ESP_LOGE(TAG, "free emc1072_info_t failed");
    }
}

esp_err_t emc1072_init(emc1072_info_t * emc1072_info, smbus_info_t * smbus_info)
{
    esp_err_t err = ESP_FAIL;
    if ((emc1072_info != NULL) & (smbus_info != NULL))
    {
        emc1072_info->smbus_info = smbus_info;
        emc1072_info->awake = false;
        emc1072_info->conversion_rate = DEFAULT_CONVERSION_RATE;
        emc1072_info->temp_range = TEMP_RANGE_DEFAULT;
        emc1072_info->device_type = INVALID_DEVICE;
        emc1072_info->mask_all = MASK_DISABLE;
        emc1072_info->init = true;

        // read the ID register and confirm that it is as expected for this device
        reg_default_t device_type = INVALID_DEVICE;
        reg_default_t device_rev = INVALID_DEVICE;
        reg_default_t device_mfr = INVALID_DEVICE;
        err = emc1072_device_id(emc1072_info, &device_type, &device_mfr, &device_rev);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Detected device ID 0x%02x, manufacturer 0x%02x, revision 0x%02x on I2C address 0x%02x", 
                     device_type, device_mfr, device_rev, smbus_info->address);
            if (_check_device_id(device_type, device_mfr, device_rev))
            {
                emc1072_info->device_type = device_type;
                emc1072_info->device_mfr = device_mfr;
                emc1072_info->device_rev = device_rev;

                err = _tune_all_settings(emc1072_info);
                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG, "Successfully tuned chip settings");
                }                
                else
                {
                    ESP_LOGE(TAG, "Could not tune chip settings");
                    _power_down(emc1072_info);
                }
            }
            else
            {
                ESP_LOGE(TAG, "Unsupported device detected");
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "emc1072_info or smbus_info is NULL");
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t emc1072_device_id(const emc1072_info_t * emc1072_info, reg_default_t * device_type, 
                            reg_default_t * device_mfr, reg_default_t * device_rev)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(emc1072_info) && device_type && device_mfr && device_rev)
    {
        uint8_t id = 0;
        err = smbus_read_byte(emc1072_info->smbus_info, REG_DEVICE_TYPE | SMB_COMMAND, &id);
        if (err == ESP_OK)
        {
            *device_type = id;
            err = smbus_read_byte(emc1072_info->smbus_info, REG_DEVICE_MFR | SMB_COMMAND, &id);
            if (err == ESP_OK)
            {
                *device_mfr = id;
                err = smbus_read_byte(emc1072_info->smbus_info, REG_DEVICE_REV | SMB_COMMAND, &id);
                if (err == ESP_OK)
                {
                    *device_rev = id;
                }
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read device ID");
        }
    }
    return err;
}


// Assumes device is already awake up
esp_err_t _tune_conversion_rate(emc1072_info_t * emc1072_info, reg_default_t conversion_rate)
{
    esp_err_t err = ESP_FAIL;
    if (emc1072_info != NULL && emc1072_info->awake)
    {
        if ((err = smbus_write_byte(emc1072_info->smbus_info, REG_CONVERSION_RATE | SMB_COMMAND, conversion_rate)) == ESP_OK)
        {
            emc1072_info->conversion_rate = conversion_rate;
            ESP_LOGI(TAG, "Conversion rate tuned to 0x%02x", conversion_rate);
        }
        else 
        {
            ESP_LOGE(TAG, "Could not set conversion rate");
        }
    }
    return err;
}

// Assumes device is already awake
esp_err_t _tune_chip_settings(emc1072_info_t * emc1072_info, reg_default_t mask_all, 
            reg_default_t run_stop, reg_default_t alert_mode, 
            reg_default_t temp_range, reg_default_t dynamic_avg)
{
    esp_err_t err = ESP_FAIL;
    reg_default_t NEW_CONFIGURATION = DEFAULT_CONFIGURATION | mask_all | run_stop | alert_mode | temp_range | dynamic_avg;

    if (emc1072_info != NULL)
    {
        if ((err = smbus_write_byte(emc1072_info->smbus_info, REG_CONFIGURATION | SMB_COMMAND, NEW_CONFIGURATION)) == ESP_OK)
        {
            emc1072_info->mask_all = mask_all;
            emc1072_info->run_stop = run_stop;
            emc1072_info->alert_mode = alert_mode;
            emc1072_info->temp_range = temp_range;
            emc1072_info->dynamic_avg = dynamic_avg;
            ESP_LOGI(TAG, "All chip settings updated at 0x%02x", emc1072_info->smbus_info->address);
        }
        else 
        {
            ESP_LOGE(TAG, "Could not configure chip");
        }
    }
    return err;
}

esp_err_t _tune_all_settings(emc1072_info_t * emc1072_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(emc1072_info))
    {
        if ((err = _wake_up(emc1072_info)) == ESP_OK)
        {
            if ((err = _tune_chip_settings(emc1072_info, MASK_DISABLE, CHIP_WAKE_UP, ALERT_INTERRUPT, 
                                             TEMP_RANGE_DEFAULT, DYNAVG_DISABLE)) == ESP_OK)
            {
                if ((err = _tune_conversion_rate(emc1072_info, DESIRED_CONVERSION_RATE))== ESP_OK) 
                {
                    ESP_LOGI(TAG, "All settings updated at 0x%02x", emc1072_info->smbus_info->address);
                }
                else
                {
                    ESP_LOGE(TAG, "Could not configure all settings");
                }   
            }
        }
    }
    return err;
}

esp_err_t emc1072_diode_data(const emc1072_info_t * emc1072_info, reg_default_t * diode_status, 
                             reg_extend_t * diode_int, reg_extend_t * diode_ext) 
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(emc1072_info) && diode_status && diode_int && diode_ext)
    {
        uint8_t INVALID = 0x99;
        uint8_t id = INVALID;
        uint8_t hi = INVALID;
        uint8_t lo = INVALID;
        err = smbus_read_byte(emc1072_info->smbus_info, REG_DIODE_STATUS | SMB_COMMAND, &id);
        if (err == ESP_OK)
        {
            *diode_status = id;
            ESP_LOGI(TAG, "Diode status: 0x%08x", *diode_status);

            err = smbus_read_byte(emc1072_info->smbus_info, REG_DIODEDATA_INT_H | SMB_COMMAND, &hi);
            if (err == ESP_OK)
            {
                err = smbus_read_byte(emc1072_info->smbus_info, REG_DIODEDATA_INT_L | SMB_COMMAND, &lo);
                if (err == ESP_OK)
                {
                    *diode_int = (hi << 8) | lo;
                    ESP_LOGI(TAG, "Internal data: 0x%04x (%03x + %03x)", *diode_int, hi, lo);

                    err = smbus_read_byte(emc1072_info->smbus_info, REG_DIODEDATA_EXT_H | SMB_COMMAND, &hi);
                    if (err == ESP_OK)
                    {
                        err = smbus_read_byte(emc1072_info->smbus_info, REG_DIODEDATA_EXT_L | SMB_COMMAND, &lo);
                        if (err == ESP_OK)
                        {
                            *diode_ext = (hi << 8) | lo;
                            ESP_LOGI(TAG, "External data: 0x%04x (%03x + %03x)", *diode_ext, hi, lo);
                        }
                    }
                }
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read diode data");
        }
    }
    return err;
}

// Assumes device is already awake and configured to use the extended temerature range
float _compute_temp(emc1072_info_t * emc1072_info, reg_extend_t diode)
{
    reg_extend_t ZERO_REF_DEFAULT = 0b0;
    reg_extend_t ZERO_REF_EXTEND  = 0b01000000000;
    reg_extend_t zero_ref = 0;
    float scale = 0.0;
    
    switch (emc1072_info->temp_range)
    {
    case TEMP_RANGE_DEFAULT:
    default:
        zero_ref = ZERO_REF_DEFAULT;
        scale = 0.025;
        break;
    case TEMP_RANGE_EXTEND:
        zero_ref = ZERO_REF_EXTEND;
        scale = 0.125;
        break;
    }
    float temp = ((diode >> 5) * scale) - zero_ref;
    ESP_LOGI(TAG, "data: 0x%03x, temp: %.3f, zero ref: 0x%01x", diode, temp, zero_ref);
    return temp;
}

esp_err_t emc1072_read(emc1072_info_t * emc1072_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(emc1072_info))
    {
        if ((err = _wake_up(emc1072_info)) == ESP_OK)
        {
            // wait at least as long as the current conversion period
            TickType_t delay = 500;
            switch (emc1072_info->conversion_rate)
            {
            case RATE_1X16:
                delay = 16000;
                break;
            case RATE_1X8:
                delay = 8000;
                break;
            case RATE_1X4:
                delay = 4000;
                break;
            case RATE_1X2:
                delay = 2000;
                break;
            case RATE_1X1:
                delay = 1000;
                break;
            case RATE_2X1:
                delay = 500;
                break;
            case RATE_4X1:
                delay = 250;
                break;
            case RATE_8X1:
                delay = 125;
                break;
            case RATE_16X1:
                delay = 63;
                break;
            case RATE_32X1:
                delay = 32;
                break;
            default:
                ESP_LOGW(TAG, "Invalid conversion rate: %d", emc1072_info->conversion_rate);
                // fall through (i.e. don't support rates above 1Hz)          
            }
            vTaskDelay((delay - 1) / portTICK_RATE_MS + 1);

            // read the Diode registers and confirm that they are functional
            reg_default_t diode_status = 0x99;
            reg_extend_t diode_int = 0x99;
            reg_extend_t diode_ext = 0x99;

            if ((err = emc1072_diode_data(emc1072_info, &diode_status, &diode_int, &diode_ext)) == ESP_OK)
            {
                emc1072_info->temp_int = _compute_temp(emc1072_info, diode_int);
                emc1072_info->temp_ext = _compute_temp(emc1072_info, diode_ext);
            }
        }
    }
    return err;
}

/*
esp_err_t _tune_alert_settings(emc1072_info_t * emc1072_info, reg_default_t consecutive_alert)
{
    esp_err_t err = ESP_FAIL;
    if (emc1072_info != NULL && emc1072_info->awake)
    {
        if ((err = smbus_write_byte(emc1072_info->smbus_info, REG_CONSECUTIVE_ALERT | SMB_COMMAND, consecutive_alert)) == ESP_OK)
        {
            emc1072_info->consecutive_alert = consecutive_alert;
        }
    }
    return err;
}*/