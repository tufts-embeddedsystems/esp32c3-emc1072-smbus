/**
 * @file emc1072.h
 * @brief Interface definitions for the ESP32-compatible (I hope?) EMC1072 temperature sensor
 *
 * This component provides structures and functions that are useful for communicating with the device.
 */

#ifndef EMC1072_H
#define EMC1072_H

#include <stdbool.h>
#include "smbus.h"
/*
#ifdef __cplusplus
extern "C" {
#endif
*/
// Register addresses (temp sensor)
#define EMC1072_SMBUS_ADDRESS    0b1001100  // defined by 10k pullup resistor on THERM pin
#define EMC1072_SMBUS_TIMEOUT    1500       // milliseconds to wait for slave response

#define REG_DIODEDATA_INT_H      0x00
#define REG_DIODEDATA_INT_L      0x29
#define REG_DIODEDATA_EXT_H      0x01
#define REG_DIODEDATA_EXT_L      0x10
#define REG_DIODE_STATUS         0x02
// tune diode settings
#define REG_DIODELIMIT_INT_HIGH  0x05  // internal high 
#define REG_DIODELIMIT_INT_LOW   0x06  // internal low 
#define REG_DIODELIMIT_EXT_HIGHH 0x07  // external high (to 1 deg.)
#define REG_DIODELIMIT_EXT_HIGHL 0x13  // external high (to 0.25 deg.)
#define REG_DIODELIMIT_EXT_LOWH  0x08  // external low (to 1 deg.)
#define REG_DIODELIMIT_EXT_LOWL  0x14  // external low (to 0.25 deg.)
#define REG_THERMLIMIT_EXT       0x19  // external diode THERM limit
#define REG_THERMLIMIT_INT       0x20  // internal diode THERM limit
#define REG_THERMLIMIT_HYS       0x21  // THERM hysteresis
#define REG_DIODEIDEALITY_EXT    0x27  // external ideality factor
#define REG_DIODEIDEALITY_INT    0x27  // internal ideality factor
// global settings
#define REG_CONFIGURATION        0x03  // basic operations (mirrored at 0x09)
#define REG_CONVERSION_RATE      0x04  // samples/sec (mirrored at 0x0A)
#define REG_CHANNEL_MASK         0x1F
#define REG_CONSECUTIVE_ALERT    0x22  // alert buffer size
#define REG_FILTER_CONTROL       0x40
#define REG_SCRATCHPAD1     0x11
#define REG_SCRATCHPAD2     0x12
#define REG_ONE_SHOT        0x0F // wake up from standby & update all measurements (one cycle)
#define REG_DEVICE_TYPE     0xFD
#define REG_DEVICE_MFR      0xFE
#define REG_DEVICE_REV      0xFF


// The following values are bitwise ORed with register addresses to create a command value
#define SMB_BLOCK           0x10  // Transaction to use Block Write/Read protocol
#define SMB_WORD            0x20  // Transaction to use Word Write/Read protocol
#define SMB_CLEAR           0x40  // Clear any pending interrupt (self-clearing)
#define SMB_COMMAND         0x80  // Select command register

#define INVALID_DEVICE    0x0
#define VALID_DEVICE_ID   0x20
#define VALID_DEVICE_MFR  0x5D
#define VALID_DEVICE_REV  0x03

// Device defaults:
#define DEFAULT_CONFIGURATION     0b00011000
#define DEFAULT_CONSECUTIVE_ALERT ALERT_BUFFER_4T1A
#define DEFAULT_CONVERSION_RATE   RATE_4X1
#define DESIRED_CONVERSION_RATE   RATE_1X1

/**
 * @brief Enum for supported conversion rates
 */
typedef enum
{
    RATE_1X16 = 0x0,    // min: 1 sample every 16 seconds
    RATE_1X8 = 0x1,
    RATE_1X4 = 0x2,
    RATE_1X2 = 0x3,
    RATE_1X1 = 0x4,
    RATE_2X1 = 0x5,
    RATE_4X1 = 0x6, // default: 4 samples per second
    RATE_8X1 = 0x7,
    RATE_16X1 = 0x8,
    RATE_32X1 = 0x9,
    RATE_64X1 = 0xA,    // max: 64 samples per second
    // remaining values all map to 1 sample per second
} emc1072_conversion_rate_t;

/**
 * @brief Enum for valid mask-all settings, stored in bit[7] of configuration register
 */
typedef enum
{
    MASK_ENABLE  = 0b1000000,
    MASK_DISABLE = 0b0000000,
} emc1072_mask_all_t;

/**
 * @brief Enum for valid run/stop settings, stored in bit[6] of configuration register
 */
typedef enum
{
    CHIP_WAKE_UP = 0b0000000,
    CHIP_STANDBY = 0b0100000,
} emc1072_run_stop_t;

/**
 * @brief Enum for valid alert/comp settings, stored in bit[5] of configuration register
 */
typedef enum
{
    ALERT_INTERRUPT  = 0b0000000,
    ALERT_COMPARATOR = 0b0010000,
} emc1072_alert_comp_t;

/**
 * @brief Enum for valid temp-range settings, stored in bit[2] of configuration register
 */
typedef enum
{
    TEMP_RANGE_DEFAULT = 0b0000000, //   0 deg. C - 127 deg. C, binary format
    TEMP_RANGE_EXTEND  = 0b0000100, // -64 deg. C - 191 deg. C, offset binary format
} emc1072_temp_range_t;

/**
 * @brief Enum for valid dynamic-averaging settings, stored in bit[1] of configuration register
 */
typedef enum
{
    DYNAVG_ENABLE  = 0b0000000, 
    DYNAVG_DISABLE = 0b0000010, 
} emc1072_dynamic_avg_t;

/**
 * @brief Enum for supported alert buffer sizes:
 * wait for X consecutive out-of-limit conditions before asserting THERM or ALERT pins 
 */
typedef enum
{
    CONSECUTIVE_ALERT1 = 0b000,   ///< 1 consecutive out-of-limit condition asserts the appropriate pin
    CONSECUTIVE_ALERT2 = 0b001,   ///< 2 consecutive out-of-limits
    CONSECUTIVE_ALERT3 = 0b011,   ///< 3 consecutive out-of-limits
    CONSECUTIVE_ALERT4 = 0b111,   ///< 4 consecutive out-of-limits
} emc1072_consecutive_alert_t;

/**
 * @brief Enum for common supported alert buffer register settings:
 * bit[7] = TIMEOUT_ENABLED (default: 0)
 * bit[6:4] = CONSECUTIVE_THERM[2:0]
 * bit[3:1] = CONSECUTIVE_ALERT[2:0]
 * bit[0] = unused
 */
typedef enum
{
    ALERT_BUFFER_4T1A = 0x70,   ///< default: 4 OOL = 1 THERM, 1 THERM = 1 ALERT, TIMEOUT = 0
} emc1072_consecutive_alert_reg_t;

typedef uint8_t reg_default_t;      ///< The type of a standard 1-byte register
typedef uint16_t reg_extend_t;      ///< The type of an extended 2-byte register (offset binary)
/*
typedef uint8_t emc1072_device_id_t;      ///< The type of the IC's product ID
typedef uint8_t emc1072_device_mfr_t;     ///< The type of the IC's manufacturer
typedef uint8_t emc1072_device_rev_t;     ///< The type of the IC's revision number
typedef uint8_t emc1072_data_binary_t;    ///< The type of an default diode measurement value (binary)
typedef uint16_t emc1072_data_offset_t;   ///< The type of an extended-grange diode measurement value (offset binary)
*/
/**
 * @brief Structure containing information related to the SMBus protocol.
 */
typedef struct
{
    bool          init;                   ///< True if struct has been initialised, otherwise false
    bool          awake;                  ///< True if the device has been woken up
    smbus_info_t * smbus_info;            ///< Pointer to associated SMBus info
    reg_default_t device_type;            ///< Detected product ID of device 
    reg_default_t device_mfr;             ///< Detected manufacturer of device 
    reg_default_t device_rev;             ///< Detected revision of device 
    float         temp_int;           ///< Measured temperature data of interior diode
    float         temp_ext;           ///< Measured temperature data of exterior diode
    reg_extend_t  diode_int;          ///< Raw data of exterior diode
    reg_extend_t  diode_ext;          ///< Raw data of exterior diode
    reg_default_t consecutive_alert;  ///< Current consecutive alert buffer for ALERT pin
    reg_default_t consecutive_therm;  ///< Current consecutive alert buffer for THERM pin
    reg_default_t conversion_rate;    ///< Current frequency of measurements
    reg_default_t timeout;            ///< Enable/disable SMBus timeout
    reg_default_t mask_all;           ///< Enable/disable pin masking 
    reg_default_t run_stop;           ///< Wake-mode or standby-mode 
    reg_default_t alert_mode;         ///< Set ALERT pin to interrupt or comparator mode 
    reg_default_t temp_range;         ///< Default or extended temperature range
    reg_default_t dynamic_avg;        ///< Enable/disable dynamic averaging
} emc1072_info_t;

/**
 * @brief Construct a new EMC1072 info instance.
 *        New instance should be initialised before calling other functions.
 * @return Pointer to new device info instance, or NULL if it cannot be created.
 */
emc1072_info_t * emc1072_malloc(void);

/**
 * @brief Delete an existing EMC1072 info instance.
 * @param[in,out] emc1072_info Pointer to EMC1072 info instance that will be freed and set to NULL.
 */
void emc1072_free(emc1072_info_t ** emc1072_info);

/**
 * @brief Initialise a EMC1072 info instance with the specified SMBus information.
 * @param[in] emc1072_info Pointer to EMC1072 info instance.
 * @param[in] smbus_info Pointer to SMBus info instance.
 */
esp_err_t emc1072_init(emc1072_info_t * emc1072_info, smbus_info_t * smbus_info);

/**
 * @brief Retrieve the Device Type ID, manufacturer and revision number from the device.
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @param[out] device_id  The retrieved Device Type ID.
 * @param[out] device_mfr The retrieved Device manufacturer.
 * @param[out] device_rev The retrieved Device Revision number.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t emc1072_device_id(const emc1072_info_t * emc1072_info, reg_default_t * device_id, 
                            reg_default_t * device_mfr, reg_default_t * device_rev);
/**
 * @brief Retrieve the Diode Status and Data from the device.
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @param[out] diode_int   The retrieved Internal Diode Data.
 * @param[out] diode_ext   The retrieved Internal Diode Data.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t emc1072_diode_data(const emc1072_info_t * emc1072_info, reg_default_t * diode_status, 
                             reg_extend_t * diode_int, reg_extend_t * diode_ext);

/**
 * @brief Set the consecutive alert buffer for the THERM and ALERT pins. 
 *        These values are set together as they are programmed via the same register.
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @param[in] consecutive_therm The consecutive alert buffer to use for the THERM pin.
 * @param[in] consecutive_alert The consecutive alert buffer to use for the ALERT pin.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
/*
esp_err_t _tune_alert_settings(emc1072_info_t * emc1072_info, reg_default_t consecutive_alert, 
                                                                     reg_default_t consecutive_therm);
*/
/**
 * @brief Set the conversion rate, either samples per second or seconds per sample
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @param[in] conversion_rate The frequency of measurements.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t _tune_conversion_rate(emc1072_info_t * emc1072_info, reg_default_t conversion_rate);

/**
 * @brief Tune the configuration register, which is used to set the alert mask, alert operation mode, and data range,
 *        tell device to wake up or stand by; and enable/disable dynamic averaging. 
 *        These values are all set together as they are programmed via the same register.
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @param[in] mask_all     Enable/disable alert pin masking
 * @param[in] run_stop     Tell device to wake up or stand by
 * @param[in] alert_comp   Set alert pin mode to interrupt or comparator
 * @param[in] temp_range   0: 0 deg. C - 127 deg. C, binary format (default)
 *                         1: -64 deg. C - 191 deg. C, offset binary format
 * @param[in] dynamic_avg  Enable/disable dynamic averaging
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t _tune_chip_settings(emc1072_info_t * emc1072_info, reg_default_t mask_all, 
            reg_default_t run_stop, reg_default_t alert_comp, 
            reg_default_t temp_range, reg_default_t dynamic_avg);

/**
 * @brief Tune the configuration register, which is used to set the alert mask, alert operation mode, and data range,
 *        tell device to wake up or stand by; and enable/disable dynamic averaging. 
 *        These values are all set together as they are programmed via the same register.
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t _tune_all_settings(emc1072_info_t * emc1072_info);

/**
 * @brief Retrieve the internal and external diode measurements from the device.
 *        This function will sleep until the appropriate conversion period has passed.
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @param[out] diode_int The interior diode measurement.
 * @param[out] diode_ext The exterior diode measurement.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */ 
esp_err_t emc1072_read(emc1072_info_t * emc1072_info); /*, reg_extend_t * diode_int, reg_extend_t * diode_ext);*/

/**
 * @brief Calculate temperature value from diode data
 * @param[in] emc1072_info Pointer to initialised EMC1072 info instance.
 * @param[in] high_byte  
 * @param[in] low_byte  
 * @return Calculated temp based on binary register data
 */ 
float _compute_temp(emc1072_info_t * emc1072_info, reg_extend_t diode);

#endif  // EMC1072_H
