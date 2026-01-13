#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "esp_log.h"


static const char *TAG = "example";


#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22

#define I2C_MASTER_FREQ_HZ  100000
#define MS5611_SENSOR_ADDR  0x77
#define MS5611_D1_OSR_4096 0x48
#define MS5611_D2_OSR_4096 0x58

uint16_t prom_data[7];
uint32_t D1;
uint32_t D2;

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    //Set I2C master bus configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,  
    };
    //Initialize I2C master bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    //Set I2C device configuration for MS5611 sensor
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MS5611_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };

    //Initialize I2C device handle for MS5611 sensor
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

}

static void read_prom(i2c_master_dev_handle_t dev_handle)
{
        for (int i = 0; i < 6; i++) {
            // Send PROM read command
            uint8_t cmd = 0xA2 + (i*2); // Send Prom Command
            ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &cmd, 1, -1));
            // Read PROM data
            uint8_t data[2];
            ESP_ERROR_CHECK(i2c_master_receive(dev_handle, data, 2, -1));
            prom_data[i+1] = ((uint16_t)data[0] << 8) | ((uint16_t)data[1]);

        }
}

static void read_conversion(i2c_master_dev_handle_t dev_handle, uint8_t cmd, uint32_t *result)
{
    ESP_LOGI(TAG, "Sending conversion command: 0x%02X", cmd);
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &cmd, 1, -1));
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for conversion to complete

    //ADC READ SEQUENCE
    uint8_t read_cmd = 0x00; // Command to read ADC result
    uint8_t data[3];
    // Send ADC read command
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &read_cmd, 1, -1));

    // Read 3 bytes of either D1 or D2 result
    ESP_ERROR_CHECK(i2c_master_receive(dev_handle, data, 3, -1));
    *result = ((uint32_t) data[0] << 16) | (uint32_t) (data[1] << 8) | (uint32_t) data[2];
}

static int32_t calculateTemperature()
{
    int32_t dT = D2 - (((uint32_t) prom_data[5])<<8);
    int64_t TEMP = 2000 + ((int64_t)dT * prom_data[6]) / (1LL << 23);

    return (int32_t) TEMP;
}

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    //Initialize I2C Master bus and device
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C Master initialized and MS5611 device added.");

    //Read device Prom values
    read_prom(dev_handle);

    while(1){
        //Read D1 conversion result
        read_conversion(dev_handle, MS5611_D1_OSR_4096, &D1);
        //Read D2 conversion result
        read_conversion(dev_handle, MS5611_D2_OSR_4096, &D2);


    }

}