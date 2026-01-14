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

//--Calculation Variables--
uint16_t prom_data[7] = {0};
uint32_t D1;
uint32_t D2;

int32_t dT;
int32_t TEMP;

int64_t OFF;
int64_t SENS;
int32_t P;
int32_t T2;
int64_t OFF2;
int64_t SENS2;

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    //Set I2C master bus configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
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

static void ms5611_reset(i2c_master_dev_handle_t dev_handle)
{
    uint8_t rst = 0x1E;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &rst, 1, -1));
    vTaskDelay(pdMS_TO_TICKS(3)); // datasheet: ~2.8ms max
}

static void ms5611_read_prom(i2c_master_dev_handle_t dev_handle)
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

static void ms5611_read_conversion(i2c_master_dev_handle_t dev_handle, uint8_t cmd, uint32_t *result)
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

static void ms5611_calculateTemperature()
{   
    dT = D2 - (((uint32_t) prom_data[5])<<8);
    int64_t calcTemp = 2000 + ((int64_t)dT * prom_data[6]) / (1LL << 23);
    TEMP = (int32_t) calcTemp;

    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
    //Compensate for low Temp
    if(TEMP<2000)
    {
        T2 = (dT*dT)/(1LL << 31);
        OFF2 = 5*(TEMP-2000)*(TEMP-2000)/2;
        SENS2 = 5*(TEMP-2000)*(TEMP-2000)/4;
    }

    if(TEMP<-1500)
    {
        OFF2 += 7*(TEMP+1500)*(TEMP+1500);
        SENS2 += 11*(TEMP+1500)*(TEMP+1500)/2;
    }

    //Final Temp result
    TEMP -= T2;
}

static void ms5611_calculatePressure()
{
    OFF = (((uint32_t) prom_data[2])<<16) + (prom_data[4]*((int64_t) dT))/(1LL << 7);
    SENS = (((uint32_t) prom_data[1])<<15) + (prom_data[3]*((int64_t) dT))/(1LL << 8);
    OFF -= OFF2;
    SENS -= SENS2;
    //Final Pressure Result
    P = ((D1*(SENS)/(1LL << 21)) - OFF)/(1LL << 15);
}

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    //Initialize I2C Master bus and device
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C Master initialized and MS5611 device added.");

    //Reset ms5611 sensor
    ms5611_reset(dev_handle);
    //Read device Prom values
    ms5611_read_prom(dev_handle);

    while(1){
        //Read D1 conversion result
        ms5611_read_conversion(dev_handle, MS5611_D1_OSR_4096, &D1);
        //Read D2 conversion result
        ms5611_read_conversion(dev_handle, MS5611_D2_OSR_4096, &D2);
        //Calculates Temperature 
        ms5611_calculateTemperature();
        ms5611_calculatePressure();

        printf("The temparture is %d\n", TEMP);
        prinf("The pressure is %d", P);
    }

}