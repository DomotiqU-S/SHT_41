#include "esp_log.h"
#include "freertos/include/freertos/task.h"
#include "SHT41.hpp"
#include "I2CController.hpp"
#include "stdint.h"

#define ADDRESS (uint8_t)0x00
#define SDA_PIN GPIO_NUM_0
#define SCL_PIN GPIO_NUM_1

extern "C" void app_main(void)
{
    I2CController i2c_master(ADDRESS, SDA_PIN, SCL_PIN, 100000);
    SHT41 temp_sensor(i2c_master);

    while(1)
    {
        if(temp_sensor.read() == ESP_OK)
        {
            ESP_LOGI("TempSensor","Temperature: %f",temp_sensor.getTemperature());
            ESP_LOGI("TempSensor","Humidity: %f",temp_sensor.getHumidity());
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}