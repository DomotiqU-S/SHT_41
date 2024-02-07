#include "Temp_Sensor.hpp"

Temp_Sensor::Temp_Sensor(uint8_t address, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed)
{
    this->i2c_controller = new I2CController(address, sda_pin, scl_pin, clk_speed);
    this->i2c_controller->begin();
}


Temp_Sensor::~Temp_Sensor()
{
}

esp_err_t Temp_Sensor::read()
{
    uint8_t data[3];
    esp_err_t ret = this->i2c_controller->read(data, REG_ADDR, 3, false);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to read data from sensor");
        return ret;
    }
    ESP_LOGI(TAG,"data read from sensor: %02x %02x %02x", data[0], data[1], data[2]);
    this->temp = getTemp(data);
    ESP_LOGI(TAG, "temp is %f", this->temp);
    ret = this->i2c_controller->read(data, REG_ADDR, 3, true);
    ESP_ERROR_CHECK(ret);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to read data from sensor");
        return ret;
    }
    ESP_LOGI(TAG,"data read from sensor: %02x %02x %02x", data[0], data[1], data[2]);
    this->humidity = getHumidity(data);
    ESP_LOGI(TAG, "humidity is %f", this->humidity);
    return ret;
}
float Temp_Sensor::getTemp(uint8_t *data)
{
    int bitValue = data[0] * 256 + data[1];
    // int crcT = data[2];
    temp = (float)(-45 + 175 * (float)(bitValue / 65535));
    return temp;
}
float Temp_Sensor::getHumidity(uint8_t *data)
{
    int bitValue = data[0] * 256 + data[1];
    // int crcH = data[2];
    float humidity = (float)(-6 + 125 * (float)(bitValue / 65535));
    if (humidity > 100)
    {
        humidity = 100;
    }
    if (humidity < 0)
    {
        humidity = 0;
    }
    return humidity;
}
