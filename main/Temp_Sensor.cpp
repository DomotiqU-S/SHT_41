#include "Temp_Sensor.hpp"

TempSensor::~TempSensor()
{
}

esp_err_t TempSensor::read()
{
    uint8_t data[6];
    esp_err_t ret = this->i2c_controller.read(data, REG_ADDR, 6, false);
    if (ret != ESP_OK)
    {
        #ifdef DEBUG_SENSOR
            ESP_LOGE(TAG_SENSOR, "failed to read data from sensor");
        #endif
        return ret;
    }
    #ifdef DEBUG_SENSOR
        ESP_LOGI(TAG_SENSOR, "data read from sensor: %02x %02x %02x %02x %02x %02x", data[0], data[1], data[2], data[3], data[4], data[5]);
    #endif

    if (crc8(data, 2) != data[2] || crc8(data + 3, 2) != data[5])
    {
        return ESP_FAIL;
    }
    this->temp = getTemp(data);
    #ifdef DEBUG_SENSOR
        ESP_LOGI(TAG_SENSOR, "temp is %f", this->temp);
    #endif

    this->humidity = getHumidity(data);

    #ifdef DEBUG_SENSOR
        ESP_LOGI(TAG_SENSOR, "humidity is %f", this->humidity);
    #endif

    return ret;
}

float TempSensor::getTemp(uint8_t *data)
{
    float bit_value = data[0] * 256 + data[1];
    temp = (float)(-45 + 175 * (float)((float)bit_value / 65535.0));
    return temp;
}

float TempSensor::getHumidity(uint8_t *data)
{

    float bit_value = data[3] * 256 + data[4];
    float humidity = (float)(-6 + 125 * (float)((float)bit_value / 65535.0));
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

uint8_t TempSensor::crc8(const uint8_t *data, int len)
{
    const uint8_t POLYNOMIAL(0x31);
    uint8_t crc(0xFF);

    for (int j = len; j; --j)
    {
        crc ^= *data++;

        for (int i = 8; i; --i)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}