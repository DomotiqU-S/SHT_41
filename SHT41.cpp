#include "SHT41.hpp"

SHT41::~SHT41()
{
}

esp_err_t SHT41::read()
{
    uint8_t data[6];
    esp_err_t ret = this->i2c_controller->read(data, REG_ADDR, 6, false);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (crc8(data, 2) != data[2] || crc8(data + 3, 2) != data[5])
    {
        return ESP_FAIL;
    }
    this->temp = readTemperature(data);

    this->humidity = readHumidity(data);

    return ret;
}

float SHT41::readTemperature(uint8_t *data)
{
    float bit_value = data[0] * 256 + data[1];
    temp = (float)(-45 + 175 * (float)((float)bit_value / 65535.0));
    return temp;
}

float SHT41::readHumidity(uint8_t *data)
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

uint8_t SHT41::crc8(const uint8_t *data, int len)
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