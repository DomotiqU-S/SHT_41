#include "Temp_Sensor.hpp"

Temp_Sensor::Temp_Sensor(uint8_t address, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed)
{
    this->i2c_controller = new I2CController(address, sda_pin, scl_pin, clk_speed);
    this->i2c_controller->begin();
    // soft reset on begin
    this->i2c_controller->write(nullptr, 0x94, 0);
}

Temp_Sensor::~Temp_Sensor()
{
}

esp_err_t Temp_Sensor::read()
{
    uint8_t data[6];
    esp_err_t ret = this->i2c_controller->read(data, REG_ADDR, 6, false);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to read data from sensor");
        return ret;
    }
    ESP_LOGI(TAG, "data read from sensor: %02x %02x %02x %02x %02x %02x", data[0], data[1], data[2], data[3], data[4], data[5]);
    if (this->Crc8(data, 2) != data[2] || this->Crc8(data + 3, 2) != data[5])
    {
        return ESP_FAIL;
    }
    this->temp = getTemp(data);
    ESP_LOGI(TAG, "temp is %f", this->temp);
    this->humidity = getHumidity(data);
    ESP_LOGI(TAG, "humidity is %f", this->humidity);
    return ret;
}
float Temp_Sensor::getTemp(uint8_t *data)
{
    float bitValue = data[0] * 256 + data[1];
    temp = (float)(-45 + 175 * (float)((float)bitValue / 65535.0));
    return temp;
}
float Temp_Sensor::getHumidity(uint8_t *data)
{
    float bitValue = data[3] * 256 + data[4];
    float humidity = (float)(-6 + 125 * (float)((float)bitValue / 65535.0));
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
esp_err_t Temp_Sensor::Crc8(const uint8_t *data, int len)
{
    /*
     *
     * CRC-8 formula
     *
     * Test data 0xBE, 0xEF should yield 0x92
     *
     * Initialization data 0xFF
     * Polynomial 0x31 (x8 + x5 +x4 +1)
     * Final XOR 0x00
     */

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