#include "Temp_Sensor.hpp"
 /**
 * @brief Constructor for the TempSensor class.
 *  
 * This constructor initializes the I2C controller with the given parameters.
 * 
 * @param address The I2C address of the temperature sensor.
 * @param sda_pin The GPIO pin number for the SDA line.
 * @param scl_pin The GPIO pin number for the SCL line.
 * @param clk_speed The clock speed of the I2C bus.
 * 
 * @return N/A
*/
TempSensor::TempSensor(uint8_t address, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed)
{
    this->i2c_controller = new I2CController(address, sda_pin, scl_pin, clk_speed);
    this->i2c_controller->begin();
    // soft reset on begin
    this->i2c_controller->write(nullptr, 0x94, 0);
}
/**
 * @brief Destructor for the TempSensor class.
 * 
 * This destructor frees the memory allocated for the I2C controller.
 * 
 * @return N/A
*/
TempSensor::~TempSensor()
{
    delete this->i2c_controller;
}
/**
 * @brief Reads data from the temperature sensor.
 * 
 * This function reads data from the temperature sensor using the I2C controller.
 * It retrieves the temperature and humidity values from the sensor data and stores them in the respective member variables.
 * The function also performs a CRC check on the received data to ensure its integrity.
 *
 * @return ESP_OK if the data is read successfully, ESP_FAIL otherwise. 
*/
esp_err_t TempSensor::read()
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
/**
 * @brief Gets the temperature value from the sensor data.
 * 
 * This function calculates the temperature value from the sensor data.
 * 
 * @param data The sensor data.
 * 
 * @return The float temperature value.
*/
float TempSensor::getTemp(uint8_t *data)
{
    float bit_value = data[0] * 256 + data[1];
    temp = (float)(-45 + 175 * (float)((float)bit_value / 65535.0));
    return temp;
}
/**
 * @brief Gets the humidity value from the sensor data.
 * 
 * This function calculates the humidity value from the sensor data.
 * 
 * @param data The sensor data.
 * 
 * @return The float humidity value.
*/
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
/**
 * @brief Calculates the CRC-8 value of the given data.
 * 
 * This function calculates the CRC-8 value of the given data using the polynomial 0x31.
 * 
 * @param data The data to calculate the CRC-8 value for.
 * @param len The length of the data.
 * 
 * @return The CRC-8 value.
*/
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