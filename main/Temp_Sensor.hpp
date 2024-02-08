#ifndef TEMP_SENSOR_HPP
#define TEMP_SENSOR_HPP

#include "I2CController.cpp"

#define I2C_MASTER_FREQ_HZ  400000          /*!< I2C master clock frequency */
#define SDA_PIN             (gpio_num_t)1   /*!< GPIO pin for the SDA line */
#define SCL_PIN             (gpio_num_t)2   /*!< GPIO pin for the SCL line */
#define SENSOR_ADDR         0x44            /*!< Slave address of the temp sensor */
#define REG_ADDR            0xFD            /*!< Register addresses of the data */

class TempSensor{
private:
    I2CController *i2c_controller;
    float temp;
    float humidity;
public:
    TempSensor(uint8_t address = SENSOR_ADDR, gpio_num_t sda_pin = SDA_PIN, gpio_num_t scl_pin = SCL_PIN, uint32_t clk_speed = I2C_MASTER_FREQ_HZ);
    ~TempSensor();
    esp_err_t read();
    float getTemp(uint8_t *data);
    float getHumidity(uint8_t *data);
    uint8_t crc8(const uint8_t *data, int len);
};

#endif // TEMP_SENSOR_HPP