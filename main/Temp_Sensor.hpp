#ifndef TEMP_SENSOR_HPP
#define TEMP_SENSOR_HPP

#include "I2CController.cpp"

#define I2C_MASTER_FREQ_HZ      400000        /*!< I2C master clock frequency */
#define SDA_PIN                 (gpio_num_t)1
#define SCL_PIN                 (gpio_num_t)2
#define SENSOR_ADDR             0x44    /*!< Slave address of the temp sensor */
#define REG_ADDR                0xFD    /*!< Register addresses of the data */

class Temp_Sensor{
private:
    I2CController *i2c_controller;
    float temp;
    float humidity;
public:
    Temp_Sensor(uint8_t address = SENSOR_ADDR, gpio_num_t sda_pin = SDA_PIN, gpio_num_t scl_pin = SCL_PIN, uint32_t clk_speed = I2C_MASTER_FREQ_HZ);
    ~Temp_Sensor();
    esp_err_t read();
    float getTemp(uint8_t *data);
    float getHumidity(uint8_t *data);
};

#endif // TEMP_SENSOR_HPP