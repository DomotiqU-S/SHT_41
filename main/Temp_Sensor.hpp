#ifndef TEMP_SENSOR_HPP
#define TEMP_SENSOR_HPP

#include <I2CController.hpp>

#define I2C_MASTER_FREQ_HZ  400000          /*!< I2C master clock frequency */
#define SDA_PIN             (gpio_num_t)1   /*!< GPIO pin for the SDA line */
#define SCL_PIN             (gpio_num_t)2   /*!< GPIO pin for the SCL line */
#define SENSOR_ADDR         0x44            /*!< Slave address of the temp sensor */
#define REG_ADDR            0xFD            /*!< Register addresses of the data */
#define TAG_SENSOR          "TEMP_SENSOR"   /*!< Tag for the temperature sensor */
//#define DEBUG_SENSOR        0               /*!< Enable debug logs for the temperature sensor */

class TempSensor{
private:
    I2CController i2c_controller;
    float temp;
    float humidity;
public:
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
    */
    TempSensor(uint8_t address = SENSOR_ADDR, gpio_num_t sda_pin = SDA_PIN, gpio_num_t scl_pin = SCL_PIN, uint32_t clk_speed = I2C_MASTER_FREQ_HZ) : i2c_controller(address, sda_pin, scl_pin, clk_speed) {
        this->i2c_controller.begin();
        i2c_controller.write(nullptr, 0x94, 0);
    }
    ~TempSensor();

    /**
     * @brief Reads data from the temperature sensor.
     * 
     * This function reads data from the temperature sensor using the I2C controller.
     * It retrieves the temperature and humidity values from the sensor data and stores them in the respective member variables.
     * The function also performs a CRC check on the received data to ensure its integrity.
     *
     * @return ESP_OK if the data is read successfully, ESP_FAIL otherwise. 
    */
    esp_err_t read();

    /**
     * @brief Gets the temperature value from the sensor data.
     * 
     * This function calculates the temperature value from the sensor data.
     * 
     * @param data The sensor data.
     * 
     * @return The float temperature value.
    */
    float getTemp(uint8_t *data);

    /**
     * @brief Gets the humidity value from the sensor data.
     * 
     * This function calculates the humidity value from the sensor data.
     * 
     * @param data The sensor data.
     * 
     * @return The float humidity value.
    */
    float getHumidity(uint8_t *data);

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
    uint8_t crc8(const uint8_t *data, int len);
};

#endif // TEMP_SENSOR_HPP