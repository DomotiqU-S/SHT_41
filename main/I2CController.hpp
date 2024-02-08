#ifndef I2C_CONTROLLER_H
#define I2C_CONTROLLER_H

#include "BusController.hpp"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM              (i2c_port_t)0   /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000            /*!< I2C timeout in milliseconds */
#define DEBUG_I2C_CONTROLLER        1               /*!< Enable debug logs for the I2C controller */

static const char *TAG = "I2C_CONTROLLER";

class I2CController : public BusController {
private:
    uint8_t address;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t clk_speed;
public:
    I2CController(uint8_t address, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed = I2C_MASTER_FREQ_HZ);
    ~I2CController();
    esp_err_t begin();
    esp_err_t readByte(uint8_t *rx_buffer, uint8_t reg, bool restart = false);
    esp_err_t readWord(uint8_t *rx_buffer, uint8_t reg, bool restart = false);
    esp_err_t read(uint8_t *rx_buffer, uint8_t reg, uint8_t len, bool restart = false);
    esp_err_t writeByte(uint8_t *tx_buffer, uint8_t reg);
    esp_err_t writeWord(uint8_t *tx_buffer, uint8_t reg);
    esp_err_t write(uint8_t *tx_buffer, uint8_t reg, uint8_t len);
};

#endif