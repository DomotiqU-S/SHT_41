#include "I2CController.hpp"

I2CController::I2CController(uint8_t address, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed)
{
    this->sda_pin = sda_pin;
    this->scl_pin = scl_pin;
    this->clk_speed = clk_speed;
}

esp_err_t I2CController::begin() {
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.master.clk_speed = clk_speed;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.clk_flags = 0;

    i2c_param_config(i2c_master_port, &conf);

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Initializing I2C controller");
    #endif

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

I2CController::~I2CController()
{
}

esp_err_t I2CController::readByte(uint8_t *rx_buffer, uint8_t reg, bool restart)
{
    return this->read(rx_buffer, reg, 1, restart);
}

esp_err_t I2CController::readWord(uint8_t *rx_buffer, uint8_t reg, bool restart)
{
    return this->read(rx_buffer, reg, 4, restart);
}

esp_err_t I2CController::read(uint8_t *rx_buffer, uint8_t reg, uint8_t len, bool restart)
{
    int ret;

    uint8_t write_buffer[1] = {reg};

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Reading %d bytes from register 0x%02x", len, reg);
    #endif
    if(restart)
    {
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, this->address, write_buffer, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ret write: %d", ret);
        ret |= i2c_master_read_from_device(I2C_MASTER_NUM, this->address, rx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ret read: %d", ret);
    }
    else
    {
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, this->address, write_buffer, 1, rx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ret Write_Read: %d", ret);
    }

    return ret;
}

esp_err_t I2CController::writeByte(uint8_t *tx_buffer, uint8_t reg)
{
    return this->write(tx_buffer, reg, 1);
}

esp_err_t I2CController::writeWord(uint8_t *tx_buffer, uint8_t reg)
{
    return this->write(tx_buffer, reg, 4);
}

esp_err_t I2CController::write(uint8_t *tx_buffer, uint8_t reg, uint8_t len)
{
    int ret;
    uint8_t write_buffer[len + 1];
    
    // Fill the write buffer with the register address and data
    write_buffer[0] = reg;
    memcpy(write_buffer + 1, tx_buffer, len);

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Writing %d bytes to register 0x%02x", len, reg);
    #endif
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, this->address, write_buffer, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    return ret;
}