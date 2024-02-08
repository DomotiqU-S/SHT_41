#include "I2CController.hpp"

/**
 * @brief Constructor for the I2CController class.
 * 
 * This constructor initializes the I2C controller with the given parameters.
 * 
 * @param address The I2C address of the slave device.
 * @param sda_pin The GPIO pin number for the SDA line.
 * @param scl_pin The GPIO pin number for the SCL line.
 * @param clk_speed The clock speed of the I2C bus.
 * 
 * @return N/A
*/
I2CController::I2CController(uint8_t address, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed)
{
    this->sda_pin = sda_pin;
    this->scl_pin = scl_pin;
    this->clk_speed = clk_speed;
    this->address = address;
}

/**
 * @brief Initializes the I2C controller.
 * 
 * This function initializes the I2C controller with the given parameters.
 * 
 * @return ESP_OK if the I2C controller is initialized successfully, ESP_FAIL otherwise.
*/
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

/**
 * @brief Destructor for the I2CController class.
 * 
 * This destructor frees the memory allocated for the I2C controller.
 * 
 * @return N/A
*/
I2CController::~I2CController()
{
}
/**
 * @brief Reads a byte from the slave device.
 * 
 * This function reads a byte from the slave device at the given register address.
 * 
 * @param rx_buffer Pointer to the buffer to store the received data.
 * @param reg The register address to read from.
 * @param restart if sensor support restart command
 * 
 * @return ESP_OK if the byte is read successfully, ESP_FAIL otherwise.
*/
esp_err_t I2CController::readByte(uint8_t *rx_buffer, uint8_t reg, bool restart)
{
    return this->read(rx_buffer, reg, 1, restart);
}

/**
 * @brief Reads a word from the slave device.
 * 
 * This function reads a word from the slave device at the given register address.
 * 
 * @param rx_buffer Pointer to the buffer to store the received data.
 * @param reg The register address to read from.
 * @param restart if sensor support restart command
 * 
 * @return ESP_OK if the word is read successfully, ESP_FAIL otherwise.
*/
esp_err_t I2CController::readWord(uint8_t *rx_buffer, uint8_t reg, bool restart)
{
    return this->read(rx_buffer, reg, 4, restart);
}

/**
 * @brief Reads data from the slave device.
 * 
 * This function reads data from the slave device at the given register address.
 * 
 * @param rx_buffer Pointer to the buffer to store the received data.
 * @param reg The register address to read from.
 * @param len The number of bytes to read.
 * @param restart if sensor support restart command
 * 
 * @return ESP_OK if the data is read successfully, ESP_FAIL otherwise.
*/
esp_err_t I2CController::read(uint8_t *rx_buffer, uint8_t reg, uint8_t len, bool restart)
{
    int ret;

    uint8_t write_buffer[1] = {reg};

    #if DEBUG_I2C_CONTROLLER
        ESP_LOGI(TAG, "Reading %d bytes from register 0x%02x in address 0x%02x", len, reg, this->address);
    #endif
    if(!restart)
    {
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, this->address, write_buffer, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ret write: %d", ret);
        vTaskDelay(1);
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
/**
 * @brief Writes a byte to the slave device.
 * 
 * This function writes a byte to the slave device at the given register address.
 * 
 * @param tx_buffer Pointer to the buffer containing the data to write.
 * @param reg The register address to write to.
 * 
 * @return ESP_OK if the byte is written successfully, ESP_FAIL otherwise.
*/
esp_err_t I2CController::writeByte(uint8_t *tx_buffer, uint8_t reg)
{
    return this->write(tx_buffer, reg, 1);
}
/**
 * @brief Writes a word to the slave device.
 * 
 * This function writes a word to the slave device at the given register address.
 * 
 * @param tx_buffer Pointer to the buffer containing the data to write.
 * @param reg The register address to write to.
 * 
 * @return ESP_OK if the word is written successfully, ESP_FAIL otherwise.
*/
esp_err_t I2CController::writeWord(uint8_t *tx_buffer, uint8_t reg)
{
    return this->write(tx_buffer, reg, 4);
}
/**
 * @brief Writes data to the slave device.
 * 
 * This function writes data to the slave device at the given register address.
 * 
 * @param tx_buffer Pointer to the buffer containing the data to write.
 * @param reg The register address to write to.
 * @param len The number of bytes to write.
 * 
 * @return ESP_OK if the data is written successfully, ESP_FAIL otherwise.
*/
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