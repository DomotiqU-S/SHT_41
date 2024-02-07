#ifndef BUSCONTROLLER_H
#define BUSCONTROLLER_H

#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class BusController {
private:
public:
    virtual esp_err_t begin() = 0;
    virtual esp_err_t readByte(uint8_t *rx_buffer, uint8_t reg, bool restart = false) = 0;
    virtual esp_err_t readWord(uint8_t *rx_buffer, uint8_t reg, bool restart = false) = 0;
    virtual esp_err_t read(uint8_t *rx_buffer, uint8_t reg, uint8_t len, bool restart = false) = 0;
    virtual esp_err_t writeByte(uint8_t *tx_buffer, uint8_t reg) = 0;
    virtual esp_err_t writeWord(uint8_t *tx_buffer, uint8_t reg) = 0;
    virtual esp_err_t write(uint8_t *tx_buffer, uint8_t reg, uint8_t len) = 0;
};

#endif