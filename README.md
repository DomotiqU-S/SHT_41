| Supported Targets | ESP32 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3

# Temperature Sensor Example

## Overview

This exemple read the temperature and humidity from a DHT40 sensor and print the values to the console.

## How to use example

### Hardware Required

To run this example, you should have one ESP32, ESP32-S, ESP32-C or ESP32-H based development board as well as a DHT40 sensor. 

#### Pin Assignment:

In this example, the DHT40 sensor is connected to the ESP32 as follows:
    SDA pin of DHT40 sensor is connected to GPIO 8 of ESP32-H2.
    SCL pin of DHT40 sensor is connected to GPIO 9 of ESP32-H2.

**Note:** There's no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)
