#include "Temp_Sensor.hpp"

extern "C" void app_main(void)
{
    TempSensor temp_sensor(0x44,(gpio_num_t)8,(gpio_num_t)9, 100000);
    temp_sensor.read();
}