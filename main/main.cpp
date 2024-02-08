#include "Temp_Sensor.cpp"

extern "C" void app_main(void)
{
    Temp_Sensor *temp_sensor = new Temp_Sensor(0x44,(gpio_num_t)8,(gpio_num_t)9, 100000);
    temp_sensor->read();
    delete temp_sensor;
}

