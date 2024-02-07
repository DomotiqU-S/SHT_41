#include "Temp_Sensor.cpp"

extern "C" void app_main(void)
{
    Temp_Sensor *temp_sensor = new Temp_Sensor(0x44<<1, (gpio_num_t)1, (gpio_num_t)2, 100000);
    temp_sensor->read();
    delete temp_sensor;
}
