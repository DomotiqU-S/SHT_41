#include "Temp_Sensor.cpp"

extern "C" void app_main(void)
{
    TempSensor *temp_sensor = new TempSensor(0x44,(gpio_num_t)8,(gpio_num_t)9, 100000);
    temp_sensor->read();
    delete temp_sensor;
}

