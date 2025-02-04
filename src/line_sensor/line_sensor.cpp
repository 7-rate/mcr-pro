#include "line_sensor.h"

#if defined( CONFIG_LINE_SENSOR_STEALTH )
#include "stealth.h"
line_sensor_stealth line_sensor_stealth_instance;
line_sensor& ls = line_sensor_stealth_instance;
#elif defined( CONFIG_LINE_SENSOR_D5A2 )
line_sensor_d5a2 line_sensor_d5a2_instance;
line_sensor& ls = line_sensor_d5a2_instance;
#endif