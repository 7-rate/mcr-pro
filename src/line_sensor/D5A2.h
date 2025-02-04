/*
 * 概要：デジタルセンサ5個、アナログセンサ2個のセンサ構成用
 */

#if defined( CONFIG_LINE_SENSOR_D5A2 )
#include <Arduino.h>
#include "defines.h"
#include "line_sensor.h"
#include "features.h"
/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/

/***********************************/
/* Local Variables                 */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/

/***********************************/
/* Class implementions             */
/***********************************/
void line_sensor_d5a2::update() {
    u1 temp_line_digital = 0;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_CENTER ) << 4;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_8 ) << 3;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_4 ) << 2;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_2 ) << 1;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_1 ) << 0;

    // u1 temp_gate = !my_digital_read( PIN_LINE_DIGITAL_GATE );

    noInterrupts();
    line_digital = temp_line_digital;
    gate = 0;
    interrupts();
}

/***********************************/
/* Global functions                */
/***********************************/

#endif // #if defined( CONFIG_LINE_SENSOR_D5A2 )