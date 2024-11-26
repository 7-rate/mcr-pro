/*
 * 概要：ハードウェアのデバッグを行う
 */

// シリアル通信にて以下のデバッグを行う
// 1. モーターの動作確認
// 2. パラメータの操作
// 3. LEDの動作確認
// 4. バッテリーの確認

#include <Arduino.h>
#include <SimpleSerialShell.h>
#include "defines.h"
#include "motor_driver.h"
#include "calibration.h"
#include "time_measure.h"
#include "sensors.h"
#include "command_led.h"
#include "command_motor.h"
#include "command_parameter.h"
#include "command_buzzer.h"
#include "command_sd.h"

#if defined( F )
#undef F
#endif

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

/***********************************/
/* Global functions                */
/***********************************/
void test_mode_setup() {
    shell.attach( Serial );
    shell.addCommand( F( "param" ), command_parameter::func );
    shell.addCommand( F( "motor" ), command_motor::func );
    shell.addCommand( F( "led" ), command_led::func );
    shell.addCommand( F( "buzzer" ), command_buzzer::func );
    shell.addCommand( F( "sd" ), command_sd::func );
}

void test_mode_main_task() {
    shell.executeIfInput();
}
