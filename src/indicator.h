/*
 * 概要：マイコンボードの単色LED及び、ドライブ基板のNeoPixelLEDを制御する
 */
#pragma once

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
enum e_board_led_pattern {
    BOARD_LED_PATTERN_OFF = 0,
    BOARD_LED_PATTERN_ON,
    BOARD_LED_PATTERN_BLINK,
    BOARD_LED_PATTERN_FAST_BLINK,
};

enum e_neopixel_led_pattern {
    NEOPIXEL_LED_PATTERN_OFF = 0,
    NEOPIXEL_LED_PATTERN_GATE_WAITING,       // 虹色
    NEOPIXEL_LED_PATTERN_SAVING,             // 青点灯
    NEOPIXEL_LED_PATTERN_DIFFICULT_ONE_SHOT, // 0.5秒間緑点灯

    // エラー状態
    NEOPIXEL_LED_PATTERN_NO_SDCARD,  // 黄点滅
    NEOPIXEL_LED_PATTERN_MOTOR_FAIL, // 赤点灯
    NEOPIXEL_LED_PATTERN_ERROR_OFF,  // エラーも消灯
};

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void indicator_init();
void indicator_exec();
void indicator_set_board_led( enum e_board_led_pattern pattern );
void indicator_set_neopixel_led( enum e_neopixel_led_pattern pattern );

/***********************************/
/* Global Variables                */
/***********************************/