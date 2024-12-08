/*
 * 概要：マイコンボードの単色LED及び、ドライブ基板のNeoPixelLEDを制御する
 */
#include <Arduino.h>
#include <FastLED.h>
#include "indicator.h"
#include "defines.h"
#include "pin_defines.h"
#include "interval.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
#define NEOPIXEL_LEDS ( 2 )

/***********************************/
/* Local Variables                 */
/***********************************/
// ボードLED
static enum e_neopixel_led_pattern neopixel_led_pattern = NEOPIXEL_LED_PATTERN_OFF;
static PinStatus board_led_status = LOW; // 点滅時の状態を保持

// NeoPixelLED
static enum e_board_led_pattern board_led_pattern = BOARD_LED_PATTERN_OFF;
static CRGB neopixel[NEOPIXEL_LEDS];
static u4 one_shot_tmr = 0;
static bool neopixel_blink_flag = LOW; // 点滅時の状態を保持

/***********************************/
/* Global Variables                */
/***********************************/

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
static void board_led_exec() {
    static uint32_t prev_time = 0;
    uint32_t current_time = millis();

    EVERY_MS( 33 ) {
        switch ( board_led_pattern ) {
        case BOARD_LED_PATTERN_OFF:
            digitalWrite( PIN_BOARD_LED_D3, HIGH );
            break;
        case BOARD_LED_PATTERN_ON:
            digitalWrite( PIN_BOARD_LED_D3, LOW );
            break;
        case BOARD_LED_PATTERN_BLINK: // 500ms周期
            if ( current_time - prev_time > 500 ) {
                digitalWrite( PIN_BOARD_LED_D3, board_led_status );
                board_led_status = board_led_status == LOW ? HIGH : LOW;
                prev_time = current_time;
            }
            break;
        case BOARD_LED_PATTERN_FAST_BLINK: // 100ms周期
            if ( current_time - prev_time > 100 ) {
                digitalWrite( PIN_BOARD_LED_D3, board_led_status );
                board_led_status = board_led_status == LOW ? HIGH : LOW;
                prev_time = current_time;
            }
            break;
        default:
            break;
        }
    }
    END_EVERY_MS
}

static void neopixel_exec() {
    static u1 hue = 0;
    static uint32_t prev_time = 0;

    EVERY_MS( 33 ) {
        uint32_t current_time = millis();
        switch ( neopixel_led_pattern ) {
        case NEOPIXEL_LED_PATTERN_GATE_WAITING:
            hue = ( hue + 8 ) % 256;
            neopixel[0] = CHSV( hue, 255, 63 );
            break;
        case NEOPIXEL_LED_PATTERN_SAVING:
            neopixel[0] = CHSV( 240, 255, 63 );
            break;
        case NEOPIXEL_LED_PATTERN_DIFFICULT_ONE_SHOT:
            neopixel[0] = CHSV( 120, 255, 255 );
            if ( millis() - one_shot_tmr > 500 ) {
                neopixel_led_pattern = NEOPIXEL_LED_PATTERN_OFF;
            }
            break;
        case NEOPIXEL_LED_PATTERN_NO_SDCARD:
            if ( current_time - prev_time > 500 ) {
                neopixel[1] = neopixel_blink_flag ? CHSV( 60, 255, 63 ) : CHSV( 0, 0, 0 );
                neopixel_blink_flag = !neopixel_blink_flag;
                prev_time = current_time;
            }
            break;
        case NEOPIXEL_LED_PATTERN_MOTOR_FAIL:
            neopixel[1] = CHSV( 0, 255, 63 );
            break;
        case NEOPIXEL_LED_PATTERN_ERROR_OFF:
            neopixel[1] = CHSV( 0, 0, 0 );
            break;
        default:
        case NEOPIXEL_LED_PATTERN_OFF:
            neopixel[0] = CHSV( 0, 0, 0 );
            break;
        }
        FastLED.show();
    }
    END_EVERY_MS
}

/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void indicator_init() {
    pinMode( PIN_BOARD_LED_D3, OUTPUT );
    digitalWrite( PIN_BOARD_LED_D3, HIGH );
    board_led_status = HIGH;

    FastLED.addLeds<WS2812, PIN_NEOPIXEL, RGB>( neopixel, NEOPIXEL_LEDS );

    indicator_set_board_led( BOARD_LED_PATTERN_OFF );
    indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_OFF );
    indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_ERROR_OFF );
}

void indicator_exec() {
    board_led_exec();
    neopixel_exec();
}

void indicator_set_board_led( enum e_board_led_pattern pattern ) {
    if ( BOARD_LED_PATTERN_OFF <= pattern && pattern <= BOARD_LED_PATTERN_FAST_BLINK ) {
        board_led_pattern = pattern;
    }
}

void indicator_set_neopixel_led( enum e_neopixel_led_pattern pattern ) {
    if ( NEOPIXEL_LED_PATTERN_OFF <= pattern && pattern <= NEOPIXEL_LED_PATTERN_MOTOR_FAIL ) {
        neopixel_led_pattern = pattern;
        one_shot_tmr = millis();
    }
}