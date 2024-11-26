#pragma once
#include <Arduino.h>
#include <SimpleSerialShell.h>
#include "defines.h"
#include "indicator.h"

namespace command_led {
int help() {
    shell.println( F( "===ledコマンドについてのヘルプ===\n"
                      "下記のサブコマンドがあります。\n"
                      "    list\n"
                      "         LEDの種類と設定値を確認できます\n"
                      "         例 : led list\n"
                      "    set\n"
                      "         LEDの状態を設定できます\n"
                      "         例 : led set boardLED 0\n"
                      "         例 : led set neopixel 1\n"
                      "    alloff\n"
                      "         全てのLEDを消灯します\n"
                      "         例 : led alloff\n" ) );
}

int func( int argc, char** argv ) {
    if ( strcmp( (const char*)argv[1], "list" ) == 0 ) {
        shell.println( F( " boardLED" ) );
        shell.println( F( " 0:OFF" ) );
        shell.println( F( " 1:ON" ) );
        shell.println( F( " 2:BLINK" ) );
        shell.println( F( " 3:FAST_BLINK" ) );
        shell.println();

        shell.println( F( " neopixel" ) );
        shell.println( F( " 0:OFF" ) );
        shell.println( F( " 1:GATE_WAITING" ) );
        shell.println( F( " 2:SAVING" ) );
        shell.println( F( " 3:DIFFICULT_ONE_SHOT" ) );
        shell.println( F( " 4:NO_SDCARD" ) );
        shell.println( F( " 5:MOTOR_FAIL" ) );
        shell.println( F( " 6:ERROR_OFF" ) );
    } else if ( strcmp( (const char*)argv[1], "set" ) == 0 ) {
        if ( argc != 4 ) {
            shell.println( F( "引数が足りません\n使い方→led set [led_name] [value]" ) );
            return -1;
        }
        if ( strcmp( (const char*)argv[2], "boardLED" ) == 0 ) {
            u1 value = atoi( (const char*)argv[3] );
            if ( value > 3 ) {
                shell.println( F( "valueが不正です" ) );
                shell.println( F( "valueの範囲は0から3までです" ) );
                return -1;
            }
            indicator_set_board_led( (e_board_led_pattern)value );
        } else if ( strcmp( (const char*)argv[2], "neopixel" ) == 0 ) {
            u1 value = atoi( (const char*)argv[3] );
            if ( value > 6 ) {
                shell.println( F( "valueが不正です" ) );
                shell.println( F( "valueの範囲は0から6までです" ) );
                return -1;
            }
            indicator_set_neopixel_led( (e_neopixel_led_pattern)value );
        } else {
            shell.println( F( "LEDがありません" ) );
            return -1;
        }

    } else if ( strcmp( (const char*)argv[1], "alloff" ) == 0 ) {
        indicator_set_board_led( BOARD_LED_PATTERN_OFF );
        indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_OFF );
        indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_ERROR_OFF );
    } else {
        shell.println( F( "コマンドがありません" ) );
        help();
    }

    return 0;
}

} // namespace command_led