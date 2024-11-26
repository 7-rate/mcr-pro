#pragma once
#include <Arduino.h>
#include <SimpleSerialShell.h>
#include "defines.h"
#include "motor_driver.h"
#include "motor_control.h"

namespace command_motor {
int help() {
    shell.println( F( "===motorコマンドについてのヘルプ===\n"
                      "下記のサブコマンドがあります。\n"
                      "    list\n"
                      "         モーターの種類を確認できます\n"
                      "         例 : motor list\n"
                      "    setpwm\n"
                      "         モーターにpwmを設定します"
                      "         例 : motor setpwm FL 100\n"
                      "              →FLモーターに100のpwmを設定します\n"
                      "    setfreq\n"
                      "         モーターの制御周波数を設定します\n"
                      "         例 : motor setfreq FL 5000\n"
                      "              →FLモーターの制御周波数を5000Hzに設定します\n"
                      "    stop\n"
                      "         全てのモーターを停止します\n"
                      "         例 : motor stop\n" ) );
}

int func( int argc, char** argv ) {
    if ( strcmp( (const char*)argv[1], "list" ) == 0 ) {
        shell.println( " FL" );
        shell.println( " FR" );
        shell.println( " RL" );
        shell.println( " RR" );
    } else if ( strcmp( (const char*)argv[1], "setpwm" ) == 0 ) {
        if ( argc != 4 ) {
            shell.println( F( "引数が足りません\n使い方→motor setpwm [motor] [pwm]" ) );
            return -1;
        }
        s4 pwm = atoi( (const char*)argv[3] );
        if ( -100 > pwm || pwm > 100 ) {
            shell.println( F( "pwmが不正です" ) );
            shell.println( F( "pwmの範囲は-100から100までです" ) );
            return -1;
        }
        if ( strcmp( (const char*)argv[2], "FL" ) == 0 ) {
            motor_FL.set_pwm( pwm );
        } else if ( strcmp( (const char*)argv[2], "FR" ) == 0 ) {
            motor_FR.set_pwm( pwm );
        } else if ( strcmp( (const char*)argv[2], "RL" ) == 0 ) {
            motor_RL.set_pwm( pwm );
        } else if ( strcmp( (const char*)argv[2], "RR" ) == 0 ) {
            motor_RR.set_pwm( pwm );
        } else {
            shell.println( F( "モーターがありません" ) );
        }
    } else if ( strcmp( (const char*)argv[1], "setfreq" ) == 0 ) {
        if ( argc != 4 ) {
            shell.println( F( "引数が足りません\n使い方→motor setfreq [motor] [freq_hz]" ) );
            return -1;
        }
        u4 freq_hz = atoi( (const char*)argv[3] );
        if ( freq_hz < 0 ) {
            shell.println( F( "freq_hzが不正です" ) );
            shell.println( F( "freq_hzは0以上の整数です" ) );
            return -1;
        }
        if ( strcmp( (const char*)argv[2], "FL" ) == 0 ) {
            motor_FL.set_frequency( freq_hz );
        } else if ( strcmp( (const char*)argv[2], "FR" ) == 0 ) {
            motor_FR.set_frequency( freq_hz );
        } else if ( strcmp( (const char*)argv[2], "RL" ) == 0 ) {
            motor_RL.set_frequency( freq_hz );
        } else if ( strcmp( (const char*)argv[2], "RR" ) == 0 ) {
            motor_RR.set_frequency( freq_hz );
        } else {
            shell.println( F( "モーターがありません" ) );
        }
    } else if ( strcmp( (const char*)argv[1], "stop" ) == 0 ) {
        motor_FL.stop();
        motor_FR.stop();
        motor_RL.stop();
        motor_RR.stop();
    } else {
        shell.println( F( "コマンドがありません" ) );
        help();
    }
    return 0;
}

} // namespace command_motor