#pragma once
#include <Arduino.h>
#include <SimpleSerialShell.h>
#include "defines.h"
#include "mcr_logger.h"

extern mcr_logger logger;

namespace command_sd {
int help() {
    shell.println( F( "===sdコマンドについてのヘルプ===\n"
                      "下記のサブコマンドがあります。\n"
                      "    logstart\n"
                      "         ロギングをスタートできます\n"
                      "    logstop\n"
                      "         ロギングをストップできます\n"
                      "    ls\n"
                      "         ログファイルの一覧を表示します\n"
                      "    cat [ファイル名]\n"
                      "         ログファイルの内容を表示します\n" ) );
}

int func( int argc, char** argv ) {
    if ( strcmp( (const char*)argv[1], "logstart" ) == 0 ) {
        logger.make_log_file();
        logger.write_header( "micros,run_mode,run_status,line_error,steer_angle,line_digital,FL,FR,RL,RR,SV,speed,battery_voltage,"
                             "slope_status,speed_stable,speed_slope,speed_curve,speed_crossline,speed_L_lanechange,speed_R_lanechange,"
                             "speed_L_crank,speed_R_crank,failer,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z" );
        logger.logging_begin();
    } else if ( strcmp( (const char*)argv[1], "logstop" ) == 0 ) {
        logger.logging_end();
    } else if ( strcmp( (const char*)argv[1], "ls" ) == 0 ) {
        logger.ls();
    } else if ( strcmp( (const char*)argv[1], "cat" ) == 0 ) {
        if ( argc < 3 ) {
            shell.println( F( "ファイル名を指定してください" ) );
            return 1;
        }
        logger.cat( argv[2] );
    } else {
        shell.println( F( "サブコマンドが間違っています" ) );
        help();
        return 1;
    }

    return 0;
}

} // namespace command_sd