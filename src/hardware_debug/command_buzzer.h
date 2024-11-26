#pragma once
#include <Arduino.h>
#include <SimpleSerialShell.h>
#include "defines.h"
#include "buzzer.h"

extern buzzer bz;

namespace command_buzzer {
int help() {
    shell.println( F( "===buzzerコマンドについてのヘルプ===\n"
                      "下記のサブコマンドがあります。\n"
                      "    set\n"
                      "         16進数でブザーパターンをセットできます\n"
                      "         例 : buzzer set 00000003\n" ) );
}

int func( int argc, char** argv ) {
    if ( strcmp( (const char*)argv[1], "set" ) == 0 ) {
        if ( argc != 3 ) {
            shell.println( F( "引数が足りません\n使い方→buzzer set [16進数8桁]" ) );
            return -1;
        }
        u4 bz_battern;
        sscanf( (const char*)argv[2], "%x", &bz_battern );
        bz.set( bz_battern );
    } else {
        shell.println( F( "コマンドがありません" ) );
        help();
    }

    return 0;
}
} // namespace command_buzzer