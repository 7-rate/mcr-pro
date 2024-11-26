#pragma once
#include <Arduino.h>
#include <SimpleSerialShell.h>
#include "defines.h"
#include "nvm.h"
#include "calibration.h"

namespace command_parameter {
int help() {
    shell.println( F( "===paramコマンドについてのヘルプ===\n"
                      "下記のサブコマンドがあります。\n"
                      "    list\n"
                      "         parametersの一覧を確認できます\n"
                      "         例 : param list\n"
                      "    set\n"
                      "         parametersに値を設定できます\n"
                      "         例 : param set 0 105\n"
                      "              →parameters[0]に105を設定します\n"
                      "    load\n"
                      "         nvm_loadを実行します\n"
                      "         例 : param load\n"
                      "    save\n"
                      "         nvm_saveを実行します\n"
                      "         例 : param save\n"
                      "    clear\n"
                      "         nvm_eraseを実行します\n"
                      "         例 : param clear\n" ) );
}

int func( int argc, char** argv ) {
    if ( strcmp( (const char*)argv[1], "list" ) == 0 ) {
        for ( int i = 0; i < parameters.size(); i++ ) {
            shell.print( i );
            shell.print( " : " );
            shell.print( parameters[i]->get_short_name() );
            shell.print( " = " );
            shell.println( parameters[i]->get() );
        }
    } else if ( strcmp( (const char*)argv[1], "set" ) == 0 ) {
        if ( argc != 4 ) {
            shell.println( F( "引数が足りません\n使い方→param set [parameter_No] [value]" ) );
            return -1;
        }
        u4 parameter_No = atoi( (const char*)argv[2] );
        s4 value = atoi( (const char*)argv[3] );
        if ( 0 > parameter_No || parameter_No >= parameters.size() ) {
            shell.println( F( "parameter_Noが不正です" ) );
            shell.println( "parameter_Noの範囲は0から" + String( parameters.size() - 1 ) + "です" );
            return -1;
        }
        *parameters[parameter_No] = value;
    } else if ( strcmp( (const char*)argv[1], "load" ) == 0 ) {
        nvm_load();
    } else if ( strcmp( (const char*)argv[1], "save" ) == 0 ) {
        shell.println( F( "start save" ) );
        nvm_save();
        shell.println( F( "end save" ) );
    } else if ( strcmp( (const char*)argv[1], "clear" ) == 0 ) {
        shell.println( F( "start clear" ) );
        nvm_erase();
        shell.println( F( "end clear" ) );
    } else {
        shell.println( F( "コマンドがありません" ) );
        help();
    }
    return 0;
}

} // namespace command_parameter