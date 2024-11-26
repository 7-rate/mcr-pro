#pragma once
#include <Arduino.h>

static int used;          // ヒストリーの使用数
static int index_history; // ヒストリーのインデックス
static int index_past;    // up/down操作のインデックス
static char command_history[10][88];

static void command_history_init() {
    for ( int i = 0; i < 10; i++ ) {
        for ( int j = 0; j < 88; j++ ) {
            command_history[i][j] = 0;
        }
    }
    used = 0;
    index_history = 0;
    index_past = 0;
}

static void command_history_add( char* command ) {
    used++;
    if ( used > 9 ) {
        used = 9;
    }

    index_history = ( index_history + 1 ) % 10;
    index_past = index_history;

    strcpy( command_history[index_history], command );
}

static void command_history_print() {
    int cnt = 1;

    if ( used == 9 ) {
        for ( int i = index_history + 1; i < 10; i++ ) {
            shell.print( cnt++ );
            shell.print( " : " );
            shell.println( command_history[i] );
        }
        for ( int i = 0; i < index_history; i++ ) {
            shell.print( cnt++ );
            shell.print( " : " );
            shell.println( command_history[i] );
        }
    } else {
        for ( int i = 0; i < used; i++ ) {
            shell.print( cnt++ );
            shell.print( " : " );
            shell.println( command_history[i] );
        }
    }
}

static char* command_history_up() {
    char* ret = command_history[index_past];

    index_past--;
    if ( index_past == index_history ) {
        index_past = ( index_history + 1 ) % 10;
    }
    if ( index_past < 0 ) {
        if ( used == 9 ) {
            index_past = 9;
        } else {
            index_past = 0;
        }
    }

    return ret;
}
static char* command_history_dw() {
    char* ret = command_history[index_past];

    index_past++;
    if ( index_past == ( index_history + 1 ) % 10 ) {
        index_past = index_history;
    }
    if ( index_past > 9 ) {
        index_past = 0;
    }

    return ret;
}