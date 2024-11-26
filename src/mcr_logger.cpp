/*
 * 概要：SdFatライブラリを使用してSDカードにログを書き込む
 */

#include <Arduino.h>
#include "mcr_logger.h"
#include "defines.h"
#include "version.h"
#include "calibration.h"
#include "mini-printf.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
const u1 chipSelect = CS1;
const u4 CLOCK_MHZ = 10;
const char* file_name_prefix = "log"; // ex:log0000.csv

#define SD_CONFIG SdSpiConfig( chipSelect, DEDICATED_SPI, SD_SCK_MHZ( CLOCK_MHZ ), &SPI1 )

#define LOG_FILE_SIZE ( 64 * 1000 * 60 ) // 最大60sec

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
/*
 * 概要：SDカードの初期化を行う
 * 引数：log_data:ロギングするデータのテーブル data_num:ログデータの数
 * 戻り値：初期化成功:true 失敗:false
 * 詳細：SDカードの初期化を行う
 */
bool mcr_logger::init() {
    sd.end();
    DBG_PRINT( "SD card init\n" );
    if ( !sd.begin( SD_CONFIG ) ) {
        DBG_PRINT( "sd.begin failed\n" );
        sd.initErrorPrint( &Serial );
        fault = true;
        return false;
    }

    return true;
}

/*
 * 概要：ログファイルを作成する
 * 引数：なし
 * 戻り値：作成成功:true 失敗:false
 * 詳細：csvファイルを作成する
 */
bool mcr_logger::make_log_file() {
    bool ret = true;
    u1 file_number = 0;
    char file_name[12] = { 0 };

    if ( fault ) {
        return false;
    }
    sprintf( file_name, "%s%04d.csv", file_name_prefix, file_number );
    while ( sd.exists( file_name ) ) {
        file_number++;
        sprintf( file_name, "%s%04d.csv", file_name_prefix, file_number );
    }
    DBG_PRINT( "make file_name is :" );
    DBG_PRINT( file_name );
    DBG_PRINT( "\n" );

    if ( !file.open( file_name, O_RDWR | O_CREAT | O_TRUNC ) ) {
        DBG_PRINT( "open failed\n" );
        file.close();
        sd.initErrorPrint( &Serial );
        ret = false;
        goto fail;
    }

    if ( !file.preAllocate( (uint64_t)LOG_FILE_SIZE ) ) {
        DBG_PRINT( "preAllocate failed\n" );
        sd.initErrorPrint( &Serial );
        file.close();
        ret = false;
        goto fail;
    }

    rb.begin( &file );

fail:
    if ( !ret ) {
        fault = true;
    }
    return ret;
}

/*
 * 概要：文字列を書き込む
 * 引数：なし
 * 戻り値：なし
 * 詳細：SDカードに文字列を書き込む。
 */
void mcr_logger::put( const char* str ) {
    if ( !fault ) {
        size_t n = rb.bytesUsed();

        if ( n >= 512 && !file.isBusy() ) {
            if ( 512 != rb.writeOut( 512 ) ) {
            }
        }
        rb.print( str );
    }
}

/*
 * 概要：ロギングモードのときに文字列を書き込む
 * 引数：なし
 * 戻り値：なし
 * 詳細：SDカードに文字列を書き込む。
 */
void mcr_logger::put_log( const char* str ) {
    if ( logging ) {
        put( str );
    }
}

/*
 * 概要：プログラム情報を書き込む
 * 引数：なし
 * 戻り値：なし
 * 詳細：以下情報を書き込む
 *     ビルド日時
 *     Git リビジョン
 *     キャリブレーションパラメータ
 */
void mcr_logger::write_program_info() {
    if ( fault ) {
        return;
    }

    // ビルド日時
    put( "BUILD_DATE\n" );
    put( BUILD_DATE );
    put( "\n\n" );

    // Git リビジョン
    put( "GIT_REVISION\n" );
    put( GIT_REVISION );
    put( "\n\n" );

    // キャリブレーションパラメータ
    put( "PARAMETERS\n" );
    for ( u1 i = 0; i < parameters.size(); i++ ) {
        char buf[128];
        parameter* prm = parameters[i];
        mini_snprintf( buf, sizeof( buf ), "%s,%d,%s\n", prm->get_short_name(), prm->get(), prm->get_description() );
        put( buf );
    }
    put( "\n" );

    // 100行目まで改行で埋める
    u1 padding_line_num =
        100 - ( 3 + 3 + parameters.size() + 1 + 1 ); // ビルド日時(4行)、Gitリビジョン(4行)、パラメータの数、最後の改行(1行)、0始まりなので+1
    for ( u1 i = 0; i < padding_line_num; i++ ) {
        put( "\n" );
    }
}

/*
 * 概要：csvヘッダを作成して書き込む
 * 引数：なし
 * 戻り値：なし
 * 詳細：csvヘッダを作成して書き込む
 */
void mcr_logger::write_header( const char* header ) {
    if ( fault ) {
        return;
    }
    write_program_info();
    put( header );
    put( "\n" );
}

/*
 * 概要：SDカードのファイル一覧を表示する
 * 引数：なし
 * 戻り値：なし
 * 詳細：SDカードのファイル一覧を表示する
 */
void mcr_logger::ls() {
    if ( fault ) {
        return;
    }
    SdFile file;
    SdFile dir;
    dir.open( "/", O_RDONLY );
    while ( file.openNext( &dir, O_RDONLY ) ) {
        file.printName( &Serial );
        if ( file.isDir() ) {
            Serial.print( '/' );
        }
        Serial.println();
        file.close();
    }
    dir.close();
}

/*
 * 概要：SDカードのファイルを表示する
 * 引数：filename:ファイル名
 * 戻り値：なし
 * 詳細：SDカードのファイルを表示する
 */
void mcr_logger::cat( const char* filename ) {
    if ( fault ) {
        return;
    }
    if ( !file.open( filename, O_RDONLY ) ) {
        Serial.println( "open failed" );
        return;
    }

    while ( file.available() ) {
        int c = file.read();
        if ( c < 0 ) {
            break;
        }
        Serial.write( c );
    }

    file.close();
}

/*
 * 概要：ロギング開始
 * 引数：なし
 * 戻り値：タスク作成成功:true 失敗:false
 * 詳細：ロギングタスクを作成する
 */
void mcr_logger::logging_begin() {
    this->logging = true; // loggingはアトミック操作で設定されるので排他処理は不要
}

/*
 * 概要：ロギング終了
 * 引数：なし
 * 戻り値：trueのみ
 * 詳細：ロギングを終了する
 */
void mcr_logger::logging_end() {
    this->logging = false; // loggingはアトミック操作で設定されるので排他処理は不要

    wait_ms( 1000 );

    rb.sync();
    file.truncate();
    file.rewind();

    for ( uint8_t n = 0; n < 500 && file.available(); ) {
        int c = file.read();
        if ( c < 0 ) {
            break;
        }
        Serial.write( c );
        if ( c == '\n' )
            n++;
    }

    file.close();
    // sd.end();
}

/***********************************/
/* Global functions                */
/***********************************/
