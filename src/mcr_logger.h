/*
 * 概要：SdFatライブラリを使用してSDカードにログを書き込む
 */

#pragma once
#include <stdlib.h>
#include <SPI.h>
#include "RingBuf.h"
#include "SdFat.h"
#include "defines.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/

#define RING_BUF_CAPACITY ( 1024 ) // bytes

/***********************************/
/* Class                           */
/***********************************/
class mcr_logger {
  public:
    mcr_logger(){};
    ~mcr_logger(){};
    bool init();
    bool make_log_file();
    void put( const char* str );
    void put_log( const char* str );
    void write_program_info();
    void write_header( const char* header );
    void logging_begin();
    void logging_end();
    /*
     * 概要：ロギングを実施する
     * 引数：なし
     * 戻り値：なし
     * 詳細：SDカードにログを書き込む
     *      1ms周期で呼び出すこと
     *      引数を全てs4型として扱い、カンマ区切りの文字列にして書き込む
     */
    inline void mcr_logger_1ms( const s4* data, size_t len ) {
        if ( logging && !fault ) {
            size_t n = rb.bytesUsed();

            if ( n >= 512 && !file.isBusy() ) {
                if ( 512 != rb.writeOut( 512 ) ) {
                }
            }

            rb.print( micros() );
            rb.print( ',' );
            for ( int i = 0; i < len; i++ ) {
                rb.print( data[i] );
                if ( i != len - 1 ) {
                    rb.print( ',' );
                }
            }
            rb.print( '\n' );
        }
    }
    void ls();
    void cat( const char* filename );
    bool is_fault() {
        return fault;
    }

  private:
    SdFs sd;
    FsFile file;

    RingBuf<FsFile, RING_BUF_CAPACITY> rb;
    size_t maxUsed = 0;
    uint32_t m;

    uint32_t max_time = 0;
    bool fault = false;
    bool logging = false;
};

/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/
