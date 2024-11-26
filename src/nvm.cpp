/*
 * 概要：不揮発メモリに対してparameterの保存と読み込みを行う
 */

#include <CRC32.h>
#include <EEPROM.h>
#include "calibration.h"
#include "nvm.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/

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
// 以下でCRC32が計算できる
// checksum = CRC32::calculate(byteBuffer, numBytes);

/*
 * 概要：引数のcrcをキーとしてEEPROMのアドレスを取得する
 * 引数：crc(descriptionのCRC32)
 * 戻り値：アドレス
 * 詳細：引数のcrcをキーとしてEEPROMのアドレスを取得する
 * 備考：見つからない場合は-1を返す
 */
static s4 get_address( u4 crc ) {
    s4 i;
    u4 temp;

    // 4byte単位でEEPROMを0x200個探索する(0x200 * 4 = 0x800 = 2048byte)
    for ( i = 0; i < 0x200; i++ ) {
        EEPROM.get( i * 4, temp );
        if ( temp == crc ) {
            return i * 4;
        }
    }

    return -1;
}
/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
/*
 * 概要：EEPROMを初期化する
 * 引数：なし
 * 戻り値：true:成功 false:失敗
 * 詳細：EEPROMを初期化する
 * 備考：これにより、全てのデータが0xFFで埋められる
 */
bool nvm_erase() {
    for ( int i = 0; i < 0x200; i++ ) {
        EEPROM.write( i * 4, 0xFF );
    }

    return true;
}

/*
 * 概要：EEPROMからparametersを読み込む
 * 引数：なし
 * 戻り値：true:成功 false:失敗
 * 詳細：descriptionの文字列のCRC32を計算し、それをキーとしてcurrent_valueをEEPROMから読み込む
 * 備考：キーが無ければcurrent_valueは初期値のまま
 */
bool nvm_load() {
    u4 crc;
    s4 address;
    s4 temp;

    for ( int i = 0; i < parameters.size(); i++ ) {
        crc = CRC32::calculate( parameters[i]->get_description(), strlen( parameters[i]->get_description() ) );
        address = get_address( crc );
        if ( address != -1 ) {
            EEPROM.get( address + 4, temp );
            *parameters[i] = temp;
        }
        DBG_PRINT( "description:%s crc:%08X address:%04d value:%04d\n", parameters[i]->get_description(), crc, address, *parameters[i] );
    }

    return true;
}

/*
 * 概要：parametersをEEPROMに保存する
 * 引数：なし
 * 戻り値：true:成功 false:失敗
 * 詳細：descriptionの文字列のCRC32を計算し、それをキーとしてcurrent_valueとともにEEPROMに保存する
 * 備考1：以下のようなデータ構造とする
 *
 * +--------------------+-----------------+--------------------+--------------------+---
 * | parameter1の       | parameter1の    | parameter2の        | parameter2の       |
 * | descriptionのCRC32 | current_value   | descriptionのCRC32  | current_value      |...
 *  <----- 4byte ------> <----- 4byte --> <----- 4byte ------>  <----- 4byte ------>
 *
 * 備考2：この関数は元あるデータは無視して上書きする
 * 備考3：この関数は27レコード当たり約2500msかかる
 * そのうちDataFlashBlockDeviceを直参照するなりしてパフォーマンス改善したほうがいい
 */
bool nvm_save() {
    // nvm_erase(); //べき論ではあったほうがいいが、パフォーマンス上許容できないためコメント
    // 影響があるのはparameterを減らしたとき。増やしたときは問題ない。
    // 減らしたり増やしたりしすぎるとデータが断片化してメモリオーバーするかもしれない(よっぽどないと思うけど...)
    // EEPROM(実際にはDFだが)は8kBあり、1レコードが8byteなので、1024レコードまで保存できる
    // 万が一、問題がでたら一度nvm_eraseを呼び出せば直る(ただし、データは初期値になるので注意)

    u4 crc;
    for ( int i = 0; i < parameters.size(); i++ ) {
        crc = CRC32::calculate( parameters[i]->get_description(), strlen( parameters[i]->get_description() ) );
        EEPROM.put( i * 8, crc );
        EEPROM.put( ( i * 8 ) + 4, parameters[i]->get() );
        DBG_PRINT( "description:%s crc:%08X address:%04d value:%04d\n", parameters[i]->get_description(), crc, i * 4, *parameters[i] );
    }

    return true;
}

/*
 * 概要：4byte単位でダンプする(デバッグ用)
 * 引数：なし
 * 戻り値：true:成功 false:失敗
 * 詳細：4byte単位でダンプする(デバッグ用)
 */
bool nvm_dump() {
    if ( MCR_APP_DEBUG ) {
        DBG_PRINT( "start nvm dump\n" );
        for ( int i = 0; i < 0x200; i++ ) {
            u4 temp;
            EEPROM.get( i * 4, temp );
            DBG_PRINT( "%08X:%08X\n", i, temp );
        }
    }

    return true;
}