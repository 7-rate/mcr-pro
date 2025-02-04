/*
 * 概要：デジタルセンサ5個、アナログセンサ2個のセンサ構成
 */

#if defined( CONFIG_LINE_SENSOR_D5A2 )
#include <Arduino.h>
#include "defines.h"
#include "line_sensor.h"
#include "calibration.h"
#include "features.h"

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

/***********************************/
/* Class implementions             */
/***********************************/
class line_sensor_d5a2 : public line_sensor {
  public:
    line_sensor_d5a2() {
    }
    ~line_sensor_d5a2() {
    }

    /*
     * 概要：初期化
     * 引数：なし
     * 戻り値：なし
     * 詳細：初期化する
     */
    void init() override {
        pinMode( PIN_LINE_DIGITAL_CENTER, INPUT );
        pinMode( PIN_LINE_DIGITAL_8, INPUT );
        pinMode( PIN_LINE_DIGITAL_4, INPUT );
        pinMode( PIN_LINE_ANALOG_LEFT, INPUT );
        pinMode( PIN_LINE_DIGITAL_GATE, INPUT );
        pinMode( PIN_LINE_ANALOG_RIGHT, INPUT );
        pinMode( PIN_LINE_DIGITAL_2, INPUT );
        pinMode( PIN_LINE_DIGITAL_1, INPUT );
        analogReadResolution( 10 );
    }

    /***********************************/
    /* センサ値の更新                   */
    /***********************************/
    /*
     * 概要：センサ値アップデート
     * 引数：なし
     * 戻り値：なし
     * 詳細：センサ値をアップデートする
     */
    void update() override {
        u1 temp_line_digital = 0;
        temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_CENTER ) << 4;
        temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_8 ) << 3;
        temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_4 ) << 2;
        temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_2 ) << 1;
        temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_1 ) << 0;

        u1 temp_gate = !my_digital_read( PIN_LINE_DIGITAL_GATE );

        line_left_raw = analogRead( PIN_LINE_ANALOG_LEFT );
        line_right_raw = analogRead( PIN_LINE_ANALOG_RIGHT );

        noInterrupts();
        line_digital = temp_line_digital;
        gate = temp_gate;

        line_left = map( line_left_raw, prm_line_trace_left_B.get(), prm_line_trace_left_W.get(), 800, 0 );
        line_right = map( line_right_raw, prm_line_trace_right_B.get(), prm_line_trace_right_W.get(), 800, 0 );
        line_left = constrain( line_left, 0, 800 );
        line_right = constrain( line_right, 0, 800 );

        line_error = line_left - line_right;

        // デジタルセンサが反応していたら上書きする
        switch ( line_digital ) {
        // 右に切れている
        case 0x14:
            line_error = -800;
            break;
        case 0x0C:
            line_error = -900;
            break;
        case 0x08:
            line_error = -1024;
            break;

        // 左に切れている
        case 0x12:
            line_error = 800;
            break;
        case 0x03:
            line_error = 900;
            break;
        case 0x01:
            line_error = 1023;
            break;

        default:
            break;
        }
        interrupts();
    }

    /***********************************/
    /* ライン状態判断                   */
    /***********************************/
    /*
     * 概要：クロスライン判断
     * 引数：デジタルセンサ値
     * 戻り値：true:クロスライン検出
     * 詳細：クロスライン判断
     */
    bool x_line( u1 ld ) override {
        return ( ld == 0x1f ) || ( ld == 0x1e ) || ( ld == 0x17 );
    }

    /*
     * 概要：左ハーフライン判断
     * 引数：デジタルセンサ値
     * 戻り値：true:左ハーフライン検出
     * 詳細：左ハーフライン判断
     */
    bool left_half_line( u1 ld ) override {
        return ( ld == 0x1c ) || ( ld == 0x1e ) || ( ld == 0x0c );
    }

    /*
     * 概要：右ハーフライン判断
     * 引数：デジタルセンサ値
     * 戻り値：true:右ハーフライン検出
     * 詳細：右ハーフライン判断
     */
    bool right_half_line( u1 ld ) override {
        return ( ld == 0x13 ) || ( ld == 0x17 ) || ( ld == 0x03 );
    }

    /*
     * 概要：難所判断
     * 引数：デジタルセンサ値
     * 戻り値：true:難所判断
     * 詳細：難所判断
     */
    bool difficult( u1 ld ) override {
        return x_line( ld ) || left_half_line( ld ) || right_half_line( ld );
    }

    /*
     * 概要：中央付近判断
     * 引数：デジタルセンサ値
     * 戻り値：true:中央付近
     * 詳細：中央付近判断
     */
    bool near_center( u1 ld ) override {
        return ( ld == 0x12 ) || ( ld == 0x10 ) || ( ld == 0x14 );
    }

    /*
     * 概要：ゲート判断
     * 引数：デジタルセンサ値
     * 戻り値：true:ゲート判断
     * 詳細：ゲート判断
     */
    bool get_gate() override {
        return gate;
    }

    /*
     * 概要：全黒判断
     * 引数：デジタルセンサ値
     * 戻り値：true:全黒
     * 詳細：全黒判断
     */
    bool all_black() override {
        return line_digital == 0x00;
    }

    /*
     * 概要：緊急停止パターン判断
     * 引数：デジタルセンサ値
     * 戻り値：true:緊急停止パターン
     * 詳細：緊急停止パターン判断
     */
    bool stop_pattern() override {
        return ( line_digital == 0x1f ) || ( line_digital == 0x1e ) || ( line_digital == 0x1d ) || ( line_digital == 0x1b ) ||
               ( line_digital == 0x17 ) || ( line_digital == 0x19 );
    }

    /***********************************/
    /* 右クランク用判断                  */
    /***********************************/
    /*
     * 概要：右クランク外側ライン検知判断
     * 引数：なし
     * 戻り値：true:右クランク外側ライン検知
     * 詳細：右クランク外側ライン検知判断
     */
    bool right_crank_outline() {
        return ( line_digital == 0x14 ) || ( line_digital == 0x10 ) || ( line_digital == 0x12 ) || ( line_digital == 0x03 ) ||
               ( line_digital == 0x01 );
    }

    /*
     * 概要：右クランク内側ライン検知判断
     * 引数：なし
     * 戻り値：true:右クランク内側ライン検知
     * 詳細：右クランク内側ライン検知判断
     */
    bool right_crank_inline() {
        return ( line_digital == 0x00 ) || ( line_digital == 0x08 ) || ( line_digital == 0x0C ) || ( line_digital == 0x1C );
    }

    /***********************************/
    /* 左クランク用判断                  */
    /***********************************/
    /*
     * 概要：左クランク外側ライン検知判断
     * 引数：なし
     * 戻り値：true:左クランク外側ライン検知
     * 詳細：左クランク外側ライン検知判断
     */
    bool left_crank_outline() {
        return ( line_digital == 0x12 ) || ( line_digital == 0x10 ) || ( line_digital == 0x14 ) || ( line_digital == 0x0C ) ||
               ( line_digital == 0x08 );
    }

    /*
     * 概要：左クランク内側ライン検知判断
     * 引数：なし
     * 戻り値：true:左クランク内側ライン検知
     * 詳細：左クランク内側ライン検知判断
     */
    bool left_crank_inline() {
        return ( line_digital == 0x00 ) || ( line_digital == 0x01 ) || ( line_digital == 0x03 ) || ( line_digital == 0x13 );
    }

    /***********************************/
    /* 右レーンチェンジ用判断            */
    /***********************************/
    /*
     * 概要：右レーンチェンジ用次レーン判断
     * 引数：なし
     * 戻り値：true:右レーンチェンジ用次レーン
     * 詳細：右レーンチェンジ用次レーン判断
     */
    bool right_lanechange_next_lane() {
        return ( line_digital & 0x16 );
    }

    /***********************************/
    /* 左レーンチェンジ用判断            */
    /***********************************/
    /*
     * 概要：左レーンチェンジ用次レーン判断
     * 引数：なし
     * 戻り値：true:左レーンチェンジ用次レーン
     * 詳細：左レーンチェンジ用次レーン判断
     */
    bool left_lanechange_next_lane() {
        return ( line_digital & 0x16 );
    }

  private:
    bool gate;
    u4 line_left_raw;  // アナログセンサーの左側の生値 10bitADCの値 LSB:1[-]
    u4 line_right_raw; // アナログセンサーの右側の生値 10bitADCの値 LSB:1[-]
    s4 line_left;      // アナログセンサーの左側の値(補正後) LSB:1[-]
    s4 line_right;     // アナログセンサーの右側の値(補正後) LSB:1[-]
};

/***********************************/
/* Global functions                */
/***********************************/

#endif // #if defined( CONFIG_LINE_SENSOR_STEALTH )
