/*
 * 概要：ステルスアーム用
 */

#if defined( CONFIG_LINE_SENSOR_STEALTH )
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
#define DIGITAL_THRESHOLD ( 800 )

// アナログセンサの管理クラス
// フィルター処理(移動平均)を行う
class AnalogSensor {
  public:
    u4 raw;

  private:
    constexpr static u1 move_average_size = 3;

  public:
    AnalogSensor( parameter* black, parameter* white ) : black( black ), white( white ) {
        memset( filter_buf, 0, sizeof( filter_buf ) );
        idx = 0;
    }
    void push( u4 raw ) {
        this->raw = raw;
        filter_buf[idx] = raw;
        idx = ( idx + 1 ) % move_average_size;
    }
    u4 get() {
        u4 sum = 0;
        for ( u1 i = 0; i < move_average_size; i++ ) {
            sum += filter_buf[i];
        }
        return sum / move_average_size;
    }
    u4 corrected() {
        s4 temp = map( get(), black->get(), white->get(), 0, 1023 );
        temp = constrain( temp, 0, 1023 );
        return (u4)temp;
    }

  private:
    u4 filter_buf[move_average_size]; // フィルター用バッファ
    u1 idx;
    parameter* black;
    parameter* white;
};

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
static u1 bit_count( u1 value ) {
    value = value - ( ( value >> 1 ) & 0x55 );
    value = ( value & 0x33 ) + ( ( value >> 2 ) & 0x33 );
    return ( value + ( value >> 4 ) ) & 0x0F;
}

/***********************************/
/* Class implementions             */
/***********************************/
class line_sensor_stealth : public line_sensor {
  public:
    line_sensor_stealth() {
        ar3 = new AnalogSensor( &prm_line_AR3_B, &prm_line_AR3_W );
        ar2 = new AnalogSensor( &prm_line_AR2_B, &prm_line_AR2_W );
        ar1 = new AnalogSensor( &prm_line_AR1_B, &prm_line_AR1_W );
        ac = new AnalogSensor( &prm_line_AC_B, &prm_line_AC_W );
        al1 = new AnalogSensor( &prm_line_AL1_B, &prm_line_AL1_W );
        al2 = new AnalogSensor( &prm_line_AL2_B, &prm_line_AL2_W );
        al3 = new AnalogSensor( &prm_line_AL3_B, &prm_line_AL3_W );
    }
    ~line_sensor_stealth() {
        delete ar3;
        delete ar2;
        delete ar1;
        delete ac;
        delete al1;
        delete al2;
        delete al3;
    }

    /*
     * 概要：初期化
     * 引数：なし
     * 戻り値：なし
     * 詳細：初期化する
     */
    void init() override {
        pinMode( PIN_LINE_AR3, INPUT );
        pinMode( PIN_LINE_AR2, INPUT );
        pinMode( PIN_LINE_AR1, INPUT );
        pinMode( PIN_LINE_AC, INPUT );
        pinMode( PIN_LINE_AL1, INPUT );
        pinMode( PIN_LINE_AL2, INPUT );
        pinMode( PIN_LINE_AL3, INPUT );
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
        ar3->push( analogRead( PIN_LINE_AR3 ) );
        ar2->push( analogRead( PIN_LINE_AR2 ) );
        ar1->push( analogRead( PIN_LINE_AR1 ) );
        ac->push( analogRead( PIN_LINE_AC ) );
        al1->push( analogRead( PIN_LINE_AL1 ) );
        al2->push( analogRead( PIN_LINE_AL2 ) );
        al3->push( analogRead( PIN_LINE_AL3 ) );

        s4 sensor_values[7] = { (s4)ar3->corrected(), (s4)ar2->corrected(), (s4)ar1->corrected(), (s4)ac->corrected(),
                                (s4)al1->corrected(), (s4)al2->corrected(), (s4)al3->corrected() };

        // 山の数を数える
        // 下がったところで山としてカウントする
        u1 mount_count = 0;
        s1 trend = 0; // -1:下り 0:平坦 1:上り
        s1 trend_old = 0;
        if ( sensor_values[0] > 0 ) {
            mount_count++;
            trend_old = 1;
        }
        for ( u1 i = 1; i < 7; i++ ) { // 1始まりであることに注意
            trend = ( sensor_values[i - 1] < sensor_values[i] ) ? 1 : ( sensor_values[i - 1] > sensor_values[i] ) ? -1 : 0;
            // 上りの後に下りになったら山とする
            if ( ( trend_old == 0 || trend_old == -1 ) && trend == 1 ) {
                mount_count++;
            }

            trend_old = trend;
        }

        s4 gravity_center;
        s4 bunshi; // 分子
        s4 bunbo;  // 分母
        s4 pivot_value;
        // 山の数によって処理を変える
        switch ( mount_count ) {
        case 2: {
            // 山が2つある場合は前回値を軸(pivot)とし、軸からの距離で重みを付けて重心を求めるようにする
            // まずはpivotを求める
            u1 pivot = 0;
            u4 diff_pivot_min = UINT32_MAX;
            pivot_value = 3072;
            for ( u1 i = 0; i < 7; i++, pivot_value -= 1024 ) {
                u4 diff_pivot = abs( pivot_value - line_error_old );
                if ( diff_pivot < diff_pivot_min ) {
                    pivot = i;
                    diff_pivot_min = diff_pivot;
                }
            }

            // pivotを100%として離れているところは以下のように重みを付ける
            // 0(pivot) : 100%
            // ±1 : 90%
            // ±2 : 75%
            // ±3 : 20%
            // ±4 : 0%
            // ±5 : 0%
            // ±6 : 0%
            for ( u1 i = 0; i < 7; i++ ) {
                u1 diff = abs( pivot - i );
                if ( diff == 0 ) {
                    ; // 100%
                } else if ( diff == 1 ) {
                    sensor_values[i] = ( sensor_values[i] * 90 ) / 100;
                } else if ( diff == 2 ) {
                    sensor_values[i] = ( sensor_values[i] * 75 ) / 100;
                } else if ( diff == 3 ) {
                    sensor_values[i] = ( sensor_values[i] * 20 ) / 100;
                } else {
                    sensor_values[i] = 0;
                }
            }
            /* fall-through */
        }
        case 1: {
            // 重心を求める
            bunshi = 0;
            bunbo = 0;
            pivot_value = 3072;
            for ( u1 i = 0; i < 7; i++, pivot_value -= 1024 ) {
                bunshi += ( sensor_values[i] * pivot_value );
                bunbo += sensor_values[i];
            }
            if ( bunbo != 0 ) {
                gravity_center = bunshi / bunbo;
            } else {
                gravity_center = 0;
            }

            break;
        }
        default:
            // 山が0もしくは3以上の場合は難所の恐れがある。その場合はステアリングを変に操作したくないため0を返す。
            gravity_center = 0;
            break;
        }

        noInterrupts();
        line_error = gravity_center;
        line_error_old = line_error;

        for ( u1 i = 0; i < 7; i++ ) {
            line_digital |= ( sensor_values[i] > DIGITAL_THRESHOLD ) << ( 6 - i );
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
        // xxxxxxx
        return bit_count( ld ) >= 6;
    }

    /*
     * 概要：左ハーフライン判断
     * 引数：デジタルセンサ値
     * 戻り値：true:左ハーフライン検出
     * 詳細：左ハーフライン判断
     */
    bool left_half_line( u1 ld ) override {
        // xxxxx--
        return bit_count( ld & 0x7C ) >= 4;
    }

    /*
     * 概要：右ハーフライン判断
     * 引数：デジタルセンサ値
     * 戻り値：true:右ハーフライン検出
     * 詳細：右ハーフライン判断
     */
    bool right_half_line( u1 ld ) override {
        // --xxxxx
        return bit_count( ld & 0x1F ) >= 4;
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
        // --xxx--
        return bit_count( ld & 0x1C ) >= 0;
    }

    /*
     * 概要：ゲート判断
     * 引数：デジタルセンサ値
     * 戻り値：true:ゲート判断
     * 詳細：ゲート判断
     */
    bool get_gate() override {
        // 例: すべてのセンサがラインを検出していればゲートと判断
        return ( line_digital == 0xFF );
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
        return bit_count( line_digital ) >= 6;
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
        // xxx----
        return line_digital & 0x70;
    }

    /*
     * 概要：右クランク内側ライン検知判断
     * 引数：なし
     * 戻り値：true:右クランク内側ライン検知
     * 詳細：右クランク内側ライン検知判断
     */
    bool right_crank_inline() {
        // ----xxx
        return ( line_digital & 0x07 );
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
        // ----xxx
        return ( line_digital & 0x07 );
    }

    /*
     * 概要：左クランク内側ライン検知判断
     * 引数：なし
     * 戻り値：true:左クランク内側ライン検知
     * 詳細：左クランク内側ライン検知判断
     */
    bool left_crank_inline() {
        // xxx----
        return line_digital & 0x70;
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
        // ----xxx
        return ( line_digital & 0x07 );
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
        // xxx----
        return line_digital & 0x70;
    }

  public:
    AnalogSensor* ar3;
    AnalogSensor* ar2;
    AnalogSensor* ar1;
    AnalogSensor* ac;
    AnalogSensor* al1;
    AnalogSensor* al2;
    AnalogSensor* al3;

  private:
    s4 line_error_old;
};

/***********************************/
/* Global functions                */
/***********************************/

#endif // #if defined( CONFIG_LINE_SENSOR_STEALTH )
