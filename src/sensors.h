/*
 * 概要：各センサーデータを処理する
 * センサーデータからの生値(1次信号)だけでなく、フィルター処理や微分、積分を行った2次信号も処理する
 */

#pragma once
#include "defines.h"
#include "calibration.h"
#include "ezButton.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/

/***********************************/
/* Class                           */
/***********************************/

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
/* Global functions                */
/***********************************/
extern void angle_update();
extern void encoder();
extern void button_screen_update();
extern void encoder_reset();
extern void sensors_init();
extern void sensors_update();
extern void line_sensor_update();

/***********************************/
/* Global Variables                */
/***********************************/
// センサ関連
extern s4 encoder_pulse_1ms; // 1msあたりのエンコーダのパルス数 LSB:1[-]
extern s4 speed_raw;         // エンコーダのパルス数から計算した速度の生値 LSB:0.01[m/s]
extern s4 speed;             // 速度(ローパスフィルタ) LSB:0.01[m/s]
extern s4 distance;          // 走行距離 LSB:1[mm]

extern s4 temperature; // 温度 LSB:1[degC]

extern u4 servo_enc_pulse_1ms; // 1msあたりのサーボエンコーダのパルス数 LSB:1[-]
extern s2 steer_angle;         // ステアリング角度 LSB:0.1[deg]

extern AnalogSensor ar3; // アナログセンサーの生値 10bitADCの値 LSB:1[-]
extern AnalogSensor ar2; // アナログセンサーの生値 10bitADCの値 LSB:1[-]
extern AnalogSensor ar1; // アナログセンサーの生値 10bitADCの値 LSB:1[-]
extern AnalogSensor ac;  // アナログセンサーの生値 10bitADCの値 LSB:1[-]
extern AnalogSensor al1; // アナログセンサーの生値 10bitADCの値 LSB:1[-]
extern AnalogSensor al2; // アナログセンサーの生値 10bitADCの値 LSB:1[-]
extern AnalogSensor al3; // アナログセンサーの生値 10bitADCの値 LSB:1[-]
extern s4 line_error;    // アナログセンサーの左右差分値 10bit -1024~1023 LSB:1[-]

extern u4 slope_raw;    // 坂センサーの生値 14bitADCの値 LSB:1[-]
extern s1 slope_status; // 坂ステータス -1:下り 0:平坦 1:上り

extern u1 line_digital; // ラインセンサーのデジタル値 bit4:中央 bit3:左 bit2:左中央 bit1:右中央 bit0:右
extern u1 gate;         // ゲートセンサーのデジタル値 bit0:ゲート検知

extern u4 battery_voltage_raw; // バッテリー電圧の生値 14bitADCの値 LSB:1[-]
extern u4 battery_voltage;     // バッテリー電圧 LSB:0.01[V]

extern s4 turning_radius;    // 旋回半径 LSB:1[mm]
extern s4 centrifugal_force; // 旋回時の遠心力 LSB:0.1[N]

extern s4 acc_x;  // 加速度センサーのX軸の値 LSB:0.001[m/s^2]
extern s4 acc_y;  // 加速度センサーのY軸の値 LSB:0.001[m/s^2]
extern s4 acc_z;  // 加速度センサーのZ軸の値 LSB:0.001[m/s^2]
extern s4 gyro_x; // ジャイロセンサーのX軸の値 LSB:0.001[deg/s]
extern s4 gyro_y; // ジャイロセンサーのY軸の値 LSB:0.001[deg/s]
extern s4 gyro_z; // ジャイロセンサーのZ軸の値 LSB:0.001[deg/s]

extern dip_switch_t dip_switch; // DIPスイッチのデジタル値 bit0:SW1 bit1:SW2 bit2:SW3 bit3:SW4

extern ezButton button_start; // スタートボタン
extern ezButton button_up;    // 上ボタン
extern ezButton button_down;  // 下ボタン
extern ezButton button_right; // 右ボタン
extern ezButton button_left;  // 左ボタン
extern ezButton button_enter; // エンターボタン
extern ezButton button_save;  // セーブボタン
