/*
 * 概要：各センサーデータを処理する
 * センサーデータからの生値(1次信号)だけでなく、フィルター処理や微分、積分を行った2次信号も処理する
 */

#include <ezButton.h>
#include <Wire.h>
#include "interval.h"
#include "sensors.h"
#include "calibration.h"
#include "mcr_gpt_lib.h"
#include "line_sensor.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/

/***********************************/
/* Local Variables                 */
/***********************************/
static s4 distance_10um = 0;

/***********************************/
/* Global Variables                */
/***********************************/
s4 encoder_pulse_1ms; // 1msあたりのエンコーダのパルス数 LSB:1[-]
s4 speed_raw;         // エンコーダのパルス数から計算した速度の生値 LSB:0.01[mm/s]
s4 speed;             // 速度(ローパスフィルタ) LSB:0.01[m/s]
s4 distance;          // 走行距離 LSB:1[mm]

s4 temperature; // 温度 LSB:1[degC]

u4 servo_enc_pulse_1ms; // 1msあたりのサーボエンコーダのパルス数 LSB:1[-]
s2 steer_angle;         // ステアリング角度 LSB:0.1[deg]

u4 slope_raw;    // 坂センサーの生値 14bitADCの値 LSB:1[-]
s1 slope_status; // 坂ステータス -1:下り 0:平坦 1:上り

u4 battery_voltage_raw; // バッテリー電圧の生値 14bitADCの値 LSB:1[-]
u4 battery_voltage;     // バッテリー電圧 LSB:0.01[V]

s4 turning_radius;    // 旋回半径 LSB:1[mm]
s4 centrifugal_force; // 旋回時の遠心力 LSB:0.1[N]

dip_switch_t dip_switch; // DIPスイッチのデジタル値 bit0:SW1 bit1:SW2 bit2:SW3 bit3:SW4 bit4:board_sw1 bit5:board_sw2

ezButton button_start( PIN_BUTTON_START );

ezButton button_up( PIN_BUTTON_UP );
ezButton button_down( PIN_BUTTON_DOWN );
ezButton button_right( PIN_BUTTON_RIGHT );
ezButton button_left( PIN_BUTTON_LEFT );
ezButton button_enter( PIN_BUTTON_ENTER );
ezButton button_save( PIN_BUTTON_SAVE );

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/

/*
 * 概要：坂道センサを更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：TODO 閾値の設定をパラメータでできるようにする。
 *      ヒステリシスも設ける
 */
static void slope_update() {
    slope_raw = 0;
    slope_status = 0;
}

/*
 * 概要：バッテリー電圧を更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：バッテリー電圧を更新する
 */
static void battery_update() {
    noInterrupts();
    battery_voltage_raw = analogRead( PIN_BATT_VOLTAGE );
    battery_voltage = battery_voltage_raw * 1400 / 4096; // LSB 0.01V
    interrupts();
}

/*
 * 概要：ディップスイッチを更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：ディップスイッチを更新する
 */
static void dip_switch_update() {
    dip_switch.bit.sw1 = !my_digital_read( PIN_DIPSW_1 );
    dip_switch.bit.sw2 = !my_digital_read( PIN_DIPSW_2 );
    dip_switch.bit.sw3 = !my_digital_read( PIN_DIPSW_3 );
    dip_switch.bit.sw4 = !my_digital_read( PIN_DIPSW_4 );
    dip_switch.bit.board_sw1 = !my_digital_read( PIN_BOARD_DIPSW_1 );
    dip_switch.bit.board_sw2 = !my_digital_read( PIN_BOARD_DIPSW_2 );
}

/*
 * 概要：ボタンの更新
 * 引数：なし
 * 戻り値：なし
 * 詳細：
 */
static void button_update() {
    button_start.loop();
}

/*
 * 概要：旋回半径と遠心力を更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：ステアリング角度から旋回半径を計算する
 *      旋回半径と速度から遠心力を計算する
 *
 *      旋回半径 = 車体長 / tan(ステアリング角度)
 *      遠心力 = (車体重量 * 速度^2) / 旋回半径
 */
static void centrifugal_force_update() {
    // 旋回半径[1mm]
    float steer_angle_rad = radians( static_cast<float>( steer_angle ) / 10.0f );
    float tan_steer_angle = tanf( steer_angle_rad );
    if ( fabs( tan_steer_angle ) < 1e-6 ) {
        turning_radius = INT32_MAX;
    } else {
        float turning_radius_float = static_cast<float>( CAR_LENGTH ) / tan_steer_angle;
        turning_radius = static_cast<s4>( turning_radius_float );
    }

    // 旋回時の遠心力[0.1N]
    s4 temp = ( ( CAR_WEIGHT ) * ( speed * speed ) );
    temp /= turning_radius; // 旋回時の遠心力[0.1N]
    temp /= 1000;
    noInterrupts();
    centrifugal_force = temp;
    interrupts();
}

/***********************************/
/* Global functions                */
/***********************************/
/*
 * 概要：ステアリング角度を更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：ステアリング角度を更新する
 */
void angle_update() {
    s2 servo_pulse_cnt = (s2)GPT7_CNT;
    steer_angle = ( (s4)servo_pulse_cnt * 3600 ) / ( ANGLE_PULSE * CAR_STEER_GEAR_RATIO / 10 );
    steer_angle = -steer_angle;
}

/*
 * 概要：エンコーダの更新
 * 引数：なし
 * 戻り値：なし
 * 詳細：エンコーダの値を更新する
 * 備考：1ms周期で呼び出すこと
 */
void encoder() {
    noInterrupts();
    encoder_pulse_1ms = (s2)GPT6_CNT;
    R_GPT6->GTCR_b.CST = 0;
    GPT6_CNT = 0;
    R_GPT6->GTCR_b.CST = 1;

    speed_raw = ( encoder_pulse_1ms * ENCODER_WHEEL_LENGTH * 1 ) / ( ENCODER_PULSE_PER_REV / 100 );
    speed = speed_raw;
    distance_10um += speed;
    distance = distance_10um / 100;
    interrupts();
}

/*
 * 概要：エンコーダの値をリセットする
 * 引数：なし
 * 戻り値：なし
 * 詳細：距離と角度をリセットする
 * 備考：走行開始時に呼び出すこと
 */
void encoder_reset() {
    distance = 0;
    distance_10um = 0;
}

/*
 * 概要：ボタンの更新
 * 引数：なし
 * 戻り値：なし
 * 詳細：
 */
void button_screen_update() {
    button_up.loop();
    button_down.loop();
    button_right.loop();
    button_left.loop();
    button_enter.loop();
    button_save.loop();
}

/*
 * 概要：センサーの各GPIO、ペリフェラルの初期化
 * 引数：なし
 * 戻り値：なし
 * 詳細：センサーの各GPIO、ペリフェラルの初期化
 */
void sensors_init() {
    // ラインセンサーの初期化
    ls.init();

    // ディップスイッチの初期化
    dip_switch.byte = 0;
    pinMode( PIN_BOARD_DIPSW_1, INPUT_PULLUP );
    pinMode( PIN_BOARD_DIPSW_2, INPUT_PULLUP );
    pinMode( PIN_DIPSW_1, INPUT_PULLUP );
    pinMode( PIN_DIPSW_2, INPUT_PULLUP );
    pinMode( PIN_DIPSW_3, INPUT_PULLUP );
    pinMode( PIN_DIPSW_4, INPUT_PULLUP );

    // エンコーダの初期化
    pinMode( PIN_ENCODER_A, INPUT_PULLUP );
    pinMode( PIN_ENCODER_B, INPUT_PULLUP );
    startGPT6_2SouEncoder( 6, 0, 6, 1 );

    pinMode( PIN_GP_ENC_A, INPUT_PULLUP );
    pinMode( PIN_GP_ENC_B, INPUT_PULLUP );
    startGPT7_2SouEncoder( 3, 3, 3, 4 );

    // バッテリー関連の初期化
    pinMode( PIN_BATT_VOLTAGE, INPUT );
}

/*
 * 概要：センサーの更新
 * 引数：なし
 * 戻り値：なし
 * 詳細：センサーの更新
 */
void sensors_update() {
    ls.update();
    slope_update();
    battery_update();
    dip_switch_update();
    button_update();
    centrifugal_force_update();
}
