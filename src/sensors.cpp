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

AnalogSensor ar3( &prm_line_AR3_B, &prm_line_AR3_W ); // アナログセンサーの生値 10bitADCの値 LSB:1[-]
AnalogSensor ar2( &prm_line_AR2_B, &prm_line_AR2_W ); // アナログセンサーの生値 10bitADCの値 LSB:1[-]
AnalogSensor ar1( &prm_line_AR1_B, &prm_line_AR1_W ); // アナログセンサーの生値 10bitADCの値 LSB:1[-]
AnalogSensor ac( &prm_line_AC_B, &prm_line_AC_W );    // アナログセンサーの生値 10bitADCの値 LSB:1[-]
AnalogSensor al1( &prm_line_AL1_B, &prm_line_AL1_W ); // アナログセンサーの生値 10bitADCの値 LSB:1[-]
AnalogSensor al2( &prm_line_AL2_B, &prm_line_AL2_W ); // アナログセンサーの生値 10bitADCの値 LSB:1[-]
AnalogSensor al3( &prm_line_AL3_B, &prm_line_AL3_W ); // アナログセンサーの生値 10bitADCの値 LSB:1[-]

s4 line_error;     // アナログセンサーの左右差分値 10bit -1024~1023 LSB:1[-]
s4 line_error_old; // 前回値

u4 slope_raw;    // 坂センサーの生値 14bitADCの値 LSB:1[-]
s1 slope_status; // 坂ステータス -1:下り 0:平坦 1:上り

u1 line_digital; // ラインセンサーのデジタル値 bit4:中央 bit3:左 bit2:左中央 bit1:右中央 bit0:右
u1 gate;         // ゲートセンサーのデジタル値 bit0:ゲート検知

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
 * 概要： デジタルセンサーの値を更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：
 */
static void digital_sensor_update() {
    u1 temp_line_digital = 0;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_CENTER ) << 4;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_8 ) << 3;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_4 ) << 2;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_2 ) << 1;
    // temp_line_digital |= !my_digital_read( PIN_LINE_DIGITAL_1 ) << 0;

    // u1 temp_gate = !my_digital_read( PIN_LINE_DIGITAL_GATE );

    noInterrupts();
    line_digital = temp_line_digital;
    gate = 0;
    interrupts();
}

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
 * 概要： ラインセンサーの値を更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：センサ基板がセンターからどの程度ずれているかを-3072~3072で返す
 */
void line_sensor_update() {
    ar3.push( analogRead( PIN_LINE_AR3 ) );
    ar2.push( analogRead( PIN_LINE_AR2 ) );
    ar1.push( analogRead( PIN_LINE_AR1 ) );
    ac.push( analogRead( PIN_LINE_AC ) );
    al1.push( analogRead( PIN_LINE_AL1 ) );
    al2.push( analogRead( PIN_LINE_AL2 ) );
    al3.push( analogRead( PIN_LINE_AL3 ) );

    s4 sensor_values[7] = { (s4)ar3.corrected(), (s4)ar2.corrected(), (s4)ar1.corrected(), (s4)ac.corrected(),
                            (s4)al1.corrected(), (s4)al2.corrected(), (s4)al3.corrected() };

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

    line_error = gravity_center;
    line_error_old = line_error;
}

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
    pinMode( PIN_LINE_AR3, INPUT );
    pinMode( PIN_LINE_AR2, INPUT );
    pinMode( PIN_LINE_AR1, INPUT );
    pinMode( PIN_LINE_AC, INPUT );
    pinMode( PIN_LINE_AL1, INPUT );
    pinMode( PIN_LINE_AL2, INPUT );
    pinMode( PIN_LINE_AL3, INPUT );
    analogReadResolution( 10 );

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
    digital_sensor_update();
    slope_update();
    battery_update();
    dip_switch_update();
    button_update();
    centrifugal_force_update();
}
