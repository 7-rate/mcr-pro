/*
 * 概要：マイコンカーの走行に関する制御を行う
 */

#include <Arduino.h>
#include <LibPrintf.h>
#include <FspTimer.h>
#include "defines.h"
#include "calibration.h"
#include "time_measure.h"
#include "distance_measure.h"
#include "sensors.h"
#include "hardware_debug/test_mode_main.h"
#include "mcr_logger.h"
#include "buzzer.h"
#include "nvm.h"
#include "calc_utils.h"
#include "mcr_gpt_lib.h"
#include "screen.h"
#include "target_speed.h"
#include "mini-printf.h"
#include "indicator.h"
#include "interval.h"
#include "motor_control.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/*********************************************************/
/* Local definitions                                     */
/*********************************************************/
/*カーブ回転比*/
static const u1 revolution_difference_fin[] = { // 前輪(内輪)の回転比
    100, 100, 100, 98, 97, 96, 95, 95, 94, 94, 93, 93, 92, 90, 89, 88, 88, 87, 86, 85, 84, 84, 83,
    83,  82,  82,  82, 82, 82, 82, 81, 81, 81, 81, 81, 80, 80, 80, 80, 80, 80, 78, 78, 78, 78 };

static const u1 revolution_difference_rout[] = { // 後輪(外輪)の回転比
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 99, 98, 98, 98, 97, 97, 97, 96, 96, 94, 94,
    93,  93,  91,  91,  91,  90,  90,  90,  90,  90,  90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90 };

static const u1 revolution_difference_rin[] = { // 後輪(内輪)の回転比
    100, 100, 100, 98, 97, 95, 93, 91, 90, 89, 87, 86, 84, 82, 80, 79, 78, 77, 76, 75, 74, 74, 74, 50,
    50,  50,  50,  50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };

/*********************************************************/
/* Local Variables                                       */
/*********************************************************/
// 1ms周期タイマー
FspTimer fsp_timer;

// 停止判断関連
time_measure check_stop_timer;                          // 停止判断用タイマー
distance_measure check_stop_comp_distance( &distance ); // 停止判断用走行距離計測(完走用)
distance_measure check_stop_fail_distance( &distance ); // 停止判断用走行距離計測(あり得ないデジタルセンサパターン用)

// 走行制御関連
u1 run_mode = RUN_STOP;
u1 run_status = S00;
u1 line_digital_sum = 0;                    // 難所判定用ラインセンサーのデジタル値の論理和
distance_measure dist_measure( &distance ); // 走行中の制御に使う走行距離計測クラス
time_measure timer_start_mode_timer;        // タイマスタートモード用タイマー
time_measure running_timer;                 // 走行時間計測用タイマー

// ロガー関連
mcr_logger logger;
s4 log_interval;
s4 target_speed_now;

/*********************************************************/
/* Global Variables                                      */
/*********************************************************/
// ブザー関連
buzzer bz( PIN_BUZZER );

// failer
u4 failer = 0x00;

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/*********************************************************/
/* Local functions                                       */
/*********************************************************/
/*
 * 概要：前外側のpwmから前内側の駆動配分を取得する
 * 引数：front_out:-100~100 (負の値は逆転)
 *       angle:ステアリング角度(LSB:0.1deg)
 * 戻り値：前内側の駆動配分pwm
 * 詳細：前外側のpwmから前内側の駆動配分を取得する
 */
static s4 front_in( s4 front_out, s4 angle ) {
    s4 i, ret;
    i = abs( angle );
    i = min( i, 449 ); // revolution_difference_finでの配列外参照防止
    i /= 10;           // LSB変換 0.1deg -> 1deg
    ret = revolution_difference_fin[i] * front_out / 100;
    return ret;
}

/*
 * 概要：前外側のpwmから後内側の駆動配分を取得する
 * 引数：front_out:-100~100 (負の値は逆転)
 *       angle:ステアリング角度(LSB:0.1deg)
 * 戻り値：後内側の駆動配分pwm
 * 詳細：前外側のpwmから後内側の駆動配分を取得する
 */
static s4 rear_in( s4 front_out, s4 angle ) {
    s4 i, ret;
    i = abs( angle );
    i = min( i, 449 ); // revolution_difference_finでの配列外参照防止
    i /= 10;           // LSB変換 0.1deg -> 1deg
    ret = revolution_difference_rin[i] * front_out / 100;
    return ret;
}

/*
 * 概要：前外側のpwmから後外側の駆動配分を取得する
 * 引数：front_out:-100~100 (負の値は逆転)
 *       angle:ステアリング角度(LSB:0.1deg)
 * 戻り値：後外側の駆動配分pwm
 * 詳細：前外側のpwmから後外側の駆動配分を取得する
 */
static s4 rear_out( s4 front_out, s4 angle ) {
    s4 i, ret;
    i = abs( angle );
    i = min( i, 449 ); // revolution_difference_finでの配列外参照防止
    i /= 10;           // LSB変換 0.1deg -> 1deg
    ret = revolution_difference_rout[i] * front_out / 100;
    return ret;
}

/*
 * 概要：ログ用のtarget_speedを設定する
 * 引数：target_speed
 * 戻り値：なし
 * 詳細：ログ用のtarget_speedを設定する
 */
static s4 set_target_speed_now( s4 target_speed ) {
    target_speed_now = target_speed;
}

/*
 * 概要：定常走行制御時の速度制御
 * 引数：目標速度(LSB:0.01m/s)
 * 戻り値：なし
 * 詳細：定常走行制御時にステアリング角度と速度に応じたpwm値を設定する
 *       速度制御はP(ID)制御を行う
 */
static void spdctrl_stable( s4 target_speed ) {
    s4 error;
    s4 front_out_temp;
    s4 front_out, fin, rout, rin;

    set_target_speed_now( target_speed );
    error = target_speed - speed;

    front_out_temp = error * prm_speed_stable_P.get() / prm_speed_stable_P.get_lsb();
    front_out_temp = constrain( front_out_temp, -100, 100 );
    front_out = front_out_temp;
    fin = front_in( front_out, steer_angle );
    rout = rear_out( front_out, steer_angle );
    rin = rear_in( front_out, steer_angle );

    motor_mode( BRAKE, BRAKE, BRAKE, BRAKE );
    if ( steer_angle >= 0 ) {
        motor_pwm( front_out, fin, rout, rin );
    } else {
        motor_pwm( fin, front_out, rin, rout );
    }
}

/*
 * 概要：急カーブ走行制御時の速度制御
 * 引数：目標速度(LSB:0.01m/s)
 * 戻り値：なし
 * 詳細：急カーブ走行時、後輪内側をブレーキする
 *      速度がspeed_curve*150%のときに-100%のフルブレーキとし、speed_curve*100%のときに定常走行制御と同じ駆動配分となるようにする。
 *      pwm指令が連続するように、速度による線形補完で後輪内側の駆動配分を決定する。
 *      他の駆動輪は定常走行制御と同じ。
 */
static void spdctrl_sharp_curve( s4 target_speed ) {
    s4 error;
    s4 front_out_temp;
    s4 front_out, fin, rout, rin;
    s4 rout_stable;
    s4 rout_max_brake;

    set_target_speed_now( target_speed );
    error = target_speed - speed;

    front_out_temp = error * prm_speed_stable_P.get() / prm_speed_stable_P.get_lsb();
    front_out_temp = constrain( front_out_temp, -100, 100 );
    front_out = front_out_temp;
    fin = front_in( front_out, steer_angle );
    rin = rear_in( front_out, steer_angle );

    rout_stable = rear_out( front_out, steer_angle );
    rout_max_brake = -100;
    rout = map( speed, speed_curve * 100 / 100, speed_curve * 150 / 100, rout_stable, rout_max_brake );
    rout = constrain( rout, rout_max_brake, rout_stable );

    motor_mode( BRAKE, BRAKE, BRAKE, BRAKE );
    if ( steer_angle >= 0 ) {
        motor_pwm( front_out, fin, rout, rin );
    } else {
        motor_pwm( fin, front_out, rin, rout );
    }
}

/*
 * 概要：クランク走行制御時の速度制御
 * 引数：目標速度(LSB:0.01m/s)
 * 戻り値：なし
 * 詳細：クランク走行時、後輪内側をブレーキする
 *      速度がspeed_L(R)_crank*150%のときに-100%のフルブレーキとし、speed_L(R)_crank*100%のときに定常走行制御と同じ駆動配分となるようにする。
 *      pwm指令が連続するように、速度による線形補完で後輪内側の駆動配分を決定する。
 *      他の駆動輪は定常走行制御と同じ。
 */
static void spdctrl_crank( s4 target_speed ) {
    s4 error;
    s4 front_out_temp;
    s4 front_out, fin, rout, rin;
    s4 rout_stable;
    s4 rout_max_brake;

    set_target_speed_now( target_speed );
    error = target_speed - speed;

    front_out_temp = error * prm_speed_stable_P.get() / prm_speed_stable_P.get_lsb();
    front_out_temp = constrain( front_out_temp, -100, 100 );
    front_out = front_out_temp;
    fin = front_in( front_out, steer_angle );
    rin = rear_in( front_out, steer_angle );

    rout_stable = rear_out( front_out, steer_angle );
    rout_max_brake = -100;
    rout = map( speed, speed_curve * 100 / 100, speed_curve * 150 / 100, rout_stable, rout_max_brake );
    rout = constrain( rout, rout_max_brake, rout_stable );

    motor_mode( BRAKE, BRAKE, BRAKE, BRAKE );
    if ( steer_angle >= 0 ) {
        motor_pwm( front_out, fin, rout, rin );
    } else {
        motor_pwm( fin, front_out, rin, rout );
    }
}

/*
 * 概要：レーンチェンジ走行制御時の速度制御
 * 引数：target_speed:目標速度(LSB:0.01m/s)
 *       target_angle:目標ステアリング角度(LSB:0.1deg)
 * 戻り値：なし
 * 詳細：現在ステアリング角度ではなく、目標ステアリング角度に応じた駆動配分を行う
 *       速度制御のP(ID)ゲインは定常走行制御と同じ
 */
static void spdctrl_lane_change( s4 target_speed, s4 target_angle ) {
    s4 error;
    s4 front_out_temp;
    s4 front_out, fin, rout, rin;

    set_target_speed_now( target_speed );
    error = target_speed - speed;

    front_out_temp = error * prm_speed_stable_P.get() / prm_speed_stable_P.get_lsb();
    front_out_temp = constrain( front_out_temp, -100, 100 );
    front_out = front_out_temp;
    fin = front_in( front_out, target_angle );
    rout = rear_out( front_out, target_angle );
    rin = rear_in( front_out, target_angle );

    motor_mode( BRAKE, BRAKE, BRAKE, BRAKE );
    if ( target_angle >= 0 ) {
        motor_pwm( front_out, fin, rout, rin );
    } else {
        motor_pwm( fin, front_out, rin, rout );
    }
}

/*****************************************************/
/* Running Judgement functions                       */
/*****************************************************/
/*
 * 概要：停止判断
 * 引数：なし
 * 戻り値：true:走行停止、false:走行停止しない
 * 詳細：以下のいずれかの場合に停止判断する。
 *      1.(ありえないデジタルセンサパターン or 車速無し) が500ms以上継続
 *      2.(ありえないデジタルセンサパターン or 車速無し) のまま100mm以上走行
 *      3.停止距離以上走行完了
 */
bool run_judge_check_stop() {
    bool stop;
    bool pre_stop_flag;

    stop = false;
    if ( running_timer.measure() > 500 ) {
        pre_stop_flag = false;
        if ( ( line_digital == 0x1f ) || ( line_digital == 0x1e ) || ( line_digital == 0x1d ) || ( line_digital == 0x1b ) ||
             ( line_digital == 0x17 ) || ( line_digital == 0x19 ) || ( speed <= 10 ) ) {
            pre_stop_flag = true;
        }

        if ( pre_stop_flag == false ) {
            check_stop_timer.restart();
            check_stop_fail_distance.restart();
        }

        if ( ( check_stop_timer.measure() > 500 )                                           // 500ms以上継続
             || ( check_stop_fail_distance.measure() > 100 )                                // 100mm以上走行
             || ( check_stop_comp_distance.measure() > prm_stop_distance.get() * 1000 ) ) { // 停止距離以上走行完了
            stop = true;
        }
    }
    return stop;
}

/*
 * 概要：左ハーフライン判断
 * 引数：デジタルセンサ値
 * 戻り値：true:左ハーフライン検出
 * 詳細：左ハーフライン判断
 */
bool run_judge_left_half_line( u1 ld ) {
    return ( ld == 0x1c ) || ( ld == 0x1e ) || ( ld == 0x0c );
}

/*
 * 概要：右ハーフライン判断
 * 引数：デジタルセンサ値
 * 戻り値：true:右ハーフライン検出
 * 詳細：右ハーフライン判断
 */
bool run_judge_right_half_line( u1 ld ) {
    return ( ld == 0x13 ) || ( ld == 0x17 ) || ( ld == 0x03 );
}

/*
 * 概要：クロスライン判断
 * 引数：デジタルセンサ値
 * 戻り値：true:クロスライン検出
 * 詳細：クロスライン判断
 */
bool run_judge_cross_line( u1 ld ) {
    // 0x1eや0x17の場合はアナログセンサも見て判断したほうがいいかもしれない
    return ( ld == 0x1f ) || ( ld == 0x1e ) || ( ld == 0x17 );
}

/*
 * 概要：難所判断
 * 引数：デジタルセンサ値
 * 戻り値：true:難所判断
 * 詳細：難所判断
 */
bool run_judge_difficult( u1 ld ) {
    return run_judge_left_half_line( ld ) || run_judge_right_half_line( ld ) || run_judge_cross_line( ld );
}

/*
 * 概要：急カーブ判断
 * 引数：なし
 * 戻り値：true:急カーブ判断
 * 詳細：遠心力(centrifugal_force)が一定以上の場合に急カーブと判断
 */
bool run_judge_sharp_curve() {
    return ( centrifugal_force >= prm_sharp_curve_force.get() );
}

/*
 * 概要：急カーブ終了判断
 * 引数：なし
 * 戻り値：true:急カーブ終了判断
 * 詳細：遠心力(centrifugal_force)が一定以下の場合に急カーブ終了と判断
 */
bool run_judge_end_sharp_curve() {
    return ( centrifugal_force <= (s4)( prm_sharp_curve_force.get() * 0.9 ) );
}

/***************************************************/
/* Running Control Common functions                */
/***************************************************/
/*
 * 概要：run_modeを変更する
 * 引数：mode:変更後のrun_mode
 *      dist_measure_reset:走行距離計測をリセットするかどうか(デフォルト：リセットする)
 *     run_status_reset:run_statusをリセットするかどうか(デフォルト：リセットする)
 * 戻り値：なし
 * 詳細：run_modeを変更する
 */
void run_mode_change_to( enum e_run_mode mode, bool dist_measure_reset = true, bool run_status_reset = true ) {
    run_mode = mode;
    run_status = S00;
    if ( dist_measure_reset ) {
        dist_measure.restart();
    }
}

/***************************************************/
/* Running Control functions                       */
/***************************************************/
/* RUN_STOP
 * 概要：スタート前停止状態
 * 引数：なし
 * 戻り値：なし
 * 詳細：スタートボタンが押されたらロギングタスクへ通知し、ロギング準備が出来たらPRE_STARTへ遷移する
 */
void running_stop() {
    set_servo_mode( STOP );
    motor_pwm( 0, 0, 0, 0 );

    if ( button_start.isPressed() ) {
        logger.make_log_file();
        logger.write_header( "run_mode,run_status,line_error,steer_angle,line_digital,FL,FR,RL,RR,SV,speed,battery_voltage,"
                             "slope_status,target_speed" );

        bz.set( 0x00000F0F );
        timer_start_mode_timer.restart();

        run_mode_change_to( RUN_PRE_START );
    }
}

/* RUN_PRE_START
 * 概要：スタートボタン押下後、走行スタート待ち状態
 * 引数：なし
 * 戻り値：なし
 * 詳細：DIPスイッチで指定されている走行スタートモードに応じて走行スタート待ちを行う
 */
void running_pre_start() {
    bool go = false;

    set_servo_mode( STOP );
    motor_pwm( 0, 0, 0, 0 );

    indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_GATE_WAITING );

    if ( dip_switch.bit.sw1 ) {
        // ゲートセンサOPEN待ち
        if ( gate == OPEN ) {
            go = true;
        }
    } else if ( dip_switch.bit.sw2 ) {
        // 3秒待ち
        if ( timer_start_mode_timer.measure() >= 3000 ) {
            go = true;
        }
    } else if ( dip_switch.bit.sw3 ) {
        // スタートボタン押下待ち
        if ( button_start.isPressed() ) {
            go = true;
        }
    } else {
        // DIPスイッチ設定ミス
        bz.set( 0x00005555 ); // エラー音
    }

    if ( go ) {
        // 走行開始
        indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_OFF );
        bz.set( 0x000000FF );
        logger.logging_begin();
        run_mode_change_to( RUN_STABLE );
        encoder_reset();
        running_timer.restart();
        check_stop_fail_distance.restart();
        check_stop_comp_distance.restart();
    }
}

/* RUN_STABLE
 * 概要：定常走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：速度制限、ステアリング角度に応じたライントレース定常走行を行う
 *       難所、急カーブ、坂道判定が成立した場合は、それぞれのモードへ遷移する
 * 備考：ステアリング角度に応じて目標速度をspeed_stable～speed_curveで補完して決定する
 */
void running_stable() {
    set_servo_mode( LINE_TRACE );

    // ステアリング角度に応じた目標速度を決定
    // 0degのときはspeed_stable、4.0deg以降はカーブとし、speed_curveを目標速度にする
    s4 target_speed = map( abs( steer_angle ), 0, 40, speed_stable, speed_curve );
    target_speed = constrain( target_speed, speed_stable, speed_curve );

    spdctrl_stable( target_speed );

    if ( run_judge_difficult( line_digital ) ) {
        line_digital_sum = 0x00;
        run_mode_change_to( RUN_PRE_DIFFICULT );
    }
    if ( run_judge_sharp_curve() ) {
        run_mode_change_to( RUN_SHARP_CURVE );
    }
    if ( slope_status != 0 ) {
        run_mode_change_to( RUN_SLOPE );
    }
}

/* RUN_SHARP_CURVE
 * 概要：急カーブ走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：急カーブを安全に曲がるための4輪、ステアリング制御を行う
 */
void running_sharp_curve() {
    set_servo_mode( LINE_TRACE );
    spdctrl_sharp_curve( speed_curve );

    if ( run_judge_end_sharp_curve() ) {
        run_mode_change_to( RUN_STABLE );
    }
}

/* RUN_PRE_DIFFICULT
 * 概要：クロスライン、ハーフライン読み取り後の検査走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：クロスライン、ハーフライン読み取り後、一定距離はデジタルセンサ値の論理和を取り続ける
 *      一定距離走行後の論理和でクロスライン、ハーフラインを確定させ、
 *      X_LINE_TRACE or R_LANE_CHANGE or L_LANE_CHANGE or STABLEへ遷移する
 *      この時、ライントレースをしてしまうと急激にステアリングを切る恐れがあるため、サーボモーターは停止させる
 */
void running_pre_difficult() {
    set_servo_mode( STOP );
    spdctrl_stable( speed_stable );

    s4 distance = dist_measure.measure();

    // 　一定距離デジタルセンサ値の論理和をとる
    //  しかし、pre_difficult遷移直後の値は捨てるため、ここでは7mm以降の値を取得する
    if ( distance >= 7 ) {
        line_digital_sum |= line_digital;
    }

    // 40mm走行したら然るべきステートに遷移する
    if ( distance >= 40 ) {
        if ( run_judge_cross_line( line_digital_sum ) ) {
            run_mode_change_to( RUN_X_LINE_TRACE );
            start_difficult( RUN_X_LINE_TRACE );
            indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_DIFFICULT_ONE_SHOT );
            bz.set( 0x00000003 );
        } else if ( run_judge_left_half_line( line_digital_sum ) ) {
            run_mode_change_to( RUN_L_LANE_CHANGE );
            start_difficult( RUN_L_LANE_CHANGE );
            indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_DIFFICULT_ONE_SHOT );
            bz.set( 0x00000003 );
        } else if ( run_judge_right_half_line( line_digital_sum ) ) {
            run_mode_change_to( RUN_R_LANE_CHANGE );
            start_difficult( RUN_R_LANE_CHANGE );
            indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_DIFFICULT_ONE_SHOT );
            bz.set( 0x00000003 );
        } else {
            run_mode_change_to( RUN_STABLE );
        }
    }
}

/* RUN_X_LINE_TRACE
 * 概要：クロスライン後のライントレース走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：クロスライン通過後はクランク突入速度で速度制御を行う
 *       ハーフラインを検知したらR_CRANK or L_CRANKへ遷移する
 */
void running_x_line_trace() {
    set_servo_mode( LINE_TRACE );
    spdctrl_stable( speed_crossline );
    if ( run_judge_right_half_line( line_digital ) ) {
        run_mode_change_to( RUN_R_CRANK );
    } else if ( run_judge_left_half_line( line_digital ) ) {
        run_mode_change_to( RUN_L_CRANK );
    } else if ( dist_measure.measure() >= 1200 ) {
        run_mode_change_to( RUN_STABLE ); // ここに来るのは難所誤判定
    } else {
        ; // ここに来続けることは無い
    }
}

/* RUN_R_CRANK
 * 概要：右クランク走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：ステアリングを右に大きく切り、右クランクを曲がる。各ステートでの詳細は各ステート内で記述する
 * 備考：
 */
void running_r_crank() {
    switch ( run_status ) {
    case S00: // 右クランク曲げ制御
        set_servo_mode( INTELI_ANGLE_CTRL, prm_angle_R_crank.get(), prm_time_R_crank.get(), prm_k_R_crank.get() );
        spdctrl_crank( speed_R_crank );

        if ( dist_measure.measure() >= 50 ) {
            switch ( line_digital ) {
            case 0x00: // -- - --
            case 0x08: // x- - --
            case 0x0c: // xx - --
            case 0x1c: // xx x --
                dist_measure.restart();
                run_status = S10;
                break;
            case 0x14: // -x x --
            case 0x10: // -- x --
            case 0x12: // -- x x-
            case 0x03: // -- - xx
            case 0x01: // -- - -x
                dist_measure.restart();
                run_status = S20;
                break;
            default:
                // ここに来るのはセンサ故障の疑いあり
                dist_measure.restart();
                run_status = S10;
                break;
            }
            break;
        }
    case S10: // 外ライン検出中 曲げ続ける
        set_servo_mode( INTELI_ANGLE_CTRL, prm_angle_R_crank.get(), prm_time_R_crank.get(), prm_k_R_crank.get() );
        spdctrl_crank( speed_R_crank );

        if ( dist_measure.measure() >= 80 ) {
            // センターライン検出
            if ( line_digital == 0x14 || line_digital == 0x10 || line_digital == 0x12 ) {
                dist_measure.restart();
                run_status = S20;
            } else {
                // 500mm走ってもセンターライン検出できない場合は強制的にS20に飛ばしてライントレースさせる
                // ここに来るのはセンサ故障の疑いあり。アナログセンサ頼みで制御する
                if ( dist_measure.measure() > 500 ) {
                    dist_measure.restart();
                    run_status = S20;
                }
            }
        }
        break;
    case S20: // センターライン検出 ライントレースへ復帰
        // 安定させるために一気に加速はさせない
        // 速度は定常時とクランク時の平均とする
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( average( speed_R_crank, speed_stable ) );

        // 比較的ゆっくり300mmも走れば安定するはず
        if ( dist_measure.measure() >= 300 ) {
            increase_section_cnt();
            run_mode_change_to( RUN_STABLE );
        }
        break;
    }
}

/* RUN_L_CRANK
 * 概要：左クランク走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：ステアリングを左に大きく切り、左クランクを曲がる。各ステートでの詳細は各ステート内で記述する
 * 備考：
 */
void running_l_crank() {
    switch ( run_status ) {
    case S00: // 左クランク曲げ制御
        set_servo_mode( INTELI_ANGLE_CTRL, prm_angle_L_crank.get(), prm_time_L_crank.get(), prm_k_L_crank.get() );
        spdctrl_crank( speed_L_crank );

        if ( dist_measure.measure() >= 50 ) {
            switch ( line_digital ) {
            case 0x00: // -- - --
            case 0x01: // -- - -x
            case 0x03: // -- - xx
            case 0x13: // -- x xx
                dist_measure.restart();
                run_status = S10;
                break;
            case 0x12: // -- x x-
            case 0x10: // -- x --
            case 0x14: // -x x --
            case 0x0c: // xx - --
            case 0x08: // x- - --
                dist_measure.restart();
                run_status = S20;
                break;
            default:
                // ここに来るのはセンサ故障の疑いあり
                dist_measure.restart();
                run_status = S10;
                break;
            }
        }
        break;
    case S10: // 外ライン検出中 曲げ続ける
        set_servo_mode( INTELI_ANGLE_CTRL, prm_angle_L_crank.get(), prm_time_L_crank.get(), prm_k_L_crank.get() );
        spdctrl_crank( speed_L_crank );

        if ( dist_measure.measure() >= 80 ) {
            // センターライン検出
            if ( line_digital == 0x14 || line_digital == 0x10 || line_digital == 0x12 ) {
                dist_measure.restart();
                run_status = S20;
            } else {
                // 500mm走ってもセンターライン検出できない場合は強制的にS20に飛ばしてライントレースさせる
                // ここに来るのはセンサ故障の疑いあり。アナログセンサ頼みで制御する
                if ( dist_measure.measure() > 500 ) {
                    dist_measure.restart();
                    run_status = S20;
                }
            }
        }
        break;
    case S20: // センターライン検出 ライントレースへ復帰
        // 安定させるために一気に加速はさせない
        // 速度は定常時とクランク時の平均とする
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( average( speed_L_crank, speed_stable ) );

        // 比較的ゆっくり300mmも走れば安定するはず
        if ( dist_measure.measure() >= 300 ) {
            increase_section_cnt();
            run_mode_change_to( RUN_STABLE );
        }
        break;
    }
}

/* RUN_R_LANE_CHANGE
 * 概要：右レーンチェンジ走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：ステアリングを右に切り、右レーンチェンジを行う。各ステートでの詳細は各ステート内で記述する
 */
void running_r_lane_change() {
    switch ( run_status ) {
    case S00: // ハーフライン検出前 オフセット走行
        set_servo_mode( LINE_TRACE, prm_offset_R_lanechange.get() );
        spdctrl_stable( speed_R_lanechange );

        if ( line_digital == 0x00 ) {
            dist_measure.restart();
            run_status = S10;
        }
        if ( dist_measure.measure() >= 1200 ) {
            run_mode_change_to( RUN_STABLE ); // ここに来るのは難所誤判定
        }
        break;
    case S10: // 次のレーンが見えるまで曲げる
        set_servo_mode( INTELI_ANGLE_CTRL, prm_angle_R_lanechange.get(), prm_time_R_lanechange.get(), prm_k_R_lanechange.get() );
        spdctrl_lane_change( speed_R_lanechange, prm_angle_R_lanechange.get() );

        // 念のため、チャタ防止として50mm以上走行してからデジタルセンサを見るようにする
        if ( dist_measure.measure() >= 50 ) {
            // 中心寄りのデジタルセンサがどれか一つでも検出されたら次ステートへ
            // 端っこのデジタルセンサはコース外側を見てる可能性があるため
            if ( line_digital & 0x16 ) {
                dist_measure.restart();
                run_status = S20;
            }
        }
        break;
    case S20: // ステアリングをまっすぐにし、若干行き過ぎるまで走行
        set_servo_mode( INTELI_ANGLE_CTRL, 0, prm_time_R_lanechange.get(), prm_k_R_lanechange.get() );
        spdctrl_lane_change( speed_R_lanechange, 0 );

        if ( dist_measure.measure() >= 50 ) {
            // 左端っこのセンサが反応するまで走行する
            if ( line_digital == 0x08 || line_digital == 0x0c ) {
                dist_measure.restart();
                run_status = S30;
            }
        }
        break;
    case S30: // ステアリングを逆に切り、しばらく走行して中心線を捉えたら定常復帰
        set_servo_mode( INTELI_ANGLE_CTRL, -prm_angle_R_lanechange.get(), prm_time_R_lanechange.get(), prm_k_R_lanechange.get() );
        spdctrl_lane_change( speed_R_lanechange, -prm_angle_R_lanechange.get() );

        if ( dist_measure.measure() >= 50 ) {
            if ( line_digital == 0x10 || line_digital == 0x12 || line_digital == 0x14 ) {
                dist_measure.restart();
                run_status = S40;
            }
        }
        break;
    case S40: // 走行安定化
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( average( speed_R_lanechange,
                                 speed_stable ) ); // 速度は定常時とレーンチェンジ時の平均とする

        if ( dist_measure.measure() >= 300 ) {
            increase_section_cnt();
            run_mode_change_to( RUN_STABLE );
        }
        break;
    }
}

/* RUN_L_LANE_CHANGE
 * 概要：左レーンチェンジ走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：ステアリングを左に切り、左レーンチェンジを行う。各ステートでの詳細は各ステート内で記述する
 */
void running_l_lane_change() {
    switch ( run_status ) {
    case S00: // ハーフライン検出前 オフセット走行
        set_servo_mode( LINE_TRACE, prm_offset_L_lanechange.get() );
        spdctrl_stable( speed_L_lanechange );

        if ( line_digital == 0x00 ) {
            dist_measure.restart();
            run_status = S10;
        }
        if ( dist_measure.measure() >= 1200 ) {
            run_mode_change_to( RUN_STABLE ); // ここに来るのは難所誤判定
        }
        break;
    case S10: // 次のレーンが見えるまで曲げる
        set_servo_mode( INTELI_ANGLE_CTRL, prm_angle_L_lanechange.get(), prm_time_L_lanechange.get(), prm_k_L_lanechange.get() );
        spdctrl_lane_change( speed_L_lanechange, prm_angle_L_lanechange.get() );

        // 念のため、チャタ防止として50mm以上走行してからデジタルセンサを見るようにする
        if ( dist_measure.measure() >= 50 ) {
            // 中心寄りのデジタルセンサがどれか一つでも検出されたら次ステートへ
            // 端っこのデジタルセンサはコース外側を見てる可能性があるため
            if ( line_digital & 0x16 ) {
                dist_measure.restart();
                run_status = S20;
            }
        }
        break;
    case S20: // ステアリングをまっすぐにし、若干行き過ぎるまで走行
        set_servo_mode( INTELI_ANGLE_CTRL, 0, prm_time_L_lanechange.get(), prm_k_L_lanechange.get() );
        spdctrl_lane_change( speed_L_lanechange, 0 );

        if ( dist_measure.measure() >= 50 ) {
            // 左端っこのセンサが反応するまで走行する
            if ( line_digital == 0x01 || line_digital == 0x03 ) {
                dist_measure.restart();
                run_status = S30;
            }
        }
        break;
    case S30: // ステアリングを逆に切り、しばらく走行して中心線を捉えたら定常復帰
        set_servo_mode( INTELI_ANGLE_CTRL, -prm_angle_L_lanechange.get(), prm_time_L_lanechange.get(), prm_k_L_lanechange.get() );
        spdctrl_lane_change( speed_L_lanechange, -prm_angle_L_lanechange.get() );

        if ( dist_measure.measure() >= 50 ) {
            if ( line_digital == 0x10 || line_digital == 0x12 || line_digital == 0x14 ) {
                dist_measure.restart();
                run_status = S40;
            }
        }
        break;
    case S40: // 走行安定化
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( average( speed_L_lanechange,
                                 speed_stable ) ); // 速度は定常時とレーンチェンジ時の平均とする

        if ( dist_measure.measure() >= 300 ) {
            increase_section_cnt();
            run_mode_change_to( RUN_STABLE );
        }
        break;
    }
}

/* RUN_SLOPE
 * 概要：坂道走行制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：登坂～下り坂までの走行制御を行う
 *      登頂時には飛ばないように減速し、下り坂では速度制限を超えないように制御する
 * 備考：本当はレギュレーション通り、下り坂も考慮したほうがいい
 */
void running_slope() {
    switch ( run_status ) {
    case S00: // 登坂中
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( speed_stable );

        // 難所判断
        if ( run_judge_difficult( line_digital ) ) {
            // ここに来る場合は以下のいずれか。1はアナログで判断している関係上、比較的可能性が高いのでデジタルセンサを信じて難所処理する。
            // 1.坂道センサが誤検出した
            // 2.デジタルセンサが誤検出した
            line_digital_sum = line_digital;
            run_mode_change_to( RUN_PRE_DIFFICULT );
        } else if ( dist_measure.measure() >= 1150 ) { // 1150mm走行で登頂一歩手前に来るはず(車体により誤差あり)
            dist_measure.restart();
            run_status = S10;
        } else {
            ; // ここに来続けることは無い
        }
        break;
    case S10: // 登頂直前で目標速度切替によるブレーキ(ミサイル禁止)
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( speed_slope );

        // 難所判断
        if ( run_judge_difficult( line_digital ) ) {
            // ここに来る場合は以下のいずれか。1はアナログで判断している関係上、比較的可能性が高いのでデジタルセンサを信じて難所処理する。
            // 1.坂道センサが誤検出した
            // 2.デジタルセンサが誤検出した
            line_digital_sum = line_digital;
            run_mode_change_to( RUN_PRE_DIFFICULT );
        } else if ( dist_measure.measure() >= 500 ) {
            // ここに来るのは坂道誤判定だった場合
            run_mode_change_to( RUN_STABLE );
        } else if ( slope_status == -1 ) { // 登頂
            dist_measure.restart();
            run_status = S20;
        } else {
            ; // ここに来続けることは無い
        }

        break;
    case S20: // 登頂後走行制御
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( speed_slope );

        // 坂の上で難所はありえないので考慮しない

        if ( dist_measure.measure() >= 500 ) {
            if ( slope_status == -1 ) { // 下り始め
                dist_measure.restart();
                run_status = S30;
            } else if ( slope_status == 1 ) { // バカな...？下り終わっただと！？
                dist_measure.restart();
                run_status = S40;
            } else if ( dist_measure.measure() >= 10000 ) {
                // 10000mm(10m)走行しても坂センサが反応しない場合は坂道誤判定として定常走行へ
                run_mode_change_to( RUN_STABLE );
            } else {
                ; // ここに来続けることは無い
            }
        }
        break;
    case S30: // 下り制御
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( speed_stable );

        // 坂の上で難所はありえないので考慮しない

        if ( dist_measure.measure() >= 1100 ) {
            dist_measure.restart();
            run_status = S40;
        }
        break;
    case S40: // 下り終わり直前ブレーキ
        set_servo_mode( LINE_TRACE );
        spdctrl_stable( speed_slope );

        if ( dist_measure.measure() >= 300 ) { // 300mmも走れば下り終わっている
            run_mode_change_to( RUN_STABLE );
        }
        break;
    }
}

/* RUN_END
 * 概要：走行終了制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：ロギングタスクに走行終了を通知し、書き込みが終わるまで待つ
 */
void running_end() {
    set_servo_mode( STOP );
    motor_pwm( 0, 0, 0, 0 );
    bz.set( 0x00000155 );
    logger.logging_end();
    run_mode = RUN_STOP;
}

/*
 * 概要：モータテストを行う
 * 引数：なし
 * 戻り値：なし
 * 詳細：モータテストを行う
 */
void running_test_motor() {
    set_servo_mode( STOP );
    motor_pwm( motor_test_fl, motor_test_fr, motor_test_rl, motor_test_rr );
}

/*
 * 概要：ライントレーステストを行う
 * 引数：なし
 * 戻り値：なし
 * 詳細：ライントレーステストを行う
 */
void running_test_linetrace() {
    switch ( line_trace_test_mode ) {
    case LINETRACE_TEST_NORMAL:
        set_servo_mode( LINE_TRACE );
        break;
    case LINETRACE_TEST_OFFSET_R:
        set_servo_mode( LINE_TRACE, prm_offset_R_lanechange.get() );
        break;
    case LINETRACE_TEST_OFFSET_L:
        set_servo_mode( LINE_TRACE, prm_offset_L_lanechange.get() );
        break;
    default:
        set_servo_mode( STOP );
        break;
    }
    motor_pwm( 0, 0, 0, 0 );
}

/*
 * 概要：角度制御テストを行う
 * 引数：なし
 * 戻り値：なし
 * 詳細：角度制御テストを行う
 */
void running_test_angle() {
    switch ( angle_ctrl_test_state ) {
    case 1:
        set_servo_mode( INTELI_ANGLE_CTRL, test_deg, test_time, test_k );
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        set_servo_mode( STOP );
        break;
    }
    motor_pwm( 0, 0, 0, 0 );
}

/*
 * 概要：走行制御メイン関数
 * 引数：なし
 * 戻り値：なし
 * 詳細：走行制御
 */
void ruuning() {
    switch ( run_mode ) {

    // 通常走行モード
    case RUN_STOP:
        running_stop();
        break;
    case RUN_PRE_START:
        running_pre_start();
        break;
    case RUN_STABLE:
        running_stable();
        break;
    case RUN_SHARP_CURVE:
        running_sharp_curve();
        break;
    case RUN_PRE_DIFFICULT:
        running_pre_difficult();
        break;
    case RUN_X_LINE_TRACE:
        running_x_line_trace();
        break;
    case RUN_R_CRANK:
        running_r_crank();
        break;
    case RUN_L_CRANK:
        running_l_crank();
        break;
    case RUN_R_LANE_CHANGE:
        running_r_lane_change();
        break;
    case RUN_L_LANE_CHANGE:
        running_l_lane_change();
        break;
    case RUN_SLOPE:
        running_slope();
        break;
    case RUN_END:
        running_end();
        break;

    // テストモード
    case RUN_TEST_MOTOR:
        running_test_motor();
        break;
    case RUN_TEST_TRACE:
        running_test_linetrace();
        break;
    case RUN_TEST_ANGLE:
        running_test_angle();
        break;

    default:
        run_mode = RUN_STOP;
        break;
    }

    // 停止共通判断
    if ( ( RUN_STABLE <= run_mode ) && ( run_mode <= RUN_SLOPE ) && run_judge_check_stop() ) {
        run_mode = RUN_END;
    }
}

/*
 * 概要：フェールセーフ制御
 * 引数：なし
 * 戻り値：なし
 * 詳細：フェールを検出し、フェール表示を行う
 */
void failsafe() {
    // フェール検出
    failer |= logger.is_fault() ? 1 : 0;

    // フェール表示
    if ( failer & 0x01 ) {
        indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_NO_SDCARD ); // SDカードエラー
    }
}

/*******************************/
/* Task functions              */
/*******************************/
/*
 * 概要：1ms周期で動くタイマタスク
 * 引数：なし
 * 戻り値：なし
 * 詳細：1ms周期で動くタイマタスク
 *      0.5ms以上かかる処理はこのタスク内で行わないこと
 * 備考：割り込みコンテキストで実行される
 */
void timer_1ms_task( timer_callback_args_t* p_args ) {
    encoder();
    angle_update();
    line_sensor_update();
    servo_control();
    motor_control();

    if ( log_interval++ > 10 ) {
        char buf[256];
        mini_snprintf( buf, 256, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", run_mode, run_status, line_error, steer_angle, line_digital, FL, FR,
                       RL, RR, SV, speed, battery_voltage, slope_status, target_speed_now );

        logger.put_log( buf );
        log_interval = 0;
    }

    bz.process_1ms();
}

/***********************************/
/* Global functions                */
/***********************************/
void setup() {
    Serial.begin( 460800 );
    wait_ms( 2000 );
    DBG_PRINT( "Start setup\n" );

    motor_init();
    screen_setup();
    sensors_init();
    logger.init();
    // nvm_erase(); キャリブレーションパラメータを減らしたときに実行すること
    nvm_load();

    indicator_init();

    fsp_timer.begin( TIMER_MODE_PERIODIC, AGT_TIMER, 1, 24000 - 1, 1, (timer_source_div_t)TIMER_SOURCE_DIV_1, timer_1ms_task );
    IRQManager::getInstance().addPeripheral( IRQ_AGT, (void*)fsp_timer.get_cfg() );
    fsp_timer.open();
    fsp_timer.start(); // APIのバグでopen()でstartされているが、APIが修正されたときにも動くようにするためにstart()を一応呼ぶ

    // テストモードのときはシェルを起動する
    if ( COMMAND_TEST_MODE ) {
        test_mode_setup();
    }
}

void loop() {
    sensors_update();
    target_speed_update();
    if ( run_mode == RUN_STOP || run_mode == RUN_TEST_MOTOR || run_mode == RUN_TEST_TRACE || run_mode == RUN_TEST_ANGLE ) {
        screen_exec();
    }
    ruuning();
    indicator_exec();
    failsafe();

    // テストモードのときはシェルを実行する
    if ( COMMAND_TEST_MODE ) {
        test_mode_main_task();
    }
}
