/*
 * 概要：モーター制御
 * 備考：本ファイルの関数はモーター制御を行うための関数群です。
 */

#include "motor_control.h"
#include "time_measure.h"
#include "calibration.h"
#include "calc_utils.h"
#include "sensors.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/

/***********************************/
/* Local Variables                 */
/***********************************/
// サーボモード
static servo_mode_t servo_mode = STOP;
static servo_mode_t servo_mode_old = STOP;

// ライントレース用パラメータ
static s4 servo_ctrl_line_trace_offset = 0;
static s4 servo_ctrl_line_error_sum = 0; // ライントレース I制御用 アナログセンサーの左右差分積分値
static s4 servo_ctrl_line_trace_offset_old = 0;
static s4 servo_ctrl_line_error_old = 0;

// 角度制御用パラメータ
static s4 servo_ctrl_target_angle = 0;
static s4 servo_ctrl_target_angle_time = 0;
static s4 servo_ctrl_target_angle_k = 0;
static time_measure servo_ctrl_target_angle_timer;
static s4 servo_ctrl_start_angle = 0;
static s4 servo_ctrl_angle_error_sum = 0;
static s4 servo_ctrl_angle_old = 0;

// マニュアルモード用パラメータ
static s4 servo_target_pwm = 0;

static s4 iSensorBefore = 0;

/***********************************/
/* Global Variables                */
/***********************************/
// モーター定義
motor_driver motor_FL( PIN_MOTOR_FL_PWMH, PIN_MOTOR_FL_PWML, PIN_MOTOR_FL_PHASE, PIN_MOTOR_FL_SR );
motor_driver motor_FR( PIN_MOTOR_FR_PWMH, PIN_MOTOR_FR_PWML, PIN_MOTOR_FR_PHASE, PIN_MOTOR_FR_SR );
motor_driver motor_RL( PIN_MOTOR_RL_PWMH, PIN_MOTOR_RL_PWML, PIN_MOTOR_RL_PHASE, PIN_MOTOR_RL_SR );
motor_driver motor_RR( PIN_MOTOR_RR_PWMH, PIN_MOTOR_RR_PWML, PIN_MOTOR_RR_PHASE, PIN_MOTOR_RR_SR );
motor_driver motor_SV( PIN_MOTOR_SV_PWMH, PIN_MOTOR_SV_PWML, PIN_MOTOR_SV_PHASE, PIN_MOTOR_SV_SR );

s2 FL; // FLモーターのpwm指令値
s2 FR; // FRモーターのpwm指令値
s2 RL; // RLモーターのpwm指令値
s2 RR; // RRモーターのpwm指令値
s2 SV; // SVモーターのpwm指令値

s2 debug_target_angle; // デバッグ用目標角度

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/

/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
/*
 * 概要：サーボモーター制御の設定を行う
 * 引数：mode:LINE_TRACE or ANGLE_CTRL or MANUAL_CTRL or STOP
 * 引数：parameter:各モードに応じたパラメータ
 *         →LINE_TRACE時:ライントレースオフセット
 *        →INTELI_ANGLE_CTRL時:目標角度(LSB:0.1deg)、目標角度に到達するまでの時間(ms)、係数k
 *         →MANUAL_CTRL時:目標pwm値(-100~100)
 * 戻り値：なし
 * 詳細：サーボモーター制御の設定を行う
 */
void set_servo_mode( servo_mode_t mode, s4 param1, s4 param2, s4 param3 ) {
    noInterrupts();
    servo_mode = mode;

    switch ( servo_mode ) {
    case LINE_TRACE:
        servo_ctrl_line_trace_offset = param1;
        (void)param2;
        (void)param3;
        // 目標が変わった時にパラメータをリセット
        if ( servo_mode_old != LINE_TRACE || servo_ctrl_line_trace_offset != servo_ctrl_line_trace_offset_old ) {
            servo_ctrl_line_error_sum = 0;
            servo_ctrl_line_trace_offset_old = servo_ctrl_line_trace_offset;
            servo_ctrl_line_error_old = line_error;
        }
        break;
    case INTELI_ANGLE_CTRL:
        // 目標が変わった時にパラメータをリセット
        if ( ( servo_mode_old != INTELI_ANGLE_CTRL ) || ( servo_ctrl_target_angle != param1 ) || ( servo_ctrl_target_angle_time != param2 ) ||
             ( servo_ctrl_target_angle_k != param3 ) ) {
            // 角度制御パラメータ更新
            servo_ctrl_target_angle = param1;
            servo_ctrl_target_angle_time = param2;
            servo_ctrl_target_angle_k = param3;
            // 角度制御用タイマーとPID制御の要素を初期化
            servo_ctrl_target_angle_timer.restart();
            servo_ctrl_start_angle = steer_angle;
            servo_ctrl_angle_error_sum = 0;
            servo_ctrl_angle_old = steer_angle;
        }
        break;
    case MANUAL_CTRL:
        servo_target_pwm = param1;
        (void)param2;
        (void)param3;
        break;
    case STOP:
        /* break; */ /* fall through */
    default:
        (void)param1;
        (void)param2;
        (void)param3;
        servo_mode = STOP;
        break;
    }
    servo_mode_old = servo_mode;
    interrupts();
}

/*
 * 概要：サーボモーター制御を行う
 * 引数：なし
 * 戻り値：なし
 * 詳細：PID制御のため、1ms周期で呼び出すこと
 *       ライントレース、角度制御、マニュアル制御、停止のいずれかを行う
 */
void servo_control() {
    s4 target;
    s4 error;
    s4 kp, ki, kd;
    s4 out_p, out_i, out_d;
    s4 pwm;

    s4 i, iD, iP, iRet;

    switch ( servo_mode ) {
    case LINE_TRACE:
        // ライントレース制御
        s4 line_error_diff;
        kp = prm_line_trace_P.get();
        ki = prm_line_trace_I.get();
        kd = prm_line_trace_D.get();

        target = 0 + servo_ctrl_line_trace_offset;
        error = target - line_error; // line_errorは-1024～1023
        servo_ctrl_line_error_sum += error;
        servo_ctrl_line_error_sum = constrain( servo_ctrl_line_error_sum, -1024, 1023 );
        line_error_diff = error - servo_ctrl_line_error_old;

        // kp補正
        //   line_errorがtargetに近いエリアではkpを小さくする
        //   target   0----196--------1024
        //   補正    1/2    1          1
        kp = map( abs( error ), 0, 196, kp / 2, kp );
        kp = min( kp, prm_line_trace_P.get() );

        out_p = ( kp * error ) / 1024;
        out_i = ( ki * servo_ctrl_line_error_sum ) / 1024;
        out_d = ( kd * line_error_diff ) / 1024;

        pwm = out_p + out_i + out_d;
        servo_ctrl_line_error_old = error;
        break;
    case INTELI_ANGLE_CTRL:
        // 角度制御
        s4 angle_error_diff;
        kp = prm_angle_ctrl_P.get();
        ki = prm_angle_ctrl_I.get();
        kd = prm_angle_ctrl_D.get();

        target = calc_angle_linear( servo_ctrl_start_angle, servo_ctrl_target_angle, servo_ctrl_target_angle_time,
                                    servo_ctrl_target_angle_timer.measure() );
        debug_target_angle = target;

        error = steer_angle - target;
        servo_ctrl_angle_error_sum += error;
        servo_ctrl_angle_error_sum = constrain( servo_ctrl_angle_error_sum, -8192, 8192 );
        angle_error_diff = error - servo_ctrl_angle_old;

        out_p = ( kp * error ) >> 5;
        out_i = ( ki * servo_ctrl_angle_error_sum ) >> 12;
        out_d = ( kd * angle_error_diff ) >> 8;

        pwm = out_p + out_i + out_d;
        servo_ctrl_angle_old = steer_angle;
        break;
    case MANUAL_CTRL:
        // マニュアル制御
        pwm = servo_target_pwm;
        break;
    case STOP:
        // break;  fall through
    default:
        // 停止
        pwm = 0;
        break;
    }
    motor_SV.set_mode( BRAKE );
    motor_SV.set_pwm( pwm );
    SV = pwm;
}

/*
 * 概要：各モーターのブレーキ/コーストモード設定
 * 引数：mode:BRAKE or COAST
 * 戻り値：なし
 * 詳細：各モーターのブレーキ/コーストモード設定
 */
void motor_mode_FL( BC mode ) {
    motor_FL.set_mode( mode );
}
void motor_mode_FR( BC mode ) {
    motor_FR.set_mode( mode );
}
void motor_mode_RL( BC mode ) {
    motor_RL.set_mode( mode );
}
void motor_mode_RR( BC mode ) {
    motor_RR.set_mode( mode );
}
void motor_mode( BC mode_fl, BC mode_fr, BC mode_rl, BC mode_rr ) {
    motor_mode_FL( mode_fl );
    motor_mode_FR( mode_fr );
    motor_mode_RL( mode_rl );
    motor_mode_RR( mode_rr );
}

/*
 * 概要：各モーターのpwm指令値値設定
 * 引数：pwm:-100~100 (負の値は逆転)
 * 戻り値：なし
 * 詳細：ここでは指令値を設定するだけで、実際の出力はmotor_control()で行う
 */
void motor_pwm_FL( s4 pwm ) {
    FL = pwm;
}
void motor_pwm_FR( s4 pwm ) {
    FR = pwm;
}
void motor_pwm_RL( s4 pwm ) {
    RL = pwm;
}
void motor_pwm_RR( s4 pwm ) {
    RR = pwm;
}
void motor_pwm( s4 pwm_fl, s4 pwm_fr, s4 pwm_rl, s4 pwm_rr ) {
    motor_pwm_FL( pwm_fl );
    motor_pwm_FR( pwm_fr );
    motor_pwm_RL( pwm_rl );
    motor_pwm_RR( pwm_rr );
}

/*
 * 概要：各モーターのpwm出力
 * 引数：なし
 * 戻り値：なし
 * 詳細：motor_pwm()で設定されたpwm指令値を出力する
 */
void motor_control() {
    motor_FL.set_pwm( FL );
    motor_FR.set_pwm( FR );
    motor_RL.set_pwm( RL );
    motor_RR.set_pwm( RR );
}

void motor_init() {
    motor_FL.begin();
    motor_FL.set_mode( BRAKE );
    motor_FR.begin();
    motor_FR.set_mode( BRAKE );
    motor_RL.begin();
    motor_RL.set_mode( BRAKE );
    motor_RR.begin();
    motor_RR.set_mode( BRAKE );
    motor_SV.begin();
    motor_SV.set_mode( BRAKE );
}