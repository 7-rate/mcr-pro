/*
 * 概要：モーター制御
 * 備考：本ファイルの関数はモーター制御を行うための関数群です。
 */

#pragma once
#include <Arduino.h>
#include "defines.h"
#include "motor_driver.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
/* サーボモーターの制御モード */
typedef enum e_servo_mode {
    LINE_TRACE,
    INTELI_ANGLE_CTRL,
    MANUAL_CTRL,
    STOP,
} servo_mode_t;

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void set_servo_mode( servo_mode_t mode, s4 param1 = 0, s4 param2 = 0, s4 param3 = 0 );
void servo_control();

void motor_mode_FL( BC mode );
void motor_mode_FR( BC mode );
void motor_mode_RL( BC mode );
void motor_mode_RR( BC mode );
void motor_mode( BC mode_fl, BC mode_fr, BC mode_rl, BC mode_rr );
void motor_pwm_FL( s4 pwm );
void motor_pwm_FR( s4 pwm );
void motor_pwm_RL( s4 pwm );
void motor_pwm_RR( s4 pwm );
void motor_pwm( s4 pwm_fl, s4 pwm_fr, s4 pwm_rl, s4 pwm_rr );
void motor_control();
void motor_init();

/***********************************/
/* Global Variables                */
/***********************************/
extern motor_driver motor_FL;
extern motor_driver motor_FR;
extern motor_driver motor_RL;
extern motor_driver motor_RR;
extern motor_driver motor_SV;

extern s2 FL; // FLモーターのpwm指令値
extern s2 FR; // FRモーターのpwm指令値
extern s2 RL; // RLモーターのpwm指令値
extern s2 RR; // RRモーターのpwm指令値
extern s2 SV; // SVモーターのpwm指令値

extern s2 debug_target_angle; // デバッグ用目標角度