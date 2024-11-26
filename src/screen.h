/*
 * 概要：スクリーン&ボタンの制御
 */
#pragma once
#include <Arduino.h>
#include "defines.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
enum linetrace_test_mode {
    LINETRACE_TEST_NORMAL,   // 通常ライントレース
    LINETRACE_TEST_OFFSET_R, // オフセットテスト 右
    LINETRACE_TEST_OFFSET_L, // オフセットテスト 左
};

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void screen_setup();
void screen_exec();

/***********************************/
/* Global Variables                */
/***********************************/
extern bool screen_is_connected; // スクリーン接続状態 false:未接続 true:接続

// ライントレーステスト関連
extern u1 line_trace_test_mode; // オフセットテストのモード 0:オフセットなし 1:右 2:左

// 角度制御テスト関連
extern s4 test_deg;
extern u4 test_time;
extern u4 test_k;
extern u1 angle_ctrl_test_state; // 0:初期状態 1:テスト中

// モータテスト関連
extern s1 motor_test_fl; // モータ 左前のpwm値
extern s1 motor_test_fr; // モータ 右前のpwm値
extern s1 motor_test_rl; // モータ 左後のpwm値
extern s1 motor_test_rr; // モータ 右後のpwm値