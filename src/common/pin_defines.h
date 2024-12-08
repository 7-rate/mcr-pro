#pragma once
#include <Arduino.h>
#include <FastLED.h>

/***********************************/
/* Pin Defines                     */
/***********************************/
// FLモーター
#define PIN_MOTOR_FL_PWMH ( D47 )
#define PIN_MOTOR_FL_PWML ( D46 )
#define PIN_MOTOR_FL_PHASE ( D57 )
#define PIN_MOTOR_FL_SR ( D58 )

// FRモーター
#define PIN_MOTOR_FR_PWMH ( D2 )
#define PIN_MOTOR_FR_PWML ( D3 )
#define PIN_MOTOR_FR_PHASE ( D6 )
#define PIN_MOTOR_FR_SR ( D7 )

// RLモーター
#define PIN_MOTOR_RL_PWMH ( D76 )
#define PIN_MOTOR_RL_PWML ( D75 )
#define PIN_MOTOR_RL_PHASE ( D78 )
#define PIN_MOTOR_RL_SR ( D77 )

// RRモーター
#define PIN_MOTOR_RR_PWMH ( D67 )
#define PIN_MOTOR_RR_PWML ( D68 )
#define PIN_MOTOR_RR_PHASE ( D65 )
#define PIN_MOTOR_RR_SR ( D66 )

// Servoモーター
#define PIN_MOTOR_SV_PWMH ( D53 )
#define PIN_MOTOR_SV_PWML ( D52 )
#define PIN_MOTOR_SV_PHASE ( D48 )
#define PIN_MOTOR_SV_SR ( D49 )

// ブザー
#define PIN_BUZZER ( D72 )

// エンコーダ
#define PIN_ENCODER_A ( D70 )
#define PIN_ENCODER_B ( D69 )

// ラインセンサ
// Sens_0:D35
// Sens_1:D36
// Sens_2:D38
// Sens_3:D39
// Sens_4:D40
// Sens_5:D41
// Sens_6:D42
// Sens_7:A3
#define PIN_LINE_DIGITAL_CENTER ( D35 )
#define PIN_LINE_DIGITAL_8 ( D36 )
#define PIN_LINE_DIGITAL_4 ( D38 )
#define PIN_LINE_ANALOG_LEFT ( D39 )
#define PIN_LINE_DIGITAL_GATE ( D40 )
#define PIN_LINE_ANALOG_RIGHT ( D41 )
#define PIN_LINE_DIGITAL_2 ( D42 )
#define PIN_LINE_DIGITAL_1 ( A3 )

// 電力センサ
#define PIN_BATT_VOLTAGE ( D63 )

// ボタン
#define PIN_BUTTON_START ( D27 )
#define PIN_BUTTON_LEFT ( D61 )
#define PIN_BUTTON_UP ( A0 )
#define PIN_BUTTON_RIGHT ( A1 )
#define PIN_BUTTON_DOWN ( A2 )
#define PIN_BUTTON_ENTER ( D32 )
#define PIN_BUTTON_SAVE ( D31 )

// ディップスイッチ
#define PIN_BOARD_DIPSW_1 ( D25 )
#define PIN_BOARD_DIPSW_2 ( D26 )
#define PIN_DIPSW_1 ( D33 )
#define PIN_DIPSW_2 ( D28 )
#define PIN_DIPSW_3 ( D29 )
#define PIN_DIPSW_4 ( D30 )

// LED
#define PIN_NEOPIXEL ( D74 )
_FL_DEFPIN( PIN_NEOPIXEL, BSP_IO_PORT_06_PIN_09, R_PORT6_BASE ); // FastLEDのための追加のピン定義

#define PIN_BOARD_LED_D3 ( D13 )

// 汎用位相計数ピン
#define PIN_GP_ENC_A ( D8 )
#define PIN_GP_ENC_B ( D9 )

// 汎用デジタルピン
#define PIN_GP_1 ( D44 )
#define PIN_GP_2 ( D43 )

// 汎用ADCピン
#define PIN_GP_AN_1 ( D62 )
#define PIN_GP_AN_2 ( D64 )
