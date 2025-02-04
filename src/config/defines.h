/*
 * 概要：マイコンカーに関するグローバルに使用される定数や変数を定義する
 */

#pragma once
#include <Arduino.h>
#include "pin_defines.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
/* 走行モード */
enum e_run_mode {
    RUN_STOP,          /* 0 */
    RUN_PRE_START,     /* 1 */
    RUN_STABLE,        /* 2 */
    RUN_SHARP_CURVE,   /* 3 */
    RUN_PRE_DIFFICULT, /* 4 */
    RUN_X_LINE_TRACE,  /* 5 */
    RUN_R_CRANK,       /* 6 */
    RUN_L_CRANK,       /* 7 */
    RUN_R_LANE_CHANGE, /* 8 */
    RUN_L_LANE_CHANGE, /* 9 */
    RUN_SLOPE,         /* 10 */
    RUN_TEST_MOTOR,    /* 11 */
    RUN_TEST_TRACE,    /* 12 */
    RUN_TEST_ANGLE,    /* 13 */
    RUN_END,           /* 14 */
};

/* 走行モードのステータス */
enum e_run_status {
    S00,
    S01,
    S02,
    S03,
    S04,
    S05,
    S06,
    S07,
    S08,
    S09,
    S10,
    S11,
    S12,
    S13,
    S14,
    S15,
    S16,
    S17,
    S18,
    S19,
    S20,
    S21,
    S22,
    S23,
    S24,
    S25,
    S26,
    S27,
    S28,
    S29,
    S30,
    S31,
    S32,
    S33,
    S34,
    S35,
    S36,
    S37,
    S38,
    S39,
    S40,
    S41,
    S42,
    S43,
    S44,
    S45,
    S46,
    S47,
    S48,
    S49,
    S50,
    S51,
    S52,
    S53,
    S54,
    S55,
    S56,
    S57,
    S58,
    S59,
    S60,
    S61,
    S62,
    S63,
    S64,
    S65,
    S66,
    S67,
    S68,
    S69,
    S70,
    S71,
    S72,
    S73,
    S74,
    S75,
    S76,
    S77,
    S78,
    S79,
    S80,
    S81,
    S82,
    S83,
    S84,
    S85,
    S86,
    S87,
    S88,
    S89,
    S90,
    S91,
    S92,
    S93,
    S94,
    S95,
    S96,
    S97,
    S98,
    S99,
};

/* 型定義 */
/* 既存の型(intなど)は基本的に使わず、ここで定義されいている型を使うこと */
typedef unsigned char u1;
typedef char s1;
typedef unsigned short u2;
typedef short s2;
typedef unsigned long u4;
typedef long s4;
typedef unsigned long long u8;
typedef long long s8;
typedef float f4;
typedef double f8;

/* ブレーキ・コースト */
typedef bool BC;
#define BRAKE ( true )
#define COAST ( false )

/* ゲートセンサ */
#define OPEN ( 0 )
#define CLOSE ( 1 )

enum lsb {
    LSB_0001 = 1,
    LSB_001 = 10,
    LSB_01 = 100,
    LSB_1 = 1000,
    LSB_10 = 10000,
    LSB_100 = 100000,
    LSB_1000 = 1000000,
};

enum category {
    CATEGORY_LINE_TRACE,
    CATEGORY_SENSOR_CALIBRATION,
    CATEGORY_ANGLE_CTRL,
    CATEGORY_SPEED,
    CATEGORY_CURVE,
    CATEGORY_SECTION_SPEED,
    CATEGORY_DISTANCE,
    CATEGORY_DIFFICULT_KIND,
    CATEGORY_LANE_CHANGE,
    CATEGORY_CRANK,
};

/* ディップスイッチの値を入れる型 */
typedef union {
    u1 byte;
    struct {
        u1 sw1 : 1;
        u1 sw2 : 1;
        u1 sw3 : 1;
        u1 sw4 : 1;
        u1 board_sw1 : 1;
        u1 board_sw2 : 1;
        u1 dummy1 : 1;
        u1 dummy2 : 1;
    } bit;
} dip_switch_t;

/* セクション数の最大値 */
#define SECTION_MAX ( 10 )

/* 難所種別 */
enum {
    CR_L,
    CR_R,
    LC_L,
    LC_R,
};
/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
/* デバッグ用 */
#if CONFIG_MCR_APP_DEBUG
#include <LibPrintf.h>
#define DBG_PRINT( fmt, ... )                                                                                                                        \
    do {                                                                                                                                             \
        printf( fmt, ##__VA_ARGS__ );                                                                                                                \
    } while ( 0 )

#else
#define DBG_PRINT( fmt, ... )                                                                                                                        \
    do {                                                                                                                                             \
    } while ( 0 )

#endif

#define average( a, b ) ( ( ( a ) + ( b ) ) / 2 )

#define array_size( a ) ( sizeof( a ) / sizeof( a[0] ) )

static void wait_us( u4 us ) {
    volatile u4 start = micros();
    while ( micros() - start < us ) {
    }
}

/*
 * 概要： 指定した時間待つ
 * 引数：なし
 * 戻り値：なし
 * 詳細：Arduinoのdelay関数は精度が悪いので使わない
 */
static void wait_ms( u4 ms ) {
    volatile u4 start = millis();
    while ( millis() - start < ms ) {
    }
}

/*
 * 概要： 指定したピンのデジタル入力を読む
 * 引数：なし
 * 戻り値：なし
 * 詳細：ArduinoのdigitalReadは22usかかる。一方、以下の実装なら3usで読める。
 */
static int my_digital_read( int pin ) {
    return R_BSP_PinRead( g_pin_cfg[pin].pin );
}

/***********************************/
/* Global Variables                */
/***********************************/
#define ENCODER_PULSE_PER_REV ( 600 ) // エンコーダの1回転あたりのパルス数(300pulse * 2相 * 両エッジ)
#define ENCODER_WHEEL_DIAMETER ( 24 ) // 車輪の直径[mm] !!! cation !!!使わない
#define ENCODER_WHEEL_LENGTH ( 100 )  // 車輪の周長[mm]

#define CAR_WEIGHT ( 800 )           // 車体重量[g]
#define CAR_WIDTH ( 180 )            // 車体トレッド[mm] タイヤ中心間距離
#define CAR_LENGTH ( 180 )           // 車体ホイールベース[mm] 後輪中心～前輪中心
#define CAR_CENTER_OF_GRAVITY ( 10 ) // 車体重心位置[mm]

#define CAR_STEER_GEAR_RATIO ( 700 ) // LSB 0.1[-]

#define ANGLE_PULSE ( 128 ) // エンコーダ1回転あたりのパルス数(32pulse * 2相 * 両エッジ)

#define ACCELERATION ( 90 )   // 減速度[0.1m/s^2]
#define DECELERATION ( -120 ) // 減速度[0.1m/s^2]

/* fail code */
#define FAIL_SD_CARD ( 0x1 )

extern u4 failer;
