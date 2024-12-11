/*
 * 概要：スクリーン&ボタンの制御
 */
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "mcr_logger.h"
#include "calibration.h"
#include "defines.h"
#include "sensors.h"
#include "screen.h"
#include "motor_driver.h"
#include "motor_control.h"
#include "nvm.h"
#include "indicator.h"
#include "interval.h"
#include "time_measure.h"

extern void run_mode_change_to( enum e_run_mode mode, bool dist_measure_reset = true, bool run_status_reset = true );

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
#define OLED_ADDRESS ( 0x3C )

#define SCREEN_WIDTH ( 128 )
#define SCREEN_HEIGHT ( 64 )

/* 表示物位置 */
#define ROW_TITLE ( 1 )
#define COL_TITLE ( 1 )

#define COL_CURSOR ( 2 )
#define COL_VALUE_L ( 2 )
#define COL_VALUE_R ( 32 )
#define COL_ITEM ( 10 )
#define ROW_1 ( 11 )
#define ROW_2 ( 21 )
#define ROW_3 ( 31 )
#define ROW_4 ( 41 )
#define ROW_5 ( 51 )
#define ROW_6 ( 61 )
#define ROW_7 ( 71 )
#define ROW_8 ( 81 )
#define ROW_9 ( 91 )
#define ROW_10 ( 101 )
#define ROW_11 ( 111 )
#define ROW_12 ( 121 )

#define STR_SIZE ( 32 )

/* 表示パターン */
typedef enum e_display_pattern {
    DISPLAY_MAIN_MENU,
    DISPLAY_SENSOR_VIEW,
    DISPLAY_LINE_TRACE_TEST,
    DISPLAY_ANGLE_CTRL_TEST,
    DISPLAY_MOTOR_TEST,
    DISPLAY_SENSOR_CALIBRATION,
    DISPLAY_PARAMETER,
    DISPLAY_PARAMETER_PARAMETER,
    DISPLAY_SAVE,
} display_pattern_t;

/* センサー補正状態 */
typedef enum e_calibration_Status {
    CALIBRATION_W,   // 白補正
    CALIBRATION_B,   // 黒補正
    CALIBRATION_SAVE // セーブ
} calibration_status_t;

/***********************************/
/* Local Variables                 */
/***********************************/
Adafruit_SSD1306 display( SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1 );
display_pattern_t display_pattern = DISPLAY_MAIN_MENU;
std::vector<const char*> parameter_category;

/***********************************/
/* Global Variables                */
/***********************************/
bool screen_is_connected;

// ライントレーステスト関連
u1 line_trace_test_mode = LINETRACE_TEST_NORMAL; // オフセットテストのモード 0:オフセットなし 1:右 2左

// 角度制御テスト関連
s4 test_deg = 150;
u4 test_time = 150;
u4 test_k = 100;
u1 angle_ctrl_test_state = 0; // 0:初期状態 1:テスト中

// モータテスト関連
s1 motor_test_fl = 0; // モータ 左前のpwm値
s1 motor_test_fr = 0; // モータ 右前のpwm値
s1 motor_test_rl = 0; // モータ 左後のpwm値
s1 motor_test_rr = 0; // モータ 右後のpwm値

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
/*
 * 概要：パラメータの一意なカテゴリリストを作成する
 * 引数：なし
 * 戻り値：なし
 * 詳細：parameter_categoryにカテゴリ名のリストを作る
 *      例：parameter_category =  "LineTrace", "AngleCtrl"...
 */
static void parameter_category_unique() {
    for ( u1 i = 0; i < parameters.size(); i++ ) {
        bool is_unique = true;
        for ( u1 j = 0; j < parameter_category.size(); j++ ) {
            // すでにリストに存在している or 表示したくないカテゴリーの場合はリストに追加しない
            if ( ( strcmp( parameters[i]->get_category_name(), parameter_category[j] ) == 0 ) ||
                 ( parameters[i]->get_category() == CATEGORY_SENSOR_CALIBRATION ) ) {
                is_unique = false;
                break;
            }
        }
        if ( is_unique ) {
            parameter_category.push_back( parameters[i]->get_category_name() );
        }
    }
}

/*
 * 概要：カテゴリ名からパラメータのリストを取得する
 * 引数：カテゴリ名
 * 戻り値：パラメータのリスト
 * 詳細：
 *      例：引数 -> "LineTrace"
 *          戻り値 -> prm_line_trace_P, prm_line_trace_I, prm_line_trace_D
 */
static std::vector<parameter*> get_parameters_by_category( const char* category ) {
    std::vector<parameter*> ret;
    for ( u1 i = 0; i < parameters.size(); i++ ) {
        if ( strcmp( parameters[i]->get_category_name(), category ) == 0 ) {
            ret.push_back( parameters[i] );
        }
    }
    return ret;
}

/*
 * 概要：ディスプレイのタイトルを表示する
 * 引数：タイトル名
 * 戻り値：なし
 * 詳細：タイトルを表示する
 */
static void display_draw_title( const char* str ) {
    display.fillRect( 0, 0, SCREEN_WIDTH, 10, WHITE );
    display.setTextColor( BLACK );
    display.setCursor( ROW_TITLE, COL_TITLE );
    display.print( str );
}

/*
 * 概要：ディスプレイに文字列を表示する
 * 引数：x:表示位置x, y:表示位置y, format:表示文字列
 * 戻り値：なし
 * 詳細：x,y座標とprintfと同じフォーマットで文字列を指定できます
 */
static void display_draw_str( u1 x, u1 y, const char* format, ... ) {
    va_list args;
    char buf[STR_SIZE];
    va_start( args, format );
    vsnprintf( buf, sizeof( buf ), format, args );
    va_end( args );
    display.setCursor( x, y );
    display.setTextColor( WHITE );
    display.print( buf );
}

/*
 * 概要：ディスプレイのトップメニューを表示する
 * 引数：なし
 * 戻り値：なし
 * 詳細：トップメニューを表示
 */
static void display_menu() {
    static u1 menu_cursor = 0;

    display_draw_title( "Main Menu" );

    display_draw_str( COL_ITEM, ROW_1, "Sensor" );
    display_draw_str( COL_ITEM, ROW_2, "LineTrace" );
    display_draw_str( COL_ITEM, ROW_3, "MotorTest" );
    display_draw_str( COL_ITEM, ROW_4, "SensorCal" );
    display_draw_str( COL_ITEM, ROW_5, "AngleCtrl" );
    display_draw_str( COL_ITEM, ROW_6, "Parameter" );

    display_draw_str( COL_CURSOR, ROW_1 + ( menu_cursor ) * 10, ">" );

    button_up.key_repeat_process( []() {
        if ( menu_cursor > 0 ) {
            menu_cursor--;
        }
    } );
    button_down.key_repeat_process( []() {
        if ( menu_cursor < 6 - 1 ) {
            menu_cursor++;
        }
    } );
    if ( button_enter.isPressed() ) {
        switch ( menu_cursor ) {
        case 0:
            display_pattern = DISPLAY_SENSOR_VIEW;
            break;

        case 1:
            display_pattern = DISPLAY_LINE_TRACE_TEST;
            break;

        case 2:
            display_pattern = DISPLAY_MOTOR_TEST;
            break;

        case 3:
            display_pattern = DISPLAY_SENSOR_CALIBRATION;
            break;

        case 4:
            display_pattern = DISPLAY_ANGLE_CTRL_TEST;
            break;

        case 5:
            display_pattern = DISPLAY_PARAMETER;
            break;

        default:
            break;
        }
    }
}

/*
 * 概要：センサ画面
 * 引数：なし
 * 戻り値：なし
 * 詳細：センサ画面
 */
void display_sensor_view() {
    static u1 sensor_view_cursor = 0;

    display_draw_title( "SensorView" );

    display_draw_str( 0, ROW_1, "Dig" );
    display_draw_str( COL_VALUE_R, ROW_1, "%4X", line_digital | gate << 5 );

    display_draw_str( 0, ROW_2, "Err" );
    display_draw_str( COL_VALUE_R, ROW_2, "%4d", line_error );

    display_draw_str( 0, ROW_5, "Enc" );
    display_draw_str( COL_VALUE_R, ROW_5, "%4d", distance );

    display_draw_str( 0, ROW_6, "Slope" );
    display_draw_str( COL_VALUE_R, ROW_6, "%4d", slope_raw );

    display_draw_str( 0, ROW_7, "Angle" );
    display_draw_str( COL_VALUE_R, ROW_7, "%4d", steer_angle );

    display_draw_str( 0, ROW_8, "Batt" );
    display_draw_str( COL_VALUE_R, ROW_8, "%4d", battery_voltage );

    display_draw_str( COL_ITEM, ROW_12, "Back" );

    display_draw_str( COL_CURSOR, ROW_12, ">" );

    if ( button_enter.isPressed() ) {
        display_pattern = DISPLAY_MAIN_MENU;
    }
}

/*
 * 概要：ライントレーステスト画面
 * 引数：なし
 * 戻り値：なし
 * 詳細：ライントレーステスト画面
 */
static u1 line_trace_test_cursor = 0;
static bool trace_test = false;
void display_line_trace_test() {

    display_draw_title( "LTraceTest" );

    parameter* line_trace_parameter[] = { &prm_line_trace_P, &prm_line_trace_I, &prm_line_trace_D, &prm_offset_R_lanechange,
                                          &prm_offset_L_lanechange };
    u1 arysize = array_size( line_trace_parameter );

    display_draw_str( COL_ITEM, ROW_1, "P: %d", ( *line_trace_parameter[0] ).get() );
    display_draw_str( COL_ITEM, ROW_2, "I: %d", ( *line_trace_parameter[1] ).get() );
    display_draw_str( COL_ITEM, ROW_3, "D: %d", ( *line_trace_parameter[2] ).get() );
    display_draw_str( COL_ITEM, ROW_4, "O_R:%d", ( *line_trace_parameter[3] ).get() );
    display_draw_str( COL_ITEM, ROW_5, "O_L:%d", ( *line_trace_parameter[4] ).get() );
    display_draw_str( COL_ITEM, ROW_6, "mode: %d", line_trace_test_mode );
    if ( trace_test ) {
        display_draw_str( COL_ITEM, ROW_7, "[ON] OFF" );
        run_mode_change_to( RUN_TEST_TRACE );
    } else {
        display_draw_str( COL_ITEM, ROW_7, "ON [OFF]" );
        run_mode_change_to( RUN_STOP );
    }

    display_draw_str( COL_ITEM, ROW_12, "Back" );

    if ( line_trace_test_cursor > ( arysize + 1 ) ) {
        display_draw_str( COL_CURSOR, ROW_12, ">" );
    } else {
        display_draw_str( COL_CURSOR, ROW_1 + ( line_trace_test_cursor * 10 ), ">" );
    }

    button_up.key_repeat_process( [&]() {
        if ( line_trace_test_cursor > 0 ) {
            line_trace_test_cursor--;
        }
    } );
    button_down.key_repeat_process( [&]() {
        if ( line_trace_test_cursor <= arysize + 1 ) {
            line_trace_test_cursor++;
        }
    } );
    button_left.key_repeat_process( [&]() {
        if ( line_trace_test_cursor < arysize ) {
            --( *( line_trace_parameter[line_trace_test_cursor] ) );
        } else if ( line_trace_test_cursor == arysize ) {
            if ( line_trace_test_mode > LINETRACE_TEST_NORMAL ) {
                --line_trace_test_mode;
            }
        } else if ( line_trace_test_cursor == ( arysize + 1 ) ) {
            trace_test = true;
        } else {
            // 何もしない
        }
    } );
    button_right.key_repeat_process( [&]() {
        if ( line_trace_test_cursor < arysize ) {
            ++( *( line_trace_parameter[line_trace_test_cursor] ) );
        } else if ( line_trace_test_cursor == arysize ) {
            if ( line_trace_test_mode <= LINETRACE_TEST_OFFSET_L ) {
                ++line_trace_test_mode;
            }
        } else if ( line_trace_test_cursor == ( arysize + 1 ) ) {
            trace_test = false;
        } else {
            // 何もしない
        }
    } );

    if ( button_enter.isPressed() ) {
        if ( ( line_trace_test_cursor > ( arysize + 1 ) ) && ( !trace_test ) ) {
            line_trace_test_cursor = 0;
            display_pattern = DISPLAY_MAIN_MENU;
        }
    }
}

/*
 * 概要：角度制御テスト画面
 * 引数：なし
 * 戻り値：なし
 * 詳細：角度制御テスト画面
 */
static u4 angle_ctrl_start_time = 0;
static u1 angle_ctrl_test_cursor = 0;
void display_angle_ctrl_test() {
    switch ( angle_ctrl_test_state ) {
    case 0:
        display_draw_title( "AngleCtrl" );
        display_draw_str( COL_ITEM, ROW_1, "P: %d", prm_angle_ctrl_P.get() );
        display_draw_str( COL_ITEM, ROW_2, "I: %d", prm_angle_ctrl_I.get() );
        display_draw_str( COL_ITEM, ROW_3, "D: %d", prm_angle_ctrl_D.get() );
        display_draw_str( COL_ITEM, ROW_4, "deg: %d", test_deg );
        display_draw_str( COL_ITEM, ROW_5, "time: %d", test_time );
        display_draw_str( COL_ITEM, ROW_6, "k: %d", test_k );
        display_draw_str( COL_ITEM, ROW_7, "now_deg" );
        display_draw_str( COL_VALUE_R, ROW_8, "%4d", steer_angle );

        display_draw_str( COL_ITEM, ROW_11, "test" );
        display_draw_str( COL_ITEM, ROW_12, "Back" );

        if ( angle_ctrl_test_cursor == 6 ) {
            display_draw_str( COL_CURSOR, ROW_11, ">" );
        } else if ( angle_ctrl_test_cursor == 7 ) {
            display_draw_str( COL_CURSOR, ROW_12, ">" );
        } else {
            display_draw_str( COL_CURSOR, ROW_1 + ( angle_ctrl_test_cursor * 10 ), ">" );
        }

        button_up.key_repeat_process( [&]() {
            if ( angle_ctrl_test_cursor > 0 ) {
                angle_ctrl_test_cursor--;
            }
        } );
        button_down.key_repeat_process( [&]() {
            if ( angle_ctrl_test_cursor < 7 ) {
                angle_ctrl_test_cursor++;
            }
        } );
        button_left.key_repeat_process( [&]() {
            if ( angle_ctrl_test_cursor < 3 ) {
                switch ( angle_ctrl_test_cursor ) {
                case 0:
                    --prm_angle_ctrl_P;
                    break;
                case 1:
                    --prm_angle_ctrl_I;
                    break;
                case 2:
                    --prm_angle_ctrl_D;
                    break;
                default:
                    break;
                }
            } else if ( angle_ctrl_test_cursor == 3 ) {
                if ( test_deg > -450 ) {
                    --test_deg;
                }
            } else if ( angle_ctrl_test_cursor == 4 ) {
                if ( test_time > 0 ) {
                    --test_time;
                }
            } else if ( angle_ctrl_test_cursor == 5 ) {
                if ( test_k > 0 ) {
                    --test_k;
                }
            } else {
                // 何もしない
            }
        } );

        button_right.key_repeat_process( [&]() {
            if ( angle_ctrl_test_cursor < 3 ) {
                switch ( angle_ctrl_test_cursor ) {
                case 0:
                    ++prm_angle_ctrl_P;
                    break;
                case 1:
                    ++prm_angle_ctrl_I;
                    break;
                case 2:
                    ++prm_angle_ctrl_D;
                    break;
                default:
                    break;
                }
            } else if ( angle_ctrl_test_cursor == 3 ) {
                if ( test_deg < 450 ) {
                    ++test_deg;
                }
            } else if ( angle_ctrl_test_cursor == 4 ) {
                if ( test_time < 1000 ) {
                    ++test_time;
                }
            } else if ( angle_ctrl_test_cursor == 5 ) {
                if ( test_k < 100 ) {
                    ++test_k;
                }
            } else {
                // 何もしない
            }
        } );

        if ( button_enter.isPressed() ) {
            if ( angle_ctrl_test_cursor == 6 ) { // test
                run_mode_change_to( RUN_TEST_ANGLE );
                angle_ctrl_test_state = 1;
            } else if ( angle_ctrl_test_cursor == 7 ) { // back
                display_pattern = DISPLAY_MAIN_MENU;
            } else {
                // 何もしない
            }
        }
        break;

    case 1:
        DBG_PRINT( "case1\n" );
        angle_ctrl_start_time = millis();
        angle_ctrl_test_state = 2;
        break;

    case 2:
        display_draw_str( 0, 0, "Testing..." );
        DBG_PRINT( "case2\n" );
        if ( millis() - angle_ctrl_start_time > 1000 ) {
            angle_ctrl_test_state = 3;
        }
        break;

    case 3:
        DBG_PRINT( "case3\n" );
        run_mode_change_to( RUN_STOP );
        angle_ctrl_test_state = 0;
        break;
    }
}

/*
 * 概要：モーターテスト画面
 * 引数：なし
 * 戻り値：なし
 * 詳細：モーターテスト画面
 */
static u1 motor_test_cursor = 0;
static bool motor_test = false;
void display_motor_test() {

    s1* motor_test_parameter[] = { &motor_test_fl, &motor_test_fr, &motor_test_rl, &motor_test_rr };
    u1 arysize = array_size( motor_test_parameter );

    display_draw_title( "Motortest" );

    display_draw_str( COL_ITEM, ROW_1, "FL: %d", *motor_test_parameter[0] );
    display_draw_str( COL_ITEM, ROW_2, "FR: %d", *motor_test_parameter[1] );
    display_draw_str( COL_ITEM, ROW_3, "RL: %d", *motor_test_parameter[2] );
    display_draw_str( COL_ITEM, ROW_4, "RR: %d", *motor_test_parameter[3] );
    if ( motor_test ) {
        display_draw_str( COL_ITEM, ROW_5, "[ON] OFF" );
        run_mode_change_to( RUN_TEST_MOTOR );
    } else {
        display_draw_str( COL_ITEM, ROW_5, "ON [OFF]" );
        run_mode_change_to( RUN_STOP );
    }

    display_draw_str( COL_ITEM, ROW_12, "Back" );

    if ( motor_test_cursor > arysize ) {
        display_draw_str( COL_CURSOR, ROW_12, ">" );
    } else {
        display_draw_str( COL_CURSOR, ROW_1 + ( motor_test_cursor * 10 ), ">" );
    }

    button_up.key_repeat_process( [&]() {
        if ( motor_test_cursor > 0 ) {
            motor_test_cursor--;
        }
    } );
    button_down.key_repeat_process( [&]() {
        if ( motor_test_cursor <= arysize ) {
            motor_test_cursor++;
        }
    } );
    button_left.key_repeat_process( [&]() {
        if ( motor_test_cursor < arysize ) {
            if ( *( motor_test_parameter[motor_test_cursor] ) > -100 ) {
                --( *( motor_test_parameter[motor_test_cursor] ) );
            } else {
                *( motor_test_parameter[motor_test_cursor] ) = -100;
            }
        } else if ( motor_test_cursor == arysize ) {
            motor_test = true;
        } else {
            // 何もしない
        }
    } );
    button_right.key_repeat_process( [&]() {
        if ( motor_test_cursor < ( array_size( motor_test_parameter ) ) ) {
            if ( *( motor_test_parameter[motor_test_cursor] ) < 100 ) {
                ++( *( motor_test_parameter[motor_test_cursor] ) );
            } else {
                *( motor_test_parameter[motor_test_cursor] ) = 100;
            }
        } else if ( motor_test_cursor == arysize ) {
            motor_test = false;
        } else {
            // 何もしない
        }
    } );

    if ( button_enter.isPressed() ) {
        if ( ( motor_test_cursor > arysize ) && ( !motor_test ) ) {
            motor_test_cursor = 0;
            display_pattern = DISPLAY_MAIN_MENU;
        }
    }
}

/*
 * 概要：アナログセンサキャリブレーション画面
 * 引数：なし
 * 戻り値：なし
 * 詳細：アナログセンサキャリブレーション画面
 */
static u1 sensor_calibration_cursor = 0;
static calibration_status_t sensor_calibration_status = CALIBRATION_W;
void display_sensor_calibration() {

    display_draw_title( "SensCal" );

    if ( sensor_calibration_status == CALIBRATION_W ) {
        display_draw_str( COL_ITEM, ROW_1, "White" );
    } else {
        display_draw_str( COL_ITEM, ROW_1, "Black" );
    }

    display_draw_str( COL_ITEM, ROW_2, " %4d", ar3_raw );
    display_draw_str( COL_ITEM, ROW_3, " %4d", ar2_raw );
    display_draw_str( COL_ITEM, ROW_4, " %4d", ar1_raw );
    display_draw_str( COL_ITEM, ROW_5, " %4d", ac_raw );
    display_draw_str( COL_ITEM, ROW_6, " %4d", al1_raw );
    display_draw_str( COL_ITEM, ROW_7, " %4d", al2_raw );
    display_draw_str( COL_ITEM, ROW_8, " %4d", al3_raw );

    if ( sensor_calibration_status == CALIBRATION_W ) {
        display_draw_str( COL_ITEM, ROW_11, "Next" );
    } else {
        display_draw_str( COL_ITEM, ROW_11, "Save" );
    }

    display_draw_str( COL_ITEM, ROW_12, "Back" );

    display_draw_str( COL_CURSOR, ROW_11 + ( sensor_calibration_cursor * 10 ), ">" );

    button_up.key_repeat_process( [&]() {
        if ( sensor_calibration_cursor > 0 ) {
            sensor_calibration_cursor--;
        }
    } );
    button_down.key_repeat_process( [&]() {
        if ( sensor_calibration_cursor < 1 ) {
            sensor_calibration_cursor++;
        }
    } );

    if ( button_enter.isPressed() ) {
        if ( sensor_calibration_cursor == 0 ) {
            switch ( sensor_calibration_status ) {
            case CALIBRATION_W:
                prm_line_AR3_W = ar3_raw;
                prm_line_AR2_W = ar2_raw;
                prm_line_AR1_W = ar1_raw;
                prm_line_AC_W = ac_raw;
                prm_line_AL1_W = al1_raw;
                prm_line_AL2_W = al2_raw;
                prm_line_AL3_W = al3_raw;

                sensor_calibration_status = CALIBRATION_B;
                break;
            case CALIBRATION_B:
                prm_line_AR3_B = ar3_raw;
                prm_line_AR2_B = ar2_raw;
                prm_line_AR1_B = ar1_raw;
                prm_line_AC_B = ac_raw;
                prm_line_AL1_B = al1_raw;
                prm_line_AL2_B = al2_raw;
                prm_line_AL3_B = al3_raw;
                sensor_calibration_status = CALIBRATION_SAVE;
                break;
            default:
                break;
            }
        } else {
            sensor_calibration_cursor = 0;
            sensor_calibration_status = CALIBRATION_W;
            display_pattern = DISPLAY_MAIN_MENU;
        }
    }
}

/*
 * 概要：パラメータ画面
 * 引数：なし
 * 戻り値：なし
 * 詳細：実際にパラメータを変更する画面を表示する
 *      パラメータ数に応じて自動的にページスクロールも対応するため、あまり触る必要無いはず
 */
static const char* selected_parameter_category;
static u1 parameter_parameter_cursor = 0;
void display_parameter_parameter() {

    display_draw_title( selected_parameter_category );

    // カテゴリに属するパラメータを取得
    std::vector<parameter*> prms = get_parameters_by_category( selected_parameter_category );

    // パラメータの表示
    // 1ページに5個まで表示できる
    // 例：prms.size()が13のとき、cursorが1の時は1~5、cursorが6の時は6~10、cursorが11の時は11~13を表示する
    u1 view_page = ( parameter_parameter_cursor ) / 5;   // 0始まり
    u1 view_num = min( 5, prms.size() - view_page * 5 ); // 5個以下
    for ( u1 i = 0; i < view_num; i++ ) {
        u1 idx = i + view_page * 5;
        // パラメータ名
        display_draw_str( COL_ITEM, ROW_1 + i * 20, prms[idx]->get_short_name() );

        // パラメータ値
        // enumがある場合は文字列で表示。それ以外は数値で表示
        if ( prms[idx]->get_enum_num() > 0 ) {
            display_draw_str( 20, ROW_2 + i * 20, "%s", prms[idx]->get_enum_str()[prms[idx]->get()] );
        } else {
            display_draw_str( COL_VALUE_R, ROW_2 + i * 20, "%4d", prms[idx]->get() );
        }
    }

    // 最終ページならBackを表示
    if ( parameter_parameter_cursor / 5 >= prms.size() / 5 ) {
        display_draw_str( COL_ITEM, ROW_12, "Back" );
    }

    // カーソルの表示
    if ( parameter_parameter_cursor == prms.size() ) {
        display_draw_str( COL_CURSOR, ROW_12, ">" );
    } else {
        display_draw_str( COL_CURSOR, ROW_1 + ( (parameter_parameter_cursor)-view_page * 5 ) * 20, ">" );
    }

    button_up.key_repeat_process( [&]() {
        if ( parameter_parameter_cursor > 0 ) {
            parameter_parameter_cursor--;
        }
    } );
    button_down.key_repeat_process( [&]() {
        if ( parameter_parameter_cursor < prms.size() ) {
            parameter_parameter_cursor++;
        }
    } );
    button_left.key_repeat_process(
        [&]() {
            if ( parameter_parameter_cursor < prms.size() ) {
                --( *( prms[parameter_parameter_cursor] ) );
            }
        },
        prms[parameter_parameter_cursor]->get_max_min_diff() );
    button_right.key_repeat_process(
        [&]() {
            if ( parameter_parameter_cursor < prms.size() ) {
                ++( *( prms[parameter_parameter_cursor] ) );
            }
        },
        prms[parameter_parameter_cursor]->get_max_min_diff() );
    if ( button_enter.isPressed() ) {
        if ( parameter_parameter_cursor == prms.size() ) {
            display_pattern = DISPLAY_PARAMETER;
        }
    }
    if ( parameter_parameter_cursor < prms.size() ) {
        DBG_PRINT( "prms[%d]:%d\n", parameter_parameter_cursor, prms[parameter_parameter_cursor]->get() );
    }
}

/*
 * 概要：パラメータカテゴリ画面
 * 引数：なし
 * 戻り値：なし
 * 詳細：パラメータのカテゴリ画面を表示する
 *      parameterに応じて自動的に画面が作られるので、あまり触る必要無いはず
 */
void display_parameter() {
    static u1 parameter_cursor = 0;

    display_draw_title( "Parameter" );

    for ( u1 i = 0; i < parameter_category.size(); i++ ) {
        display_draw_str( COL_ITEM, ROW_1 + i * 10, parameter_category[i] );
    }

    display_draw_str( COL_ITEM, ROW_12, "Back" );

    if ( parameter_cursor == parameter_category.size() ) {
        display_draw_str( COL_CURSOR, ROW_12, ">" );
    } else {
        display_draw_str( COL_CURSOR, ROW_1 + ( parameter_cursor ) * 10, ">" );
    }

    button_up.key_repeat_process( []() {
        if ( parameter_cursor > 0 ) {
            parameter_cursor--;
        }
    } );
    button_down.key_repeat_process( []() {
        if ( parameter_cursor < parameter_category.size() ) {
            parameter_cursor++;
        }
    } );
    if ( button_left.isPressed() ) {
        display_pattern = DISPLAY_MAIN_MENU;
    }
    if ( button_enter.isPressed() ) {
        if ( parameter_cursor == parameter_category.size() ) {
            parameter_cursor = 0;
            display_pattern = DISPLAY_MAIN_MENU;
        } else {
            selected_parameter_category = parameter_category[parameter_cursor];
            parameter_parameter_cursor = 0;
            display_pattern = DISPLAY_PARAMETER_PARAMETER;
        }
    }
}

/*
 * 概要：セーブ画面への遷移を判断する
 * 引数：なし
 * 戻り値：なし
 * 詳細：セーブ画面への遷移
 */
static time_measure saving_timer;
static display_pattern_t after_save_display_pattern; // セーブ後に遷移する画面
static void judge_save() {
    bool start_save = false;

    // 画面ごとでセーブの仕方を変える
    switch ( display_pattern ) {
    case DISPLAY_MAIN_MENU:
    case DISPLAY_SENSOR_VIEW:
    case DISPLAY_LINE_TRACE_TEST:
    case DISPLAY_MOTOR_TEST:
    case DISPLAY_PARAMETER:
    case DISPLAY_PARAMETER_PARAMETER:
    case DISPLAY_ANGLE_CTRL_TEST:
        // セーブボタンが押されたとき
        if ( button_save.isPressed() ) {
            after_save_display_pattern = display_pattern; // セーブ前の画面に戻す
            start_save = true;
        }
        break;
    case DISPLAY_SENSOR_CALIBRATION:
        // 黒補正でNext押下されたとき
        if ( sensor_calibration_status == CALIBRATION_SAVE ) {
            after_save_display_pattern = DISPLAY_SENSOR_CALIBRATION;
            sensor_calibration_status = CALIBRATION_W; // 白補正画面に戻す
            start_save = true;
        }
    default:
        // そもそもここに入るのはおかしいので何もしない
        break;
    }

    // 遷移先がセーブ画面の場合
    if ( start_save ) {
        nvm_save();
        saving_timer.restart();
        display_pattern = DISPLAY_SAVE;
    }
}

/*
 * 概要：セーブ画面処理
 * 引数：なし
 * 戻り値：なし
 * 詳細：パラメータのセーブを行う。実際にはセーブしたことを知らせる画面
 */
static void display_save() {
    display_draw_str( 0, 0, "Saving..." );
    indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_SAVING );
    if ( saving_timer.measure() > 750 ) {
        indicator_set_neopixel_led( NEOPIXEL_LED_PATTERN_OFF );
        nvm_dump();
        display_pattern = after_save_display_pattern;
    }
}

/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
/*
 * 概要：スクリーンセットアップ処理
 * 引数：なし
 * 戻り値：なし
 * 詳細：スクリーンの初期化処理を行う
 * 備考：スクリーン接続検知(screen_is_connected)は起動時のみ判断するため、途中でスクリーンを付けた場合は再起動(orリセット)が必要
 */
void screen_setup() {
    DBG_PRINT( "screen_setup\n" );

    // スクリーン接続検知
    R_IIC1->ICCR1 = 0x1F; // リセット後にSDAがHighになることがあるため初期値をソフトウェアからも再設定
    // Wire.end();
    Wire.begin();

    Wire.setClock( 400000L );
    int retry = 2;
    while ( !screen_is_connected && retry-- ) {
        Wire.beginTransmission( OLED_ADDRESS );
        screen_is_connected = ( Wire.endTransmission() == 0 );
        DBG_PRINT( "%s\n", screen_is_connected ? "screen connected." : "screen disconnected" );
    }

    // スクリーン初期化
    if ( !display.begin( SSD1306_SWITCHCAPVCC, OLED_ADDRESS ) ) {
        DBG_PRINT( "SSD1306 allocation failed\n" );
    }
    display.setTextSize( 1 );
    display.setTextColor( WHITE );
    display.setRotation( 3 );
    display.display();

    // パラメータカテゴリリスト作成
    parameter_category_unique();
    for ( u1 i = 0; i < parameter_category.size(); i++ ) {
        DBG_PRINT( "category[%d]:%s\n", i, parameter_category[i] );
    }
}

/*
 * 概要：スクリーンメイン処理
 * 引数：なし
 * 戻り値：なし
 * 詳細：スクリーンが接続されている場合のみ、スクリーンの制御を行う
 * 備考：スクリーン接続検知(screen_is_connected)は起動時のみ判断するため、途中でスクリーンを付けた場合は再起動(orリセット)が必要
 */
void screen_exec() {
    if ( screen_is_connected ) {
        button_screen_update();
        display.clearDisplay();
        switch ( display_pattern ) {
        case DISPLAY_MAIN_MENU:
            display_menu();
            break;
        case DISPLAY_SENSOR_VIEW:
            display_sensor_view();
            break;
        case DISPLAY_LINE_TRACE_TEST:
            display_line_trace_test();
            break;
        case DISPLAY_ANGLE_CTRL_TEST:
            display_angle_ctrl_test();
            break;
        case DISPLAY_MOTOR_TEST:
            display_motor_test();
            break;
        case DISPLAY_SENSOR_CALIBRATION:
            display_sensor_calibration();
            break;
        case DISPLAY_PARAMETER:
            display_parameter();
            break;
        case DISPLAY_PARAMETER_PARAMETER:
            display_parameter_parameter();
            break;
        case DISPLAY_SAVE:
            display_save();
            break;
        default:
            display_pattern = DISPLAY_MAIN_MENU;
            break;
        }
        display.display();

        // セーブ画面遷移判断
        judge_save();
    }
}