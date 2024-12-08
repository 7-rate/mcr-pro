/*
 * 概要：状況により目標速度を判断する
 */

#include "target_speed.h"
#include "calibration.h"
#include "sensors.h"
#include "distance_measure.h"
#include "calc_utils.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
#define DISTANCE_STALE_BUFFER ( 50 ) // 安定区間のバッファ距離 1[mm]
// 調整したくなったらパラメータ化してもいいかも？

typedef struct {
    // 難所マーカー～難所開始までの距離
    s4 difficult_distance;
    // 難所種別
    s4 kind;
    // 難所の最終目標速度
    s4 speed_difficult;
    // 難所マーカー～難所開始までを以下の3区間に区切って制御する
    s4 distance_free_run;      // 1.空走区間
    s4 distance_slow_down;     // 2.減速区間
    s4 distance_stable_buffer; // 3.安定区間
} difficult_ctrl_t;

/***********************************/
/* Local Variables                 */
/***********************************/
static u1 section_cnt;
static enum e_run_mode run_mode_difficult;
static distance_measure difficult_marker_distance( &distance ); // 難所読み取りからの距離計測
static s4 speed_initial_from_marker;                            // 難所読み取り時の速度
static difficult_ctrl_t difficult_ctrl;                         // 難所制御パラメータ

static parameter* difficult_distances[] = { &prm_difficult_distance_sec0, &prm_difficult_distance_sec1, &prm_difficult_distance_sec2,
                                            &prm_difficult_distance_sec3, &prm_difficult_distance_sec4, &prm_difficult_distance_sec5,
                                            &prm_difficult_distance_sec6, &prm_difficult_distance_sec7, &prm_difficult_distance_sec8,
                                            &prm_difficult_distance_sec9 };
static parameter* difficult_kinds[] = { &prm_difficult_kind_0, &prm_difficult_kind_1, &prm_difficult_kind_2,
                                        &prm_difficult_kind_3, &prm_difficult_kind_4, &prm_difficult_kind_5,
                                        &prm_difficult_kind_6, &prm_difficult_kind_7, &prm_difficult_kind_8 };
static parameter* section_speeds[] = { &prm_max_speed_sec0, &prm_max_speed_sec1, &prm_max_speed_sec2, &prm_max_speed_sec3, &prm_max_speed_sec4,
                                       &prm_max_speed_sec5, &prm_max_speed_sec6, &prm_max_speed_sec7, &prm_max_speed_sec8, &prm_max_speed_sec9 };

/***********************************/
/* Global Variables                */
/***********************************/
s4 speed_stable;
s4 speed_slope;
s4 speed_curve;
s4 speed_crossline;
s4 speed_L_lanechange;
s4 speed_R_lanechange;
s4 speed_L_crank;
s4 speed_R_crank;

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
/*
 * 概要：難所時の目標速度計算
 * 引数：speed_initial ハーフライン、クロスライン読み取り時の速度 0.01[m/s]
 *      speed_final 難所制御時の速度 0.01[m/s]
 *      distance 難所読み取りからの距離 1[mm]
 * 戻り値：distanceに応じた目標速度(v) 0.01[m/s]
 * 詳細：運動方程式を用いて目標速度を計算する
 *       v^2 - v0^2 = 2ax
 *       v = sqrt( v0^2 + 2ax )
 */
static s4 calc_target_speed( s4 speed_initial, s4 speed_final, s4 distance ) {
    s4 speed_target;
    if ( speed_initial < speed_final ) {
        speed_target = isqrt( sq( speed_initial ) + 2 * ( ACCELERATION * distance ) );
        speed_target = constrain( speed_target, speed_initial, speed_final );
    } else {
        speed_target = isqrt( sq( speed_initial ) + 2 * ( DECELERATION * distance ) );
        speed_target = constrain( speed_target, speed_final, speed_initial );
    }

    return speed_target;
}

/*
 * 概要：難所目標速度の更新
 * 引数：なし
 * 戻り値：なし
 * 詳細：難所目標速度の更新
 */
static void difficult_target_speed_update() {
    u1 idx = section_cnt;
    s4 distance;

    switch ( difficult_ctrl.kind ) {
    case CR_L:
    case CR_R: // fallthrough
        if ( difficult_ctrl.distance_free_run > difficult_marker_distance.measure() ) {
            // 空走区間
            speed_crossline = speed_stable;
        } else if ( difficult_ctrl.distance_slow_down > difficult_marker_distance.measure() ) {
            // 減速区間
            distance = difficult_marker_distance.measure() - difficult_ctrl.distance_free_run;
            speed_crossline = calc_target_speed( speed_initial_from_marker, prm_max_speed_crossline.get(), distance );
        } else {
            // 安定区間
            speed_crossline = prm_max_speed_crossline.get();
        }
        break;
    case LC_L:
        if ( difficult_ctrl.distance_free_run > difficult_marker_distance.measure() ) {
            // 空走区間
            speed_L_lanechange = speed_stable;
        } else if ( difficult_ctrl.distance_slow_down > difficult_marker_distance.measure() ) {
            // 減速区間
            distance = difficult_marker_distance.measure() - difficult_ctrl.distance_free_run;
            speed_L_lanechange = calc_target_speed( speed_initial_from_marker, prm_max_speed_L_lanechange.get(), distance );
        } else {
            // 安定区間
            speed_L_lanechange = prm_max_speed_L_lanechange.get();
        }
        break;
    case LC_R:
        if ( difficult_ctrl.distance_free_run > difficult_marker_distance.measure() ) {
            // 空走区間
            speed_R_lanechange = speed_stable;
        } else if ( difficult_ctrl.distance_slow_down > difficult_marker_distance.measure() ) {
            // 減速区間
            distance = difficult_marker_distance.measure() - difficult_ctrl.distance_free_run;
            speed_R_lanechange = calc_target_speed( speed_initial_from_marker, prm_max_speed_R_lanechange.get(), distance );
        } else {
            // 安定区間
            speed_R_lanechange = prm_max_speed_R_lanechange.get();
        }
        break;
    default:
        break;
    }
}

/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
/*
 * 概要：セクションカウントを増やす
 * 引数：なし
 * 戻り値：なし
 * 詳細：セクションカウントを増やす
 *      セクション数が最大値を超えないようにする
 *      超えた場合は0に戻す
 */
void increase_section_cnt() {
    if ( section_cnt < SECTION_MAX - 1 ) {
        section_cnt++;
    } else {
        section_cnt = 0;
    }
}

/*
 * 概要：難所読み取りからの距離計測をスタートする
 * 引数：run_mode 走行モード
 * 戻り値：なし
 * 詳細：難所読み取りからの距離計測をスタートする
 */
void start_difficult( enum e_run_mode mode ) {
    u1 idx = section_cnt;

    switch ( mode ) {
    case RUN_X_LINE_TRACE:
    case RUN_R_LANE_CHANGE:
    case RUN_L_LANE_CHANGE:
        run_mode_difficult = mode;
        speed_initial_from_marker = speed;
        difficult_marker_distance.restart();

        // 難所種別
        difficult_ctrl.kind = difficult_kinds[idx]->get();

        // 難所の最終目標速度
        switch ( difficult_ctrl.kind ) {
        case CR_L:
        case CR_R: // fallthrough
            difficult_ctrl.speed_difficult = prm_max_speed_crossline.get();
            break;
        case LC_L:
            difficult_ctrl.speed_difficult = prm_max_speed_L_lanechange.get();
            break;
        case LC_R:
            difficult_ctrl.speed_difficult = prm_max_speed_R_lanechange.get();
            break;
        default:
            break;
        }

        // マーカー～難所開始までの距離(パラメータで設定した値)
        difficult_ctrl.difficult_distance = difficult_distances[idx]->get();

        // 減速区間距離
        difficult_ctrl.distance_slow_down =
            ( ( difficult_ctrl.speed_difficult - speed_initial_from_marker ) / DECELERATION ) * speed_initial_from_marker;

        // 安定区間距離(固定値)
        difficult_ctrl.distance_stable_buffer = DISTANCE_STALE_BUFFER;

        // 空走区間距離
        difficult_ctrl.distance_free_run =
            difficult_ctrl.difficult_distance - difficult_ctrl.distance_slow_down - difficult_ctrl.distance_stable_buffer;
        break;

    default:
        break;
    }
}

/*
 * 概要：状況に応じてパラメータの速度を超えない範囲で目標速度を更新する
 * 引数：なし
 * 戻り値：なし
 * 詳細：各目標速度の更新
 */
void target_speed_update() {
    // 走行距離によるグリップ低下を考慮した係数
    s4 distance_k = map( distance, 0, prm_max_speed_distance.get() * 1000, 100, prm_max_speed_decline.get() ); // 20mで10%低下
    distance_k = constrain( distance_k, 90, 100 );

    // 温度によるグリップ低下を考慮した係数
    s4 temperature_k = map( temperature, -5, 20, 95, 100 ); // -5℃で5%低下
    temperature_k = constrain( temperature_k, 95, 100 );

    // 定常時の目標速度判断
    // 影響するもの
    // ・セクション区間
    // ・走行距離によるグリップ低下
    // ・温度
    speed_stable = min( prm_max_speed.get(), section_speeds[section_cnt]->get() );
    speed_stable = ( speed_stable * distance_k ) / 100;
    speed_stable = ( speed_stable * temperature_k ) / 100;

    // 坂道時の目標速度判断
    speed_slope = prm_max_speed_slope.get();
    speed_slope = ( speed_slope * distance_k ) / 100;
    speed_slope = ( speed_slope * temperature_k ) / 100;

    // カーブ時の目標速度判断
    speed_curve = prm_max_speed_curve.get();
    speed_curve = ( speed_curve * distance_k ) / 100;
    speed_curve = ( speed_curve * temperature_k ) / 100;

    // 左クランク時の目標速度判断
    speed_L_crank = prm_max_speed_L_crank.get();
    speed_L_crank = ( speed_L_crank * distance_k ) / 100;
    speed_L_crank = ( speed_L_crank * temperature_k ) / 100;

    // 右クランク時の目標速度判断
    speed_R_crank = prm_max_speed_R_crank.get();
    speed_R_crank = ( speed_R_crank * distance_k ) / 100;
    speed_R_crank = ( speed_R_crank * temperature_k ) / 100;

    // クロスライン、レーンチェンジ時の目標速度判断
    difficult_target_speed_update();
    speed_crossline = ( speed_crossline * distance_k ) / 100;
    speed_crossline = ( speed_crossline * temperature_k ) / 100;
    speed_L_lanechange = ( speed_L_lanechange * distance_k ) / 100;
    speed_L_lanechange = ( speed_L_lanechange * temperature_k ) / 100;
    speed_R_lanechange = ( speed_R_lanechange * distance_k ) / 100;
    speed_R_lanechange = ( speed_R_lanechange * temperature_k ) / 100;
}
