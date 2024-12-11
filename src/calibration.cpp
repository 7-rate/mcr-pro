/*
 * 概要：マイコンカーで扱う走行用パラメータを管理する
 */

#include "calibration.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
#define SPEED_LSB ( LSB_001 ) // 速度のLSB 0.01m/s

/***********************************/
/* Local Variables                 */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/
std::vector<parameter*> parameters;

/* 省略名は9文字まで */
/*        パラメータ名 / 初期値 / min / max / LSB / カテゴリー  / 省略名 / 内容 */
parameter prm_line_trace_P( 100, 0, 10000, LSB_1, CATEGORY_LINE_TRACE, "lineP", "ライントレース時のP値" );
parameter prm_line_trace_I( 10, 0, 1000, LSB_1, CATEGORY_LINE_TRACE, "lineI", "ライントレース時のI値" );
parameter prm_line_trace_D( 10, 0, 1000, LSB_1, CATEGORY_LINE_TRACE, "lineD", "ライントレース時のD値" );

parameter prm_line_AR3_W( 800, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ar3_W", "AR3_白" );
parameter prm_line_AR2_W( 800, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ar2_W", "AR2_白" );
parameter prm_line_AR1_W( 800, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ar1_W", "AR1_白" );
parameter prm_line_AC_W( 800, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ac_W", "AC_白" );
parameter prm_line_AL1_W( 800, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "al1_W", "AL1_白" );
parameter prm_line_AL2_W( 800, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "al2_W", "AL2_白" );
parameter prm_line_AL3_W( 800, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "al3_W", "AL3_白" );
parameter prm_line_AR3_B( 100, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ar3_B", "AR3_黒" );
parameter prm_line_AR2_B( 100, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ar2_B", "AR2_黒" );
parameter prm_line_AR1_B( 100, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ar1_B", "AR1_黒" );
parameter prm_line_AC_B( 100, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "ac_B", "AC_黒" );
parameter prm_line_AL1_B( 100, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "al1_B", "AL1_黒" );
parameter prm_line_AL2_B( 100, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "al2_B", "AL2_黒" );
parameter prm_line_AL3_B( 100, 0, 1024, LSB_1, CATEGORY_SENSOR_CALIBRATION, "al3_B", "AL3_黒" );

parameter prm_angle_ctrl_P( 1000, 0, 1000, LSB_1, CATEGORY_ANGLE_CTRL, "angleP", "角度制御時のP値" );
parameter prm_angle_ctrl_I( 0, 0, 1000, LSB_1, CATEGORY_ANGLE_CTRL, "angleI", "角度制御時のI値" );
parameter prm_angle_ctrl_D( 0, 0, 1000, LSB_1, CATEGORY_ANGLE_CTRL, "angleD", "角度制御時のD値" );

parameter prm_speed_stable_P( 700, 0, 2000, LSB_1, CATEGORY_SPEED, "speedP", "定常速度制御時のP値" );
parameter prm_max_speed( 600, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_max", "最大速度(定常) LSB:0.01m/s" );
parameter prm_max_speed_slope( 600, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_slope", "最大速度(坂道) LSB:0.01m/s" );
parameter prm_max_speed_curve( 400, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_curve", "最大速度(カーブ) LSB:0.01m/s" );
parameter prm_max_speed_crossline( 320, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_xline", "最大速度(クロスライン検出後) LSB:0.01m/s" );
parameter prm_max_speed_L_lanechange( 420, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_Llane", "最大速度(左レーンチェンジ) LSB:0.01m/s" );
parameter prm_max_speed_R_lanechange( 420, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_Rlane", "最大速度(右レーンチェンジ) LSB:0.01m/s" );
parameter prm_max_speed_L_crank( 320, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_Lcrank", "最大速度(左クランク) LSB:0.01m/s" );
parameter prm_max_speed_R_crank( 320, 0, 1200, SPEED_LSB, CATEGORY_SPEED, "sp_Rcrank", "最大速度(右クランク) LSB:0.01m/s" );
parameter prm_max_speed_distance( 20, 0, 200, LSB_1, CATEGORY_SPEED, "sp_dist", "最大速度補正距離 LSB:1m" );
parameter prm_max_speed_decline( 90, 0, 100, LSB_1, CATEGORY_SPEED, "sp_declin", "最大速度補正割合 LSB:1per" );

parameter prm_sharp_curve_force( 400, 0, 1200, LSB_01, CATEGORY_CURVE, "curveN_th", "急カーブ判定遠心力閾値 LSB:0.1N" );

parameter prm_section_num( 4, 1, 10, LSB_1, CATEGORY_SECTION_SPEED, "sec_num", "セクション数" );
parameter prm_max_speed_sec0( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec0", "セクション0最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec1( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec1", "セクション1最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec2( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec2", "セクション2最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec3( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec3", "セクション3最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec4( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec4", "セクション4最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec5( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec5", "セクション5最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec6( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec6", "セクション6最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec7( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec7", "セクション7最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec8( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec8", "セクション8最大速度 LSB:0.01m/s" );
parameter prm_max_speed_sec9( 600, 0, 1200, SPEED_LSB, CATEGORY_SECTION_SPEED, "sp_sec9", "セクション9最大速度 LSB:0.01m/s" );

parameter prm_stop_distance( 10, 0, 200, LSB_1, CATEGORY_DISTANCE, "stop_dist", "走行距離 LSB:1m" );
parameter prm_difficult_distance_sec0( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt0", "0番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec1( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt1", "1番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec2( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt2", "2番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec3( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt3", "3番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec4( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt4", "4番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec5( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt5", "5番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec6( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt6", "6番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec7( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt7", "7番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec8( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt8", "8番目難所マーカーから難所実行までの距離 LSB:1mm" );
parameter prm_difficult_distance_sec9( 500, 0, 1200, LSB_1, CATEGORY_DISTANCE, "difficlt9", "9番目難所マーカーから難所実行までの距離 LSB:1mm" );

const char* difficult_kind[] = { "CR_L", "CR_R", "LC_L", "LC_R" };
parameter prm_difficult_kind_0( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind0", "0番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_1( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind1", "1番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_2( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind2", "2番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_3( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind3", "3番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_4( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind4", "4番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_5( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind5", "5番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_6( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind6", "6番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_7( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind7", "7番目の難所種別 LSB:1[-]", difficult_kind, 4 );
parameter prm_difficult_kind_8( 0, 0, 3, LSB_1, CATEGORY_DIFFICULT_KIND, "dif_kind8", "8番目の難所種別 LSB:1[-]", difficult_kind, 4 );

parameter prm_angle_R_lanechange( 120, 0, 600, LSB_01, CATEGORY_LANE_CHANGE, "an_Rlane", "右レーンチェンジ時曲げ角度 LSB:0.1deg" );
parameter prm_angle_L_lanechange( -120, -600, 0, LSB_01, CATEGORY_LANE_CHANGE, "an_Llane", "左レーンチェンジ時曲げ角度 LSB:0.1deg" );
parameter prm_time_R_lanechange( 50, 0, 10000, LSB_1, CATEGORY_LANE_CHANGE, "tm_Rlane", "右レーンチェンジ時曲げ時間 LSB:1ms" );
parameter prm_time_L_lanechange( 50, 0, 10000, LSB_1, CATEGORY_LANE_CHANGE, "tm_Llane", "左レーンチェンジ時曲げ時間 LSB:1ms" );
parameter prm_k_R_lanechange( 100, 0, 1000, LSB_001, CATEGORY_LANE_CHANGE, "k_Rlane", "右レーンチェンジ角度制御時係数 LSB:0.01" );
parameter prm_k_L_lanechange( 100, 0, 1000, LSB_001, CATEGORY_LANE_CHANGE, "k_Llane", "左レーンチェンジ角度制御時係数 LSB:0.01" );
parameter prm_offset_R_lanechange( 200, -1000, 1000, LSB_1, CATEGORY_LANE_CHANGE, "offset_R", "右レーンチェンジ時オフセット" );
parameter prm_offset_L_lanechange( -200, -1000, 1000, LSB_1, CATEGORY_LANE_CHANGE, "offset_L", "左レーンチェンジ時オフセット" );

parameter prm_angle_R_crank( 400, 0, 600, LSB_01, CATEGORY_CRANK, "an_Rcrank", "右クランク時曲げ角度 LSB:0.1deg" );
parameter prm_angle_L_crank( -400, -600, 0, LSB_01, CATEGORY_CRANK, "an_Lcrank", "左クランク時曲げ角度 LSB:0.1deg" );
parameter prm_time_R_crank( 140, 0, 10000, LSB_1, CATEGORY_CRANK, "tm_Rcrank", "右クランク時曲げ時間 LSB:1ms" );
parameter prm_time_L_crank( 140, 0, 10000, LSB_1, CATEGORY_CRANK, "tm_Lcrank", "左クランク時曲げ時間 LSB:1ms" );
parameter prm_k_R_crank( 100, 0, 1000, LSB_001, CATEGORY_CRANK, "k_Rcrank", "右クランク角度制御時係数 LSB:0.01" );
parameter prm_k_L_crank( 100, 0, 1000, LSB_001, CATEGORY_CRANK, "k_Lcrank", "左クランク角度制御時係数 LSB:0.01" );

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/

/***********************************/
/* Class implementions             */
/***********************************/
parameter::parameter( s4 default_val, s4 min_val, s4 max_val, s4 lsb, s4 cat, const char* sname, const char* desc, const char** enum_str,
                      u1 enum_num )
    : current_value( default_val ), default_value( default_val ), min_value( min_val ), max_value( max_val ), lsb( lsb ), category( cat ),
      short_name( sname ), description( desc ), enum_str( enum_str ), enum_num( enum_num ) {
    parameters.push_back( this );
}
