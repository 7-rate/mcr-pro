/*
 * 概要：マイコンカーで扱う走行用パラメータを管理する
 */

#pragma once
#include "defines.h"
#include <vector>

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/

/***********************************/
/* Class                           */
/***********************************/
class parameter {
  private:
    s4 current_value;
    s4 default_value;
    s4 min_value;
    s4 max_value;
    s4 lsb;
    s4 category;
    const char* short_name;
    const char* description;
    s4 round( s4 value ) {
        if ( value < min_value ) {
            return min_value;
        } else if ( value > max_value ) {
            return max_value;
        } else {
            return value;
        }
    }
    const char** enum_str;
    u1 enum_num;

  public:
    parameter( s4 default_val, s4 min_val, s4 max_val, s4 lsb, s4 cat, const char* sname, const char* desc, const char** enum_str = nullptr,
               u1 enum_num = 0 );

    parameter& operator=( s4 value ) {
        current_value = round( value );
        return *this;
    }

    parameter& operator+=( s4 value ) {
        s4 temp;
        temp = value + current_value;
        current_value = round( temp );
        return *this;
    }

    parameter& operator-=( s4 value ) {
        s4 temp;
        temp = current_value - value;
        current_value = round( temp );
        return *this;
    }

    parameter& operator*=( s4 value ) {
        s4 temp;
        temp = value * current_value;
        current_value = round( temp );
        return *this;
    }

    parameter& operator/=( s4 value ) {
        s4 temp;
        temp = current_value / value;
        current_value = round( temp );
        return *this;
    }

    parameter& operator++() {
        current_value = round( current_value + 1 );
        return *this;
    }

    parameter& operator--() {
        current_value = round( current_value - 1 );
        return *this;
    }

    parameter operator++( int ) {
        parameter temp = *this;
        ++current_value;
        return temp;
    }

    parameter operator--( int ) {
        parameter temp = *this;
        --current_value;
        return temp;
    }

    s4 get() {
        return current_value;
    }
    s4 get_lsb() {
        return lsb;
    }
    s4 get_category() {
        return category;
    }
    s4 get_min() {
        return min_value;
    }
    s4 get_max() {
        return max_value;
    }
    s4 get_max_min_diff() {
        return ( max_value - min_value );
    }
    const char* get_category_name() {
        switch ( category ) {
        case CATEGORY_LINE_TRACE:
            return "LineTrace";
        case CATEGORY_SENSOR_CALIBRATION:
            return "SensCal";
        case CATEGORY_ANGLE_CTRL:
            return "AngleCtrl";
        case CATEGORY_SPEED:
            return "Speed";
        case CATEGORY_CURVE:
            return "Curve";
        case CATEGORY_SECTION_SPEED:
            return "SectSpeed";
        case CATEGORY_DISTANCE:
            return "Distance";
        case CATEGORY_LANE_CHANGE:
            return "LaneChang";
        case CATEGORY_DIFFICULT_KIND:
            return "DiffKind";
        case CATEGORY_CRANK:
            return "Crank";
        default:
            return "Others";
        }
    }
    const char* get_short_name() {
        return short_name;
    }
    const char* get_description() {
        return description;
    }
    const char** get_enum_str() {
        return enum_str;
    }
    u1 get_enum_num() {
        return enum_num;
    }
};

/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/
extern parameter prm_line_trace_P;
extern parameter prm_line_trace_I;
extern parameter prm_line_trace_D;

extern parameter prm_line_AR3_W;
extern parameter prm_line_AR2_W;
extern parameter prm_line_AR1_W;
extern parameter prm_line_AC_W;
extern parameter prm_line_AL1_W;
extern parameter prm_line_AL2_W;
extern parameter prm_line_AL3_W;
extern parameter prm_line_AR3_B;
extern parameter prm_line_AR2_B;
extern parameter prm_line_AR1_B;
extern parameter prm_line_AC_B;
extern parameter prm_line_AL1_B;
extern parameter prm_line_AL2_B;
extern parameter prm_line_AL3_B;

extern parameter prm_angle_ctrl_P;
extern parameter prm_angle_ctrl_I;
extern parameter prm_angle_ctrl_D;

extern parameter prm_speed_stable_P;
extern parameter prm_max_speed;
extern parameter prm_max_speed_slope;
extern parameter prm_max_speed_curve;
extern parameter prm_max_speed_crossline;
extern parameter prm_max_speed_L_lanechange;
extern parameter prm_max_speed_R_lanechange;
extern parameter prm_max_speed_L_crank;
extern parameter prm_max_speed_R_crank;
extern parameter prm_max_speed_distance;
extern parameter prm_max_speed_decline;

extern parameter prm_sharp_curve_force;

extern parameter prm_section_num;
extern parameter prm_max_speed_sec0;
extern parameter prm_max_speed_sec1;
extern parameter prm_max_speed_sec2;
extern parameter prm_max_speed_sec3;
extern parameter prm_max_speed_sec4;
extern parameter prm_max_speed_sec5;
extern parameter prm_max_speed_sec6;
extern parameter prm_max_speed_sec7;
extern parameter prm_max_speed_sec8;
extern parameter prm_max_speed_sec9;

extern parameter prm_stop_distance;
extern parameter prm_difficult_distance_sec0;
extern parameter prm_difficult_distance_sec1;
extern parameter prm_difficult_distance_sec2;
extern parameter prm_difficult_distance_sec3;
extern parameter prm_difficult_distance_sec4;
extern parameter prm_difficult_distance_sec5;
extern parameter prm_difficult_distance_sec6;
extern parameter prm_difficult_distance_sec7;
extern parameter prm_difficult_distance_sec8;
extern parameter prm_difficult_distance_sec9;

extern parameter prm_difficult_kind_0;
extern parameter prm_difficult_kind_1;
extern parameter prm_difficult_kind_2;
extern parameter prm_difficult_kind_3;
extern parameter prm_difficult_kind_4;
extern parameter prm_difficult_kind_5;
extern parameter prm_difficult_kind_6;
extern parameter prm_difficult_kind_7;
extern parameter prm_difficult_kind_8;

extern parameter prm_angle_R_lanechange;
extern parameter prm_angle_L_lanechange;
extern parameter prm_k_R_lanechange;
extern parameter prm_k_L_lanechange;
extern parameter prm_time_R_lanechange;
extern parameter prm_time_L_lanechange;
extern parameter prm_offset_R_lanechange;
extern parameter prm_offset_L_lanechange;

extern parameter prm_angle_R_crank;
extern parameter prm_angle_L_crank;
extern parameter prm_k_R_crank;
extern parameter prm_k_L_crank;
extern parameter prm_time_R_crank;
extern parameter prm_time_L_crank;

extern std::vector<parameter*> parameters;
