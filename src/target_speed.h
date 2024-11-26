/*
 * 概要：状況により目標速度を判断する
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

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void target_speed_update();
void increase_section_cnt();
void start_difficult( enum e_run_mode mode );

/***********************************/
/* Global Variables                */
/***********************************/
extern s4 speed_stable;
extern s4 speed_slope;
extern s4 speed_curve;
extern s4 speed_crossline;
extern s4 speed_L_lanechange;
extern s4 speed_R_lanechange;
extern s4 speed_L_crank;
extern s4 speed_R_crank;