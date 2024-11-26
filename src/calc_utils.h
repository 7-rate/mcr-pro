/*
 * 概要：計算用のユーティリティ関数
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
f4 fast_exp( f4 x );
s4 isqrt( s4 x );
s4 calc_custom_sigmoid( s4 start_a, s4 target_a, s4 t, s4 x, s4 k );
s4 calc_custom_sigmoid_k_min( s4 a, s4 t );
s4 calc_angle_sigmoid( s4 start_angle, s4 tar_angle, s4 tar_time, s4 time, s4 k );
s4 calc_angle_linear( s4 start_angle, s4 tar_angle, s4 tar_time, s4 time );

/***********************************/
/* Global Variables                */
/***********************************/
