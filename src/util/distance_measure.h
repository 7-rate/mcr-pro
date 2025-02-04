/*
 * 概要：走行距離計測用クラス
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
/*
 * 概要：走行距離計測用クラス
 * コンストラクタで現在走行距離を記録し、measureメソッドで記録した地点からの走行距離を取得できます
 * restartメソッドで走行距離を再記録できます
 */
class distance_measure {
  public:
    distance_measure( s4* p_distance ) {
        this->p_distance = p_distance;
        this->start_distance = *p_distance;
    }
    ~distance_measure() {
    }
    void restart() {
        this->start_distance = *p_distance;
    }
    s4 measure() {
        return *p_distance - this->start_distance;
    }

  private:
    s4 start_distance;
    s4* p_distance;
};

/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/