/*
 * 概要：時間計測用クラス
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
 * 概要：時間計測用クラス
 * コンストラクタで現在時刻を記録し、measureメソッドで経過時間を取得できます
 * restartメソッドで現在時刻を再記録できます
 */
class time_measure {
  public:
    time_measure() {
        start_time = millis();
    }
    ~time_measure() {
    }
    void restart() {
        start_time = millis();
    }
    u4 measure() {
        return millis() - start_time;
    }

  private:
    u4 start_time;
};

/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/