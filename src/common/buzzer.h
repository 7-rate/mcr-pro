/*
 * 概要：ブザーを扱う
 * ブザーは単純なON/OFFで音が鳴るものを対象としている(周波数は一定のブザー)
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
 * 概要：ブザーを鳴らすクラス
 * setメソッドを呼び出すと、指定したパターンでブザーが鳴ります
 * 4byte(32bit)で指定でき、1bitあたり50msの間隔で鳴ります
 * 鳴らしている途中にsetメソッドを呼び出すと、新しいパターンに置き換わります
 * 例1：set(0x00000001); の場合、setしてから0~50msだけ音が鳴り、あとは無音です(ﾋﾟｯ)
 * 例2：set(0x00000005); の場合、setしてから0~50msと100~150msだけ音が鳴り、あとは無音です(ﾋﾟｯ ﾋﾟｯ)
 * 注意：process_1msメソッドは1ms周期で呼び出す必要があります
 */
class buzzer {
  public:
    buzzer( pin_size_t pin, u4 interval_ms = 50 ) {
        this->pin = pin;
        this->interval_ms = interval_ms;
        pinMode( pin, OUTPUT );
        pattern = 0;
        temp_time = 0;
    }
    ~buzzer() {
    }
    void set( u4 pattern ) {
        this->pattern = pattern;
        temp_time = millis();
    }
    void process_1ms() {
        u4 time = millis() - temp_time;
        if ( time > interval_ms ) {
            if ( pattern & 0x01 ) {
                digitalWrite( pin, HIGH );
            } else {
                digitalWrite( pin, LOW );
            }
            pattern >>= 1;
            temp_time = millis();
        }
    }

  private:
    pin_size_t pin;
    u4 interval_ms;
    u4 pattern;
    u4 temp_time;
};
/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/