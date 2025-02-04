/*
 * 概要：モータードライバ(A3921)を扱う
 */

#pragma once
#include <Arduino.h>
#include <pwm.h>
#include "defines.h"
#include "mcr_gpt_lib.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
#define FORWARD ( true )
#define REVERSE ( false )

/***********************************/
/* Class                           */
/***********************************/
class motor_driver {
  public:
    motor_driver( pin_size_t pin_h, pin_size_t pin_l, pin_size_t pin_phase, pin_size_t pin_sr, bool invert = false );
    ~motor_driver();

    void begin();
    void stop();
    void set_frequency( u4 freq_hz );
    void set_pwm( s4 pwm );
    void set_mode( BC brake_mode );
    void set_mode_and_pwm( BC brake_mode, s4 pwm );
    s4 get_pwm() {
        return pwm;
    }

  private:
    pin_size_t pin_h;     // A3921 PWMH
    pin_size_t pin_l;     // A3921 PWML
    pin_size_t pin_phase; // A3921 PHASE
    pin_size_t pin_sr;    // A3921 SR
    BC brake_mode;        // BRAKE_MODE or COAST_MODE
    s4 pwm;               // -100 to 100
    bool direction;       // FORWARD or REVERSE
    u4 freq_hz;           // hz
    u1 channel;           // GPT channel(0-7)
    u4 width_value;       // PWM width value[-]
    bool invert;          // Invert output

    void freq_to_width_value( u4 freq_hz );
    void out_pwm( pin_size_t pin, s4 pwm );

    u1 get_port( pin_size_t pin ) {
        return g_pin_cfg[pin].pin >> 8;
    }
    u1 get_bit( pin_size_t pin ) {
        return g_pin_cfg[pin].pin & 0xFF;
    }
    void startPWM_GPT( u1 channel, uint16_t syuuki ) {
        switch ( channel ) {
        case 0:
            startPWM_GPT0( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        case 1:
            startPWM_GPT1( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        case 2:
            startPWM_GPT2( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        case 3:
            startPWM_GPT3( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        case 4:
            startPWM_GPT4( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        case 5:
            startPWM_GPT5( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        case 6:
            startPWM_GPT6( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        case 7:
            startPWM_GPT7( GTIOCA | GTIOCB, DIV1, syuuki );
            break;
        }
    }
};

/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/