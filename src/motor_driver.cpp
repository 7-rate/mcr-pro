/*
 * 概要：モータードライバ(A3921)を扱う
 */

#include "motor_driver.h"
#include "sensors.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
#define DEFAULT_FREQUENCY_HZ ( 18000 )

/***********************************/
/* Local Variables                 */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/

/***********************************/
/* Class implementions             */
/***********************************/
/* pin_hとpin_lは同じタイマチャンネルのピンを指定すること！ */
/* さもなくば爆発します                                  */
motor_driver::motor_driver( pin_size_t pin_h, pin_size_t pin_l, pin_size_t pin_phase, pin_size_t pin_sr, bool invert )
    : pin_h( pin_h ), pin_l( pin_l ), pin_phase( pin_phase ), pin_sr( pin_sr ), invert( invert ), brake_mode( BRAKE ), pwm( 0 ), direction( FORWARD ),
      freq_hz( DEFAULT_FREQUENCY_HZ ) {
    auto pinconfig = getPinCfgs( pin_h, PIN_CFG_REQ_PWM );
    channel = GET_CHANNEL( pinconfig[0] );
}

motor_driver::~motor_driver() {
    stop();
}

/*
 * 概要：モータードライバの初期化
 * 引数：なし
 * 戻り値：なし
 * 詳細：ピンの初期化とPWMの初期化を行う
 */
void motor_driver::begin() {
    pinMode( pin_phase, OUTPUT );
    pinMode( pin_sr, OUTPUT );

    pinMode( pin_h, OUTPUT );
    pinMode( pin_l, OUTPUT );
    setGPTterminal( get_port( pin_h ), get_bit( pin_h ) );
    DBG_PRINT( "channel: %d\n", channel );
    DBG_PRINT( "pin_h: %d, port: %d, bit: %d\n", pin_h, get_port( pin_h ), get_bit( pin_h ) );
    setGPTterminal( get_port( pin_l ), get_bit( pin_l ) );
    DBG_PRINT( "pin_l: %d, port: %d, bit: %d\n", pin_l, get_port( pin_l ), get_bit( pin_l ) );

    freq_to_width_value( freq_hz );
    DBG_PRINT( "freq_hz: %d, width_value: %d\n", freq_hz, width_value );
    startPWM_GPT( channel, width_value );

    stop();
}

/*
 * 概要：モーターを停止する
 * 引数：なし
 * 戻り値：なし
 * 詳細：モーターを停止する
 */
void motor_driver::stop() {
    set_mode( BRAKE );
    set_pwm( 0 );
}

/*
 * 概要：モーターのpwmを設定する
 * 引数：pwm:-100~100
 * 戻り値：なし
 * 詳細：モーターのpwmを設定する
 */
void motor_driver::set_pwm( s4 pwm ) {
    if ( invert ) {
        pwm = -pwm;
    }

    if ( pwm == this->pwm ) {
        return;
    }

    this->pwm = constrain( pwm, -100, 100 );

    direction = ( pwm > 0 ) ? FORWARD : REVERSE;
    if ( direction == FORWARD ) {
        digitalWrite( pin_phase, HIGH );
    } else {
        digitalWrite( pin_phase, LOW );
    }

    if ( brake_mode == BRAKE ) {
        out_pwm( pin_l, 100 );
        out_pwm( pin_h, pwm );
    } else {
        out_pwm( pin_l, pwm );
        out_pwm( pin_h, pwm );
    }
}

/*
 * 概要：モーターのモードを設定する
 * 引数：brake_mode:COAST or BRAKE
 * 戻り値：なし
 * 詳細：BRAKE,COASTモードの設定を行います
 */
void motor_driver::set_mode( BC brake_mode ) {
    this->brake_mode = brake_mode;
}

/*
 * 概要：モーターのモードとpwmを設定する
 * 引数：brake_mode:COAST or BRAKE
 *      pwm:-100~100
 * 戻り値：なし
 * 詳細：モーターのモードとpwmを設定する
 */
void motor_driver::set_mode_and_pwm( BC brake_mode, s4 pwm ) {
    set_mode( brake_mode );
    set_pwm( pwm );
}

/*
 * 概要：pwmの周波数を設定する
 * 引数：freq_hz:周波数[Hz]
 * 戻り値：なし
 * 詳細：pwmの周波数を設定する
 */
void motor_driver::set_frequency( u4 freq_hz ) {
    this->freq_hz = freq_hz;
    freq_to_width_value( freq_hz );
    startPWM_GPT( channel, width_value );
}

/*
 * 概要：周波数からGPT周期の値を計算する
 * 引数：freq_hz:周波数[Hz]
 * 戻り値：なし
 * 詳細：DIVは1固定で計算するため、733Hz以上の値を設定してください
 */
void motor_driver::freq_to_width_value( u4 freq_hz ) {
    width_value = ( 48000000 / freq_hz ) - 1; // [us]
}

/*
 * 概要：モーターにpwmを出力する
 * 引数：pin:出力するピン
 *     pwm:-100~100[%]
 * 戻り値：なし
 * 詳細：モーターにpwmを出力する
 */
void motor_driver::out_pwm( pin_size_t pin, s4 pwm ) {
    s4 abs_pwm = abs( pwm );
    // 電圧による補正
    // battery_voltageが10Vの時を100%として補正する
    abs_pwm = abs_pwm * 1000 / battery_voltage;
    abs_pwm = constrain( abs_pwm, 0, 100 );

    auto pinconfig = getPinCfgs( pin, PIN_CFG_REQ_PWM );
    bool is_A = IS_PWM_ON_A( pinconfig[0] );
    u4 value = width_value * abs_pwm / 100;

    switch ( channel ) {
    case 0:
        if ( is_A ) {
            GTIOC0A = value;
        } else {
            GTIOC0B = value;
        }
        break;
    case 1:
        if ( is_A ) {
            GTIOC1A = value;
        } else {
            GTIOC1B = value;
        }
        break;
    case 2:

        if ( is_A ) {
            GTIOC2A = value;
        } else {
            GTIOC2B = value;
        }
        break;
    case 3:
        if ( is_A ) {
            GTIOC3A = value;
        } else {
            GTIOC3B = value;
        }
        break;
    case 4:
        if ( is_A ) {
            GTIOC4A = value;
        } else {
            GTIOC4B = value;
        }
        break;
    case 5:
        if ( is_A ) {
            GTIOC5A = value;
        } else {
            GTIOC5B = value;
        }
        break;
    case 6:
        if ( is_A ) {
            GTIOC6A = value;
        } else {
            GTIOC6B = value;
        }
        break;
    case 7:
        if ( is_A ) {
            GTIOC7A = value;
        } else {
            GTIOC7B = value;
        }
        break;
    }
}

/***********************************/
/* Global functions                */
/***********************************/