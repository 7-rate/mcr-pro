/*
 * 概要：計算用のユーティリティ関数
 */
#include "calc_utils.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/

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

/***********************************/
/* Global functions                */
/***********************************/
/*
 * 概要：e^xを高速に計算する
 * 引数：x:指数
 * 戻り値：e^x
 * 詳細：e^xを高速に計算する
 */
f4 fast_exp( f4 x ) {
    constexpr f4 a = ( 1 << 23 ) / 0.69314718f;
    constexpr f4 b = ( 1 << 23 ) * ( 127 - 0.043677448f );
    x = a * x + b;

    constexpr f4 c = ( 1 << 23 );
    constexpr f4 d = ( 1 << 23 ) * 255;
    if ( x < c || x > d )
        x = ( x < c ) ? 0.0f : d;

    u4 n = static_cast<u4>( x );
    memcpy( &x, &n, 4 );
    return x;
}

/*
 * 概要：整数の平方根を求める関数
 * 引数：x 平方根を求めたい整数 (非負整数)
 * 戻り値：xの平方根の整数部分
 * 詳細：2分探索を用いて整数の平方根を計算する
 */
s4 isqrt( s4 x ) {
    if ( x < 0 )
        return 0; // 負の数の場合は0を返す
    if ( x == 0 )
        return 0;
    if ( x == 1 )
        return 1;

    s4 left = 1;
    s4 right = x;
    s4 result = 0;

    while ( left <= right ) {
        // オーバーフロー対策のため、加算で中間値を求める
        s4 mid = left + ( right - left ) / 2;

        // mid * midのオーバーフロー対策
        // x/mid == midとなる最大のmidを探す
        if ( mid <= x / mid ) {
            result = mid;
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    return result;
}

/*
 * 概要：角度制御のためのシグモイド関数
 * 引数： start_a...start_angle[0.1deg]
 *       target_a...target_angle[0.1deg]
 *       t...target_time[ms]
 *       x...time[ms]
 *       k...係数[0.001]
 * 戻り値：((target_a-start_a)/(1+e^(-k(x-t/2))))+start_a [0.1deg]
 * 詳細：f(x) = ((target_a-start_a)/(1+e^(-k(x-t/2))))+start_aと定義されるシグモイド関数を計算する
 * kが大きいほど急峻なシグモイド関数になる
 */
s4 calc_custom_sigmoid( s4 start_a, s4 target_a, s4 t, s4 x, s4 k ) {
    f4 ex;
    f4 temp;

    ex = (f4)( -k / 1000.0f ) * ( (f4)x - ( (f4)t / 2.0f ) );
    temp = (s4)( (f4)( target_a - start_a ) / ( 1.0f + fast_exp( ex ) ) );
    temp += start_a;

    return (s4)temp;
}

/*
 * 概要：シグモイド関数の係数kの最小値を計算する
 * 引数： a...target_angle[0.1deg]
 *       t...target_time[ms]
 * 戻り値：係数kの最小値[0.001]
 * 詳細：以下2つの条件を満たす係数kの最小値を計算する
 *             0 ≦ f(0) ≦ 1
 *           a-1 ≦ f(t) ≦ a
 * 備考：傾きを求めたいだけであるため、a,tは正の値にして計算する
 */
s4 calc_custom_sigmoid_k_min( s4 a, s4 t ) {
    s4 k_min;
    s4 f_0;
    s4 f_t;

    a = abs( a );
    t = abs( t );

    // 少なくとも正の範囲であるため0.0fからスタートして探索する
    // 多分単一式で解けるはずだが、この関数は走行中に呼び出されるものではないため、パフォーマンスは無視して確実に動作することを優先する
    for ( k_min = 1;; k_min++ ) {
        f_0 = calc_custom_sigmoid( 0, a, t, 0, k_min );
        f_t = calc_custom_sigmoid( 0, a, t, t, k_min );
        if ( 0 <= f_0 && f_0 <= 1 && a - 1 <= f_t && f_t <= a ) {
            break;
        }
    }

    return k_min;
}

/*
 * 概要：角度を計算する
 * 引数： tar_angle...目標角度[0.1deg]
 *       tar_time...目標時間[ms]
 *       time...制御開始からの時間[ms]
 *       k...シグモイド関数の係数k[0.001]
 * 戻り値：現在の目標角度[0.1deg]
 * 詳細：目標角度を時間に応じて変化させる
 * 備考：1回あたり4us程度の計算時間がかかる
 */
s4 calc_angle_sigmoid( s4 start_angle, s4 tar_angle, s4 tar_time, s4 time, s4 k ) {
    return calc_custom_sigmoid( start_angle, tar_angle, tar_time, time, k );
}

/*
 * 概要：角度を計算する(線形)
 * 引数： tar_angle...目標角度[0.1deg]
 *       tar_time...目標時間[ms]
 *       time...制御開始からの時間[ms]
 * 戻り値：現在の目標角度[0.1deg]
 * 詳細：目標角度を時間に応じて変化させる
 * 備考
 */
s4 calc_angle_linear( s4 start_angle, s4 tar_angle, s4 tar_time, s4 time ) {
    s4 angle;
    angle = ( tar_angle - start_angle ) * time / tar_time + start_angle;
    if ( tar_angle - start_angle > 0 ) {
        angle = constrain( angle, start_angle, tar_angle );
    } else {
        angle = constrain( angle, tar_angle, start_angle );
    }
    return angle;
}