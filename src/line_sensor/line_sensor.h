/*
 * 概要：ラインセンサーの抽象レイヤー
 */
#pragma once
#include <Arduino.h>
#include "defines.h"
#include "features.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/

/***********************************/
/* Class                           */
/***********************************/
class line_sensor {
  public:
    line_sensor() {};
    ~line_sensor() {};

  public:
    /***********************************/
    /* 初期化                           */
    /***********************************/
    virtual void init() = 0;

    /***********************************/
    /* センサ値の更新                   */
    /***********************************/
    virtual void update() = 0;

    /***********************************/
    /* ライン状態判断                   */
    /***********************************/
    virtual bool x_line( u1 ld ) = 0;
    virtual bool left_half_line( u1 ld ) = 0;
    virtual bool right_half_line( u1 ld ) = 0;
    virtual bool difficult( u1 ld ) = 0;
    virtual bool near_center( u1 ld ) = 0;
    virtual bool get_gate() = 0;
    virtual bool all_black() = 0;
    virtual bool stop_pattern() = 0;

    /***********************************/
    /* 右クランク用判断                  */
    /***********************************/
    virtual bool right_crank_outline() = 0;
    virtual bool right_crank_inline() = 0;

    /***********************************/
    /* 左クランク用判断                  */
    /***********************************/
    virtual bool left_crank_outline() = 0;
    virtual bool left_crank_inline() = 0;

    /***********************************/
    /* 右レーンチェンジ用判断            */
    /***********************************/
    virtual bool right_lanechange_next_lane() = 0;

    /***********************************/
    /* 左レーンチェンジ用判断            */
    /***********************************/
    virtual bool left_lanechange_next_lane() = 0;

    bool x_line() {
        return x_line( line_digital );
    }
    bool left_half_line() {
        return left_half_line( line_digital );
    }
    bool right_half_line() {
        return right_half_line( line_digital );
    }
    bool difficult() {
        return difficult( line_digital );
    }
    bool near_center() {
        return near_center( line_digital );
    }

  public:
    u1 line_digital; // ラインセンサーのデジタル値
    s4 line_error;   // センタラインからのズレ量
};

/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/
extern line_sensor& ls;