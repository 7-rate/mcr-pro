#ifndef MCR_GPT_LIB_H
#define MCR_GPT_LIB_H

#include "Arduino.h"
#include "FspTimer.h"

// GPT関係
#define GTCCR_A ( 0 )
#define GTCCR_B ( 1 )
#define GTCCR_C ( 2 )
#define GTCCR_E ( 3 )
#define GTCCR_D ( 4 )
#define GTCCR_F ( 5 )

#define GTIOCA ( 0x01 )
#define GTIOCB ( 0x02 )

#define GTETRGA ( 0x01 )
#define GTETRGB ( 0x02 )

// カウントクロック選択
#define DIV1 ( 0b000 )
#define DIV4 ( 0b001 )
#define DIV16 ( 0b010 )
#define DIV64 ( 0b011 )
#define DIV256 ( 0b100 )
#define DIV1024 ( 0b101 )

// PWM　ONの幅設定
#define GTIOC0A ( R_GPT0->GTCCR[GTCCR_C] )
#define GTIOC0B ( R_GPT0->GTCCR[GTCCR_E] )
#define GTIOC1A ( R_GPT1->GTCCR[GTCCR_C] )
#define GTIOC1B ( R_GPT1->GTCCR[GTCCR_E] )
#define GTIOC2A ( R_GPT2->GTCCR[GTCCR_C] )
#define GTIOC2B ( R_GPT2->GTCCR[GTCCR_E] )
#define GTIOC3A ( R_GPT3->GTCCR[GTCCR_C] )
#define GTIOC3B ( R_GPT3->GTCCR[GTCCR_E] )
#define GTIOC4A ( R_GPT4->GTCCR[GTCCR_C] )
#define GTIOC4B ( R_GPT4->GTCCR[GTCCR_E] )
#define GTIOC5A ( R_GPT5->GTCCR[GTCCR_C] )
#define GTIOC5B ( R_GPT5->GTCCR[GTCCR_E] )
#define GTIOC6A ( R_GPT6->GTCCR[GTCCR_C] )
#define GTIOC6B ( R_GPT6->GTCCR[GTCCR_E] )
#define GTIOC7A ( R_GPT7->GTCCR[GTCCR_C] )
#define GTIOC7B ( R_GPT7->GTCCR[GTCCR_E] )

#define GPT0_CNT ( R_GPT0->GTCNT )
#define GPT1_CNT ( R_GPT1->GTCNT )
#define GPT2_CNT ( R_GPT2->GTCNT )
#define GPT3_CNT ( R_GPT3->GTCNT )
#define GPT4_CNT ( R_GPT4->GTCNT )
#define GPT5_CNT ( R_GPT5->GTCNT )
#define GPT6_CNT ( R_GPT6->GTCNT )
#define GPT7_CNT ( R_GPT7->GTCNT )

#define INT_GPT0_CNT ( (int32_t)R_GPT0->GTCNT )
#define INT_GPT1_CNT ( (int32_t)R_GPT1->GTCNT )
#define INT_GPT2_CNT ( (int16_t)R_GPT2->GTCNT )
#define INT_GPT3_CNT ( (int16_t)R_GPT3->GTCNT )
#define INT_GPT4_CNT ( (int16_t)R_GPT4->GTCNT )
#define INT_GPT5_CNT ( (int16_t)R_GPT5->GTCNT )
#define INT_GPT6_CNT ( (int16_t)R_GPT6->GTCNT )
#define INT_GPT7_CNT ( (int16_t)R_GPT7->GTCNT )

void setGPTterminal( uint8_t port1, uint8_t port2 );

void startPWM_GPT0( uint8_t ch_ab, uint8_t div, uint16_t syuuki );
void startPWM_GPT1( uint8_t ch_ab, uint8_t div, uint16_t syuuki );
void startPWM_GPT2( uint8_t ch_ab, uint8_t div, uint16_t syuuki );
void startPWM_GPT3( uint8_t ch_ab, uint8_t div, uint16_t syuuki );
void startPWM_GPT4( uint8_t ch_ab, uint8_t div, uint16_t syuuki );
void startPWM_GPT5( uint8_t ch_ab, uint8_t div, uint16_t syuuki );
void startPWM_GPT6( uint8_t ch_ab, uint8_t div, uint16_t syuuki );
void startPWM_GPT7( uint8_t ch_ab, uint8_t div, uint16_t syuuki );

void startGPT0_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );
void startGPT1_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );
void startGPT2_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );
void startGPT3_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );
void startGPT4_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );
void startGPT5_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );
void startGPT6_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );
void startGPT7_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 );

void startGPT0_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );
void startGPT1_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );
void startGPT2_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );
void startGPT3_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );
void startGPT4_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );
void startGPT5_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );
void startGPT6_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );
void startGPT7_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 );

#endif // MCR_GPT_LIB_H
