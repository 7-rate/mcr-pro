//**************************************************************************
// ファイル内容     General PWM Timer (GPT)  制御ライブラリ
// バージョン   Ver.1.00
// Date         2023.09.04
// Copyright    ジャパンマイコンカーラリー実行委員会
// ライセンス   This software is released under the MIT License.
//              http://opensource.org/licenses/mit-license.php
//**************************************************************************

//======================================
// インクルード
//======================================
#include "mcr_gpt_lib.h"

//**********************************************************************
// GPT　端子選択
//**********************************************************************
void setGPTterminal( uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用
}

//**********************************************************************
// GPT　PWM出力開始
//**********************************************************************
void startPWM_GPT0( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT0->GTCR_b.MD = 0b000;  // GPT0 のこぎり波PWMモード
    R_GPT0->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT0->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT0->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT0->GTUDDTYC_b.UD = 1;
    R_GPT0->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT0->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT0->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT0->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT0->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT0->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT0->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT0->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT0->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT0->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT0->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT0->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT0->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT0->GTCR_b.CST = 1;     // カウント動作を実行
}

void startPWM_GPT1( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT1->GTCR_b.MD = 0b000;  // GPT1 のこぎり波PWMモード
    R_GPT1->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT1->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT1->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT1->GTUDDTYC_b.UD = 1;
    R_GPT1->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT1->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT1->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT1->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT1->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT1->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT1->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT1->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT1->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT1->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT1->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT1->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT1->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT1->GTCR_b.CST = 1;     // カウント動作を実行
}

void startPWM_GPT2( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT2->GTCR_b.MD = 0b000;  // GPT2 のこぎり波PWMモード
    R_GPT2->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT2->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT2->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT2->GTUDDTYC_b.UD = 1;
    R_GPT2->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT2->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT2->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT2->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT2->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT2->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT2->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT2->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT2->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT2->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT2->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT2->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT2->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT2->GTCR_b.CST = 1;     // カウント動作を実行
}

void startPWM_GPT3( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT3->GTCR_b.MD = 0b000;  // GPT3 のこぎり波PWMモード
    R_GPT3->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT3->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT3->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT3->GTUDDTYC_b.UD = 1;
    R_GPT3->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT3->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT3->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT3->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT3->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT3->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT3->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT3->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT3->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT3->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT3->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT3->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT3->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT3->GTCR_b.CST = 1;     // カウント動作を実行
}

void startPWM_GPT4( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT4->GTCR_b.MD = 0b000;  // GPT4 のこぎり波PWMモード
    R_GPT4->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT4->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT4->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT4->GTUDDTYC_b.UD = 1;
    R_GPT4->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT4->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT4->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT4->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT4->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT4->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT4->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT4->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT4->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT4->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT4->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT4->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT4->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT4->GTCR_b.CST = 1;     // カウント動作を実行
}

void startPWM_GPT5( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT5->GTCR_b.MD = 0b000;  // GPT5 のこぎり波PWMモード
    R_GPT5->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT5->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT5->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT5->GTUDDTYC_b.UD = 1;
    R_GPT5->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT5->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT5->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT5->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT5->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT5->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT5->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT5->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT5->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT5->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT5->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT5->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT5->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT5->GTCR_b.CST = 1;     // カウント動作を実行
}

void startPWM_GPT6( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT6->GTCR_b.MD = 0b000;  // GPT6 のこぎり波PWMモード
    R_GPT6->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT6->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT6->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT6->GTUDDTYC_b.UD = 1;
    R_GPT6->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT6->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT6->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT6->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT6->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT6->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT6->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT6->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT6->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT6->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT6->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT6->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT6->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT6->GTCR_b.CST = 1;     // カウント動作を実行
}

void startPWM_GPT7( uint8_t ch_ab, uint8_t div, uint16_t syuuki ) {

    R_GPT7->GTCR_b.MD = 0b000;  // GPT7 のこぎり波PWMモード
    R_GPT7->GTUDDTYC_b.UDF = 1; // カウント方向強制設定 強制設定する
    R_GPT7->GTUDDTYC_b.UD = 1;  // カウント方向設定 1:GTCNTカウンタはアップカウント
    R_GPT7->GTUDDTYC_b.UDF = 0; // カウント方向強制設定 強制設定しない
    R_GPT7->GTUDDTYC_b.UD = 1;
    R_GPT7->GTCR_b.TPCS = div;    // カウントクロック選択 000:1 001:1/4 010:1/16 011:1/64 100:1/256 101:1/1024
    R_GPT7->GTPR_b.GTPR = syuuki; // 周期設定
    R_GPT7->GTCNT_b.GTCNT = 0;    // カウンタクリア
    if ( ( ch_ab & GTIOCA ) != 0x00 ) {
        R_GPT7->GTIOR_b.GTIOA = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT7->GTIOR_b.OAE = 1;         // GTIOCA端子出力許可 1:出力を許可
        R_GPT7->GTBER_b.CCRA = 0b01;     // GTCCRAバッファ動作 シングルバッファ動作（GTCCRAレジスタ⇔GTCCRCレジスタ）
    }
    if ( ( ch_ab & GTIOCB ) != 0x00 ) {
        R_GPT7->GTIOR_b.GTIOB = 0b01001; // 周期の終わりでHigh出力 GTCCRA/GTCCRBコンペアマッチでLow出力
        R_GPT7->GTIOR_b.OBE = 1;         // GTIOCB端子出力許可 1:出力を許可
        R_GPT7->GTBER_b.CCRB = 0b01;     // GTCCRBバッファ動作 シングルバッファ動作（GTCCRBレジスタ⇔GTCCREレジスタ）
    }
    R_GPT7->GTCCR[GTCCR_A] = 0; // GTIOC1A コンペアマッチ値設定
    R_GPT7->GTCCR[GTCCR_C] = 0; // GTIOC1A デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT7->GTCCR[GTCCR_B] = 0; // GTIOC1B コンペアマッチ値設定
    R_GPT7->GTCCR[GTCCR_E] = 0; // GTIOC1B デューティ比設定 (GTPR+1)=100%  0=ほぼ0%(1パルス330ns分だけ1になる)
    R_GPT7->GTCR_b.CST = 1;     // カウント動作を実行
}

//**********************************************************************
// GPT　１相ロータリエンコーダ
//**********************************************************************
void startGPT0_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT0->GTUDDTYC_b.UDF = 1;
    R_GPT0->GTUDDTYC_b.UD = 1;
    R_GPT0->GTUDDTYC_b.UDF = 0;
    R_GPT0->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT0->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT0->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT0->GTCNT_b.GTCNT = 0;
    R_GPT0->GTCR_b.CST = 1;
}

void startGPT1_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT1->GTUDDTYC_b.UDF = 1;
    R_GPT1->GTUDDTYC_b.UD = 1;
    R_GPT1->GTUDDTYC_b.UDF = 0;
    R_GPT1->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT1->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT1->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT1->GTCNT_b.GTCNT = 0;
    R_GPT1->GTCR_b.CST = 1;
}

void startGPT2_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT2->GTUDDTYC_b.UDF = 1;
    R_GPT2->GTUDDTYC_b.UD = 1;
    R_GPT2->GTUDDTYC_b.UDF = 0;
    R_GPT2->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT2->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT2->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT2->GTCNT_b.GTCNT = 0;
    R_GPT2->GTCR_b.CST = 1;
}

void startGPT3_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT3->GTUDDTYC_b.UDF = 1;
    R_GPT3->GTUDDTYC_b.UD = 1;
    R_GPT3->GTUDDTYC_b.UDF = 0;
    R_GPT3->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT3->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT3->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT3->GTCNT_b.GTCNT = 0;
    R_GPT3->GTCR_b.CST = 1;
}

void startGPT4_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT4->GTUDDTYC_b.UDF = 1;
    R_GPT4->GTUDDTYC_b.UD = 1;
    R_GPT4->GTUDDTYC_b.UDF = 0;
    R_GPT4->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT4->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT4->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT4->GTCNT_b.GTCNT = 0;
    R_GPT4->GTCR_b.CST = 1;
}

void startGPT5_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT5->GTUDDTYC_b.UDF = 1;
    R_GPT5->GTUDDTYC_b.UD = 1;
    R_GPT5->GTUDDTYC_b.UDF = 0;
    R_GPT5->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT5->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT5->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT5->GTCNT_b.GTCNT = 0;
    R_GPT5->GTCR_b.CST = 1;
}

void startGPT6_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT6->GTUDDTYC_b.UDF = 1;
    R_GPT6->GTUDDTYC_b.UD = 1;
    R_GPT6->GTUDDTYC_b.UDF = 0;
    R_GPT6->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT6->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT6->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT6->GTCNT_b.GTCNT = 0;
    R_GPT6->GTCR_b.CST = 1;
}

void startGPT7_1SouEncoder( uint8_t ch_ab, uint8_t port1, uint8_t port2 ) {
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PSEL = 0b00010; // 1相エンコーダは、GTETRGA端子またはGTETRGB端子を使う
    R_PFS->PORT[port1].PIN[port2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT7->GTUDDTYC_b.UDF = 1;
    R_GPT7->GTUDDTYC_b.UD = 1;
    R_GPT7->GTUDDTYC_b.UDF = 0;
    R_GPT7->GTUDDTYC_b.UD = 1;
    if ( ( ch_ab & GTETRGA ) != 0x00 ) {
        R_GPT7->GTUPSR = 0x00000003; // GTETRGA端子　立上がり、立下がりエッジでカウント
    } else if ( ( ch_ab & GTETRGB ) != 0x00 ) {
        R_GPT7->GTUPSR = 0x0000000c; // GTETRGB端子　立上がり、立下がりエッジでカウント
    }
    R_GPT7->GTCNT_b.GTCNT = 0;
    R_GPT7->GTCR_b.CST = 1;
}

//**********************************************************************
// GPT　２相ロータリエンコーダ
//**********************************************************************
void startGPT0_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT0->GTUDDTYC_b.UDF = 1;
    R_GPT0->GTUDDTYC_b.UD = 1;
    R_GPT0->GTUDDTYC_b.UDF = 0;
    R_GPT0->GTUDDTYC_b.UD = 1;
    R_GPT0->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT0->GTDNSR = 0x00009600;
    R_GPT0->GTCNT_b.GTCNT = 0;
    R_GPT0->GTCR_b.CST = 1;
}

void startGPT1_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT1->GTUDDTYC_b.UDF = 1;
    R_GPT1->GTUDDTYC_b.UD = 1;
    R_GPT1->GTUDDTYC_b.UDF = 0;
    R_GPT1->GTUDDTYC_b.UD = 1;
    R_GPT1->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT1->GTDNSR = 0x00009600;
    R_GPT1->GTCNT_b.GTCNT = 0;
    R_GPT1->GTCR_b.CST = 1;
}

void startGPT2_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT2->GTUDDTYC_b.UDF = 1;
    R_GPT2->GTUDDTYC_b.UD = 1;
    R_GPT2->GTUDDTYC_b.UDF = 0;
    R_GPT2->GTUDDTYC_b.UD = 1;
    R_GPT2->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT2->GTDNSR = 0x00009600;
    R_GPT2->GTCNT_b.GTCNT = 0;
    R_GPT2->GTCR_b.CST = 1;
}

void startGPT3_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT3->GTUDDTYC_b.UDF = 1;
    R_GPT3->GTUDDTYC_b.UD = 1;
    R_GPT3->GTUDDTYC_b.UDF = 0;
    R_GPT3->GTUDDTYC_b.UD = 1;
    R_GPT3->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT3->GTDNSR = 0x00009600;
    R_GPT3->GTCNT_b.GTCNT = 0;
    R_GPT3->GTCR_b.CST = 1;
}

void startGPT4_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT4->GTUDDTYC_b.UDF = 1;
    R_GPT4->GTUDDTYC_b.UD = 1;
    R_GPT4->GTUDDTYC_b.UDF = 0;
    R_GPT4->GTUDDTYC_b.UD = 1;
    R_GPT4->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT4->GTDNSR = 0x00009600;
    R_GPT4->GTCNT_b.GTCNT = 0;
    R_GPT4->GTCR_b.CST = 1;
}

void startGPT5_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT5->GTUDDTYC_b.UDF = 1;
    R_GPT5->GTUDDTYC_b.UD = 1;
    R_GPT5->GTUDDTYC_b.UDF = 0;
    R_GPT5->GTUDDTYC_b.UD = 1;
    R_GPT5->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT5->GTDNSR = 0x00009600;
    R_GPT5->GTCNT_b.GTCNT = 0;
    R_GPT5->GTCR_b.CST = 1;
}

void startGPT6_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT6->GTUDDTYC_b.UDF = 1;
    R_GPT6->GTUDDTYC_b.UD = 1;
    R_GPT6->GTUDDTYC_b.UDF = 0;
    R_GPT6->GTUDDTYC_b.UD = 1;
    R_GPT6->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT6->GTDNSR = 0x00009600;
    R_GPT6->GTCNT_b.GTCNT = 0;
    R_GPT6->GTCR_b.CST = 1;
}

void startGPT7_2SouEncoder( uint8_t port1_1, uint8_t port2_1, uint8_t port1_2, uint8_t port2_2 ) {
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_1].PIN[port2_1].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PSEL = 0b00011; // GTIOCxx
    R_PFS->PORT[port1_2].PIN[port2_2].PmnPFS_b.PMR = 1;        // 0:汎用入出力 1:周辺機能用

    R_GPT7->GTUDDTYC_b.UDF = 1;
    R_GPT7->GTUDDTYC_b.UD = 1;
    R_GPT7->GTUDDTYC_b.UDF = 0;
    R_GPT7->GTUDDTYC_b.UD = 1;
    R_GPT7->GTUPSR = 0x00006900; // 2相の場合　 位相計数モード１設定
    R_GPT7->GTDNSR = 0x00009600;
    R_GPT7->GTCNT_b.GTCNT = 0;
    R_GPT7->GTCR_b.CST = 1;
}
