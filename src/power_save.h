// 消費電力を下げるための処理をまとめる
#pragma once

#include <Arduino.h>

// CPU クロックを落とすなど、起動時に一度だけ行う設定
void powerSaveBegin();

// 何もすることが無いときに CPU をライトスリープさせる。
// LoRa の DIO1 / タッチ INT / ボタン / タイマーで復帰する。
// USB 給電中は復帰時にシリアルが切れて不便なので、スリープせず delay で待つ。
void powerSaveIdle();
