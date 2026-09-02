// GPIO42 の単音ブザーを LEDC の PWM で鳴らす。
// 再生は専用の FreeRTOS タスクで行うので、buzzerPlay() は即座に戻る。
#pragma once

#include <Arduino.h>

struct Note {
    uint16_t freq_hz;  // 0 は休符
    uint16_t ms;
};

void buzzerBegin();

// メロディを非同期で鳴らす。再生中に呼ぶと新しいメロディで置き換わる。
void buzzerPlay(const Note* notes, size_t count);

template <size_t N>
inline void buzzerPlay(const Note (&notes)[N])
{
    buzzerPlay(notes, N);
}

bool buzzerIsPlaying();
void buzzerStop();

// 起動時のテスト音 / LoRa 受信時の効果音
extern const Note BOOT_BEEP[3];
extern const Note LEVEL_UP_MELODY[10];
extern const Note SEND_BEEP[2];
