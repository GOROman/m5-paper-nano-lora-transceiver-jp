#include "buzzer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static constexpr int BUZZER_PIN          = 42;
static constexpr int BUZZER_LEDC_FREQ_HZ = 2000;
static constexpr int BUZZER_LEDC_BITS    = 10;
static constexpr int BUZZER_LEDC_CH      = 7;
// 10bit 分解能での 50% デューティ。矩形波はこれが最大音量になる。
static constexpr uint32_t BUZZER_DUTY_MAX = 1 << (BUZZER_LEDC_BITS - 1);
// 圧電ブザーは共振周波数(数kHz)付近が一番大きく鳴るので、メロディ全体を持ち上げる。
// 1 = そのまま / 2 = 1オクターブ上 / 4 = 2オクターブ上
static constexpr uint16_t BUZZER_OCTAVE_SHIFT = 2;

static constexpr size_t BUZZER_MAX_NOTES     = 32;
static constexpr uint32_t BUZZER_TASK_STACK  = 2048;
static constexpr UBaseType_t BUZZER_TASK_PRI = 2;

// 音階 (Hz)
static constexpr uint16_t NOTE_C5 = 523, NOTE_E5 = 659, NOTE_G5 = 784;
static constexpr uint16_t NOTE_C6 = 1047, NOTE_E6 = 1319;
static constexpr uint16_t NOTE_G6 = 1568, NOTE_A6 = 1760, NOTE_C7 = 2093;

const Note BOOT_BEEP[3] = {
    {NOTE_C6, 80}, {0, 40}, {NOTE_G6, 120},
};

// レベルアップのファンファーレ (上昇アルペジオ + トップ音伸ばし)
const Note LEVEL_UP_MELODY[10] = {
    {NOTE_C5, 90},  {NOTE_E5, 90},  {NOTE_G5, 90},  {NOTE_C6, 90},
    {NOTE_E6, 90},  {NOTE_G6, 90},  {NOTE_C7, 240}, {0, 60},
    {NOTE_A6, 120}, {NOTE_C7, 420},
};

// タッチ送信の効果音 (受信のファンファーレと区別できる短いピッ)
const Note SEND_BEEP[2] = {
    {NOTE_E6, 40}, {NOTE_C7, 60},
};

// 再生タスクへ渡すメロディ。書き込みは buzzerPlay、読み出しは再生タスクのみ。
static Note s_notes[BUZZER_MAX_NOTES];
static volatile size_t s_note_count = 0;
static volatile bool s_playing      = false;
// 新しいメロディが来たら再生中のものを打ち切るための世代番号
static volatile uint32_t s_generation = 0;
static TaskHandle_t s_task            = nullptr;

static void toneOn(uint32_t freq_hz)
{
    ledcWriteTone(BUZZER_LEDC_CH, freq_hz);
    if (freq_hz) {
        // ledcWriteTone はデューティ 50% を設定するが、音量を最大に保つため明示しておく
        ledcWrite(BUZZER_LEDC_CH, BUZZER_DUTY_MAX);
    }
}

static void toneOff()
{
    ledcWriteTone(BUZZER_LEDC_CH, 0);
    ledcWrite(BUZZER_LEDC_CH, 0);
}

static void buzzerTask(void*)
{
    for (;;) {
        // buzzerPlay() から通知が来るまで眠る
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const uint32_t generation = s_generation;
        const size_t count        = s_note_count;
        s_playing                 = true;

        for (size_t i = 0; i < count; ++i) {
            // 再生中に別のメロディを頼まれたら、こちらは即座に降りる
            if (s_generation != generation) {
                break;
            }
            toneOn(s_notes[i].freq_hz * BUZZER_OCTAVE_SHIFT);
            vTaskDelay(pdMS_TO_TICKS(s_notes[i].ms));
        }

        if (s_generation == generation) {
            toneOff();
            s_playing = false;
        }
    }
}

void buzzerBegin()
{
    ledcSetup(BUZZER_LEDC_CH, BUZZER_LEDC_FREQ_HZ, BUZZER_LEDC_BITS);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    toneOff();

    if (!s_task) {
        xTaskCreate(buzzerTask, "buzzer", BUZZER_TASK_STACK, nullptr, BUZZER_TASK_PRI, &s_task);
    }
}

void buzzerPlay(const Note* notes, size_t count)
{
    if (!s_task || !notes || count == 0) {
        return;
    }
    if (count > BUZZER_MAX_NOTES) {
        count = BUZZER_MAX_NOTES;
    }

    memcpy(s_notes, notes, count * sizeof(Note));
    s_note_count = count;
    ++s_generation;
    xTaskNotifyGive(s_task);
}

bool buzzerIsPlaying()
{
    return s_playing;
}

void buzzerStop()
{
    ++s_generation;
    s_playing = false;
    toneOff();
}
