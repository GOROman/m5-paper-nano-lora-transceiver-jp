// M5Paper Mono (C153) の LoRa (SX1262) を受信専用で扱う
#pragma once

#include <Arduino.h>

struct LoraPacket {
    String text;
    float rssi_dbm;
    float snr_db;
};

struct LoraDiagnostics {
    uint32_t irq_count;
    uint32_t read_ok_count;
    uint32_t read_error_count;
    uint32_t receive_start_count;
    uint32_t receive_start_errors;
    bool irq_pending;
    bool ready;
    int16_t last_error;
};

// LoRa の電源投入・リセット・SX1262 の初期化を行い、受信待ちに入る
bool loraBegin();

// 初期化に失敗したときの RadioLib のエラーコード
int16_t loraLastError();

// IRQ未到達とRadioLibの読出し失敗を切り分ける受信統計。
LoraDiagnostics loraDiagnostics();

// 受信していたら true を返し packet に内容を入れる。loop から呼ぶ
bool loraPoll(LoraPacket& packet);

// text を送信する。送信後は自動で受信待ちに戻る
bool loraSend(const String& text);
