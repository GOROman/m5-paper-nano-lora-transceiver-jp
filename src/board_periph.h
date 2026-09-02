// M5Paper Mono の電源管理 IC (M5PM1) と I/O エキスパンダ (M5IOE1) をまとめて扱う。
// LoRa の電源も RGB LED もこの 2 つの先にぶら下がっているので、
// インスタンスを 1 か所に集約して共有する。
#pragma once

#include <M5IOE1.h>
#include <M5PM1.h>

bool periphBegin();

M5PM1& periphPm1();
M5IOE1& periphIoe1();

// RGB LED。赤は M5PM1 の LED_EN (点灯/消灯のみ)、
// 緑と青は M5IOE1 の PWM (0〜100%) で明るさを変えられる。
bool ledSet(bool red_on, uint8_t green_percent, uint8_t blue_percent);
void ledOff();
