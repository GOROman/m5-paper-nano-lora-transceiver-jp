#include "power_save.h"

#include <M5Unified.h>

// NOTE: 実機で以下 2 つの不具合が確認されたため、省電力機能は現状すべて無効化している。
//   - esp_light_sleep_start(): USB Serial/JTAG が切れてボタン/タッチが反応しなくなる
//   - setCpuFrequencyMhz(80) を含む初期化: 起動から数秒後に
//     "Interrupt wdt timeout (ISR context)" でリブートするようになった
//     (CPU クロック変更が I2C/USB のタイミングを乱している可能性)
// 安定動作を優先し、電源まわりの見直しは別途行う。

void powerSaveBegin()
{
}

void powerSaveIdle()
{
    delay(10);
}
