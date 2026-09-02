// Step 4: LoRa 受信/送信 + フロントライト調整
//
//   受信  : メッセージ窓だけを部分更新し、SE と LED 点滅で知らせる
//   タッチ: 画面は書き換えず、LED を白フラッシュして今表示中の文字列を送信
//   ボタン: BtnA = 暗く / BtnB = 明るく (フロントライト)
//
// バッテリー動作を伸ばすため、待機中は CPU をライトスリープさせ、
// E-Ink パネルは描画が終わるたびに省電力状態へ落とす。
//
// E-Ink は drawString を直接呼ぶたびにリフレッシュが走ってしまうので、
// 「ヘッダー」と「メッセージ窓」の 2 枚のキャンバスに描いてから
// 必要な窓だけを 1 回 pushSprite する。全面更新より格段に速い。
#include <M5Unified.h>

#include "board_periph.h"
#include "buzzer.h"
#include "lora.h"
#include "power_save.h"

/* -------------------------------------------------------------------------- */
/*                                  レイアウト                                 */
/* -------------------------------------------------------------------------- */
static constexpr int32_t HEADER_H = 150;
static constexpr int32_t MARGIN   = 24;

static M5Canvas canvas_header(&M5.Display);
static M5Canvas canvas_msg(&M5.Display);

static int32_t msg_x = 0, msg_y = 0, msg_w = 0, msg_h = 0;

// 部分更新を何回続けたら quality で焼き直すか
static constexpr uint32_t QUALITY_REFRESH_INTERVAL = 10;

/* -------------------------------------------------------------------------- */
/*                                   状態                                      */
/* -------------------------------------------------------------------------- */
static uint32_t rx_count = 0;
static uint32_t tx_count = 0;
static bool lora_ready   = false;
static LoraPacket last_packet;

// フロントライトの明るさ (0〜10 段階)
static constexpr uint8_t BRIGHTNESS_MAX_LEVEL = 10;
static uint8_t brightness_level               = 0;

// 送信するものが無いときに送る文字列
static constexpr const char* DEFAULT_TX_TEXT = "PING";

// USBシリアルから1行受け取ってLoRa送信するための入力バッファ。
static char serial_tx_buf[256];
static size_t serial_tx_len = 0;

/* -------------------------------------------------------------------------- */
/*                                    描画                                     */
/* -------------------------------------------------------------------------- */
static void drawHeader()
{
    canvas_header.fillSprite(TFT_WHITE);
    canvas_header.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas_header.setTextSize(1);
    canvas_header.setTextDatum(top_center);

    const int32_t cx = canvas_header.width() / 2;

    canvas_header.setFont(&fonts::FreeSansBold24pt7b);
    canvas_header.drawString("LoRa TRANSCEIVER", cx, 12);

    canvas_header.setFont(&fonts::FreeSansBold12pt7b);
    canvas_header.drawString("920.6MHz   BW125   SF7   CR4/5   sync 0x34", cx, 68);

    char line[96];
    if (lora_ready) {
        snprintf(line, sizeof(line), "RX %lu   TX %lu   LIGHT %u/%u   BATT %ld%%", (unsigned long)rx_count,
                 (unsigned long)tx_count, brightness_level, BRIGHTNESS_MAX_LEVEL,
                 (long)M5.Power.getBatteryLevel());
    } else {
        snprintf(line, sizeof(line), "LoRa INIT FAILED (RadioLib %d)", loraLastError());
    }
    canvas_header.drawString(line, cx, 100);

    canvas_header.setFont(&fonts::FreeSansBold9pt7b);
    canvas_header.drawString("TOUCH: send    BTN A/B: brightness", cx, 128);
}

static void drawMessage()
{
    canvas_msg.fillSprite(TFT_WHITE);
    canvas_msg.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas_msg.setTextSize(1);
    canvas_msg.drawRect(0, 0, canvas_msg.width(), canvas_msg.height(), TFT_BLACK);

    const int32_t cx = canvas_msg.width() / 2;

    // RX カウントはヘッダーではなくこの窓に置く。受信時はこの窓だけ書き換えれば済むようにするため
    canvas_msg.setTextDatum(top_center);
    canvas_msg.setFont(&fonts::FreeSansBold12pt7b);
    canvas_msg.drawString("RX " + String(rx_count), cx, 12);

    if (rx_count == 0) {
        canvas_msg.setTextDatum(middle_center);
        canvas_msg.setFont(&fonts::FreeSansBold24pt7b);
        canvas_msg.drawString("waiting...", cx, canvas_msg.height() / 2);
        return;
    }

    // 受信文字列は主役なので、太字を 2 倍に拡大して大きく出す
    canvas_msg.setTextDatum(middle_center);
    canvas_msg.setFont(&fonts::FreeSansBold24pt7b);
    canvas_msg.setTextSize(2);
    canvas_msg.drawString(last_packet.text, cx, canvas_msg.height() / 2 - 10);

    canvas_msg.setTextSize(1);
    canvas_msg.setFont(&fonts::FreeSansBold12pt7b);
    char info[64];
    snprintf(info, sizeof(info), "RSSI %.1f dBm    SNR %.1f dB", last_packet.rssi_dbm, last_packet.snr_db);
    canvas_msg.setTextDatum(bottom_center);
    canvas_msg.drawString(info, cx, canvas_msg.height() - 16);
}

// NOTE: 転送のたびに M5.Display.powerSaveOn()/Off() を呼ぶと、実機で
// waitDisplay() が BUSY ピンから戻らずハングする不具合が確認されたため、
// パネルの powerSave 切り替えは行わずシンプルな転送のみにしている。
static void pushCanvas(M5Canvas& canvas, int32_t x, int32_t y, bool quality)
{
    M5.Display.setEpdMode(quality ? epd_mode_t::epd_quality : epd_mode_t::epd_fast);
    canvas.pushSprite(x, y);
    M5.Display.waitDisplay();
}

static void pushHeader(bool quality)
{
    drawHeader();
    pushCanvas(canvas_header, 0, 0, quality);
}

// 受信のたびに呼ばれる。触るのはメッセージ窓だけなので速い
static void pushMessage(bool quality)
{
    drawMessage();
    pushCanvas(canvas_msg, msg_x, msg_y, quality);
}

/* -------------------------------------------------------------------------- */
/*                            LED (非同期で点滅させる)                          */
/* -------------------------------------------------------------------------- */
static constexpr uint32_t LED_BLINK_STEP_MS = 60;

static uint8_t led_steps_left  = 0;
static uint32_t led_next_ms    = 0;
static bool led_white_flash    = false;

// 受信通知: 緑と青を交互に 3 回ずつ
static void startReceiveBlink()
{
    led_white_flash = false;
    led_steps_left  = 6;
    led_next_ms     = millis();
}

// 送信通知: 白 (赤+緑+青) を 1 回
static void startWhiteFlash()
{
    led_white_flash = true;
    led_steps_left  = 2;
    led_next_ms     = millis();
    ledSet(true, 100, 100);
    buzzerPlay(SEND_BEEP);  // 非同期なのですぐ戻る
}

static void updateLed()
{
    if (led_steps_left == 0 || (int32_t)(millis() - led_next_ms) < 0) {
        return;
    }
    --led_steps_left;
    led_next_ms = millis() + (led_white_flash ? LED_BLINK_STEP_MS * 2 : LED_BLINK_STEP_MS);

    if (led_steps_left == 0) {
        ledOff();
    } else if (led_white_flash) {
        ledSet(true, 100, 100);  // 白
    } else if (led_steps_left % 2) {
        ledSet(false, 100, 0);  // 緑
    } else {
        ledSet(false, 0, 100);  // 青
    }
}

static void pollSerialTx()
{
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value < 0) {
            return;
        }
        if (value == '\r') {
            continue;
        }
        if (value == '\n') {
            serial_tx_buf[serial_tx_len] = '\0';
            if (serial_tx_len > 0) {
                const String text(serial_tx_buf);
                Serial.printf("serial tx: \"%s\"\n", text.c_str());
                startWhiteFlash();
                if (loraSend(text)) {
                    ++tx_count;
                    Serial.println("serial tx: OK");
                } else {
                    Serial.printf("serial tx: FAILED (RadioLib %d)\n", loraLastError());
                }
            }
            serial_tx_len = 0;
            continue;
        }
        if (serial_tx_len < sizeof(serial_tx_buf) - 1) {
            serial_tx_buf[serial_tx_len++] = static_cast<char>(value);
        } else {
            serial_tx_len = 0;
            Serial.println("serial tx: FAILED (message exceeds 255 bytes)");
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                              フロントライト                                 */
/* -------------------------------------------------------------------------- */
static void applyBrightness()
{
    M5.Display.setBrightness(brightness_level * (255 / BRIGHTNESS_MAX_LEVEL));
    Serial.printf("brightness: %u/%u\n", brightness_level, BRIGHTNESS_MAX_LEVEL);
}

static void changeBrightness(int8_t delta)
{
    const int16_t next = constrain(brightness_level + delta, 0, BRIGHTNESS_MAX_LEVEL);
    if (next == brightness_level) {
        return;
    }
    brightness_level = next;
    applyBrightness();

    static const Note UP_TONES[]   = {{1047, 24}, {1319, 24}, {1760, 32}};
    static const Note DOWN_TONES[] = {{1568, 24}, {1175, 24}, {880, 32}};
    buzzerPlay(delta > 0 ? UP_TONES : DOWN_TONES, 3);

    // 明るさの表示はヘッダーにあるので、そこだけ書き換える
    pushHeader(false);
}

/* -------------------------------------------------------------------------- */
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);

    M5.Display.setRotation(1);

    msg_x = MARGIN;
    msg_y = HEADER_H;
    msg_w = M5.Display.width() - MARGIN * 2;
    msg_h = M5.Display.height() - HEADER_H - MARGIN;

    buzzerBegin();
    buzzerPlay(BOOT_BEEP);

    periphBegin();
    lora_ready = loraBegin();

    // 4bpp グレースケール。800x150 + 752x306 で 350KB ほど (PSRAM に載る)
    canvas_header.setColorDepth(4);
    canvas_header.createSprite(M5.Display.width(), HEADER_H);
    canvas_msg.setColorDepth(4);
    canvas_msg.createSprite(msg_w, msg_h);

    applyBrightness();

    // 起動時だけは全面を quality で焼く
    M5.Display.setEpdMode(epd_mode_t::epd_quality);
    M5.Display.fillScreen(TFT_WHITE);
    M5.Display.display();
    pushHeader(true);
    pushMessage(true);

    powerSaveBegin();

    Serial.printf("display: %dx%d / LoRa: %s\n", M5.Display.width(), M5.Display.height(),
                  lora_ready ? "ready" : "failed");
    Serial.println("[LoRa][diag] automatic diagnostics enabled");
    Serial.println("[LoRa][tx] type a line and press Enter to transmit");
}

void loop()
{
    M5.update();
    updateLed();
    pollSerialTx();

    /* -------------------------------- 受信 -------------------------------- */
    LoraPacket packet;
    if (loraPoll(packet)) {
        last_packet = packet;
        ++rx_count;
        Serial.printf("[LoRa] rx #%lu: \"%s\" RSSI %.1f SNR %.1f\n", (unsigned long)rx_count, packet.text.c_str(),
                      packet.rssi_dbm, packet.snr_db);

        // 音と LED は非同期なので、待たずに描画へ進む
        buzzerPlay(LEVEL_UP_MELODY);
        startReceiveBlink();

        // 受信時に書き換えるのはメッセージ窓だけ。ヘッダーには触らない
        pushMessage((rx_count % QUALITY_REFRESH_INTERVAL) == 0);
    }

    /* ------------------- タッチ: 画面は変えずに送信する ------------------- */
    if (M5.Touch.getDetail().wasPressed()) {
        Serial.println("touch: pressed");
        const String text = (rx_count > 0) ? last_packet.text : String(DEFAULT_TX_TEXT);
        startWhiteFlash();
        if (loraSend(text)) {
            ++tx_count;
        }
    }

    /* --------------------- ボタン: フロントライト調整 --------------------- */
    if (M5.BtnA.wasPressed()) {
        Serial.println("BtnA: pressed");
        changeBrightness(-1);
    }
    if (M5.BtnB.wasPressed()) {
        Serial.println("BtnB: pressed");
        changeBrightness(+1);
    }

    // 受信不能時の切り分け用。IRQが増えなければ信号未到達または
    // 周波数/変調条件不一致、IRQだけ増えてread errorなら受信品質/CRCを疑う。
    static uint32_t last_lora_diag = 0;
    if (millis() - last_lora_diag >= 2000) {
        last_lora_diag = millis();
        const LoraDiagnostics d = loraDiagnostics();
        Serial.printf("[LoRa][diag] t=%lus ready=%d irq=%lu read_ok=%lu read_err=%lu "
                      "rx_start=%lu rx_start_err=%lu pending=%d last=%d\n",
                      (unsigned long)(millis() / 1000), d.ready ? 1 : 0,
                      (unsigned long)d.irq_count, (unsigned long)d.read_ok_count,
                      (unsigned long)d.read_error_count, (unsigned long)d.receive_start_count,
                      (unsigned long)d.receive_start_errors, d.irq_pending ? 1 : 0,
                      d.last_error);
    }

    // デバッグ: 生の GPIO レベルを 1 秒ごとに出す (プルアップ/配線の切り分け用)
    static uint32_t last_raw_log = 0;
    if (millis() - last_raw_log > 1000) {
        last_raw_log = millis();
        Serial.printf("raw: BtnA(GPIO2)=%d BtnB(GPIO3)=%d\n", digitalRead(2), digitalRead(3));
    }

    // 待機中は寝る (USB 給電中は寝ずにシリアルを維持する)
    powerSaveIdle();
}
