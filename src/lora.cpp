#include "lora.h"

#include "board_periph.h"

#include <M5Unified.h>
#include <RadioLib.h>
#include <SPI.h>

/* -------------------------------------------------------------------------- */
/*                                   配線                                      */
/* -------------------------------------------------------------------------- */
// 公式デモ (M5PaperMono-UserDemo main/hal/hal_lora.cpp) と同じ配線。
// LoRa の電源とリセットは ESP32 の GPIO ではなく、
// 電源管理 IC (M5PM1) と I/O エキスパンダ (M5IOE1) の先にぶら下がっている。
static constexpr int LORA_SPI_SCK  = 39;
static constexpr int LORA_SPI_MISO = 40;
static constexpr int LORA_SPI_MOSI = 38;
static constexpr int LORA_NSS      = 41;
static constexpr int LORA_DIO1     = 5;
static constexpr int LORA_BUSY     = 21;

static constexpr m5pm1_gpio_num_t LORA_EN_PM1_PIN = M5PM1_GPIO_NUM_2;   // LoRa 電源
static constexpr uint8_t LORA_RST_IOE1_PIN        = M5IOE1_PIN_10;      // LoRa リセット
static constexpr uint8_t LORA_ANT_SW_IOE1_PIN     = M5IOE1_PIN_2;       // アンテナ切替

/* -------------------------------------------------------------------------- */
/*                              通信パラメータ                                 */
/* -------------------------------------------------------------------------- */
// 日本の920MHz帯向けの保守的な設定。
// 送信側 (Cardputer ADV + Cap LoRa-1262) と完全に同じ値にすること。
// 1つでも違うとパケットは復調できない。
// 注意: 周波数設定だけでは技適を満たさない。モジュール、アンテナ、出力、
// 占有帯域幅などを含む構成全体が日本の技術基準に適合している必要がある。
static constexpr float LORA_FREQ_MHZ       = 920.6f;
static constexpr float LORA_BW_KHZ         = 125.0f;
static constexpr uint8_t LORA_SF           = 7;
static constexpr uint8_t LORA_CR           = 5;
static constexpr uint8_t LORA_SYNC_WORD    = 0x34;
static constexpr int8_t LORA_TX_POWER_DBM  = 13;
static constexpr uint16_t LORA_PREAMBLE    = 10;
static constexpr float LORA_TCXO_V         = 3.0f;
static constexpr bool LORA_REGULATOR_LDO   = true;
static constexpr float LORA_CURRENT_LIMIT_MA = 140.0f;
static constexpr uint32_t LORA_SPI_FREQ_HZ = 8000000;
static constexpr size_t LORA_MAX_PACKET    = 255;
// NOTE: startReceiveDutyCycleAuto() は「Interrupt wdt timeout (ISR context)」で
// 実機がリブートする不具合が確認されたため、現状は無効化して連続受信にしている。
// 省電力化はいったん見送り、安定動作を優先する。
static constexpr bool LORA_RX_DUTY_CYCLE = false;

static SPIClass s_lora_spi(HSPI);

static SX1262* s_radio      = nullptr;
static Module* s_module     = nullptr;
static int16_t s_last_error = RADIOLIB_ERR_UNKNOWN;
static bool s_ready         = false;

// DIO1 割り込みから呼ばれるので、ここでは受信フラグを立てるだけにする
static volatile bool s_rx_flag = false;
static volatile uint32_t s_irq_count = 0;
static volatile uint32_t s_read_ok_count = 0;
static volatile uint32_t s_read_error_count = 0;
static volatile uint32_t s_receive_start_count = 0;
static volatile uint32_t s_receive_start_errors = 0;
static void ICACHE_RAM_ATTR onLoraPacketReceived()
{
    s_rx_flag = true;
    ++s_irq_count;
}

// 受信待ちに入る。省電力設定に応じて連続受信とデューティサイクル受信を切り替える
static int16_t startListening()
{
    if (!s_radio) {
        return RADIOLIB_ERR_UNKNOWN;
    }
    ++s_receive_start_count;
    const int16_t state = LORA_RX_DUTY_CYCLE ? s_radio->startReceiveDutyCycleAuto() : s_radio->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        ++s_receive_start_errors;
    }
    return state;
}

static bool powerUpLoraModule()
{
    if (!periphBegin()) {
        return false;
    }
    M5PM1& s_pm1   = periphPm1();
    M5IOE1& s_ioe1 = periphIoe1();

    // LoRa モジュールへの給電
    s_pm1.gpioSetFunc(LORA_EN_PM1_PIN, M5PM1_GPIO_FUNC_GPIO);
    if (s_pm1.gpioSet(LORA_EN_PM1_PIN, M5PM1_GPIO_MODE_OUTPUT, 1, M5PM1_GPIO_PULL_NONE, M5PM1_GPIO_DRIVE_PUSHPULL) !=
        M5PM1_OK) {
        Serial.println("[LoRa] power enable failed");
        return false;
    }

    // アンテナスイッチを LoRa 側へ、そのあとリセットを解除する
    s_ioe1.pinMode(LORA_ANT_SW_IOE1_PIN, OUTPUT);
    s_ioe1.pinMode(LORA_RST_IOE1_PIN, OUTPUT);
    s_ioe1.setDriveMode(LORA_ANT_SW_IOE1_PIN, M5IOE1_DRIVE_PUSHPULL);
    s_ioe1.setDriveMode(LORA_RST_IOE1_PIN, M5IOE1_DRIVE_PUSHPULL);
    s_ioe1.digitalWrite(LORA_ANT_SW_IOE1_PIN, HIGH);
    s_ioe1.digitalWrite(LORA_RST_IOE1_PIN, LOW);
    delay(100);
    s_ioe1.digitalWrite(LORA_RST_IOE1_PIN, HIGH);
    delay(20);
    return true;
}

bool loraBegin()
{
    if (s_ready) {
        return true;
    }
    if (!powerUpLoraModule()) {
        s_last_error = RADIOLIB_ERR_CHIP_NOT_FOUND;
        return false;
    }

    s_lora_spi.begin(LORA_SPI_SCK, LORA_SPI_MISO, LORA_SPI_MOSI, LORA_NSS);

    // リセットは M5IOE1 側で済ませているので RADIOLIB_NC を渡す
    s_module = new Module(LORA_NSS, LORA_DIO1, RADIOLIB_NC, LORA_BUSY, s_lora_spi,
                          SPISettings(LORA_SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
    s_radio  = new SX1262(s_module);

    s_last_error = s_radio->begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER_DBM,
                                  LORA_PREAMBLE, LORA_TCXO_V, LORA_REGULATOR_LDO);
    if (s_last_error != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] begin failed: %d\n", s_last_error);
        return false;
    }

    s_radio->setDio2AsRfSwitch(true);
    s_radio->setCurrentLimit(LORA_CURRENT_LIMIT_MA);
    s_radio->setPacketReceivedAction(onLoraPacketReceived);

    s_last_error = startListening();
    if (s_last_error != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] startReceive failed: %d\n", s_last_error);
        return false;
    }

    s_ready = true;
    Serial.printf("[LoRa] ready: %.1fMHz BW%.0f SF%u CR%u sync 0x%02X\n", LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                  LORA_SYNC_WORD);
    return true;
}

int16_t loraLastError()
{
    return s_last_error;
}

LoraDiagnostics loraDiagnostics()
{
    LoraDiagnostics d{};
    d.irq_count = s_irq_count;
    d.read_ok_count = s_read_ok_count;
    d.read_error_count = s_read_error_count;
    d.receive_start_count = s_receive_start_count;
    d.receive_start_errors = s_receive_start_errors;
    d.irq_pending = s_rx_flag;
    d.ready = s_ready;
    d.last_error = s_last_error;
    return d;
}

bool loraPoll(LoraPacket& packet)
{
    if (!s_ready || !s_rx_flag) {
        return false;
    }
    s_rx_flag = false;

    uint8_t buf[LORA_MAX_PACKET + 1] = {0};
    const int16_t state              = s_radio->readData(buf, LORA_MAX_PACKET);

    // 読み出したら必ず受信待ちへ戻す
    startListening();

    if (state != RADIOLIB_ERR_NONE) {
        ++s_read_error_count;
        if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.printf("[LoRa] CRC ERROR: readData failed (%d)\n", state);
        } else {
            Serial.printf("[LoRa] readData failed: %d\n", state);
        }
        s_last_error = state;
        return false;
    }

    ++s_read_ok_count;

    packet.text     = String(reinterpret_cast<const char*>(buf));
    packet.rssi_dbm = s_radio->getRSSI();
    packet.snr_db   = s_radio->getSNR();
    return true;
}

bool loraSend(const String& text)
{
    if (!s_ready || text.isEmpty() || text.length() > LORA_MAX_PACKET) {
        return false;
    }

    // SF7 / BW125 なら短いパケットは数十ミリ秒で送り終わるので、
    // ここは素直にブロッキング送信でよい
    const int16_t state = s_radio->transmit(reinterpret_cast<const uint8_t*>(text.c_str()), text.length());
    s_last_error        = state;

    // 送ったら必ず受信待ちへ戻す
    s_rx_flag = false;
    startListening();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] transmit failed: %d\n", state);
        return false;
    }
    Serial.printf("[LoRa] tx: \"%s\"\n", text.c_str());
    return true;
}
