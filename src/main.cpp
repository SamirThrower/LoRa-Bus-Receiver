#include <Arduino.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>

// ----- Pin definitions for Heltec LoRa 32 V3 -----
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_RST   12
#define LORA_BUSY  13

#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10

#define OLED_SCL   18
#define OLED_SDA   17
#define OLED_RST   21

// ----- LoRa config — must match V4 transmitter exactly -----
#define FREQUENCY     915.0   // MHz  (change to 868.0 for EU)
#define BANDWIDTH     125.0   // kHz
#define SPREAD_FACTOR 7
#define CODING_RATE   5
#define SYNC_WORD     0x12    // private network; use 0x34 for LoRaWAN public
#define TX_POWER      14      // dBm (RX only, but RadioLib needs this for init)
#define PREAMBLE_LEN  8

// ----- OLED -----
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);

// ----- Radio -----
SPIClass spi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi);

// ----- Packet struct — must match V4 transmitter -----
struct GpsPacket {
    float lat;
    float lon;
    float alt;      // meters, optional; set to 0 if not used
};

// ----- State -----
volatile bool rxFlag = false;

void IRAM_ATTR onReceive() {
    rxFlag = true;
}

void displayCoords(float lat, float lon, float alt, int rssi, float snr) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);

    display.setCursor(0, 10);
    display.printf("GPS Receiver (V3)");

    display.setCursor(0, 24);
    display.printf("Lat:  %.6f", lat);

    display.setCursor(0, 36);
    display.printf("Lon: %.6f", lon);

    display.setCursor(0, 48);
    display.printf("Alt:  %.1f m", alt);

    display.setCursor(0, 60);
    display.printf("RSSI:%ddBm SNR:%.1f", rssi, snr);

    display.sendBuffer();
}

void displayStatus(const char* msg) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    display.setCursor(0, 10);
    display.print("GPS Receiver (V3)");
    display.setCursor(0, 30);
    display.print(msg);
    display.sendBuffer();
}

void setup() {
    Serial.begin(115200);

    // OLED
    display.begin();
    displayStatus("Initializing...");

    // SPI
    spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // Radio init
    int state = radio.begin(
        FREQUENCY,
        BANDWIDTH,
        SPREAD_FACTOR,
        CODING_RATE,
        SYNC_WORD,
        TX_POWER,
        PREAMBLE_LEN
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio init failed: %d\n", state);
        displayStatus("Radio FAIL!");
        while (true);
    }

    // DIO2 as RF switch (required on Heltec V3)
    radio.setDio2AsRfSwitch(true);

    // Attach interrupt and start listening
    radio.setDio1Action(onReceive);
    state = radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("startReceive failed: %d\n", state);
        displayStatus("RX start FAIL!");
        while (true);
    }

    Serial.println("Listening for GPS packets...");
    displayStatus("Listening...");
}

void loop() {
    if (!rxFlag) return;
    rxFlag = false;

    GpsPacket pkt;
    int state = radio.readData((uint8_t*)&pkt, sizeof(pkt));

    if (state == RADIOLIB_ERR_NONE) {
        int rssi     = radio.getRSSI();
        float snr    = radio.getSNR();

        Serial.printf("Lat: %.6f  Lon: %.6f  Alt: %.1fm  RSSI: %ddBm  SNR: %.2fdB\n",
                      pkt.lat, pkt.lon, pkt.alt, rssi, snr);

        displayCoords(pkt.lat, pkt.lon, pkt.alt, rssi, snr);

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("CRC error — packet corrupted");
        displayStatus("CRC Error");
    } else {
        Serial.printf("readData failed: %d\n", state);
        displayStatus("RX Error");
    }

    // Re-arm receiver
    radio.startReceive();
}