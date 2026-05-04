#include <Arduino.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// ----- LoRa pins (Heltec V3/V4 identical) -----
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10

// ----- OLED pins -----
#define OLED_SCL   18
#define OLED_SDA   17
#define OLED_RST   21

// ----- GPS UART pins -----
#define GPS_RX_PIN 39   
#define GPS_TX_PIN 38   
#define GPS_VCTRL_PIN 34
#define GPS_RST_PIN 42
#define GPS_WAKE_PIN 40
#define GPS_BAUD   9600

// ----- LoRa config — must match V3 receiver exactly -----
#define FREQUENCY     915.0   // MHz  (change to 868.0 for EU)
#define BANDWIDTH     125.0   // kHz
#define SPREAD_FACTOR 12
#define CODING_RATE   5
#define SYNC_WORD     0x12
#define TX_POWER      20      // dBm
#define PREAMBLE_LEN  8

// ----- Transmit interval -----
#define TX_INTERVAL_MS 1000   // send a ping every 1 second

// ----- Packet struct — must match V3 receiver exactly -----
struct GpsPacket {
    float lat;
    float lon;
    float alt;   // meters above sea level; 0.0 if unavailable
};

// ----- Peripherals -----
SPIClass spi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);   // UART1

// ----- State -----
unsigned long lastTx = 0;

void displayStatus(const char* line1, const char* line2 = "", const char* line3 = "") {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    display.setCursor(0, 10); display.print("GPS Transmitter (V4)");
    display.setCursor(0, 26); display.print(line1);
    display.setCursor(0, 40); display.print(line2);
    display.setCursor(0, 54); display.print(line3);
    display.sendBuffer();
}

void setup() {
    Serial.begin(115200);

    // OLED
    display.begin();
    displayStatus("Initializing...");

    // Wake/reset GNSS module
    pinMode(GPS_WAKE_PIN, OUTPUT);
    digitalWrite(GPS_WAKE_PIN, HIGH);

    pinMode(GPS_VCTRL_PIN, OUTPUT);
    digitalWrite(GPS_VCTRL_PIN, LOW);   // LOW enables power on P-channel MOSFET
    delay(100);


    pinMode(GPS_RST_PIN, OUTPUT);
    digitalWrite(GPS_RST_PIN, HIGH);
    delay(100);
    digitalWrite(GPS_RST_PIN, LOW);
    delay(200);
    digitalWrite(GPS_RST_PIN, HIGH);
    delay(1500);

    // GPS serial
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS serial started");

    // SPI + Radio
    spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

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
        displayStatus("Radio FAIL!", "Check wiring");
        while (true);
    }

    // Required on Heltec V3/V4 — DIO2 controls antenna switch
    radio.setDio2AsRfSwitch(true);

    Serial.println("Radio ready. Waiting for GPS fix...");
    displayStatus("Radio OK", "Waiting for GPS fix...");
}

void loop() {
    // Feed GPS parser
    while (gpsSerial.available()) 
    {
        uint8_t c = gpsSerial.read();
        gps.encode(c);
        Serial.write(c);
    }


    // Transmit on interval, only when we have a valid fix
    unsigned long now = millis();
    if (now - lastTx < TX_INTERVAL_MS) return;
    lastTx = now;


    if (!gps.location.isValid()) {
        Serial.printf(
            "GPS chars=%lu sats=%d valid=%d failed=%lu available=%d\n",
            gps.charsProcessed(),
            gps.satellites.value(),
            gps.location.isValid(),
            gps.failedChecksum(),
            gpsSerial.available()
        );
        Serial.println("No GPS fix yet...");
        char buf[32];
        snprintf(buf, sizeof(buf), "Sats: %d", gps.satellites.value());
        displayStatus("No GPS fix", buf, "Move outdoors");
        return;
    }

    GpsPacket pkt;
    pkt.lat = (float)gps.location.lat();
    pkt.lon = (float)gps.location.lng();
    pkt.alt = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;

    Serial.printf("Sending: Lat=%.6f Lon=%.6f Alt=%.1fm\n", pkt.lat, pkt.lon, pkt.alt);

    int state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Packet sent OK");
        char lat_buf[20], lon_buf[20], alt_buf[20];
        snprintf(lat_buf, sizeof(lat_buf),  "Lat: %.6f",  pkt.lat);
        snprintf(lon_buf, sizeof(lon_buf),  "Lon: %.6f", pkt.lon);
        snprintf(alt_buf, sizeof(alt_buf),  "Alt: %.1fm",  pkt.alt);
        displayStatus(lat_buf, lon_buf, alt_buf);
    } else {
        Serial.printf("Transmit failed: %d\n", state);
        displayStatus("TX Failed!", "Check antenna");
    }
}