#include <Arduino.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>


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

// ----- WiFi -----
const char* AP_SSID = "LoRa-Receiver";
const char* AP_PASS = "tracker123";

WebServer server(80);

// ----- State -----
volatile bool rxFlag = false;

void IRAM_ATTR onReceive() {
    rxFlag = true;
}

float latestLat = 0.0;
float latestLon = 0.0;
float latestAlt = 0.0;
int latestRssi = 0;
float latestSnr = 0.0;
bool hasGpsPacket = false;
unsigned long latestPacketTime = 0;

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>LoRa GPS Receiver</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 40px;
      background: #111;
      color: #eee;
    }
    .card {
      background: #222;
      padding: 24px;
      border-radius: 12px;
      max-width: 500px;
    }
    .value {
      font-size: 24px;
      margin: 8px 0;
    }
    .label {
      color: #aaa;
    }
  </style>
</head>
<body>
  <div class="card">
    <h1>LoRa GPS Receiver</h1>
    <div class="value"><span class="label">Status:</span> <span id="status">Loading...</span></div>
    <div class="value"><span class="label">Lat:</span> <span id="lat">--</span></div>
    <div class="value"><span class="label">Lon:</span> <span id="lon">--</span></div>
    <div class="value"><span class="label">Alt:</span> <span id="alt">--</span></div>
    <div class="value"><span class="label">RSSI:</span> <span id="rssi">--</span></div>
    <div class="value"><span class="label">SNR:</span> <span id="snr">--</span></div>
    <div class="value"><span class="label">Age:</span> <span id="age">--</span></div>
  </div>

  <script>
    async function updateGps() {
      try {
        const res = await fetch('/gps');
        const data = await res.json();

        document.getElementById('status').textContent = data.hasFix ? 'Receiving' : 'Waiting';
        document.getElementById('lat').textContent = data.lat.toFixed(6);
        document.getElementById('lon').textContent = data.lon.toFixed(6);
        document.getElementById('alt').textContent = data.alt.toFixed(1) + ' m';
        document.getElementById('rssi').textContent = data.rssi + ' dBm';
        document.getElementById('snr').textContent = data.snr.toFixed(1) + ' dB';
        document.getElementById('age').textContent = data.ageMs + ' ms';
      } catch (e) {
        document.getElementById('status').textContent = 'Disconnected';
      }
    }

    setInterval(updateGps, 1000);
    updateGps();
  </script>
</body>
</html>
)rawliteral";

    server.send(200, "text/html", html);
}

void handleGpsJson() {
    unsigned long ageMs = hasGpsPacket ? millis() - latestPacketTime : 0;

    String json = "{";
    json += "\"hasFix\":" + String(hasGpsPacket ? "true" : "false") + ",";
    json += "\"lat\":" + String(latestLat, 6) + ",";
    json += "\"lon\":" + String(latestLon, 6) + ",";
    json += "\"alt\":" + String(latestAlt, 1) + ",";
    json += "\"rssi\":" + String(latestRssi) + ",";
    json += "\"snr\":" + String(latestSnr, 1) + ",";
    json += "\"ageMs\":" + String(ageMs);
    json += "}";

    server.send(200, "application/json", json);
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

    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("Access point started. Connect to: ");
    Serial.println(AP_SSID);
    Serial.print("Then open: http://");
    Serial.println(WiFi.softAPIP());  // will print 192.168.4.1

    
    Serial.println();
    Serial.print("WiFi connected. Open: http://");
    Serial.println(WiFi.localIP());

    Serial.println();
    Serial.print("WiFi connected. Open: http://");
    Serial.println(WiFi.localIP());

    server.on("/", handleRoot);
    server.on("/gps", handleGpsJson);
    server.begin();
    Serial.println("Web server started");

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
    server.handleClient();
    
    if (!rxFlag) return;
    rxFlag = false;

    GpsPacket pkt;
    int state = radio.readData((uint8_t*)&pkt, sizeof(pkt));

    if (state == RADIOLIB_ERR_NONE) 
    {
        int rssi  = radio.getRSSI();
        float snr = radio.getSNR();

        latestLat = pkt.lat;
        latestLon = pkt.lon;
        latestAlt = pkt.alt;
        latestRssi = rssi;
        latestSnr = snr;
        latestPacketTime = millis();
        hasGpsPacket = true;

        Serial.printf("Lat: %.6f  Lon: %.6f  Alt: %.1fm  RSSI: %ddBm  SNR: %.2fdB\n",
                    pkt.lat, pkt.lon, pkt.alt, rssi, snr);

        displayCoords(pkt.lat, pkt.lon, pkt.alt, rssi, snr);
    }
}