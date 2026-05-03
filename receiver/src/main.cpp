#include <Arduino.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

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
#define FREQUENCY     915.0
#define BANDWIDTH     125.0
#define SPREAD_FACTOR 7
#define CODING_RATE   5
#define SYNC_WORD     0x12
#define TX_POWER      14
#define PREAMBLE_LEN  8

// ----- Known GPS locations -----
struct NamedLocation {
    const char* name;
    float lat;
    float lon;
};

// Fill in real coordinates for Mountainview and EB.
const NamedLocation LOCATIONS[] = {
    {"Hillside",     42.087, -75.978},
    {"Mountainview", 42.0840,   -75.9703},
    {"Hinman", 42.0881, -75.9727},
    {"Engineering Building", 42.0868, -75.9682},
    {"Dickinson", 42.0873, -75.9648},
    {"Newing", 42.0884, -75.9628},
    {"Cooper", 42.0896, -75.9658},
    {"East Gym", 42.0909, -75.9674},
    {"Welcome Center", 42.0931, -75.9685},
    {"West Gym", 42.0931, -75.9712},
    {"Physical Facilities", 42.0916, -75.9740},
    {"Clearview", 42.0890, -75.9755},
    {"Susquehanna", 42.08625, -75.9744},
};

const int NUM_LOCATIONS = sizeof(LOCATIONS) / sizeof(LOCATIONS[0]);

// Receiver is located at Hillside
const float RECEIVER_LAT = 42.087;
const float RECEIVER_LON = -75.978;

// ----- OLED -----
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(
    U8G2_R0,
    OLED_SCL,
    OLED_SDA,
    OLED_RST
);

// ----- Radio -----
SPIClass spi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi);

// ----- Packet struct — must match V4 transmitter -----
struct GpsPacket {
    float lat;
    float lon;
    float alt;
};

// ----- WiFi -----
const char* AP_SSID = "LoRa-Receiver";
const char* AP_PASS = "tracker123";

WebServer server(80);

// ----- State -----
volatile bool rxFlag = false;

float latestLat = 0.0;
float latestLon = 0.0;
float latestAlt = 0.0;
int latestRssi = 0;
float latestSnr = 0.0;
bool hasGpsPacket = false;
unsigned long latestPacketTime = 0;

// ----- Interrupt -----
void IRAM_ATTR onReceive() {
    rxFlag = true;
}

// ----- Distance helpers -----
float distanceMeters(float lat1, float lon1, float lat2, float lon2) {
    // Simple flat-earth distance formula.
    // This is accurate enough for nearby coordinates.
    const float metersPerDegreeLat = 111320.0;

    float averageLatRad = ((lat1 + lat2) / 2.0) * DEG_TO_RAD;
    float metersPerDegreeLon = 111320.0 * cos(averageLatRad);

    float dLat = (lat2 - lat1) * metersPerDegreeLat;
    float dLon = (lon2 - lon1) * metersPerDegreeLon;

    return sqrt((dLat * dLat) + (dLon * dLon));
}

String findClosestLocation(float txLat, float txLon, float &closestDistanceM) {
    int closestIndex = 0;

    closestDistanceM = distanceMeters(
        txLat,
        txLon,
        LOCATIONS[0].lat,
        LOCATIONS[0].lon
    );

    for (int i = 1; i < NUM_LOCATIONS; i++) {
        float distanceM = distanceMeters(
            txLat,
            txLon,
            LOCATIONS[i].lat,
            LOCATIONS[i].lon
        );

        if (distanceM < closestDistanceM) {
            closestDistanceM = distanceM;
            closestIndex = i;
        }
    }

    return String(LOCATIONS[closestIndex].name);
}

// ----- Website ----- http://192.168.4.1
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
      max-width: 750px;
    }

    .value {
      font-size: 22px;
      margin: 10px 0;
    }

    .label {
      color: #aaa;
    }

    hr {
      border: none;
      border-top: 1px solid #444;
      margin: 20px 0;
    }

    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
      font-size: 18px;
    }

    th, td {
      text-align: left;
      padding: 8px;
      border-bottom: 1px solid #444;
    }

    th {
      color: #aaa;
    }

    .closest {
      color: #7CFC00;
      font-weight: bold;
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

    <hr>

    <div class="value">
      <span class="label">Closest Location:</span>
      <span id="closest">--</span>
    </div>

    <div class="value">
      <span class="label">Receiver to Transmitter:</span>
      <span id="receiverDistance">--</span>
    </div>

    <hr>

    <h2>Distances to Known Locations</h2>

    <table>
      <thead>
        <tr>
          <th>Location</th>
          <th>Distance</th>
        </tr>
      </thead>
      <tbody id="locationTable">
        <tr>
          <td colspan="2">Loading...</td>
        </tr>
      </tbody>
    </table>
  </div>

  <script>
    async function updateGps() {
      try {
        const res = await fetch('/gps');
        const data = await res.json();

        document.getElementById('status').textContent =
          data.hasFix ? 'Receiving' : 'Waiting';

        document.getElementById('lat').textContent =
          data.lat.toFixed(6);

        document.getElementById('lon').textContent =
          data.lon.toFixed(6);

        document.getElementById('alt').textContent =
          data.alt.toFixed(1) + ' m';

        document.getElementById('rssi').textContent =
          data.rssi + ' dBm';

        document.getElementById('snr').textContent =
          data.snr.toFixed(1) + ' dB';

        document.getElementById('age').textContent =
          data.ageMs + ' ms';

        document.getElementById('closest').textContent =
          data.closestLocation + ' (' + data.closestDistanceM.toFixed(1) + ' m)';

        document.getElementById('receiverDistance').textContent =
          data.receiverDistanceM.toFixed(1) + ' m';

        const table = document.getElementById('locationTable');
        table.innerHTML = '';

        for (const location of data.locations) {
          const row = document.createElement('tr');

          if (location.name === data.closestLocation) {
            row.classList.add('closest');
          }

          const nameCell = document.createElement('td');
          nameCell.textContent = location.name;

          const distanceCell = document.createElement('td');
          distanceCell.textContent = location.distanceM.toFixed(1) + ' m';

          row.appendChild(nameCell);
          row.appendChild(distanceCell);
          table.appendChild(row);
        }

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

    float closestDistanceM = 0.0;
    String closestLocation = "--";
    float receiverDistanceM = 0.0;

    if (hasGpsPacket) {
        closestLocation = findClosestLocation(
            latestLat,
            latestLon,
            closestDistanceM
        );

        receiverDistanceM = distanceMeters(
            RECEIVER_LAT,
            RECEIVER_LON,
            latestLat,
            latestLon
        );
    }

    String json = "{";

    json += "\"hasFix\":" + String(hasGpsPacket ? "true" : "false") + ",";
    json += "\"lat\":" + String(latestLat, 6) + ",";
    json += "\"lon\":" + String(latestLon, 6) + ",";
    json += "\"alt\":" + String(latestAlt, 1) + ",";
    json += "\"rssi\":" + String(latestRssi) + ",";
    json += "\"snr\":" + String(latestSnr, 1) + ",";
    json += "\"ageMs\":" + String(ageMs) + ",";

    json += "\"closestLocation\":\"" + closestLocation + "\",";
    json += "\"closestDistanceM\":" + String(closestDistanceM, 1) + ",";
    json += "\"receiverDistanceM\":" + String(receiverDistanceM, 1) + ",";

    json += "\"locations\":[";

    for (int i = 0; i < NUM_LOCATIONS; i++) {
        float distanceM = 0.0;

        if (hasGpsPacket) {
            distanceM = distanceMeters(
                latestLat,
                latestLon,
                LOCATIONS[i].lat,
                LOCATIONS[i].lon
            );
        }

        json += "{";
        json += "\"name\":\"" + String(LOCATIONS[i].name) + "\",";
        json += "\"distanceM\":" + String(distanceM, 1);
        json += "}";

        if (i < NUM_LOCATIONS - 1) {
            json += ",";
        }
    }

    json += "]";
    json += "}";

    server.send(200, "application/json", json);
}

// ----- OLED display -----
void displayCoords(float lat, float lon, float alt, int rssi, float snr) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);

    display.setCursor(0, 10);
    display.print("GPS Receiver (V3)");

    display.setCursor(0, 24);
    display.printf("Lat: %.6f", lat);

    display.setCursor(0, 36);
    display.printf("Lon: %.6f", lon);

    display.setCursor(0, 48);
    display.printf("Alt: %.1f m", alt);

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

// ----- Setup -----
void setup() {
    Serial.begin(115200);

    WiFi.softAP(AP_SSID, AP_PASS);

    Serial.print("Access point started. Connect to: ");
    Serial.println(AP_SSID);
    Serial.print("Then open: http://");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/gps", handleGpsJson);
    server.begin();

    Serial.println("Web server started");

    display.begin();
    displayStatus("Initializing...");

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
        displayStatus("Radio FAIL!");
        while (true);
    }

    radio.setDio2AsRfSwitch(true);

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

// ----- Main loop -----
void loop() {
    server.handleClient();

    if (!rxFlag) {
        return;
    }

    rxFlag = false;

    GpsPacket pkt;
    int state = radio.readData((uint8_t*)&pkt, sizeof(pkt));

    if (state == RADIOLIB_ERR_NONE) {
        int rssi = radio.getRSSI();
        float snr = radio.getSNR();

        latestLat = pkt.lat;
        latestLon = pkt.lon;
        latestAlt = pkt.alt;
        latestRssi = rssi;
        latestSnr = snr;
        latestPacketTime = millis();
        hasGpsPacket = true;

        Serial.printf(
            "Lat: %.6f  Lon: %.6f  Alt: %.1fm  RSSI: %ddBm  SNR: %.2fdB\n",
            pkt.lat,
            pkt.lon,
            pkt.alt,
            rssi,
            snr
        );

        displayCoords(pkt.lat, pkt.lon, pkt.alt, rssi, snr);
    }
}