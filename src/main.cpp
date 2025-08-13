#include <Arduino.h>
#include <WiFi.h>           // Native ESP32 WiFi
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// UART1 for EC200U LTE module
HardwareSerial ecSerial(1);
#define EC200U_RX 5
#define EC200U_TX 6

// Wi-Fi credentials for the user's home LAN
const char* WIFI_SSID = "Manan";
const char* WIFI_PASS = "88888888";
const String FIREBASE_HOST = "myperro-gps-default-rtdb.firebaseio.com";

// Geofence and state variables
const int GEOFENCE_RSSI_THRESHOLD = -70; // RSSI value in dBm to trigger geofence breach
String currentMode = "geosafe_off";      // Default mode
bool trackingActive = false;             // Tracks if the LTE/GPS module is actively tracking

// Function declarations
void connectToWiFi();
void initializeLTE();
String getGeosafeModeFromFirebase();
void geofenceCheck();
void liveTracking();
bool getRawLoc(String &lat, String &lon, String &fixTime);
String convertToDecimal(String dms);
void sendToFirebase(const String &payload);
String sendAT(const String &cmd, uint32_t timeout);
String sendHTTPGET(const String &url, uint32_t timeout);

void setup() {
  Serial.begin(115200);
  ecSerial.begin(115200, SERIAL_8N1, EC200U_RX, EC200U_TX);
  delay(500);
  connectToWiFi();
}

void loop() {
  currentMode = getGeosafeModeFromFirebase();
  Serial.printf("⚙ Current Geosafe Mode: %s\n", currentMode.c_str());

  if (currentMode.equals("geosafe_off")) {
    geofenceCheck();
  } else if (currentMode.equals("geosafe_off_runaway")) {
    liveTracking();
  } else {
    Serial.println("✅ Geosafe mode is ON. Geofencing is disabled for now.");
    if (trackingActive) {
      Serial.println("🛑 Disabling tracking as Geosafe is ON.");
      sendAT("AT+QGPSEND", 300);
      sendAT("AT+QIDEACT=1", 300);
      trackingActive = false;
    }
  }

  delay(60000); // Poll every 1 minute
}

void geofenceCheck() {
  if (WiFi.status() == WL_CONNECTED && WiFi.RSSI() > GEOFENCE_RSSI_THRESHOLD) {
    Serial.println("📍 Inside Wi-Fi geofence. Idle mode.");
    if (trackingActive) {
      Serial.println("🛑 Back in range, disabling tracking.");
      sendAT("AT+QGPSEND", 300);
      sendAT("AT+QIDEACT=1", 300);
      trackingActive = false;
    }
  } else {
    Serial.println("🚪 Wi-Fi geofence breached! Activating tracking...");
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    if (!trackingActive) {
      initializeLTE();
      sendAT("AT+QGPS=1", 500);
      trackingActive = true;
    }

    String latRaw, lonRaw, fixTime;
    if (getRawLoc(latRaw, lonRaw, fixTime)) {
      String latDD = convertToDecimal(latRaw);
      String lonDD = convertToDecimal(lonRaw);
      Serial.printf("🚨 Geofence breached. Lat: %s, Lon: %s\n", latDD.c_str(), lonDD.c_str());

      JsonDocument doc;
      doc["timestamp"] = fixTime;
      doc["latitude"] = latDD;
      doc["longitude"] = lonDD;
      doc["alert"] = "Wi-Fi geofence lost. Dog is out of range.";

      String payload;
      serializeJson(doc, payload);
      sendToFirebase(payload);
    } else {
      Serial.println("❌ No GPS fix available.");
    }
  }
}

void liveTracking() {
  Serial.println("🏃 Live tracking mode activated. Sending frequent updates...");

  if (!trackingActive) {
    initializeLTE();
    sendAT("AT+QGPS=1", 500);
    trackingActive = true;
  }

  String latRaw, lonRaw, fixTime;
  if (getRawLoc(latRaw, lonRaw, fixTime)) {
    String latDD = convertToDecimal(latRaw);
    String lonDD = convertToDecimal(lonRaw);
    Serial.printf("🏃 Live Location: Lat: %s, Lon: %s\n", latDD.c_str(), lonDD.c_str());

    JsonDocument doc;
    doc["timestamp"] = fixTime;
    doc["latitude"] = latDD;
    doc["longitude"] = lonDD;
    doc["alert"] = "Live tracking update.";

    String payload;
    serializeJson(doc, payload);
    sendToFirebase(payload);
  } else {
    Serial.println("❌ No GPS fix available for live tracking.");
  }
}

String getGeosafeModeFromFirebase() {
  String url = "https://" + FIREBASE_HOST + "/collar_settings.json?print=silent";
  String res = sendHTTPGET(url, 2000);
  
  if (res.length() > 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, res);
    if (!error) {
      if (doc["geosafe_mode"].is<String>()) {
        return doc["geosafe_mode"].as<String>();
      }
    }
  }
  return "geosafe_off";
}

String sendHTTPGET(const String &url, uint32_t timeout) {
  sendAT("AT+QHTTPCFG=\"contextid\",1", 200);
  sendAT("AT+QHTTPCFG=\"requestheader\",0", 200);
  sendAT("AT+QHTTPCFG=\"sslctxid\",1", 200);
  sendAT("AT+QSSLCFG=\"sslversion\",1,4", 200);
  sendAT("AT+QSSLCFG=\"seclevel\",1,0", 200);
  sendAT("AT+QHTTPURL=" + String(url.length()) + ",80", 500);
  ecSerial.println(url);
  delay(100);
  ecSerial.println("AT+QHTTPGET=80");

  String resp;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (ecSerial.available()) resp += char(ecSerial.read());
  }
  int bodyStart = resp.indexOf("{");
  if (bodyStart >= 0) {
    resp = resp.substring(bodyStart);
    int bodyEnd = resp.lastIndexOf("}");
    if (bodyEnd >= 0) {
      resp = resp.substring(0, bodyEnd + 1);
    }
  }
  return resp;
}

void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("🔌 Connecting to Wi-Fi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\n✅ Wi-Fi connected" : "\n❌ Wi-Fi not found. Operating without geofence.");
}

void initializeLTE() {
  sendAT("AT", 200);
  sendAT("AT+CFUN=1", 300);
  sendAT("AT+QICSGP=1,1,\"airtelgprs.com\",\"\",\"\",1", 300);
  sendAT("AT+CGATT=1", 300);
  sendAT("AT+QIACT=1", 1000);
}

bool getRawLoc(String &lat, String &lon, String &fixTime) {
  String res = sendAT("AT+QGPSLOC?", 1000);
  int i = res.indexOf("+QGPSLOC:");
  if (i < 0) return false;
  String line = res.substring(i + 10, res.indexOf("\r", i));
  fixTime = line.substring(0, 6);
  int p1 = line.indexOf(','), p2 = line.indexOf(',', p1 + 1), p3 = line.indexOf(',', p2 + 1);
  lat = line.substring(p1 + 1, p2);
  lon = line.substring(p2 + 1, p3);
  return true;
}

String convertToDecimal(String dms) {
  char hemi = dms[dms.length() - 1];
  dms.remove(dms.length() - 1);
  int degDigits = (dms.length() == 9) ? 2 : 3;
  float deg = dms.substring(0, degDigits).toFloat();
  float min = dms.substring(degDigits).toFloat();
  float dec = deg + (min / 60.0);
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return String(dec, 6);
}

void sendToFirebase(const String &payload) {
  sendAT("AT+QHTTPCFG=\"contextid\",1", 200);
  sendAT("AT+QHTTPCFG=\"requestheader\",0", 200);
  sendAT("AT+QHTTPCFG=\"sslctxid\",1", 200);
  sendAT("AT+QSSLCFG=\"sslversion\",1,4", 200);
  sendAT("AT+QSSLCFG=\"seclevel\",1,0", 200);
  sendAT("AT+QHTTPCFG=\"url\",\"https://" + FIREBASE_HOST + "/locations.json?print=silent\"", 500);

  ecSerial.println("AT+QHTTPPOST=" + String(payload.length()) + ",60,60");
  delay(200);
  if (ecSerial.find("CONNECT")) {
    ecSerial.print(payload);
    Serial.println("📤 Sent: " + payload);
  } else {
    Serial.println("❌ POST failed");
  }

  delay(800);
  while (ecSerial.available()) Serial.write(ecSerial.read());
}

String sendAT(const String &cmd, uint32_t timeout) {
  ecSerial.println(cmd);
  String resp;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (ecSerial.available()) resp += char(ecSerial.read());
  }
  Serial.printf("🔁 %s\n%s\n", cmd.c_str(), resp.c_str());
  return resp;
}
