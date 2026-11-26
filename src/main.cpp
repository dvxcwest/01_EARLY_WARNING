
// WiFi
const char* WIFI_SSID     = "YVL";
const char* WIFI_PASSWORD = "wordwordle66";

// ThingsBoard Cloud server.  US region: "mqtt.thingsboard.cloud"
const char* TB_HOST = "mqtt.thingsboard.cloud";

// 0 = plain MQTT on port 1883 (easiest for testing)
// 1 = MQTT over TLS on port 8883 (encrypted)
#define TB_USE_TLS 0

// Access token of each ThingsBoard device:  Devices > click the device > "Copy access token".
const char* TOKEN_SOIL   = "rqSHYKYSwy5ZXNjG2LSH";        // device "SOIL MOISTURE" (SEN-0016)
const char* TOKEN_RAIN   = "fBDYc3zBof6xkEUauNQI";        // device "RAIN DROP"     (FC-37)
const char* TOKEN_LUX    = "Ysn02SDKzMdFEV7Suy0B";        // device "LUX"           (TSL2591)
const char* TOKEN_THERMO = "vlTwXDRONyunlVzv8LwD";        // device "THERMOHYGROMETER FARM" (DHT22)
const char* TOKEN_NODE   = "";                            // OPTIONAL device "ESP32 NODE" ("" = off)

// ---------------------------------------------------------------------
// 2. INCLUDES
// ---------------------------------------------------------------------
#include <Arduino.h>
#include <WiFi.h>
#if TB_USE_TLS
  #include <WiFiClientSecure.h>
#endif
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <esp_task_wdt.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include "soil_forecast_model.h"      // MLR V7 model

#define FIRMWARE_VERSION "MON-V6-TBCLOUD"

// ---------------------------------------------------------------------
// 3. CONSTANTS & CALIBRATION PARAMETERS
// ---------------------------------------------------------------------
#define WDT_TIMEOUT_SEC      90          // watchdog: restart if the loop hangs for 90 s
#define WIFI_MAX_RETRY       20          // x 500 ms = 10 s per reconnect attempt
#define WIFI_RETRY_GAP_MS    30000UL     // try to bring WiFi back at most every 30 s
#define WIFI_MAX_DOWN_MS     1800000UL   // restart only after 30 min without WiFi
#define TB_CONNECT_TRIES     2           // connection attempts per device and cycle
#define TB_PAYLOAD_SIZE      512

const int SOIL_MOISTURE_PIN = 33;
const int RAINDROP_PIN      = 32;
const int DHT_PIN           = 15;
#define DHT_TYPE DHT22
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Must equal SOIL_SAMPLE_INTERVAL_S in soil_forecast_model.h (300 s): 5-minute sampling interval
const unsigned long READ_INTERVAL_MS = 300000UL;
const int NUM_SAMPLES = 10;

// --- SOIL MOISTURE CALIBRATION (SEN-0016) ---
// Formula: GWC(%) = CALIB_SLOPE * (CALIB_ADC_MAX / (ADC * CALIB_V_REF)) + CALIB_INTERCEPT
// Evaluated from 200g dry soil with 20ml - 160ml water addition
const float CALIB_SLOPE     = 38.3538f;
const float CALIB_INTERCEPT = -2.9418f;
const float CALIB_V_REF     = 3.3f;
const float CALIB_ADC_MAX   = 4095.0f;
const float CALIB_MIN_PCT   = 10.0f;     // lower bound of experimental validation (20ml / 200g = 10%)
const float CALIB_MAX_PCT   = 80.0f;     // upper bound of experimental validation (160ml / 200g = 80%)

// FC-37 rain sensor: used ONLY as rain / no rain (rain_binary)
const int WARNING_THRESHOLD = 2800;      // raw below this = rain (same as RAIN_THRESHOLD in notebook)
const int WET_MIN           = 1500;      // raw below this = heavy rain (status text only)

// ---------------------------------------------------------------------
// 4. OBJECTS AND GLOBALS
// ---------------------------------------------------------------------
#if TB_USE_TLS
  WiFiClientSecure netClient;
  const uint16_t TB_PORT = 8883;
#else
  WiFiClient netClient;
  const uint16_t TB_PORT = 1883;
#endif
PubSubClient     mqtt(netClient);
DHT              dht(DHT_PIN, DHT_TYPE);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

unsigned long lastTick      = 0;
unsigned long wifiDownSince = 0;
bool          tslInitialized = false;
unsigned long cyclesDone    = 0;

SoilForecaster soilForecaster;
int   samplesPushed = 0;
float lastGoodTemp = NAN, lastGoodHum = NAN, lastGoodLux = NAN;
bool  needTemp = false, needHum = false, needLux = false, needClock = false;

struct SensorData {
    int         soilRaw;
    float       soilVoltage;         // V
    float       soilInvV;            // V^-1
    float       soilPct;             // GWC %
    int         rainRaw;
    int         rainBinary;          // 1 = rain, 0 = no rain
    const char* rainStatus;
    bool        dhtOk;
    float       temperature;
    float       humidity;
    bool        tslOk;
    float       lux;
    uint16_t    ir, full, visible;
    bool        allOk;
};

struct ForecastResult {
    bool  valid;       // a real forecast was computed this cycle
    float value;       // soil moisture expected in 30 min (%)
    float change;      // value minus soil moisture now (percentage points)
    bool  alert;       // forecast below SOIL_ALERT_THRESHOLD
    char  status[40];  // "OK MLR", "Warm-up 2/4", "Waiting NTP" ...
};

// =====================================================================
// WATCHDOG SETUP
// =====================================================================
void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    esp_task_wdt_config_t cfg;
    cfg.timeout_ms     = WDT_TIMEOUT_SEC * 1000;
    cfg.idle_core_mask = 0;
    cfg.trigger_panic  = true;
    esp_task_wdt_reconfigure(&cfg);
#else
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif
    esp_task_wdt_add(NULL);
}

// =====================================================================
// WIFI MANAGEMENT
// =====================================================================
void setupWiFi() {
    Serial.printf("\nMenghubungkan ke WiFi: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < WIFI_MAX_RETRY) {
        delay(500); Serial.print("."); attempts++;
        esp_task_wdt_reset();
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n✓ WiFi terhubung. RSSI %d dBm\n", WiFi.RSSI());
    } else {
        Serial.println("\n✗ Gagal WiFi. Restart...");
        delay(1000);
        ESP.restart();
    }
}

bool checkWiFi() {
    if (WiFi.status() == WL_CONNECTED) { wifiDownSince = 0; return true; }

    unsigned long now = millis();
    if (wifiDownSince == 0) wifiDownSince = now;

    static unsigned long lastAttempt = 0;
    if (lastAttempt != 0 && (now - lastAttempt) < WIFI_RETRY_GAP_MS) return false;
    lastAttempt = now;

    Serial.println("⚠ WiFi terputus! Mencoba menghubungkan kembali...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < WIFI_MAX_RETRY) {
        delay(500); Serial.print("."); attempts++;
        esp_task_wdt_reset();
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n✓ WiFi pulih. RSSI %d dBm\n", WiFi.RSSI());
        wifiDownSince = 0;
        return true;
    }
    Serial.println("\n✗ WiFi belum terhubung (sensor tetap dibaca secara offline).");
    if (now - wifiDownSince > WIFI_MAX_DOWN_MS) {
        Serial.println("✗ WiFi terputus > 30 menit. Merestart sistem...");
        delay(500);
        ESP.restart();
    }
    return false;
}

// =====================================================================
// SENSORS ACQUISITION & CALIBRATION
// =====================================================================
bool setupTSL2591() {
    Serial.print("Inisialisasi TSL2591... ");
    if (!tsl.begin()) {
        Serial.println("✗ GAGAL! Periksa jalur I2C (SDA:21, SCL:22)");
        return false;
    }
    Serial.println("✓ OK");
    tsl.setGain(TSL2591_GAIN_LOW);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
    return true;
}

void readSoilMoisture(SensorData &d) {
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(SOIL_MOISTURE_PIN);
        delay(10);
    }
    d.soilRaw = (int)(sum / NUM_SAMPLES);

    // Safeguard against open circuit, zero readings, or sensor disconnected
    if (d.soilRaw <= 10) {
        d.soilVoltage = 0.0f;
        d.soilInvV    = 0.0f;
        d.soilPct     = CALIB_MIN_PCT;
        return;
    }

    // 1. Calculate analog voltage output V
    d.soilVoltage = ((float)d.soilRaw / CALIB_ADC_MAX) * CALIB_V_REF;

    // 2. Compute reciprocal / inverse voltage (V^-1)
    d.soilInvV = 1.0f / d.soilVoltage;

    // 3. Apply calibrated linear model: GWC(%) = 38.3538 * V^-1 - 2.9418
    float calculatedPct = (CALIB_SLOPE * d.soilInvV) + CALIB_INTERCEPT;

    // 4. Bound to experimental calibration domain [10.0%, 80.0%]
    if (calculatedPct < CALIB_MIN_PCT) calculatedPct = CALIB_MIN_PCT;
    if (calculatedPct > CALIB_MAX_PCT) calculatedPct = CALIB_MAX_PCT;
    d.soilPct = calculatedPct;
}

void readRaindrop(SensorData &d) {
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) { sum += analogRead(RAINDROP_PIN); delay(10); }
    d.rainRaw    = (int)(sum / NUM_SAMPLES);
    d.rainBinary = (d.rainRaw < WARNING_THRESHOLD) ? 1 : 0;
    if (d.rainRaw >= WARNING_THRESHOLD)      d.rainStatus = "Tidak Hujan";
    else if (d.rainRaw >= WET_MIN)           d.rainStatus = "Gerimis";
    else                                     d.rainStatus = "Hujan Deras";
}

void readDHT22(SensorData &d) {
    d.humidity    = dht.readHumidity();
    d.temperature = dht.readTemperature();
    d.dhtOk = !(isnan(d.humidity) || isnan(d.temperature));
    if (!d.dhtOk) { Serial.println("✗ Gagal membaca DHT22!"); d.allOk = false; }
}

void readTSL2591(SensorData &d) {
    if (!tslInitialized) tslInitialized = setupTSL2591();
    d.tslOk = false; d.lux = 0; d.ir = 0; d.full = 0; d.visible = 0;
    if (!tslInitialized) { d.allOk = false; return; }

    uint32_t lum = tsl.getFullLuminosity();
    d.ir      = lum >> 16;
    d.full    = lum & 0xFFFF;
    d.visible = d.full - d.ir;
    d.lux     = tsl.calculateLux(d.full, d.ir);
    if (d.lux < 0 || d.lux > 88000) { Serial.println("✗ Pembacaan TSL2591 di luar batas wajar!"); d.lux = 0; d.allOk = false; return; }
    d.tslOk = true;
}

SensorData readAllSensors() {
    SensorData d;
    memset(&d, 0, sizeof(d));
    d.allOk = true;
    Serial.println("\n===== PEMBACAAN SENSOR =====");
    readSoilMoisture(d);
    readRaindrop(d);
    readDHT22(d);
    readTSL2591(d);
    return d;
}

void printSensorData(const SensorData &d) {
    Serial.printf("Tanah  : raw %4d -> %.4f V (V^-1: %.4f) -> %.2f %%\n", d.soilRaw, d.soilVoltage, d.soilInvV, d.soilPct);
    Serial.printf("Hujan  : raw %4d -> %s (rain_binary=%d)\n", d.rainRaw, d.rainStatus, d.rainBinary);
    if (d.dhtOk) Serial.printf("DHT22  : %.1f °C | %.1f %%\n", d.temperature, d.humidity);
    else         Serial.println("DHT22  : ERROR");
    if (d.tslOk) Serial.printf("TSL2591: %.2f lux (IR: %u, Full: %u, Visible: %u)\n", d.lux, d.ir, d.full, d.visible);
    else         Serial.println("TSL2591: ERROR");
}

// =====================================================================
// 30-MINUTE MLR FORECASTING
// =====================================================================
bool modelUses(const char* featureName) {
    for (int i = 0; i < SOIL_N_FEATURES; i++)
        if (strcmp(SOIL_FEATURE_NAMES[i], featureName) == 0) return true;
    return false;
}

void setupForecast() {
    needTemp  = modelUses("temperature");
    needHum   = modelUses("humidity");
    needLux   = modelUses("lux");
    needClock = modelUses("hour_sin") || modelUses("hour_cos");

    Serial.printf("  Model  : %s (%s), %d fitur\n", SOIL_MODEL_NAME, SOIL_DEPLOY_MODE, SOIL_N_FEATURES);
    Serial.printf("  Warm-up: %d sampel x %d menit\n", SOIL_BUF_LEN, SOIL_SAMPLE_INTERVAL_S / 60);

#if defined(SOIL_INV_V_SLOPE) && defined(SOIL_INV_V_INTERCEPT)
    if (fabsf(SOIL_INV_V_SLOPE - CALIB_SLOPE) > 0.01f || fabsf(SOIL_INV_V_INTERCEPT - CALIB_INTERCEPT) > 0.01f)
        Serial.println("  ⚠ Kalibrasi inverse voltage firmware BERBEDA dari header model! Periksa soil_forecast_model.h.");
    else
        Serial.println("  ✓ Kalibrasi inverse voltage firmware selaras dengan header model (V7)");
#else
    Serial.println("  ℹ Header model terdeteksi; pastikan header telah diekspor dari notebook V7.");
#endif

    if ((int)SOIL_RAIN_THRESHOLD != WARNING_THRESHOLD)
        Serial.println("  ⚠ Ambang hujan firmware berbeda dari model! Samakan WARNING_THRESHOLD dengan RAIN_THRESHOLD notebook.");
    if ((unsigned long)SOIL_SAMPLE_INTERVAL_S * 1000UL != READ_INTERVAL_MS)
        Serial.println("  ⚠ READ_INTERVAL_MS tidak sama dengan interval model! Prakiraan tidak valid.");
}

bool getHourFraction(float &hourFrac) {
    struct tm t;
    if (!getLocalTime(&t, 100)) return false;
    hourFrac = t.tm_hour + t.tm_min / 60.0f + t.tm_sec / 3600.0f;
    return true;
}

ForecastResult updateForecast(const SensorData &d) {
    ForecastResult r;
    r.valid = false; r.value = NAN; r.change = NAN; r.alert = false; r.status[0] = '\0';

    if (d.dhtOk) { lastGoodTemp = d.temperature; lastGoodHum = d.humidity; }
    if (d.tslOk) lastGoodLux = d.lux;

    soilForecaster.push(d.soilPct, (float)d.rainRaw);
    if (samplesPushed < SOIL_BUF_LEN) samplesPushed++;

    if (!soilForecaster.ready()) {
        snprintf(r.status, sizeof(r.status), "Warm-up %d/%d", samplesPushed, SOIL_BUF_LEN);
        return r;
    }
    if ((needTemp && isnan(lastGoodTemp)) || (needHum && isnan(lastGoodHum)) || (needLux && isnan(lastGoodLux))) {
        snprintf(r.status, sizeof(r.status), "No env data");
        return r;
    }
    float hourFrac = 12.0f;
    if (needClock && !getHourFraction(hourFrac)) {
        snprintf(r.status, sizeof(r.status), "Waiting NTP");
        return r;
    }

    float f = soilForecaster.forecast(needTemp ? lastGoodTemp : 0.0f, needHum ? lastGoodHum : 0.0f,
                                      needLux ? lastGoodLux : 0.0f, hourFrac);
    r.valid  = true;
    r.value  = f;
    r.change = f - d.soilPct;
    r.alert  = soilForecaster.alert(f);
    snprintf(r.status, sizeof(r.status), "OK %s", SOIL_DEPLOY_MODE);
    return r;
}

void printForecast(const ForecastResult &fc, const SensorData &d) {
    Serial.printf("Prakiraan: %s", fc.status);
    if (fc.valid)
        Serial.printf(" | saat ini %.2f %% -> 30 menit ke depan %.2f %% (%+.2f)%s", d.soilPct, fc.value, fc.change, fc.alert ? "  [PERINGATAN]" : "");
    Serial.println();
}

// =====================================================================
// THINGSBOARD CLOUD MQTT INTEGRATION
// =====================================================================
class JsonOut {
 public:
    char   buf[TB_PAYLOAD_SIZE];
    size_t len;
    bool   hasKeys, wrapped;

    void begin(uint64_t tsMs) {
        len = 0; hasKeys = false; wrapped = (tsMs != 0);
        buf[0] = '\0';
        if (wrapped) put("{\"ts\":%llu,\"values\":{", (unsigned long long)tsMs);
        else         put("{");
    }
    void addInt(const char* k, long v)           { sep(); put("\"%s\":%ld", k, v); }
    void addFloat(const char* k, float v, int d) { if (!isfinite(v)) return; sep(); put("\"%s\":%.*f", k, d, v); }
    void addStr(const char* k, const char* v)    { sep(); put("\"%s\":\"%s\"", k, v); }
    const char* end() { put(wrapped ? "}}" : "}"); return buf; }

 private:
    void sep() { if (hasKeys) put(","); hasKeys = true; }
    void put(const char* fmt, ...) {
        va_list ap; va_start(ap, fmt);
        int n = vsnprintf(buf + len, sizeof(buf) - len, fmt, ap);
        va_end(ap);
        if (n > 0) len = (len + n < sizeof(buf) - 1) ? len + n : sizeof(buf) - 1;
    }
};

uint64_t nowEpochMs() {
    time_t t = time(nullptr);
    return (t > 1700000000) ? (uint64_t)t * 1000ULL : 0ULL;
}

static bool tokenMissing(const char* t) { return t == nullptr || t[0] == '\0' || strncmp(t, "PASTE_", 6) == 0; }

void tbSetupClient() {
#if TB_USE_TLS
    netClient.setInsecure();
#endif
    mqtt.setServer(TB_HOST, TB_PORT);
    mqtt.setBufferSize(TB_PAYLOAD_SIZE + 64);
    mqtt.setKeepAlive(30);
    mqtt.setSocketTimeout(8);
}

const char* mqttStateText(int rc) {
    switch (rc) {
        case -4: return "timeout";
        case -3: return "koneksi terputus";
        case -2: return "gagal tersambung ke server";
        case -1: return "terputus";
        case 4:  return "kredensial salah";
        case 5:  return "tidak berwenang / access token salah";
        default: return "lainnya";
    }
}

bool tbPublish(const char* label, const char* token, const char* topic, const char* payload) {
    if (tokenMissing(token)) { Serial.printf("  - %-8s: token kosong, dilewati\n", label); return false; }
    if (WiFi.status() != WL_CONNECTED) { Serial.printf("  - %-8s: tidak ada koneksi WiFi\n", label); return false; }

    char clientId[32];
    snprintf(clientId, sizeof(clientId), "esp32-%s", label);

    for (int attempt = 1; attempt <= TB_CONNECT_TRIES; attempt++) {
        esp_task_wdt_reset();
        if (mqtt.connect(clientId, token, NULL)) {
            bool ok = mqtt.publish(topic, payload);
            mqtt.loop();
            delay(150);
            mqtt.disconnect();
            if (ok) { Serial.printf("  ✓ %-8s: terkirim (%u byte)\n", label, (unsigned)strlen(payload)); return true; }
            Serial.printf("  ✗ %-8s: gagal publish\n", label);
        } else {
            int rc = mqtt.state();
            Serial.printf("  ✗ %-8s: gagal terhubung rc=%d (%s) [percobaan %d/%d]\n", label, rc, mqttStateText(rc), attempt, TB_CONNECT_TRIES);
            if (rc == 4 || rc == 5) return false;
        }
        delay(500);
    }
    return false;
}

int sendCycleToThingsBoard(const SensorData &d, const ForecastResult &fc) {
    const char* T = "v1/devices/me/telemetry";
    uint64_t ts = nowEpochMs();
    JsonOut j;
    int ok = 0, total = 0;

    Serial.println("Mengirim telemetri ke ThingsBoard Cloud...");

    // --- device SOIL MOISTURE (SEN-0016): raw, voltage, inverse voltage, and calibrated % ---
    j.begin(ts);
    j.addInt("sensor_ok", 1);
    j.addInt("soil_moisture_raw", d.soilRaw);
    j.addFloat("soil_moisture_voltage", d.soilVoltage, 3);
    j.addFloat("soil_moisture_inv_v", d.soilInvV, 3);
    j.addFloat("soil_moisture_percent", d.soilPct, 2);
    if (fc.valid) {
        j.addFloat("soil_forecast_30m", fc.value, 2);
        j.addFloat("soil_forecast_change", fc.change, 2);
        j.addInt("soil_forecast_alert", fc.alert ? 1 : 0);
    }
    j.addStr("forecast_status", fc.status);
    total++; ok += tbPublish("soil", TOKEN_SOIL, T, j.end());

    // --- device RAIN DROP (FC-37) ---
    j.begin(ts);
    j.addInt("sensor_ok", 1);
    j.addInt("rain_raw", d.rainRaw);
    j.addInt("rain_binary", d.rainBinary);
    j.addStr("rain_status", d.rainStatus);
    total++; ok += tbPublish("rain", TOKEN_RAIN, T, j.end());

    // --- device LUX (TSL2591) ---
    j.begin(ts);
    j.addInt("sensor_ok", d.tslOk ? 1 : 0);
    if (d.tslOk) {
        j.addFloat("lux", d.lux, 2);
        j.addInt("light_ir", d.ir);
        j.addInt("light_full", d.full);
        j.addInt("light_visible", d.visible);
    }
    total++; ok += tbPublish("lux", TOKEN_LUX, T, j.end());

    // --- device THERMOHYGROMETER FARM (DHT22) ---
    j.begin(ts);
    j.addInt("sensor_ok", d.dhtOk ? 1 : 0);
    if (d.dhtOk) {
        j.addFloat("temperature", d.temperature, 1);
        j.addFloat("humidity", d.humidity, 1);
    }
    total++; ok += tbPublish("thermo", TOKEN_THERMO, T, j.end());

    // --- optional device ESP32 NODE ---
    if (!tokenMissing(TOKEN_NODE)) {
        j.begin(ts);
        j.addInt("wifi_rssi", WiFi.RSSI());
        j.addInt("uptime_s", (long)(millis() / 1000UL));
        j.addInt("free_heap", (long)ESP.getFreeHeap());
        j.addInt("sensor_status_ok", d.allOk ? 1 : 0);
        j.addInt("forecast_ready", fc.valid ? 1 : 0);
        total++; ok += tbPublish("node", TOKEN_NODE, T, j.end());
    }

    Serial.printf("Hasil: %d/%d device terkirim\n", ok, total);
    return ok;
}

void sendBootAttributes() {
    JsonOut j;
    j.begin(0);
    j.addStr("firmware", FIRMWARE_VERSION);
    j.addStr("forecast_model", SOIL_MODEL_NAME);
    j.addStr("forecast_mode", SOIL_DEPLOY_MODE);
    j.addInt("forecast_horizon_min", SOIL_HORIZON_MIN);
    j.addInt("sample_interval_s", SOIL_SAMPLE_INTERVAL_S);
    j.addFloat("alert_threshold_pct", SOIL_ALERT_THRESHOLD, 1);
    j.addStr("calibration_method", "Inverse Voltage Linear Regression");
    j.addFloat("calib_slope", CALIB_SLOPE, 4);
    j.addFloat("calib_intercept", CALIB_INTERCEPT, 4);
    j.addFloat("calib_min_pct", CALIB_MIN_PCT, 1);
    j.addFloat("calib_max_pct", CALIB_MAX_PCT, 1);
    tbPublish("soil", TOKEN_SOIL, "v1/devices/me/attributes", j.end());
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    setupWatchdog();

    Serial.println("\n==============================================");
    Serial.println(" ESP32 MULTI-SENSOR -> THINGSBOARD CLOUD + MLR");
    Serial.println(" " FIRMWARE_VERSION);
    Serial.println("==============================================");

    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(RAINDROP_PIN, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Wire.begin(I2C_SDA, I2C_SCL);
    dht.begin();
    tslInitialized = setupTSL2591();

    setupWiFi();

    Serial.println("► Sinkronisasi waktu (NTP, UTC+8 / WITA)...");
    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    {
        struct tm t; int waited = 0;
        while (!getLocalTime(&t, 1000) && waited < 10) { waited++; esp_task_wdt_reset(); }
        Serial.println(waited < 10 ? "  ✓ NTP tersinkron" : "  ⚠ NTP belum tersinkron (sementara menggunakan waktu server)");
    }

    Serial.println("► Konfigurasi model prakiraan 30 menit...");
    setupForecast();

    Serial.printf("► Menghubungkan ke ThingsBoard Cloud: %s:%u (%s)\n", TB_HOST, (unsigned)TB_PORT, TB_USE_TLS ? "TLS" : "plain");
    tbSetupClient();
    sendBootAttributes();

    Serial.printf("\nSistem aktif. Pengambilan sampel setiap %lu detik.\n", READ_INTERVAL_MS / 1000UL);
    lastTick = millis() - READ_INTERVAL_MS;       // trigger first sample immediately
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
    esp_task_wdt_reset();
    checkWiFi();

    unsigned long now = millis();
    if (now - lastTick >= READ_INTERVAL_MS) {
        lastTick = now;

        SensorData d = readAllSensors();
        printSensorData(d);

        ForecastResult fc = updateForecast(d);
        printForecast(fc, d);

        sendCycleToThingsBoard(d, fc);
        cyclesDone++;
        Serial.printf("→ Pengambilan sampel berikutnya dalam %lu detik\n", READ_INTERVAL_MS / 1000UL);
    }
    delay(10);
}