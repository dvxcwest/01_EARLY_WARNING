// #include <Arduino.h>
// #include <WiFi.h>

// GPIO 2 is usually the built-in LED on most ESP32 boards.
// If your board does not have an LED, this does nothing (safe to ignore).
#include <Arduino.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <ArduinoJson.h>

// ============================================
// KONFIGURASI WIFI
// ============================================
const char* WIFI_SSID = "2.4G_5NJ5HfYL";
const char* WIFI_PASSWORD = "N45rqpWL";

// ============================================
// KONFIGURASI THINGSBOARD
// ============================================
const char* THINGSBOARD_SERVER = "demo.thingsboard.io"; // Ganti dengan server Anda
const uint16_t THINGSBOARD_PORT = 1883;
const char* DEVICE_TOKEN = "cyjrv3qx3pnxzkip3ukn"; // Ganti dengan device token

// ============================================
// KONFIGURASI SENSOR
// ============================================
const int SOIL_PIN = 32;           // GPIO34 (ADC1_CH6) - pin analog ESP32
const int READ_INTERVAL = 5000;    // Interval baca data (ms) - 5 detik

// Kalibrasi sensor (sesuaikan dengan sensor Anda)
const int DRY_VALUE = 4095;        // Nilai saat tanah kering (maks ADC 12-bit)
const int WET_VALUE = 1500;        // Nilai saat tanah basah

// ============================================
// VARIABEL GLOBAL
// ============================================
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient);

// ============================================
// FUNGSI KONEKSI WIFI
// ============================================
void initWiFi() {
    Serial.println("Menghubungkan ke WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Terhubung!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nGagal terhubung ke WiFi!");
    }
}

// ============================================
// FUNGSI KONEKSI THINGSBOARD
// ============================================
bool reconnectThingsBoard() {
    if (tb.connected()) {
        return true;
    }
    
    Serial.println("Menghubungkan ke ThingsBoard...");
    if (tb.connect(THINGSBOARD_SERVER, DEVICE_TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Terhubung ke ThingsBoard!");
        return true;
    } else {
        Serial.println("Gagal terhubung ke ThingsBoard!");
        return false;
    }
}

// ============================================
// FUNGSI BACA SENSOR KELEMBABAN TANAH
// ============================================
struct SoilData {
    int rawAnalog;           // Nilai analog mentah (0-4095)
    int moisturePercent;     // Persentase kelembaban (0-100%)
    String status;           // Status: Kering/Lembab/Basah
};

SoilData readSoilMoisture() {
    SoilData data;
    
    // Baca nilai analog (ESP32 ADC 12-bit: 0-4095)
    data.rawAnalog = analogRead(SOIL_PIN);
    
    // Konversi ke persentase (0% = kering, 100% = basah)
    // Map terbalik karena biasanya nilai rendah = basah, tinggi = kering
    data.moisturePercent = map(data.rawAnalog, DRY_VALUE, WET_VALUE, 0, 100);
    
    // Batasi range 0-100
    data.moisturePercent = constrain(data.moisturePercent, 0, 100);
    
    // Tentukan status
    if (data.moisturePercent < 30) {
        data.status = "Kering";
    } else if (data.moisturePercent < 70) {
        data.status = "Lembab";
    } else {
        data.status = "Basah";
    }
    
    return data;
}

// ============================================
// FUNGSI FORMAT TIMESTAMP
// ============================================
String getFormattedTimestamp() {
    // Format: mm/dd/yyyy/00:00
    // Menggunakan millis() sebagai pengganti RTC (untuk demo)
    // Untuk produksi, gunakan NTP atau RTC DS3231
    
    // Jika menggunakan NTP (aktifkan jika perlu):
    /*
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7 untuk Indonesia
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "00/00/0000/00:00";
    }
    
    char buffer[20];
    strftime(buffer, sizeof(buffer), "%m/%d/%Y/%H:%M", &timeinfo);
    return String(buffer);
    */
    
    // Simulasi timestamp untuk demo (atau gunakan epoch time)
    unsigned long epoch = millis() / 1000;
    int minutes = (epoch / 60) % 60;
    int hours = (epoch / 3600) % 24;
    int days = (epoch / 86400) % 30 + 1;
    int months = ((epoch / 86400) / 30) % 12 + 1;
    int years = 2024 + ((epoch / 86400) / 365);
    
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d/%02d:%02d", 
             months, days, years, hours, minutes);
    
    return String(buffer);
}

// ============================================
// FUNGSI KIRIM DATA KE THINGSBOARD
// ============================================
void sendToThingsBoard(SoilData& data) {
    if (!tb.connected()) {
        if (!reconnectThingsBoard()) {
            return;
        }
    }
    
    String timestamp = getFormattedTimestamp();
    
    // Buat JSON payload
    StaticJsonDocument<512> doc;
    
    // Data utama kelembaban tanah
    doc["timelapse"] = timestamp;
    doc["kel tanah"] = data.moisturePercent;                       // Format: mm/dd/yyyy/00:00
    doc["raw_analog"] = data.rawAnalog;                // Output analog mentah
    doc["status"] = data.status;                        // Status: Kering/Lembab/Basah
    
    // Metadata tambahan
    doc["device_id"] = "ESP32_SoilSensor_01";
    
    // Serialize dan kirim
    char payload[512];
    serializeJson(doc, payload);
    
    Serial.println("Mengirim data ke ThingsBoard:");
    Serial.println(payload);
    
    // Kirim sebagai telemetry
    bool success = tb.sendTelemetryJson(payload);
    
    if (success) {
        Serial.println("Data berhasil dikirim!");
    } else {
        Serial.println("Gagal mengirim data!");
    }
    
    // Proses loop ThingsBoard (untuk keep-alive dan callback)
    tb.loop();
}

// ============================================
// FUNGSI SETUP
// ============================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("ESP32 Soil Moisture Sensor");
    Serial.println("ThingsBoard Integration");
    Serial.println("========================================");
    
    // Inisialisasi pin sensor
    pinMode(SOIL_PIN, INPUT);
    analogReadResolution(12);  // Set ADC 12-bit (0-4095)
    analogSetAttenuation(ADC_11db);  // Range 0-3.3V
    
    // Koneksi WiFi
    initWiFi();
    
    // Koneksi ThingsBoard
    reconnectThingsBoard();
    
    Serial.println("Setup selesai. Memulai pembacaan sensor...");
    Serial.println("========================================\n");
}

// ============================================
// FUNGSI LOOP UTAMA
// ============================================
void loop() {
    // Cek koneksi WiFi
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi terputus! Mencoba reconnect...");
        initWiFi();
    }
    
    // Cek koneksi ThingsBoard
    if (!tb.connected()) {
        reconnectThingsBoard();
    }
    
    // Baca data sensor
    SoilData soilData = readSoilMoisture();
    
    // Tampilkan di Serial Monitor
    Serial.println("----------------------------------------");
    Serial.print("Timestamp: ");
    Serial.println(getFormattedTimestamp());
    Serial.print("Raw Analog: ");
    Serial.println(soilData.rawAnalog);
    Serial.print("Kelembaban Tanah: ");
    Serial.print(soilData.moisturePercent);
    Serial.println("%");
    Serial.print("Status: ");
    Serial.println(soilData.status);
    Serial.println("----------------------------------------\n");
    
    // Kirim ke ThingsBoard
    sendToThingsBoard(soilData);
    
    // Loop ThingsBoard
    tb.loop();
    
    // Delay sesuai interval
    delay(READ_INTERVAL);
}