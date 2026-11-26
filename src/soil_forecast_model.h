
#pragma once
#include <math.h>
#include <stdint.h>

#define SOIL_DEPLOY_MODE "MLR"
#define SOIL_MODEL_NAME  "MLR_V7_pearson"

static const int   SOIL_HORIZON_MIN        = 30;
static const int   SOIL_SAMPLE_INTERVAL_S  = 300;
static const int   SOIL_BUF_LEN            = 1;
static const float SOIL_RAIN_THRESHOLD     = 2800.0f;
static const float SOIL_ALERT_THRESHOLD    = 30.0f;
static const float SOIL_INV_V_SLOPE        = 38.3538f;
static const float SOIL_INV_V_INTERCEPT    = -2.9418f;
static const float SOIL_V_REF              = 3.3f;
static const float SOIL_ADC_MAX            = 4095.0f;
static const float SOIL_MIN_PCT            = 10.0f;
static const float SOIL_MAX_PCT            = 80.0f;
static const int   SOIL_N_FEATURES         = 2;
static const char* const SOIL_FEATURE_NAMES[2] = { "soil_now", "rain_now" };
static const float SOIL_INTERCEPT          = 0.878188828f;
static const float SOIL_COEF[2]     = { 0.967562483f, 2.64888645f };

class SoilForecaster {
 public:
  SoilForecaster() : _count(0) {
    for (int i = 0; i < SOIL_BUF_LEN; i++) { _soil[i] = 0.0f; _rainB[i] = 0.0f; }
  }

  static float adcToSoilMoisture(float adc) {
    if (adc <= 0.0f) return SOIL_MIN_PCT;
    float invV = SOIL_ADC_MAX / (adc * SOIL_V_REF);
    float pct = SOIL_INV_V_SLOPE * invV + SOIL_INV_V_INTERCEPT;
    if (pct < SOIL_MIN_PCT) pct = SOIL_MIN_PCT;
    if (pct > SOIL_MAX_PCT) pct = SOIL_MAX_PCT;
    return pct;
  }

  void push(float soilPct, float rainRaw) {
    for (int i = SOIL_BUF_LEN - 1; i > 0; i--) {
      _soil[i] = _soil[i - 1]; _rainB[i] = _rainB[i - 1];
    }
    _soil[0]  = soilPct;
    _rainB[0] = (rainRaw < SOIL_RAIN_THRESHOLD) ? 1.0f : 0.0f;
    if (_count < SOIL_BUF_LEN) _count++;
  }

  bool ready() const { return _count >= SOIL_BUF_LEN; }

  float forecast(float temperature, float humidity, float lux, float hourFrac) const {
    (void)temperature; (void)humidity; (void)lux; (void)hourFrac;
    const float x[SOIL_N_FEATURES] = {
      _soil[0]  /* soil_now */,
      _rainB[0]  /* rain_now */
    };
    float y = SOIL_INTERCEPT;
    for (int i = 0; i < SOIL_N_FEATURES; i++) y += SOIL_COEF[i] * x[i];
    if (y < SOIL_MIN_PCT) y = SOIL_MIN_PCT;
    if (y > SOIL_MAX_PCT) y = SOIL_MAX_PCT;
    return y;
  }

  bool alert(float forecastValue) const { return forecastValue < SOIL_ALERT_THRESHOLD; }

 private:
  float _soil[SOIL_BUF_LEN];
  float _rainB[SOIL_BUF_LEN];
  int   _count;
};
