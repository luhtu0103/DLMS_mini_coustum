/*
 * DLMS Mini - ESP32-S3-N16R8 Firmware V11 (ULTIMATE)
 * 100% Synced with index.html Final V20
 * Fitur: Signal Gen, NVS Save/Load, Compressor, Delay, Global Bypass, FFT, BLE Stabil
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLESecurity.h>
#include <BLE2902.h>
#include "driver/i2s_std.h"
#include <math.h>
#include "esp_dsp.h"
#include <Preferences.h>

#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID    "0000ffe1-0000-1000-8000-00805f9b34fb"

#define I2S0_BCK  4
#define I2S0_WS   5
#define I2S0_DIN  6
#define I2S0_DOUT 7
#define I2S1_BCK  15
#define I2S1_WS   16
#define I2S1_DOUT 17

i2s_chan_handle_t rx_handle = NULL;
i2s_chan_handle_t tx0_handle = NULL;
i2s_chan_handle_t tx1_handle = NULL;

#define SAMPLE_RATE 48000.0f
#define FFT_SIZE 256
#define MAX_DELAY_SAMPLES 512

typedef struct {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
} biquad_t;

typedef struct {
  bool hpf_on; int hpf_slope; float hpf_freq;
  bool lpf_on; int lpf_slope; float lpf_freq;
  float peq_freq[3]; float peq_gain[3];
  float gain_db; float gain_linear; float delay_ms; bool is_mute; bool phase_180;
  int routing; bool limiter_on; float limiter_thresh; float limiter_linear;
  bool comp_on; float comp_thresh; float comp_ratio;
  float comp_attack_coeff; float comp_release_coeff; float comp_envelope;
  biquad_t hpf; biquad_t lpf; biquad_t peq[3];
} OutputChannel;

OutputChannel out1, out2, out3, out4;

typedef struct {
  float gain_db; bool is_mute;
  float peq_freq; float peq_gain; float gain_linear;
  biquad_t peq;
} InputChannel;

InputChannel inL, inR;
float master_volume = 0.0f;
float master_linear_cached = 1.0f;
portMUX_TYPE dspCoeffMux = portMUX_INITIALIZER_UNLOCKED;
bool global_bypass = false;
Preferences preferences;

float delay_line[4][MAX_DELAY_SAMPLES];
int delay_write_pos[4] = {0, 0, 0, 0};

volatile int gen_type = 0;
float gen_freq = 1000.0;
volatile float gen_level_db = -12.0f;
volatile float gen_level_linear = 0.25118864f;
float gen_phase = 0.0;
float pink_b0=0, pink_b1=0, pink_b2=0, pink_b3=0, pink_b4=0, pink_b5=0, pink_b6=0;

bool deviceConnected = false;
BLECharacteristic *g_levelChar = nullptr;
float *fft_buffer = NULL;
float *window = NULL;
float *fft_input = NULL;
volatile int spectrum[7] = {0,0,0,0,0,0,0};
volatile int g_levelL = 0, g_levelR = 0;
volatile int g_levelOut1 = 0, g_levelOut2 = 0, g_levelOut3 = 0, g_levelOut4 = 0;
volatile int osc_buffer[50] = {0};
volatile int osc_buffer_out1[50] = {0};
volatile int osc_buffer_out2[50] = {0};
volatile int osc_buffer_out3[50] = {0};
volatile int osc_buffer_out4[50] = {0};
int fft_sample_count = 0;

// ==========================================
// FUNGSI FILTER & DSP
// ==========================================
void publishBiquadCoefficients(biquad_t *dst, const biquad_t *src) {
  portENTER_CRITICAL(&dspCoeffMux);
  dst->b0=src->b0; dst->b1=src->b1; dst->b2=src->b2; dst->a1=src->a1; dst->a2=src->a2;
  portEXIT_CRITICAL(&dspCoeffMux);
}
void calcBiquadHighpass(biquad_t *f, float freq, float Q) {
  if(freq<20.0f)freq=20.0f;
  const float w0=2.0f*(float)M_PI*freq/SAMPLE_RATE, alpha=sinf(w0)/(2.0f*Q), cw=cosf(w0), a0=1.0f+alpha;
  biquad_t n={}; n.b0=((1.0f+cw)/2.0f)/a0; n.b1=-(1.0f+cw)/a0; n.b2=((1.0f+cw)/2.0f)/a0; n.a1=(-2.0f*cw)/a0; n.a2=(1.0f-alpha)/a0;
  publishBiquadCoefficients(f,&n);
}
void calcBiquadLowpass(biquad_t *f, float freq, float Q) {
  if(freq<20.0f)freq=20.0f; if(freq>20000.0f)freq=20000.0f;
  const float w0=2.0f*(float)M_PI*freq/SAMPLE_RATE, alpha=sinf(w0)/(2.0f*Q), cw=cosf(w0), a0=1.0f+alpha;
  biquad_t n={}; n.b0=((1.0f-cw)/2.0f)/a0; n.b1=(1.0f-cw)/a0; n.b2=((1.0f-cw)/2.0f)/a0; n.a1=(-2.0f*cw)/a0; n.a2=(1.0f-alpha)/a0;
  publishBiquadCoefficients(f,&n);
}
void calcBiquadPEQ(biquad_t *f, float freq, float gain_db, float Q) {
  if(freq<20.0f)freq=20.0f; if(freq>20000.0f)freq=20000.0f;
  const float A=powf(10.0f,gain_db/40.0f), w0=2.0f*(float)M_PI*freq/SAMPLE_RATE, alpha=sinf(w0)/(2.0f*Q), cw=cosf(w0), a0=1.0f+alpha/A;
  biquad_t n={}; n.b0=(1.0f+alpha*A)/a0; n.b1=(-2.0f*cw)/a0; n.b2=(1.0f-alpha*A)/a0; n.a1=(-2.0f*cw)/a0; n.a2=(1.0f-alpha/A)/a0;
  publishBiquadCoefficients(f,&n);
}

float processBiquad(float input, biquad_t *f) {
  float b0,b1,b2,a1,a2;
  portENTER_CRITICAL(&dspCoeffMux); b0=f->b0; b1=f->b1; b2=f->b2; a1=f->a1; a2=f->a2; portEXIT_CRITICAL(&dspCoeffMux);
  float y = (b0 * input) + (b1 * f->x1) + (b2 * f->x2) - (a1 * f->y1) - (a2 * f->y2);
  f->x2 = f->x1; f->x1 = input; f->y2 = f->y1; f->y1 = y; 
  return y;
}

float processCompressor(float input, OutputChannel *out) {
  if (!out->comp_on) return input;
  float input_db = 20.0f * log10f(fabsf(input) + 1e-6f);
  float gr_db = 0.0f;
  if (input_db > out->comp_thresh) {
    gr_db = (input_db - out->comp_thresh) * (1.0f - (1.0f / out->comp_ratio));
  }
  float coeff = (gr_db > out->comp_envelope) ? out->comp_attack_coeff : out->comp_release_coeff;
  out->comp_envelope = coeff * out->comp_envelope + (1.0f - coeff) * gr_db;
  return input * expf(-out->comp_envelope * 0.11512925465f); // envelope changes per sample; use single-precision exp
}

float processOutputChannel(OutputChannel *out, float sigL, float sigR, float master_linear, int ch_idx) {
  // Routing NONE must be silent, including filter and delay-line tails.
  if (out->routing == 3) return 0.0f;
  float o = 0;
  if (out->routing == 0) o = sigL;
  else if (out->routing == 1) o = sigR;
  else if (out->routing == 2) o = (sigL + sigR) * 0.5f;
  else o = 0.0f; // routing 3 = none/silence
  
  o *= out->gain_linear;
  
  if (!global_bypass) {
    if (out->hpf_on) { o = processBiquad(o, &out->hpf); if (out->hpf_slope == 24) o = processBiquad(o, &out->hpf); }
    if (out->lpf_on) { o = processBiquad(o, &out->lpf); if (out->lpf_slope == 24) o = processBiquad(o, &out->lpf); }
    o = processBiquad(o, &out->peq[0]); o = processBiquad(o, &out->peq[1]); o = processBiquad(o, &out->peq[2]);
    o = processCompressor(o, out);
    if (out->limiter_on) { 
      float lim = out->limiter_linear; 
      if (o > lim) o = lim; if (o < -lim) o = -lim; 
    }
  }
  
  int delay_samples = (int)(out->delay_ms * 48.0f);
  if (delay_samples > 0) {
    if (delay_samples >= MAX_DELAY_SAMPLES) delay_samples = MAX_DELAY_SAMPLES - 1;
    delay_line[ch_idx][delay_write_pos[ch_idx]] = o;
    int read_pos = (delay_write_pos[ch_idx] - delay_samples + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
    o = delay_line[ch_idx][read_pos];
    delay_write_pos[ch_idx] = (delay_write_pos[ch_idx] + 1) % MAX_DELAY_SAMPLES;
  }
  
  if (out->phase_180) o = -o;
  o *= master_linear; 
  if (out->is_mute) o = 0; 
  return constrain(o, -1.0f, 1.0f);
}

// ==========================================
// NVS SAVE / LOAD
// ==========================================
// Guard persisted floating-point values before they reach DSP calculations.
static float safeNvsFloat(float value, float fallback, float minValue, float maxValue) {
  if (!isfinite(value)) value = fallback;
  return fminf(maxValue, fmaxf(minValue, value));
}

void saveToNVS() {
  if (!preferences.begin("dlms_v11", false)) {
    Serial.println("❌ NVS open for write failed");
    return;
  }
  preferences.putFloat("m_vol", master_volume);
  preferences.putBool("bypass", global_bypass);
  for (int i = 1; i <= 4; i++) {
    OutputChannel *out = (i == 1) ? &out1 : (i == 2) ? &out2 : (i == 3) ? &out3 : &out4;
    String p = "o" + String(i) + "_";
    preferences.putBool((p + "mute").c_str(), out->is_mute);
    preferences.putBool((p + "phase").c_str(), out->phase_180);
    preferences.putFloat((p + "gain").c_str(), out->gain_db);
    preferences.putFloat((p + "delay").c_str(), out->delay_ms);
    preferences.putInt((p + "routing").c_str(), out->routing);
    preferences.putBool((p + "lim_on").c_str(), out->limiter_on);
    preferences.putFloat((p + "lim_t").c_str(), out->limiter_thresh);
    preferences.putBool((p + "hpf_on").c_str(), out->hpf_on);
    preferences.putFloat((p + "hpf_f").c_str(), out->hpf_freq);
    preferences.putInt((p + "hpf_slope").c_str(), out->hpf_slope);
    preferences.putBool((p + "lpf_on").c_str(), out->lpf_on);
    preferences.putFloat((p + "lpf_f").c_str(), out->lpf_freq);
    preferences.putInt((p + "lpf_slope").c_str(), out->lpf_slope);
    for (int j = 0; j < 3; j++) {
      String k = String(j + 1);
      preferences.putFloat((p + "peqf" + k).c_str(), out->peq_freq[j]);
      preferences.putFloat((p + "pegg" + k).c_str(), out->peq_gain[j]);
    }
    preferences.putBool((p + "comp_on").c_str(), out->comp_on);
    preferences.putFloat((p + "comp_t").c_str(), out->comp_thresh);
    preferences.putFloat((p + "comp_r").c_str(), out->comp_ratio);
    preferences.putFloat((p + "comp_a").c_str(), out->comp_attack_coeff);
    preferences.putFloat((p + "comp_rel").c_str(), out->comp_release_coeff);
  }
  preferences.putFloat("inL_gain", inL.gain_db);
  preferences.putBool("inL_mute", inL.is_mute);
  preferences.putFloat("inL_peqf", inL.peq_freq);
  preferences.putFloat("inL_peqg", inL.peq_gain);
  preferences.putFloat("inR_gain", inR.gain_db);
  preferences.putBool("inR_mute", inR.is_mute);
  preferences.putFloat("inR_peqf", inR.peq_freq);
  preferences.putFloat("inR_peqg", inR.peq_gain);
  preferences.end();
  Serial.println("💾 Full DSP state saved to NVS");
}

void loadFromNVS() {
  if (!preferences.begin("dlms_v11", true)) {
    Serial.println("❌ NVS open for read failed");
    return;
  }
  if (preferences.isKey("m_vol")) {
    master_volume = safeNvsFloat(preferences.getFloat("m_vol", 0.0f), 0.0f, -60.0f, 0.0f);
    master_linear_cached = powf(10.0f, master_volume / 20.0f);
    global_bypass = preferences.getBool("bypass", false);
    for (int i = 1; i <= 4; i++) {
      OutputChannel *out = (i == 1) ? &out1 : (i == 2) ? &out2 : (i == 3) ? &out3 : &out4;
      String p = "o" + String(i) + "_";
      out->is_mute = preferences.getBool((p + "mute").c_str(), out->is_mute);
      out->phase_180 = preferences.getBool((p + "phase").c_str(), out->phase_180);
      out->gain_db = safeNvsFloat(preferences.getFloat((p + "gain").c_str(), out->gain_db), out->gain_db, -60.0f, 12.0f);
      out->gain_linear = powf(10.0f, out->gain_db / 20.0f);
      out->delay_ms = safeNvsFloat(preferences.getFloat((p + "delay").c_str(), out->delay_ms), out->delay_ms, 0.0f, 10.0f);
      out->routing = constrain(preferences.getInt((p + "routing").c_str(), out->routing), 0, 3);
      out->limiter_on = preferences.getBool((p + "lim_on").c_str(), out->limiter_on);
      out->limiter_thresh = safeNvsFloat(preferences.getFloat((p + "lim_t").c_str(), out->limiter_thresh), out->limiter_thresh, -20.0f, 0.0f);
      out->limiter_linear = powf(10.0f, out->limiter_thresh / 20.0f);
      out->hpf_on = preferences.getBool((p + "hpf_on").c_str(), out->hpf_on);
      out->hpf_freq = safeNvsFloat(preferences.getFloat((p + "hpf_f").c_str(), out->hpf_freq), out->hpf_freq, 20.0f, 20000.0f);
      { int slope = preferences.getInt((p + "hpf_slope").c_str(), out->hpf_slope); out->hpf_slope = (slope == 12) ? 12 : 24; }
      out->lpf_on = preferences.getBool((p + "lpf_on").c_str(), out->lpf_on);
      out->lpf_freq = safeNvsFloat(preferences.getFloat((p + "lpf_f").c_str(), out->lpf_freq), out->lpf_freq, 20.0f, 20000.0f);
      { int slope = preferences.getInt((p + "lpf_slope").c_str(), out->lpf_slope); out->lpf_slope = (slope == 12) ? 12 : 24; }
      for (int j = 0; j < 3; j++) {
        String k = String(j + 1);
        out->peq_freq[j] = safeNvsFloat(preferences.getFloat((p + "peqf" + k).c_str(), out->peq_freq[j]), out->peq_freq[j], 20.0f, 20000.0f);
        out->peq_gain[j] = safeNvsFloat(preferences.getFloat((p + "pegg" + k).c_str(), out->peq_gain[j]), out->peq_gain[j], -15.0f, 15.0f);
        calcBiquadPEQ(&out->peq[j], out->peq_freq[j], out->peq_gain[j], 1.0f);
      }
      calcBiquadHighpass(&out->hpf, out->hpf_freq, 0.707f);
      calcBiquadLowpass(&out->lpf, out->lpf_freq, 0.707f);
      out->comp_on = preferences.getBool((p + "comp_on").c_str(), out->comp_on);
      out->comp_thresh = safeNvsFloat(preferences.getFloat((p + "comp_t").c_str(), out->comp_thresh), out->comp_thresh, -60.0f, 0.0f);
      out->comp_ratio = safeNvsFloat(preferences.getFloat((p + "comp_r").c_str(), out->comp_ratio), out->comp_ratio, 1.0f, 20.0f);
      out->comp_attack_coeff = safeNvsFloat(preferences.getFloat((p + "comp_a").c_str(), out->comp_attack_coeff), out->comp_attack_coeff, 0.0f, 1.0f);
      out->comp_release_coeff = safeNvsFloat(preferences.getFloat((p + "comp_rel").c_str(), out->comp_release_coeff), out->comp_release_coeff, 0.0f, 1.0f);
      out->comp_envelope = 0.0f;
    }
    inL.gain_db = safeNvsFloat(preferences.getFloat("inL_gain", inL.gain_db), inL.gain_db, -20.0f, 20.0f);
    inL.gain_linear = powf(10.0f, inL.gain_db / 20.0f);
    inL.is_mute = preferences.getBool("inL_mute", inL.is_mute);
    inL.peq_freq = safeNvsFloat(preferences.getFloat("inL_peqf", inL.peq_freq), inL.peq_freq, 20.0f, 20000.0f);
    inL.peq_gain = safeNvsFloat(preferences.getFloat("inL_peqg", inL.peq_gain), inL.peq_gain, -20.0f, 20.0f);
    calcBiquadPEQ(&inL.peq, inL.peq_freq, inL.peq_gain, 1.0f);
    inR.gain_db = safeNvsFloat(preferences.getFloat("inR_gain", inR.gain_db), inR.gain_db, -20.0f, 20.0f);
    inR.gain_linear = powf(10.0f, inR.gain_db / 20.0f);
    inR.is_mute = preferences.getBool("inR_mute", inR.is_mute);
    inR.peq_freq = safeNvsFloat(preferences.getFloat("inR_peqf", inR.peq_freq), inR.peq_freq, 20.0f, 20000.0f);
    inR.peq_gain = safeNvsFloat(preferences.getFloat("inR_peqg", inR.peq_gain), inR.peq_gain, -20.0f, 20.0f);
    calcBiquadPEQ(&inR.peq, inR.peq_freq, inR.peq_gain, 1.0f);
    Serial.println("📂 Full DSP state loaded from NVS");
  }
  preferences.end();
}

// ==========================================
// BLE CALLBACKS
// ==========================================
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; Serial.println("✅ BLE Connected"); }
  void onDisconnect(BLEServer* pServer) { deviceConnected = false; gen_type = 0; pServer->startAdvertising(); Serial.println("❌ BLE Disconnected; signal generator stopped"); }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String data = pCharacteristic->getValue().c_str();
    // The web UI appends LF; some BLE stacks/devices deliver CRLF.
    // Strip framing whitespace before splitting and parsing the numeric value.
    data.trim();
    if (data.length() == 0) return;

    int comma = data.indexOf(',');
    if (comma <= 0 || comma >= data.length() - 1) return;

    String cmd = data.substring(0, comma);
    String valueText = data.substring(comma + 1);
    cmd.trim();
    valueText.trim();
    if (cmd.length() == 0 || valueText.length() == 0) return;

    char *endPtr = nullptr;
    float val = strtof(valueText.c_str(), &endPtr);
    // Reject malformed, NaN and infinite numbers before any DSP/state change.
    if (endPtr == valueText.c_str() || *endPtr != '\0' || !isfinite(val)) return;

    if (cmd == "SIG_GEN") { if (val != 0 && val != 1 && val != 2) return; gen_type = (int)val; if(gen_type==2) gen_phase=0.0; }
    else if (cmd == "SIG_FREQ") { if (val < 20 || val > 20000) return; gen_freq = val; gen_phase = 0.0; }
    else if (cmd == "TONE_LEVEL") { if (val < -60 || val > 0) return; gen_level_db = val; gen_level_linear = powf(10.0f, val / 20.0f); }
    else if (cmd == "INFO") {
      if (val != 1) return;
      if (g_levelChar != nullptr) {
        g_levelChar->setValue("INFO:FW=V11;SR=48000");
        g_levelChar->notify();
      }
    }
    else if (cmd == "SAVE_TO_NVS") { if (val == 1) saveToNVS(); }
    else if (cmd == "LOAD_FROM_NVS") { if (val == 1) loadFromNVS(); }
    else if (cmd == "MASTER") { if (val < -60 || val > 0) return; master_volume = val; master_linear_cached = powf(10.0f, val / 20.0f); }
    else if (cmd == "ALL_BYPASS") { global_bypass = (val > 0); }
    else if (cmd.startsWith("IN_")) {
      if ((cmd.endsWith("_GAIN") || cmd.endsWith("_PEQGAIN")) && (val < -20 || val > 20)) return;
      if (cmd.endsWith("_PEQFREQ") && (val < 20 || val > 20000)) return;
      if (cmd.endsWith("_MUTE") && val != 0 && val != 1) return;
      if (cmd == "IN_L_GAIN") { inL.gain_db = val; inL.gain_linear = powf(10.0f, val / 20.0f); }
      else if (cmd == "IN_L_PEQFREQ") { inL.peq_freq = val; calcBiquadPEQ(&inL.peq, val, inL.peq_gain, 1.0); }
      else if (cmd == "IN_L_PEQGAIN") { inL.peq_gain = val; calcBiquadPEQ(&inL.peq, inL.peq_freq, val, 1.0); }
      else if (cmd == "IN_L_MUTE") inL.is_mute = (val > 0);
      else if (cmd == "IN_R_GAIN") { inR.gain_db = val; inR.gain_linear = powf(10.0f, val / 20.0f); }
      else if (cmd == "IN_R_PEQFREQ") { inR.peq_freq = val; calcBiquadPEQ(&inR.peq, val, inR.peq_gain, 1.0); }
      else if (cmd == "IN_R_PEQGAIN") { inR.peq_gain = val; calcBiquadPEQ(&inR.peq, inR.peq_freq, val, 1.0); }
      else if (cmd == "IN_R_MUTE") inR.is_mute = (val > 0);
    }
    else if (cmd.startsWith("OUT")) {
      int ch = cmd.substring(3,4).toInt();
      if (ch < 1 || ch > 4) return;
      OutputChannel *out = (ch==1)?&out1:(ch==2)?&out2:(ch==3)?&out3:&out4;
      String p = cmd.substring(5);
      // Enforce physical/UI ranges on every output parameter.
      if (p == "GAIN" && (val < -60 || val > 12)) return;
      if (p == "DELAY" && (val < 0 || val > 10)) return;
      if ((p.endsWith("FREQ")) && (val < 20 || val > 20000)) return;
      if ((p.startsWith("PEQ") && p.endsWith("GAIN")) && (val < -15 || val > 15)) return;
      if (p == "COMP_THRESH" && (val < -60 || val > 0)) return;
      if (p == "COMP_RATIO" && (val < 1 || val > 20)) return;
      if ((p == "COMP_ATK" && (val < 1 || val > 100)) || (p == "COMP_REL" && (val < 50 || val > 1000))) return;
      if (p == "LIMIT" && (val < -20 || val > 0)) return;
      if ((p == "HPF_ON" || p == "LPF_ON" || p == "COMP_ON" || p == "LIMIT_ON" || p == "MUTE" || p == "PHASE") && val != 0 && val != 1) return;
      if (p == "ROUTING" && (val < 0 || val > 3 || floorf(val) != val)) return;

      if (p == "GAIN") { out->gain_db = val; out->gain_linear = powf(10.0f, val / 20.0f); }
      else if (p == "DELAY") out->delay_ms = val;
      else if (p == "MUTE") out->is_mute = (val > 0);
      else if (p == "PHASE") out->phase_180 = (val > 0);
      else if (p == "HPF_ON") out->hpf_on = (val > 0);
       else if (p == "HPFSLOPE") out->hpf_slope = (val >= 24) ? 24 : 12;
      else if (p == "HPFFREQ") { out->hpf_freq = val; calcBiquadHighpass(&out->hpf, val, 0.707); }
      else if (p == "LPF_ON") out->lpf_on = (val > 0);
       else if (p == "LPFSLOPE") out->lpf_slope = (val >= 24) ? 24 : 12;
      else if (p == "LPFFREQ") { out->lpf_freq = val; calcBiquadLowpass(&out->lpf, val, 0.707); }
      else if (p == "PEQ1FREQ") { out->peq_freq[0] = val; calcBiquadPEQ(&out->peq[0], val, out->peq_gain[0], 1.0); }
      else if (p == "PEQ1GAIN") { out->peq_gain[0] = val; calcBiquadPEQ(&out->peq[0], out->peq_freq[0], val, 1.0); }
      else if (p == "PEQ2FREQ") { out->peq_freq[1] = val; calcBiquadPEQ(&out->peq[1], val, out->peq_gain[1], 1.0); }
      else if (p == "PEQ2GAIN") { out->peq_gain[1] = val; calcBiquadPEQ(&out->peq[1], out->peq_freq[1], val, 1.0); }
      else if (p == "PEQ3FREQ") { out->peq_freq[2] = val; calcBiquadPEQ(&out->peq[2], val, out->peq_gain[2], 1.0); }
      else if (p == "PEQ3GAIN") { out->peq_gain[2] = val; calcBiquadPEQ(&out->peq[2], out->peq_freq[2], val, 1.0); }
      else if (p == "COMP_ON") out->comp_on = (val > 0);
      else if (p == "COMP_THRESH") out->comp_thresh = val;
      else if (p == "COMP_RATIO") out->comp_ratio = val;
      else if (p == "COMP_ATK") out->comp_attack_coeff = expf(-1.0f / (val * 48.0f));
      else if (p == "COMP_REL") out->comp_release_coeff = expf(-1.0f / (val * 48.0f));
      else if (p == "LIMIT_ON") out->limiter_on = (val > 0);
      else if (p == "LIMIT") { out->limiter_thresh = val; out->limiter_linear = powf(10.0f, val / 20.0f); }
       else if (p == "ROUTING") out->routing = constrain((int)val, 0, 3);
    }
    else if (cmd.startsWith("ROUTING_OUT")) {
      int ch = cmd.substring(11,12).toInt();
      if (ch < 1 || ch > 4) return;
      OutputChannel *out = (ch==1)?&out1:(ch==2)?&out2:(ch==3)?&out3:&out4;
      if(cmd.endsWith("_L")) {
          if (val > 0) { if (out->routing == 1) out->routing = 2; else out->routing = 0; }
          else { if (out->routing == 0) out->routing = 3; else if (out->routing == 2) out->routing = 1; }
      }
      if(cmd.endsWith("_R")) {
          if (val > 0) { if (out->routing == 0) out->routing = 2; else out->routing = 1; }
          else { if (out->routing == 1) out->routing = 3; else if (out->routing == 2) out->routing = 0; }
      }
    }
  }
};

// ==========================================
// FFT & AUDIO TASK
// ==========================================
void processFFT() {
  for (int i = 0; i < FFT_SIZE; i++) { fft_input[i*2] = fft_buffer[i] * window[i]; fft_input[i*2+1] = 0.0f; }
  dsps_fft2r_fc32(fft_input, FFT_SIZE);
  dsps_bit_rev_fc32(fft_input, FFT_SIZE);
  int band_bins[8] = {0, 1, 2, 3, 6, 13, 32, 80};
  for (int b = 0; b < 7; b++) {
    float sum = 0; int count = 0;
    for (int i = band_bins[b]; i <= band_bins[b+1] && i < FFT_SIZE/2; i++) {
      sum += sqrt(fft_input[i*2]*fft_input[i*2] + fft_input[i*2+1]*fft_input[i*2+1]); count++;
    }
    float mag = count > 0 ? sum / count : 0;
    spectrum[b] = (mag > 0.001) ? constrain((int)(20 * log10(mag) + 50), 0, 100) : 0;
  }
}

void audioProcessingTask(void *parameter) {
  int16_t rx_buf[512 * 2], tx0_buf[512 * 2], tx1_buf[512 * 2];
  size_t bytes_read, bytes_written;
  
  while (true) {
    const esp_err_t readResult = i2s_channel_read(rx_handle, rx_buf, sizeof(rx_buf), &bytes_read, portMAX_DELAY);
    if (readResult != ESP_OK || bytes_read < 100 * sizeof(int16_t) || bytes_read > sizeof(rx_buf)) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // Process complete stereo frames only; never divide by zero or read past a partial frame.
    int samples = (int)(bytes_read / sizeof(int16_t));
    samples &= ~1;
    if (samples < 100 || samples > (int)(sizeof(rx_buf) / sizeof(rx_buf[0]))) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    const size_t validBytes = (size_t)samples * sizeof(int16_t);
    float master_linear = master_linear_cached;
    
    int64_t sumL = 0, sumR = 0;
    for (int i = 0; i < samples; i += 2) { sumL += (int64_t)rx_buf[i]*rx_buf[i]; sumR += (int64_t)rx_buf[i+1]*rx_buf[i+1]; }
    float rmsL = sqrt((float)sumL / (samples/2)) / 32768.0f;
    float rmsR = sqrt((float)sumR / (samples/2)) / 32768.0f;
    g_levelL = (rmsL > 0.001) ? constrain((int)(20 * log10(rmsL) + 60), 0, 100) : 0;
    g_levelR = (rmsR > 0.001) ? constrain((int)(20 * log10(rmsR) + 60), 0, 100) : 0;
    
    for(int j=0; j<50; j++) osc_buffer[j] = map(rx_buf[j * (samples / 50)], -32768, 32767, 100, 0);
    
    int64_t sumO1=0, sumO2=0, sumO3=0, sumO4=0;
    for (int i = 0; i < samples; i += 2) {
      float rawL = rx_buf[i] / 32768.0f;
      float rawR = rx_buf[i+1] / 32768.0f;
      
      if (gen_type == 1) {
        float white = (random(0, 65535) / 32767.5f) - 1.0f;
        pink_b0 = 0.99886f*pink_b0 + white*0.0555179f; pink_b1 = 0.99332f*pink_b1 + white*0.0750759f;
        pink_b2 = 0.96900f*pink_b2 + white*0.1538520f; pink_b3 = 0.86650f*pink_b3 + white*0.3104856f;
        pink_b4 = 0.55000f*pink_b4 + white*0.5329522f; pink_b5 = -0.7616f*pink_b5 - white*0.0168980f;
        float pink = pink_b0+pink_b1+pink_b2+pink_b3+pink_b4+pink_b5+pink_b6 + white*0.5362f;
        pink_b6 = white * 0.115926f;
        rawL = pink * 0.5f; rawR = pink * 0.5f;
      } else if (gen_type == 2) {
        float sine = sinf(gen_phase) * 0.5f;
        rawL = sine; rawR = sine;
        gen_phase += (2.0f * M_PI * gen_freq) / SAMPLE_RATE;
        if (gen_phase >= 2.0f * M_PI) gen_phase -= 2.0f * M_PI;
      }
      
      if (gen_type != 0) { const float toneScale = gen_level_linear; rawL *= toneScale; rawR *= toneScale; }

      if (fft_sample_count < FFT_SIZE) {
        fft_buffer[fft_sample_count] = (rawL + rawR) * 0.5f;
        if (++fft_sample_count == FFT_SIZE) { processFFT(); fft_sample_count = 0; }
      }
      
      float sampL = inL.is_mute ? 0 : processBiquad(rawL * inL.gain_linear, &inL.peq);
      float sampR = inR.is_mute ? 0 : processBiquad(rawR * inR.gain_linear, &inR.peq);
      
      float o1 = processOutputChannel(&out1, sampL, sampR, master_linear, 0);
      float o2 = processOutputChannel(&out2, sampL, sampR, master_linear, 1);
      float o3 = processOutputChannel(&out3, sampL, sampR, master_linear, 2);
      float o4 = processOutputChannel(&out4, sampL, sampR, master_linear, 3);
      
      tx0_buf[i] = constrain((int16_t)(o1 * 32767.0f), -32768, 32767);
      tx0_buf[i+1] = constrain((int16_t)(o2 * 32767.0f), -32768, 32767);
      tx1_buf[i] = constrain((int16_t)(o3 * 32767.0f), -32768, 32767);
      tx1_buf[i+1] = constrain((int16_t)(o4 * 32767.0f), -32768, 32767);
      
      sumO1 += (int64_t)tx0_buf[i]*tx0_buf[i]; sumO2 += (int64_t)tx0_buf[i+1]*tx0_buf[i+1];
      sumO3 += (int64_t)tx1_buf[i]*tx1_buf[i]; sumO4 += (int64_t)tx1_buf[i+1]*tx1_buf[i+1];
    }
    
    g_levelOut1 = (sqrt((float)sumO1/(samples/2))/32768.0f > 0.001) ? constrain((int)(20*log10(sqrt((float)sumO1/(samples/2))/32768.0f)+60), 0, 100) : 0;
    g_levelOut2 = (sqrt((float)sumO2/(samples/2))/32768.0f > 0.001) ? constrain((int)(20*log10(sqrt((float)sumO2/(samples/2))/32768.0f)+60), 0, 100) : 0;
    g_levelOut3 = (sqrt((float)sumO3/(samples/2))/32768.0f > 0.001) ? constrain((int)(20*log10(sqrt((float)sumO3/(samples/2))/32768.0f)+60), 0, 100) : 0;
    g_levelOut4 = (sqrt((float)sumO4/(samples/2))/32768.0f > 0.001) ? constrain((int)(20*log10(sqrt((float)sumO4/(samples/2))/32768.0f)+60), 0, 100) : 0;

    // Ambil 50 sampel merata dari setiap output stereo setelah DSP selesai.
    // Indeks selalu genap agar pasangan kanal tidak melewati batas buffer.
    if (samples >= 100) {
      for (int j = 0; j < 50; j++) {
        int sampleIndex = ((j * (samples / 2)) / 50) * 2;
        osc_buffer_out1[j] = map(tx0_buf[sampleIndex], -32768, 32767, 100, 0);
        osc_buffer_out2[j] = map(tx0_buf[sampleIndex + 1], -32768, 32767, 100, 0);
        osc_buffer_out3[j] = map(tx1_buf[sampleIndex], -32768, 32767, 100, 0);
        osc_buffer_out4[j] = map(tx1_buf[sampleIndex + 1], -32768, 32767, 100, 0);
      }
    }

    i2s_channel_write(tx0_handle, tx0_buf, validBytes, &bytes_written, portMAX_DELAY);
    i2s_channel_write(tx1_handle, tx1_buf, validBytes, &bytes_written, portMAX_DELAY);
  }
}

void bleTxTask(void *parameter) {
  char buf[256];
  while (true) {
    if (deviceConnected && g_levelChar) {
      snprintf(buf, sizeof(buf), "LEVEL:IN_L:%d", g_levelL); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2);
      snprintf(buf, sizeof(buf), "LEVEL:IN_R:%d", g_levelR); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2);
      snprintf(buf, sizeof(buf), "LEVEL:OUT1:%d", g_levelOut1); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2);
      snprintf(buf, sizeof(buf), "LEVEL:OUT2:%d", g_levelOut2); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2);
      snprintf(buf, sizeof(buf), "LEVEL:OUT3:%d", g_levelOut3); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2);
      snprintf(buf, sizeof(buf), "LEVEL:OUT4:%d", g_levelOut4); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2);
      
      snprintf(buf, sizeof(buf), "SPECTRUM:%d,%d,%d,%d,%d,%d,%d", spectrum[0],spectrum[1],spectrum[2],spectrum[3],spectrum[4],spectrum[5],spectrum[6]);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2);
      
      char osc_str[300] = "OSC:"; int pos = 4;
      for(int j=0; j<50; j++) { pos += snprintf(osc_str+pos, sizeof(osc_str)-pos, "%d,", osc_buffer[j]); if(pos >= (int)sizeof(osc_str)-10) break; }
      if(pos > 4) osc_str[pos-1] = '\0';
      g_levelChar->setValue(osc_str); g_levelChar->notify(); vTaskDelay(2);

      // Kirim empat kanal OSC terpisah agar setiap notifikasi BLE tetap kecil.
      char osc_out_str[256];
      for (int channel = 1; channel <= 4; channel++) {
        const volatile int *data = channel == 1 ? osc_buffer_out1 :
                                   channel == 2 ? osc_buffer_out2 :
                                   channel == 3 ? osc_buffer_out3 : osc_buffer_out4;
        int outPos = snprintf(osc_out_str, sizeof(osc_out_str), "OSC_OUT%d:", channel);
        for (int j = 0; j < 50 && outPos < (int)sizeof(osc_out_str) - 8; j++) {
          outPos += snprintf(osc_out_str + outPos, sizeof(osc_out_str) - outPos,
                             j == 49 ? "%d" : "%d,", data[j]);
        }
        if (outPos > 0 && outPos < (int)sizeof(osc_out_str)) {
          g_levelChar->setValue(osc_out_str);
          g_levelChar->notify();
        }
        vTaskDelay(2);
      }
    }
    vTaskDelay(50);
  }
}

void setup() {
  Serial.begin(115200);
  fft_buffer = (float*)malloc(sizeof(float)*FFT_SIZE);
  fft_input = (float*)malloc(sizeof(float)*FFT_SIZE*2);
  window = (float*)malloc(sizeof(float)*FFT_SIZE);
  dsps_wind_hann_f32(window, FFT_SIZE);
  dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  
  for(int i=0; i<4; i++) { for(int j=0; j<MAX_DELAY_SAMPLES; j++) delay_line[i][j] = 0.0f; delay_write_pos[i] = 0; }
  
  i2s_chan_config_t cfg0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&cfg0, &tx0_handle, &rx_handle);
  i2s_std_config_t std0 = { .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000), .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)I2S0_BCK, .ws = (gpio_num_t)I2S0_WS, .dout = (gpio_num_t)I2S0_DOUT, .din = (gpio_num_t)I2S0_DIN } };
  i2s_channel_init_std_mode(rx_handle, &std0); i2s_channel_init_std_mode(tx0_handle, &std0);
  i2s_channel_enable(rx_handle); i2s_channel_enable(tx0_handle);
  
  i2s_chan_config_t cfg1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  i2s_new_channel(&cfg1, &tx1_handle, NULL);
  i2s_std_config_t std1 = { .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000), .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)I2S1_BCK, .ws = (gpio_num_t)I2S1_WS, .dout = (gpio_num_t)I2S1_DOUT, .din = I2S_GPIO_UNUSED } };
  i2s_channel_init_std_mode(tx1_handle, &std1); i2s_channel_enable(tx1_handle);
  
  BLEDevice::init("DLMS_CUSTOM");
  BLESecurity *bleSecurity = new BLESecurity();
  bleSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  bleSecurity->setCapability(ESP_IO_CAP_OUT);
  bleSecurity->setStaticPIN(482731);
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pChar = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pChar->addDescriptor(new BLE2902());
  pChar->setCallbacks(new MyCallbacks());
  g_levelChar = pChar;
  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  
  for(int i=1; i<=4; i++) {
    OutputChannel *out = (i==1)?&out1:(i==2)?&out2:(i==3)?&out3:&out4;
    out->delay_ms = 0.0f; out->hpf_slope = 24; out->lpf_slope = 24; out->comp_on = false; out->comp_thresh = -20.0f; out->comp_ratio = 4.0f;
    out->gain_db = 0.0f; out->gain_linear = 1.0f; out->limiter_thresh = 0.0f; out->limiter_linear = 1.0f;
    out->comp_attack_coeff = expf(-1.0f / (10.0 * 48.0f)); out->comp_release_coeff = expf(-1.0f / (200.0 * 48.0f));
    calcBiquadHighpass(&out->hpf, 20, 0.707); calcBiquadLowpass(&out->lpf, 20000, 0.707);
    out->peq_freq[0] = 100.0f; out->peq_freq[1] = 1000.0f; out->peq_freq[2] = 5000.0f;
    for(int j=0; j<3; j++) { out->peq_gain[j] = 0.0f; calcBiquadPEQ(&out->peq[j], out->peq_freq[j], out->peq_gain[j], 1.0); }
  }
  inL.gain_db = 0.0f; inL.gain_linear = 1.0f; inL.is_mute = false; inL.peq_freq = 1000.0f; inL.peq_gain = 0.0f;
  inR.gain_db = 0.0f; inR.gain_linear = 1.0f; inR.is_mute = false; inR.peq_freq = 1000.0f; inR.peq_gain = 0.0f;
  calcBiquadPEQ(&inL.peq, inL.peq_freq, inL.peq_gain, 1.0f);
  calcBiquadPEQ(&inR.peq, inR.peq_freq, inR.peq_gain, 1.0f);

  loadFromNVS();
  
  xTaskCreatePinnedToCore(audioProcessingTask, "Audio", 16384, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(bleTxTask, "BleTx", 8192, NULL, 3, NULL, 0);
  Serial.println("🚀 Firmware V11 Ready! 100% Synced with UI.");
}

void loop() { vTaskDelay(1000); }
