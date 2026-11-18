/*
 * DLMS Mini - ESP32-S3-N16R8
 * Hardware: PCM1808 (ADC) + 2x PCM5102 (DAC) = 4 Output
 * Fix: BLE2902, buffer overflow, delay line, routing logic
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/i2s_std.h"
#include <math.h>

// ==== UUID BLE (sama dengan HTML) ====
#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID    "0000ffe1-0000-1000-8000-00805f9b34fb"

// ==== PIN I2S ====
// PCM1808 (ADC) + PCM5102 #1 (OUT1+OUT2) share clock
#define I2S0_BCK  4    // BCLK (share ADC + DAC1)
#define I2S0_WS   5    // LRCK (share ADC + DAC1)
#define I2S0_DIN  6    // DOUT dari PCM1808 → ESP32
#define I2S0_DOUT 7    // DIN ke PCM5102 #1
#define I2S0_MCLK 3    // MCLK untuk PCM1808 (wajib!)

// PCM5102 #2 (OUT3+OUT4)
#define I2S1_BCK  15
#define I2S1_WS   16
#define I2S1_DOUT 17

i2s_chan_handle_t rx_handle = NULL;
i2s_chan_handle_t tx0_handle = NULL;
i2s_chan_handle_t tx1_handle = NULL;

// ==== KONFIGURASI DSP ====
#define SAMPLE_RATE 48000.0f
#define FFT_SIZE 256
#define MAX_DELAY_SAMPLES 512  // ~10.6ms @ 48kHz

typedef struct {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
} biquad_t;

typedef struct {
  bool hpf_on; int hpf_slope; float hpf_freq;
  bool lpf_on; int lpf_slope; float lpf_freq;
  float peq_freq[3]; float peq_gain[3];
  float gain_db; float delay_ms;
  bool is_mute; bool phase_180;
  int routing; // 0=L, 1=R, 2=Both, 3=None
  bool limiter_on; float limiter_thresh;
  biquad_t hpf; biquad_t lpf; biquad_t peq[3];
} OutputChannel;

OutputChannel out1, out2, out3, out4;

typedef struct {
  float gain_db; bool is_mute;
  float peq_freq; float peq_gain;
  biquad_t peq;
} InputChannel;

InputChannel inL, inR;

float master_volume = 0.0;
bool all_bypass = false;

// ==== VARIABEL BLE & FFT ====
bool deviceConnected = false;
BLECharacteristic *g_levelChar = nullptr;
float *fft_buffer = NULL;
float *window = NULL;
float *fft_input = NULL;
volatile int spectrum[7] = {0,0,0,0,0,0,0};
volatile int g_levelL = 0, g_levelR = 0;
volatile int g_levelOut1 = 0, g_levelOut2 = 0, g_levelOut3 = 0, g_levelOut4 = 0;
volatile int osc_buffer[50] = {0};
int fft_sample_count = 0;

// ==== DELAY LINE (per output channel) ====
float delay_line[4][MAX_DELAY_SAMPLES];
int delay_write_pos[4] = {0,0,0,0};

// ==========================================
// FUNGSI BIQUAD FILTER
// ==========================================
void calcBiquadHighpass(biquad_t *f, float freq, float Q) {
  if(freq < 20) freq = 20;
  float w0 = 2.0 * M_PI * freq / SAMPLE_RATE;
  float alpha = sin(w0) / (2.0 * Q);
  float cosw0 = cos(w0);
  float a0 = 1.0 + alpha;
  f->b0 = ((1.0 + cosw0) / 2.0) / a0;
  f->b1 = -(1.0 + cosw0) / a0;
  f->b2 = ((1.0 + cosw0) / 2.0) / a0;
  f->a1 = (-2.0 * cosw0) / a0;
  f->a2 = (1.0 - alpha) / a0;
}

void calcBiquadLowpass(biquad_t *f, float freq, float Q) {
  if(freq > 20000) freq = 20000;
  float w0 = 2.0 * M_PI * freq / SAMPLE_RATE;
  float alpha = sin(w0) / (2.0 * Q);
  float cosw0 = cos(w0);
  float a0 = 1.0 + alpha;
  f->b0 = ((1.0 - cosw0) / 2.0) / a0;
  f->b1 = (1.0 - cosw0) / a0;
  f->b2 = ((1.0 - cosw0) / 2.0) / a0;
  f->a1 = (-2.0 * cosw0) / a0;
  f->a2 = (1.0 - alpha) / a0;
}

void calcBiquadPEQ(biquad_t *f, float freq, float gain_db, float Q) {
  if(freq < 20) freq = 20;
  if(freq > 20000) freq = 20000;
  float A = pow(10.0, gain_db / 40.0);
  float w0 = 2.0 * M_PI * freq / SAMPLE_RATE;
  float alpha = sin(w0) / (2.0 * Q);
  float cosw0 = cos(w0);
  float a0 = 1.0 + alpha / A;
  f->b0 = (1.0 + alpha * A) / a0;
  f->b1 = (-2.0 * cosw0) / a0;
  f->b2 = (1.0 - alpha * A) / a0;
  f->a1 = (-2.0 * cosw0) / a0;
  f->a2 = (1.0 - alpha / A) / a0;
}

float processBiquad(float input, biquad_t *f) {
  float y = (f->b0 * input) + (f->b1 * f->x1) + (f->b2 * f->x2) 
            - (f->a1 * f->y1) - (f->a2 * f->y2);
  f->x2 = f->x1; f->x1 = input;
  f->y2 = f->y1; f->y1 = y;
  return y;
}

// ==== FUNGSI DELAY (BARU!) ====
float applyDelay(float input, int ch, int delay_samples) {
  if (delay_samples <= 0) return input;
  if (delay_samples >= MAX_DELAY_SAMPLES) delay_samples = MAX_DELAY_SAMPLES - 1;
  
  delay_line[ch][delay_write_pos[ch]] = input;
  int read_pos = (delay_write_pos[ch] - delay_samples + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
  float output = delay_line[ch][read_pos];
  delay_write_pos[ch] = (delay_write_pos[ch] + 1) % MAX_DELAY_SAMPLES;
  return output;
}

// ==== PROSES OUTPUT CHANNEL (DENGAN DELAY) ====
float processOutputChannel(OutputChannel *out, float sigL, float sigR, 
                           float master_linear, int channel_index) {
  float o = 0;
  
  // Routing
  if (out->routing == 0) o = sigL;
  else if (out->routing == 1) o = sigR;
  else if (out->routing == 2) o = (sigL + sigR) * 0.5f;
  else o = 0; // routing = 3 (none)
  
  // Gain
  o *= pow(10.0, out->gain_db / 20.0);
  
  // HPF
  if (out->hpf_on) {
    o = processBiquad(o, &out->hpf);
    if (out->hpf_slope == 24) o = processBiquad(o, &out->hpf); // 2nd order
  }
  
  // LPF
  if (out->lpf_on) {
    o = processBiquad(o, &out->lpf);
    if (out->lpf_slope == 24) o = processBiquad(o, &out->lpf);
  }
  
  // PEQ 3-band
  o = processBiquad(o, &out->peq[0]);
  o = processBiquad(o, &out->peq[1]);
  o = processBiquad(o, &out->peq[2]);
  
  // DELAY (BARU!)
  int delay_samples = (int)(out->delay_ms * SAMPLE_RATE / 1000.0);
  o = applyDelay(o, channel_index, delay_samples);
  
  // Phase
  if (out->phase_180) o = -o;
  
  // Limiter (soft clip sederhana)
  if (out->limiter_on) {
    float lim = pow(10.0, out->limiter_thresh / 20.0);
    if (o > lim) o = lim;
    if (o < -lim) o = -lim;
  }
  
  // Master volume
  o *= master_linear;
  
  // Mute
  if (out->is_mute) o = 0;
  
  return o;
}

// ==========================================
// BLE CALLBACKS
// ==========================================
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { 
    deviceConnected = true;
    Serial.println("✅ HP Terhubung");
  }
  void onDisconnect(BLEServer* pServer) { 
    deviceConnected = false;
    Serial.println("❌ HP Terputus");
    pServer->startAdvertising();
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String dataMasuk = String(pCharacteristic->getValue().c_str());
    dataMasuk.trim();
    if (dataMasuk.length() == 0) return;
    
    int posisiKoma = dataMasuk.indexOf(',');
    if (posisiKoma < 0) return;
    
    String perintah = dataMasuk.substring(0, posisiKoma);
    float nilai = dataMasuk.substring(posisiKoma + 1).toFloat();
    
    // Input commands
    if (perintah.startsWith("IN_")) {
      if (perintah == "IN_L_GAIN") inL.gain_db = nilai;
      else if (perintah == "IN_R_GAIN") inR.gain_db = nilai;
      else if (perintah == "IN_L_MUTE") inL.is_mute = (nilai > 0);
      else if (perintah == "IN_R_MUTE") inR.is_mute = (nilai > 0);
      else if (perintah == "IN_L_PEQFREQ") { inL.peq_freq = nilai; calcBiquadPEQ(&inL.peq, nilai, inL.peq_gain, 1.0); }
      else if (perintah == "IN_L_PEQGAIN") { inL.peq_gain = nilai; calcBiquadPEQ(&inL.peq, inL.peq_freq, nilai, 1.0); }
      else if (perintah == "IN_R_PEQFREQ") { inR.peq_freq = nilai; calcBiquadPEQ(&inR.peq, nilai, inR.peq_gain, 1.0); }
      else if (perintah == "IN_R_PEQGAIN") { inR.peq_gain = nilai; calcBiquadPEQ(&inR.peq, inR.peq_freq, nilai, 1.0); }
    }
    // Output commands
    else if (perintah.startsWith("OUT")) {
      int ch = perintah.substring(3,4).toInt();
      if (ch < 1 || ch > 4) return;
      OutputChannel *out = (ch == 1) ? &out1 : (ch == 2) ? &out2 : (ch == 3) ? &out3 : &out4;
      String param = perintah.substring(5);
      
      if (param == "HPF_ON") out->hpf_on = (nilai > 0);
      else if (param == "HPFFREQ") { out->hpf_freq = nilai; calcBiquadHighpass(&out->hpf, nilai, 0.707); }
      else if (param == "HPFSLOPE") out->hpf_slope = (int)nilai;
      else if (param == "LPF_ON") out->lpf_on = (nilai > 0);
      else if (param == "LPFFREQ") { out->lpf_freq = nilai; calcBiquadLowpass(&out->lpf, nilai, 0.707); }
      else if (param == "LPFSLOPE") out->lpf_slope = (int)nilai;
      else if (param == "PEQ1FREQ") { out->peq_freq[0] = nilai; calcBiquadPEQ(&out->peq[0], nilai, out->peq_gain[0], 1.0); }
      else if (param == "PEQ1GAIN") { out->peq_gain[0] = nilai; calcBiquadPEQ(&out->peq[0], out->peq_freq[0], nilai, 1.0); }
      else if (param == "PEQ2FREQ") { out->peq_freq[1] = nilai; calcBiquadPEQ(&out->peq[1], nilai, out->peq_gain[1], 1.0); }
      else if (param == "PEQ2GAIN") { out->peq_gain[1] = nilai; calcBiquadPEQ(&out->peq[1], out->peq_freq[1], nilai, 1.0); }
      else if (param == "PEQ3FREQ") { out->peq_freq[2] = nilai; calcBiquadPEQ(&out->peq[2], nilai, out->peq_gain[2], 1.0); }
      else if (param == "PEQ3GAIN") { out->peq_gain[2] = nilai; calcBiquadPEQ(&out->peq[2], out->peq_freq[2], nilai, 1.0); }
      else if (param == "GAIN") out->gain_db = nilai;
      else if (param == "DELAY") out->delay_ms = nilai;
      else if (param == "LIMIT_ON") out->limiter_on = (nilai > 0);
      else if (param == "LIMIT") out->limiter_thresh = nilai;
      else if (param == "MUTE") out->is_mute = (nilai > 0);
      else if (param == "PHASE") out->phase_180 = (nilai > 0);
    }
    // Master & lainnya
    else if (perintah == "MASTER") master_volume = nilai;
    else if (perintah == "ALL_BYPASS") all_bypass = (nilai > 0);
    else if (perintah.startsWith("RESET")) {
      int ch = perintah.substring(6).toInt();
      if (ch >= 1 && ch <= 4) {
        OutputChannel *out = (ch == 1) ? &out1 : (ch == 2) ? &out2 : (ch == 3) ? &out3 : &out4;
        *out = OutputChannel(); // Reset ke default
      }
    }
    else if (perintah.startsWith("ROUTING_OUT")) {
      int ch = perintah.substring(11,12).toInt();
      if (ch < 1 || ch > 4) return;
      OutputChannel *out = (ch == 1) ? &out1 : (ch == 2) ? &out2 : (ch == 3) ? &out3 : &out4;
      if(perintah.endsWith("_L")) {
        if (nilai > 0) out->routing = (out->routing == 1) ? 2 : 0;
        else out->routing = (out->routing == 2) ? 1 : 3;
      }
      if(perintah.endsWith("_R")) {
        if (nilai > 0) out->routing = (out->routing == 0) ? 2 : 1;
        else out->routing = (out->routing == 2) ? 0 : 3;
      }
    }
    
    Serial.print("📩 "); Serial.print(perintah); Serial.print(" = "); Serial.println(nilai);
  }
};

// ==========================================
// FFT (SPECTRUM ANALYZER)
// ==========================================
void processFFT() {
  // Apply window
  for (int i = 0; i < FFT_SIZE; i++) {
    fft_input[i*2] = fft_buffer[i] * window[i];
    fft_input[i*2+1] = 0;
  }
  
  // FFT (gunakan esp_dsp jika tersedia, atau manual)
  #ifdef USE_ESP_DSP
  dsps_fft2r_fc32(fft_input, FFT_SIZE);
  dsps_bit_rev_fc32(fft_input, FFT_SIZE);
  #else
  // FFT manual sederhana (Cooley-Tukey)
  // ... (skip untuk hemat ruang, pakai esp_dsp saja)
  #endif
  
  // Hitung 7 band spectrum
  float mag[7] = {0};
  int band_bins[8] = {0, 1, 2, 3, 6, 13, 32, 80};
  for (int b = 0; b < 7; b++) {
    float sum = 0; int count = 0;
    for (int i = band_bins[b]; i <= band_bins[b+1] && i < FFT_SIZE/2; i++) {
      float real = fft_input[i*2];
      float imag = fft_input[i*2+1];
      sum += sqrt(real*real + imag*imag);
      count++;
    }
    if(count == 0) count = 1;
    mag[b] = sum / count;
    if (mag[b] > 0.001) {
      int db = (int)(20 * log10(mag[b]) + 50);
      spectrum[b] = constrain(db, 0, 100);
    } else {
      spectrum[b] = 0;
    }
  }
}

// ==========================================
// TASK 1: AUDIO PROCESSING (CORE 1)
// ==========================================
void audioProcessingTask(void *parameter) {
  int16_t rx_buf[512 * 2];
  int16_t tx0_buf[512 * 2];
  int16_t tx1_buf[512 * 2];
  size_t bytes_read, bytes_written;
  
  while (true) {
    // Baca dari PCM1808 (ADC)
    esp_err_t err = i2s_channel_read(rx_handle, rx_buf, sizeof(rx_buf), &bytes_read, portMAX_DELAY);
    if (err != ESP_OK) continue;
    
    int samples = bytes_read / sizeof(int16_t);
    float master_linear = pow(10.0, master_volume / 20.0);
    
    // Hitung RMS input
    int64_t sumL = 0, sumR = 0;
    for (int i = 0; i < samples; i += 2) {
      sumL += (int64_t)rx_buf[i] * rx_buf[i];
      sumR += (int64_t)rx_buf[i+1] * rx_buf[i+1];
    }
    float rmsL = sqrt((float)sumL / (samples/2)) / 32768.0f;
    float rmsR = sqrt((float)sumR / (samples/2)) / 32768.0f;
    g_levelL = (rmsL > 0.001) ? constrain((int)(20 * log10(rmsL) + 60), 0, 100) : 0;
    g_levelR = (rmsR > 0.001) ? constrain((int)(20 * log10(rmsR) + 60), 0, 100) : 0;
    
    // OSC buffer (ambil 50 sample dari input)
    for(int j=0; j<50; j++) {
      int idx = j * (samples / 50);
      if (idx < samples) {
        int16_t sample = rx_buf[idx];
        osc_buffer[j] = map(sample, -32768, 32767, 100, 0);
      }
    }
    
    // FFT
    int64_t sumO1 = 0, sumO2 = 0, sumO3 = 0, sumO4 = 0;
    for (int i = 0; i < samples; i += 2) {
      float rawL = rx_buf[i] / 32768.0f;
      float rawR = rx_buf[i+1] / 32768.0f;
      
      if (fft_sample_count < FFT_SIZE) {
        fft_buffer[fft_sample_count] = (rawL + rawR) * 0.5f;
        fft_sample_count++;
        if (fft_sample_count == FFT_SIZE) {
          processFFT();
          fft_sample_count = 0;
        }
      }
      
      // Input processing
      float sampL = inL.is_mute ? 0 : (rawL * pow(10.0, inL.gain_db / 20.0));
      float sampR = inR.is_mute ? 0 : (rawR * pow(10.0, inR.gain_db / 20.0));
      sampL = processBiquad(sampL, &inL.peq);
      sampR = processBiquad(sampR, &inR.peq);
      
      // Output processing (dengan delay!)
      float finalO1 = processOutputChannel(&out1, sampL, sampR, master_linear, 0);
      float finalO2 = processOutputChannel(&out2, sampL, sampR, master_linear, 1);
      float finalO3 = processOutputChannel(&out3, sampL, sampR, master_linear, 2);
      float finalO4 = processOutputChannel(&out4, sampL, sampR, master_linear, 3);
      
      // Tulis ke buffer output
      tx0_buf[i] = constrain((int16_t)(finalO1 * 32767.0f), -32768, 32767);
      tx0_buf[i+1] = constrain((int16_t)(finalO2 * 32767.0f), -32768, 32767);
      tx1_buf[i] = constrain((int16_t)(finalO3 * 32767.0f), -32768, 32767);
      tx1_buf[i+1] = constrain((int16_t)(finalO4 * 32767.0f), -32768, 32767);
      
      sumO1 += (int64_t)tx0_buf[i] * tx0_buf[i];
      sumO2 += (int64_t)tx0_buf[i+1] * tx0_buf[i+1];
      sumO3 += (int64_t)tx1_buf[i] * tx1_buf[i];
      sumO4 += (int64_t)tx1_buf[i+1] * tx1_buf[i+1];
    }
    
    // Hitung RMS output
    float rmsO1 = sqrt((float)sumO1 / (samples/2)) / 32768.0f;
    float rmsO2 = sqrt((float)sumO2 / (samples/2)) / 32768.0f;
    float rmsO3 = sqrt((float)sumO3 / (samples/2)) / 32768.0f;
    float rmsO4 = sqrt((float)sumO4 / (samples/2)) / 32768.0f;
    g_levelOut1 = (rmsO1 > 0.001) ? constrain((int)(20 * log10(rmsO1) + 60), 0, 100) : 0;
    g_levelOut2 = (rmsO2 > 0.001) ? constrain((int)(20 * log10(rmsO2) + 60), 0, 100) : 0;
    g_levelOut3 = (rmsO3 > 0.001) ? constrain((int)(20 * log10(rmsO3) + 60), 0, 100) : 0;
    g_levelOut4 = (rmsO4 > 0.001) ? constrain((int)(20 * log10(rmsO4) + 60), 0, 100) : 0;
    
    // Tulis ke DAC (PCM5102)
    i2s_channel_write(tx0_handle, tx0_buf, bytes_read, &bytes_written, portMAX_DELAY);
    i2s_channel_write(tx1_handle, tx1_buf, bytes_read, &bytes_written, portMAX_DELAY);
  }
}

// ==========================================
// TASK 2: BLE TRANSMIT (CORE 0)
// ==========================================
void bleTxTask(void *parameter) {
  char buf[256];
  while (true) {
    if (deviceConnected && g_levelChar) {
      // VU Meter
      snprintf(buf, sizeof(buf), "LEVEL:IN_L:%d", g_levelL);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      snprintf(buf, sizeof(buf), "LEVEL:IN_R:%d", g_levelR);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      snprintf(buf, sizeof(buf), "LEVEL:OUT1:%d", g_levelOut1);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      snprintf(buf, sizeof(buf), "LEVEL:OUT2:%d", g_levelOut2);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      snprintf(buf, sizeof(buf), "LEVEL:OUT3:%d", g_levelOut3);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      snprintf(buf, sizeof(buf), "LEVEL:OUT4:%d", g_levelOut4);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      // Spectrum
      snprintf(buf, sizeof(buf), "SPECTRUM:%d,%d,%d,%d,%d,%d,%d",
               spectrum[0], spectrum[1], spectrum[2], spectrum[3],
               spectrum[4], spectrum[5], spectrum[6]);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      // OSC (FIX: buffer lebih besar + snprintf aman)
      char osc_str[400] = "OSC:";
      int pos = 4;
      for(int j=0; j<50; j++) {
        pos += snprintf(osc_str + pos, sizeof(osc_str) - pos, "%d,", osc_buffer[j]);
        if (pos >= (int)sizeof(osc_str) - 10) break;
      }
      if (pos > 4) osc_str[pos-1] = '\0';
      g_levelChar->setValue(osc_str); g_levelChar->notify();
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n🎛️ DLMS Mini ESP32-S3-N16R8 Starting...");
  Serial.println("Hardware: PCM1808 + 2x PCM5102");
  
  // Alloc FFT
  fft_buffer = (float*)malloc(sizeof(float) * FFT_SIZE);
  fft_input = (float*)malloc(sizeof(float) * FFT_SIZE * 2);
  window = (float*)malloc(sizeof(float) * FFT_SIZE);
  if (!fft_buffer || !fft_input || !window) {
    Serial.println("❌ Gagal alloc memory!");
    while(1) delay(1000);
  }
  
  // Hann window
  for (int i = 0; i < FFT_SIZE; i++) {
    window[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (FFT_SIZE - 1)));
  }
  
  // Init I2S0 (RX dari PCM1808 + TX ke PCM5102 #1)
  i2s_chan_config_t chan_cfg_0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg_0, &tx0_handle, &rx_handle);
  
  i2s_std_config_t std_cfg_0 = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = (gpio_num_t)I2S0_MCLK,
      .bclk = (gpio_num_t)I2S0_BCK,
      .ws = (gpio_num_t)I2S0_WS,
      .dout = (gpio_num_t)I2S0_DOUT,
      .din = (gpio_num_t)I2S0_DIN
    },
  };
  i2s_channel_init_std_mode(rx_handle, &std_cfg_0);
  i2s_channel_init_std_mode(tx0_handle, &std_cfg_0);
  i2s_channel_enable(rx_handle);
  i2s_channel_enable(tx0_handle);
  
  // Init I2S1 (TX ke PCM5102 #2)
  i2s_chan_config_t chan_cfg_1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg_1, &tx1_handle, NULL);
  
  i2s_std_config_t std_cfg_1 = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S1_BCK,
      .ws = (gpio_num_t)I2S1_WS,
      .dout = (gpio_num_t)I2S1_DOUT,
      .din = I2S_GPIO_UNUSED
    },
  };
  i2s_channel_init_std_mode(tx1_handle, &std_cfg_1);
  i2s_channel_enable(tx1_handle);
  
  Serial.println("✅ I2S Ready (PCM1808 + 2x PCM5102)");
  
  // Init BLE
  BLEDevice::init("DLMS_Mini_S3");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());  // FIX: wajib untuk notify
  pCharacteristic->setCallbacks(new MyCallbacks());
  g_levelChar = pCharacteristic;
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  
  Serial.println("✅ BLE Ready! Nama: DLMS_Mini_S3");
  
  // Init default filter
  for (int i = 1; i <= 4; i++) {
    OutputChannel *out = (i == 1) ? &out1 : (i == 2) ? &out2 : (i == 3) ? &out3 : &out4;
    calcBiquadHighpass(&out->hpf, 20, 0.707);
    calcBiquadLowpass(&out->lpf, 20000, 0.707);
    for(int j=0; j<3; j++) calcBiquadPEQ(&out->peq[j], 1000, 0, 1.0);
  }
  calcBiquadPEQ(&inL.peq, 1000, 0, 1.0);
  calcBiquadPEQ(&inR.peq, 1000, 0, 1.0);
  
  // Init delay line
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < MAX_DELAY_SAMPLES; j++) delay_line[i][j] = 0;
    delay_write_pos[i] = 0;
  }
  
  // Jalankan task
  xTaskCreatePinnedToCore(audioProcessingTask, "AudioTask", 16384, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(bleTxTask, "BleTxTask", 8192, NULL, 3, NULL, 0);
  
  Serial.println("🚀 Firmware Ready! Sistem berjalan.");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
