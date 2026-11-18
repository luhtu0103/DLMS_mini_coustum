/*
 * DLMS Mini - ESP32-S3-N16R8 Bi-Amp/Tri-Amp Firmware V7
 * Config: OUT1=MH-R, OUT2=MH-L, OUT3=Low(Stereo), OUT4=Sub(Stereo)
 * Fix: BLE Notify, FFT Interleaved, Clipping Protection, NVS Save
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>          // Wajib untuk BLE Notify stabil
#include "driver/i2s_std.h"
#include <math.h>
#include "esp_dsp.h"
#include <Preferences.h>      // Simpan settingan ke memori internal

// ==== UUID BLE (Sama dengan HTML) ====
#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID    "0000ffe1-0000-1000-8000-00805f9b34fb"

// ==== PIN I2S (PCM1808 + 2x PCM5102) ====
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

// ==== KONFIGURASI DSP ====
#define SAMPLE_RATE 48000.0f
#define FFT_SIZE 256

typedef struct {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
} biquad_t;

typedef struct {
  bool hpf_on; int hpf_slope; float hpf_freq;
  bool lpf_on; int lpf_slope; float lpf_freq;
  float peq_freq[3]; float peq_gain[3];
  float gain_db; bool is_mute; bool phase_180;
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
Preferences preferences;

// ==== VARIABEL FFT & BLE ====
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

// ==========================================
// FUNGSI MATEMATIKA FILTER (BIQUAD)
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
  float y = (f->b0 * input) + (f->b1 * f->x1) + (f->b2 * f->x2) - (f->a1 * f->y1) - (f->a2 * f->y2);
  f->x2 = f->x1; f->x1 = input; 
  f->y2 = f->y1; f->y1 = y; 
  return y;
}

// PROSES OUTPUT CHANNEL (Optimasi untuk Bi-Amp)
float processOutputChannel(OutputChannel *out, float sigL, float sigR, float master_linear) {
  float o = 0;
  
  // LOGIKA ROUTING BARU UNTUK BI-AMP/TRI-AMP
  // OUT 1 & 2 dipaksa Stereo murni berdasarkan index channel
  // OUT 3 & 4 dipaksa Mono Sum (L+R)/2
  
  if (out == &out1) o = sigR;       // OUT 1 = Mid-High R
  else if (out == &out2) o = sigL;  // OUT 2 = Mid-High L
  else o = (sigL + sigR) * 0.5f;    // OUT 3 & 4 = Low/Sub Stereo Sum
  
  o *= pow(10.0, out->gain_db / 20.0);
  if (out->hpf_on) { o = processBiquad(o, &out->hpf); if (out->hpf_slope == 24) o = processBiquad(o, &out->hpf); }
  if (out->lpf_on) { o = processBiquad(o, &out->lpf); if (out->lpf_slope == 24) o = processBiquad(o, &out->lpf); }
  o = processBiquad(o, &out->peq[0]); 
  o = processBiquad(o, &out->peq[1]); 
  o = processBiquad(o, &out->peq[2]);
  
  if (out->phase_180) o = -o;
  if (out->limiter_on) { 
    float lim = pow(10.0, out->limiter_thresh / 20.0); 
    if (o > lim) o = lim; 
    if (o < -lim) o = -lim; 
  }
  o *= master_linear; 
  if (out->is_mute) o = 0; 
  
  // FIX: Cegah clipping digital yang merusak speaker/DAC
  return constrain(o, -1.0f, 1.0f); 
}

// ==========================================
// FITUR SIMPAN & MUAT DARI NVS (EEPROM)
// ==========================================
void saveSettings() {
  preferences.begin("dlms_v7", false);
  preferences.putFloat("m_vol", master_volume);
  preferences.putBool("inL_mute", inL.is_mute); preferences.putFloat("inL_gain", inL.gain_db);
  preferences.putFloat("inL_peq_f", inL.peq_freq); preferences.putFloat("inL_peq_g", inL.peq_gain);
  
  for(int i=1; i<=4; i++) {
    OutputChannel *out = (i==1)?&out1:(i==2)?&out2:(i==3)?&out3:&out4;
    String pfx = "o"+String(i)+"_";
    preferences.putBool((pfx+"mute").c_str(), out->is_mute);
    preferences.putFloat((pfx+"gain").c_str(), out->gain_db);
    preferences.putFloat((pfx+"hpf_f").c_str(), out->hpf_freq);
    preferences.putBool((pfx+"hpf_on").c_str(), out->hpf_on);
    preferences.putFloat((pfx+"lpf_f").c_str(), out->lpf_freq);
    preferences.putBool((pfx+"lpf_on").c_str(), out->lpf_on);
  }
  preferences.end();
  Serial.println("💾 Settingan disimpan ke NVS!");
}

void loadSettings() {
  preferences.begin("dlms_v7", true);
  if (preferences.isKey("m_vol")) {
    master_volume = preferences.getFloat("m_vol", 0.0);
    inL.is_mute = preferences.getBool("inL_mute", false);
    inL.gain_db = preferences.getFloat("inL_gain", 0.0);
    inL.peq_freq = preferences.getFloat("inL_peq_f", 1000.0);
    inL.peq_gain = preferences.getFloat("inL_peq_g", 0.0);
    calcBiquadPEQ(&inL.peq, inL.peq_freq, inL.peq_gain, 1.0);

    for(int i=1; i<=4; i++) {
      OutputChannel *out = (i==1)?&out1:(i==2)?&out2:(i==3)?&out3:&out4;
      String pfx = "o"+String(i)+"_";
      out->is_mute = preferences.getBool((pfx+"mute").c_str(), false);
      out->gain_db = preferences.getFloat((pfx+"gain").c_str(), 0.0);
      
      out->hpf_freq = preferences.getFloat((pfx+"hpf_f").c_str(), 20.0);
      out->hpf_on = preferences.getBool((pfx+"hpf_on").c_str(), false);
      calcBiquadHighpass(&out->hpf, out->hpf_freq, 0.707);
      
      out->lpf_freq = preferences.getFloat((pfx+"lpf_f").c_str(), 20000.0);
      out->lpf_on = preferences.getBool((pfx+"lpf_on").c_str(), false);
      calcBiquadLowpass(&out->lpf, out->lpf_freq, 0.707);
    }
    Serial.println("📂 Settingan dimuat dari NVS!");
  } else {
    Serial.println("⚠️ Tidak ada settingan tersimpan, menggunakan default.");
  }
  preferences.end();
}

// ==========================================
// CALLBACK BLE
// ==========================================
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; Serial.println("✅ HP Terhubung"); }
  void onDisconnect(BLEServer* pServer) { deviceConnected = false; pServer->startAdvertising(); Serial.println("❌ HP Terputus"); }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String dataMasuk = pCharacteristic->getValue().c_str();
    if (dataMasuk.length() > 0) {
      int posisiKoma = dataMasuk.indexOf(',');
      if (posisiKoma > 0) {
        String perintah = dataMasuk.substring(0, posisiKoma);
        float nilai = dataMasuk.substring(posisiKoma + 1).toFloat();
        
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
        else if (perintah.startsWith("OUT")) {
          int ch = perintah.substring(3,4).toInt();
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
          else if (param == "LIMIT_ON") out->limiter_on = (nilai > 0);
          else if (param == "LIMIT") out->limiter_thresh = nilai; 
          else if (param == "MUTE") out->is_mute = (nilai > 0);
          else if (param == "PHASE") out->phase_180 = (nilai > 0);
        }
        else if (perintah == "MASTER") { master_volume = nilai; }
        else if (perintah == "SAVE_TO_NVS") { saveSettings(); }
        else if (perintah == "LOAD_FROM_NVS") { loadSettings(); }
      }
    }
  }
};

// ==========================================
// FUNGSI FFT (SPECTRUM ANALYZER)
// ==========================================
void processFFT() {
  // FIX: FFT ESP-DSP membutuhkan format interleaved [Real, Imag, Real, Imag, ...]
  for (int i = 0; i < FFT_SIZE; i++) { 
    fft_input[i * 2] = fft_buffer[i] * window[i]; // Bagian Real
    fft_input[i * 2 + 1] = 0.0f;                  // Bagian Imaginer
  }
  dsps_fft2r_fc32(fft_input, FFT_SIZE);
  dsps_bit_rev_fc32(fft_input, FFT_SIZE);
  
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
// TASK 1: PEMROSESAN AUDIO (CORE 1)
// ==========================================
void audioProcessingTask(void *parameter) {
  int16_t rx_buf[512 * 2];
  int16_t tx0_buf[512 * 2];
  int16_t tx1_buf[512 * 2];
  size_t bytes_read, bytes_written;
  
  while (true) {
    i2s_channel_read(rx_handle, rx_buf, sizeof(rx_buf), &bytes_read, portMAX_DELAY);
    int samples = bytes_read / sizeof(int16_t);
    float master_linear = pow(10.0, master_volume / 20.0);
    
    int64_t sumL = 0, sumR = 0;
    for (int i = 0; i < samples; i += 2) {
      sumL += (int64_t)rx_buf[i] * rx_buf[i];
      sumR += (int64_t)rx_buf[i+1] * rx_buf[i+1];
    }
    float rmsL = sqrt((float)sumL / (samples/2)) / 32768.0f;
    float rmsR = sqrt((float)sumR / (samples/2)) / 32768.0f;
    g_levelL = (rmsL > 0.001) ? constrain((int)(20 * log10(rmsL) + 60), 0, 100) : 0;
    g_levelR = (rmsR > 0.001) ? constrain((int)(20 * log10(rmsR) + 60), 0, 100) : 0;
    
    for(int j=0; j<50; j++) {
      int idx = j * (samples / 50);
      int16_t sample = rx_buf[idx];
      osc_buffer[j] = map(sample, -32768, 32767, 100, 0);
    }
    
    int64_t sumO1 = 0, sumO2 = 0, sumO3 = 0, sumO4 = 0;
    for (int i = 0; i < samples; i += 2) {
      float rawL = rx_buf[i] / 32768.0f;
      float rawR = rx_buf[i+1] / 32768.0f;
      
      if (fft_sample_count < FFT_SIZE) {
        fft_buffer[fft_sample_count] = (rawL + rawR) * 0.5f;
        fft_sample_count++;
        if (fft_sample_count == FFT_SIZE) { processFFT(); fft_sample_count = 0; }
      }
      
      float sampL = inL.is_mute ? 0 : (rawL * pow(10.0, inL.gain_db / 20.0));
      float sampR = inR.is_mute ? 0 : (rawR * pow(10.0, inR.gain_db / 20.0));
      sampL = processBiquad(sampL, &inL.peq);
      sampR = processBiquad(sampR, &inR.peq);
      
      // Pemrosesan sesuai konfigurasi Bi-Amp Anda
      float finalO1 = processOutputChannel(&out1, sampL, sampR, master_linear); // MH-R
      float finalO2 = processOutputChannel(&out2, sampL, sampR, master_linear); // MH-L
      float finalO3 = processOutputChannel(&out3, sampL, sampR, master_linear); // Low Stereo
      float finalO4 = processOutputChannel(&out4, sampL, sampR, master_linear); // Sub Stereo
      
      // FIX: Constrain untuk mencegah overflow integer saat dikonversi ke int16_t
      tx0_buf[i]   = (int16_t)constrain(finalO1 * 32767.0f, -32768.0f, 32767.0f);
      tx0_buf[i+1] = (int16_t)constrain(finalO2 * 32767.0f, -32768.0f, 32767.0f);
      tx1_buf[i]   = (int16_t)constrain(finalO3 * 32767.0f, -32768.0f, 32767.0f);
      tx1_buf[i+1] = (int16_t)constrain(finalO4 * 32767.0f, -32768.0f, 32767.0f);
      
      sumO1 += (int64_t)tx0_buf[i] * tx0_buf[i];
      sumO2 += (int64_t)tx0_buf[i+1] * tx0_buf[i+1];
      sumO3 += (int64_t)tx1_buf[i] * tx1_buf[i];
      sumO4 += (int64_t)tx1_buf[i+1] * tx1_buf[i+1];
    }
    
    float rmsO1 = sqrt((float)sumO1 / (samples/2)) / 32768.0f;
    float rmsO2 = sqrt((float)sumO2 / (samples/2)) / 32768.0f;
    float rmsO3 = sqrt((float)sumO3 / (samples/2)) / 32768.0f;
    float rmsO4 = sqrt((float)sumO4 / (samples/2)) / 32768.0f;
    
    g_levelOut1 = (rmsO1 > 0.001) ? constrain((int)(20 * log10(rmsO1) + 60), 0, 100) : 0;
    g_levelOut2 = (rmsO2 > 0.001) ? constrain((int)(20 * log10(rmsO2) + 60), 0, 100) : 0;
    g_levelOut3 = (rmsO3 > 0.001) ? constrain((int)(20 * log10(rmsO3) + 60), 0, 100) : 0;
    g_levelOut4 = (rmsO4 > 0.001) ? constrain((int)(20 * log10(rmsO4) + 60), 0, 100) : 0;
    
    i2s_channel_write(tx0_handle, tx0_buf, bytes_read, &bytes_written, portMAX_DELAY);
    i2s_channel_write(tx1_handle, tx1_buf, bytes_read, &bytes_written, portMAX_DELAY);
  }
}

// ==========================================
// TASK 2: PENGIRIM DATA VISUAL (CORE 0)
// ==========================================
void bleTxTask(void *parameter) {
  char buf[256];
  while (true) {
    if (deviceConnected && g_levelChar) {
      snprintf(buf, sizeof(buf), "LEVEL:IN_L:%d", g_levelL); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      snprintf(buf, sizeof(buf), "LEVEL:IN_R:%d", g_levelR); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      snprintf(buf, sizeof(buf), "LEVEL:OUT1:%d", g_levelOut1); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      snprintf(buf, sizeof(buf), "LEVEL:OUT2:%d", g_levelOut2); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      snprintf(buf, sizeof(buf), "LEVEL:OUT3:%d", g_levelOut3); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      snprintf(buf, sizeof(buf), "LEVEL:OUT4:%d", g_levelOut4); g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      snprintf(buf, sizeof(buf), "SPECTRUM:%d,%d,%d,%d,%d,%d,%d", spectrum[0], spectrum[1], spectrum[2], spectrum[3], spectrum[4], spectrum[5], spectrum[6]);
      g_levelChar->setValue(buf); g_levelChar->notify(); vTaskDelay(2 / portTICK_PERIOD_MS);
      
      // FIX: Buffer diperbesar & pakai snprintf agar tidak crash (Buffer Overflow)
      char osc_str[300] = "OSC:";
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
// SETUP UTAMA
// ==========================================
void setup() {
  Serial.begin(115200);
  
  fft_buffer = (float*)malloc(sizeof(float) * FFT_SIZE);
  fft_input = (float*)malloc(sizeof(float) * FFT_SIZE * 2);
  window = (float*)malloc(sizeof(float) * FFT_SIZE);
  dsps_wind_hann_f32(window, FFT_SIZE);
  dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  
  // Init I2S0 (RX ADC + TX DAC1)
  i2s_chan_config_t chan_cfg_0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg_0, &tx0_handle, &rx_handle);
  i2s_std_config_t std_cfg_0 = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)I2S0_BCK, .ws = (gpio_num_t)I2S0_WS, .dout = (gpio_num_t)I2S0_DOUT, .din = (gpio_num_t)I2S0_DIN },
  };
  i2s_channel_init_std_mode(rx_handle, &std_cfg_0); i2s_channel_init_std_mode(tx0_handle, &std_cfg_0);
  i2s_channel_enable(rx_handle); i2s_channel_enable(tx0_handle);
  
  // Init I2S1 (TX DAC2)
  i2s_chan_config_t chan_cfg_1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg_1, &tx1_handle, NULL);
  i2s_std_config_t std_cfg_1 = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)I2S1_BCK, .ws = (gpio_num_t)I2S1_WS, .dout = (gpio_num_t)I2S1_DOUT, .din = I2S_GPIO_UNUSED },
  };
  i2s_channel_init_std_mode(tx1_handle, &std_cfg_1); i2s_channel_enable(tx1_handle);
  
  // Init BLE
  BLEDevice::init("DLMS_CUSTOM");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  
  // FIX: Wajib ditambahkan agar iOS & Android bisa menerima notifikasi BLE
  pCharacteristic->addDescriptor(new BLE2902()); 
  pCharacteristic->setCallbacks(new MyCallbacks());
  g_levelChar = pCharacteristic;
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  
  // Init Filter Default
  for(int i=1; i<=4; i++) {
    OutputChannel *out = (i==1)?&out1:(i==2)?&out2:(i==3)?&out3:&out4;
    calcBiquadHighpass(&out->hpf, 20, 0.707); calcBiquadLowpass(&out->lpf, 20000, 0.707);
    for(int j=0; j<3; j++) calcBiquadPEQ(&out->peq[j], 1000, 0, 1.0);
  }
  calcBiquadPEQ(&inL.peq, 1000, 0, 1.0); calcBiquadPEQ(&inR.peq, 1000, 0, 1.0);
  
  // MUAT SETTINGAN DARI MEMORI
  loadSettings();
  
  xTaskCreatePinnedToCore(audioProcessingTask, "AudioTask", 16384, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(bleTxTask, "BleTxTask", 8192, NULL, 3, NULL, 0);
  
  Serial.println("🚀 Firmware V7 Bi-Amp Siap! Sistem telah diaudit & dioptimasi.");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
