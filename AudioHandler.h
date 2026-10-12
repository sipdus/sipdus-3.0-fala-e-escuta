#ifndef AUDIO_HANDLER_H
#define AUDIO_HANDLER_H

// =============================================================================
// AudioHandler.h - microfone (MAX4466) e alto-falante para a assistente Sip
//
//  - Microfone MAX4466 no GPIO35 (ADC1, funciona com Wi-Fi ligado)
//  - Alto-falante no conector SPEAK da placa, ligado ao DAC do GPIO26
//  - Taxa de audio: 16 kHz, mono (mesma do servidor da Sip)
//
// Como funciona:
//  - Um timer de hardware gera 16000 "ticks" por segundo e acorda uma tarefa
//    (FreeRTOS, nucleo 0). A tarefa le o ADC (gravando) ou escreve no DAC
//    (tocando). Nao bloqueia o loop() nem o LVGL.
//  - Gravacao: amostras PCM 16 bits com sinal vao para um buffer circular que o
//    loop() esvazia com audio_record_read().
//  - Reproducao: bytes PCM 8 bits SEM sinal (como o servidor envia) entram num
//    buffer circular com audio_play_write() e saem pelo DAC.
//  - Um unico buffer de 16 KB e alocado na HEAP (nao na DRAM estatica, que ja
//    esta quase cheia por causa do LVGL).
//
// Nao altera nenhum pino, filtro, BLE ou o sensor MAX30102.
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <math.h>

// ---------------------------- configuracao -----------------------------------
#define AUDIO_MIC_PIN        35        // MAX4466 (ADC1)
#define AUDIO_DAC_PIN        26        // alto-falante (DAC2)
#define AUDIO_SAMPLE_RATE    16000     // Hz
#define AUDIO_BUF_BYTES      16384     // buffer circular (heap)
#define AUDIO_PREBUFFER      4096      // bytes acumulados antes de comecar a tocar
#ifndef AUDIO_TASK_CORE
#define AUDIO_TASK_CORE      1         // nucleo do loop(); o Wi-Fi roda no nucleo 0 e nao atrapalha o ritmo do audio
#endif
#define AUDIO_TASK_PRIO      6
#define AUDIO_TASK_STACK     4096

// ---------------------------- estado interno ---------------------------------
enum AudioMode : uint8_t { AUDIO_IDLE = 0, AUDIO_REC = 1, AUDIO_PLAY = 2 };
enum AudioPlayState : uint8_t { PS_RAMP_UP = 0, PS_PREBUF, PS_PLAY, PS_RAMP_DOWN };

static hw_timer_t*   audio_timer       = nullptr;
static TaskHandle_t  audio_task_handle = nullptr;
static uint8_t*      audio_buf         = nullptr;      // buffer circular (heap)
static bool          audio_ok          = false;

static volatile uint8_t  audio_mode        = AUDIO_IDLE;
static volatile uint32_t audio_isr_ticks   = 0;        // incrementado na interrupcao
static uint32_t          audio_done_ticks  = 0;        // processado pela tarefa
static volatile uint32_t audio_max_lag     = 0;
static volatile uint32_t audio_resyncs     = 0;

// indices do buffer circular (em unidades: amostras int16 na gravacao, bytes no play)
static volatile uint32_t audio_head = 0;               // produtor
static volatile uint32_t audio_tail = 0;               // consumidor
static uint32_t          audio_cap_samples = AUDIO_BUF_BYTES / 2;
static uint32_t          audio_cap_bytes   = AUDIO_BUF_BYTES;

// gravacao
static int               audio_cap_div     = 1;        // 1 = 16 kHz direto, 2 = 8 kHz interpolado
static int               audio_oversample  = 1;        // leituras do ADC somadas por amostra (1..4): menos ruido
static int               audio_div_phase   = 0;
static int32_t           audio_dc_q8       = 0;        // nivel DC estimado (x256)
static bool              audio_dc_init     = false;
static int32_t           audio_prev_sample = 0;
static volatile int32_t  audio_level       = 0;        // nivel medio |amostra| (escala 16 bits)
static volatile uint32_t audio_overruns    = 0;
#ifndef AUDIO_MIC_GAIN_PADRAO
#define AUDIO_MIC_GAIN_PADRAO 4   // x16 saturava com voz normal (o MAX4466 ja amplifica); 4 deixa folga
#endif
static int               audio_mic_gain    = AUDIO_MIC_GAIN_PADRAO;
static uint32_t          audio_rec_samples = 0;
static volatile uint32_t audio_rec_clips   = 0;        // amostras que estouraram o limite de 16 bits (ganho digital alto)
static volatile uint32_t audio_rec_adc_rail = 0;       // leituras do ADC coladas em 0 ou 4095 (MAX4466 saturando)
static volatile int32_t  audio_rec_peak    = 0;        // maior |amostra| da gravacao
static int64_t           audio_rec_t0_us   = 0;

// reproducao
static volatile uint8_t  audio_play_state    = PS_RAMP_UP;
static volatile bool     audio_play_finished = false;
static int               audio_level_dac     = 0;      // valor atual no DAC (0..255)
static uint8_t           audio_volume        = 100;    // 0..100 (100 = sem requantizar; o servidor ja aplica o volume)
static uint32_t          audio_play_underruns = 0;

// ---------------------------- interrupcao ------------------------------------
void IRAM_ATTR audio_isr() {
  audio_isr_ticks = audio_isr_ticks + 1;
  BaseType_t hp = pdFALSE;
  vTaskNotifyGiveFromISR(audio_task_handle, &hp);
  if (hp == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static inline void audio_timer_run(bool on) {
  if (!audio_timer) return;
  if (on) timerStart(audio_timer);
  else    timerStop(audio_timer);
}

// ---------------------------- gravacao ---------------------------------------
static inline void audio_push_sample(int32_t s) {
  {
    int32_t mag = s < 0 ? -s : s;
    if (mag > 32767) { audio_rec_clips = audio_rec_clips + 1; mag = 32767; }
    if (mag > audio_rec_peak) audio_rec_peak = mag;
  }
  if (s > 32767) s = 32767;
  if (s < -32768) s = -32768;
  uint32_t head = audio_head;
  uint32_t next = head + 1;
  if (next >= audio_cap_samples) next = 0;
  if (next == audio_tail) {              // cheio: descarta a amostra nova
    audio_overruns = audio_overruns + 1;
    return;
  }
  ((int16_t*)audio_buf)[head] = (int16_t)s;
  __sync_synchronize();
  audio_head = next;
  audio_rec_samples++;
  int32_t a = s < 0 ? -s : s;
  audio_level = audio_level + (a - audio_level) / 64;
}

static inline void audio_rec_tick() {
  if (audio_cap_div == 2) {              // captura a 8 kHz: uma leitura a cada 2 ticks
    audio_div_phase ^= 1;
    if (audio_div_phase) return;
  }
  int32_t acc = 0;                       // media de varias leituras (o ADC do ESP32 e ruidoso)
  for (int k = 0; k < audio_oversample; k++) acc += analogRead(AUDIO_MIC_PIN);   // 0..4095 cada
  int32_t raw_q8 = (acc << 8) / audio_oversample;
  {
    int32_t media = acc / audio_oversample;
    if (media <= 15 || media >= 4080) audio_rec_adc_rail = audio_rec_adc_rail + 1;
  }
  if (!audio_dc_init) {
    audio_dc_q8 = raw_q8;
    audio_dc_init = true;
  }
  audio_dc_q8 += (raw_q8 - audio_dc_q8) >> 8;         // filtro DC lento (~10 Hz)
  int32_t s = ((raw_q8 - audio_dc_q8) * audio_mic_gain) >> 8;

  if (audio_cap_div == 2) {              // interpola para entregar sempre 16 kHz
    audio_push_sample((audio_prev_sample + s) / 2);
    audio_push_sample(s);
  } else {
    audio_push_sample(s);
  }
  audio_prev_sample = s;
}

// ---------------------------- reproducao -------------------------------------
static inline void audio_dac_write(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  audio_level_dac = v;
  dacWrite(AUDIO_DAC_PIN, (uint8_t)v);
}

static inline uint32_t audio_play_avail() {
  uint32_t h = audio_head, t = audio_tail;
  return (h >= t) ? (h - t) : (audio_cap_bytes - t + h);
}

static inline void audio_play_tick() {
  switch (audio_play_state) {
    case PS_RAMP_UP:                       // sobe devagar ate o nivel medio (sem estalo)
      if (audio_level_dac < 128) audio_dac_write(audio_level_dac + 1);
      else audio_play_state = PS_PREBUF;
      break;

    case PS_PREBUF:                        // espera acumular um pouco de audio
      if (audio_play_avail() >= AUDIO_PREBUFFER || audio_play_finished) {
        audio_play_state = PS_PLAY;
      }
      break;

    case PS_PLAY:
      if (audio_tail != audio_head) {
        int v = audio_buf[audio_tail];
        uint32_t nt = audio_tail + 1;
        if (nt >= audio_cap_bytes) nt = 0;
        __sync_synchronize();
        audio_tail = nt;
        if (audio_volume < 100) v = 128 + ((v - 128) * (int)audio_volume) / 100;
        audio_dac_write(v);
      } else if (audio_play_finished) {
        audio_play_state = PS_RAMP_DOWN;
      } else {
        audio_play_underruns++;            // faltou dado: mantem o nivel atual
      }
      break;

    case PS_RAMP_DOWN:                     // desce devagar ate 0 (sem estalo)
      if (audio_level_dac > 0) {
        audio_dac_write(audio_level_dac - 1);
      } else {
        audio_timer_run(false);
        audio_mode = AUDIO_IDLE;
      }
      break;
  }
}

// ---------------------------- tarefa -----------------------------------------
static void audio_task(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);          // acorda a cada tick (ou lote)
    uint32_t lag = audio_isr_ticks - audio_done_ticks;
    if (lag > audio_max_lag) audio_max_lag = lag;
    if (lag > 4000) {                                  // atrasou > 250 ms: ressincroniza
      audio_done_ticks = audio_isr_ticks;
      audio_resyncs = audio_resyncs + 1;
      continue;
    }
    while (audio_done_ticks != audio_isr_ticks) {
      audio_done_ticks++;
      uint8_t m = audio_mode;
      if (m == AUDIO_REC) audio_rec_tick();
      else if (m == AUDIO_PLAY) audio_play_tick();
    }
  }
}

// =============================================================================
//  API PUBLICA
// =============================================================================

// Chame UMA vez no setup(), ANTES do BLE_setup() (o teste de velocidade do ADC
// fica mais confiavel sem o radio ligado e o buffer e alocado antes do Wi-Fi/BLE).
static bool audio_setup() {
  if (audio_ok) return true;

  audio_buf = (uint8_t*)heap_caps_malloc(AUDIO_BUF_BYTES, MALLOC_CAP_8BIT);
  if (!audio_buf) {
    Serial.println("[audio] ERRO: sem memoria para o buffer de audio");
    return false;
  }

  analogReadResolution(12);
  analogSetPinAttenuation(AUDIO_MIC_PIN, ADC_11db);
  pinMode(AUDIO_MIC_PIN, INPUT);
  (void)analogRead(AUDIO_MIC_PIN);                     // inicializa o canal do ADC

  // mede quanto tempo leva uma leitura do ADC para escolher 16 kHz ou 8 kHz
  uint32_t t0 = micros();
  uint32_t acc = 0;
  for (int i = 0; i < 200; i++) acc += analogRead(AUDIO_MIC_PIN);
  float us = (micros() - t0) / 200.0f;
  audio_cap_div = (us <= 30.0f) ? 1 : 2;
  // usa ate ~65% do tempo de cada amostra lendo o ADC varias vezes e fazendo a media
  float orcamento_us = 62.5f * audio_cap_div * 0.65f;
  int os = (int)(orcamento_us / (us > 1.0f ? us : 1.0f));
  audio_oversample = os < 1 ? 1 : (os > 4 ? 4 : os);
  Serial.printf("[audio] leitura do ADC: %.1f us -> captura a %d Hz%s, media de %d leitura(s)\n",
                us, AUDIO_SAMPLE_RATE / audio_cap_div,
                audio_cap_div == 2 ? " (interpolada para 16 kHz)" : "",
                audio_oversample);

  xTaskCreatePinnedToCore(audio_task, "audio", AUDIO_TASK_STACK, nullptr,
                          AUDIO_TASK_PRIO, &audio_task_handle, AUDIO_TASK_CORE);
  if (!audio_task_handle) {
    Serial.println("[audio] ERRO: nao criou a tarefa de audio");
    return false;
  }

  audio_timer = timerBegin(AUDIO_SAMPLE_RATE);         // 16000 ticks/s
  timerAttachInterrupt(audio_timer, &audio_isr);
  timerAlarm(audio_timer, 1, true, 0);                 // dispara a cada tick
  timerStop(audio_timer);                              // so roda gravando/tocando

  audio_ok = true;
  Serial.printf("[audio] pronto. heap livre: %u bytes\n", (unsigned)ESP.getFreeHeap());
  return true;
}

static bool audio_ready()   { return audio_ok; }
static bool audio_busy()    { return audio_mode != AUDIO_IDLE; }
static bool audio_playing() { return audio_mode == AUDIO_PLAY; }
static bool audio_recording() { return audio_mode == AUDIO_REC; }

static void audio_set_volume(uint8_t pct)  { audio_volume = pct > 100 ? 100 : pct; }
static void audio_set_mic_gain(int gain)   { audio_mic_gain = gain < 1 ? 1 : (gain > 64 ? 64 : gain); }

// ------------------------------ gravacao -------------------------------------
// Comeca a gravar. Retorna false se o audio nao esta pronto ou esta tocando.
static bool audio_record_start() {
  if (!audio_ok || audio_mode == AUDIO_PLAY) return false;
  if (audio_mode == AUDIO_REC) return true;
  audio_timer_run(false);
  audio_head = audio_tail = 0;
  audio_dc_init = false;
  audio_div_phase = 0;
  audio_prev_sample = 0;
  audio_level = 0;
  audio_overruns = 0;
  audio_rec_samples = 0;
  audio_rec_clips = 0;
  audio_rec_adc_rail = 0;
  audio_rec_peak = 0;
  audio_max_lag = 0;
  audio_resyncs = 0;
  audio_done_ticks = audio_isr_ticks;
  audio_rec_t0_us = esp_timer_get_time();
  audio_mode = AUDIO_REC;
  audio_timer_run(true);
  return true;
}

// Copia ate 'max' amostras PCM16 (com sinal, 16 kHz, mono) para 'out'.
// Nao bloqueia. Retorna quantas amostras copiou.
static size_t audio_record_read(int16_t* out, size_t max) {
  size_t n = 0;
  while (n < max && audio_tail != audio_head) {
    uint32_t t = audio_tail;
    out[n++] = ((int16_t*)audio_buf)[t];
    t++;
    if (t >= audio_cap_samples) t = 0;
    __sync_synchronize();
    audio_tail = t;
  }
  return n;
}

static size_t audio_record_available() {
  uint32_t h = audio_head, t = audio_tail;
  return (h >= t) ? (h - t) : (audio_cap_samples - t + h);
}

static void audio_record_stop() {
  if (audio_mode != AUDIO_REC) return;
  audio_timer_run(false);
  audio_mode = AUDIO_IDLE;
}

// nivel medio do sinal (0..32767). Silencio costuma ficar baixo; fala sobe bastante.
static int  audio_record_level()     { return (int)audio_level; }
static uint32_t audio_record_overruns() { return audio_overruns; }
// Diagnostico do microfone (valem para a ultima gravacao):
static int audio_record_peak_pct()      { return (int)(100L * audio_rec_peak / 32767L); }   // 0..100; ideal 30..80
static uint32_t audio_record_clip_count()    { return audio_rec_clips; }      // > 0: ganho digital alto demais
static uint32_t audio_record_adc_rail_count() { return audio_rec_adc_rail; }  // > 0: gire o trimpot do MAX4466 para baixo

// 100 = captura em dia; abaixo de ~95 = o ADC nao esta dando conta do ritmo.
static int audio_record_health_pct() {
  int64_t el = esp_timer_get_time() - audio_rec_t0_us;
  if (el < 200000) return 100;
  float expected = (float)el * AUDIO_SAMPLE_RATE / 1000000.0f;
  float pct = 100.0f * (float)(audio_rec_samples + audio_overruns) / expected;
  return pct > 100.0f ? 100 : (int)pct;
}
static uint32_t audio_resync_count() { return audio_resyncs; }
static uint32_t audio_underrun_count() { return audio_play_underruns; }   // faltas de dado na reproducao (Wi-Fi lento)
static uint32_t audio_max_lag_ticks()  { return audio_max_lag; }          // maior atraso da tarefa de audio (em 1/16000 s)

// ------------------------------ reproducao -----------------------------------
// Prepara para tocar PCM 8 bits SEM sinal, 16 kHz, mono. Depois use audio_play_write().
static bool audio_play_begin() {
  if (!audio_ok) return false;
  if (audio_mode == AUDIO_REC) audio_record_stop();
  if (audio_mode == AUDIO_PLAY) return true;
  audio_timer_run(false);
  audio_head = audio_tail = 0;
  audio_play_finished = false;
  audio_play_state = PS_RAMP_UP;
  audio_level_dac = 0;
  audio_play_underruns = 0;
  audio_done_ticks = audio_isr_ticks;
  audio_mode = AUDIO_PLAY;
  audio_timer_run(true);
  return true;
}

// Espaco livre no buffer de reproducao (bytes).
static size_t audio_play_free() {
  return audio_cap_bytes - 1 - audio_play_avail();
}

// Coloca bytes na fila. Nao bloqueia: retorna quantos bytes coube (pode ser 0).
static size_t audio_play_write(const uint8_t* data, size_t n) {
  if (audio_mode != AUDIO_PLAY || audio_play_finished) return 0;
  size_t free_b = audio_play_free();
  if (n > free_b) n = free_b;
  uint32_t h = audio_head;
  for (size_t i = 0; i < n; i++) {
    audio_buf[h] = data[i];
    h++;
    if (h >= audio_cap_bytes) h = 0;
  }
  __sync_synchronize();
  audio_head = h;
  return n;
}

// Avisa que nao vem mais audio; toca o que sobrou e termina sozinho.
static void audio_play_end() { audio_play_finished = true; }

// Interrompe na hora (descarta o que ainda nao tocou).
static void audio_play_stop() {
  if (audio_mode != AUDIO_PLAY) return;
  audio_tail = audio_head;
  audio_play_finished = true;
  if (audio_play_state == PS_RAMP_UP || audio_play_state == PS_PREBUF) {
    audio_play_state = PS_RAMP_DOWN;
  }
}

// Bip curto (aviso sonoro, ex.: "pode falar"). Nao bloqueia: o tom cabe no buffer.
static bool audio_beep(uint16_t hz = 880, uint16_t ms = 120) {
  if (!audio_ok || audio_mode == AUDIO_REC) return false;
  if (audio_mode == AUDIO_PLAY) return false;
  uint32_t n = (uint32_t)AUDIO_SAMPLE_RATE * ms / 1000;
  if (n > AUDIO_BUF_BYTES - 2) n = AUDIO_BUF_BYTES - 2;
  if (!audio_play_begin()) return false;
  uint32_t fade = AUDIO_SAMPLE_RATE / 200;             // 5 ms
  uint8_t tmp[64];
  uint32_t i = 0;
  while (i < n) {
    uint32_t k = 0;
    while (k < sizeof(tmp) && i < n) {
      float env = 1.0f;
      if (i < fade) env = (float)i / fade;
      else if (n - i < fade) env = (float)(n - i) / fade;
      float v = sinf(2.0f * 3.14159265f * hz * i / AUDIO_SAMPLE_RATE);
      tmp[k++] = (uint8_t)(128 + (int)(v * 90.0f * env));
      i++;
    }
    audio_play_write(tmp, k);
  }
  audio_play_end();
  return true;
}

// ------------------------------ diagnostico ----------------------------------
// Grava 2 s e imprime estatisticas no Serial. Fale ou bata palma durante o teste.
static void audio_selftest() {
  if (!audio_ok) { Serial.println("[audio] selftest: audio nao iniciado"); return; }
  Serial.println("[audio] selftest: gravando 2 s... faca barulho!");
  if (!audio_record_start()) { Serial.println("[audio] selftest: nao conseguiu gravar"); return; }
  int16_t tmp[256];
  int32_t mn = 32767, mx = -32768;
  double sum = 0, sq = 0;
  uint32_t total = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    size_t n = audio_record_read(tmp, 256);
    for (size_t i = 0; i < n; i++) {
      int32_t s = tmp[i];
      if (s < mn) mn = s;
      if (s > mx) mx = s;
      sum += s;
      sq += (double)s * s;
    }
    total += n;
    if (n == 0) delay(1);
  }
  int health = audio_record_health_pct();
  audio_record_stop();
  double mean = total ? sum / total : 0;
  double rms = total ? sqrt(sq / total) : 0;
  Serial.printf("[audio] amostras=%u (esperado ~32000) min=%ld max=%ld media=%.0f rms=%.0f\n",
                (unsigned)total, (long)mn, (long)mx, mean, rms);
  Serial.printf("[audio] saude da captura=%d%%  overruns=%u  ressincronizacoes=%u\n",
                health, (unsigned)audio_record_overruns(), (unsigned)audio_resync_count());
  if (rms < 20) Serial.println("[audio] AVISO: sinal quase zero. Confira fio do MAX4466 no GPIO35, 3V3 e GND.");
  if (mx >= 32767 || mn <= -32768) Serial.println("[audio] AVISO: saturou. Reduza o ganho do MAX4466 ou use audio_set_mic_gain().");
}

#endif // AUDIO_HANDLER_H
