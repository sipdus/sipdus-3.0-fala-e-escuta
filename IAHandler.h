
#ifndef IA_HANDLER_H
#define IA_HANDLER_H

// =============================================================================
// IAHandler.h - conversa com a assistente Sip
//
//  - Conecta no Wi-Fi e no servidor da Sip (Mac) por TCP.
//  - Segurando o dedo na tela (ia_toque) a Sip toca um bip, grava a pergunta
//    (microfone), envia ao servidor junto com os sinais vitais e toca a resposta.
//  - ia_narrar("texto") faz a Sip falar um texto (para narrar a tela).
//
// Tudo roda em uma tarefa propria (nucleo 0): o loop(), o LVGL e o sensor nao
// ficam esperando a rede.
//
// Ordem no setup():  audio_setup();  ...  BLE_setup();  ia_setup();
// No loop():         nada (a tarefa trabalha sozinha). A tela chama ia_toque().
//
// Protocolo (igual ao server.py):  quadro = 1 byte tipo + 4 bytes tamanho (LE) + dados
//   enviamos: H hello | V vitais JSON | A audio PCM16 16 kHz | E fim | S texto para falar
//   recebemos: T texto | A audio PCM8 16 kHz | D fim | X erro | C comando de voz
//
// Comandos de voz (quadro C): o servidor entende "aumentar volume", "abrir historico"...
// e manda o comando. Quem trata e a funcao registrada em ia_set_comando_cb()
// (ela roda na tarefa da Sip, nao no loop: nao mexa no LVGL dentro dela).
// =============================================================================

#include <WiFi.h>
#include <freertos/queue.h>
#include "SipConfig.h"
#include "AudioHandler.h"

// valores atuais do sensor (definidos no .ino e no Sensor.h do projeto)
extern int   latest_bpm;
extern float latest_spo2;
extern float latest_glucose;
extern bool  finger_detected;

#ifndef SIP_HOLD_MS
#define SIP_HOLD_MS      900
#endif
#ifndef SIP_SILENCIO_MS
#define SIP_SILENCIO_MS  1100
#endif
#ifndef SIP_SEM_FALA_MS
#define SIP_SEM_FALA_MS  5000
#endif
#ifndef SIP_MAX_FALA_MS
#define SIP_MAX_FALA_MS  9000
#endif
#define IA_MIN_FALA_MS   350        // fala minima para valer como pergunta
#define IA_PISO_MS       300        // tempo inicial medindo o ruido de fundo
#define IA_TASK_STACK    8192
#define IA_TASK_PRIO     3
#define IA_TASK_CORE     0
#define IA_BUF_BYTES     2048
#ifndef SIP_MIC_GAIN
#define SIP_MIC_GAIN     4          // ganho digital do microfone (1..64). Menor = menos distorcao
#endif

enum IaCmd : uint8_t { IA_CMD_PERGUNTAR = 1, IA_CMD_NARRAR = 2 };
enum IaEstado : uint8_t { IA_PARADA = 0, IA_OCIOSA, IA_CONECTANDO, IA_OUVINDO, IA_PENSANDO, IA_FALANDO };

struct IaMsg {
  uint8_t cmd;
  char    texto[320];
};

static volatile uint8_t ia_estado = IA_PARADA;
static WiFiClient       ia_cli;
static QueueHandle_t    ia_fila   = nullptr;
static uint8_t*         ia_buf    = nullptr;
static uint32_t         ia_wifi_ultima_tentativa = 0;

// ganchos opcionais (o projeto registra no setup)
static void        (*ia_cmd_cb)(const char*) = nullptr;   // recebe "VOL+", "TELA=historico"...
static int         (*ia_ctx_bateria)()       = nullptr;   // 0..100, ou -1 sem medidor
static int         (*ia_ctx_volume)()        = nullptr;   // 0..100
static const char* (*ia_ctx_tela)()          = nullptr;   // "calibrando" | "monitoramento" | "historico"

// ------------------------------------------------------------------ rede
static bool ia_ler_exato(uint8_t* dst, size_t n, uint32_t timeout_ms) {
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < n) {
    int disp = ia_cli.available();
    if (disp > 0) {
      int r = ia_cli.read(dst + got, n - got);
      if (r > 0) {
        got += (size_t)r;
        t0 = millis();
      }
    } else {
      if (!ia_cli.connected()) return false;
      if (millis() - t0 > timeout_ms) return false;
      delay(1);
    }
  }
  return true;
}

static bool ia_enviar_quadro(char tipo, const uint8_t* dados, uint32_t n) {
  uint8_t head[5] = {(uint8_t)tipo, (uint8_t)(n & 0xFF), (uint8_t)((n >> 8) & 0xFF),
                     (uint8_t)((n >> 16) & 0xFF), (uint8_t)((n >> 24) & 0xFF)};
  if (ia_cli.write(head, 5) != 5) return false;
  if (n && ia_cli.write(dados, n) != n) return false;
  return true;
}

static void ia_wifi_iniciar() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // menos atraso no audio
  WiFi.begin(SIP_WIFI_SSID, SIP_WIFI_SENHA);
  ia_wifi_ultima_tentativa = millis();
  Serial.printf("[ia] conectando no Wi-Fi \"%s\" ...\n", SIP_WIFI_SSID);
}

static bool ia_wifi_ok() { return WiFi.status() == WL_CONNECTED; }

// Liga o Wi-Fi o MAIS CEDO possivel no setup(), antes do display e do BLE,
// quando a memoria ainda esta inteira (o driver de Wi-Fi precisa de blocos grandes).
static bool ia_wifi_ligado_cedo = false;
static void ia_wifi_antecipar() {
  if (ia_wifi_ligado_cedo) return;
  ia_wifi_ligado_cedo = true;
  ia_wifi_iniciar();
  Serial.printf("[ia] Wi-Fi iniciado cedo. heap livre: %u, maior bloco: %u\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
}

static bool ia_conectar() {
  if (!ia_wifi_ok()) {
    uint32_t t0 = millis();
    while (!ia_wifi_ok() && millis() - t0 < 6000) delay(100);
    if (!ia_wifi_ok()) {
      Serial.println("[ia] sem Wi-Fi");
      return false;
    }
  }
  if (ia_cli.connected()) return true;
  ia_cli.stop();
  ia_estado = IA_CONECTANDO;
  Serial.printf("[ia] conectando em %s:%u ...\n", SIP_HOST, (unsigned)SIP_PORTA);
  if (!ia_cli.connect(SIP_HOST, SIP_PORTA, 4000)) {
    Serial.println("[ia] nao conectou no servidor (ligado? IP certo? mesmo Wi-Fi?)");
    return false;
  }
  ia_cli.setNoDelay(true);

  char hello[96];
  snprintf(hello, sizeof(hello), "{\"token\":\"%s\"}", SIP_TOKEN);
  if (!ia_enviar_quadro('H', (const uint8_t*)hello, strlen(hello))) {
    ia_cli.stop();
    return false;
  }
  uint8_t head[5];
  if (!ia_ler_exato(head, 5, 5000)) {
    Serial.println("[ia] servidor nao respondeu ao hello");
    ia_cli.stop();
    return false;
  }
  uint32_t n = (uint32_t)head[1] | ((uint32_t)head[2] << 8) | ((uint32_t)head[3] << 16) |
               ((uint32_t)head[4] << 24);
  if (head[0] == 'X') {
    char msg[64] = {0};
    size_t k = n < sizeof(msg) - 1 ? n : sizeof(msg) - 1;
    ia_ler_exato((uint8_t*)msg, k, 2000);
    Serial.printf("[ia] servidor recusou: %s (confira o token)\n", msg);
    ia_cli.stop();
    return false;
  }
  Serial.println("[ia] conectado no servidor");
  return true;
}

// ------------------------------------------------------------------ audio
static bool ia_novo_comando() { return ia_fila && uxQueueMessagesWaiting(ia_fila) > 0; }

static void ia_esperar_audio_livre(uint32_t max_ms) {
  uint32_t t0 = millis();
  while (audio_busy() && millis() - t0 < max_ms) delay(5);
}

static void ia_bip(uint16_t hz, uint16_t ms) {
  ia_esperar_audio_livre(1500);
  audio_beep(hz, ms);
  ia_esperar_audio_livre(1500);
}

static void ia_erro_sonoro() {
  ia_bip(400, 140);
  delay(80);
  ia_bip(300, 200);
}

static void ia_enviar_vitais() {
  char js[220];
  snprintf(js, sizeof(js),
           "{\"bpm\":%d,\"spo2\":%.0f,\"glicemia\":%.0f,\"dedo\":%s,"
           "\"bateria\":%d,\"volume\":%d,\"tela\":\"%s\"}",
           latest_bpm, (double)latest_spo2, (double)latest_glucose,
           finger_detected ? "true" : "false",
           ia_ctx_bateria ? ia_ctx_bateria() : -1,
           ia_ctx_volume ? ia_ctx_volume() : 100,
           ia_ctx_tela ? ia_ctx_tela() : "monitoramento");
  ia_enviar_quadro('V', (const uint8_t*)js, strlen(js));
}

// Recebe a resposta do servidor (T/A/X ate o D) e toca o audio.
// Retorna false se a rede falhou antes de receber qualquer coisa.
static bool ia_receber(bool* recebeu_algo) {
  bool tocando = false;
  bool descartar = false;               // true depois de interrompida: so le ate o D
  uint32_t total = 0;
  ia_estado = IA_PENSANDO;
  *recebeu_algo = false;

  for (;;) {
    uint8_t head[5];
    if (!ia_ler_exato(head, 5, 30000)) {
      Serial.println("[ia] sem resposta do servidor");
      ia_cli.stop();
      if (tocando) audio_play_stop();
      return *recebeu_algo;
    }
    *recebeu_algo = true;
    uint32_t n = (uint32_t)head[1] | ((uint32_t)head[2] << 8) | ((uint32_t)head[3] << 16) |
                 ((uint32_t)head[4] << 24);
    if (n > IA_BUF_BYTES) {
      Serial.println("[ia] quadro grande demais, abortando");
      ia_cli.stop();
      if (tocando) audio_play_stop();
      return true;
    }
    if (n && !ia_ler_exato(ia_buf, n, 5000)) {
      Serial.println("[ia] quadro incompleto");
      ia_cli.stop();
      if (tocando) audio_play_stop();
      return true;
    }
    char tipo = (char)head[0];

    if (tipo == 'A') {
      if (!descartar && ia_novo_comando()) {           // chegou outro pedido: corta a fala
        descartar = true;
        if (tocando) audio_play_stop();
      }
      if (descartar) continue;
      if (!tocando) {
        ia_esperar_audio_livre(800);
        if (!audio_play_begin()) {
          descartar = true;
          continue;
        }
        tocando = true;
        ia_estado = IA_FALANDO;
      }
      size_t off = 0;
      uint32_t t_sem_progresso = millis();
      while (off < n) {
        size_t w = audio_play_write(ia_buf + off, n - off);
        if (w > 0) {
          off += w;
          t_sem_progresso = millis();
        } else {
          if (millis() - t_sem_progresso > 3000) break;  // travou: desiste deste pedaco
          if (ia_novo_comando()) {
            descartar = true;
            audio_play_stop();
            break;
          }
          delay(1);
        }
      }
      total += n;
    } else if (tipo == 'T') {
      ia_buf[n] = 0;
      Serial.printf("[ia] %s\n", (char*)ia_buf);
    } else if (tipo == 'C') {
      ia_buf[n] = 0;
      Serial.printf("[ia] comando de voz: %s\n", (char*)ia_buf);
      if (ia_cmd_cb) ia_cmd_cb((const char*)ia_buf);
    } else if (tipo == 'X') {
      ia_buf[n] = 0;
      Serial.printf("[ia] ERRO do servidor: %s\n", (char*)ia_buf);
    } else if (tipo == 'D') {
      break;
    }
  }

  if (tocando) {
    audio_play_end();
    Serial.printf("[ia] audio recebido: %u bytes (%.1f s)\n", (unsigned)total,
                  total / (float)AUDIO_SAMPLE_RATE);
    while (audio_playing()) {
      if (ia_novo_comando()) {
        audio_play_stop();
        break;
      }
      delay(5);
    }
    // diagnostico de qualidade: 'faltas' > 0 = Wi-Fi nao entregou a tempo (voz picotada/chiada)
    Serial.printf("[ia] reproducao: faltas de dado=%u, atraso max da tarefa=%u amostras\n",
                  (unsigned)audio_underrun_count(), (unsigned)audio_max_lag_ticks());
  }
  return true;
}

// ------------------------------------------------------------------ pedidos
static void ia_fazer_narracao(const char* texto) {
  for (int tentativa = 0; tentativa < 2; tentativa++) {
    if (!ia_conectar()) {
      ia_erro_sonoro();
      return;
    }
    if (!ia_enviar_quadro('S', (const uint8_t*)texto, strlen(texto))) {
      ia_cli.stop();
      continue;                                        // conexao velha: tenta de novo
    }
    bool algo = false;
    if (ia_receber(&algo) || algo) return;
    ia_cli.stop();                                     // nada recebido: reconecta e repete
  }
  ia_erro_sonoro();
}

static void ia_fazer_pergunta() {
  if (!ia_conectar()) {
    ia_erro_sonoro();
    return;
  }
  ia_enviar_vitais();

  // bip de "pode falar" e grava
  ia_estado = IA_OUVINDO;
  ia_bip(880, 110);
  delay(100);
  if (!audio_record_start()) {
    Serial.println("[ia] nao conseguiu iniciar a gravacao");
    ia_erro_sonoro();
    return;
  }

  static int16_t tmp[256];
  uint32_t total_ms = 0, fala_ms = 0, sil_ms = 0, piso_n = 0;
  float piso = 0;
  bool falou = false;
  bool rede_ok = true;
  uint32_t t0 = millis();

  for (;;) {
    size_t n = audio_record_read(tmp, 256);
    if (n == 0) {
      if (millis() - t0 > (uint32_t)SIP_MAX_FALA_MS + 1500) break;   // seguranca
      delay(2);
      continue;
    }
    if (!ia_enviar_quadro('A', (const uint8_t*)tmp, (uint32_t)(n * 2))) {
      rede_ok = false;
      break;
    }
    uint32_t ms = (uint32_t)(n * 1000 / AUDIO_SAMPLE_RATE);
    total_ms += ms;
    int lvl = audio_record_level();

    if (total_ms <= IA_PISO_MS) {                      // mede o ruido de fundo
      piso = (piso * piso_n + lvl) / (piso_n + 1);
      piso_n++;
      continue;
    }
    float limiar = piso * 3.0f + 500.0f;
    if (lvl > limiar) {
      fala_ms += ms;
      sil_ms = 0;
      if (fala_ms >= IA_MIN_FALA_MS) falou = true;
    } else if (falou) {
      sil_ms += ms;
    }
    if (falou && sil_ms >= SIP_SILENCIO_MS) break;
    if (!falou && total_ms >= SIP_SEM_FALA_MS) break;
    if (total_ms >= SIP_MAX_FALA_MS) break;
  }
  audio_record_stop();
  Serial.printf("[ia] gravou %u ms (fala %u ms, ruido de fundo %.0f, estouros %u)\n",
                (unsigned)total_ms, (unsigned)fala_ms, piso, (unsigned)audio_record_overruns());
  // diagnostico do microfone: pico ideal 30..80%; saturacao = voz distorcida e a Sip nao entende
  Serial.printf("[ia] microfone: pico %d%%, saturou no digital %u vez(es), ADC colado no limite %u vez(es)\n",
                audio_record_peak_pct(), (unsigned)audio_record_clip_count(),
                (unsigned)audio_record_adc_rail_count());
  if (audio_record_adc_rail_count() > 20)
    Serial.println("[ia] AVISO: o MAX4466 esta saturando. Gire o trimpot dele para BAIXO ou afaste-se do microfone.");
  else if (audio_record_clip_count() > 20)
    Serial.println("[ia] AVISO: ganho digital alto. Diminua SIP_MIC_GAIN no SipConfig.h.");
  else if (fala_ms > 0 && audio_record_peak_pct() < 15)
    Serial.println("[ia] AVISO: voz muito fraca. Chegue mais perto (10 a 15 cm) ou aumente SIP_MIC_GAIN.");

  if (!rede_ok) {
    Serial.println("[ia] a rede caiu durante a gravacao");
    ia_cli.stop();
    ia_erro_sonoro();
    return;
  }
  ia_enviar_quadro('E', nullptr, 0);
  ia_bip(560, 80);                                     // "ouvi, vou pensar"

  bool algo = false;
  if (!ia_receber(&algo) && !algo) ia_erro_sonoro();
}

// ------------------------------------------------------------------ tarefa
static void ia_task(void*) {
  if (!ia_wifi_ligado_cedo) ia_wifi_iniciar();
  ia_estado = IA_OCIOSA;
  for (;;) {
    IaMsg m;
    if (xQueueReceive(ia_fila, &m, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (m.cmd == IA_CMD_PERGUNTAR) ia_fazer_pergunta();
      else if (m.cmd == IA_CMD_NARRAR) ia_fazer_narracao(m.texto);
      ia_estado = IA_OCIOSA;
    }
    if (!ia_wifi_ok() && millis() - ia_wifi_ultima_tentativa > 15000) {
      ia_wifi_iniciar();                               // tenta de novo a cada 15 s
    }
  }
}

// =============================================================================
//  API PUBLICA
// =============================================================================

// Chame uma vez no setup(), depois de audio_setup() e de BLE_setup().
static bool ia_setup() {
  if (ia_fila) return true;
  if (!audio_ready()) {
    Serial.println("[ia] ERRO: chame audio_setup() antes de ia_setup()");
    return false;
  }
  ia_buf = (uint8_t*)heap_caps_malloc(IA_BUF_BYTES + 8, MALLOC_CAP_8BIT);
  audio_set_mic_gain(SIP_MIC_GAIN);
  ia_fila = xQueueCreate(4, sizeof(IaMsg));
  if (!ia_buf || !ia_fila) {
    Serial.println("[ia] ERRO: sem memoria para a assistente");
    return false;
  }
  xTaskCreatePinnedToCore(ia_task, "sip_ia", IA_TASK_STACK, nullptr, IA_TASK_PRIO, nullptr,
                          IA_TASK_CORE);
  Serial.printf("[ia] pronta. heap livre: %u bytes\n", (unsigned)ESP.getFreeHeap());
  return true;
}

// Faz a Sip falar um texto (narracao). Um pedido novo interrompe a fala atual.
static void ia_narrar(const char* texto) {
  if (!ia_fila || !texto || !texto[0]) return;
  IaMsg m;
  m.cmd = IA_CMD_NARRAR;
  strncpy(m.texto, texto, sizeof(m.texto) - 1);
  m.texto[sizeof(m.texto) - 1] = 0;
  if (xQueueSend(ia_fila, &m, 0) != pdTRUE) {          // fila cheia: descarta o mais antigo
    IaMsg velho;
    xQueueReceive(ia_fila, &velho, 0);
    xQueueSend(ia_fila, &m, 0);
  }
}

// Pede para a Sip escutar uma pergunta (ignora se ja estiver ouvindo/pensando).
static void ia_perguntar() {
  if (!ia_fila) return;
  if (ia_estado == IA_OUVINDO || ia_estado == IA_PENSANDO || ia_estado == IA_CONECTANDO) return;
  IaMsg m;
  m.cmd = IA_CMD_PERGUNTAR;
  m.texto[0] = 0;
  xQueueSend(ia_fila, &m, 0);
}

// Chame a CADA leitura do toque da tela (pressionado = true enquanto o dedo esta na tela).
// Segurar o dedo por SIP_HOLD_MS em qualquer lugar ativa a escuta.
static void ia_toque(bool pressionado) {
  static bool anterior = false;
  static bool disparou = false;
  static uint32_t t_inicio = 0;
  if (pressionado) {
    if (!anterior) {
      t_inicio = millis();
      disparou = false;
    } else if (!disparou && millis() - t_inicio >= SIP_HOLD_MS) {
      disparou = true;
      ia_perguntar();
    }
  } else {
    disparou = false;
  }
  anterior = pressionado;
}

static bool ia_ocupada() { return ia_estado != IA_OCIOSA && ia_estado != IA_PARADA; }
static uint8_t ia_estado_atual() { return ia_estado; }     // IaEstado: OUVINDO, PENSANDO, FALANDO...

// Registra o tratador dos comandos de voz (volume, trocar de tela...).
static void ia_set_comando_cb(void (*cb)(const char*)) { ia_cmd_cb = cb; }
// Informa a Sip sobre o aparelho (vai junto com cada pergunta): bateria, volume e tela atual.
static void ia_set_contexto(int (*bateria)(), int (*volume)(), const char* (*tela)()) {
  ia_ctx_bateria = bateria;
  ia_ctx_volume = volume;
  ia_ctx_tela = tela;
}
static bool ia_wifi_conectado() { return ia_wifi_ok(); }

#endif // IA_HANDLER_H
