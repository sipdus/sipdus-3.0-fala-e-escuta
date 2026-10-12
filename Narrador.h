#ifndef NARRADOR_H
#define NARRADOR_H

// =============================================================================
// Narrador.h - a Sip lendo as telas, volume e comandos de voz
//
// Inclua DEPOIS de Display.h, Sensor.h e BLEHandler.h. No setup(), chame
// audio_setup(), ia_setup() e por ULTIMO narrador_setup(). No loop(): narrador_loop().
//
// O que este arquivo faz:
//   - Narra cada tela para quem nao enxerga (calibrando, monitoramento, historico),
//     com os valores e a faixa de cada um. "Glicemia" sempre dita como ESTIMADA.
//   - Avisa quando o dedo e detectado/removido e marca o progresso da afericao.
//   - Controla o volume (botoes + e -, e por voz) e guarda o valor na memoria.
//   - Executa os comandos de voz que o servidor reconhece (volume, abrir historico,
//     voltar, ler a tela, nova sessao).
//   - Mostra no botao FALAR COM A SIP o que a assistente esta fazendo.
//
// Tudo que mexe no LVGL roda no loop() (nunca na tarefa da Sip).
// =============================================================================

#include <Preferences.h>
#include "AudioHandler.h"
#include "IAHandler.h"

#ifndef SIP_VOLUME_MIN
#define SIP_VOLUME_MIN    20
#endif
#ifndef SIP_VOLUME_MAX
#define SIP_VOLUME_MAX    100
#endif
#ifndef SIP_VOLUME_PASSO
#define SIP_VOLUME_PASSO  10
#endif
#ifndef SIP_VOLUME_PADRAO
#define SIP_VOLUME_PADRAO 100
#endif

// ------------------------------------------------------------------ volume
static volatile int  nar_volume        = SIP_VOLUME_PADRAO;
static volatile bool nar_volume_dirty  = true;       // precisa atualizar o texto da tela
static volatile bool nar_volume_salvar = false;      // precisa gravar na memoria
static uint32_t      nar_volume_mudou_em = 0;
static Preferences   nar_prefs;

static void nar_aplicar_volume(int v) {
  v = ((v + 5) / 10) * 10;                            // sempre multiplo de 10
  if (v < SIP_VOLUME_MIN) v = SIP_VOLUME_MIN;
  if (v > SIP_VOLUME_MAX) v = SIP_VOLUME_MAX;
  nar_volume = v;
  audio_set_volume((uint8_t)v);                       // vale na hora (inclusive para a fala seguinte)
  nar_volume_dirty = true;
  nar_volume_salvar = true;
  nar_volume_mudou_em = millis();
}

// ------------------------------------------------------------------ fila de comandos (tarefa da Sip -> loop)
#define NAR_CMD_HISTORICO 1
#define NAR_CMD_VOLTAR    2
#define NAR_CMD_LER       4
#define NAR_CMD_NOVA      8
static volatile uint8_t nar_cmd_pendente = 0;

// Roda na tarefa da Sip: so anota. Volume e aplicado direto (e so uma variavel).
static void nar_comando(const char* c) {
  if (!c) return;
  if (!strcmp(c, "VOL+"))              nar_aplicar_volume(nar_volume + SIP_VOLUME_PASSO);
  else if (!strcmp(c, "VOL-"))         nar_aplicar_volume(nar_volume - SIP_VOLUME_PASSO);
  else if (!strncmp(c, "VOL=", 4))     nar_aplicar_volume(atoi(c + 4));
  else if (!strcmp(c, "TELA=historico")) __atomic_fetch_or(&nar_cmd_pendente, NAR_CMD_HISTORICO, __ATOMIC_SEQ_CST);
  else if (!strcmp(c, "TELA=voltar"))  __atomic_fetch_or(&nar_cmd_pendente, NAR_CMD_VOLTAR, __ATOMIC_SEQ_CST);
  else if (!strcmp(c, "LER"))          __atomic_fetch_or(&nar_cmd_pendente, NAR_CMD_LER, __ATOMIC_SEQ_CST);
  else if (!strcmp(c, "NOVA"))         __atomic_fetch_or(&nar_cmd_pendente, NAR_CMD_NOVA, __ATOMIC_SEQ_CST);
  else Serial.printf("[narrador] comando desconhecido: %s\n", c);
}

// ------------------------------------------------------------------ ganchos chamados pelo Display.h (loop do LVGL)
static void sip_ui_toque(bool pressed) { ia_toque(pressed); }

static void sip_ui_falar() { ia_perguntar(); }

static void sip_ui_volume(int delta) {
  nar_aplicar_volume(nar_volume + delta);
  if (!ia_ocupada() && !audio_busy()) audio_beep(700, 70);   // o bip sai no volume novo
  Serial.printf("[narrador] volume = %d%%\n", (int)nar_volume);
}

// ------------------------------------------------------------------ contexto enviado junto com as perguntas
static int nar_ctx_volume() { return (int)nar_volume; }
static const char* nar_ctx_tela() {
  switch (display_screen_id()) {
    case 0: return "calibrando";
    case 2: return "historico";
    default: return "monitoramento";
  }
}

// ------------------------------------------------------------------ texto das narracoes
// Texto de faixa em minusculas e sem "Sem leitura" (ex.: "Faixa usual" -> "faixa usual").
static void nar_minusculas(const char* in, char* out, size_t n) {
  size_t i = 0;
  for (; in[i] && i + 1 < n; i++) out[i] = (char)tolower((unsigned char)in[i]);
  out[i] = 0;
}

static bool nar_tem_leitura() {
  return latest_bpm > 0 && latest_spo2 > 0 && latest_glucose > 0;
}

// "Sua glicemia estimada esta em 110 ... faixa usual. Batimentos ... Oximetria ..."
static void nar_valores(char* out, size_t n) {
  char fg[24], fb[24], fo[24];
  nar_minusculas(get_glucose_status(latest_glucose), fg, sizeof(fg));
  nar_minusculas(get_bpm_status(latest_bpm), fb, sizeof(fb));
  nar_minusculas(get_spo2_status(latest_spo2), fo, sizeof(fo));
  snprintf(out, n,
           "Sua glicemia estimada está em %d miligramas por decilitro, %s. "
           "Batimentos, %d por minuto, %s. "
           "Oximetria, %d por cento, %s.",
           (int)roundf(latest_glucose), fg, latest_bpm, fb, (int)roundf(latest_spo2), fo);
}

static void nar_texto_tela(int tela, int anterior, char* out, size_t n) {
  if (tela == 0) {
    snprintf(out, n,
             "Calibrando o dispositivo para uma melhor aferição. "
             "Adicione o dedo no sensor, localizado no lado direito.");
    return;
  }

  if (tela == 1) {
    char corpo[220];
    if (nar_tem_leitura()) {
      nar_valores(corpo, sizeof(corpo));
    } else {
      snprintf(corpo, sizeof(corpo),
               "Coloque o dedo no sensor, localizado no lado direito, e aguarde a aferição.");
    }
    if (anterior == 0) {                               // primeira vez: ensina como falar com a Sip
      snprintf(out, n,
               "Tela de monitoramento. Para falar comigo, toque e segure a tela por um segundo. %s",
               corpo);
    } else {
      snprintf(out, n, "Tela de monitoramento. %s", corpo);
    }
    return;
  }

  // historico
  int total = measurement_count;
  if (total <= 0) {
    snprintf(out, n,
             "Tela de histórico. Nenhuma aferição registrada ainda. "
             "Diga voltar para retornar à tela de monitoramento.");
    return;
  }
  char fm[24];
  float media = getFinalAverage();
  nar_minusculas(get_glucose_status(media), fm, sizeof(fm));
  char cont[40];
  if (isMeasurementComplete()) {
    snprintf(cont, sizeof(cont), "Aferições concluídas, %d de %d.", total, MAX_MEASUREMENTS);
  } else if (total == 1) {
    snprintf(cont, sizeof(cont), "Uma aferição registrada.");
  } else {
    snprintf(cont, sizeof(cont), "%d aferições registradas.", total);
  }
  snprintf(out, n,
           "Tela de histórico. %s Média da glicemia estimada, %d miligramas por decilitro, %s. "
           "Diga voltar para retornar, ou nova sessão para recomeçar.",
           cont, (int)roundf(media), fm);
}

// ------------------------------------------------------------------ envio com espera
// Uma narracao nova substitui a pendente. So e enviada quando a Sip esta livre,
// para nao cortar uma resposta no meio (ia_narrar interrompe a fala atual).
static char     nar_pend[320];
static bool     nar_pend_ativo = false;
static uint32_t nar_pend_desde = 0;
static uint32_t nar_ultimo_envio = 0;

static void nar_agendar(const char* texto) {
  snprintf(nar_pend, sizeof(nar_pend), "%s", texto);
  nar_pend_ativo = true;
  nar_pend_desde = millis();
}

// Aviso de baixa prioridade: so entra se nao ha nada pendente e a Sip esta livre.
static void nar_aviso(const char* texto) {
  if (nar_pend_ativo || ia_ocupada()) return;
  nar_agendar(texto);
}

static void nar_enviar_pendente() {
  if (!nar_pend_ativo) return;
  uint32_t agora = millis();
  if (agora - nar_ultimo_envio < 700) return;
  // espera o Wi-Fi (ate 12 s) para o primeiro aviso nao virar bip de erro
  if (!ia_wifi_conectado() && agora - nar_pend_desde < 12000) return;
  // espera a Sip ficar livre (ate 40 s)
  if (ia_ocupada() && agora - nar_pend_desde < 40000) return;
  Serial.printf("[narrador] %s\n", nar_pend);
  ia_narrar(nar_pend);
  nar_pend_ativo = false;
  nar_ultimo_envio = agora;
}

// ------------------------------------------------------------------ setup / loop
static int      nar_tela_anterior = -1;
static int      nar_marco_ultimo = 0;
static bool     nar_dedo_antes = false;
static uint32_t nar_dedo_mudou_em = 0;
static uint32_t nar_aviso_dedo_em = 0;
static uint32_t nar_aviso_remover_em = 0;
static int      nar_estado_sip_antes = -1;

static void narrador_setup() {
  nar_prefs.begin("sip", false);
  int v = nar_prefs.getInt("vol", SIP_VOLUME_PADRAO);
  nar_aplicar_volume(v);
  nar_volume_salvar = false;                           // acabou de ler: nao precisa gravar de novo
  Serial.printf("[narrador] volume salvo: %d%%\n", (int)nar_volume);

  ia_set_comando_cb(nar_comando);
  ia_set_contexto(nullptr, nar_ctx_volume, nar_ctx_tela);   // sem medidor de bateria

  display_set_volume(nar_volume);
  nar_volume_dirty = false;
}

static void narrador_loop() {
  uint32_t agora = millis();

  // ---- mudanca de tela -> narra a tela nova ----
  int tela = display_screen_id();
  if (tela != nar_tela_anterior) {
    int anterior = nar_tela_anterior;
    nar_tela_anterior = tela;
    char texto[320];
    nar_texto_tela(tela, anterior, texto, sizeof(texto));
    nar_agendar(texto);
    nar_marco_ultimo = measurement_count;
  }

  // ---- comandos de voz que mexem na tela ----
  uint8_t cmds = __atomic_exchange_n(&nar_cmd_pendente, (uint8_t)0, __ATOMIC_SEQ_CST);
  if (cmds && display_screen_id() != 0) {              // durante a calibracao os comandos de tela sao ignorados
    if (cmds & NAR_CMD_NOVA) {
      resetHistorico();
      show_main_screen();
      update_measurement_counter();
      nar_marco_ultimo = 0;
    } else if (cmds & NAR_CMD_HISTORICO) {
      if (display_screen_id() == 1) open_history_screen();
      else nar_agendar("Você já está na tela de histórico.");
    } else if (cmds & NAR_CMD_VOLTAR) {
      if (display_screen_id() == 2) show_main_screen();
      else nar_agendar("Você já está na tela de monitoramento.");
    }
    if (cmds & NAR_CMD_LER) {
      char texto[320];
      nar_texto_tela(display_screen_id(), -1, texto, sizeof(texto));
      nar_agendar(texto);
    }
  } else if (cmds & NAR_CMD_LER) {
    char texto[320];
    nar_texto_tela(0, -1, texto, sizeof(texto));
    nar_agendar(texto);
  }

  // ---- dedo no sensor e marcos da afericao (so na tela de monitoramento) ----
  if (display_screen_id() == 1) {
    bool dedo = finger_detected;
    if (dedo != nar_dedo_antes) {
      nar_dedo_antes = dedo;
      nar_dedo_mudou_em = agora;
    }
    if (dedo && agora - nar_dedo_mudou_em > 1200 && agora - nar_aviso_dedo_em > 20000 &&
        measurement_count == 0) {
      nar_aviso_dedo_em = agora;
      nar_aviso("Dedo detectado. Mantenha o dedo parado no sensor.");
    }
    if (!dedo && measurement_count > 0 && !isMeasurementComplete() &&
        agora - nar_dedo_mudou_em > 2500 && agora - nar_aviso_remover_em > 20000) {
      nar_aviso_remover_em = agora;
      nar_aviso("Dedo removido do sensor. Coloque o dedo no sensor para continuar.");
    }

    if (measurement_count < nar_marco_ultimo) nar_marco_ultimo = measurement_count;   // nova sessao
    if ((measurement_count == 10 || measurement_count == 20) &&
        measurement_count != nar_marco_ultimo && nar_tem_leitura()) {
      nar_marco_ultimo = measurement_count;
      char valores[220], texto[300];
      nar_valores(valores, sizeof(valores));
      snprintf(texto, sizeof(texto), "%d aferições. %s", measurement_count, valores);
      nar_aviso(texto);
    }
  }

  // ---- volume: atualiza o texto e grava na memoria depois de 3 s sem mudar ----
  if (nar_volume_dirty) {
    nar_volume_dirty = false;
    display_set_volume(nar_volume);
  }
  if (nar_volume_salvar && agora - nar_volume_mudou_em > 3000) {
    nar_volume_salvar = false;
    nar_prefs.putInt("vol", (int)nar_volume);
  }

  // ---- botao FALAR COM A SIP mostra o que a assistente esta fazendo ----
  int est = ia_estado_atual();
  if (est != nar_estado_sip_antes) {
    nar_estado_sip_antes = est;
    switch (est) {
      case IA_CONECTANDO: display_set_sip_button("CONECTANDO...", true); break;
      case IA_OUVINDO:    display_set_sip_button("OUVINDO...", true); break;
      case IA_PENSANDO:   display_set_sip_button("PENSANDO...", true); break;
      case IA_FALANDO:    display_set_sip_button("FALANDO...", true); break;
      default:            display_set_sip_button("FALAR COM A SIP", false); break;
    }
  }

  nar_enviar_pendente();
}

#endif // NARRADOR_H
