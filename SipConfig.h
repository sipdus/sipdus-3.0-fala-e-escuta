#ifndef SIP_CONFIG_H
#define SIP_CONFIG_H

// =============================================================================
// SipConfig.h - SUAS configuracoes da assistente Sip
// Preencha os campos abaixo. NAO compartilhe este arquivo com a senha do Wi-Fi.
// =============================================================================

// --- Wi-Fi (precisa ser rede de 2,4 GHz) ---
#define SIP_WIFI_SSID   "Desktop_F0411408"
#define SIP_WIFI_SENHA  "32961748"

// --- Servidor da Sip (o Mac onde roda o server.py) ---
#define SIP_HOST        "192.168.1.26"     // IP do Mac: ipconfig getifaddr en0
#define SIP_PORTA       8765
#define SIP_TOKEN       "2026"             // o mesmo SIP_TOKEN do servidor

// --- Ajustes opcionais ---
#define SIP_HOLD_MS         900            // tempo segurando o dedo na tela para falar
#define SIP_SILENCIO_MS     1100           // silencio depois da fala para encerrar a pergunta
#define SIP_SEM_FALA_MS     5000           // desiste se ninguem falar nesse tempo
#define SIP_MAX_FALA_MS     9000           // pergunta mais longa permitida

#endif // SIP_CONFIG_H
