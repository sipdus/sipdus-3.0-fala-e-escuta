#ifndef BLEHANDLER_H
#define BLEHANDLER_H

#include <Arduino.h>

// =============================================================================
// BLE do SIPDUS
//
// Usa a biblioteca NimBLE-Arduino: gasta bem menos RAM que o BLE original do ESP32,
// e o Wi-Fi + a assistente Sip precisam dessa memoria.
// Nome do aparelho, UUIDs e formato dos dados "bpm,spo2,glucose" sao os mesmos.
// =============================================================================

// ==== UUIDs do serviço e característica BLE ====
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Precisa da biblioteca "NimBLE-Arduino" (autor h2zero, versao 2.x):
// Arduino IDE > Sketch > Include Library > Manage Libraries > buscar NimBLE-Arduino.
// Se der o erro "NimBLEDevice.h: No such file", ela nao esta instalada.
#include <NimBLEDevice.h>

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pCharacteristic = nullptr;

// ==== Inicialização BLE (NimBLE) ====
inline void BLE_setup() {
    Serial.println("🔵 Inicializando BLE (NimBLE, leve)...");

    NimBLEDevice::init("ESP32_MAX30102");

    pServer = NimBLEDevice::createServer();

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // Propriedade de notificacao (o NimBLE cria sozinho o descritor 0x2902)
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->enableScanResponse(true);
    pAdvertising->start();

    Serial.println("✅ BLE iniciado e anunciando serviço!");
}

// ==== Envio de dados via BLE ====
// O app espera dados no formato "bpm,spo2,glucose"
inline void BLE_send(int bpm, float spo2, float glucose, bool finger_detected) {
    if (!pCharacteristic) return;

    if (!finger_detected) {
        Serial.println("⚠️ Dedo não detectado — enviando zeros");
        bpm = 0;
        spo2 = 0;
        glucose = 0;
    }

    // Monta payload simples (CSV)
    String payload = String(bpm) + "," + String(spo2, 1) + "," + String(glucose, 1);

    // Converte e envia
    std::string strPayload = payload.c_str();
    pCharacteristic->setValue((uint8_t*)strPayload.c_str(), strPayload.length());
    pCharacteristic->notify();

    // Imprime so quando o valor muda (ou a cada 10 s), para nao encher o Serial Monitor
    // (antes imprimia 10 vezes por segundo e escondia as outras mensagens).
    static String ultimo_payload = "";
    static unsigned long ultimo_print = 0;

    if (payload != ultimo_payload || millis() - ultimo_print > 10000) {
        ultimo_payload = payload;
        ultimo_print = millis();
        Serial.print("📤 Enviado via BLE: ");
        Serial.println(payload);
    }
}

#endif // BLEHANDLER_H
