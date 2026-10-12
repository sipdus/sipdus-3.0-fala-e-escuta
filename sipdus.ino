// =============================================================================
// SIPDUS - projeto completo: monitoramento + BLE + assistente de voz Sip
//
// Arquivos na MESMA pasta deste sketch:
//   projetolimpo.ino (este)   Display.h   Sensor.h   filters.h   BLEHandler.h
//   AudioHandler.h   IAHandler.h   Narrador.h   SipConfig.h
//
// Placa: ESP32 Dev Module.  Em Tools > Partition Scheme escolha
//        "Huge APP (3MB No OTA/1MB SPIFFS)"  (Wi-Fi + Bluetooth + LVGL nao cabem no padrao).
// =============================================================================

#include <lvgl.h>
#include <Ticker.h>
#include <esp_bt.h>

#include "Display.h"
#include "Sensor.h"
#include "BLEHandler.h"
#include "Narrador.h"      // inclui AudioHandler.h, IAHandler.h e SipConfig.h

int latest_bpm = 0;
float latest_spo2 = 0.0;
float latest_glucose = 0.0;
static bool ble_ok = false;


void setup()
{
    Serial.begin(115200);

    // Sensor primeiro (como no projeto original que funcionava)
    sensor_setup();

    // ORDEM IMPORTANTE (a RAM do ESP32 e curta): primeiro o que mais precisa de memoria
    // contigua, o Wi-Fi e o BLE; depois o display e o audio.
    // ORDEM IMPORTANTE (a RAM do ESP32 e curta): primeiro o que mais precisa de memoria
    // contigua, o Wi-Fi e o BLE; depois o display e o audio.
    // Libera a memoria do Bluetooth classico (so usamos BLE).
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    Serial.println("Heap livre no inicio: " + String(ESP.getFreeHeap()) + " bytes");

    ia_wifi_antecipar();

    // BLE (se faltar memoria, segue sem BLE em vez de travar a placa)
    if (ESP.getMaxAllocHeap() >= 60000)
    {
        Serial.println("BLE configurado com MTU = 200");
        BLE_setup();
        ble_ok = true;
    }
    else
    {
        Serial.println("AVISO: pouca memoria, BLE nao iniciado.");
    }
    Serial.println("Heap livre depois do BLE: " + String(ESP.getFreeHeap()) + " bytes");

    lvgl_setup();
    Serial.println("Heap livre depois do display: " + String(ESP.getFreeHeap()) + " bytes");

    splash_start_ms = millis();

    Serial.println("LVGL version:");
    Serial.print(LVGL_VERSION_MAJOR);
    Serial.print(".");
    Serial.print(LVGL_VERSION_MINOR);
    Serial.print(".");
    Serial.println(LVGL_VERSION_PATCH);

    // Audio (microfone + alto-falante)
    if (!audio_setup())
    {
        Serial.println("AVISO: audio nao iniciou. A assistente Sip ficara sem voz.");
    }
    Serial.println("Heap livre depois do audio: " + String(ESP.getFreeHeap()) + " bytes");

    // Assistente Sip (Wi-Fi + servidor no Mac) e narracao das telas
    ia_setup();
    narrador_setup();

    Serial.println("Heap livre depois de iniciar tudo: " + String(ESP.getFreeHeap()) + " bytes");
}


void loop()
{
    static unsigned long last_lvgl_update = 0;
    static unsigned long last_ble_update = 0;

    unsigned long agora = millis();


    // =================================================
    // LVGL
    // =================================================

    if (agora - last_lvgl_update >= 10)
    {
        last_lvgl_update = agora;

        lv_timer_handler();

        display_loop();
    }


    // =================================================
    // SENSOR
    // =================================================

    sensor_loop();


    // =================================================
    // SIP: narracao das telas, volume, comandos de voz
    // =================================================

    narrador_loop();


    // =================================================
    // BLE
    // =================================================

    if (ble_ok && agora - last_ble_update >= 100)
    {
        last_ble_update = agora;

        BLE_send(
            latest_bpm,
            latest_spo2,
            latest_glucose,
            true
        );
    }
}
