#ifndef SENSOR_H
#define SENSOR_H
#include <MAX3010x.h>
#include "filters.h"
#include <Wire.h>
#include "Display.h"
#include "BLEHandler.h"

// Pinos I2C do sensor MAX30102.
// O SDA foi movido do GPIO 21 (usado pelo backlight da tela) para o GPIO 27 (conector CN1).
#define SENSOR_SDA_PIN 27
#define SENSOR_SCL_PIN 22

MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;
const float kEdgeThreshold = -2000.0;

LowPassFilter low_pass_red(5.0, kSamplingFrequency);
LowPassFilter low_pass_ir(5.0, kSamplingFrequency);
HighPassFilter high_pass_red(0.5, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<5> averager_bpm;
MovingAverageFilter<5> averager_spo2;
MovingAverageFilter<5> averager_glucose;

MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

unsigned long last_display_update = 0;
const unsigned long kDisplayInterval = 1000;
int beat_counter = 0;
const int beats_per_stat_reset = 5;


// ================= EXTERNAS =================
extern void update_sensor_values(int bpm, float spo2);
extern void add_glucose_measurement(float g);

// ================= FUNÇÕES =================
void scan_i2c() {
    Serial.println("📡 Escaneando barramento I2C...");
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Dispositivo encontrado em 0x");
            if(address<16) Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
    }
    if (nDevices == 0) Serial.println("❌ Nenhum dispositivo I2C encontrado.");
    else Serial.println("✅ Escaneamento concluído.");
}

bool sensor_ok = false;
unsigned long sensor_ultima_tentativa = 0;

bool sensor_iniciar() {
    sensor_ultima_tentativa = millis();
    if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
        sensor_ok = true;
        Serial.println("✅ Sensor MAX30102 inicializado com sucesso!");
    } else {
        sensor_ok = false;
        Serial.println("❌ Erro: sensor MAX30102 não encontrado. Confira os fios (SDA=GPIO27, SCL=GPIO22, 3V3, GND). Tentando de novo a cada 5 s.");
    }
    return sensor_ok;
}

void sensor_setup() {
    Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
    scan_i2c();
    // Antes, se o sensor nao respondesse, a placa travava aqui para sempre (while(true))
    // e a tela de calibracao ficava congelada. Agora o resto do aparelho continua funcionando.
    sensor_iniciar();
}

// ================= CÁLCULO DE GLICOSE INTERPOLADO =================
struct GlucosePoint {
    int bpm;
    int spo2;
    int glucose;
};

// Tabela original + novo dado
const GlucosePoint glucose_table[] = {
    {68, 97, 102},
    {82, 98, 123},
    {74, 98, 114},
    {61, 92, 110},
    {78, 98, 117},
    {99, 98, 131},
    {71, 98, 113},
    {76, 94, 102},
    {71, 97, 104},
    {78, 97, 106},
    {107, 99, 219}  // novo dado sem prioridade
};
const int table_size = sizeof(glucose_table)/sizeof(glucose_table[0]);

float calcularGlicose(int bpm, int spo2) {
    GlucosePoint closest = glucose_table[0];
    int min_dist = 10000;

    // Procura o ponto mais próximo na tabela, ignorando 107/99
    for(int i=0;i<table_size;i++){
        if(glucose_table[i].bpm==107 && glucose_table[i].spo2==99) continue;
        int dist = abs(glucose_table[i].bpm - bpm) + abs(glucose_table[i].spo2 - spo2);
        if(dist < min_dist) {
            min_dist = dist;
            closest = glucose_table[i];
        }
    }

    // Interpolação simples
    int delta_bpm = bpm - closest.bpm;
    int delta_spo2 = spo2 - closest.spo2;
    float adj = delta_bpm * 0.5 + delta_spo2 * 0.5; // pesos ajustáveis
    float result = closest.glucose + adj;

    if(result < 50) result = 50;
    if(result > 240) result = 240;

    return result;
}

// ================= LOOP PRINCIPAL =================
void sensor_loop() {
    if(!sensor_ok) {
        if(millis() - sensor_ultima_tentativa > 5000) {
            Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
            sensor_iniciar();
        }
        return;
    }

    // Timeout curto: se o sensor nao mandar dados, a tela e os botoes continuam respondendo
    // (antes esperava ate 1 s por amostra e a tela congelava).
    auto sample = sensor.readSample(40);
    float red = sample.red;
    float ir = sample.ir;

    // --- DETECÇÃO DO DEDO ---
    if(red > kFingerThreshold) {
        if(millis() - finger_timestamp > kFingerCooldownMs)
            finger_detected = true;
    } else {
        differentiator.reset();
        averager_bpm.reset();
        averager_spo2.reset();
        averager_glucose.reset();
        low_pass_red.reset();
        low_pass_ir.reset();
        high_pass_red.reset();
        stat_red.reset();
        stat_ir.reset();
        finger_detected = false;
        finger_timestamp = millis();
        beat_counter = 0;
        return;
    }



    if(finger_detected) {
        // --- FILTROS ---
        red = low_pass_red.process(red);
        ir = low_pass_ir.process(ir);

        stat_red.process(red);
        stat_ir.process(ir);

        float filtered_red = high_pass_red.process(red);
        float diff = differentiator.process(filtered_red);

        // --- DETECÇÃO DE BATIMENTO ---
        if(!isnan(diff) && !isnan(last_diff)) {
            if(last_diff > 0 && diff < 0) {
                crossed = true;
                crossed_time = millis();
            }
            if(diff > 0) crossed = false;

            if(crossed && diff < kEdgeThreshold) {
                if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
                    int bpm = 60000 / (crossed_time - last_heartbeat);

                    float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
                    float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
                    float r = rred / rir;
                    float spo2 = constrain(104.0 - 17.0 * r, 85.0, 100.0);

                    int avg_bpm = averager_bpm.process(bpm);
                    float avg_spo2 = averager_spo2.process(spo2);

                    // --- CÁLCULO DE GLICOSE ---
                    float glicose = calcularGlicose(avg_bpm, avg_spo2);
                    float avg_glucose = averager_glucose.process(glicose);

                    // --- ATUALIZA DISPLAY ---
                    if(millis() - last_display_update > kDisplayInterval) {
                        last_display_update = millis();
                        update_sensor_values(avg_bpm, avg_spo2);
                        add_glucose_measurement(avg_glucose);
                    }

                    beat_counter++;
                    if(beat_counter >= beats_per_stat_reset) {
                        stat_red.reset();
                        stat_ir.reset();
                        beat_counter = 0;
                    }
                }
                crossed = false;
                last_heartbeat = crossed_time;
            }
        }
        last_diff = diff;
    }
}

#endif
