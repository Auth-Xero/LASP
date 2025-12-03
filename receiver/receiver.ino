#include <Arduino.h>
#include "heltec.h"
#include <RadioLib.h>
#include <EEPROM.h>
#include <WiFi.h>
#include "HT_SSD1306Wire.h"
#include <driver/i2s.h>

SSD1306Wire audioDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

#define LORA_FREQUENCY 915.0
#define LORA_BANDWIDTH 500.0
#define LORA_SPREADING 5
#define LORA_CODING_RATE 5
#define LORA_TX_POWER 21
#define LORA_PREAMBLE 6
#define LORA_SYNC_WORD 0x12

#define LORA_NSS 8
#define LORA_IRQ 14
#define LORA_RST 12
#define LORA_BUSY 13
#define USER_BUTTON 0
#define LED_PIN 35

#define I2S_BCK_PIN 5
#define I2S_WS_PIN 7
#define I2S_DATA_PIN 6

#define SAMPLE_RATE 8000
#define I2S_NUM I2S_NUM_0

#define COMPRESSED_PACKET_SIZE 240
#define AUDIO_SAMPLES_PER_PACKET 480
#define PACKET_HEADER_SIZE 3
#define TOTAL_PACKET_SIZE (PACKET_HEADER_SIZE + COMPRESSED_PACKET_SIZE)

#define AUDIO_BUFFER_SIZE 19200
#define MIN_START_THRESHOLD 7200
#define MAX_START_THRESHOLD 9600
#define MIN_STOP_THRESHOLD 240

const int16_t ulaw4_decode_table[16] = {
    0,      
    520,    
    1056,   
    1616,   
    2208,   
    2848,   
    3552,   
    4336,   
    5216,   
    6208,   
    7328,   
    8592,   
    10016,  
    11616,  
    13408,  
    15408   
};

int16_t ulaw4Decode(uint8_t ulaw4, bool sign) {
    int16_t decoded = ulaw4_decode_table[ulaw4 & 0x0F];
    return sign ? -decoded : decoded;
}

class SimpleDCBlocker {
private:
    float lastInput;
    float lastOutput;
    float alpha;

public:
    SimpleDCBlocker() : lastInput(0), lastOutput(0), alpha(0.995f) {}

    void reset() {
        lastInput = 0;
        lastOutput = 0;
    }

    int16_t process(int16_t input) {
        float output = input - lastInput + alpha * lastOutput;
        lastInput = input;
        lastOutput = output;

        if (output > 32000.0f) output = 32000.0f;
        if (output < -32000.0f) output = -32000.0f;

        return (int16_t)output;
    }
};

class SimpleLowPassFilter {
private:
    float a0, a1, a2;
    float b0, b1, b2;
    float x1, x2;
    float y1, y2;

public:
    SimpleLowPassFilter() {
        float sampleRate = 8000.0f;
        float cutoffFreq = 3400.0f;

        float omega = 2.0f * 3.14159265f * cutoffFreq / sampleRate;
        float sinOmega = sin(omega);
        float cosOmega = cos(omega);

        float Q = 0.707f;
        float alpha = sinOmega / (2.0f * Q);

        a0 = 1.0f + alpha;
        b0 = ((1.0f - cosOmega) / 2.0f) / a0;
        b1 = (1.0f - cosOmega) / a0;
        b2 = ((1.0f - cosOmega) / 2.0f) / a0;
        a1 = (-2.0f * cosOmega) / a0;
        a2 = (1.0f - alpha) / a0;

        reset();
    }

    void reset() {
        x1 = x2 = 0;
        y1 = y2 = 0;
    }

    int16_t process(int16_t input) {
        float x0 = (float)input;
        float y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

        x2 = x1;
        x1 = x0;
        y2 = y1;
        y1 = y0;

        if (y0 > 32767.0f) y0 = 32767.0f;
        if (y0 < -32768.0f) y0 = -32768.0f;

        return (int16_t)y0;
    }
};

SimpleDCBlocker dcBlocker;
SimpleLowPassFilter audioFilter;

SX1262 radio = new Module(LORA_NSS, LORA_IRQ, LORA_RST, LORA_BUSY);

uint16_t myDeviceId;
uint16_t pairedDeviceId = 0;
bool isPaired = false;
bool pairingMode = false;
unsigned long pairingStartTime = 0;

uint8_t audioRingBuffer[AUDIO_BUFFER_SIZE];
volatile int writeIndex = 0;
volatile int readIndex = 0;
volatile int bufferLevel = 0;
volatile bool audioPlaying = false;
portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastLedToggle = 0;
const unsigned long LED_TOGGLE_INTERVAL = 100;
bool ledState = false;

volatile bool buttonCheckNeeded = false;
unsigned long lastButtonCheck = 0;

struct AdaptiveBufferParams {
    int startThreshold;
    int stopThreshold;
    int targetBufferLevel;
    float avgPacketInterval;
    float dataRate;
    int nominalPlayInterval;

    unsigned long totalAirTimeMicros;
    unsigned long minAirTimeMicros;
    unsigned long maxAirTimeMicros;
    unsigned long packetCount;
    unsigned long lastAirTimeMs;

    float jitter;
    int consecutiveDrops;
    int qualityScore;

    unsigned long lastPacketTimes[10];
    int timeIndex;
    bool historyFull;

    int stablePlaybackCount;
    bool initialBufferingComplete;
    unsigned long lastUnderrunTime;
} bufferParams;

unsigned long packetsReceived = 0;
unsigned long packetsDropped = 0;
unsigned long samplesDecompressed = 0;
unsigned long bufferUnderruns = 0;
unsigned long bufferOverruns = 0;
unsigned long bytesReceived = 0;
uint8_t lastSequence = 0;
int lastRSSI = 0;
float lastSNR = 0;

bool debugMode = true;

TaskHandle_t audioTaskHandle = NULL;
TaskHandle_t radioTaskHandle = NULL;

QueueHandle_t audioPacketQueue;

typedef struct {
    uint8_t compressedData[COMPRESSED_PACKET_SIZE];
    uint8_t sequence;
    unsigned long timestamp;
    unsigned long airTimeMicros;
} AudioPacket_t;

volatile bool receivedFlag = false;
volatile unsigned long lastReceiveMicros = 0;

unsigned long lastWatchdog = 0;
const unsigned long WATCHDOG_TIMEOUT = 10000;

bool needsFadeIn = false;
int fadeInSamples = 0;

IRAM_ATTR void setFlag(void) {
    receivedFlag = true;
    lastReceiveMicros = micros();
}

void decompressAudioBlock(uint8_t* input, uint8_t* output) {

    for (int i = 0; i < COMPRESSED_PACKET_SIZE; i++) {
        uint8_t byte = input[i];

        uint8_t sample1 = byte & 0x0F;
        bool sign1 = (byte & 0x08) != 0;
        int16_t decoded1 = ulaw4Decode(sample1 & 0x07, sign1);

        uint8_t sample2 = (byte >> 4) & 0x0F;
        bool sign2 = (sample2 & 0x08) != 0;
        int16_t decoded2 = ulaw4Decode(sample2 & 0x07, sign2);

        output[i * 2] = (uint8_t)((decoded1 / 256) + 128);
        output[i * 2 + 1] = (uint8_t)((decoded2 / 256) + 128);
    }

    samplesDecompressed += AUDIO_SAMPLES_PER_PACKET;
}

void VextON(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(100);
}

void updateLed(bool forceState = false, bool state = false) {
    unsigned long now = millis();

    if (!forceState && (now - lastLedToggle < LED_TOGGLE_INTERVAL)) {
        return;
    }

    if (forceState) {
        digitalWrite(LED_PIN, state);
        ledState = state;
    } else {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }

    lastLedToggle = now;
}

void setup() {
    Serial.begin(115200);
    delay(100);

    VextON();

    dcBlocker.reset();
    audioFilter.reset();

    initAdaptiveParams();

    audioDisplay.init();
    audioDisplay.clear();
    audioDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
    audioDisplay.setFont(ArialMT_Plain_16);
    audioDisplay.drawString(64, 10, "LoRa Audio");
    audioDisplay.setFont(ArialMT_Plain_10);
    audioDisplay.drawString(64, 30, "4-bit μ-law RX");
    audioDisplay.drawString(64, 45, "Starting...");
    audioDisplay.display();

    Heltec.begin(false, false, true, false, LORA_FREQUENCY);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(USER_BUTTON, INPUT_PULLUP);

    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    delay(100);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    myDeviceId = (mac[4] << 8) | mac[5];

    EEPROM.begin(512);
    loadPairingInfo();

    audioPacketQueue = xQueueCreate(20, sizeof(AudioPacket_t));
    if (audioPacketQueue == NULL) {
        Serial.println("Failed to create packet queue!");
        while (1);
    }

    initI2S();
    initLoRa();

    lastWatchdog = millis();

    Serial.println("\n=== LoRa Audio Receiver - 4-bit μ-law 8kHz ===");
    Serial.printf("Device ID: 0x%04X\n", myDeviceId);
    Serial.printf("Sample Rate: %d Hz\n", SAMPLE_RATE);
    Serial.printf("4-bit μ-law: Stateless compression\n");
    Serial.printf("480 samples per packet\n");
    Serial.printf("Packet loss resilient - no state corruption!\n\n");

    if (isPaired) {
        Serial.printf("Paired with transmitter: 0x%04X\n", pairedDeviceId);
        Serial.println("Ready to receive μ-law audio\n");
    } else {
        Serial.println("Not paired - entering pairing mode\n");
        delay(1000);
        enterPairingMode();
    }

    xTaskCreatePinnedToCore(audioOutputTask, "Audio", 4096, NULL, 24, &audioTaskHandle, 1);
    xTaskCreatePinnedToCore(radioReceiveTask, "Radio", 4096, NULL, 20, &radioTaskHandle, 0);
    xTaskCreatePinnedToCore(packetProcessorTask, "Processor", 4096, NULL, 22, NULL, 0);
    xTaskCreatePinnedToCore(adaptiveBufferTask, "Adaptive", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(watchdogTask, "Watchdog", 2048, NULL, 5, NULL, 0);

    Serial.println("All tasks started - system ready\n");
}

void loop() {
    unsigned long now = millis();

    if (now - lastButtonCheck > 50) {
        //handleButton();
        lastButtonCheck = now;
    }

    static unsigned long lastDisplayUpdate = 0;
    if (now - lastDisplayUpdate > 500) {
        updateDisplay();
        lastDisplayUpdate = now;
    }

    static unsigned long lastStatusReport = 0;
    if (now - lastStatusReport > 5000 && packetsReceived > 0) {
        printStatusReport();
        lastStatusReport = now;
    }

    lastWatchdog = now;
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

void watchdogTask(void* parameter) {
    Serial.println("Watchdog task started");

    while (true) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        unsigned long now = millis();
        if (now - lastWatchdog > WATCHDOG_TIMEOUT) {
            Serial.println("\n⚠️ WATCHDOG: System appears hung, attempting recovery...");

            portENTER_CRITICAL(&bufferMux);
            audioPlaying = false;
            bufferLevel = 0;
            writeIndex = 0;
            readIndex = 0;
            portEXIT_CRITICAL(&bufferMux);

            audioFilter.reset();
            dcBlocker.reset();

            radio.setDio1Action(setFlag);
            radio.startReceive();

            lastWatchdog = now;
            Serial.println("Watchdog: Recovery attempted\n");
        }
    }
}

void initAdaptiveParams() {
    float samplesPerPacket = (float)AUDIO_SAMPLES_PER_PACKET;
    float msPerPacket = (samplesPerPacket * 1000.0) / (float)SAMPLE_RATE;
    float packetsPerSec = 1000.0 / msPerPacket;

    bufferParams.startThreshold = 7200;
    bufferParams.stopThreshold = 240;
    bufferParams.targetBufferLevel = 8000;
    bufferParams.nominalPlayInterval = (int)(msPerPacket + 0.5);
    bufferParams.avgPacketInterval = msPerPacket;
    bufferParams.dataRate = packetsPerSec;
    bufferParams.jitter = 0.0;
    bufferParams.consecutiveDrops = 0;
    bufferParams.qualityScore = 50;
    bufferParams.timeIndex = 0;
    bufferParams.historyFull = false;
    bufferParams.totalAirTimeMicros = 0;
    bufferParams.minAirTimeMicros = 999999999;
    bufferParams.maxAirTimeMicros = 0;
    bufferParams.packetCount = 0;
    bufferParams.lastAirTimeMs = 0;
    bufferParams.stablePlaybackCount = 0;
    bufferParams.initialBufferingComplete = false;
    bufferParams.lastUnderrunTime = 0;

    memset(bufferParams.lastPacketTimes, 0, sizeof(bufferParams.lastPacketTimes));
}

void adaptiveBufferTask(void* parameter) {
    Serial.println("Adaptive buffer task started");

    while (true) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        if (packetsReceived < 20) {
            continue;
        }

        updateAdaptiveMetrics();
        adjustBufferThresholds();

        if (debugMode && packetsReceived % 200 == 0) {
            printAdaptiveStatus();
        }
    }
}

void updateAdaptiveMetrics() {
    if (bufferParams.historyFull) {
        float sum = 0;
        float sumSquares = 0;
        int validIntervals = 0;

        for (int i = 1; i < 10; i++) {
            if (bufferParams.lastPacketTimes[i] > 0 && bufferParams.lastPacketTimes[i - 1] > 0) {
                float interval = bufferParams.lastPacketTimes[i] - bufferParams.lastPacketTimes[i - 1];
                if (interval > 40 && interval < 80) {
                    sum += interval;
                    sumSquares += interval * interval;
                    validIntervals++;
                }
            }
        }

        if (validIntervals > 3) {
            float avg = sum / validIntervals;
            float variance = (sumSquares / validIntervals) - (avg * avg);

            bufferParams.avgPacketInterval = 0.8 * bufferParams.avgPacketInterval + 0.2 * avg;
            bufferParams.jitter = 0.8 * bufferParams.jitter + 0.2 * sqrt(max(0.0f, variance));

            if (bufferParams.avgPacketInterval > 40 && bufferParams.avgPacketInterval < 80) {
                bufferParams.dataRate = 1000.0 / bufferParams.avgPacketInterval;
            }
        }
    }

    int score = 100;

    if (packetsReceived > 0) {
        float dropRate = (float)packetsDropped / (packetsReceived + packetsDropped);
        score -= (int)(dropRate * 40);
    }

    if (bufferParams.jitter > 5) score -= 10;
    if (bufferParams.jitter > 10) score -= 20;

    if (bufferUnderruns > 0) {
        score -= min(30, (int)bufferUnderruns * 5);

        if (millis() - bufferParams.lastUnderrunTime < 10000) {
            score -= 20;
        }
    }

    if (bufferParams.stablePlaybackCount > 20) score += 10;
    if (bufferParams.stablePlaybackCount > 50) score += 10;

    if (lastRSSI < -110) score -= 15;
    else if (lastRSSI < -100) score -= 5;

    if (bufferParams.lastAirTimeMs > 0) {
        if (bufferParams.lastAirTimeMs > 70) score -= 15;
        else if (bufferParams.lastAirTimeMs < 50) score += 10;
    }

    bufferParams.qualityScore = max(0, min(100, score));
}

void adjustBufferThresholds() {
    if (bufferParams.stablePlaybackCount > 30) {
        if (bufferParams.qualityScore >= 90 && bufferUnderruns == 0) {
            int reduction = AUDIO_SAMPLES_PER_PACKET;
            bufferParams.startThreshold = max(MIN_START_THRESHOLD,
                                            bufferParams.startThreshold - reduction);
            bufferParams.targetBufferLevel = bufferParams.startThreshold + (AUDIO_SAMPLES_PER_PACKET * 5);
        }
        return;
    }

    int basePackets;
    if (bufferParams.qualityScore >= 85) {
        basePackets = 8;
    } else if (bufferParams.qualityScore >= 70) {
        basePackets = 10;
    } else if (bufferParams.qualityScore >= 50) {
        basePackets = 12;
    } else {
        basePackets = 15;
    }

    if (bufferParams.jitter > 3) {
        int jitterPackets = min(3, (int)(bufferParams.jitter / 5));
        basePackets += jitterPackets;
    }

    if (bufferParams.dataRate < 16 && bufferParams.dataRate > 10) {
        basePackets += 2;
    }

    bufferParams.startThreshold = basePackets * AUDIO_SAMPLES_PER_PACKET;
    bufferParams.startThreshold = max(MIN_START_THRESHOLD,
                                      min(MAX_START_THRESHOLD, bufferParams.startThreshold));

    bufferParams.stopThreshold = max(MIN_STOP_THRESHOLD, AUDIO_SAMPLES_PER_PACKET * 2);
    bufferParams.targetBufferLevel = bufferParams.startThreshold + (AUDIO_SAMPLES_PER_PACKET * 5);
    bufferParams.targetBufferLevel = min(bufferParams.targetBufferLevel, 14400);
}

void recordPacketTiming(unsigned long timestamp, unsigned long airTimeMicros) {
    bufferParams.lastPacketTimes[bufferParams.timeIndex] = timestamp;
    bufferParams.timeIndex = (bufferParams.timeIndex + 1) % 10;
    if (bufferParams.timeIndex == 0) {
        bufferParams.historyFull = true;
    }

    if (airTimeMicros > 0 && airTimeMicros < 200000) {
        bufferParams.totalAirTimeMicros += airTimeMicros;
        bufferParams.packetCount++;
        bufferParams.lastAirTimeMs = airTimeMicros / 1000.0;

        if (airTimeMicros < bufferParams.minAirTimeMicros) {
            bufferParams.minAirTimeMicros = airTimeMicros;
        }
        if (airTimeMicros > bufferParams.maxAirTimeMicros) {
            bufferParams.maxAirTimeMicros = airTimeMicros;
        }
    }
}

void initI2S() {
    Serial.println("Initializing I2S DAC for 8kHz playback...");

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 480,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_DATA_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    esp_err_t err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("I2S driver install failed: %d\n", err);
    }

    err = i2s_set_pin(I2S_NUM, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("I2S pin configuration failed: %d\n", err);
    }

    i2s_zero_dma_buffer(I2S_NUM);
    Serial.println("I2S initialized for 8kHz 4-bit μ-law audio");
}

void audioOutputTask(void* parameter) {
    Serial.println("Audio output task started - 8kHz 4-bit μ-law");

    int16_t i2sBuffer[480];
    unsigned long lastPlayTime = 0;
    unsigned long stablePlaybackStart = 0;
    int underrunCount = 0;
    int consecutiveGoodPackets = 0;

    while (true) {
        unsigned long now = millis();

        portENTER_CRITICAL(&bufferMux);
        int currentLevel = bufferLevel;
        portEXIT_CRITICAL(&bufferMux);

        if (!audioPlaying && currentLevel >= bufferParams.startThreshold) {
            audioPlaying = true;
            stablePlaybackStart = now;
            bufferParams.initialBufferingComplete = true;
            underrunCount = 0;
            consecutiveGoodPackets = 0;
            needsFadeIn = true;
            fadeInSamples = 0;

            audioFilter.reset();
            dcBlocker.reset();

            Serial.println("▶️  Playback starting (8kHz 4-bit μ-law)");
            lastPlayTime = now;
        }

        if (audioPlaying) {
            const int playbackInterval = (AUDIO_SAMPLES_PER_PACKET * 1000) / SAMPLE_RATE;

            if (now - lastPlayTime >= playbackInterval) {
                if (currentLevel >= AUDIO_SAMPLES_PER_PACKET) {

                    portENTER_CRITICAL(&bufferMux);
                    for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++) {
                        uint8_t sample8 = audioRingBuffer[readIndex];

                        int16_t centered = (int16_t)sample8 - 128;
                        int16_t sample16 = centered * 256;

                        if (needsFadeIn) {
                            float fadeGain = min(1.0f, (fadeInSamples + i) / 240.0f);
                            sample16 = (int16_t)(sample16 * fadeGain);
                        }

                        sample16 = dcBlocker.process(sample16);
                        sample16 = audioFilter.process(sample16);

                        if (sample16 > 24000) {
                            sample16 = 24000 + (sample16 - 24000) / 4;
                        } else if (sample16 < -24000) {
                            sample16 = -24000 + (sample16 + 24000) / 4;
                        }

                        i2sBuffer[i] = sample16;

                        readIndex = (readIndex + 1) % AUDIO_BUFFER_SIZE;
                        bufferLevel--;
                    }
                    portEXIT_CRITICAL(&bufferMux);

                    if (needsFadeIn) {
                        fadeInSamples += AUDIO_SAMPLES_PER_PACKET;
                        if (fadeInSamples >= 240) {
                            needsFadeIn = false;
                        }
                    }

                    consecutiveGoodPackets++;

                    size_t bytesWritten;
                    i2s_write(I2S_NUM, i2sBuffer, sizeof(i2sBuffer), &bytesWritten, portMAX_DELAY);

                    lastPlayTime = now;

                    if (now - stablePlaybackStart > 3000) {
                        bufferParams.stablePlaybackCount++;
                    }

                } else if (currentLevel < bufferParams.stopThreshold) {
                    audioPlaying = false;
                    bufferUnderruns++;
                    underrunCount++;
                    bufferParams.lastUnderrunTime = now;
                    bufferParams.stablePlaybackCount = 0;

                    audioFilter.reset();
                    dcBlocker.reset();

                    Serial.printf("\n⚠️  UNDERRUN #%lu\n", bufferUnderruns);

                    if (underrunCount > 2) {
                        bufferParams.startThreshold = min(MAX_START_THRESHOLD,
                                                          bufferParams.startThreshold + AUDIO_SAMPLES_PER_PACKET * 2);
                    } else {
                        bufferParams.startThreshold = min(MAX_START_THRESHOLD,
                                                          bufferParams.startThreshold + AUDIO_SAMPLES_PER_PACKET);
                    }
                    bufferParams.targetBufferLevel = bufferParams.startThreshold + AUDIO_SAMPLES_PER_PACKET * 5;
                }
            }
        } else {
            memset(i2sBuffer, 0, sizeof(i2sBuffer));
            size_t bytesWritten;
            i2s_write(I2S_NUM, i2sBuffer, sizeof(i2sBuffer), &bytesWritten, 10);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void radioReceiveTask(void* parameter) {
    Serial.println("Radio receive task started");

    uint8_t packet[TOTAL_PACKET_SIZE];
    unsigned long lastPacketMicros = 0;
    int consecutiveReceives = 0;
    int packetsThisSecond = 0;
    unsigned long secondStart = millis();
    int crcErrors = 0;

    while (true) {
        if (pairingMode) {
            handlePairingMode();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        if (receivedFlag) {
            receivedFlag = false;

            unsigned long receiveTime = millis();
            unsigned long receiveMicros = micros();

            int state = radio.readData(packet, TOTAL_PACKET_SIZE);

            if (state == RADIOLIB_ERR_NONE) {
                lastRSSI = radio.getRSSI();
                lastSNR = radio.getSNR();
                consecutiveReceives++;
                packetsThisSecond++;

                unsigned long airTimeMicros = 0;
                if (lastPacketMicros > 0 && receiveMicros > lastPacketMicros) {
                    airTimeMicros = receiveMicros - lastPacketMicros;
                }
                lastPacketMicros = receiveMicros;

                uint16_t targetId = (packet[0] << 8) | packet[1];
                bool packetForUs = !isPaired || (targetId == myDeviceId);

                if (packetForUs) {
                    uint8_t sequence = packet[2];

                    AudioPacket_t audioPacket;
                    audioPacket.sequence = sequence;
                    audioPacket.timestamp = receiveTime;
                    audioPacket.airTimeMicros = airTimeMicros;
                    memcpy(audioPacket.compressedData, &packet[3], COMPRESSED_PACKET_SIZE);

                    if (xQueueSend(audioPacketQueue, &audioPacket, 0) != pdTRUE) {
                        packetsDropped++;
                    }
                }
            } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                crcErrors++;
            }

            radio.startReceive();

        } else {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        unsigned long now = millis();
        if (now - secondStart >= 1000) {
            if (packetsThisSecond > 0 || crcErrors > 0) {
                Serial.printf("RX: %d good, %d CRC err\n", packetsThisSecond, crcErrors);
            }
            packetsThisSecond = 0;
            crcErrors = 0;
            secondStart = now;
        }
    }
}

void packetProcessorTask(void* parameter) {
    Serial.println("Packet processor task started");

    AudioPacket_t audioPacket;
    int sequenceErrors = 0;
    uint8_t decompressedBuffer[AUDIO_SAMPLES_PER_PACKET];
    uint8_t lastValidSequence = 0;
    bool firstPacket = true;

    while (true) {
        if (xQueueReceive(audioPacketQueue, &audioPacket, portMAX_DELAY) == pdTRUE) {
            recordPacketTiming(audioPacket.timestamp, audioPacket.airTimeMicros);

            if (!firstPacket) {
                uint8_t expectedSeq = (lastValidSequence + 1) & 0xFF;

                if (audioPacket.sequence != expectedSeq) {
                    int gap = (audioPacket.sequence - expectedSeq) & 0xFF;

                    if (gap > 0 && gap < 50) {
                        packetsDropped += gap;
                        sequenceErrors++;

                        Serial.printf("⚠️  Gap: %d packets (μ-law handles gracefully)\n", gap);

                        needsFadeIn = true;
                        fadeInSamples = 0;

                        bufferParams.consecutiveDrops++;
                        bufferParams.stablePlaybackCount = max(0, bufferParams.stablePlaybackCount - 2);
                    }
                } else {
                    bufferParams.consecutiveDrops = 0;
                }
            }

            lastValidSequence = audioPacket.sequence;
            firstPacket = false;

            decompressAudioBlock(audioPacket.compressedData, decompressedBuffer);

            portENTER_CRITICAL(&bufferMux);
            int spaceAvailable = AUDIO_BUFFER_SIZE - bufferLevel;

            if (spaceAvailable >= AUDIO_SAMPLES_PER_PACKET) {
                for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++) {
                    audioRingBuffer[writeIndex] = decompressedBuffer[i];
                    writeIndex = (writeIndex + 1) % AUDIO_BUFFER_SIZE;
                    bufferLevel++;
                }
            } else {
                bufferOverruns++;
                bufferParams.stablePlaybackCount = 0;

                int toRemove = AUDIO_SAMPLES_PER_PACKET - spaceAvailable;
                readIndex = (readIndex + toRemove) % AUDIO_BUFFER_SIZE;
                bufferLevel -= toRemove;

                for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++) {
                    audioRingBuffer[writeIndex] = decompressedBuffer[i];
                    writeIndex = (writeIndex + 1) % AUDIO_BUFFER_SIZE;
                    bufferLevel++;
                }
            }
            portEXIT_CRITICAL(&bufferMux);

            packetsReceived++;
            bytesReceived += COMPRESSED_PACKET_SIZE;
            updateLed();

            if (packetsReceived % 50 == 0) {
                Serial.printf("RX #%lu: buff=%d%% (4-bit μ-law)\n",
                            packetsReceived,
                            (bufferLevel * 100) / AUDIO_BUFFER_SIZE);
            }
        }
    }
}

void printAdaptiveStatus() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║   8kHz 4-bit μ-law ADAPTIVE STATUS    ║");
    Serial.println("╠════════════════════════════════════════╣");

    float avgAirMs = bufferParams.packetCount > 0 ? bufferParams.totalAirTimeMicros / (1000.0 * bufferParams.packetCount) : 0;

    Serial.printf("║ Packet rate: %.1f/s                    ║\n", bufferParams.dataRate);
    Serial.printf("║ Air time: %.1fms avg                   ║\n", avgAirMs);
    Serial.printf("║ Jitter: %.1fms                         ║\n", bufferParams.jitter);
    Serial.printf("║ Quality: %d/100                        ║\n", bufferParams.qualityScore);
    Serial.printf("║ Buffer start: %d packets               ║\n",
                  bufferParams.startThreshold / AUDIO_SAMPLES_PER_PACKET);
    Serial.println("╚════════════════════════════════════════╝\n");
}

void printStatusReport() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║    8kHz 4-bit μ-law RECEIVER STATUS   ║");
    Serial.println("╠════════════════════════════════════════╣");

    float lossRate = packetsReceived > 0 ? (100.0 * packetsDropped / (packetsReceived + packetsDropped)) : 0;

    Serial.printf("║ Packets: %lu received                  ║\n", packetsReceived);
    Serial.printf("║ Loss: %.1f%%                           ║\n", lossRate);

    portENTER_CRITICAL(&bufferMux);
    int level = bufferLevel;
    bool playing = audioPlaying;
    portEXIT_CRITICAL(&bufferMux);

    Serial.printf("║ Buffer: %.1f%%                         ║\n",
                  (100.0 * level / AUDIO_BUFFER_SIZE));
    Serial.printf("║ Status: %s                             ║\n",
                  playing ? "PLAYING" : "BUFFERING");
    Serial.printf("║ Underruns: %lu                         ║\n", bufferUnderruns);
    Serial.printf("║ Signal: %d dBm                         ║\n", lastRSSI);
    Serial.printf("║ Quality: %d/100                        ║\n", bufferParams.qualityScore);

    Serial.println("╚════════════════════════════════════════╝\n");
}

void handleButton() {
    static unsigned long buttonPressStart = 0;
    static bool buttonPressed = false;
    static bool longPressHandled = false;
    static bool veryLongPressHandled = false;

    if (digitalRead(USER_BUTTON) == LOW) {
        if (!buttonPressed) {
            buttonPressed = true;
            buttonPressStart = millis();
            longPressHandled = false;
            veryLongPressHandled = false;
        } else {
            unsigned long pressDuration = millis() - buttonPressStart;
            
            // Very long press (5+ seconds) - force clear pairing and enter pairing mode
            if (!veryLongPressHandled && pressDuration > 5000) {
                Serial.println("\n=== FORCE PAIRING MODE ===");
                Serial.println("Clearing ALL pairing data...");
                
                // Force clear pairing
                isPaired = false;
                pairedDeviceId = 0;
                
                // Clear EEPROM
                EEPROM.write(0, 0x00);
                EEPROM.write(1, 0x00);
                EEPROM.write(2, 0x00);
                EEPROM.commit();
                
                Serial.println("Pairing cleared - entering pairing mode");
                enterPairingMode();
                veryLongPressHandled = true;
            }
            // Normal long press (3+ seconds) - enter pairing mode
            else if (!longPressHandled && pressDuration > 3000 && !pairingMode && !veryLongPressHandled) {
                Serial.println("Button held 3s - entering pairing mode");
                enterPairingMode();
                longPressHandled = true;
            }
        }
    } else {
        if (buttonPressed) {
            unsigned long pressDuration = millis() - buttonPressStart;
            
            // Short press - print status
            if (!longPressHandled && !veryLongPressHandled && pressDuration < 3000) {
                printStatusReport();
            }
        }
        buttonPressed = false;
        longPressHandled = false;
        veryLongPressHandled = false;
    }
}

void initLoRa() {
    Serial.println("Initializing LoRa radio...");

    int state = radio.begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH,
        LORA_SPREADING,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio initialization failed: %d\n", state);
        while (1);
    }

    state = radio.setCRC(false);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Failed to enable CRC: %d\n", state);
    }

    state = radio.implicitHeader(TOTAL_PACKET_SIZE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Implicit header failed: %d\n", state);
    }

    state = radio.fixedPacketLengthMode(TOTAL_PACKET_SIZE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Fixed packet mode: %d\n", state);
    }

    delay(100);

    radio.setDio1Action(setFlag);
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Failed to start receive: %d\n", state);
    }

    Serial.println("LoRa radio ready");
}

void enterPairingMode() {
    Serial.println("\n=== ENTERING PAIRING MODE ===");

    // Stop audio playback and clear buffers
    portENTER_CRITICAL(&bufferMux);
    audioPlaying = false;
    bufferLevel = 0;
    writeIndex = 0;
    readIndex = 0;
    portEXIT_CRITICAL(&bufferMux);

    xQueueReset(audioPacketQueue);
    initAdaptiveParams();

    // Clear existing pairing to allow pairing with ANY transmitter
    if (isPaired) {
        Serial.printf("Clearing existing pairing with 0x%04X\n", pairedDeviceId);
    }
    
    pairingMode = true;
    pairingStartTime = millis();
    isPaired = false;  // Clear pairing flag
    pairedDeviceId = 0;  // Clear paired device ID

    // Clear EEPROM pairing - don't save until new pairing succeeds
    EEPROM.write(0, 0x00);  // Clear the valid flag
    EEPROM.write(1, 0x00);
    EEPROM.write(2, 0x00);
    EEPROM.commit();

    // Reset radio to ensure clean state
    radio.standby();
    delay(100);
    radio.startReceive();

    updateLed(true, true);
    Serial.println("Cleared old pairing - listening for ANY transmitter beacon...");
    Serial.println("Will accept pairing from any transmitter");
}

void handlePairingMode() {
    if (millis() - pairingStartTime > 30000) {
        Serial.println("Pairing timeout - no transmitter found");
        pairingMode = false;
        updateLed(true, false);
        
        // Restore previous pairing if it existed
        loadPairingInfo();
        if (isPaired) {
            Serial.printf("Restored previous pairing with 0x%04X\n", pairedDeviceId);
        }
        
        // Restart receiving
        radio.standby();
        delay(50);
        radio.startReceive();
        return;
    }

    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        updateLed();
        lastBlink = millis();

        int remaining = (30000 - (millis() - pairingStartTime)) / 1000;
        Serial.printf("Listening for ANY transmitter... %d seconds remaining\n", remaining);
    }

    uint8_t beacon[10];
    int state = radio.receive(beacon, sizeof(beacon), 100);

    if (state == RADIOLIB_ERR_NONE) {
        // Check if this is a beacon packet
        if (beacon[0] == 0xAA && beacon[1] == 0xAA) {
            uint16_t beaconId = (beacon[2] << 8) | beacon[3];
            uint8_t deviceType = beacon[4];

            // Check if it's from a transmitter (device type 0x01)
            if (deviceType == 0x01) {
                Serial.printf("\n✓ Found transmitter: 0x%04X\n", beaconId);
                
                // IMPORTANT: We accept ANY transmitter during pairing mode
                Serial.printf("Accepting pairing from transmitter 0x%04X\n", beaconId);

                // Send response immediately
                uint8_t response[6];
                response[0] = 0xBB;  // Response marker
                response[1] = 0xBB;
                response[2] = (myDeviceId >> 8) & 0xFF;  // Our ID (receiver)
                response[3] = myDeviceId & 0xFF;
                response[4] = (beaconId >> 8) & 0xFF;    // Their ID (transmitter we're responding to)
                response[5] = beaconId & 0xFF;

                // Transmit response
                int txState = radio.transmit(response, sizeof(response));
                if (txState == RADIOLIB_ERR_NONE) {
                    Serial.println("Response sent to transmitter");
                } else {
                    Serial.printf("Failed to send response: %d\n", txState);
                }

                // Go back to receive mode to listen for confirmation
                radio.startReceive();

                Serial.println("Waiting for confirmation from transmitter...");
                unsigned long confirmStart = millis();
                bool confirmed = false;
                int confirmationsReceived = 0;

                // Wait for confirmation packets (TX sends multiple)
                while (millis() - confirmStart < 5000 && !confirmed) {
                    uint8_t confirm[10];
                    state = radio.receive(confirm, sizeof(confirm), 100);

                    if (state == RADIOLIB_ERR_NONE) {
                        // Check if this is a confirmation packet
                        if (confirm[0] == 0xCC && confirm[1] == 0xCC) {
                            uint16_t txId = (confirm[2] << 8) | confirm[3];
                            uint16_t rxId = (confirm[4] << 8) | confirm[5];

                            // Verify the confirmation is for us from the transmitter we responded to
                            if (rxId == myDeviceId && txId == beaconId) {
                                confirmationsReceived++;
                                Serial.printf("Confirmation %d received from 0x%04X\n", 
                                            confirmationsReceived, txId);

                                if (!confirmed) {
                                    // Set new pairing
                                    pairedDeviceId = beaconId;
                                    isPaired = true;
                                    savePairingInfo();

                                    Serial.println("\n╔════════════════════════════════════════╗");
                                    Serial.println("║      ✓ PAIRING SUCCESSFUL!            ║");
                                    Serial.printf("║  Paired with transmitter: 0x%04X      ║\n", pairedDeviceId);
                                    Serial.println("║  4-bit μ-law audio ready               ║");
                                    Serial.println("╚════════════════════════════════════════╝\n");

                                    confirmed = true;
                                    pairingMode = false;
                                    updateLed(true, false);

                                    // Give time for any remaining confirmations
                                    delay(500);
                                    
                                    // Restart normal receiving
                                    radio.standby();
                                    delay(100);
                                    radio.startReceive();
                                }
                            } else {
                                Serial.printf("Confirmation was for different RX (0x%04X) or from different TX (0x%04X)\n", 
                                            rxId, txId);
                            }
                        }
                    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
                        // Ignore timeout errors during confirmation wait
                        Serial.printf("Error during confirmation wait: %d\n", state);
                    }

                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }

                if (!confirmed) {
                    Serial.println("No confirmation received - pairing failed");
                    // Continue in pairing mode to try again
                }
            } else {
                Serial.printf("Beacon from non-transmitter device type: 0x%02X\n", deviceType);
            }
        } else {
            // Not a beacon packet
            Serial.printf("Non-beacon packet received: %02X %02X\n", beacon[0], beacon[1]);
        }
    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.printf("Receive error: %d\n", state);
    }
}

uint8_t calculateChecksum(uint8_t* data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void loadPairingInfo() {
    uint8_t flag = EEPROM.read(0);
    if (flag == 0x55) {
        isPaired = true;
        pairedDeviceId = (EEPROM.read(1) << 8) | EEPROM.read(2);
        Serial.printf("Loaded saved pairing: 0x%04X\n", pairedDeviceId);
    } else {
        Serial.println("No saved pairing found");
    }
}

void savePairingInfo() {
    EEPROM.write(0, 0x55);
    EEPROM.write(1, (pairedDeviceId >> 8) & 0xFF);
    EEPROM.write(2, pairedDeviceId & 0xFF);
    EEPROM.commit();
    Serial.println("Pairing saved to EEPROM");
}

void updateDisplay() {
    audioDisplay.clear();

    audioDisplay.setFont(ArialMT_Plain_10);
    audioDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    audioDisplay.drawString(0, 0, "RX");

    if (isPaired) {
        audioDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
        audioDisplay.drawString(64, 0, "4b-μlaw");
    }

    audioDisplay.setTextAlignment(TEXT_ALIGN_RIGHT);
    if (lastRSSI != 0) {
        audioDisplay.drawString(128, 0, String(lastRSSI));
    }

    audioDisplay.setTextAlignment(TEXT_ALIGN_CENTER);

    portENTER_CRITICAL(&bufferMux);
    int level = bufferLevel;
    bool playing = audioPlaying;
    portEXIT_CRITICAL(&bufferMux);

    if (pairingMode) {
        audioDisplay.setFont(ArialMT_Plain_16);
        audioDisplay.drawString(64, 20, "Pairing...");
        audioDisplay.setFont(ArialMT_Plain_10);
        int remaining = (30000 - (millis() - pairingStartTime)) / 1000;
        audioDisplay.drawString(64, 40, String(remaining) + "s");
    } else if (playing) {
        audioDisplay.setFont(ArialMT_Plain_16);
        audioDisplay.drawString(64, 18, "♪ PLAYING ♪");

        audioDisplay.setFont(ArialMT_Plain_10);
        char infoStr[32];
        sprintf(infoStr, "Q:%d", bufferParams.qualityScore);
        audioDisplay.drawString(64, 35, infoStr);

        int barWidth = 100;
        int barHeight = 6;
        int barX = 14;
        int barY = 47;
        int fillWidth = (level * barWidth) / AUDIO_BUFFER_SIZE;
        int targetMarker = (bufferParams.targetBufferLevel * barWidth) / AUDIO_BUFFER_SIZE;

        audioDisplay.drawRect(barX, barY, barWidth, barHeight);
        audioDisplay.fillRect(barX, barY, fillWidth, barHeight);
        audioDisplay.drawVerticalLine(barX + targetMarker, barY - 1, 8);

        audioDisplay.drawString(64, 54, String((level * 100) / AUDIO_BUFFER_SIZE) + "%");
    } else if (isPaired) {
        audioDisplay.setFont(ArialMT_Plain_16);

        if (level > 0) {
            audioDisplay.drawString(64, 20, "Buffering");
            audioDisplay.setFont(ArialMT_Plain_10);
            int percent = (level * 100) / bufferParams.startThreshold;
            if (percent > 100) percent = 100;
            audioDisplay.drawString(64, 40, String(percent) + "%");
        } else {
            audioDisplay.drawString(64, 20, "Ready");
            audioDisplay.setFont(ArialMT_Plain_10);
            audioDisplay.drawString(64, 40, "4-bit μ-law");
        }
    } else {
        audioDisplay.setFont(ArialMT_Plain_16);
        audioDisplay.drawString(64, 20, "Not Paired");
        audioDisplay.setFont(ArialMT_Plain_10);
        audioDisplay.drawString(64, 40, "Hold button 3s");
    }

    audioDisplay.display();
}