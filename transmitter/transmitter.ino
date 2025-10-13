#include <Arduino.h>
#include "heltec.h"
#include <RadioLib.h>
#include <EEPROM.h>
#include <WiFi.h>
#include "HT_SSD1306Wire.h"

SSD1306Wire audioDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

#define LORA_FREQUENCY 915.0
#define LORA_BANDWIDTH 500.0
#define LORA_SPREADING 5
#define LORA_CODING_RATE 5
#define LORA_TX_POWER 20
#define LORA_PREAMBLE 6
#define LORA_SYNC_WORD 0x12

#define LORA_NSS 8
#define LORA_IRQ 14
#define LORA_RST 12
#define LORA_BUSY 13
#define USER_BUTTON 0
#define LED_PIN 35

#define AUDIO_SAMPLES_PER_PACKET 480  
#define COMPRESSED_PACKET_SIZE 240    
#define PACKET_HEADER_SIZE 3
#define TOTAL_PACKET_SIZE (PACKET_HEADER_SIZE + COMPRESSED_PACKET_SIZE)

#define FRAME_START_1 0x61
#define FRAME_START_2 0x6f
#define FRAME_START_3 0x73
#define FRAME_TYPE_AUDIO 0xAD
#define FRAME_TYPE_COMMAND 0x67
#define SERIAL_AUDIO_PACKET_SIZE 486  

const int16_t stepSizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

const int8_t indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

struct ADPCMState {
    int16_t prevSample;   
    int8_t index;         

    void reset() {
        prevSample = 0;
        index = 0;
    }
};

ADPCMState encoderState;

#define TX_QUEUE_SIZE 10
QueueHandle_t txQueue;

struct TxPacket {
    uint8_t data[COMPRESSED_PACKET_SIZE];  
    uint32_t timestamp;
};

#define SERIAL_BUFFER_SIZE 9600  
uint8_t serialBuffer[SERIAL_BUFFER_SIZE];
volatile int serialWriteIdx = 0;
volatile int serialReadIdx = 0;
volatile int serialBufferLevel = 0;
portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool audioStreamLocked = false;
volatile unsigned long firstAudioPacketTime = 0;
volatile unsigned long audioPacketsReceived = 0;

enum TestMode {
    MODE_SERIAL,
    MODE_SILENCE,
    MODE_SINE,
    MODE_SWEEP,
    MODE_NONE
};

TestMode currentMode = MODE_NONE;
float sinePhase = 0;
float sweepFreq = 100;

SX1262 radio = new Module(LORA_NSS, LORA_IRQ, LORA_RST, LORA_BUSY);

uint16_t myDeviceId;
uint16_t pairedDeviceId = 0;
bool isPaired = false;
uint8_t packetSequence = 0;
unsigned long packetsTransmitted = 0;
unsigned long packetsDropped = 0;
unsigned long lastTransmitTime = 0;
unsigned long testStartTime = 0;

unsigned long totalAirTimeMicros = 0;
unsigned long maxAirTimeMicros = 0;
unsigned long minAirTimeMicros = 999999999;
unsigned long lastAirTimeMs = 0;
unsigned long lastLedToggle = 0;
const unsigned long LED_TOGGLE_INTERVAL = 100;

bool pairingMode = false;
unsigned long pairingStartTime = 0;

enum SerialState {
    WAITING_FOR_SYNC1,
    WAITING_FOR_SYNC2,
    WAITING_FOR_SYNC3,
    WAITING_FOR_TYPE,
    READING_LENGTH,
    READING_AUDIO_DATA,
    READING_CHECKSUM,
    READING_COMMAND
};

volatile SerialState serialState = WAITING_FOR_SYNC1;
uint8_t tempAudioBuffer[AUDIO_SAMPLES_PER_PACKET];  
volatile int tempAudioIndex = 0;
volatile uint8_t expectedLength = 0;
volatile uint8_t calculatedChecksum = 0;
volatile uint8_t frameType = 0;

unsigned long bytesTransmitted = 0;
unsigned long samplesCompressed = 0;
unsigned long transmissionStartTime = 0;
unsigned long protocolErrors = 0;
unsigned long checksumErrors = 0;

uint8_t encodeADPCM(uint8_t inputSample, ADPCMState* state) {

    int16_t sample = ((int16_t)inputSample - 128) * 256;

    int16_t diff = sample - state->prevSample;

    int16_t step = stepSizeTable[state->index];

    uint8_t code = 0;

    if (diff < 0) {
        code = 8;  
        diff = -diff;
    }

    int16_t diffq = step >> 3;  

    if (diff >= step) {
        code |= 4;
        diff -= step;
        diffq += step;
    }
    step >>= 1;  
    if (diff >= step) {
        code |= 2;
        diff -= step;
        diffq += step;
    }
    step >>= 1;  
    if (diff >= step) {
        code |= 1;
        diffq += step;
    }

    if (code & 8) {
        state->prevSample -= diffq;
    } else {
        state->prevSample += diffq;
    }

    if (state->prevSample > 32767) state->prevSample = 32767;
    if (state->prevSample < -32768) state->prevSample = -32768;

    state->index += indexTable[code];

    if (state->index < 0) state->index = 0;
    if (state->index > 88) state->index = 88;

    return code;
}

void compressAudioBlock(uint8_t* input, uint8_t* output, ADPCMState* state) {

    for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i += 2) {

        uint8_t code1 = encodeADPCM(input[i], state);

        uint8_t code2 = encodeADPCM(input[i + 1], state);

        output[i / 2] = (code2 << 4) | (code1 & 0x0F);
    }

    samplesCompressed += AUDIO_SAMPLES_PER_PACKET;
}

void VextON(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(100);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    VextON();

    encoderState.reset();

    audioDisplay.init();
    audioDisplay.clear();
    audioDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
    audioDisplay.setFont(ArialMT_Plain_16);
    audioDisplay.drawString(64, 10, "LoRa Audio");
    audioDisplay.setFont(ArialMT_Plain_10);
    audioDisplay.drawString(64, 30, "ADPCM TX");
    audioDisplay.drawString(64, 45, "Starting...");
    audioDisplay.display();

    Heltec.begin(false, false, true, false, LORA_FREQUENCY);

    pinMode(USER_BUTTON, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    delay(100);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    myDeviceId = (mac[4] << 8) | mac[5];

    EEPROM.begin(512);
    loadPairingInfo();

    initLoRa();

    txQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(TxPacket));
    if (txQueue == NULL) {
        Serial.println("Failed to create TX queue!");
        while (1);
    }

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║   LoRa Audio TX - ADPCM COMPRESSION    ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║ Device ID: 0x%04X                      ║\n", myDeviceId);
    Serial.println("║                                        ║");
    Serial.println("║ ADPCM COMPRESSION:                     ║");
    Serial.println("║   4-bit ADPCM (2:1 compression)        ║");
    Serial.println("║   480 samples per packet!              ║");
    Serial.println("║   Effective rate: 60ms audio/packet    ║");
    Serial.println("║                                        ║");
    Serial.println("║ PROTOCOL SPECIFICATION:                ║");
    Serial.println("║ Audio Packet (486 bytes input):        ║");
    Serial.println("║   [A5 5A F0 AD] - 4-byte header        ║");
    Serial.println("║   [LENGTH=480]  - Length byte          ║");
    Serial.println("║   [480 samples] - Uncompressed audio   ║");
    Serial.println("║   [CHECKSUM]    - XOR checksum         ║");
    Serial.println("║                                        ║");
    Serial.println("║ Transmitted: 243 bytes (compressed)    ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    Serial.printf("Frequency: %.1f MHz\n", LORA_FREQUENCY);
    Serial.printf("Bandwidth: %.1f kHz\n", LORA_BANDWIDTH);
    Serial.printf("Spreading Factor: %d\n", LORA_SPREADING);
    Serial.printf("TX Power: %d dBm\n", LORA_TX_POWER);

    if (isPaired) {
        Serial.printf("Paired with: 0x%04X\n", pairedDeviceId);
    } else {
        Serial.println("Not paired - press 'p' or hold button\n");
    }

    xTaskCreatePinnedToCore(serialReadTask, "Serial", 4096, NULL, 24, NULL, 0);
    xTaskCreatePinnedToCore(transmissionTask, "TX", 8192, NULL, 23, NULL, 1);
    xTaskCreatePinnedToCore(packetBuilderTask, "Builder", 4096, NULL, 22, NULL, 1);

    Serial.println("System ready - waiting for data\n");
}

void loop() {
    handleButton();

    if (pairingMode) {
        handlePairingMode();
    }

    static unsigned long lastDisplayUpdate = 0;
    unsigned long now = millis();
    if (now - lastDisplayUpdate > 250) {
        updateDisplay();
        lastDisplayUpdate = now;
    }

    static unsigned long lastStatusTime = 0;
    if (currentMode != MODE_NONE && now - lastStatusTime > 5000) {
        if (packetsTransmitted > 0) {
            printPerformanceReport();
        }
        lastStatusTime = now;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void printPerformanceReport() {
    unsigned long now = millis();
    float elapsed = (now - testStartTime) / 1000.0;
    float rate = packetsTransmitted / elapsed;
    float avgAirTimeMs = totalAirTimeMicros / (1000.0 * packetsTransmitted);

    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║     ADPCM PERFORMANCE REPORT           ║");
    Serial.println("╠═══════════════════════════════════════╣");
    Serial.printf("║ Packets sent: %-24lu ║\n", packetsTransmitted);
    Serial.printf("║ Samples compressed: %-18lu ║\n", samplesCompressed);
    Serial.printf("║ Compression ratio: 2:1                 ║\n");
    Serial.printf("║ Packet rate: %-20.1f pkt/s ║\n", rate);
    Serial.printf("║ Required: 16.7 pkt/s (was 33.3)        ║\n");

    if (rate >= 16.7) {
        Serial.printf("║ Status: ✓ GOOD (%.1f%% margin)        ║\n", ((rate - 16.7) / 16.7) * 100);
    } else {
        Serial.printf("║ Status: ⚠ SLOW (%.1f%% deficit)       ║\n", ((16.7 - rate) / 16.7) * 100);
    }

    Serial.printf("║ Avg air time: %.2f ms                 ║\n", avgAirTimeMs);
    Serial.printf("║ Protocol errors: %-21lu ║\n", protocolErrors);
    Serial.printf("║ Checksum errors: %-21lu ║\n", checksumErrors);

    UBaseType_t queueWaiting = uxQueueMessagesWaiting(txQueue);
    Serial.printf("║ TX queue: %u/%d                        ║\n", queueWaiting, TX_QUEUE_SIZE);

    Serial.println("╚═══════════════════════════════════════╝\n");
}

void transmissionTask(void* parameter) {
    Serial.println("TX task started - ADPCM mode");

    TxPacket txPkt;
    uint8_t loraPacket[TOTAL_PACKET_SIZE];
    unsigned long transmissionCount = 0;
    unsigned long lastReportTime = millis();
    unsigned long totalTxTimeMicros = 0;

    while (true) {
        if (xQueueReceive(txQueue, &txPkt, 5 / portTICK_PERIOD_MS) == pdTRUE) {
            if (!isPaired || pairingMode) {
                continue;
            }

            loraPacket[0] = (pairedDeviceId >> 8) & 0xFF;
            loraPacket[1] = pairedDeviceId & 0xFF;
            loraPacket[2] = packetSequence++;
            memcpy(&loraPacket[3], txPkt.data, COMPRESSED_PACKET_SIZE);

            unsigned long txStartMicros = micros();
            int state = radio.transmit(loraPacket, TOTAL_PACKET_SIZE);
            unsigned long txEndMicros = micros();
            unsigned long txTimeMicros = txEndMicros - txStartMicros;

            if (state == RADIOLIB_ERR_NONE) {
                packetsTransmitted++;
                transmissionCount++;
                totalTxTimeMicros += txTimeMicros;

                bytesTransmitted += COMPRESSED_PACKET_SIZE;
                lastTransmitTime = millis();
                lastAirTimeMs = txTimeMicros / 1000.0;

                totalAirTimeMicros += txTimeMicros;
                if (txTimeMicros > maxAirTimeMicros) maxAirTimeMicros = txTimeMicros;
                if (txTimeMicros < minAirTimeMicros) minAirTimeMicros = txTimeMicros;

                if (millis() - lastLedToggle > LED_TOGGLE_INTERVAL) {
                    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                    lastLedToggle = millis();
                }

                unsigned long now = millis();
                if (now - lastReportTime >= 1000) {
                    float elapsed = (now - lastReportTime) / 1000.0;
                    float rate = transmissionCount / elapsed;
                    float avgTxTime = totalTxTimeMicros / (1000.0 * transmissionCount);

                    Serial.printf("TX Rate: %.1f pkt/s (avg %.2fms/pkt) - ", rate, avgTxTime);

                    if (rate >= 16.7) {
                        Serial.printf("GOOD (%.1f%% margin) - 480 samples/pkt\n", ((rate - 16.7) / 16.7) * 100);
                    } else {
                        Serial.printf("SLOW (%.1f%% deficit)\n", ((16.7 - rate) / 16.7) * 100);
                    }

                    transmissionCount = 0;
                    totalTxTimeMicros = 0;
                    lastReportTime = now;
                }

                if (packetsTransmitted % 100 == 0) {
                    Serial.printf("TX #%lu: %.2fms air time, Queue: %d, Compressed: %lu samples\n",
                                  packetsTransmitted, 
                                  txTimeMicros / 1000.0,
                                  uxQueueMessagesWaiting(txQueue),
                                  samplesCompressed);
                }
            } else {
                Serial.printf("TX Error: %d\n", state);
            }
        }

        taskYIELD();
    }
}

void packetBuilderTask(void* parameter) {
    Serial.println("Builder task started - ADPCM compression enabled");

    unsigned long lastTestPacket = 0;
    uint8_t uncompressedBuffer[AUDIO_SAMPLES_PER_PACKET];

    while (true) {
        portENTER_CRITICAL(&serialMux);
        int available = serialBufferLevel;
        portEXIT_CRITICAL(&serialMux);

        UBaseType_t queueSpace = uxQueueSpacesAvailable(txQueue);

        if (available >= AUDIO_SAMPLES_PER_PACKET && queueSpace > 0) {
            if (currentMode == MODE_SERIAL) {
                TxPacket pkt;
                pkt.timestamp = millis();

                portENTER_CRITICAL(&serialMux);
                for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++) {
                    uncompressedBuffer[i] = serialBuffer[serialReadIdx];
                    serialReadIdx = (serialReadIdx + 1) % SERIAL_BUFFER_SIZE;
                    serialBufferLevel--;
                }
                portEXIT_CRITICAL(&serialMux);

                compressAudioBlock(uncompressedBuffer, pkt.data, &encoderState);

                xQueueSend(txQueue, &pkt, 0);
            }
        } else if (currentMode >= MODE_SILENCE && currentMode <= MODE_SWEEP && queueSpace > 0) {
            unsigned long now = millis();

            if (now - lastTestPacket >= 60) {
                TxPacket pkt;
                pkt.timestamp = now;

                generateTestPattern(uncompressedBuffer);

                compressAudioBlock(uncompressedBuffer, pkt.data, &encoderState);

                xQueueSend(txQueue, &pkt, 0);
                lastTestPacket = now;
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void serialReadTask(void* parameter) {
    Serial.println("Serial task started - expecting 480 sample packets");

    serialState = WAITING_FOR_SYNC1;
    tempAudioIndex = 0;

    int bytesProcessed = 0;
    unsigned long lastReport = millis();

    while (true) {
        int bytesInBatch = 0;
        while (Serial.available() && bytesInBatch < 500) {
            uint8_t byte = Serial.read();
            bytesProcessed++;
            bytesInBatch++;

            processSerialByte(byte);
        }

        unsigned long now = millis();
        if (now - lastReport > 5000 && bytesProcessed > 0) {
            float bytesPerSec = bytesProcessed / ((now - lastReport) / 1000.0);
            Serial.printf("Serial: %.0f B/s | Errors: %lu | Checksum: %lu | Lock: %s\n", 
                          bytesPerSec, protocolErrors, checksumErrors,
                          audioStreamLocked ? "YES" : "NO");
            bytesProcessed = 0;
            lastReport = now;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void processSerialByte(uint8_t byte) {
    switch (serialState) {
        case WAITING_FOR_SYNC1:
            if (byte == FRAME_START_1) {
                serialState = WAITING_FOR_SYNC2;
                calculatedChecksum = byte;
            }
            break;

        case WAITING_FOR_SYNC2:
            if (byte == FRAME_START_2) {
                serialState = WAITING_FOR_SYNC3;
                calculatedChecksum ^= byte;
            } else if (byte == FRAME_START_1) {
                calculatedChecksum = byte;
            } else {
                serialState = WAITING_FOR_SYNC1;
                protocolErrors++;
            }
            break;

        case WAITING_FOR_SYNC3:
            if (byte == FRAME_START_3) {
                serialState = WAITING_FOR_TYPE;
                calculatedChecksum ^= byte;
            } else if (byte == FRAME_START_1) {
                serialState = WAITING_FOR_SYNC2;
                calculatedChecksum = byte;
            } else {
                serialState = WAITING_FOR_SYNC1;
                protocolErrors++;
            }
            break;

        case WAITING_FOR_TYPE:
            frameType = byte;
            calculatedChecksum ^= byte;

            if (byte == FRAME_TYPE_AUDIO) {
                if (!audioStreamLocked) {
                    lockAudioMode();
                }
                serialState = READING_LENGTH;
            } else if (byte == FRAME_TYPE_COMMAND && !audioStreamLocked) {
                serialState = READING_COMMAND;
            } else if (byte == FRAME_TYPE_COMMAND && audioStreamLocked) {
                Serial.println("⚠️  Command rejected - audio stream locked");
                serialState = WAITING_FOR_SYNC1;
            } else {
                serialState = WAITING_FOR_SYNC1;
                protocolErrors++;
            }
            break;

        case READING_LENGTH: {  
            expectedLength = byte;
            calculatedChecksum ^= byte;

            uint16_t actualLength = (expectedLength << 1);  
            if (actualLength == AUDIO_SAMPLES_PER_PACKET) {
                tempAudioIndex = 0;
                serialState = READING_AUDIO_DATA;
            } else {
                Serial.printf("❌ Invalid length: %d (expected %d)\n", actualLength, AUDIO_SAMPLES_PER_PACKET);
                serialState = WAITING_FOR_SYNC1;
                protocolErrors++;
            }
            break;
        }  

        case READING_AUDIO_DATA:
            tempAudioBuffer[tempAudioIndex++] = byte;
            calculatedChecksum ^= byte;

            if (tempAudioIndex >= AUDIO_SAMPLES_PER_PACKET) {
                serialState = READING_CHECKSUM;
            }
            break;

        case READING_CHECKSUM:
            if (byte == calculatedChecksum) {
                storeAudioPacket();
            } else {
                Serial.printf("❌ Checksum failed: got 0x%02X, expected 0x%02X\n", byte, calculatedChecksum);
                checksumErrors++;
            }
            serialState = WAITING_FOR_SYNC1;
            tempAudioIndex = 0;
            break;

        case READING_COMMAND:
            handleCommand((char)byte);
            serialState = WAITING_FOR_SYNC1;
            break;
    }
}

void storeAudioPacket() {
    portENTER_CRITICAL(&serialMux);
    int space = SERIAL_BUFFER_SIZE - serialBufferLevel;

    if (space >= AUDIO_SAMPLES_PER_PACKET) {

        for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++) {
            serialBuffer[serialWriteIdx] = tempAudioBuffer[i];
            serialWriteIdx = (serialWriteIdx + 1) % SERIAL_BUFFER_SIZE;
            serialBufferLevel++;
        }
        audioPacketsReceived++;
    } else {
        static unsigned long lastOverflow = 0;
        if (millis() - lastOverflow > 1000) {
            Serial.println("⚠️  Serial buffer overflow!");
            lastOverflow = millis();
        }
    }
    portEXIT_CRITICAL(&serialMux);
}

void lockAudioMode() {
    audioStreamLocked = true;
    firstAudioPacketTime = millis();
    currentMode = MODE_SERIAL;
    testStartTime = millis();
    transmissionStartTime = millis();
    packetsTransmitted = 0;
    bytesTransmitted = 0;
    samplesCompressed = 0;
    totalAirTimeMicros = 0;
    maxAirTimeMicros = 0;
    minAirTimeMicros = 999999999;

    encoderState.reset();

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║   🔒 ADPCM STREAM LOCKED 🔒           ║");
    Serial.println("║                                        ║");
    Serial.println("║   Compression: 4-bit ADPCM (2:1)       ║");
    Serial.println("║   480 samples per packet               ║");
    Serial.println("║   Commands are now BLOCKED             ║");
    Serial.println("║   Reset device to unlock               ║");
    Serial.println("╚════════════════════════════════════════╝\n");
}

void handleCommand(char cmd) {
    if (audioStreamLocked) {
        Serial.println("⚠️  Commands disabled during audio streaming");
        return;
    }

    switch (cmd) {
        case 'p':
            if (!pairingMode) {
                Serial.println("Entering pairing mode...");
                enterPairingMode();
            }
            break;

        case '0':
            currentMode = MODE_NONE;
            encoderState.reset();
            Serial.println("Test stopped");
            break;

        case '1':
            startTestMode(MODE_SILENCE, "Silence");
            break;

        case '2':
            startTestMode(MODE_SINE, "1kHz Sine");
            sinePhase = 0;
            break;

        case '3':
            startTestMode(MODE_SWEEP, "Frequency Sweep");
            sweepFreq = 100;
            break;

        case 's':
            printStatus();
            break;

        default:
            Serial.printf("Unknown command: '%c'\n", cmd);
            break;
    }
}

void startTestMode(TestMode mode, const char* name) {
    currentMode = mode;
    testStartTime = millis();
    transmissionStartTime = millis();
    packetsTransmitted = 0;
    bytesTransmitted = 0;
    samplesCompressed = 0;
    totalAirTimeMicros = 0;
    maxAirTimeMicros = 0;
    minAirTimeMicros = 999999999;
    encoderState.reset();
    Serial.printf("Starting %s test with ADPCM compression\n", name);
}

void generateTestPattern(uint8_t* buffer) {

    switch (currentMode) {
        case MODE_SILENCE:
            memset(buffer, 128, AUDIO_SAMPLES_PER_PACKET);
            break;

        case MODE_SINE:
            for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++) {
                float angle = sinePhase + (i * 2 * PI / 8.0);
                float sample = sin(angle);
                buffer[i] = (uint8_t)(sample * 100 + 128);
            }
            sinePhase += (AUDIO_SAMPLES_PER_PACKET * 2 * PI / 8.0);
            while (sinePhase > 2 * PI) sinePhase -= 2 * PI;
            break;

        case MODE_SWEEP:
            for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++) {
                float samplesPerCycle = 8000.0 / sweepFreq;
                float sample = sin(sinePhase);
                buffer[i] = (uint8_t)(sample * 100 + 128);

                sinePhase += 2 * PI / samplesPerCycle;
                while (sinePhase > 2 * PI) sinePhase -= 2 * PI;

                sweepFreq += 0.025;  
                if (sweepFreq > 3000) sweepFreq = 100;
            }
            break;

        default:
            memset(buffer, 128, AUDIO_SAMPLES_PER_PACKET);
    }
}

void handleButton() {
    if (audioStreamLocked) return;

    static unsigned long buttonPressStart = 0;
    static bool buttonPressed = false;
    static bool longPressHandled = false;

    if (digitalRead(USER_BUTTON) == LOW) {
        if (!buttonPressed) {
            buttonPressed = true;
            buttonPressStart = millis();
            longPressHandled = false;
        } else if (!longPressHandled && millis() - buttonPressStart > 3000) {
            Serial.println("Button held - entering pairing mode");
            enterPairingMode();
            longPressHandled = true;
        }
    } else {
        if (buttonPressed && !longPressHandled) {
            Serial.println("Button pressed");
        }
        buttonPressed = false;
        longPressHandled = false;
    }
}

void initLoRa() {
    Serial.println("Initializing LoRa radio for ADPCM...");

    int state = radio.begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH,
        LORA_SPREADING,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Radio init failed: %d\n", state);
        while (1);
    }

    radio.setRegulatorLDO(); 

    state = radio.setCRC(false);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CRC enable failed: %d\n", state);
    }

    state = radio.implicitHeader(TOTAL_PACKET_SIZE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Implicit header failed: %d\n", state);
    }

    state = radio.fixedPacketLengthMode(TOTAL_PACKET_SIZE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Fixed packet mode: %d\n", state);
    }

    radio.setRfSwitchPins(LORA_NSS, RADIOLIB_NC);

    delay(100);
    Serial.println("LoRa radio ready for ADPCM transmission");
}

void enterPairingMode() {
    if (audioStreamLocked) {
        Serial.println("Cannot pair during audio streaming");
        return;
    }

    Serial.println("\n=== PAIRING MODE ===");
    pairingMode = true;
    pairingStartTime = millis();
    currentMode = MODE_NONE;

    portENTER_CRITICAL(&serialMux);
    serialBufferLevel = 0;
    serialReadIdx = 0;
    serialWriteIdx = 0;
    portEXIT_CRITICAL(&serialMux);

    serialState = WAITING_FOR_SYNC1;
    tempAudioIndex = 0;
    encoderState.reset();

    digitalWrite(LED_PIN, HIGH);
    Serial.println("Broadcasting beacon...");
}

void handlePairingMode() {

    if (audioStreamLocked) {
        pairingMode = false;
        digitalWrite(LED_PIN, LOW);
        return;
    }

    if (millis() - pairingStartTime > 20000) {
        Serial.println("Pairing timeout");
        pairingMode = false;
        digitalWrite(LED_PIN, LOW);
        return;
    }

    static unsigned long lastBeacon = 0;
    unsigned long now = millis();

    if (now - lastBeacon > 1000) {
        sendPairingBeacon();
        lastBeacon = now;

        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        int remaining = (20000 - (now - pairingStartTime)) / 1000;
        Serial.printf("Pairing... %ds remaining\n", remaining);

        unsigned long listenStart = millis();
        bool responseReceived = false;

        while (millis() - listenStart < 900 && !responseReceived) {
            uint8_t response[10];

            int state = radio.receive(response, sizeof(response), 100);

            if (state == RADIOLIB_ERR_NONE) {
                Serial.printf("Received packet during pairing: %02X %02X\n", 
                             response[0], response[1]);

                if (response[0] == 0xBB && response[1] == 0xBB) {
                    uint16_t responderId = (response[2] << 8) | response[3];
                    uint16_t targetId = (response[4] << 8) | response[5];

                    Serial.printf("Response from receiver: ID=0x%04X, Target=0x%04X\n", 
                                 responderId, targetId);

                    if (targetId == myDeviceId) {
                        Serial.printf("Valid response from receiver 0x%04X!\n", responderId);
                        responseReceived = true;

                        Serial.println("Sending confirmation packets...");
                        for (int i = 0; i < 5; i++) {
                            uint8_t confirm[6];
                            confirm[0] = 0xCC;
                            confirm[1] = 0xCC;
                            confirm[2] = (myDeviceId >> 8) & 0xFF;
                            confirm[3] = myDeviceId & 0xFF;
                            confirm[4] = (responderId >> 8) & 0xFF;
                            confirm[5] = responderId & 0xFF;

                            int txState = radio.transmit(confirm, sizeof(confirm));
                            if (txState == RADIOLIB_ERR_NONE) {
                                Serial.printf("Confirmation %d/5 sent\n", i + 1);
                            } else {
                                Serial.printf("Confirmation %d/5 failed: %d\n", i + 1, txState);
                            }

                            delay(100);
                        }

                        pairedDeviceId = responderId;
                        isPaired = true;
                        savePairingInfo();

                        Serial.println("\n╔════════════════════════════════════════╗");
                        Serial.printf("║  ✓ PAIRING SUCCESSFUL!                 ║\n");
                        Serial.printf("║  Paired with receiver: 0x%04X         ║\n", pairedDeviceId);
                        Serial.println("║  ADPCM audio system ready              ║");
                        Serial.println("╚════════════════════════════════════════╝\n");

                        pairingMode = false;
                        digitalWrite(LED_PIN, LOW);

                        delay(500);

                        return; 
                    }
                }
            } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {

                Serial.printf("Receive error during pairing: %d\n", state);
            }

            delay(10);
        }

        if (!responseReceived) {
            Serial.println("No response received this cycle");
        }
    }
}

void sendPairingConfirmation(uint16_t rxId) {
    uint8_t confirm[6];
    confirm[0] = 0xCC;
    confirm[1] = 0xCC;
    confirm[2] = (myDeviceId >> 8) & 0xFF;
    confirm[3] = myDeviceId & 0xFF;
    confirm[4] = (rxId >> 8) & 0xFF;
    confirm[5] = rxId & 0xFF;

    radio.transmit(confirm, sizeof(confirm));
}

void sendPairingBeacon() {
    uint8_t beacon[6];
    beacon[0] = 0xAA;
    beacon[1] = 0xAA;
    beacon[2] = (myDeviceId >> 8) & 0xFF;
    beacon[3] = myDeviceId & 0xFF;
    beacon[4] = 0x01;
    beacon[5] = calculateChecksum(beacon, 5);

    int state = radio.transmit(beacon, sizeof(beacon));
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Beacon TX failed: %d\n", state);
    }
}

void sendPairingConfirmation() {
    uint8_t confirm[6];
    confirm[0] = 0xCC;
    confirm[1] = 0xCC;
    confirm[2] = (myDeviceId >> 8) & 0xFF;
    confirm[3] = myDeviceId & 0xFF;
    confirm[4] = (pairedDeviceId >> 8) & 0xFF;
    confirm[5] = pairedDeviceId & 0xFF;

    radio.transmit(confirm, sizeof(confirm));
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
        Serial.printf("Loaded pairing: 0x%04X\n", pairedDeviceId);
    } else {
        Serial.println("No saved pairing");
    }
}

void savePairingInfo() {
    EEPROM.write(0, 0x55);
    EEPROM.write(1, (pairedDeviceId >> 8) & 0xFF);
    EEPROM.write(2, pairedDeviceId & 0xFF);
    EEPROM.commit();
    Serial.println("Pairing saved");
}

void updateDisplay() {
    audioDisplay.clear();

    audioDisplay.setFont(ArialMT_Plain_10);
    audioDisplay.setTextAlignment(TEXT_ALIGN_LEFT);

    if (audioStreamLocked) {
        audioDisplay.drawString(0, 0, "🔒ADPCM");
    } else {
        audioDisplay.drawString(0, 0, "TX");
    }

    if (isPaired) {
        audioDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
        char str[16];
        sprintf(str, "→%04X", pairedDeviceId);
        audioDisplay.drawString(64, 0, str);
    }

    audioDisplay.setTextAlignment(TEXT_ALIGN_RIGHT);
    if (packetsTransmitted > 0) {
        audioDisplay.drawString(128, 0, String(packetsTransmitted));
    }

    audioDisplay.setTextAlignment(TEXT_ALIGN_CENTER);

    if (audioStreamLocked) {
        audioDisplay.setFont(ArialMT_Plain_16);
        audioDisplay.drawString(64, 20, "STREAMING");
        audioDisplay.setFont(ArialMT_Plain_10);

        if (packetsTransmitted > 0 && lastAirTimeMs > 0) {
            char info[32];
            float rate = packetsTransmitted / ((millis() - testStartTime) / 1000.0);
            sprintf(info, "%.1f pkt/s", rate);
            audioDisplay.drawString(64, 40, info);

            if (rate >= 16.7) {
                audioDisplay.drawString(64, 52, "✓ ADPCM 2:1");
            } else {
                audioDisplay.drawString(64, 52, "⚠ SLOW");
            }
        }
    } else if (pairingMode) {
        audioDisplay.setFont(ArialMT_Plain_16);
        audioDisplay.drawString(64, 20, "Pairing...");
        audioDisplay.setFont(ArialMT_Plain_10);
        int remaining = (20000 - (millis() - pairingStartTime)) / 1000;
        audioDisplay.drawString(64, 40, String(remaining) + "s");
    } else if (isPaired) {
        audioDisplay.setFont(ArialMT_Plain_16);
        const char* modeStr = "";
        switch (currentMode) {
            case MODE_SERIAL: modeStr = "ADPCM"; break;
            case MODE_SILENCE: modeStr = "Silent"; break;
            case MODE_SINE: modeStr = "1kHz"; break;
            case MODE_SWEEP: modeStr = "Sweep"; break;
            case MODE_NONE: modeStr = "Ready"; break;
        }
        audioDisplay.drawString(64, 20, modeStr);

        if (currentMode != MODE_NONE && packetsTransmitted > 0) {
            audioDisplay.setFont(ArialMT_Plain_10);
            char info[32];
            sprintf(info, "%.1fms", lastAirTimeMs);
            audioDisplay.drawString(64, 40, info);

            if (samplesCompressed > 0) {
                sprintf(info, "%lu smpl", samplesCompressed);
                audioDisplay.drawString(64, 52, info);
            }
        }
    } else {
        audioDisplay.setFont(ArialMT_Plain_16);
        audioDisplay.drawString(64, 20, "Not Paired");
        audioDisplay.setFont(ArialMT_Plain_10);
        audioDisplay.drawString(64, 40, "Press 'p'");
    }

    audioDisplay.display();
}

void printStatus() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║       ADPCM SYSTEM STATUS              ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║ Device ID: 0x%04X                      ║\n", myDeviceId);

    if (audioStreamLocked) {
        Serial.println("║ Mode: 🔒 ADPCM LOCKED                  ║");
        Serial.printf("║ Duration: %lu seconds                  ║\n", 
                      (millis() - firstAudioPacketTime) / 1000);
    } else {
        Serial.println("║ Mode: Command ready                    ║");
    }

    Serial.println("║ Compression: 4-bit ADPCM (2:1)         ║");
    Serial.printf("║ Samples compressed: %lu                ║\n", samplesCompressed);

    if (isPaired) {
        Serial.printf("║ Paired with: 0x%04X                   ║\n", pairedDeviceId);
    } else {
        Serial.println("║ Status: Not paired                     ║");
    }

    Serial.printf("║ Packets sent: %lu                      ║\n", packetsTransmitted);
    Serial.printf("║ Protocol errors: %lu                   ║\n", protocolErrors);
    Serial.printf("║ Checksum errors: %lu                   ║\n", checksumErrors);

    if (packetsTransmitted > 0) {
        float avgAirTimeMs = totalAirTimeMicros / (1000.0 * packetsTransmitted);
        Serial.printf("║ Avg air time: %.2f ms                 ║\n", avgAirTimeMs);
    }

    Serial.println("╚════════════════════════════════════════╝\n");
}