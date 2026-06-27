#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <SPI.h>
#include "comms.h"
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncWebServer.h>
#include <MycilaWebSerial.h>
#include "SPIFFS.h"
#include <ESPmDNS.h>
/*---------------------------------------
link aceder web robo: http://robot.local
-----------------------------------------*/
//CODIGO PARA O ROBO 1----------

/* ---------------------------------------------------------------
 * VL53L7CX – biblioteca oficial stm32duino
 * Download: https://github.com/stm32duino/VL53L7CX
 * Coloca a pasta extraída do ZIP dentro de /lib do projecto
 * --------------------------------------------------------------- */
#include <Wire.h>
#include <vl53l7cx_class.h>

/* ---------------------------------------------------------------
 Biblioteca rgb sensor
 * --------------------------------------------------------------- */
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

#define SPI_SS 10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCLK 13

/* ---------------------------------------------------------------
 * Pinos I2C do VL53L7CX  ← ajusta para os teus pinos físicos
 * --------------------------------------------------------------- */
#define LIDAR_SDA   8
#define LIDAR_SCL   9
#define LIDAR_LPN  -1   /* Low-Power pin  – usa -1 se não ligado */
#define LIDAR_RST  -1   /* I2C Reset pin  – usa -1 se não ligado */

#define RGB_LED_PIN 38
#define WIFI_ACCESS_POINT 1

/* ---------------------------------------------------------------
 * Velocidades WASD e modo automático
 * --------------------------------------------------------------- */
#define CONST_V        0.25f   /* m/s   */
#define CONST_OMEGA    2.0f   /* rad/s */

/* ---------------------------------------------------------------
 * Parâmetros de obstacle-avoidance
 *
 *  OBSTACLE_DIST_MM  – distância (mm) abaixo da qual existe obstáculo
 *  MIN_VALID_ZONES   – nº mínimo de pixels válidos por zona
 * --------------------------------------------------------------- */
#define OBSTACLE_DIST_MM  170
#define MIN_VALID_ZONES     2

volatile bool destinoAlcancado  = false;
volatile bool tesouroEncontrado = false;   /* true após detetar AZUL */

/* ---------------------------------------------------------------
 * Grelha 8×8 do VL53L7CX – sensor na HORIZONTAL
 *
 *  O sensor está rodado 90°, por isso usamos LINHAS:
 *
 *  linha 0-1  →  DIREITA
 *  linha 2-5  →  CENTRO
 *  linha 6-7  →  ESQUERDA
 * --------------------------------------------------------------- */
#define GRID_COLS    4
#define GRID_ROWS    4
#define TOTAL_ZONES (GRID_COLS * GRID_ROWS)   /* 16 */

/* Devolve a linha do pixel (0..7) */
static inline int zoneRow(int idx) { return idx / GRID_COLS; }

typedef enum { ZONE_LEFT, ZONE_CENTER, ZONE_RIGHT } ZoneClass;

/* Classifica por LINHA porque o sensor está na horizontal */
static ZoneClass classifyRow(int row)
{
    if (row == 0) return ZONE_LEFT;
    if (row == 3) return ZONE_RIGHT;
    return ZONE_CENTER;
}

/* ---------------------------------------------------------------
 * Modo de operação
 * --------------------------------------------------------------- */
typedef enum { MODE_MANUAL, MODE_AUTO } DriveMode;
static volatile DriveMode driveMode = MODE_MANUAL;

/* ---------------------------------------------------------------
 * Protótipos
 * --------------------------------------------------------------- */
void TaskCommsCode   (void *parameter);
void TaskBootBtnCode (void *parameter);
void TaskLidarCode   (void *parameter);
void transferReceivePackets(comms_packet_t *packet);
String detectColor();

TaskHandle_t TaskCommsHandler;
TaskHandle_t TaskBootBtnHandler;
TaskHandle_t TaskLidarHandler;

SemaphoreHandle_t SPIMutex = NULL;
SemaphoreHandle_t CmdMutex = NULL;

PacketHandler        packetHandler;
Adafruit_NeoPixel    rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
AsyncWebServer       server(80);
WebSerial            webSerial;

/* Objecto do sensor – passa &Wire + pinos LPN e RST */
VL53L7CX sensor_vl53l7cx(&Wire, LIDAR_LPN, LIDAR_RST);

const char *ssid     = "SE_ROBOTzao";
const char *password = "";
const char *mdnsName = "robot";

uint8_t cmd = 0;

volatile float cmd_v     = 0.0f;
volatile float cmd_omega = 0.0f;

float state_x     = 0.0f;
float state_y     = 0.0f;
float state_theta = 0.0f;

/* ---------------------------------------------------------------
 * Média de distância para uma zona (mm).
 * Devolve -1.0f se não houver pixels válidos suficientes.
 * --------------------------------------------------------------- */
static float zoneMean(const uint16_t *dist, ZoneClass zone)
{
    long sum   = 0;
    int  count = 0;
    for (int i = 0; i < TOTAL_ZONES; i++) {
        if (classifyRow(zoneRow(i)) != zone) continue;
        if (dist[i] == 0) continue;
        sum += dist[i];
        count++;
    }
    if (count < MIN_VALID_ZONES) return -1.0f;
    return (float)sum / (float)count;
}

/* ---------------------------------------------------------------
 * Lógica de obstacle-avoidance
 *
 *  Prioridades:
 *   1. Caminho livre             → avança
 *   2. Obstáculo só à esquerda  → vira à direita
 *   3. Obstáculo só à direita   → vira à esquerda
 *   4. Obstáculo só ao centro   → vira para o lado com mais espaço
 *   5. Obstáculo em múltiplas   → para e vira para o lado mais livre
 * --------------------------------------------------------------- */
static void computeAutoVelocity(const uint16_t *dist)
{
    float meanL = zoneMean(dist, ZONE_LEFT);
    float meanC = zoneMean(dist, ZONE_CENTER);      
    float meanR = zoneMean(dist, ZONE_RIGHT);

    bool obsL = (meanL > 0.0f && meanL < OBSTACLE_DIST_MM);
    bool obsC = (meanC > 0.0f && meanC < OBSTACLE_DIST_MM);
    bool obsR = (meanR > 0.0f && meanR < OBSTACLE_DIST_MM);

    float v = 0.0f, omega = 0.0f;

    if (!obsL && !obsC && !obsR) {
        v = -CONST_V; omega = 0.0f;

    } else if (obsL && !obsR) {
        v     = obsC ? 0.0f : -CONST_V * 0.5f;
        omega = CONST_OMEGA;

    } else if (obsR && !obsL) {
        v     = obsC ? 0.0f : -CONST_V * 0.5f;
        omega = -CONST_OMEGA;

    } else if (obsC && !obsL && !obsR) {
        float effL = (meanL <= 0.0f) ? 9999.0f : meanL;
        float effR = (meanR <= 0.0f) ? 9999.0f : meanR;
        v     = 0.0f;
        omega = (effL >= effR) ? CONST_OMEGA : -CONST_OMEGA;

    } else {
        // ENCURRALADO – verifica se as zonas são todas semelhantes
        float effL = (meanL <= 0.0f) ? 0.0f : meanL;
        float effR = (meanR <= 0.0f) ? 0.0f : meanR;
        float diff  = fabsf(effL - effR);

        if (diff < 20.0f) {
            // zonas iguais → viragem forçada dependendo da fase
            xSemaphoreTake(CmdMutex, portMAX_DELAY);
            cmd_v     = 0.0f;
            cmd_omega = tesouroEncontrado ? CONST_OMEGA : -CONST_OMEGA;
            xSemaphoreGive(CmdMutex);

            // mantém a viragem durante 2 segundos
            vTaskDelay(pdMS_TO_TICKS(1500));

            // depois volta ao normal (deixa o loop decidir)
            return;
        }

        v     = 0.0f;
        omega = (effL >= effR) ? CONST_OMEGA : -CONST_OMEGA;
    }

    xSemaphoreTake(CmdMutex, portMAX_DELAY);
    cmd_v     = v;
    cmd_omega = omega;
    xSemaphoreGive(CmdMutex);
}
/* ---------------------------------------------------------------
 * Interpretação de comandos WebSerial
 * --------------------------------------------------------------- */
static void handleSerialCommand(const std::string &msg)
{
    if (msg.empty()) return;

    char c = (char)tolower((unsigned char)msg[0]);
    static char  last_c        = '\0';
    static float last_cmd_vaux = 0.0f;

    if (msg == "q") {
        driveMode = MODE_AUTO;
        destinoAlcancado  = false;
        tesouroEncontrado = false;
        webSerial.printf("[INFO] Modo automatico activado\n");
        webSerial.flush();
        return;
    }
    if (msg == "m") {
        driveMode = MODE_MANUAL;
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v = 0.0f; cmd_omega = 0.0f;
        xSemaphoreGive(CmdMutex);
        webSerial.printf("[INFO] Modo manual activado\n");
        webSerial.flush();
        return;
    }

    if (driveMode == MODE_AUTO) {
        if (c == 'r') {
            driveMode = MODE_MANUAL;
            last_c = '\0';
            last_cmd_vaux = 0.0f;
            xSemaphoreTake(CmdMutex, portMAX_DELAY);
            cmd_v = 0.0f; cmd_omega = 0.0f;
            xSemaphoreGive(CmdMutex);
            cmd = 'R';
            destinoAlcancado  = false;
            tesouroEncontrado = false;
            webSerial.printf("[INFO] Modo manual activado + reset pose\n");
            webSerial.flush();
        }
        return;
    }

    if (msg == "w" || c == 'w') {
        last_c = 'w'; last_cmd_vaux = -CONST_V;
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v = -CONST_V; cmd_omega = 0.0f;
        xSemaphoreGive(CmdMutex);
    } else if (msg == "s" || c == 's') {
        last_c = 's'; last_cmd_vaux = CONST_V;
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v = CONST_V; cmd_omega = 0.0f;
        xSemaphoreGive(CmdMutex);
    } else if (msg == "a" || c == 'a') {
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v     = last_cmd_vaux;
        cmd_omega = (last_c == 'w') ? -CONST_OMEGA : CONST_OMEGA;
        xSemaphoreGive(CmdMutex);
    } else if (msg == "d" || c == 'd') {
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v     = last_cmd_vaux;
        cmd_omega = (last_c == 'w') ? CONST_OMEGA : -CONST_OMEGA;
        xSemaphoreGive(CmdMutex);
    } else if (msg.find("stop") != std::string::npos || msg == "") {
        last_cmd_vaux = 0.0f;
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v = 0.0f; cmd_omega = 0.0f;
        xSemaphoreGive(CmdMutex);
    } else if (c == 'r') {
        last_cmd_vaux = 0.0f;
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v = 0.0f; cmd_omega = 0.0f;
        xSemaphoreGive(CmdMutex);
        cmd = 'R';
    } else {
        float v = 0.0f, w = 0.0f;
        sscanf(msg.c_str(), "v=%f;w=%f", &v, &w);
        last_cmd_vaux = v;
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        cmd_v = v; cmd_omega = w;
        xSemaphoreGive(CmdMutex);
    }
}

/* ================================================================
 * SETUP
 * ================================================================ */
void setup()
{
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
        return;
    }

    pinMode(SPI_SS,   OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, OUTPUT);
    pinMode(SPI_SCLK, INPUT);
    pinMode(GPIO_NUM_0, INPUT_PULLUP);

    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(1000000);

    Serial.begin(115200);
    Serial.setDebugOutput(true);

    /* I2C para o LIDAR */
    Wire.begin(LIDAR_SDA, LIDAR_SCL);
    Wire.setClock(400000);

#if WIFI_ACCESS_POINT == 1
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
#else
    WiFi.mode(WIFI_STA);
    WiFi.begin("ssid", "pass");
    while (WiFi.status() != WL_CONNECTED) { Serial.print('.'); delay(1000); }
    Serial.println(WiFi.localIP());
#endif

    if (!MDNS.begin(mdnsName)) {
        Serial.println("MDNS failed!");
        while (1) delay(1000);
    }
    MDNS.addService("_http", "_tcp", 80);

    SPIMutex = xSemaphoreCreateMutex();
    CmdMutex = xSemaphoreCreateMutex();
    if (!SPIMutex || !CmdMutex) {
        Serial.println("Mutex creation failed!");
        while (1);
    }

    xTaskCreatePinnedToCore(TaskCommsCode,   "Comms",   10000, NULL, 1, &TaskCommsHandler,   1);
    xTaskCreatePinnedToCore(TaskBootBtnCode, "BootBtn",  3000, NULL, 0, &TaskBootBtnHandler, 0);
    xTaskCreatePinnedToCore(TaskLidarCode,   "Lidar",   10000, NULL, 1, &TaskLidarHandler,   0);

    webSerial.onMessage([](const std::string &msg) { handleSerialCommand(msg); });
    webSerial.begin(&server);
    webSerial.setBuffer(100);

    server.onNotFound([](AsyncWebServerRequest *r) { r->redirect("/index.html"); });
    server.serveStatic("/", SPIFFS, "/");
    server.begin();
}

void loop() { delay(1000); }

/* ================================================================
 * TaskLidarCode – lê VL53L7CX e aplica obstacle-avoidance (10 Hz)
 * ================================================================ */
void TaskLidarCode(void *parameter)
{
    /* Inicialização */
    sensor_vl53l7cx.begin();


    while (sensor_vl53l7cx.init_sensor(0x54) != 0) {
        webSerial.printf("[LIDAR] Sensor nao encontrado, verifica ligacoes I2C\n");
        webSerial.flush();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    webSerial.printf("[LIDAR] Sensor OK\n");
    webSerial.flush();

    /* Resolução 4×4, modo contínuo, 10 Hz */
    sensor_vl53l7cx.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4);
    sensor_vl53l7cx.vl53l7cx_set_ranging_mode(VL53L7CX_RANGING_MODE_CONTINUOUS);
    sensor_vl53l7cx.vl53l7cx_set_ranging_frequency_hz(10);
    sensor_vl53l7cx.vl53l7cx_start_ranging();

    /* TCS34725 */
    while (!tcs.begin(0x29)) {
        webSerial.printf("[RGB] Sensor nao encontrado\n");
        webSerial.flush();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    webSerial.printf("[RGB] Sensor OK\n");
    webSerial.flush();

    VL53L7CX_ResultsData results;
    uint8_t              newDataReady = 0;
    uint16_t             distances[TOTAL_ZONES];

    TickType_t xLastWakeTime  = xTaskGetTickCount();
    TickType_t xLastTelemetry = xTaskGetTickCount();

    for (;;)
    {

        // Adiciona temporariamente no loop do TaskLidarCode para debug:
/* for (int row = 0; row < 4; row++) {
    uint16_t minVal = 9999;
    for (int col = 0; col < 4; col++) {
        uint16_t d = distances[row * 4 + col];
        if (d > 0 && d < minVal) minVal = d;
    }
    webSerial.printf("row%d: %u mm\n", row, minVal);
} */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); /* 10 Hz */

        sensor_vl53l7cx.vl53l7cx_check_data_ready(&newDataReady);
        if (!newDataReady) continue;

        if (sensor_vl53l7cx.vl53l7cx_get_ranging_data(&results) != VL53L7CX_STATUS_OK) continue;

        /* Extrai distâncias: target 0 = mais próximo em cada pixel */
        for (int i = 0; i < TOTAL_ZONES; i++) {
            distances[i] = (results.nb_target_detected[i] > 0)
                           ? (uint16_t)results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE * i]
                           : 0;
        }

        if (driveMode == MODE_AUTO) {
            computeAutoVelocity(distances);
        }

        /* Telemetria a ~2 Hz */
       

        if ((xTaskGetTickCount() - xLastTelemetry) >= pdMS_TO_TICKS(500))
        {
            
            xLastTelemetry = xTaskGetTickCount();
            bool parado = (cmd_v == 0.0f && cmd_omega == 0.0f);

            if(!parado){
            float mL = zoneMean(distances, ZONE_LEFT);
            float mC = zoneMean(distances, ZONE_CENTER);
            float mR = zoneMean(distances, ZONE_RIGHT);
            webSerial.printf("[LIDAR] L=%.0f C=%.0f R=%.0f mm | %s v=%.2f w=%.2f\n",
                             mL, mC, mR,
                             (driveMode == MODE_AUTO) ? "AUTO" : "MANUAL",
                             (float)cmd_v, (float)cmd_omega);

            /* Leitura de cor no modo AUTO */
            if (driveMode == MODE_AUTO && !destinoAlcancado)
            {
                String cor = detectColor();
                webSerial.printf("[RGB] Cor detetada: %s\n", cor.c_str());

                /* ── Fase 1: à procura do tesouro (AZUL) ── */
                if (!tesouroEncontrado && cor == "AZUL")
                {
                    tesouroEncontrado = true;

                    /* Para o robô imediatamente */
                    xSemaphoreTake(CmdMutex, portMAX_DELAY);
                    cmd_v     = 0.0f;
                    cmd_omega = 0.0f;
                    xSemaphoreGive(CmdMutex);

                    webSerial.printf("[INFO] *** TESOURO ENCONTRADO! A aguardar 1.5s... ***\n");
                    webSerial.flush();

                    vTaskDelay(pdMS_TO_TICKS(1500));   /* espera 1.5 segundos */

                    /* Rotação de 180°:  */
                    const uint32_t turnTime_ms = 4100;
                    xSemaphoreTake(CmdMutex, portMAX_DELAY);
                    cmd_v     = 0.0f;
                    cmd_omega = CONST_OMEGA;   /* roda no próprio eixo */
                    xSemaphoreGive(CmdMutex);

                    webSerial.printf("[INFO] A fazer 180 graus (%u ms)...\n", turnTime_ms);
                    webSerial.flush();

                    vTaskDelay(pdMS_TO_TICKS(turnTime_ms));

                    /* Para a rotação e retoma navegação autónoma */
                    xSemaphoreTake(CmdMutex, portMAX_DELAY);
                    cmd_v     = 0.0f;
                    cmd_omega = 0.0f;
                    xSemaphoreGive(CmdMutex);

                    webSerial.printf("[INFO] A navegar de volta ao inicio (procura VERMELHO)...\n");
                    webSerial.flush();
                }

                /* ── Fase 2: tesouro encontrado, à procura da posição inicial (VERMELHO) ── */
                else if (tesouroEncontrado && cor == "VERMELHO")
                {
                    destinoAlcancado = true;

                    xSemaphoreTake(CmdMutex, portMAX_DELAY);
                    cmd_v     = 0.0f;
                    cmd_omega = 0.0f;
                    xSemaphoreGive(CmdMutex);

                    driveMode = MODE_MANUAL;

                    webSerial.printf("[INFO] *** VOLTOU A POSICAO INICIAL! Robo parado. ***\n");
                    webSerial.flush();
                }
            }

            }
            

            webSerial.flush();
        }
    }
}

/* ================================================================
 * funçao detetar cores
 * ================================================================ */
String detectColor() {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    float total = r + g + b;
    
    if (total == 0) return "UNKNOWN";

    float rn = (r / total) * 255;
    float gn = (g / total) * 255;
    float bn = (b / total) * 255;

    webSerial.printf("[RGB] normalizado: R=%f G=%f B=%f C=%u\n", rn, gn, bn, c);

    if (bn > rn && bn > gn && bn>70 ) return "AZUL";
    if (rn > gn && rn > bn && rn >128) return "VERMELHO";
    if (gn > rn && gn > bn && gn>100) return "VERDE";

    if (c > 800)             return "BRANCO";
    if (c < 100)             return "PRETO";
    return "chao";
}

/* ================================================================
 * TaskCommsCode – comunica com STM32 a 10 Hz
 * ================================================================ */
void TaskCommsCode(void *parameter)
{
    TickType_t xLastWakeTime  = xTaskGetTickCount();
    TickType_t xLastTelemetry = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

        if (cmd == 'R') {
            cmd = 0;
            uint8_t zero[4] = {0};
            packetHandler.buildPacket(COMMS_TYPE_WRITE, 0x12, zero);
            transferReceivePackets(&packetHandler.packet);
        }

        float v, omega;
        xSemaphoreTake(CmdMutex, portMAX_DELAY);
        v = cmd_v; omega = cmd_omega;
        xSemaphoreGive(CmdMutex);

        packetHandler.buildPacket(COMMS_TYPE_WRITE, 0x10, (uint8_t *)&v);
        transferReceivePackets(&packetHandler.packet);

        packetHandler.buildPacket(COMMS_TYPE_WRITE, 0x11, (uint8_t *)&omega);
        transferReceivePackets(&packetHandler.packet);

        uint8_t zero[4] = {0};

        packetHandler.buildPacket(COMMS_TYPE_READ, 0x01, zero);
        transferReceivePackets(&packetHandler.packet);
        if (packetHandler.isPacketValid())
            memcpy(&state_x, packetHandler.packet.data, sizeof(float));

        packetHandler.buildPacket(COMMS_TYPE_READ, 0x02, zero);
        transferReceivePackets(&packetHandler.packet);
        if (packetHandler.isPacketValid())
            memcpy(&state_y, packetHandler.packet.data, sizeof(float));

        packetHandler.buildPacket(COMMS_TYPE_READ, 0x03, zero);
        transferReceivePackets(&packetHandler.packet);
        if (packetHandler.isPacketValid())
            memcpy(&state_theta, packetHandler.packet.data, sizeof(float));

        if ((xTaskGetTickCount() - xLastTelemetry) >= pdMS_TO_TICKS(1000))
{
    xLastTelemetry = xTaskGetTickCount();

    bool parado = (cmd_v == 0.0f && cmd_omega == 0.0f);

    if (parado) {
        webSerial.printf("robo parado\n");
    } else {
        webSerial.printf("x=%.4f;y=%.4f;theta=%.4f\n", state_x, state_y, state_theta);
    }
    webSerial.flush();
}

    }
}

/* ================================================================
 * transferReceivePackets – SPI com mutex
 * ================================================================ */
void transferReceivePackets(comms_packet_t *packet)
{
    xSemaphoreTake(SPIMutex, portMAX_DELAY);
    digitalWrite(SPI_SS, LOW);
    SPI.transferBytes((uint8_t *)packet, NULL, sizeof(comms_packet_t));
    delay(10);
    SPI.transferBytes(NULL, (uint8_t *)packet, sizeof(comms_packet_t));
    digitalWrite(SPI_SS, HIGH);
    xSemaphoreGive(SPIMutex);
}

/* ================================================================
 * TaskBootBtnCode – pisca LED roxo no BOOT (debug visual)
 * NÃO APAGAR
 * ================================================================ */
void TaskBootBtnCode(void *parameter)
{
    bool toogle = false;
    rgbLed.begin();
    rgbLed.setBrightness(10);
    for (;;)
    {
        if (digitalRead(GPIO_NUM_0) == LOW) {
            toogle = !toogle;
            rgbLed.setPixelColor(0, toogle
                ? rgbLed.Color(255, 0, 255)
                : rgbLed.Color(0, 0, 0));
        } else {
            rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
        }
        rgbLed.show();
        delay(250);
    }
}
