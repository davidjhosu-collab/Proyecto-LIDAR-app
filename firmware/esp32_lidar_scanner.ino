/*
 * =========================================================================================
 * PROYECTO: Escáner 3D LiDAR ESP32 con Servos Pan-Tilt MG996R y Sensor ToF VL53L0X
 * ARCHIVO: firmware/esp32_lidar_scanner.ino
 * AUTOR: Equipo DevOps & Hardware Firmware
 * DESCRIPCIÓN:
 *   Firmware embebido de alto rendimiento para microcontroladores ESP32 (NodeMCU / DevKit).
 *   Controla dos servomotores MG996R mediante modulación PWM LEDC nativa (50 Hz / 14 bits)
 *   y adquiere mediciones milimétricas de distancia por tiempo de vuelo (ToF) mediante el
 *   sensor láser VL53L0X vía bus I2C.
 *
 * MÁQUINA DE ESTADOS:
 *   - STATE_IDLE: En reposo, listo para recibir comandos o jogging manual.
 *   - STATE_SCANNING: Barrido activo boustrophedon con transmisión continua paso a paso.
 *   - STATE_PAUSED: Barrido congelado, servomotores energizados y fijos en ángulo exacto.
 *
 * ASIGNACIÓN DE PINES (PINOUT):
 *   - GPIO 18: Señal PWM Servomotor PAN (Giro Horizontal Acimutal)
 *   - GPIO 19: Señal PWM Servomotor TILT (Inclinación Vertical Cenital)
 *   - GPIO 21: Bus I2C - SDA (Línea de Datos VL53L0X)
 *   - GPIO 22: Bus I2C - SCL (Línea de Reloj VL53L0X)
 *   - VCC: 3.3V (VL53L0X) / 5.0V - 6.0V 3A Regulado Externo (Servos MG996R)
 *   - GND: Tierra común entre ESP32, fuente de servos y sensor VL53L0X
 *
 * ESPECIFICACIONES PWM LEDC:
 *   - Frecuencia: 50 Hz (Periodo de 20 ms = 20,000 microsegundos)
 *   - Resolución: 14 bits (Valores de ciclo de trabajo de 0 a 16,383)
 *   - Rango de pulso MG996R: 500 us (0°) a 2500 us (180°)
 *   - Compatibilidad: ESP32 Arduino Core v2.x y v3.x
 *
 * PROTOCOLO SERIAL BIDIRECCIONAL (115200 bps):
 *   Entrada (Comandos desde la aplicación PyQt6):
 *     - CAL:P,<angulo>             -> Posiciona inmediatamente el servo PAN
 *     - CAL:T,<angulo>             -> Posiciona inmediatamente el servo TILT
 *     - CMD:RES,<resolucion>       -> Ajusta la resolución angular de escaneo
 *     - CMD:CENTER                 -> Posiciona ambos servos en el centro (90.0°, 90.0°)
 *     - CMD:START_SCAN,Pmin,Pmax,Tmin,Tmax,Res -> Inicia escaneo matricial
 *     - CMD:PAUSE_SCAN             -> Congela el escaneo y motores en ángulo actual
 *     - CMD:RESUME_SCAN            -> Reanuda el escaneo exactamente donde se pausó
 *     - CMD:STOP_SCAN              -> Detiene el escaneo inmediatamente
 *
 *   Salida (Telemetría hacia la aplicación PyQt6):
 *     - Formato continuo: <PAN>,<TILT>,<DIST_MM>\n (distancia = -1 si apunta al vacío)
 *     - Respuestas de control: OK:SCAN_STARTED, OK:SCAN_PAUSED, OK:SCAN_RESUMED,
 *                              OK:SCAN_STOPPED, OK:SCAN_FINISHED
 * =========================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// -----------------------------------------------------------------------------
// DEFINICIÓN DE PINES Y CANALES HARDWARE
// -----------------------------------------------------------------------------
#define PIN_SERVO_PAN    18
#define PIN_SERVO_TILT   19
#define PIN_I2C_SDA      21
#define PIN_I2C_SCL      22

#define SERIAL_BAUD_RATE 115200

// -----------------------------------------------------------------------------
// CONFIGURACIÓN DE PWM LEDC (50 Hz / 14 BITS)
// -----------------------------------------------------------------------------
#define LEDC_TIMER_14_BIT   14      // Resolución de 14 bits = 0 a 16383
#define LEDC_BASE_FREQ      50      // Frecuencia estándar para servos (50 Hz = 20 ms)
#define LEDC_MAX_DUTY       16383   // (2^14) - 1

// Canales LEDC para Core v2.x (en Core v3.x se asocian directamente al pin)
#define LEDC_CHANNEL_PAN    0
#define LEDC_CHANNEL_TILT   1

// Rango de microsegundos para servos MG996R
#define SERVO_MIN_US        500     // Pulso para 0 grados
#define SERVO_MAX_US        2500    // Pulso para 180 grados

// -----------------------------------------------------------------------------
// MÁQUINA DE ESTADOS EXPLÍCITA DE ESCANEO
// -----------------------------------------------------------------------------
enum ScanState {
    STATE_IDLE,
    STATE_SCANNING,
    STATE_PAUSED
};

ScanState scanState = STATE_IDLE;

// -----------------------------------------------------------------------------
// VARIABLES DE ESTADO Y TELEMETRÍA
// -----------------------------------------------------------------------------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool sensor_ok = false;

// Estado actual de servos
float current_pan = 90.0;
float current_tilt = 90.0;

// Parámetros dinámicos de escaneo
float scan_pan_min = 50.0;
float scan_pan_max = 122.0;
float scan_tilt_min = 63.0;
float scan_tilt_max = 139.0;
float scan_resolution = 1.0;

// Variables de recorrido de escaneo
float scan_cur_pan = 90.0;
float scan_cur_tilt = 90.0;
int scan_pan_dir = 1;  // +1: izquierda a derecha, -1: derecha a izquierda (recorrido boustrophedon)

// Buffer para recepción serial no bloqueante
String serialBuffer = "";

// -----------------------------------------------------------------------------
// CONTROL PWM DE SERVOMOTORES
// -----------------------------------------------------------------------------
uint32_t angleToDuty(float angle) {
    // Clampear ángulo seguro entre 0° y 180°
    float clamped_angle = constrain(angle, 0.0f, 180.0f);
    
    // Mapeo lineal a microsegundos de pulso
    float pulse_us = SERVO_MIN_US + (clamped_angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    
    // Convertir microsegundos a ciclo de trabajo de 14 bits (periodo de 20,000 us)
    uint32_t duty = (uint32_t)((pulse_us * (float)LEDC_MAX_DUTY) / 20000.0f);
    return duty;
}

void moverPan(float angle) {
    current_pan = constrain(angle, 0.0f, 180.0f);
    uint32_t duty = angleToDuty(current_pan);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PIN_SERVO_PAN, duty);
#else
    ledcWrite(LEDC_CHANNEL_PAN, duty);
#endif
}

void moverTilt(float angle) {
    current_tilt = constrain(angle, 0.0f, 180.0f);
    uint32_t duty = angleToDuty(current_tilt);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PIN_SERVO_TILT, duty);
#else
    ledcWrite(LEDC_CHANNEL_TILT, duty);
#endif
}

void initServos() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // ESP32 Arduino Core 3.x API
    ledcAttach(PIN_SERVO_PAN, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
    ledcAttach(PIN_SERVO_TILT, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
#else
    // ESP32 Arduino Core 2.x API
    ledcSetup(LEDC_CHANNEL_PAN, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
    ledcAttachPin(PIN_SERVO_PAN, LEDC_CHANNEL_PAN);

    ledcSetup(LEDC_CHANNEL_TILT, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
    ledcAttachPin(PIN_SERVO_TILT, LEDC_CHANNEL_TILT);
#endif

    // Inicializar servos en posición central de reposo (90°, 90°)
    moverPan(90.0);
    moverTilt(90.0);
}

// -----------------------------------------------------------------------------
// LECTURA DEL SENSOR VL53L0X
// -----------------------------------------------------------------------------
int medirDistanciaMM() {
    if (!sensor_ok) {
        return -1;
    }

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // false = sin debug por consola serial

    // RangeStatus != 4 indica lectura física procesada
    if (measure.RangeStatus != 4) {
        int dist = (int)measure.RangeMilliMeter;
        // Filtrar lecturas físicas plausibles dentro del rango útil (30 mm a 2000 mm)
        if (dist >= 30 && dist <= 2000) {
            return dist;
        }
    }
    return -1; // Retorna -1 si el haz láser se pierde en el aire vacío o está fuera de rango
}

// -----------------------------------------------------------------------------
// PROCESAMIENTO DE COMANDOS SERIALES
// -----------------------------------------------------------------------------
void procesarComando(const String& cmd) {
    if (cmd.startsWith("CAL:P,")) {
        float p = cmd.substring(6).toFloat();
        moverPan(p);
        int d = medirDistanciaMM();
        Serial.printf("%.1f,%.1f,%d\n", current_pan, current_tilt, d);
    }
    else if (cmd.startsWith("CAL:T,")) {
        float t = cmd.substring(6).toFloat();
        moverTilt(t);
        int d = medirDistanciaMM();
        Serial.printf("%.1f,%.1f,%d\n", current_pan, current_tilt, d);
    }
    else if (cmd.startsWith("CMD:RES,")) {
        float r = cmd.substring(8).toFloat();
        if (r >= 0.1 && r <= 10.0) {
            scan_resolution = r;
        }
    }
    else if (cmd == "CMD:CENTER") {
        moverPan(90.0);
        moverTilt(90.0);
        int d = medirDistanciaMM();
        Serial.printf("%.1f,%.1f,%d\n", current_pan, current_tilt, d);
    }
    else if (cmd.startsWith("CMD:START_SCAN")) {
        // Formato esperado: CMD:START_SCAN,Pmin,Pmax,Tmin,Tmax,Res
        int idx = 0;
        int nextIdx = cmd.indexOf(',', idx); // saltar CMD:START_SCAN
        
        if (nextIdx > 0) {
            idx = nextIdx + 1;
            nextIdx = cmd.indexOf(',', idx);
            if (nextIdx > 0) scan_pan_min = cmd.substring(idx, nextIdx).toFloat();
            
            idx = nextIdx + 1;
            nextIdx = cmd.indexOf(',', idx);
            if (nextIdx > 0) scan_pan_max = cmd.substring(idx, nextIdx).toFloat();

            idx = nextIdx + 1;
            nextIdx = cmd.indexOf(',', idx);
            if (nextIdx > 0) scan_tilt_min = cmd.substring(idx, nextIdx).toFloat();

            idx = nextIdx + 1;
            nextIdx = cmd.indexOf(',', idx);
            if (nextIdx > 0) {
                scan_tilt_max = cmd.substring(idx, nextIdx).toFloat();
                idx = nextIdx + 1;
                scan_resolution = cmd.substring(idx).toFloat();
            } else {
                scan_tilt_max = cmd.substring(idx).toFloat();
            }
        }

        if (scan_resolution < 0.1) scan_resolution = 1.0;
        
        // Inicializar inicio del recorrido
        scan_cur_tilt = scan_tilt_min;
        scan_cur_pan = scan_pan_min;
        scan_pan_dir = 1;
        
        moverTilt(scan_cur_tilt);
        moverPan(scan_cur_pan);
        delay(150); // Tiempo de estabilización mecánica inicial

        scanState = STATE_SCANNING;
        Serial.println("OK:SCAN_STARTED");
    }
    else if (cmd == "CMD:PAUSE_SCAN") {
        if (scanState == STATE_SCANNING) {
            scanState = STATE_PAUSED;
            // Motores permanecen energizados y fijos en scan_cur_pan y scan_cur_tilt
            Serial.println("OK:SCAN_PAUSED");
        }
    }
    else if (cmd == "CMD:RESUME_SCAN") {
        if (scanState == STATE_PAUSED) {
            scanState = STATE_SCANNING;
            // Reanuda suavemente desde la posición exacta congelada
            Serial.println("OK:SCAN_RESUMED");
        }
    }
    else if (cmd == "CMD:STOP_SCAN") {
        scanState = STATE_IDLE;
        moverPan(90.0);
        moverTilt(90.0);
        Serial.println("OK:SCAN_STOPPED");
    }
}

void leerSerial() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                procesarComando(serialBuffer);
            }
            serialBuffer = "";
        } else {
            if (serialBuffer.length() < 120) {
                serialBuffer += c;
            }
        }
    }
}

// -----------------------------------------------------------------------------
// SETUP Y LOOP PRINCIPAL
// -----------------------------------------------------------------------------
void setup() {
    // 1. Inicializar comunicación serie
    Serial.begin(SERIAL_BAUD_RATE);
    serialBuffer.reserve(128);

    // 2. Inicializar bus I2C en los pines dedicados
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000); // Modo I2C Fast Mode (400 kHz)

    // 3. Inicializar sensor VL53L0X
    if (lox.begin()) {
        sensor_ok = true;
    } else {
        sensor_ok = false;
    }

    // 4. Inicializar generador PWM LEDC para ambos servomotores
    initServos();
    delay(200);
}

void loop() {
    // 1. Atender comandos del puerto serie (siempre prioritario y no bloqueante)
    leerSerial();

    // 2. Ejecutar paso del algoritmo de escaneo matricial tipo Boustrophedon
    if (scanState == STATE_SCANNING) {
        moverPan(scan_cur_pan);
        moverTilt(scan_cur_tilt);
        
        // Retardo dinámico para amortiguar la inercia mecánica de los MG996R
        delay(15);

        int d = medirDistanciaMM();
        // Transmite SIEMPRE la trama en cada paso angular, incluso con d = -1 (aire vacío)
        Serial.printf("%.1f,%.1f,%d\n", scan_cur_pan, scan_cur_tilt, d);

        // Avanzar el eje Pan
        scan_cur_pan += (scan_resolution * scan_pan_dir);

        // Comprobar fin de línea horizontal (Pan)
        if (scan_pan_dir > 0 && scan_cur_pan > scan_pan_max) {
            scan_cur_pan = scan_pan_max;
            scan_pan_dir = -1; // Invertir dirección para el siguiente renglón
            scan_cur_tilt += scan_resolution;
            delay(25); // Breve amortiguación al cambiar de renglón
        } else if (scan_pan_dir < 0 && scan_cur_pan < scan_pan_min) {
            scan_cur_pan = scan_pan_min;
            scan_pan_dir = 1; // Invertir dirección
            scan_cur_tilt += scan_resolution;
            delay(25);
        }

        // Comprobar fin de volumen vertical (Tilt completado)
        if (scan_cur_tilt > scan_tilt_max) {
            scanState = STATE_IDLE;
            // Retorno automático suave al centro mecánico (90°, 90°)
            moverPan(90.0);
            moverTilt(90.0);
            Serial.println("OK:SCAN_FINISHED");
        }
    } else {
        // En STATE_IDLE o STATE_PAUSED, reposo de 5 ms para no saturar la CPU
        delay(5);
    }
}
