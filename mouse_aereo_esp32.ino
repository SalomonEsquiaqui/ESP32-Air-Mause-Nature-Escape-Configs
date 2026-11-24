/*
 ==========================================================================
  MOUSE AEREO BLUETOOTH  -  ESP32 + MPU6050
 ==========================================================================

  QUE HACE
  - Convierte el ESP32 en un mouse Bluetooth (BLE) que se mueve girando
    la mano en el aire (giroscopio del MPU6050).
  - Anti-deriva: calibra el giroscopio al encender y, ademas, se
    recalibra solo cada vez que lo dejas quieto, asi el cursor no se
    va solo.

  LED AZUL (el de la placa, GPIO 2)
  - Parpadeo lento (cada 0.3 s) ........ buscando emparejamiento Bluetooth
  - Fijo (siempre encendido) ........... ya esta enlazado con el equipo
  - Parpadeo muy rapido (cada 0.08 s) .. ERROR: no encuentra el MPU6050
  - Al encender parpadea tambien mientras calibra (~2 s): DEJALO QUIETO.

  CONEXIONES
    MPU6050 VCC -> 3V3
    MPU6050 GND -> GND
    MPU6050 SDA -> GPIO 21
    MPU6050 SCL -> GPIO 22
    Boton clic izquierdo -> GPIO 18  y  GND   (opcional)
    Boton clic derecho   -> GPIO 19  y  GND   (opcional)
    (No hace falta resistencia: se usa la interna del ESP32.)

  COMO SOSTENER EL MODULO
  - Plano, con los componentes hacia arriba, y la flecha "Y" impresa en
    la placa apuntando hacia adelante (hacia donde "apuntas").
  - Girar a izquierda/derecha  -> mueve el cursor horizontal.
  - Subir/bajar la punta       -> mueve el cursor vertical.
  - Si tu montaje es distinto, usa INVERTIR_X / INVERTIR_Y o
    INTERCAMBIAR_EJES (mas abajo).

  --------------------------------------------------------------------------
  LIBRERIAS NECESARIAS (instalar ANTES de compilar)
  --------------------------------------------------------------------------
  Arduino IDE > Programa > Incluir Librería > Administrar Bibliotecas:

   1) "Adafruit MPU6050"   (de Adafruit)
      -> si pregunta por dependencias, pulsa "Instalar todo"
         (Adafruit BusIO y Adafruit Unified Sensor).

   2) "ESP32 BLE Mouse" de T-vK  (NO esta en el gestor de bibliotecas)
      -> Descargar ZIP: https://github.com/T-vK/ESP32-BLE-Mouse
         (boton verde "Code" > "Download ZIP")
      -> Arduino IDE > Programa > Incluir Librería > Añadir biblioteca .ZIP

   3) Placa: instalar el paquete "esp32" de Espressif en el Gestor de
      tarjetas y elegir "ESP32 Dev Module" (o tu modelo).

  IMPORTANTE - ERROR CON PAQUETE ESP32 VERSION 3.x
  Si al compilar sale:
     "cannot convert 'std::string' to 'String'"   en BleMouse.cpp
  es porque la libreria de T-vK es antigua. Solucion (2 lineas):
  abre  Documentos\Arduino\libraries\ESP32_BLE_Mouse\BleMouse.cpp  y cambia

     BLEDevice::init(bleMouseInstance->deviceName);
   por
     BLEDevice::init(String(bleMouseInstance->deviceName.c_str()));

     ...->manufacturer()->setValue(bleMouseInstance->deviceManufacturer);
   por
     ...->manufacturer()->setValue(String(bleMouseInstance->deviceManufacturer.c_str()));

  Guarda, cierra y reabre el Arduino IDE.
  (Alternativa: instalar el paquete esp32 version 2.0.17, donde la
   libreria funciona sin tocar nada.)

  IMPORTANTE: la carpeta del sketch debe llamarse igual que este archivo:
  mouse_aereo_esp32/mouse_aereo_esp32.ino

  --------------------------------------------------------------------------
  COMO EMPAREJAR
  --------------------------------------------------------------------------
  1. Sube el codigo y enciende el ESP32. LED parpadeando.
  2. Deja el modulo quieto sobre la mesa ~2 segundos (calibracion).
  3. En el PC/celular: Bluetooth > buscar dispositivos > "Mouse Aereo ESP32".
  4. Al enlazar, el LED queda fijo. Ya puedes mover el cursor.
  5. Si borras el mouse del equipo y no vuelve a aparecer, reinicia el ESP32.

  --------------------------------------------------------------------------
  AJUSTES RAPIDOS (seccion "AJUSTES" mas abajo)
  --------------------------------------------------------------------------
  - Muy lento/rapido ........ PIXELES_POR_GRADO_X / _Y (sube = mas rapido)
  - Cursor va al reves ...... INVERTIR_X / INVERTIR_Y = true
  - Ejes cruzados ........... INTERCAMBIAR_EJES = true
  - Se mueve solo (deriva) .. sube ZONA_MUERTA (0.025 -> 0.04)
  - Tiembla al apuntar ...... sube FILTRO (0.25 -> 0.5)
  - Responde con retraso .... baja FILTRO (0.25 -> 0.1)
 ==========================================================================
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>

#if __has_include(<BLEDevice.h>)
  #include <BLEDevice.h>
  #define PUEDE_REANUNCIAR 1
#endif

// ======================= PINES =======================
#define SDA_PIN   21
#define SCL_PIN   22
#define LED_PIN    2    // LED azul de la placa ESP32 DevKit (cambia si tu placa es otra)
#define BTN_IZQ   18
#define BTN_DER   19

// ======================= AJUSTES =======================
// Sensibilidad: pixeles que se mueve el cursor por cada grado que giras.
const float PIXELES_POR_GRADO_X = 20.0;
const float PIXELES_POR_GRADO_Y = 20.0;

const bool INVERTIR_X        = false;  // true si el cursor se mueve al lado contrario (horizontal)
const bool INVERTIR_Y        = false;  // true si se mueve al reves (vertical)
const bool INTERCAMBIAR_EJES = false;  // true si girar a los lados mueve vertical y viceversa

// Anti-deriva
const float ZONA_MUERTA      = 0.025;  // rad/s (~1.4 grados/s). Por debajo de esto el cursor no se mueve
const float FILTRO           = 0.25;   // 0 = sin filtro, 0.9 = muy suave pero con retraso
const float UMBRAL_QUIETO    = 0.04;   // rad/s: por debajo de esto se considera "quieto"
const unsigned long TIEMPO_QUIETO_MS = 400;  // tiempo quieto antes de recalibrar solo
const float VELOCIDAD_RECALIB = 0.02;  // 0.001 (lento) a 0.1 (rapido): ritmo de autocorreccion

// Calibracion inicial
const int   MUESTRAS_CALIB   = 400;    // ~2 segundos
const float LIMITE_MOVIMIENTO_CALIB = 0.15; // rad/s: si te mueves mas que esto, repite la calibracion

const unsigned long PERIODO_US = 10000; // 10 ms = 100 actualizaciones por segundo

const bool DEBUG_SERIAL = false;       // true para ver datos en el Monitor Serial (115200)

// ======================= VARIABLES =======================
Adafruit_MPU6050 mpu;
BleMouse bleMouse("Mouse Aereo ESP32", "DIY", 100);

float offX = 0, offY = 0, offZ = 0;    // error (sesgo) del giroscopio
float filtH = 0, filtV = 0;            // velocidades filtradas
float restoX = 0, restoY = 0;          // decimales acumulados del movimiento
unsigned long ultimoUs = 0;
unsigned long inicioQuietoMs = 0;

bool conectadoAntes = false;
unsigned long desconexionMs = 0;
bool reanuncioPendiente = false;

struct Boton {
  uint8_t pin;
  uint8_t mascara;
  bool estado;
  bool lectura;
  unsigned long tCambio;
};
Boton botones[] = {
  { BTN_IZQ, MOUSE_LEFT,  false, false, 0 },
  { BTN_DER, MOUSE_RIGHT, false, false, 0 }
};

const float RAD_A_GRADOS = 57.29578f;

// ======================= LED =======================
// Parpadea mientras busca emparejamiento y queda fijo al conectarse.
void actualizarLed() {
  static unsigned long t = 0;
  static bool encendido = false;
  if (bleMouse.isConnected()) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  if (millis() - t >= 300) {
    t = millis();
    encendido = !encendido;
    digitalWrite(LED_PIN, encendido);
  }
}

// Error fatal: LED parpadeando muy rapido
void errorFatal(const char* msg) {
  Serial.println(msg);
  while (true) {
    digitalWrite(LED_PIN, HIGH); delay(80);
    digitalWrite(LED_PIN, LOW);  delay(80);
  }
}

// ======================= CALIBRACION =======================
void calibrarGiroscopio() {
  Serial.println("Calibrando... deja el modulo QUIETO.");
  for (int intento = 1; intento <= 10; intento++) {
    float sx = 0, sy = 0, sz = 0;
    float minX = 1e9, minY = 1e9, minZ = 1e9;
    float maxX = -1e9, maxY = -1e9, maxZ = -1e9;

    for (int i = 0; i < MUESTRAS_CALIB; i++) {
      actualizarLed();
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      sx += g.gyro.x;  sy += g.gyro.y;  sz += g.gyro.z;
      minX = fminf(minX, g.gyro.x);  maxX = fmaxf(maxX, g.gyro.x);
      minY = fminf(minY, g.gyro.y);  maxY = fmaxf(maxY, g.gyro.y);
      minZ = fminf(minZ, g.gyro.z);  maxZ = fmaxf(maxZ, g.gyro.z);
      delay(4);
    }

    offX = sx / MUESTRAS_CALIB;
    offY = sy / MUESTRAS_CALIB;
    offZ = sz / MUESTRAS_CALIB;

    float rango = fmaxf(fmaxf(maxX - minX, maxY - minY), maxZ - minZ);
    if (rango < LIMITE_MOVIMIENTO_CALIB) {
      Serial.println("Calibracion lista.");
      return;
    }
    Serial.println("Se detecto movimiento, repitiendo calibracion...");
  }
  Serial.println("Calibracion aceptada (el modulo no estuvo del todo quieto).");
}

// ======================= BOTONES =======================
void leerBotones() {
  for (auto &b : botones) {
    bool lec = (digitalRead(b.pin) == LOW);
    if (lec != b.lectura) {
      b.lectura = lec;
      b.tCambio = millis();
    }
    if ((millis() - b.tCambio) >= 20 && b.estado != b.lectura) {  // antirrebote 20 ms
      b.estado = b.lectura;
      if (b.estado) bleMouse.press(b.mascara);
      else          bleMouse.release(b.mascara);
    }
  }
}

// Resta la zona muerta de forma suave (sin saltos)
float aplicarZonaMuerta(float v) {
  if (v >  ZONA_MUERTA) return v - ZONA_MUERTA;
  if (v < -ZONA_MUERTA) return v + ZONA_MUERTA;
  return 0;
}

// Si el bluetooth se desconecta, vuelve a anunciarse para poder reconectar
void gestionarReconexion() {
  bool conectado = bleMouse.isConnected();
  if (conectadoAntes && !conectado) {
    desconexionMs = millis();
    reanuncioPendiente = true;
  }
  conectadoAntes = conectado;

#ifdef PUEDE_REANUNCIAR
  if (reanuncioPendiente && !conectado && (millis() - desconexionMs > 1000)) {
    BLEDevice::startAdvertising();
    reanuncioPendiente = false;
  }
#endif
  if (conectado) reanuncioPendiente = false;
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BTN_IZQ, INPUT_PULLUP);
  pinMode(BTN_DER, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin(0x68, &Wire)) {   // si el pin AD0 esta a 3V3, cambia 0x68 por 0x69
    errorFatal("ERROR: no se encontro el MPU6050. Revisa SDA/SCL/VCC/GND.");
  }
  Serial.println("MPU6050 encontrado.");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // Bluetooth se inicia primero: el LED ya parpadea (buscando) durante la calibracion
  bleMouse.begin();
  Serial.println("Bluetooth iniciado. Busca 'Mouse Aereo ESP32' en tu equipo.");

  calibrarGiroscopio();
  ultimoUs = micros();
}

// ======================= LOOP =======================
void loop() {
  actualizarLed();
  gestionarReconexion();

  unsigned long ahoraUs = micros();
  if (ahoraUs - ultimoUs < PERIODO_US) {
    delay(1);
    return;
  }
  float dt = (ahoraUs - ultimoUs) / 1000000.0f;
  ultimoUs = ahoraUs;
  if (dt > 0.1f) dt = 0.1f;  // proteccion si hubo una pausa larga

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Giroscopio menos el sesgo (rad/s)
  float rx = g.gyro.x - offX;
  float ry = g.gyro.y - offY;
  float rz = g.gyro.z - offZ;

  // ---- Anti-deriva: si esta quieto un rato, corrige el sesgo solo ----
  if (fabsf(rx) < UMBRAL_QUIETO && fabsf(ry) < UMBRAL_QUIETO && fabsf(rz) < UMBRAL_QUIETO) {
    if (inicioQuietoMs == 0) inicioQuietoMs = millis();
    if (millis() - inicioQuietoMs > TIEMPO_QUIETO_MS) {
      offX += rx * VELOCIDAD_RECALIB;
      offY += ry * VELOCIDAD_RECALIB;
      offZ += rz * VELOCIDAD_RECALIB;
      rx = g.gyro.x - offX;
      ry = g.gyro.y - offY;
      rz = g.gyro.z - offZ;
    }
  } else {
    inicioQuietoMs = 0;
  }

  // Giro lateral (eje Z) -> horizontal ; subir/bajar punta (eje X) -> vertical
  float velH = rz;
  float velV = rx;
  if (INTERCAMBIAR_EJES) { float tmp = velH; velH = velV; velV = tmp; }

  velH = aplicarZonaMuerta(velH);
  velV = aplicarZonaMuerta(velV);

  filtH = FILTRO * filtH + (1.0f - FILTRO) * velH;
  filtV = FILTRO * filtV + (1.0f - FILTRO) * velV;

  if (bleMouse.isConnected()) {
    // rad/s -> grados -> pixeles (independiente del tiempo del ciclo)
    float mx = -filtH * RAD_A_GRADOS * dt * PIXELES_POR_GRADO_X;
    float my = -filtV * RAD_A_GRADOS * dt * PIXELES_POR_GRADO_Y;
    if (INVERTIR_X) mx = -mx;
    if (INVERTIR_Y) my = -my;

    // Acumula decimales para que los movimientos lentos no se pierdan
    restoX += mx;
    restoY += my;
    int dx = (int)restoX;
    int dy = (int)restoY;
    restoX -= dx;
    restoY -= dy;

    dx = constrain(dx, -127, 127);
    dy = constrain(dy, -127, 127);

    if (dx != 0 || dy != 0) {
      bleMouse.move((signed char)dx, (signed char)dy);
    }
    leerBotones();
  } else {
    restoX = 0;
    restoY = 0;
  }

  if (DEBUG_SERIAL) {
    Serial.printf("gx=%.3f gz=%.3f offX=%.4f offZ=%.4f\n", rx, rz, offX, offZ);
  }
}
