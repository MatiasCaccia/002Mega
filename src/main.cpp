#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <TimeLib.h>

// PINOUT
#define BMP_SCK  (52)
#define BMP_MISO (50)
#define BMP_MOSI (51)
#define BMP_CS_1 (49)
#define BMP_CS_2 (48)

// CONSTANTES
Adafruit_BMP280 bmp(BMP_CS_1); // hardware SPI
Adafruit_BMP280 bmp2(BMP_CS_2); // hardware SPI

// Coeficientes de calibración de cada sensor
float p_a1 = 1.0; // Pendiente sensor 1
float p_b1 = 0.0; // Ordenada sensor 1
float p_a2 = 1.0; // Pendiente sensor 2
float p_b2 = 0.0; // Ordenada sensor 2
float t_a1 = 1.0; // Pendiente sensor 1
float t_b1 = 0.0; // Ordenada sensor 1
//float t_a2 = 1.0; // Pendiente sensor 2
//float t_b2 = 0.0; // Ordenada sensor 2

// Filtro media movil
#define WINDOW_SIZE 5 // Tamaño del filtro
double READINGS_P_1[WINDOW_SIZE];
double READINGS_T_1[WINDOW_SIZE];
double READINGS_P_2[WINDOW_SIZE];
double READINGS_T_2[WINDOW_SIZE];

int INDEX_P_1 = 0;
int INDEX_T_1 = 0;
int INDEX_P_2 = 0;
int INDEX_T_2 = 0;

double SUM_P_1 = 0;
double SUM_T_1 = 0;
double SUM_P_2 = 0;
double SUM_T_2 = 0;

float AVERAGED_P_1 = 0;
float AVERAGED_T_1 = 0;
float AVERAGED_P_2 = 0;
float AVERAGED_T_2 = 0;

// Shield Ethernet
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Conexión a MQTT
IPAddress mqtt_server(192, 168, 0, 100); // IP Router SARU
IPAddress ip(192, 168, 3, 30); // IP del Arduino
EthernetClient ethClient;
PubSubClient client(ethClient);
const char* sync_topic = "Sincro";  // MQTT topic to listen for synchronization messages
const char* data_topic = "Sensor1";  // MQTT topic to publish random numbers with timestamp

// Sincronización
bool synced = false;  // Bandera para indicar inicio/fin de sincronización
float epoch;

// -------------------------------
// FUNCIONES
// -------------------------------

float calibrate(float lectura, float a, float b) {
  // Función de calibración
  return lectura * a + b;
}

void callback(char* topic, byte* payload, unsigned int length);

// -------------------------------
// SETUP
// -------------------------------
void setup() {
  Serial.begin(115200);
  //while (!Serial); // wait for native usb
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  unsigned status2 = bmp2.begin();
  if (!status) {
    Serial.println(F("No se envuentra el BMP1"));
    Serial.print("SensorID: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  if (!status2) {
    Serial.println(F("No se encuentra el BMP2"));
    Serial.print("SensorID: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  //unsigned status;
  // Inicializar ambos sensores BMP280
  /*status = bmp.begin();
  unsigned status2 = bmp2.begin();
  if (!status || !status2) {
    Serial.println(F("Error al inicializar los sensores BMP"));
    while (1);
  }
  */
  // Configurar ambos sensores BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_125);
  bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16,
                   Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_125);

  // Configurar Ethernet y MQTT
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    while (true) {
      delay(1000);
    }
  }

  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("arduino_client")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
  client.subscribe(sync_topic); // Suscripción al tópico de sincronismo
  client.setCallback(callback);
}

// -------------------------------
// LOOP
// -------------------------------
void loop() {
  client.loop();
  if (!synced) {
    return;
  }
  
  float VALUE_P_1 = 0.0;
  float VALUE_T_1 = 0.0;
  float VALUE_P_2 = 0.0;
  //float VALUE_T_2 = 0.0;

  SUM_P_1 = SUM_P_1 - READINGS_P_1[INDEX_P_1];
  SUM_T_1 = SUM_T_1 - READINGS_T_1[INDEX_T_1];
  SUM_P_2 = SUM_P_2 - READINGS_P_2[INDEX_P_2];
  //SUM_T_2 = SUM_T_2 - READINGS_T_2[INDEX_T_2];

  VALUE_P_1 = calibrate(bmp.readPressure(), p_a1, p_b1);
  VALUE_T_1 = calibrate(bmp.readTemperature(), t_a1, t_b1);
  VALUE_P_2 = calibrate(bmp2.readPressure(), p_a2, p_b2);
  //VALUE_T_2 = calibrate(bmp2.readTemperature(), t_a2, t_b2);

  READINGS_P_1[INDEX_P_1] = VALUE_P_1;
  READINGS_T_1[INDEX_T_1] = VALUE_T_1;
  READINGS_P_2[INDEX_P_2] = VALUE_P_2;
  //READINGS_T_2[INDEX_T_2] = VALUE_T_2;

  SUM_P_1 = SUM_P_1 + VALUE_P_1;
  SUM_T_1 = SUM_T_1 + VALUE_T_1;
  SUM_P_2 = SUM_P_2 + VALUE_P_2;
  //SUM_T_2 = SUM_T_2 + VALUE_T_2;

  INDEX_P_1 = (INDEX_P_1 + 1) % WINDOW_SIZE;
  INDEX_T_1 = (INDEX_T_1 + 1) % WINDOW_SIZE;
  INDEX_P_2 = (INDEX_P_2 + 1) % WINDOW_SIZE;
  //INDEX_T_2 = (INDEX_T_2 + 1) % WINDOW_SIZE;

  AVERAGED_P_1 = SUM_P_1 / WINDOW_SIZE;
  AVERAGED_T_1 = SUM_T_1 / WINDOW_SIZE;
  AVERAGED_P_2 = SUM_P_2 / WINDOW_SIZE;
  //AVERAGED_T_2 = SUM_T_2 / WINDOW_SIZE;

  Serial.print(F("T1 = ")); Serial.print(AVERAGED_T_1, 1); Serial.println(" *C");
  //Serial.print(F("T2 = ")); Serial.print(AVERAGED_T_2, 1); Serial.println(" *C");
  Serial.print(F("P1 = ")); Serial.print(AVERAGED_P_1, 1); Serial.println(" Pa");
  Serial.print(F("P2 = ")); Serial.print(AVERAGED_P_2, 1); Serial.println(" Pa");
  Serial.println();

  // Append the synchronized time to the payload
  char payload[100];
  time_t ahora = now();
  // Clear the contents of the resultString
  memset(payload, 0, sizeof(payload));
  char message[50];
  sprintf(message, "%lu", ahora);

  // Convertir el float registrado en string con dtostrf()
  char floatString[10]; // Char array to store the float as a string
  char floatStringT[10]; // Char array to store the float as a string
  char floatString2[10]; // Char array to store the float as a string
  dtostrf(AVERAGED_P_1, 4, 2, floatString); // Format: 4 digits, 2 decimal places
  dtostrf(AVERAGED_T_1, 4, 2, floatStringT); // Format: 4 digits, 2 decimal places
  dtostrf(AVERAGED_P_2, 4, 2, floatString2); // Format: 4 digits, 2 decimal places

  // Concatenar el float, separador y string
  strcat(payload, message); //Timestamp
  strcat(payload, " | ");
  strcat(payload, "Nodo1"); //Presión del sensor 1
  strcat(payload, " | ");
  strcat(payload, floatString); //Presión del sensor 1
  strcat(payload, " | ");
  strcat(payload, floatString2); //Presión del sensor 1
  strcat(payload, " | ");
  strcat(payload, floatStringT); //Temperatura del sensor 1  
  client.publish(data_topic, payload);

  delay(1000);
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (strncmp(topic, sync_topic, length) == 0) {
    payload[length] = '\0'; // Agregar el terminador nulo al final del mensaje
    String time_str = "";
    for (unsigned int i = 0; i < length; i++) {
      time_str += (char)payload[i];
    }
    time_t time_value = atol(time_str.c_str());
    setTime(time_value);
    epoch = atof((const char*)payload); // Convertir la cadena a un valor de punto flotante (float)
    synced = true;
    if (epoch == 0.0) {
      synced = false;
    }
  }
}
