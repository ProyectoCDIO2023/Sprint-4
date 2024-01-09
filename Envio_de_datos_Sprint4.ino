#include <ESP8266WiFi.h>

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

#define Offset 0.60          // Offset para el cálculo del pH (ajuste específico según el sensor)
#define samplingInterval 20  // Intervalo de tiempo entre muestras de pH (en milisegundos)
#define printInterval 800    // Intervalo de tiempo para imprimir los resultados (en milisegundos)
#define ArrayLength 50       // Longitud del array de muestras de pH
#define power_pin 5

float pHArray[ArrayLength];  // almacena las muestras
int pHArrayIndex;            // Índice actual en el array de muestras de pH

int medidaSeco = 0;
int medidaMojado = 0;

// Umbrales de iluminación basados en los voltajes medidos
const float umbralOscuridad = 60;    // Voltaje para la oscuridad
const float umbralSombra = 80;       // Voltaje para la sombra
const float umbralLuzAmbiente = 117;  // Voltaje para luz ambiente
const float umbralLinterna = 700;    // Voltaje con linterna del móvil


// Comentar/Descomentar para ver mensajes de depuracion en monitor serie y/o respuesta del HTTP server
#define PRINT_DEBUG_MESSAGES
//#define PRINT_HTTP_RESPONSE

// Comentar/Descomentar para conexion Fuera/Dentro de UPV
//#define WiFi_CONNECTION_UPV

// Selecciona que servidor REST quieres utilizar entre ThingSpeak y Dweet
#define REST_SERVER_THINGSPEAK  //Selecciona tu canal para ver los datos en la web (https://thingspeak.com/channels/360979)
//#define REST_SERVER_DWEET //Selecciona tu canal para ver los datos en la web (http://dweet.io/follow/PruebaGTI)

///////////////////////////////////////////////////////
/////////////// WiFi Definitions /////////////////////
//////////////////////////////////////////////////////

#ifdef WiFi_CONNECTION_UPV  //Conexion UPV
const char WiFiSSID[] = "GTI1";
const char WiFiPSK[] = "1PV.arduino.Toledo";
#else  //Conexion fuera de la UPV
const char WiFiSSID[] = "HUAWEI P30 Pro";
const char WiFiPSK[] = "b099abc505d0";
#endif



///////////////////////////////////////////////////////
/////////////// SERVER Definitions /////////////////////
//////////////////////////////////////////////////////

#if defined(WiFi_CONNECTION_UPV)  //Conexion UPV
const char Server_Host[] = "proxy.upv.es";
const int Server_HttpPort = 8080;
#elif defined(REST_SERVER_THINGSPEAK)  //Conexion fuera de la UPV
const char Server_Host[] = "api.thingspeak.com";
const int Server_HttpPort = 80;
#else
const char Server_Host[] = "dweet.io";
const int Server_HttpPort = 80;
#endif

WiFiClient client;

///////////////////////////////////////////////////////
/////////////// HTTP REST Connection ////////////////
//////////////////////////////////////////////////////

#ifdef REST_SERVER_THINGSPEAK
const char Rest_Host[] = "api.thingspeak.com";
String MyWriteAPIKey = "2O15UAPV0OY6TGAM";  // Escribe la clave de tu canal ThingSpeak
#else
const char Rest_Host[] = "dweet.io";
String MyWriteAPIKey = "cdiocurso2018g04";  // Escribe la clave de tu canal Dweet
#endif

#define NUM_FIELDS_TO_SEND 5  //Numero de medidas a enviar al servidor REST (Entre 1 y 8)

/////////////////////////////////////////////////////
/////////////// Pin Definitions ////////////////
//////////////////////////////////////////////////////

const int LED_PIN = 5;  // Thing's onboard, green LED

/////////////////////////////////////////////////////
/////////////// WiFi Connection ////////////////
//////////////////////////////////////////////////////

void connectWiFi() {
  byte ledStatus = LOW;

#ifdef PRINT_DEBUG_MESSAGES
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
#endif

  WiFi.begin(WiFiSSID, WiFiPSK);

  while (WiFi.status() != WL_CONNECTED) {
    // Blink the LED
    digitalWrite(LED_PIN, ledStatus);  // Write LED high/low
    ledStatus = (ledStatus == HIGH) ? LOW : HIGH;
#ifdef PRINT_DEBUG_MESSAGES
    Serial.println(".");
#endif
    delay(500);
  }
#ifdef PRINT_DEBUG_MESSAGES
  Serial.println("WiFi Connected");
  Serial.println(WiFi.localIP());  // Print the IP address
#endif
}

/////////////////////////////////////////////////////
/////////////// HTTP POST  ThingSpeak////////////////
//////////////////////////////////////////////////////

void HTTPPost(String fieldData[], int numFields) {

  // Esta funcion construye el string de datos a enviar a ThingSpeak mediante el metodo HTTP POST
  // La funcion envia "numFields" datos, del array fieldData.
  // Asegurate de ajustar numFields al número adecuado de datos que necesitas enviar y activa los campos en tu canal web

  if (client.connect(Server_Host, Server_HttpPort)) {

    // Construimos el string de datos. Si tienes multiples campos asegurate de no pasarte de 1440 caracteres

    String PostData = "api_key=" + MyWriteAPIKey;
    for (int field = 1; field < (numFields + 1); field++) {
      PostData += "&field" + String(field) + "=" + fieldData[field];
    }

// POST data via HTTP
#ifdef PRINT_DEBUG_MESSAGES
    Serial.println("Connecting to ThingSpeak for update...");
#endif
    client.println("POST http://" + String(Rest_Host) + "/update HTTP/1.1");
    client.println("Host: " + String(Rest_Host));
    client.println("Connection: close");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + String(PostData.length()));
    client.println();
    client.println(PostData);
#ifdef PRINT_DEBUG_MESSAGES
    Serial.println(PostData);
    Serial.println();
//Para ver la respuesta del servidor
#ifdef PRINT_HTTP_RESPONSE
    delay(500);
    Serial.println();
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    Serial.println();
    Serial.println();
#endif
#endif
  }
}

////////////////////////////////////////////////////
/////////////// HTTP GET  ////////////////
//////////////////////////////////////////////////////

void HTTPGet(String fieldData[], int numFields) {

  // Esta funcion construye el string de datos a enviar a ThingSpeak o Dweet mediante el metodo HTTP GET
  // La funcion envia "numFields" datos, del array fieldData.
  // Asegurate de ajustar "numFields" al número adecuado de datos que necesitas enviar y activa los campos en tu canal web

  if (client.connect(Server_Host, Server_HttpPort)) {
#ifdef REST_SERVER_THINGSPEAK
    String PostData = "GET https://api.thingspeak.com/update?api_key=";
    PostData = PostData + MyWriteAPIKey;
#else
    String PostData = "GET http://dweet.io/dweet/for/";
    PostData = PostData + MyWriteAPIKey + "?";
#endif

    for (int field = 1; field < (numFields + 1); field++) {
      PostData += "&field" + String(field) + "=" + fieldData[field];
    }


#ifdef PRINT_DEBUG_MESSAGES
    Serial.println("Connecting to Server for update...");
#endif
    client.print(PostData);
    client.println(" HTTP/1.1");
    client.println("Host: " + String(Rest_Host));
    client.println("Connection: close");
    client.println();
#ifdef PRINT_DEBUG_MESSAGES
    Serial.println(PostData);
    Serial.println();
//Para ver la respuesta del servidor
#ifdef PRINT_HTTP_RESPONSE
    delay(500);
    Serial.println();
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    Serial.println();
    Serial.println();
#endif
#endif
  }
}



void setup() {

#ifdef PRINT_DEBUG_MESSAGES
  Serial.begin(9600);
  ads.begin();
  ads.setGain(GAIN_ONE);

#endif

  connectWiFi();
  digitalWrite(LED_PIN, HIGH);

#ifdef PRINT_DEBUG_MESSAGES
  Serial.print("Server_Host: ");
  Serial.println(Server_Host);
  Serial.print("Port: ");
  Serial.println(String(Server_HttpPort));
  Serial.print("Server_Rest: ");
  Serial.println(Rest_Host);
#endif
}



int readTemperatura() {
  int16_t adc0 = ads.readADC_SingleEnded(1);
  float voltage = (adc0 * 4.096) / 32767;

  // Mostrar el valor de voltaje leído
  Serial.print("Voltage: ");
  Serial.print(voltage, 4);  // Mostrar 4 decimales para mayor precisión
  Serial.println(" V");

  // Convertir el valor analógico a temperatura usando la fórmula
  int temperature = ((voltage - 0.79) / 0.035);
  return temperature;
}

int calculatepH(int adcValue) {
  int voltage = adcValue * 0.0001875;  // La resolución es 0.1875 mV por bit para GAIN_TWOTHIRDS
  int pHValue = 3.5 * voltage + Offset;
  return pHValue;
}

String measureSalinity() {
  digitalWrite(power_pin, HIGH);  // Alimentar la sonda con un tren de pulsos
  delay(100);

  int16_t adc0 = analogRead(A0);  // Leer cuando hay un nivel alto
  digitalWrite(power_pin, LOW);
  delay(100);

  float salinidad = map(adc0, 0, 1023, 0, 100);  // Realizar el cálculo de la salinidad
  Serial.print("Lectura digital = ");
  Serial.println(adc0, DEC);
  Serial.print("Salinidad de lectura = ");
  Serial.println(salinidad, 2);

float salinidadReal = 480 + 15.55*salinidad - 0.31*pow(salinidad, 2) - 0.012*pow(salinidad, 3) + 0.0005*pow(salinidad, 4) - 0.000002*pow(salinidad, 5);
  Serial.print("Salinidad de cálculo = ");
  Serial.println(salinidadReal, 2);

  return String(salinidadReal);

  delay(1000);  // Puedes ajustar el tiempo de espera entre lecturas
}


void loop() {

  String data[NUM_FIELDS_TO_SEND + 1];  // Podemos enviar hasta 8 datos

  int humidityValueMojado = readHumedadMojado();
  int temperature = readTemperatura();

  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float voltage, pHValue;


  // Realizar muestreo a intervalos regulares
  if (millis() - samplingTime > samplingInterval) {
    // Realizar una lectura del canal del ADS1115
    int16_t adcValue;
    adcValue = ads.readADC_SingleEnded(3);

    // Calcular el valor de pH
    pHValue = calculatepH(adcValue);
    voltage = adcValue * 0.0001875;  // Convertir a voltaje

    // Almacenar la lectura en el array
    pHArray[pHArrayIndex++] = pHValue;
    if (pHArrayIndex == ArrayLength) pHArrayIndex = 0;

    // Reiniciar el tiempo de muestreo
    samplingTime = millis();
  }

  // Imprimir los resultados a intervalos regulares
  if (millis() - printTime > printInterval) {
    Serial.print("Voltage: ");
    Serial.print(voltage, 2);
    Serial.print("    pH value: ");
    Serial.println(pHValue, 2);
    Serial.print("Humedad (mojado): ");
    Serial.print(humidityValueMojado);
    Serial.println("%");
    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.println(" °C");


    printTime = millis();
  }
  measureSalinity();
  measureLightLevel();
  // Esperar 5 segundos antes de la próxima lectura
  delay(5000);


  String salinidadString = measureSalinity();
  String LightLevelString = measureLightLevel();
  data[1] = String(humidityValueMojado);  // Actualizar el dato de humedad
  data[2] = String(temperature);          // Actualizar el dato de temperatura
  data[3] = String(pHValue);              // Actualizar el dato de pH
  data[4] = String(salinidadString);      // Actualizar el dato de salinidad
  data[5] = String(LightLevelString);     // Actualizar el dato de Luz

  //Selecciona si quieres enviar con GET(ThingSpeak o Dweet) o con POST(ThingSpeak)
  //HTTPPost( data, NUM_FIELDS_TO_SEND );
  HTTPGet(data, NUM_FIELDS_TO_SEND);

  //Selecciona si quieres un retardo de 15seg para hacer pruebas o dormir el SparkFun
  delay(15000);
  //Serial.print( "Goodnight" );
  //ESP.deepSleep( sleepTimeSeconds * 1000000 );
}


int readHumedadMojado() {
  medidaMojado = ads.readADC_SingleEnded(2);

  // Mapear el valor leído para obtener la humedad en estado mojado
  int humidityValueMojado = map(medidaMojado, 17000, 31000, 100, 0);

  // Mostrar el valor crudo de humedad
  Serial.print("Medida mojado (Humedad): ");
  Serial.println(medidaMojado);

  return humidityValueMojado;
}

String measureLightLevel() {
  // Leer del canal 0 del ADS1115
  int16_t lectura = ads.readADC_SingleEnded(0);
  float voltaje = lectura * 0.002; // Convertir la lectura en milivoltios

  // Crear un string para almacenar la lectura
  String lecturaString = "ADC0: " + String(lectura) + " - Voltaje: " + String(voltaje, 2) + " mV";

  // Comparar la lectura con los umbrales y agregar el resultado al string
  if (lectura < umbralOscuridad) {
    lecturaString += " - Oscuridad";
  } else if (lectura < umbralSombra) {
    lecturaString += " - Sombra";
  } else if (lectura < umbralLuzAmbiente) {
    lecturaString += " - Luz ambiente";
  } else if (lectura < umbralLinterna) {
    lecturaString += " - Nivel alto de iluminación";
  } else {
    lecturaString += " - Muy iluminado";
  }

  // Retornar el string con la lectura
  return lecturaString;
}


