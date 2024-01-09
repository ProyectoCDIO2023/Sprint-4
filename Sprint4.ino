#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP8266WiFi.h>

Adafruit_ADS1115 ads;

#define Offset 0.60        // Offset para el cálculo del pH (ajuste específico según el sensor)
#define samplingInterval 20 // Intervalo de tiempo entre muestras de pH (en milisegundos)
#define printInterval 800  // Intervalo de tiempo para imprimir los resultados (en milisegundos)
#define ArrayLength 50     // Longitud del array de muestras de pH
#define power_pin 5

float pHArray[ArrayLength]; // almacena las muestras 
int pHArrayIndex; // Índice actual en el array de muestras de pH

int medidaSeco = 0;
int medidaMojado = 0;

// Umbrales de iluminación basados en los voltajes medidos
const float umbralOscuridad = 60;    // Voltaje para la oscuridad
const float umbralSombra = 80;       // Voltaje para la sombra
const float umbralLuzAmbiente = 117;  // Voltaje para luz ambiente
const float umbralLinterna = 700;    // Voltaje con linterna del móvil

// Tiempo en segundos para dormir (15 minutos = 900 segundos)
const int sleepTimeS = 900;

void setup() {
  Serial.begin(9600);
  Serial.println("Inicializando...");

  ads.begin();
  ads.setGain(GAIN_ONE);
}

float calculatepH(int adcValue) {
  float voltage = adcValue * 0.0001875; // La resolución es 0.1875 mV por bit para GAIN_TWOTHIRDS
  float pHValue = 3.5 * voltage + Offset;
  return pHValue;
}

void loop() {
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float voltage, pHValue;

  // Llamar a las funciones para obtener los valores de humedad y temperatura
  int humidityValueMojado = readHumedadMojado();
  int temperature = readTemperatura();

  // Realizar muestreo a intervalos regulares
  if (millis() - samplingTime > samplingInterval) {
    // Realizar una lectura del canal del ADS1115
    int16_t adcValue;
    adcValue = ads.readADC_SingleEnded(3);

    // Calcular el valor de pH 
    pHValue = calculatepH(adcValue);
    voltage = adcValue * 0.0001875; // Convertir a voltaje

    // Almacenar la lectura en el array
    pHArray[pHArrayIndex++] = pHValue;
    if (pHArrayIndex == ArrayLength) pHArrayIndex = 0;

    // Reiniciar el tiempo de muestreo
    samplingTime = millis();
  }
  measureLightLevel(); // Llama a la función para medir el nivel de iluminación

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

  // Dormir durante 15 minutos
  ESP.deepSleep(sleepTimeS * 1000000);
}

// Función para leer el valor de humedad en estado mojado
int readHumedadMojado() {
  medidaMojado = ads.readADC_SingleEnded(2);
  int humidityValueMojado = map(medidaMojado, 17000, 31000, 100, 0);
  return humidityValueMojado;
}

// Función para leer el valor de temperatura
int readTemperatura() {
  int16_t adcValue = ads.readADC_SingleEnded(1);
  float voltage = (adcValue * 4.096) / 32767;
  int temperature = ((voltage - 0.79) / 0.035);
  return temperature;
}

void measureSalinity() {
  digitalWrite(power_pin, HIGH); // Alimentar la sonda con un tren de pulsos
  delay(100);

  int16_t adc0 = analogRead(A0); // Leer cuando hay un nivel alto
  digitalWrite(power_pin, LOW);
  delay(100);

  float salinidad = map(adc0, 0, 1023, 0, 100); // Realizar el cálculo de la salinidad
  Serial.print("Lectura digital = ");
  Serial.println(adc0, DEC);
  Serial.print("Salinidad de lectura = ");
  Serial.println(salinidad, 2);

  float salinidadReal = 480 + 15.55 * salinidad - 0.31 * pow(salinidad, 2) - 0.012 * pow(salinidad, 3) + 0.0005 * pow(salinidad, 4) - 0.000002 * pow(salinidad, 5);
  Serial.print("Salinidad de cálculo = ");
  Serial.println(salinidadReal, 2);

  delay(1000); // Puedes ajustar el tiempo de espera entre lecturas
}

void measureLightLevel() {
  // Leer del canal 0 del ADS1115
  int16_t lectura = ads.readADC_SingleEnded(0);
  float voltaje = lectura * 0.002; // Convertir la lectura en milivoltios

  // Imprimir valores de depuración
  Serial.print("ADC0: ");
  Serial.print(lectura);
  Serial.print(" - Voltaje: ");
  Serial.print(voltaje);
  Serial.println(" mV");

  // Comparar la lectura con los umbrales
  if (lectura < umbralOscuridad) {
    Serial.println("Nivel de Luz: Oscuridad");
  } else if (lectura < umbralSombra) {
    Serial.println("Nivel de Luz: Sombra");
  } else if (lectura < umbralLuzAmbiente) {
    Serial.println("Nivel de Luz: Luz ambiente");
  } else if (lectura < umbralLinterna) {
    Serial.println("Nivel de Luz: Nivel alto de iluminación");
  } else {
    Serial.println("Nivel de Luz: Muy iluminado");
  }
}
