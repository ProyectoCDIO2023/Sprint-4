#include <Wire.h>
#include <Adafruit_ADS1X15.h>

class SensorSystem {
private:
  Adafruit_ADS1115 ads;
  float Offset;
  int samplingInterval;
  int printInterval;
  int ArrayLength;
  int power_pin;

  float pHArray[50];
  int pHArrayIndex;
  
  const float umbralOscuridad;
  const float umbralSombra;
  const float umbralLuzAmbiente;
  const float umbralLinterna;

public:
  SensorSystem(float offset, int sampleInterval, int printIntvl, int arrayLen, int powerPin,
               float darkThreshold, float shadowThreshold, float ambientThreshold, float flashlightThreshold)
    : 
      Offset(offset), samplingInterval(sampleInterval), printInterval(printIntvl), ArrayLength(arrayLen), power_pin(powerPin),
      umbralOscuridad(darkThreshold), umbralSombra(shadowThreshold),
      umbralLuzAmbiente(ambientThreshold), umbralLinterna(flashlightThreshold) {
    pHArrayIndex = 0;
  }

  void initialize() {
    Serial.begin(9600);
    Serial.println("Inicializando...");
    ads.begin();
    ads.setGain(GAIN_ONE);
  }

  float calculatepH(int adcValue) {
    float voltage = adcValue * 0.0001875;
    float pHValue = 3.5 * voltage + Offset;
    return pHValue;
  }

  void measureSalinity() {
    digitalWrite(power_pin, HIGH);
    delay(100);
    int16_t adc0 = analogRead(A0);
    digitalWrite(power_pin, LOW);
    delay(100);
    float salinidad = map(adc0, 0, 1023, 0, 100);
    Serial.print("Lectura digital = ");
    Serial.println(adc0, DEC);
    Serial.print("Salinidad de lectura = ");
    Serial.println(salinidad, 2);
    float salinidadReal = 480 + 15.55 * salinidad - 0.31 * pow(salinidad, 2) - 0.012 * pow(salinidad, 3) + 0.0005 * pow(salinidad, 4) - 0.000002 * pow(salinidad, 5);
    Serial.print("Salinidad de cálculo = ");
    Serial.println(salinidadReal, 2);
    delay(1000);
  }

  void measureLightLevel() {
    int16_t lectura = ads.readADC_SingleEnded(0);
    float voltaje = lectura * 0.002;
    Serial.print("ADC0: ");
    Serial.print(lectura);
    Serial.print(" - Voltaje: ");
    Serial.print(voltaje);
    Serial.println(" mV");
    if (lectura < umbralOscuridad) {
      Serial.println("Oscuridad");
    } else if (lectura < umbralSombra) {
      Serial.println("Sombra");
    } else if (lectura < umbralLuzAmbiente) {
      Serial.println("Luz ambiente");
    } else if (lectura < umbralLinterna) {
      Serial.println("Nivel alto de iluminación");
    } else {
      Serial.println("Muy iluminado");
    }
    delay(1000);
  }

  void loop() {
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float voltage, pHValue;
    int humidityValueMojado = readHumedadMojado();
    int temperature = readTemperatura();

    if (millis() - samplingTime > samplingInterval) {
      int16_t adcValue;
      adcValue = ads.readADC_SingleEnded(3);
      pHValue = calculatepH(adcValue);
      voltage = adcValue * 0.0001875;
      pHArray[pHArrayIndex++] = pHValue;
      if (pHArrayIndex == ArrayLength) pHArrayIndex = 0;
      samplingTime = millis();
    }
    measureLightLevel();
    
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

    // Verifica si ha pasado el tiempo de suspensión (15 minutos)
    if (millis() > (samplingTime + (900000))) {
      deepSleep(900);
    }

    delay(1000);
  }

  int readHumedadMojado() {
    int medidaMojado = ads.readADC_SingleEnded(2);
    int humidityValueMojado = map(medidaMojado, 17000, 31000, 100, 0);
    return humidityValueMojado;
  }

  int readTemperatura() {
    int16_t adcValue = ads.readADC_SingleEnded(1);
    float voltage = (adcValue * 4.096) / 32767;
    int temperature = ((voltage - 0.79) / 0.035);
    return temperature;
  }

  void deepSleep(int sleepTimeS) {
    Serial.println("ESP8266 in sleep mode");
    ESP.deepSleep(sleepTimeS * 1000000);
  }
};

SensorSystem sensor(0.60, 20, 800, 50, 5, 60, 80, 117, 700);

void setup() {
  sensor.initialize();
}

void loop() {
  sensor.loop();
}
