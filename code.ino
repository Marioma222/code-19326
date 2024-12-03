#define WATER_HIEGHT 10
#define Interval 4000   // millisec 

#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "Fello"; // Your Wi-Fi SSID
const char* password = "12345678"; // Your Wi-Fi password

const char* serverName = "http://192.168.137.1:1880/values"; // Replace with your Node-RED IP and port

// pHRead.ino



#define TdsSensorPin 33
#define WATER_PIN 35
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

// Constants:-
const byte pHpin = 32;    // Connect the sensor's Po output to analogue pin 0.
int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 15;       // current temperature for compensation
float water = 0;
// Variables:-
float Po;


// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
  pinMode(pHpin, INPUT);
  pinMode(WATER_PIN, INPUT);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) { //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  static unsigned long sendTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

    // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    //temperature compensation
    float compensationVoltage = averageVoltage / compensationCoefficient;

    //convert voltage value to tds value
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.9;

    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    water = analogRead(WATER_PIN);
    Serial.println("Water Analog : " + String(water));
    water = mapFloat(water, 0.0, 4096.0, 0.0, 400.0)/100.0 + WATER_HIEGHT;
    Serial.print("Water Level:");
    Serial.print(water);
    Serial.println(" cm");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("PPM");
    Po = analogRead(pHpin);  // Read and reverse the analogue input value from the pH sensor.
    Serial.println(Po);
    Po = mapFloat(Po, 0.0, 4096.0, 0.0, 14.0);   // Map the '0 to 1023' result to '0 to 14'.
    Serial.println(Po);             // Print the result in the serial monitor.
  }
  if (millis() - sendTimepoint > Interval)
  {
    sendTimepoint = millis();
    if (WiFi.status() == WL_CONNECTED) {  // Check if Wi-Fi is connected
      HTTPClient http;

      // Begin HTTP POST request
      http.begin(serverName);

      // Set the content type to application/json
      http.addHeader("Content-Type", "application/json");

      // Manually construct the JSON string
      String jsonData = "{\"millis\":"+String(millis())+",\"ph\":"+String(Po)+",\"tds\":"+String(tdsValue)+",\"water\":"+String(water)+"}";  // JSON data as a string

      // Send the POST request with JSON data
      int httpCode = http.POST(jsonData);

      // Check if the POST request was successful
      if (httpCode > 0) {
        Serial.printf("HTTP POST request sent. Response code: %d\n", httpCode);
      } else {
        Serial.printf("Error sending HTTP POST request: %s\n", http.errorToString(httpCode).c_str());
      }

      // End the HTTP request
      http.end();
    } else {
      Serial.println("WiFi not connected");
    }
  }
}
