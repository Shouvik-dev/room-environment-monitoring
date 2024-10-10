#include <DHT.h>
#include <Wire.h>


#define DHTPIN 25
#define PIR_PIN 26
#define buzzer 14
#define mLED 15
#define LightPin 35

int motionState = LOW;
const int mq135Pin = 32; 
const int MIC_PIN = 33; 

const int sampleWindow = 50;                              // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

int sensorValue;
DHT dht(DHTPIN, DHT11);

void setup() {
  // put your setup code here, to run once:

  pinMode(PIR_PIN, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(mLED, OUTPUT);
  pinMode(LightPin, INPUT);
  pinMode(MIC_PIN, INPUT);
  Serial.begin(115200);
  dht.begin();

   playClickTone(1000, 200);
   Serial.println("ROOM ENVIRONMENT MONITORING SYSTEM");
}

void loop() {

  temp_humi_sensor();
  GasSensor();
  Sound_intensity();
  motion_detection();
  lightIntenssity();
  Serial.println("--------------------------------------------------------");
  
}


void GasSensor()
{
  sensorValue = analogRead(mq135Pin);
  Serial.print("MQ135 Value: ");
  Serial.println(sensorValue);
}

void Sound_intensity()
{
   unsigned long startMillis= millis();                   // Start of sample window
   float peakToPeak = 0;                                  // peak-to-peak level
 
   unsigned int signalMax = 0;                            //minimum value
   unsigned int signalMin = 1024;                         //maximum value
 
                                                          // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(MIC_PIN);                    //get reading from microphone
      if (sample < 1024)                                  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;                           // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;                           // save just the min levels
         }
      }
   }
 
   peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
   int db = map(peakToPeak,20,900,49.5,90);  
  
    Serial.print("Sound Intencity: ");
    Serial.print(db);
    Serial.println(" db");
}

void temp_humi_sensor()
{
  delay(1000);
  // Read humidity (percentage)
  float humidity = dht.readHumidity();
  // Read temperature in Celsius
  float temperature = dht.readTemperature();
  // Check if any readings failed and exit early
          if (isnan(humidity) || isnan(temperature)) {
            Serial.println("Failed to read from DHT sensor!");
            return;
          }

  // Print the results to the Serial Monitor
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

}

void motion_detection()
{
  // Read the state of the PIR sensor
  motionState = digitalRead(PIR_PIN);

  // Check if motion is detected
  if (motionState == HIGH) {
    // Print message when motion is detected
    Serial.println("Motion detected!");
    analogWrite(mLED,800);
  } else {
    // Print message when no motion is detected
    Serial.println("No motion.");
    analogWrite(mLED,0);
  }

  // Add a small delay to avoid flooding the Serial Monitor

}

void lightIntenssity()
{
  sensorValue = analogRead(LightPin);
  Serial.print("Light Intendity: ");
  Serial.println(sensorValue);
}

void playClickTone(int frequency, int duration) {
    tone(buzzer, frequency, duration);
    delay(duration);  // Wait for the tone to finish
    noTone(buzzer);   // Stop the tone
}