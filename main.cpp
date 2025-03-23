#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>

// Wi-Fi credentials
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT broker details
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// GPIO Pins
#define TEMP_SENSOR_PIN 33
#define ULTRASONIC_TRIG_PIN 32
#define ULTRASONIC_ECHO_PIN 34
#define PH_SENSOR_PIN 35
#define SERVO_PIN 18
#define RELAY1_PIN 16  
#define RELAY2_PIN 17  
#define LED_PIN 5
#define POTETIOMETER_PIN 36

// Ultrasonic sensor variables
long duration;
float waterLevel;
float lastWaterLevel = 0;
const float minWaterLevel = 20.0;
const float maxWaterLevel = 30.0;
const float waterLevelThreshold = 0.5; // Threshold for publishing changes

// Temperature sensor setup
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
float temperature;
float lastTemperature = 0;
const float temperatureThreshold = 0.2; // Threshold for publishing changes

// pH sensor variables
float pHValue;
float lastPHValue = 0;
const float minPH = 6.8;
const float maxPH = 7.8;
const float pHThreshold = 0.1; // Threshold for publishing changes

// Feeding system variables
unsigned long lastFeedTime = 0;
const unsigned long feedInterval = 6 * 60 * 60 * 1000; // 6 hours

// LED control variables
bool isNight = false;
bool ledState = false;

// Loop timing control
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 1000; // Read sensors every second

// Servo object
Servo servoMotor;

// Function prototypes
void setupWiFi();
void callback(char* topic, byte* payload, unsigned int length);
void readAndPublishSensorData(bool forcePublish = false); // Fixed prototype with default parameter
void publishSensorValue(const char* topic, float value, float &lastValue, float threshold); // Fixed prototype with reference
void managePH();
void manageWaterLevel();
void feedFish(bool overrideTimer = false);
void manageLED();

void setup() {
  Serial.begin(115200);

  // Initialize GPIOs
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Ensure relays are initially off
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  
  // Set ADC attenuation to 11db to handle higher voltages (0-3.3V)
  analogSetAttenuation(ADC_11db);

  // Initialize Servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0); // Initial position

  setupWiFi();

  // Initialize MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32_aquvax")) {
      Serial.println("Connected to MQTT!");
      client.subscribe("/aquvax/commands");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");
      delay(5000);
    }
  }

  // Initialize temperature sensor
  sensors.begin();
  
  // Initial read and publish of all sensor values
  readAndPublishSensorData(true);
}

void loop() {
  client.loop();  // Keep MQTT connection alive

  // Read sensors more frequently to detect changes
  if (millis() - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = millis();

    // Read and publish sensor data if values have changed
    readAndPublishSensorData();

    // Manage pH and water level
    managePH();
    manageWaterLevel();

    // Check feeding time
    feedFish();

    // Manage LED lighting
    manageLED();
  }
}

void setupWiFi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String command;
  for (int i = 0; i < length; i++) {
    command += (char)payload[i];
  }

  Serial.println("MQTT Command Received: " + command);

  if (command == "FEED") {
    // Pass true to override the timer check
    feedFish(true);
    // Publish confirmation
    client.publish("/aquvax/status", "Fish fed manually");
  } else if (command == "LED") {
    // Toggle LED state
    ledState = !ledState; // Flip the state
    digitalWrite(LED_PIN, ledState ? HIGH : LOW); // Set LED to new state
    Serial.println(ledState ? "LED turned ON via MQTT" : "LED turned OFF via MQTT");
    // Publish confirmation
    client.publish("/aquvax/status", ledState ? "LED turned ON" : "LED turned OFF");
  }
}

// The function to map float values
float floatMap(float value, float in_min, float in_max, float out_min, float out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Helper function to determine if a value has changed enough to publish
void publishSensorValue(const char* topic, float value, float &lastValue, float threshold) {
  // Check if value has changed more than the threshold or this is the first reading
  if (abs(value - lastValue) >= threshold || lastValue == 0) {
    client.publish(topic, String(value).c_str());
    Serial.println("Published " + String(topic) + ": " + String(value));
    lastValue = value; // Update the last value
  }
}

void readAndPublishSensorData(bool forcePublish) {
  // Measure water level
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  waterLevel = (duration * 0.034) / 2;

  // Measure temperature
  sensors.requestTemperatures();
  delay(100); 
  temperature = sensors.getTempCByIndex(0);

  // Measure pH - Use the global variable instead of redeclaring
  int analogValue = analogRead(POTETIOMETER_PIN); // Read raw analog value
  pHValue = floatMap(analogValue, 0, 4095, 1.0, 14.0); 

  // Also read from the pH sensor chip
  int readVal = analogRead(PH_SENSOR_PIN);
  float voltage = readVal * (3.3 / 4095.0);
  float sensorPHValue = (3.5 - voltage) / 0.17;

  // Publish sensor data if values have changed significantly or forced
  if (forcePublish) {
    // Force publish all values (first reading or on demand)
  client.publish("/aquvax/sensors/waterLevel", String(waterLevel).c_str());
  Serial.println("Published water level: " + String(waterLevel));

  client.publish("/aquvax/sensors/temperature", String(temperature).c_str());
  Serial.println("Published temperature: " + String(temperature));

  client.publish("/aquvax/sensors/pH", String(pHValue).c_str());
  Serial.println("Published pH value: " + String(pHValue));
    
    // Update last values
    lastWaterLevel = waterLevel;
    lastTemperature = temperature;
    lastPHValue = pHValue;
  } else {
    // Publish only changed values
    publishSensorValue("/aquvax/sensors/waterLevel", waterLevel, lastWaterLevel, waterLevelThreshold);
    publishSensorValue("/aquvax/sensors/temperature", temperature, lastTemperature, temperatureThreshold);
    publishSensorValue("/aquvax/sensors/pH", pHValue, lastPHValue, pHThreshold);
  }
}

void managePH() {
  if (pHValue < minPH || pHValue > maxPH) {
    Serial.println("Adjusting pH...");
    digitalWrite(RELAY1_PIN, HIGH); // Turn on pH control pump
    delay(5000);
    digitalWrite(RELAY1_PIN, LOW); // Turn off pump
    
    // Force a new reading and publish after adjustment
    readAndPublishSensorData(true);
  }
}

void manageWaterLevel() {
  if (waterLevel < minWaterLevel) {
    Serial.println("Filling water...");
    digitalWrite(RELAY2_PIN, HIGH); // Use RELAY2 for water filling
    delay(5000);
    digitalWrite(RELAY2_PIN, LOW);
    
    // Force a new reading and publish after filling
    readAndPublishSensorData(true);
  } else if (waterLevel > maxWaterLevel) {
    Serial.println("Draining water...");
    // Make sure pH control isn't active
    unsigned long drainStartTime = millis();
    // Wait for a moment if pH control was recently active
    while (millis() - drainStartTime < 2000 && digitalRead(RELAY1_PIN) == HIGH) {
      delay(100);
    }
    digitalWrite(RELAY1_PIN, HIGH); // Use RELAY1 for draining
    delay(5000);
    digitalWrite(RELAY1_PIN, LOW);
    
    // Force a new reading and publish after draining
    readAndPublishSensorData(true);
  }
}

void feedFish(bool overrideTimer) {
  // Check if it's time to feed or if manual override is requested
  if (overrideTimer || millis() - lastFeedTime >= feedInterval || lastFeedTime == 0) {
    Serial.println("Feeding fish...");

    // Move servo to 90° position (open feeder)
    servoMotor.write(90);
    delay(3000);
    
    // Move servo back to 0° position (close feeder)
    servoMotor.write(0);
    
    // Update last feed time
    lastFeedTime = millis();
    
    // Log the feeding event
    Serial.println("Fish fed at " + String(millis() / 1000) + " seconds");
    
    // Publish feeding status
    client.publish("/aquvax/status/feeding", "Fish fed");
  }
}

void manageLED() {
  // This is a simple simulation of day/night cycle
  // In a real application, you would use an RTC module for accurate time
  unsigned long timeInSeconds = millis() / 1000;
  unsigned long secondsInDay = 86400; // 24 hours in seconds
  unsigned long secondsInCurrentDay = timeInSeconds % secondsInDay;
  int currentHour = (secondsInCurrentDay / 3600) % 24;
  
  // Set night hours (6 PM to 6 AM)
  bool shouldBeNight = (currentHour >= 18 || currentHour < 6);
  
  // Only toggle if there's a change
  if (shouldBeNight != isNight) {
    isNight = shouldBeNight;
    ledState = isNight;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    Serial.println(ledState ? "LED turned ON for night" : "LED turned OFF for daytime");
    
    // Publish LED status change
    client.publish("/aquvax/status/lighting", ledState ? "Night mode ON" : "Day mode ON");
  }
}