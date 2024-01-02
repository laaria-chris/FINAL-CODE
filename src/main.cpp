#include <Arduino.h>
/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL2zgKrEgch"
#define BLYNK_TEMPLATE_NAME "SENSOR PROTOTYPE"
#define BLYNK_AUTH_TOKEN "xwAofwSbst98Ovq5HCojGNiDuAENgBpO"

//Include declarations
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266_Lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"

//DHT11 pin definiton
/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

char ssid[] = "JKM10";
char pass[] = "rainbow2015";

// Hardware Serial on Mega, Leonardo, Micro...
#define EspSerial Serial1
#define ESP8266_BAUD 38400

//PWM pins for PID Control
#define PWM1 2   // Define the constants used in the program
#define PWM2 3   // Define the constants used in the program

const int LED_PIN = 8; //Define the RED LED PIN

ESP8266 wifi(&EspSerial);
LiquidCrystal_I2C lcd(0x27, 20, 4);

//Setting up a Timer.
BlynkTimer timer;

//Pin declarations
const int gasPin = A0;//Gas analog pin
const int moisturePin1= A1;//Moisture sensor pin out.
const int pH_SENSOR_PIN= A2;//Moisture sensor pin out.
const int DHTPIN = A3;//DHT pin

const int moisturePin2= A4;//Moisture sensor pin out.
const int currentSensorPin = A15;  // current sensor pin
const int voltageSensorPin = A14;  // voltage sensor pin

const int ONE_WIRE_BUS= 9;//Dallas temperature digital line
float calibrationValue = 6 - 8; 

//LED Pins declarations
const int redLed = 24;//LED digital pin
const int greenLed = 26;// LED digital pin

//Define relay pins
const int Relay_Verm= 46;
const int Relay_Fan= 52;
const int Relay_Moisture= 40;
const int RELAY_Lights=38;
const int RELAY_PH_DOWN= 44;
const int RELAY_PH_UP=42;
//Ultrasonic pins
const int trigPin = 10;
const int echoPin = 11;

//Relay Pins for Temp Control(Fans)
const int Relay_HEATER= 50;
const int Relay_COOLER= 48;

//THRESHOLDS
 float MoistureThreshold =30.0;//Moisture threshold to switch on Pump
 int GasThreshold= 50; // Your threshold value (50% gas level).
 float vermiThreshold= 30;
//PH thresholds
 float pH_THRESHOLD_HIGH = 16.0;
 float pH_THRESHOLD_LOW = -6.0;

#define DHTTYPE DHT11  // Initialize DHT sensor with defined pin and type
DHT dht(DHTPIN, DHTTYPE);

//GLOBAL Variable declaration
float pHValue;
float gasLevel;
float averageMoisture;
float vermiliquidPercentage;

//DHT global declarations
float humi;
float tempC;

//POWER VARIABLE DECLARATIONS
float multiplier = 0.100;  // Sensitivity in Volts/Ampers for the 5A model
float offset = 2.5;        // Offset for sensor output centered around 2.5V
float vIn;                 // measured voltage (3.3V = max. 16.5V, 5V = max 25V)
float vOut;
float voltageSensorVal;    // value on pin A14 (0 - 1023)
const float factor = 5.9;  // reduction factor of the Voltage Sensor shield
const float vCC = 5.00;    // Arduino input voltage (measurable by voltmeter)
float current; // to store current value
float mappedVoltage;
float power;

// defines variables for Ultrasonic Sensor
long duration;
int maxDepth = 20; // Maximum depth of the bucket in centimeters
int waterLevelPercentage; // Variable to store the calculated water level percentage

// Create a OneWire instance and DallasTemperature instance for communication with the DS18B20 temperature sensor(s)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//RED LED declarations.
#define LED_COUNT 90
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//PID PELTIER DECLARATIONS(Heater)
byte man_byte = 0;  // Final value of the step change (max = 255)
float Kc = 500;       // Proportional constant of the PID (a larger Kc makes the system faster)
float Taui = 10;     // Integral constant of the PID (setpoint for temperature)
float Taud = 0;      // Derivative constant of the PID
float T = 0.5;       // Sampling period of the process variable
float Mk = 0;        // Current PID manipulation (power)
float Mk1 = 0;       // PID manipulation from the previous sampling time
float E = 0;         // Current error
float E1 = 0;        // Previous error
float E2 = 0;        // Error two time steps back from the current sampling time
float Temp1 = 0;      // Process variable (temperature)

//---------------- REFERENCE TEMPERATURES -------------------
float Ref_HEATER =30;  // Reference temperature for Heater (°C)

//---------------- Digital PID Constants -------------------
float BC1 = 0;
float BC2 = 0;
float BC3 = 0;
//----------------

//PID PELTIER DECLARATIONS(Cooler)
byte man_bytec = 0;  // Final value of the step change (max = 255)
float Kcc = 500;       // Proportional constant of the PID (a larger Kc makes the system faster)
float Tauic = 10;     // Integral constant of the PID (setpoint for temperature)
float Taudc = 0;      // Derivative constant of the PID
float Tc = 0.5;       // Sampling period of the process variable
float Mkc = 0;        // Current PID manipulation (power)
float Mk1c = 0;       // PID manipulation from the previous sampling time
float Ec = 0;         // Current error
float E1c = 0;        // Previous error
float E2c = 0;        // Error two time steps back from the current sampling time
float Temp2 = 0;      // Process variable (temperature)

//---------------- REFERENCE TEMPERATURES(cooler) -------------------
float Ref_COOLER =25;  // Reference temperature for Cooler (°C)
//---------------- Digital PID Constants(cooler) -------------------
float BC1c = 0;
float BC2c = 0;
float BC3c = 0;

// Add a variable to keep track of the active mode (1 for heating, 2 for cooling,3 for disabling cooling and Heating)
int activeMode = 2;
int fanMode = 0;
int LedMode = 0;
int vermiMode = 0;
int lcdPageNumber = 0;
int brightness = 200; // Set a fixed brightness value (adjust as needed)

//Millis function declaration
unsigned long previousMoistureTime = 0;
unsigned long previousPhTime = 0;
unsigned long previousSensorTime = 0;
const unsigned long SensorInterval = 1700;        // Interval for checking pH (in milliseconds)
const unsigned long moistureInterval = 2700;  // Interval for checking moisture (in milliseconds)
const unsigned long pHInterval = 3500;        // Interval for checking pH (in milliseconds)


float readMoisture(int pin) {
int moistureValue = analogRead(pin);
float moisturePercentage = map(moistureValue, 0, 1023, 100, 0);
return moisturePercentage;
}

// BLYNK_CONNECTED()
// {
//   // Synchronizes values on connection
//   //Blynk.syncVirtual(V10);
//   Blynk.syncVirtual(V12);
// }
void Printpage1(){

  lcd.clear();
  //Heater
  lcd.setCursor(0, 0);
  lcd.print("HEATER:");
  lcd.print(Temp1,0);
  lcd.print("C");

  //SET HEATER
  lcd.setCursor(12, 0);
  lcd.print("SET:");
  lcd.print(Ref_HEATER,0);
  lcd.print("C");

  //COOLER
  lcd.setCursor(0, 1);
  lcd.print("COOLER:");
  lcd.print(Temp2,0);
  lcd.print("C");

  //SET COOLER
  lcd.setCursor(12, 1);
  lcd.print("SET:");
  lcd.print(Ref_COOLER,0);
  lcd.print("C");

  //PH
  lcd.setCursor(0, 2);
  lcd.print("PH:");
  lcd.print(pHValue,0);

  //Gas
  lcd.setCursor(0, 3);
  lcd.print("Gas:");
  lcd.print(gasLevel,0);
  lcd.print("%");

}
void Printpage2() {
  lcd.clear();

  // Moisture
  lcd.setCursor(0, 2);
  lcd.print("MOISTURE:");
  lcd.setCursor(2, 3);
  lcd.print(averageMoisture,0);
  lcd.print("%");

  // DHT11 HUMIDITY
  lcd.setCursor(12, 2);
  lcd.print("HUMIDITY:");
  lcd.setCursor(14, 3);
  lcd.print(humi,0);
  lcd.print("%");

  // Vermiliquid
  lcd.setCursor(0, 0);
  lcd.print("VERMI:");
  lcd.setCursor(3, 1);
  lcd.print(vermiliquidPercentage, 0);
  lcd.print("%");

  // Temp ENV
  lcd.setCursor(12, 0);
  lcd.print("E TEMP");
  lcd.setCursor(14, 1);
  lcd.print(tempC, 0);
  lcd.print("C");
}

void Printpage3(){
  lcd.clear();
  lcd.setCursor(10, 2);
  lcd.print("Voltage:");
  lcd.setCursor(12, 3);
  lcd.print(vIn, 1);
  lcd.setCursor(16, 3);
  lcd.print("V");

  lcd.setCursor(0, 2);
  lcd.print("Current:");
  lcd.setCursor(2, 3);
  lcd.print(current, 1);
  lcd.setCursor(6, 3);
  lcd.print("A");

  lcd.setCursor(0, 0);
  lcd.print("Battery:");
  lcd.setCursor(1, 1);
  lcd.print(mappedVoltage, 1);
  lcd.setCursor(7, 1);
  lcd.print("%");

  lcd.setCursor(10, 0);
  lcd.print("Wattage:");
  lcd.setCursor(11, 1);
  lcd.print(power, 1);
  lcd.setCursor(16, 1);
  lcd.print("W");

}
void UpdateLCD() {
  Serial.println("Updating LCD");
  if (lcdPageNumber == 0) {
    Printpage1();
    lcdPageNumber = 1;
  } else if (lcdPageNumber == 1) {
    Printpage2();
    lcdPageNumber = 2;
  } else if (lcdPageNumber == 2) {
    Printpage3();
    lcdPageNumber = 0;
  }
}

void VirtualWrite(){
  Serial.println("Sending data to Blynk");
  //Send Heater level data to Blynk
  Blynk.virtualWrite(V10, Temp1);
  Blynk.virtualWrite(V15, Ref_HEATER);
    //Send Cooler level data to Blynk
  Blynk.virtualWrite(V14, Temp2);
  Blynk.virtualWrite(V16, Ref_COOLER);
    //Send PH level data to Blynk
  Blynk.virtualWrite(V9, pHValue);
  // Send moisture level data to Blynk
  Blynk.virtualWrite(V0, averageMoisture);
  // Send vermiliquid level data to Blynk
  Blynk.virtualWrite(V13, vermiliquidPercentage);
  //Send power levels data to blynk
  Blynk.virtualWrite(V29, vIn);
  Blynk.virtualWrite(V30, mappedVoltage);
  Blynk.virtualWrite(V31, current);
  Blynk.virtualWrite(V32, power);
  // Send gas level data to Blynk
  Blynk.virtualWrite(V4, gasLevel);
  // Send moisture level data to Blynk
  Blynk.virtualWrite(V3, humi);
  Blynk.virtualWrite(V5, tempC);
  //Send brightness value to blynk
  Blynk.virtualWrite(V28, brightness);

  //Sending PID constants to blynk
  Blynk.virtualWrite(V37, Kc);
  Blynk.virtualWrite(V38, Taui);
  Blynk.virtualWrite(V39, Kcc);
  Blynk.virtualWrite(V40, Tauic);
  
  
}
void controlHeater(){
  sensors.requestTemperatures();
  Temp1 = sensors.getTempCByIndex(1); // Sensor 1
  //HEATER
  Serial.print("HEATER:");
  Serial.print(Temp1,0);
  Serial.print("°C");
  //SET HEATER
  Serial.print("SET HEATER= ");
  Serial.println(Ref_HEATER,0);

  E = Temp1-Ref_HEATER;
  Mk = Mk1 + BC1 * E + BC2 * E1 + BC3 * E2;  // This is the PID equation that calculates the current manipulation (Mk)
  // Here, we limit the maximum and minimum values of the manipulation to 0-255
  if (Mk > 255) {
    Mk = 255;
  }
  if (Mk < 0) {
    Mk = 0;
  }
  man_byte = Mk;
  analogWrite(PWM1, man_byte);  // Send the current manipulation to the PWM generator pin (pin 13 on the Arduino)
  // Update the current manipulation, current error, and error two time steps back from the current sampling time
  Mk1 = Mk;
  E2 = E1;
  E1 = E;
}
//Change the value of reference Heater
BLYNK_WRITE(V15)
{
  Ref_HEATER = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("Ref_HEATER is now: ");
  Serial.println(Ref_HEATER);
}
void controlCooler(){
  sensors.requestTemperatures();
  Temp2 = sensors.getTempCByIndex(0); // Sensor 1
   
  Ec = Ref_COOLER-Temp2;
  Mkc = Mk1c + BC1c * Ec + BC2c * E1c + BC3c * E2c;  // This is the PID equation that calculates the current manipulation (Mk)
  // Here, we limit the maximum and minimum values of the manipulation to 0-255
  if (Mkc > 255) {
    Mkc = 255;
  }
  if (Mkc < 0) {
    Mkc = 0;
  }
  man_bytec = Mkc;
  analogWrite(PWM2, man_bytec);  // Send the current manipulation to the PWM generator pin (pin 13 on the Arduino)
  // Update the current manipulation, current error, and error two time steps back from the current sampling time
  Mk1c = Mkc;
  E2c = E1c;
  E1c = Ec;
    //COOLER
  Serial.print("COOLER:");
  Serial.print(Temp2,0);
  Serial.print("°C");
  
  Serial.print("SET COOLER= ");
  Serial.println(Ref_COOLER,0);
  
}
//Change the value of reference Cooler.
BLYNK_WRITE(V16)
{
  Ref_COOLER = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("Ref_COOLER  is now: ");
  Serial.println(Ref_COOLER);
}

//Function to switch modes between cooler and Heater
void SyncTemp() {
  if (activeMode == 1) {
    controlHeater();
  

    // Activate Heater
    digitalWrite(Relay_HEATER, LOW);
    digitalWrite(Relay_COOLER, HIGH);

    if (lcdPageNumber == 1){
    lcd.setCursor(0, 1);
    lcd.print("COOLER:OFF");
    }
    // Disable cooling
    analogWrite(PWM2, 255);
  } 
  else if (activeMode == 2) {
    // Start cooling
    controlCooler();


    digitalWrite(Relay_HEATER, HIGH);
    digitalWrite(Relay_COOLER, LOW);

   // Disable heating
    analogWrite(PWM1, 255);

    if (lcdPageNumber == 1){
    lcd.setCursor(0, 0);
    lcd.print("HEATER:OFF");
   }
 


  } else if (activeMode == 3) {

    // Switch both heater and cooler fans off
    digitalWrite(Relay_HEATER, HIGH);
    digitalWrite(Relay_COOLER, HIGH);


    if(lcdPageNumber == 1){
    lcd.setCursor(0, 0);
    lcd.print("HEATER:OFF");
    lcd.setCursor(0, 1);
    lcd.print("COOLER:OFF");
    }


    // Disable heating and cooling using PWM.
    analogWrite(PWM1, 255);
    analogWrite(PWM2, 255);
  } else {
    // Default case if none of the conditions are met
  }

}

BLYNK_WRITE(V17)
{
  activeMode = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("Teemp MODE IS NOW: ");
  Serial.println(activeMode);
}

//FINE TUNE THE HEATER
BLYNK_WRITE(V33)
{
  Kc = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("Kc IS NOW: ");
  Serial.println(Kc);
}
BLYNK_WRITE(V34)
{
  Taui = param.asFloat(); // 
  // You can also use:
  Serial.print("Tauic IS NOW: ");
  Serial.println(Taui);
}
//FINE TUNE THE COOLER
BLYNK_WRITE(V35)
{
  Kcc = param.asFloat(); // 
  // You can also use:
  Serial.print("kcc IS NOW: ");
  Serial.println(Kcc);
}
BLYNK_WRITE(V36)
{
  Tauic = param.asFloat(); 
  // You can also use:
  Serial.print("Tauic IS NOW: ");
  Serial.println(Tauic);
}
void readPHSensor() {
  int bufferArr[10];
  unsigned long avgValue = 0;
  // Read the pH sensor values multiple times and calculate an average
  for (int i = 0; i < 10; i++) {
    bufferArr[i] = analogRead(pH_SENSOR_PIN); // Read analog pH sensor value
    delay(30); // Delay to stabilize sensor reading
  }
  // Sort the readings in ascending order (to eliminate outliers)
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (bufferArr[i] > bufferArr[j]) {
        int temp = bufferArr[i];
        bufferArr[i] = bufferArr[j];
        bufferArr[j] = temp; }
    }
  }
  // Calculate an average value from the middle readings
  for (int i = 2; i < 8; i++) {
    avgValue += bufferArr[i];
  }
  // Convert the average reading to pH value
  float volt = (float)avgValue * 5.0 / 1024 / 6;
  pHValue = -5.70 * volt + calibrationValue;
  //PH
  Serial.print("PH: ");
  Serial.println(pHValue);
  delay(100);

}
void checkPH() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousPhTime >= pHInterval) {
    previousPhTime = currentMillis;

    Serial.println("Checking PH");

    if (pHValue < pH_THRESHOLD_LOW) {
      // Activate RELAY_PH_DOWN
      digitalWrite(RELAY_PH_UP, LOW);
      digitalWrite(RELAY_PH_DOWN, HIGH);
    } else if (pHValue > pH_THRESHOLD_HIGH) {
      // Activate relay 2
      digitalWrite(RELAY_PH_DOWN, LOW);
      digitalWrite(RELAY_PH_UP, HIGH);
    } else {
      // pH is within the desired range, turn off both relays
      digitalWrite(RELAY_PH_UP, HIGH);
      digitalWrite(RELAY_PH_DOWN, HIGH);
    }
  }
}



//Button to Lower PH
BLYNK_WRITE(V7) {
  bool value1 = param.asInt();
  // Check these values and turn the relay1 ON and OFF
  if (value1 == 1) {
    digitalWrite(RELAY_PH_DOWN, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LOWERING PH");
    Serial.println("LOWERING PH");
  } 
  if (value1 == 0) {
    digitalWrite(RELAY_PH_DOWN, HIGH);
  }
}

//Button to Raise PH
BLYNK_WRITE(V8) {
  bool value2 = param.asInt();
  // Check these values and turn the relay2 ON and OFF
  if (value2 == 1) {
    digitalWrite(RELAY_PH_UP, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("INCREASING PH");
    Serial.println("INCREASING PH");
    delay(100);
  } 
  if (value2== 0) {
    digitalWrite(RELAY_PH_UP, HIGH);
  }
}
//Alter PH UP THRESHOLD
BLYNK_WRITE(V21)
{
  pH_THRESHOLD_HIGH = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("PH UP SET TO: ");
  Serial.println(pH_THRESHOLD_HIGH);
}
//Alter PH DOWN THRESHOLD
BLYNK_WRITE(V22)
{
  pH_THRESHOLD_LOW = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("PH LOW SET TO: ");
  Serial.println(pH_THRESHOLD_LOW);
}

// //Button to Switch on fans()
// BLYNK_WRITE(V18) {
//   int pin4 = param.asInt();
//   // Check these values and turn the relay1 ON and OFF
//   if (pin4 == 1) {
//     digitalWrite(Relay_Fan, LOW);
//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print("FANS ON");
//     Serial.println("FANS ON");
//     delay(100);
//     }
//   if (pin4 == 0){
//         digitalWrite(Relay_Fan, HIGH);
//   }
// };

BLYNK_WRITE(V23)
{
  GasThreshold= param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("GAS THRESHOLD SET TO: ");
  Serial.println(GasThreshold);
}

void checkMoisture() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMoistureTime >= moistureInterval) {
    previousMoistureTime = currentMillis;

    Serial.println("Checking Moisture level");

    // Activate Moisture Pump
    if (averageMoisture< MoistureThreshold) {
      digitalWrite(Relay_Moisture, LOW); // Turn on pump 1
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("MOISTURE PUMP ON");
        delay(100);
    } else {
      digitalWrite(Relay_Moisture, HIGH); // Pump 1 remains off
    }
  }
}



//BUTTON TO ACTIVATE WATER PUMP
BLYNK_WRITE(V11) {
  bool pin3 = param.asInt();
  // Check these values and turn the relay2 ON and OFF
  if (pin3 == 1) {
    digitalWrite(Relay_Moisture, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moisture Pump ON");
    Serial.println("Moisture Pump ON");
    delay(100);

  } 
  if (pin3 == 0) {
    digitalWrite(Relay_Moisture, HIGH);
  }
}
//CHANGE MOISTURE THRESHOLD
BLYNK_WRITE(V24)
{
  MoistureThreshold = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("Moisture Threshold SET TO: ");
  Serial.println(MoistureThreshold);
}
void readSensors(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousSensorTime >= SensorInterval) {
    previousSensorTime = currentMillis;


  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
 
  int sensorOffset = 5; // Offset to compensate for sensor's position above water
  int waterLevel = duration * 0.034 / 2 - sensorOffset;
  vermiliquidPercentage = map(waterLevel, 0, maxDepth, 100, 0); // Inverted mapping
  
  Serial.print("VERMI LEVEL: ");
  Serial.print(vermiliquidPercentage);
  Serial.println("%");
  //Code to read Moisture level
  float moisture1 = readMoisture(moisturePin1); // Read moisture from first sensor
  float moisture2 = readMoisture(moisturePin2); // Read moisture from second sensor
  averageMoisture = (moisture1 + moisture2) / 2.0; // Calculate average
  
  //Print Moisture level to serial Monitor
  Serial.print("Average Moisture: ");
  Serial.print(averageMoisture);
  Serial.println("%");


  int gasValue = analogRead(gasPin);
  gasLevel = map(gasValue, 0, 1023, 0, 100);

  //Serial print the Gas level
  Serial.print("Gas:");
  Serial.print(gasLevel);
  Serial.println("%");


  humi  = dht.readHumidity();    // Read humidity
  tempC = dht.readTemperature(); // Read temperature

  // // Check if any reads failed
  if (isnan(humi) || isnan(tempC)) {
  Serial.println("Failed to read data.");
  } else {
  // Print temperature and humidity to Serial Monitor
  Serial.print("Hum:");
  Serial.print(humi);
  Serial.println("%");

  Serial.print("TEMP:");
  Serial.print(tempC);
  Serial.println("C");
}

}
}
// BLYNK_WRITE(V12) {
//   bool pin1 = param.asInt();
//   // Check the value and turn the Relay_Verm ON and OFF
//   if (pin1 == 1) {
//     digitalWrite(Relay_Verm, LOW);
//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print("Vermi Pump ON");
//     Serial.println("Vermi Pump ON");
//     delay(100);
//   } 
//   if (pin1 == 0) {
//     digitalWrite(Relay_Verm, HIGH);
//   }
// }
void checkVermiliquid(){
  if (vermiliquidPercentage < vermiThreshold){
    digitalWrite(Relay_Verm, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EXTRACTING VERMILIQUID");
    Serial.println("Extracting Liquid");
   } 
   else if (vermiliquidPercentage > vermiThreshold){
    digitalWrite(Relay_Verm, HIGH);
   }
  else{
    // Default case if none of the conditions are met
  }
}
 void syncVermi(){
 if ( vermiMode == 0) {
   digitalWrite(Relay_Verm, HIGH);
   } 
   else if (vermiMode == 1){
    digitalWrite(Relay_Verm, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EXTRACTING VERMILIQUID");
    Serial.println("Extracting Liquid");
   }
    else if (vermiMode == 2) {
    checkVermiliquid();
  }
  else{
    // Default case if none of the conditions are met
  }
  }

BLYNK_WRITE(V27)
{
  vermiMode = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("vermiMode is NoW:");
  Serial.println(vermiMode);
}
BLYNK_WRITE(V41)
{
 vermiThreshold= param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("Vermi Threshold SET TO: ");
  Serial.println(vermiThreshold);
}
void checkGas() {
if (gasLevel > GasThreshold) {
  Serial.println("Checking gas level");
  Blynk.logEvent("air_purity");
  digitalWrite(Relay_Fan, LOW);  // Turn on fan when gas level exceeds threshold
  delay(100);
  Serial.println("FANS ON");
} else {
  digitalWrite(Relay_Fan, HIGH); // Turn off fan when gas level is below threshold
}
}
 void syncFans(){
   if (fanMode == 0) {
   checkGas();
   } 
   else if (fanMode == 1) {
    digitalWrite(Relay_Fan, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FANS ON");
    delay(100);
    Serial.println("FANS ON");
  } else {
    // Default case if none of the conditions are met
  }

 }
 BLYNK_WRITE(V25)
{
  fanMode = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("fanMode is NoW:");
  Serial.println(fanMode);
}
  void readPower(){
  // Read current sensor
  float sensorRead = analogRead(currentSensorPin) * (5.0 / 1023.0); // Read the current sensor output
  current = abs((sensorRead - offset) / multiplier); // Calculate the absolute current value
  // Read voltage sensor
  voltageSensorVal = analogRead(voltageSensorPin);// read the voltage sensor value (0 - 1023) 
  vOut = (voltageSensorVal / 1023.0) * vCC; // convert the value to the real voltage on the analog pin
  vIn = vOut * factor;// convert the voltage on the source by multiplying with the factor
  // Calculate power in watts (using absolute values for current and voltage)
  power = abs(vIn * current);
  // Map voltage to a percentage between 10.8V to 13.5V
  mappedVoltage = map(vIn * 100, 1080, 1350, 0, 100); // Assuming 10.8V to 13.5V mapped to 0% to 100%

  Serial.print("Voltage = ");             
  Serial.print(vIn, 1);
  Serial.println("V");

  Serial.print("Battery Percentage = ");
  Serial.print(mappedVoltage);
  Serial.println("%");

  Serial.print("Current: ");
  Serial.print(current, 3);
  Serial.println("A");

  Serial.print("Power: ");
  Serial.print(power, 3);
  Serial.println("W");
  }

void RedLED(){
  digitalWrite(RELAY_Lights, LOW);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("LIGHTS ON");
  delay(100);
  strip.clear();
  strip.setBrightness(brightness);
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, 255, 0, 0);
  }
  strip.show();
}
void syncLED(){
 if (LedMode == 0) {
 RedLED();
   } 
 else if (LedMode == 1) {
 digitalWrite(RELAY_Lights, HIGH);
    
  } else {
    // Default case if none of the conditions are met
  }
}
BLYNK_WRITE(V26)
{
  LedMode = param.asFloat(); // assigning incoming value from pin V10 to a variable, then convert to MS
  // You can also use:
  Serial.print("ledMode is NoW:");
  Serial.println(LedMode);
}
BLYNK_WRITE(V19)
{
  brightness = param.asFloat(); // assigning incoming value from pin V19 to a variable, then convert to MS
  // You can also use:
  Serial.print("Brightness set to: ");
  Serial.println(brightness);
}

BLYNK_WRITE(V20) {
  int pin6 = param.asInt();
  // Check the value and turn the RELAY_Lights ON and OFF
  if (pin6 == 1) {
    digitalWrite(RELAY_Lights, LOW);
  } 
  else if (pin6 == 0) {
    digitalWrite(RELAY_Lights, HIGH);
    lcd.clear();
  }
}

void setup(){  
  Serial.begin(9600); // Start serial communication with Arduino

  //SET PINMODE
  // pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(Relay_Moisture, OUTPUT);
  pinMode(RELAY_PH_UP, OUTPUT);
  pinMode(RELAY_PH_DOWN, OUTPUT);
  pinMode(Relay_Verm, OUTPUT);
  pinMode(Relay_HEATER, OUTPUT); 
  pinMode(Relay_COOLER, OUTPUT); 
  pinMode(ONE_WIRE_BUS, INPUT); 
  pinMode(Relay_Fan, OUTPUT);
  pinMode(RELAY_Lights, OUTPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  
  
  //SWITCH OFF RELAYS AT THE BEGINNING
  digitalWrite(Relay_Moisture, HIGH); 
  digitalWrite(RELAY_PH_UP, HIGH);  
  digitalWrite(RELAY_PH_DOWN, HIGH);  
  digitalWrite(Relay_Verm, HIGH); 
  digitalWrite(Relay_Fan, HIGH);
  digitalWrite(RELAY_Lights, HIGH);
  digitalWrite(Relay_HEATER, HIGH); 
  digitalWrite(Relay_COOLER, HIGH); 
  digitalWrite(trigPin, HIGH); 
  digitalWrite(echoPin, HIGH);

  strip.begin();
 
  
  // Initialize the LCD
  lcd.begin(); 
  lcd.backlight();

  
  lcd.setCursor(0, 0);
  lcd.print("AUTOMATED");
  lcd.setCursor(0, 1);
  lcd.print("VERMICOMPOSTING");
  lcd.setCursor(0, 2);
  lcd.print("PLANT SYSTEM");
  lcd.clear();


  EspSerial.begin(ESP8266_BAUD); // Initialize ESP8266 serial communication
  delay(1000); // Wait for ESP8266 to initialize
  
  
  
  dht.begin();// Initialize the DHT11 sensor 
  sensors.begin();// Initialize the DSB120 sensor
 

  Serial.println("Trying to connect to blynk............");
 
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass, "blynk.cloud", 80);

 
  //Setup a function to be called every second
  timer.setInterval(100L,  SyncTemp);
  timer.setInterval(150L,  syncVermi);
  
  timer.setInterval(200L, syncFans);
  timer.setInterval(500L,  syncLED);
  timer.setInterval(1000L, UpdateLCD);
  timer.setInterval(1800L, readPower);
  timer.setInterval(2000L, readPHSensor);
  timer.setInterval(3000L, VirtualWrite);
  
  //Heater
  BC1 = Kc * (1 + (T / Taui) + (Taud / T));
  BC2 = Kc * (-1 - (2 * Taud / T));
  BC3 = Kc * Taud / T;
  //Cooler
  BC1c = Kcc * (1 + (Tc / Tauic) + (Taudc / Tc));
  BC2c = Kcc * (-1 - (2 * Taudc / Tc));
  BC3c = Kcc * Taudc / Tc;

 Serial.println("CONNECTED SUCCESSFULLY");

}

void loop()
{ 
 Blynk.run();
 timer.run(); // call the BlynkTimer object
  // Call the functions at appropriate intervals
  checkMoisture();
  checkPH();
  readSensors();
}

