// New Code for Arduino Pro mini in US100 Robuste sensors

#include <NeoSWSerial.h>
#include <math.h>

// Firmware Parameters
//********************************************************************************************************
#define FirmwareVersion 2.0.0             // Version initiale du code.
#define MaxHeatingPowerPercent 90         // Puissance maximale appliqué sur la résistance de chauffage
#define HeatingSetPoint 25                // Température cible à l'intérieur du boitier
#define ExceedRangeUS100 9999             // Distance maximale valide pour le captgeur
#define numReadings 10                    // Number of readings to average for filtering
#define useThermistor           true      // Use onboard thermistor for temperature measurement
#define ThermistorPIN A0                  // Analog Pin 0
//********************************************************************************************************

#define sensorNotTiltedRedLED    LOW      // Corresponding Red LED control level
#define sensorNotTiltedGreenLED HIGH      // Corresponding GreenLED control level
#define heater_OFF_BlueLED       LOW      // Corresponding GreenLED control level

#define sensorTiltedRedLED      HIGH      // Corresponding Red LED control level
#define sensorTiltedGreenLED     LOW      // Corresponding GreenLED control level
#define heater_ON_BlueLED       HIGH      // Corresponding GreenLED control level

float vcc = 5.0;                          // set to the measured Vcc
float pad = 9990;                         // balance/pad resistor value
float thermr = 10000;                     // thermistor nominal resistance
float enclosureTemp = 20;

const byte rxPin = 2;       // SoftwareSerial RX pin
const byte txPin = 4;       // SoftwareSerial TX pin
const byte TiltSW = 3;      // Tilt Switch input pin
const byte zero = 0;

// Status Indicators
// GREEN:    State= Not tilted,    Heating OFF
// MAGENTA:  State= Not Tilted,    Heating ON,
// RED:      State= Tilted sensor, Heating OFF
// PINK:     State= Tilted Sensor, Heating ON

const byte RGBled_Red = 6;  // RGB Status led RED pin - TILT INDICATOR
const byte RGBled_Green = 7;// RGB Status led GREEN pin - NOT TILTED INDICATOR
const byte RGBled_Blue = 8; // RGB Status led BLUE pin - HEATER ON INDICATOR
const byte heater = 10;     // PWM input to Heating transistor
const byte Led = 13;        // Activity LED
bool RGBled_Blue_state;

byte RedOutputLevel = LOW;
byte GreenOutputLevel = LOW;
byte BlueOutputLevel = LOW;

int TempUS100 = 0;
int allTempReadings[numReadings];
int HeatingLimitPercent = MaxHeatingPowerPercent; // Puissance maximale appliqué sur la résistance de chauffage

bool LedState = LOW;
int HeatingPower = 0;
volatile bool Tilt = HIGH;
int TempRequest = 0; // 0:no temp request, 1:US100 temp request, 2:thermistor temp request
bool thermistorSuccess = true;

unsigned long Period = 500;
unsigned long nextSampleTime = millis() + Period;

// set up a new serial object to communicate with the US100
NeoSWSerial US100 (rxPin, txPin);

// Setup
void setup() {
  // Set the data rate for the SoftwareSerial port
  US100.begin(9600);  // Start SoftwareSerial port
  US100.flush();
  Serial.begin(9600); // Start Serial port
  Serial.flush();     // Make sure buffer is empty

  // Set I/O pin mode
  pinMode(13, OUTPUT); // Set Activity LED pin to OUTPUT
  pinMode(heater, OUTPUT); // Set heater pin to OUTPUT
  pinMode(TiltSW, INPUT); // Set tilt switch pin to INPUT
  pinMode(RGBled_Red, OUTPUT); // Set RGBled_Red as OUTPUT
  pinMode(RGBled_Green, OUTPUT); // Set RGBled_Green as OUTPUT
  pinMode(RGBled_Blue, OUTPUT); // Set RGBLed_Blue as OUTPUT

  // Initialize the RGB Led to BLUE
  digitalWrite(RGBled_Red, LOW);
  digitalWrite(RGBled_Green, HIGH);
  digitalWrite(RGBled_Blue, HIGH);
  delay(500);
  digitalWrite(RGBled_Red, HIGH);
  digitalWrite(RGBled_Blue, LOW);
  delay(2000);

  RedOutputLevel = sensorNotTiltedRedLED;;
  GreenOutputLevel = sensorNotTiltedGreenLED;
  BlueOutputLevel = heater_OFF_BlueLED;
}

// Main program loop
void loop() {
  // Monitor the data on the serial ports
  checkSerialPorts(); // Transfer data if available
  Tilt = checkTiltSW(); // Check the tilt status

  // Do additional processing every "Period" ms
  unsigned long LoopTime = millis();
  if (LoopTime > nextSampleTime) {
    nextSampleTime = LoopTime + Period - 1;
    // First Adjust heating as required
    float thermistorReading = Thermistor(analogRead(ThermistorPIN)); // read ADC and  convert it to Celsius
    delay(2);
    if (useThermistor && thermistorSuccess ) {
      // Use the temperature measured by the onboard thermistor
      enclosureTemp = thermistorReading;
    } else {
      // Use the temperature measured by the US100 sensor
      enclosureTemp = 26;
    }
    simpleThermostat(HeatingSetPoint, enclosureTemp);
    //    bool Tilt = checkTiltSW(); // check tilt ans set RGB Led color.
    // Then flash the Arduino Led to show activities
    setStatusLED(); //Set RGB status led
  }
}

// Monitor the serial ports taking account the TILT status of the sensor
// Intercept the temperature data from the US100 sensor for use bay the thermostat
void checkSerialPorts() {
  uint8_t oneByteFromSerial = 0; // One byte buffer for data comming from Serial port
  uint8_t oneByteFromUS100 = 0; // One byte buffer for data comming from US100 port
  int Temp45 = 0; // Raw temperature reading

  // Forward data from Serial to US100
  if (Serial.available()) {
    oneByteFromSerial = Serial.read(); // Read one byte from the Serial port
    //    US100.write(oneByteFromSerial); // Forward it to the US100 sensor
    switch (oneByteFromSerial) {
      case 'P': // 0X50 - Request for temperature
        US100.write(oneByteFromSerial); // Forward it to the US100 sensor
        TempRequest = 1; // It's a request for US100 temperature
        break;
      case 'U': // 0X55 - Request for distance
        // if nothing else matches, do the default
        US100.write(oneByteFromSerial); // Forward it to the US100 sensor
        TempRequest = 0; // No, must be a request for distance.
        break;
      case 'Z': // 0X5A - Request for thermistor temperature
        //do something when var equals 0X5A
        TempRequest = 2; // It's a request for thermistor temperature
        Serial.write((byte) (enclosureTemp + 45)); // Simulate output from US100 by substracting 45
        break;
      default:
        // Unknown request - Let it got through US100
        US100.write(oneByteFromSerial); // Forward it to the US100 sensor
        TempRequest = 0; // No, must be a request for distance.
        break;
    }
    LedState = HIGH;
    digitalWrite(Led, LedState);
  }

  // Forward data from US100 to Serial
  if (US100.available()) {
    oneByteFromUS100 = US100.read(); // Read one byte from the US100
    //    Serial.write(oneByteFromUS100);
    if (!Tilt) {
      // Sensor is NOT tilted, forward data byte as is
      Serial.write(oneByteFromUS100);
    } else {
      // Sensor IS Tilted
      if (!(TempRequest == 1)) {
        // It's not a temperature request. Return "0" to indicate invalid data in case of tilt
        Serial.write(zero);
      } else {
        // Its a temperature request, let the data go throught
        Serial.write(oneByteFromUS100);
      }
    }
    RGBled_Blue_state = RGBled_Blue
    LedState = HIGH;
    digitalWrite(Led, LedState);
  }
  delay (2);
  LedState = LOW;
  digitalWrite(Led, LedState);
}

// Check the state of the TILT Switch and adjust the color of the status LED
// Return the Tilt Status
bool checkTiltSW() {
  bool tiltStatus = !digitalRead(TiltSW); // Note: LOW input indicate sensor IS tilted
  if (!tiltStatus) {
    // Sensor is NOT tilted
    RedOutputLevel = sensorNotTiltedRedLED;
    GreenOutputLevel = sensorNotTiltedGreenLED;
  } else {
    // If sensor IS Tilted
    RedOutputLevel = sensorTiltedRedLED;
    GreenOutputLevel = sensorTiltedGreenLED;
  }
  return tiltStatus;
}

// Filtre par moyenne mobile pour les températures
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
int AvgTempReading(int thisReading) {
  long Avg = 0;
  for (int i = 1; i < numReadings; i++) {
    allTempReadings[i - 1] = allTempReadings[i]; //Shift all readings
    Avg += allTempReadings[i - 1]; //Total of readings except the last one
  }
  allTempReadings[numReadings - 1] = thisReading; //Current reading in the last position
  Avg += thisReading; //total including the last one
  return (Avg); // Avg sera divisé par numReadings au moment de la publication
}

// Imprémentation d'un thermostat simple ON/OFF
// La routine ne fonctionne que si un capteur de température interne est trouvé
int simpleThermostat(float setPoint, float actualTemp) {
  if (actualTemp < (setPoint - 0.5)) {
    HeatingPower =  256 *  HeatingLimitPercent / 100;
    BlueOutputLevel = heater_ON_BlueLED; // Turn blue led ON on when heater is ON
  } else if (actualTemp > (setPoint + 0.5)) {
    HeatingPower =  0;
    BlueOutputLevel = heater_OFF_BlueLED; // Turn blue led OFF on when heater is OFF
  }

  analogWrite(heater, HeatingPower);
  return HeatingPower;
}

void setStatusLED() {
  // Note The LED are active LOW. 255 is OFF, 0 is full ON
  digitalWrite(RGBled_Red, !RedOutputLevel); // Set RED color level
  digitalWrite(RGBled_Green, !GreenOutputLevel); // Set GREEN color level
  digitalWrite(RGBled_Blue, !BlueOutputLevel); // Set BLUE  color level
}

float Thermistor(int RawADC) {
  long Resistance;
  float Temp; // Dual-Purpose variable to save space.

  Resistance = pad * ((1024.0 / RawADC) - 1);
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15; // Convert Kelvin to Celsius
  if ( Temp > -20 || Temp < 45) {
    thermistorSuccess = true;
  } else {
    thermistorSuccess = false;
  }

  return Temp;                                // Return the Temperature
}



