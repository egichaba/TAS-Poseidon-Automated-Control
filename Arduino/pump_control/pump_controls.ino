#include <AccelStepper.h>
#include <ezButton.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// --- Stepper Pins ---
#define DIR_PIN1 2
#define STEP_PIN1 3
#define DIR_PIN2 4
#define STEP_PIN2 5

// --- Microstepping Pins ---
#define MS1_PIN 16
#define MS2_PIN 17
#define MS3_PIN 18

// --- Pressure Sensor Pins ---
const int pressureSensorPin1 = A0;
const int pressureSensorPin2 = A1;

// --- GLOBAL CONSTANTS FOR SENSOR ID ---
//const int SENSOR1_ID = 1;
//const int SENSOR2_ID = 2;
const int SENSOR3_ID = 1;
const int SENSOR4_ID = 2;

// --- GLOBAL VARIABLES..?
unsigned long lastSerialTime = 0;
//const unsigned long SERIAL_INTERVAL = 50; // 50ms = 20Hz update rate
const unsigned long SERIAL_INTERVAL = 500; // 500ms or aka 0.5sec = 2Hz update rate


// --- CALIBRATION CONFIGURATION ---
//ex const int SAMPLES_TO_DISCARD = 2900;
//ex const int SAMPLES_TO_COLLECT = 100;

// Span ADC per 1 PSI
//const float SENSOR1_RAW_SPAN = 8.14; 
//const float SENSOR2_RAW_SPAN = 8.15;
//ex const float SENSOR3_RAW_SPAN = 168.43;
//ex const float SENSOR4_RAW_SPAN = 165.49;


// Calibration Variables

float raw_low_1 = 0.0;
float raw_low_2 = 0.0;
float raw_high_1 = 0.0;
float raw_high_2 = 0.0;

// Pressure Range

//Older version//const float pressure_low = 0.0;
//Older Version//const float pressure_high = 51.71;
const float pressure_low = 0.0;

float pressure_high_1 = 100.0;
float pressure_high_2 = 100.0;

bool lowSet1 = false;
bool highSet1 = false;
bool lowSet2 = false;
bool highSet2 = false;

const int CAL_SAMPLES = 100;
// --- EEPROM CALIBRATION STORAGE ---
const int EEPROM_CAL_ADDR = 0;
const unsigned long CAL_MAGIC = 0x54415343; // "TASC"
const int CAL_VERSION = 1;

struct PressureCalibrationData {
  unsigned long magic;
  int version;

  float raw_low_1;
  float raw_high_1;
  float pressure_high_1;
  bool lowSet1;
  bool highSet1;

  float raw_low_2;
  float raw_high_2;
  float pressure_high_2;
  bool lowSet2;
  bool highSet2;
};
// --- Stepper & Mechanical Parameters ---
const float SMOTOR = 200;
const float M = 16;
const float PITCH = 2.0;
const float SYRINGE_DIAMETER_MM = 10.0;

// --- Stepper Objects ---
AccelStepper pump1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper pump2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

// --- Limit Switches ---
ezButton limitSwitch1(6);
ezButton limitSwitch2(7);


// --- Homing / Limit State Variables ---
bool limitHit1 = false;
bool reliefDone1 = false;
bool limitHit2 = false;
bool reliefDone2 = false;

// --- GUI Control State Variables ---
bool initPump1 = false;
bool initPump2 = false;
bool runPump1 = false;
bool runPump2 = false;

// --- Targets ---
long reliefTarget1 = 64000L*2.5;
long reliefTarget2 = 64000L*2.5;

long homeTarget1 = -3500000L;
long homeTarget2 = -3500000L;

// --- Serial Command Buffer ---
String inputString = "";
void saveCalibrationToEEPROM();
bool loadCalibrationFromEEPROM();
void clearCalibrationEEPROM();

//Recommended to add this by llm help with the readings
int readAnalogStable(int pin) {
  analogRead(pin);              // dummy read after switching channels
  delayMicroseconds(300);       // let ADC settle
  return analogRead(pin);       // real reading
}

float averageRawReading(int pin) {
  long sum = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum += readAnalogStable(pin);
    delay(5);
  }

  return (float)sum / CAL_SAMPLES;
}

void zeroSensor(int sensorNumber) {
  if (sensorNumber == 1) {
    raw_low_1 = round(averageRawReading(pressureSensorPin1));
    lowSet1 = true;

    // Force user to re-do high calibration after changing zero
    raw_high_1 = 0.0;
    highSet1 = false;

    Serial.print("ZEROED SENSOR 1: raw_low_1=");
    Serial.println(raw_low_1);
    Serial.println("Sensor 1 high calibration cleared. Run HIGH,1,0,100 next.");

    saveCalibrationToEEPROM();
  }

  else if (sensorNumber == 2) {
    raw_low_2 = round(averageRawReading(pressureSensorPin2));
    lowSet2 = true;

    // Force user to re-do high calibration after changing zero
    raw_high_2 = 0.0;
    highSet2 = false;

    Serial.print("ZEROED SENSOR 2: raw_low_2=");
    Serial.println(raw_low_2);
    Serial.println("Sensor 2 high calibration cleared. Run HIGH,2,0,100 next.");

    saveCalibrationToEEPROM();
  }
}

void highSensor(int sensorNumber, float knownPressure) {
  if (sensorNumber == 1) {
    if (!lowSet1) {
      Serial.println("ERROR: Set ZERO for sensor 1 before HIGH calibration.");
      return;
    }

    raw_high_1 = round(averageRawReading(pressureSensorPin1));

    if (raw_high_1 <= raw_low_1) {
      Serial.println("ERROR: Sensor 1 high raw equals low raw. Calibration invalid.");
      return;
    }

    pressure_high_1 = knownPressure;
    highSet1 = true;

    Serial.print("HIGH CAL SENSOR 1: raw_high_1=");
    Serial.print(raw_high_1);
    Serial.print(" at ");
    Serial.print(pressure_high_1);
    Serial.println(" mmHg");

    saveCalibrationToEEPROM();
  }

  else if (sensorNumber == 2) {
    if (!lowSet2) {
      Serial.println("ERROR: Set ZERO for sensor 2 before HIGH calibration.");
      return;
    }

    raw_high_2 = round(averageRawReading(pressureSensorPin2));

    if (raw_high_2 <= raw_low_2) {
      Serial.println("ERROR: Sensor 2 high raw equals low raw. Calibration invalid.");
      return;
    }

    pressure_high_2 = knownPressure;
    highSet2 = true;

    Serial.print("HIGH CAL SENSOR 2: raw_high_2=");
    Serial.print(raw_high_2);
    Serial.print(" at ");
    Serial.print(pressure_high_2);
    Serial.println(" mmHg");

    saveCalibrationToEEPROM();
  }
}

void saveCalibrationToEEPROM() {
  PressureCalibrationData cal;

  cal.magic = CAL_MAGIC;
  cal.version = CAL_VERSION;

  cal.raw_low_1 = raw_low_1;
  cal.raw_high_1 = raw_high_1;
  cal.pressure_high_1 = pressure_high_1;
  cal.lowSet1 = lowSet1;
  cal.highSet1 = highSet1;

  cal.raw_low_2 = raw_low_2;
  cal.raw_high_2 = raw_high_2;
  cal.pressure_high_2 = pressure_high_2;
  cal.lowSet2 = lowSet2;
  cal.highSet2 = highSet2;

  EEPROM.put(EEPROM_CAL_ADDR, cal);

  Serial.println("Calibration saved to EEPROM.");
}

bool loadCalibrationFromEEPROM() {
  PressureCalibrationData cal;
  EEPROM.get(EEPROM_CAL_ADDR, cal);

  if (cal.magic != CAL_MAGIC || cal.version != CAL_VERSION) {
    Serial.println("No valid EEPROM calibration found. Manual calibration required.");
    return false;
  }

  raw_low_1 = cal.raw_low_1;
  raw_high_1 = cal.raw_high_1;
  pressure_high_1 = cal.pressure_high_1;
  lowSet1 = cal.lowSet1;
  highSet1 = cal.highSet1;

  raw_low_2 = cal.raw_low_2;
  raw_high_2 = cal.raw_high_2;
  pressure_high_2 = cal.pressure_high_2;
  lowSet2 = cal.lowSet2;
  highSet2 = cal.highSet2;

  Serial.println("Calibration loaded from EEPROM.");
  Serial.print("Sensor 1 low/high raw: ");
  Serial.print(raw_low_1);
  Serial.print(" / ");
  Serial.println(raw_high_1);

  Serial.print("Sensor 2 low/high raw: ");
  Serial.print(raw_low_2);
  Serial.print(" / ");
  Serial.println(raw_high_2);

  return true;
}

void clearCalibrationEEPROM() {
  PressureCalibrationData cal;
  cal.magic = 0;
  cal.version = 0;

  EEPROM.put(EEPROM_CAL_ADDR, cal);

  raw_low_1 = 0.0;
  raw_high_1 = 0.0;
  raw_low_2 = 0.0;
  raw_high_2 = 0.0;

  pressure_high_1 = 100.0;
  pressure_high_2 = 100.0;

  lowSet1 = false;
  highSet1 = false;
  lowSet2 = false;
  highSet2 = false;

  Serial.println("EEPROM calibration cleared. Manual recalibration required.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  loadCalibrationFromEEPROM();

  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);

  bool calLoaded = loadCalibrationFromEEPROM();

  if (!calLoaded || !(lowSet1 && highSet1 && lowSet2 && highSet2)) {
    Serial.println("Manual pressure calibration required.");
    Serial.println("Use ZERO,1,0,0 and HIGH,1,0,100 for sensor 1.");
    Serial.println("Use ZERO,2,0,0 and HIGH,2,0,100 for sensor 2.");
  }

//ex  // 1. WARM-UP PHASE
//ex  for (int i = 0; i < SAMPLES_TO_DISCARD; i++) {
//ex    //analogRead(pressureSensorPin1); 
//ex    //analogRead(pressureSensorPin2); 
//ex    readAnalogStable(pressureSensorPin1);
//ex    readAnalogStable(pressureSensorPin2);
//ex    delay(10); 
//ex  }

//ex  // 2. SIMULTANEOUS COLLECTION INTO ARRAYS
//ex  int samples1[SAMPLES_TO_COLLECT];
//ex  int samples2[SAMPLES_TO_COLLECT];
  
//ex  for (int i = 0; i < SAMPLES_TO_COLLECT; i++) {
//ex    //samples1[i] = analogRead(pressureSensorPin1);
//ex    //samples2[i] = analogRead(pressureSensorPin2);
//ex    samples1[i] = readAnalogStable(pressureSensorPin1);
//ex    samples2[i] = readAnalogStable(pressureSensorPin2);
//ex    delay(10); 
//ex  }
  
//ex  // 3. SORT ARRAYS (Bubble Sort to organize data from lowest to highest)
//ex  for (int i = 0; i < SAMPLES_TO_COLLECT - 1; i++) {
//ex    for (int j = 0; j < SAMPLES_TO_COLLECT - i - 1; j++) {
//ex      if (samples1[j] > samples1[j+1]) {
//ex        int temp = samples1[j];
//ex        samples1[j] = samples1[j+1];
//ex        samples1[j+1] = temp;
//ex      }
//ex      if (samples2[j] > samples2[j+1]) {
//ex        int temp = samples2[j];
 //ex       samples2[j] = samples2[j+1];
 //ex       samples2[j+1] = temp;
 //ex     }
//ex    }
//ex  }

//ex  // 4. TRIMMED MEAN CALCULATION (Average only the middle 50 samples, rejecting spikes)
//ex  long sum1 = 0;
//ex  long sum2 = 0;
//ex  int trimStart = SAMPLES_TO_COLLECT / 4;       // Start at index 25
//ex  int trimEnd = SAMPLES_TO_COLLECT - trimStart;   // End at index 75
//ex  int validSamples = trimEnd - trimStart;       // 50 valid samples

//ex  for (int i = trimStart; i < trimEnd; i++) {
//ex    sum1 += samples1[i];
//ex    sum2 += samples2[i];
//ex  }

 //ex float averageRaw1 = (float)sum1 / validSamples;
 //ex raw_low_1 = round(averageRaw1);
 //ex raw_high_1 = raw_low_1 + SENSOR3_RAW_SPAN;
 //ex raw_high_1 = round(raw_high_1);

 //ex float averageRaw2 = (float)sum2 / validSamples;
 //ex raw_low_2 = round(averageRaw2);
 //ex raw_high_2 = raw_low_2 + SENSOR4_RAW_SPAN;
 //ex raw_high_2 = round(raw_high_2);
  
  //ex 2 second buffer before data stream starts
 //ex delay(2000);


  // Set microstepping pins as outputs
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);

  // Set microstepping to 1/16
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, HIGH);

  // Configure Pump 1
  pump1.setMaxSpeed(5000);
  pump1.setAcceleration(2000);
  pump1.setCurrentPosition(0);

  // Configure Pump 2
  pump2.setMaxSpeed(5000);
  pump2.setAcceleration(2000);
  pump2.setCurrentPosition(0);

  Serial.println("Arduino Mega pump control ready");
  Serial.print("F_CPU: ");
  Serial.println(F_CPU);
}

void loop() {
    // 1. Timed Sensor Reading & Serial Output (Non-blocking)
  if (millis() - lastSerialTime >= SERIAL_INTERVAL) {
    lastSerialTime = millis();

    // Read raw sensor values
    //OLD VALUES 
    //int rawValue1 = analogRead(pressureSensorPin1);
    //int rawValue2 = analogRead(pressureSensorPin2);

    int rawValue1 = readAnalogStable(pressureSensorPin1);
    int rawValue2 = readAnalogStable(pressureSensorPin2);

    // Convert to calibrated pressure (mmHg)
//ex    float pressure1 = (rawValue1 - raw_low_1) * (pressure_high - pressure_low) / (raw_high_1 - raw_low_1) + pressure_low;
//ex    float pressure2 = (rawValue2 - raw_low_2) * (pressure_high - pressure_low) / (raw_high_2 - raw_low_2) + pressure_low;
//ex    if (pressure1 < 0) pressure1 = 0;
//ex    if (pressure2 < 0) pressure2 = 0;

    float pressure1 = 0.0;
    float pressure2 = 0.0;

    if (lowSet1 && highSet1 && raw_high_1 != raw_low_1) {
      pressure1 = (rawValue1 - raw_low_1) *
                  (pressure_high_1 - pressure_low) /
                  (raw_high_1 - raw_low_1) + pressure_low;

      if (pressure1 < 0) pressure1 = 0;
    }

    if (lowSet2 && highSet2 && raw_high_2 != raw_low_2) {
      pressure2 = (rawValue2 - raw_low_2) *
                  (pressure_high_2 - pressure_low) /
                  (raw_high_2 - raw_low_2) + pressure_low;

      if (pressure2 < 0) pressure2 = 0;
    }

//ex    if (lowSet1 && highSet1 && raw_high_1 != raw_low_1) {
//ex      pressure1 = (rawValue1 - raw_low_1) *
//ex                  (pressure_high_1 - pressure_low) /
//ex                  (raw_high_1 - raw_low_1) + pressure_low;
//ex
//ex      if (pressure1 < 0) pressure1 = 0;
//ex    }
//ex
//ex    if (lowSet2 && highSet2 && raw_high_2 != raw_low_2) {
//ex      pressure2 = (rawValue2 - raw_low_2) *
//ex                  (pressure_high_2 - pressure_low) /
//ex                  (raw_high_2 - raw_low_2) + pressure_low;
//ex
//ex      if (pressure2 < 0) pressure2 = 0;
//ex}

    // CSV output: both sensors in one line
    Serial.println(String("(") + "(" + String(SENSOR3_ID) + "," + String(pressure1) + ")," + "(" + String(SENSOR4_ID) + "," + String(pressure2) + ")" + ")");
  }
  
  readSerialCommand();

  limitSwitch1.loop();
  limitSwitch2.loop();

  int state1 = limitSwitch1.getState();
  int state2 = limitSwitch2.getState();

  // =====================================================
  // PUMP 1 LIMIT DETECTION DURING INIT / HOMING
  // =====================================================
  if (initPump1 && state1 == HIGH && !limitHit1) {
    Serial.println("Pump 1 LIMIT HIT");

    pump1.stop();
    pump1.setCurrentPosition(0);

    limitHit1 = true;
    reliefDone1 = false;

    pump1.moveTo(reliefTarget1);
  }

  // Pump 1 move forward to initiation point
  if (initPump1 && limitHit1 && !reliefDone1) {
    pump1.run();

    if (pump1.distanceToGo() == 0) {
      reliefDone1 = true;
      initPump1 = false;

      Serial.println("Pump 1 stopped & at initiation point");
    }
  }

  // =====================================================
  // PUMP 2 LIMIT DETECTION DURING INIT / HOMING
  // =====================================================
  if (initPump2 && state2 == HIGH && !limitHit2) {
    Serial.println("Pump 2 LIMIT HIT");

    pump2.stop();
    pump2.setCurrentPosition(0);

    limitHit2 = true;
    reliefDone2 = false;

    pump2.moveTo(reliefTarget2);
  }

  // Pump 2 move forward to initiation point
  if (initPump2 && limitHit2 && !reliefDone2) {
    pump2.run();

    if (pump2.distanceToGo() == 0) {
      reliefDone2 = true;
      initPump2 = false;

      Serial.println("Pump 2 stopped & at initiation point");
    }
  }

  // =====================================================
  // INIT / HOMING MOTION BEFORE LIMIT SWITCH IS HIT
  // =====================================================
  if (initPump1 && !limitHit1) {
    pump1.run();
  }

  if (initPump2 && !limitHit2) {
    pump2.run();
  }

  // =====================================================
  // REGULAR RUN MOTION FROM PYTHON GUI
  // =====================================================
  if (runPump1 && !initPump1) {
    pump1.runSpeed();
  }

  if (runPump2 && !initPump2) {
    pump2.runSpeed();
  }
}

// =====================================================
// SERIAL COMMAND HANDLER
// =====================================================
void readSerialCommand() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      inputString.trim();

      if (inputString.length() > 0) {
        processCommand(inputString);
      }

      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}

void processCommand(String commandLine) {
  int firstComma = commandLine.indexOf(',');
  int secondComma = commandLine.indexOf(',', firstComma + 1);
  int thirdComma = commandLine.indexOf(',', secondComma + 1);

  if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
    Serial.print("Invalid command: ");
    Serial.println(commandLine);
    return;
  }

  String command = commandLine.substring(0, firstComma);
  int pumpNumber = commandLine.substring(firstComma + 1, secondComma).toInt();
  float value = commandLine.substring(thirdComma + 1).toFloat();

  command.trim();

  // =====================================================
  // INIT COMMAND
  // =====================================================
  if (command == "INIT") {
    if (pumpNumber == 1) {
      Serial.println("INIT PUMP 1 STARTED");

      runPump1 = false;
      initPump1 = true;

      limitHit1 = false;
      reliefDone1 = false;

      pump1.setMaxSpeed(5000);
      pump1.setAcceleration(1500);
      pump1.moveTo(homeTarget1);
    }

    else if (pumpNumber == 2) {
      Serial.println("INIT PUMP 2 STARTED");

      runPump2 = false;
      initPump2 = true;

      limitHit2 = false;
      reliefDone2 = false;

      pump2.setMaxSpeed(5000);
      pump2.setAcceleration(1500);
      pump2.moveTo(homeTarget2);
    }
  }

// =====================================================
// ZERO COMMAND
// =====================================================
  else if (command == "ZERO") {
    if (pumpNumber == 1) {
      zeroSensor(1);
    }
    else if (pumpNumber == 2) {
      zeroSensor(2);
    }
  }

  // =====================================================
  // HIGH CALIBRATION COMMAND
  // =====================================================
  else if (command == "HIGH") {
    if (value <= 0) {
      Serial.println("Invalid HIGH calibration pressure");
      return;
    }

    if (pumpNumber == 1) {
      highSensor(1, value);
    }
    else if (pumpNumber == 2) {
      highSensor(2, value);
    }
  }

  else if (command == "CLEARCAL") {
    clearCalibrationEEPROM();
  }


  // =====================================================
  // RUN COMMAND
  // =====================================================
  else if (command == "RUN") {
    if (pumpNumber == 1) {
      Serial.print("RUN PUMP 1 SPEED: ");
      Serial.println(value);

      initPump1 = false;
      runPump1 = true;

      pump1.setMaxSpeed(abs(value));
      pump1.setSpeed(abs(value));
    }

    else if (pumpNumber == 2) {
      Serial.print("RUN PUMP 2 SPEED: ");
      Serial.println(value);

      initPump2 = false;
      runPump2 = true;

      pump2.setMaxSpeed(abs(value));
      pump2.setSpeed(abs(value));
    }
  }

  // =====================================================
  // STOP COMMAND
  // =====================================================
  else if (command == "STOP") {
    if (pumpNumber == 1) {
      Serial.println("STOP PUMP 1");

      runPump1 = false;
      initPump1 = false;

      pump1.stop();
    }

    else if (pumpNumber == 2) {
      Serial.println("STOP PUMP 2");

      runPump2 = false;
      initPump2 = false;

      pump2.stop();
    }
  }

  else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}
