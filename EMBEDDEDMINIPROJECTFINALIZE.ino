
/*
  SMART HOUSE MINI PROJECT
  ARDUINO UNO + HC-05 BLUETOOTH
  SOUND SENSOR INTERRUPT VERSION

  SERIAL MONITOR: 9600 baud
  HC-05: 38400 baud

  COMMANDS:
  LED ON / LED OFF
  FAN ON / FAN OFF
  WINDOW RIGHT / WINDOW STOP / WINDOW LEFT
  OUTDOOR ON / OUTDOOR OFF / OUTDOOR AUTO
  AMPAIAN OPEN / AMPAIAN CLOSE / AMPAIAN AUTO
  STATUS / HELP

  SOUND SENSOR INTERRUPT:
  D3 = External Interrupt INT1
  Sound detected -> buzzer ON for 5 seconds
  Outdoor LEDs blink for 10 seconds

  LDR:
  <= 2000 = DAY
  > 2000  = NIGHT

  WATER:
  < 2000  = DRY
  >= 2000 = WET

  OUTDOOR AUTO:
  NIGHT OR WET = ON
  DAY AND DRY = OFF

  AMPAIAN AUTO:
  DAY AND DRY = OPEN
  NIGHT OR WET = CLOSE
*/

#define _SS_MAX_RX_BUFF 32

#include <Servo.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

// =====================================================
// PINS
// =====================================================

#define PIN_LED_INDOOR_1   2
#define PIN_LED_INDOOR_2   A2
#define PIN_FAN_RELAY      4

#define PIN_LED_OUTDOOR_1  5
#define PIN_LED_OUTDOOR_2  6

#define PIN_SOUND_SENSOR   3
#define PIN_BUZZER         8

#define PIN_SERVO_WINDOW   9

#define PIN_BT_RX          10
#define PIN_BT_TX          11

#define PIN_SERVO_AMPAIAN  12

#define PIN_WATER_SENSOR   A0
#define PIN_LDR            A1

#define PIN_DHT            13
#define DHT_TYPE           DHT11

// =====================================================
// OBJECTS
// =====================================================

SoftwareSerial BTSerial(PIN_BT_RX, PIN_BT_TX);

Servo servoWindow;
Servo servoAmpaian;

DHT dht(PIN_DHT, DHT_TYPE);

// =====================================================
// SETTINGS
// =====================================================

const int SERVO_STOP_US = 1485;
const int SERVO_RIGHT_US = 1600;
const int SERVO_LEFT_US = 1350;

const int AMPAIAN_OPEN_ANGLE = 0;
const int AMPAIAN_CLOSE_ANGLE = 90;

const int LDR_THRESHOLD = 2000;
const int WATER_THRESHOLD = 2000;

const int SOUND_ACTIVE_LEVEL = LOW;

const unsigned long BUZZ_DURATION_MS = 5000UL;
const unsigned long OUTDOOR_BLINK_DURATION_MS = 10000UL;
const unsigned long BLINK_INTERVAL_MS = 500UL;
const unsigned long SENSOR_INTERVAL_MS = 2000UL;
const unsigned long COMMAND_GAP_MS = 250UL;

// =====================================================
// SYSTEM STATES
// =====================================================

bool ledIndoorState = false;
bool fanState = false;

enum WindowState : uint8_t {
  WINDOW_STOPPED,
  WINDOW_RIGHT_STATE,
  WINDOW_LEFT_STATE
};

WindowState windowState = WINDOW_STOPPED;

bool ledOutdoorAutoState = false;
bool ledOutdoorManualState = false;
bool ledOutdoorState = false;

// true = MANUAL, false = AUTO
bool outdoorManualMode = false;

int waterRawValue = 0;
int waterValue = 0;
bool waterWet = false;

int ldrRawValue = 0;
int ldrValue = 0;

// true = DAY, false = NIGHT
bool isDay = true;

float roomTemperature = NAN;
float roomHumidity = NAN;

int ampaianPosition = AMPAIAN_CLOSE_ANGLE;

// true = AUTO, false = MANUAL
bool ampaianAutoMode = true;

// Buzzer state
bool buzzerActive = false;
bool soundTriggered = false;
bool soundReady = true;

unsigned long buzzerStartTime = 0;

// Outdoor blink state
bool soundBlinkActive = false;
bool soundBlinkOutputState = false;

unsigned long soundBlinkStartTime = 0;
unsigned long lastBlinkToggleTime = 0;

// Sensor timer
unsigned long lastSensorRead = 0;

// Command buffers
char btCommand[32];
char serialCommand[32];

uint8_t btCommandLength = 0;
uint8_t serialCommandLength = 0;

unsigned long lastBTCharTime = 0;
unsigned long lastSerialCharTime = 0;

// =====================================================
// INTERRUPT VARIABLES
// =====================================================

// Shared between interrupt callback and main loop.
volatile bool soundInterruptFlag = false;

// =====================================================
// SOUND SENSOR INTERRUPT CALLBACK
// =====================================================

// Arduino Uno D3 = external interrupt INT1.
// Keep this function short: do not use Serial, delays,
// Bluetooth or other lengthy operations inside it.

void soundSensorISR() {
  soundInterruptFlag = true;
}

// =====================================================
// MESSAGING
// =====================================================

void sendMessage(const __FlashStringHelper *message) {
  Serial.println(message);
  BTSerial.println(message);
}

void sendLabelValue(
  const __FlashStringHelper *label,
  const __FlashStringHelper *value
) {
  Serial.print(label);
  Serial.println(value);

  BTSerial.print(label);
  BTSerial.println(value);
}

void sendLabelInt(
  const __FlashStringHelper *label,
  long value,
  const __FlashStringHelper *suffix
) {
  Serial.print(label);
  Serial.print(value);
  Serial.println(suffix);

  BTSerial.print(label);
  BTSerial.print(value);
  BTSerial.println(suffix);
}

void sendLabelFloat(
  const __FlashStringHelper *label,
  float value,
  const __FlashStringHelper *suffix
) {
  Serial.print(label);
  Serial.print(value, 1);
  Serial.println(suffix);

  BTSerial.print(label);
  BTSerial.print(value, 1);
  BTSerial.println(suffix);
}

// =====================================================
// WINDOW SERVO STATUS
// =====================================================

const __FlashStringHelper *windowStateText() {
  if (windowState == WINDOW_RIGHT_STATE) {
    return F("RIGHT");
  }

  if (windowState == WINDOW_LEFT_STATE) {
    return F("LEFT");
  }

  return F("STOP");
}

void sendWindowStatusLine() {
  Serial.print(F("Window: "));
  Serial.println(windowStateText());

  BTSerial.print(F("Window: "));
  BTSerial.println(windowStateText());
}

void sendLdrStatusLine() {
  Serial.print(F("LDR: "));
  Serial.print(ldrValue);
  Serial.print(F(" = "));
  Serial.println(isDay ? F("DAY") : F("NIGHT"));

  BTSerial.print(F("LDR: "));
  BTSerial.print(ldrValue);
  BTSerial.print(F(" = "));
  BTSerial.println(isDay ? F("DAY") : F("NIGHT"));
}

// =====================================================
// INDOOR LED
// =====================================================

void setIndoorLed(bool on) {
  ledIndoorState = on;

  digitalWrite(PIN_LED_INDOOR_1, on ? HIGH : LOW);
  digitalWrite(PIN_LED_INDOOR_2, on ? HIGH : LOW);

  sendLabelValue(
    F("INDOOR LED = "),
    on ? F("ON") : F("OFF")
  );
}

// =====================================================
// FAN
// =====================================================

void setFan(bool on) {
  fanState = on;

  digitalWrite(PIN_FAN_RELAY, on ? HIGH : LOW);

  sendLabelValue(
    F("FAN = "),
    on ? F("ON") : F("OFF")
  );
}

// =====================================================
// WINDOW SERVO
// =====================================================

void servoRight() {
  servoWindow.writeMicroseconds(SERVO_RIGHT_US);

  windowState = WINDOW_RIGHT_STATE;

  sendMessage(F("WINDOW = RIGHT"));

  Serial.print(F("WINDOW SERVO = "));
  Serial.print(SERVO_RIGHT_US);
  Serial.println(F(" us"));
}

void servoStop() {
  servoWindow.writeMicroseconds(SERVO_STOP_US);

  windowState = WINDOW_STOPPED;

  sendMessage(F("WINDOW = STOP"));

  Serial.print(F("WINDOW SERVO = "));
  Serial.print(SERVO_STOP_US);
  Serial.println(F(" us"));
}

void servoLeft() {
  servoWindow.writeMicroseconds(SERVO_LEFT_US);

  windowState = WINDOW_LEFT_STATE;

  sendMessage(F("WINDOW = LEFT"));

  Serial.print(F("WINDOW SERVO = "));
  Serial.print(SERVO_LEFT_US);
  Serial.println(F(" us"));
}

// =====================================================
// OUTDOOR LED
// =====================================================

void updateOutdoorLed() {
  bool finalState;

  if (soundBlinkActive) {
    finalState = soundBlinkOutputState;
  } else if (outdoorManualMode) {
    finalState = ledOutdoorManualState;
  } else {
    finalState = ledOutdoorAutoState;
  }

  if (finalState == ledOutdoorState) {
    return;
  }

  ledOutdoorState = finalState;

  digitalWrite(
    PIN_LED_OUTDOOR_1,
    finalState ? HIGH : LOW
  );

  digitalWrite(
    PIN_LED_OUTDOOR_2,
    finalState ? HIGH : LOW
  );

  sendLabelValue(
    F("OUTDOOR LED = "),
    finalState ? F("ON") : F("OFF")
  );
}

// =====================================================
// OUTDOOR MANUAL MODE
// =====================================================

void setOutdoorManualLed(bool on) {
  outdoorManualMode = true;
  ledOutdoorManualState = on;

  sendLabelValue(
    F("OUTDOOR MANUAL = "),
    on ? F("ON") : F("OFF")
  );

  updateOutdoorLed();
}

void setOutdoorAutoMode() {
  outdoorManualMode = false;

  sendMessage(F("OUTDOOR = AUTO MODE"));

  updateOutdoorLed();
}

// =====================================================
// OUTDOOR AUTOMATIC MODE
// =====================================================

void updateOutdoorAuto() {
  // NIGHT OR WET = ON
  // DAY AND DRY = OFF

  ledOutdoorAutoState = waterWet || !isDay;

  updateOutdoorLed();
}

// =====================================================
// AMPAIAN SERVO
// =====================================================

void setAmpaianPosition(int angle) {
  angle = constrain(angle, 0, 180);

  ampaianPosition = angle;

  servoAmpaian.write(angle);

  sendLabelInt(
    F("AMPAIAN POSITION = "),
    angle,
    F(" DEG")
  );
}

void ampaianOpen() {
  ampaianAutoMode = false;

  setAmpaianPosition(AMPAIAN_OPEN_ANGLE);

  sendMessage(F("AMPAIAN = OPEN / MANUAL"));
}

void ampaianClose() {
  ampaianAutoMode = false;

  setAmpaianPosition(AMPAIAN_CLOSE_ANGLE);

  sendMessage(F("AMPAIAN = CLOSE / MANUAL"));
}

void updateAmpaianAuto();

void ampaianAuto() {
  ampaianAutoMode = true;

  sendMessage(F("AMPAIAN = AUTO MODE"));

  updateAmpaianAuto();
}

void updateAmpaianAuto() {
  if (!ampaianAutoMode) {
    return;
  }

  // NIGHT OR WET = CLOSE
  if (waterWet || !isDay) {
    if (ampaianPosition != AMPAIAN_CLOSE_ANGLE) {
      setAmpaianPosition(AMPAIAN_CLOSE_ANGLE);
    }
  } else {
    // DAY AND DRY = OPEN
    if (ampaianPosition != AMPAIAN_OPEN_ANGLE) {
      setAmpaianPosition(AMPAIAN_OPEN_ANGLE);
    }
  }
}

// =====================================================
// START SOUND ALERT
// =====================================================

void startSoundAlert() {
  unsigned long now = millis();

  soundTriggered = true;
  soundReady = false;

  // Start buzzer for 5 seconds.
  buzzerActive = true;
  buzzerStartTime = now;

  digitalWrite(PIN_BUZZER, HIGH);

  // Start outdoor LED blinking for 10 seconds.
  soundBlinkActive = true;
  soundBlinkStartTime = now;
  lastBlinkToggleTime = now;

  soundBlinkOutputState = true;

  updateOutdoorLed();

  sendMessage(F("SOUND DETECTED"));
  sendMessage(F("BUZZER ON FOR 5 SECONDS"));
  sendMessage(F("OUTDOOR LED BLINKING FOR 10 SECONDS"));
}

// =====================================================
// CHECK SOUND INTERRUPT AND BUZZER
// =====================================================

void checkSoundSensor() {
  bool soundEvent;

  // Safely copy and clear the interrupt flag.
  noInterrupts();

  soundEvent = soundInterruptFlag;
  soundInterruptFlag = false;

  interrupts();

  // Process the interrupt event in the main program.
  if (
    soundEvent &&
    soundReady &&
    !buzzerActive &&
    !soundTriggered
  ) {
    startSoundAlert();
  }

  // Turn the buzzer off after 5 seconds.
  if (
    buzzerActive &&
    millis() - buzzerStartTime >= BUZZ_DURATION_MS
  ) {
    buzzerActive = false;

    digitalWrite(PIN_BUZZER, LOW);

    sendMessage(F("BUZZER OFF AFTER 5 SECONDS"));
  }

  // Rearm after the sensor returns to HIGH.
  int currentState = digitalRead(PIN_SOUND_SENSOR);

  if (
    !buzzerActive &&
    !soundReady &&
    currentState != SOUND_ACTIVE_LEVEL
  ) {
    soundTriggered = false;
    soundReady = true;

    Serial.println(F("SOUND SENSOR READY"));
    BTSerial.println(F("SOUND SENSOR READY"));
  }
}

// =====================================================
// UPDATE OUTDOOR LED BLINK
// =====================================================

void updateSoundBlink() {
  if (!soundBlinkActive) {
    return;
  }

  unsigned long now = millis();

  // Stop blinking after 10 seconds.
  if (now - soundBlinkStartTime >= OUTDOOR_BLINK_DURATION_MS) {
    soundBlinkActive = false;

    sendMessage(F("OUTDOOR LED BLINKING FINISHED"));

    // Restore normal AUTO or MANUAL LED state.
    updateOutdoorLed();

    return;
  }

  // Toggle LEDs every 500 ms.
  if (now - lastBlinkToggleTime >= BLINK_INTERVAL_MS) {
    lastBlinkToggleTime = now;

    soundBlinkOutputState = !soundBlinkOutputState;

    updateOutdoorLed();
  }
}

// =====================================================
// LDR SENSOR
// =====================================================

void checkLDRSensor() {
  ldrRawValue = analogRead(PIN_LDR);

  // Convert ADC scale 0-1023 to 0-4095.
  ldrValue = (int)(
    (long)ldrRawValue * 4095L / 1023L
  );

  bool newDay = ldrValue <= LDR_THRESHOLD;

  if (newDay != isDay) {
    isDay = newDay;

    sendLabelValue(
      F("LDR STATUS = "),
      isDay ? F("DAY") : F("NIGHT")
    );
  }

  Serial.print(F("LDR RAW = "));
  Serial.print(ldrRawValue);

  Serial.print(F(" | SCALED = "));
  Serial.print(ldrValue);

  Serial.print(F(" | STATUS = "));
  Serial.println(isDay ? F("DAY") : F("NIGHT"));
}

// =====================================================
// DHT11 TEMPERATURE AND HUMIDITY
// =====================================================

void checkDHT11Sensor() {
  float newHumidity = dht.readHumidity();
  float newTemperature = dht.readTemperature();

  if (isnan(newHumidity) || isnan(newTemperature)) {
    Serial.println(
      F("DHT11 ERROR: Failed to read sensor. Check wiring.")
    );

    return;
  }

  roomHumidity = newHumidity;
  roomTemperature = newTemperature;

  Serial.print(F("ROOM TEMPERATURE = "));
  Serial.print(roomTemperature, 1);

  Serial.print(F(" C | HUMIDITY = "));
  Serial.print(roomHumidity, 1);

  Serial.println(F(" %"));
}

// =====================================================
// WATER SENSOR
// =====================================================

void checkWaterSensor() {
  waterRawValue = analogRead(PIN_WATER_SENSOR);

  // Convert ADC scale 0-1023 to 0-4095.
  waterValue = (int)(
    (long)waterRawValue * 4095L / 1023L
  );

  bool newWaterWet = waterValue >= WATER_THRESHOLD;

  if (newWaterWet != waterWet) {
    waterWet = newWaterWet;

    sendLabelValue(
      F("WATER = "),
      waterWet ? F("WET") : F("DRY")
    );
  }

  Serial.print(F("WATER RAW = "));
  Serial.print(waterRawValue);

  Serial.print(F(" | SCALED = "));
  Serial.print(waterValue);

  Serial.print(F(" | STATUS = "));
  Serial.println(waterWet ? F("WET") : F("DRY"));
}

// =====================================================
// UPDATE ALL SENSORS
// =====================================================

void updateSensors() {
  checkDHT11Sensor();
  checkLDRSensor();
  checkWaterSensor();

  updateOutdoorAuto();
  updateAmpaianAuto();
}

// =====================================================
// STATUS COMMAND
// =====================================================

void sendStatus() {
  sendMessage(F("======================"));
  sendMessage(F(" SMART HOUSE STATUS"));
  sendMessage(F("======================"));

  sendLabelValue(
    F("Indoor LED: "),
    ledIndoorState ? F("ON") : F("OFF")
  );

  sendLabelValue(
    F("Fan: "),
    fanState ? F("ON") : F("OFF")
  );

  sendWindowStatusLine();
  sendLdrStatusLine();

  if (
    !isnan(roomTemperature) &&
    !isnan(roomHumidity)
  ) {
    sendLabelFloat(
      F("Room Temperature: "),
      roomTemperature,
      F(" C")
    );

    sendLabelFloat(
      F("Room Humidity: "),
      roomHumidity,
      F(" %")
    );
  } else {
    sendMessage(F("DHT11: NO READING"));
  }

  sendLabelValue(
    F("Water: "),
    waterWet ? F("WET") : F("DRY")
  );

  sendLabelValue(
    F("Outdoor Mode: "),
    outdoorManualMode ? F("MANUAL") : F("AUTO")
  );

  sendLabelValue(
    F("Outdoor LED: "),
    ledOutdoorState ? F("ON") : F("OFF")
  );

  sendLabelValue(
    F("Ampaian Mode: "),
    ampaianAutoMode ? F("AUTO") : F("MANUAL")
  );

  sendLabelInt(
    F("Ampaian Position: "),
    ampaianPosition,
    F(" DEG")
  );

  sendLabelValue(
    F("Buzzer: "),
    buzzerActive ? F("ON") : F("OFF")
  );

  sendLabelValue(
    F("Sound Alert Blink: "),
    soundBlinkActive ? F("ACTIVE") : F("INACTIVE")
  );

  sendMessage(F("======================"));
}

// =====================================================
// HELP COMMAND
// =====================================================

void sendHelp() {
  sendMessage(F("====== COMMANDS ======"));

  sendMessage(F("LED ON"));
  sendMessage(F("LED OFF"));

  sendMessage(F("FAN ON"));
  sendMessage(F("FAN OFF"));

  sendMessage(F("WINDOW RIGHT"));
  sendMessage(F("WINDOW STOP"));
  sendMessage(F("WINDOW LEFT"));

  sendMessage(F("OUTDOOR ON"));
  sendMessage(F("OUTDOOR OFF"));
  sendMessage(F("OUTDOOR AUTO"));

  sendMessage(F("AMPAIAN OPEN"));
  sendMessage(F("AMPAIAN CLOSE"));
  sendMessage(F("AMPAIAN AUTO"));

  sendMessage(F("STATUS"));
  sendMessage(F("HELP"));

  sendMessage(F("======================"));
}

// =====================================================
// PROCESS COMMAND
// =====================================================

void processCommand(char *command) {
  // Remove leading spaces and tabs.
  while (*command == ' ' || *command == '\t') {
    command++;
  }

  size_t length = strlen(command);

  // Remove trailing spaces and line endings.
  while (
    length > 0 &&
    (
      command[length - 1] == ' ' ||
      command[length - 1] == '\t' ||
      command[length - 1] == '\r' ||
      command[length - 1] == '\n'
    )
  ) {
    command[--length] = '\0';
  }

  // Convert command to uppercase.
  for (size_t i = 0; i < length; i++) {
    command[i] = (char)toupper((unsigned char)command[i]);
  }

  if (length == 0) {
    return;
  }

  Serial.print(F("COMMAND RECEIVED = "));
  Serial.println(command);

  if (strcmp(command, "LED ON") == 0) {
    setIndoorLed(true);

  } else if (strcmp(command, "LED OFF") == 0) {
    setIndoorLed(false);

  } else if (strcmp(command, "FAN ON") == 0) {
    setFan(true);

  } else if (strcmp(command, "FAN OFF") == 0) {
    setFan(false);

  } else if (strcmp(command, "WINDOW RIGHT") == 0) {
    servoRight();

  } else if (strcmp(command, "WINDOW STOP") == 0) {
    servoStop();

  } else if (strcmp(command, "WINDOW LEFT") == 0) {
    servoLeft();

  } else if (strcmp(command, "OUTDOOR ON") == 0) {
    setOutdoorManualLed(true);

  } else if (strcmp(command, "OUTDOOR OFF") == 0) {
    setOutdoorManualLed(false);

  } else if (strcmp(command, "OUTDOOR AUTO") == 0) {
    setOutdoorAutoMode();

  } else if (strcmp(command, "AMPAIAN OPEN") == 0) {
    ampaianOpen();

  } else if (strcmp(command, "AMPAIAN CLOSE") == 0) {
    ampaianClose();

  } else if (strcmp(command, "AMPAIAN AUTO") == 0) {
    ampaianAuto();

  } else if (strcmp(command, "STATUS") == 0) {
    sendStatus();

  } else if (strcmp(command, "HELP") == 0) {
    sendHelp();

  } else {
    sendMessage(F("UNKNOWN COMMAND"));
    sendMessage(F("SEND HELP"));
  }
}

// =====================================================
// CHECK BLUETOOTH
// =====================================================

void checkBluetooth() {
  while (BTSerial.available()) {
    char incomingChar = (char)BTSerial.read();

    if (incomingChar == '\n' || incomingChar == '\r') {
      if (btCommandLength > 0) {
        btCommand[btCommandLength] = '\0';

        processCommand(btCommand);

        btCommandLength = 0;
      }
    } else {
      if (btCommandLength < sizeof(btCommand) - 1) {
        btCommand[btCommandLength++] = incomingChar;
      }
    }

    lastBTCharTime = millis();
  }

  // Support Bluetooth apps without newline.
  if (
    btCommandLength > 0 &&
    millis() - lastBTCharTime >= COMMAND_GAP_MS
  ) {
    btCommand[btCommandLength] = '\0';

    processCommand(btCommand);

    btCommandLength = 0;
  }
}

// =====================================================
// CHECK SERIAL MONITOR
// =====================================================

void checkSerial() {
  while (Serial.available()) {
    char incomingChar = (char)Serial.read();

    if (incomingChar == '\n' || incomingChar == '\r') {
      if (serialCommandLength > 0) {
        serialCommand[serialCommandLength] = '\0';

        processCommand(serialCommand);

        serialCommandLength = 0;
      }
    } else {
      if (serialCommandLength < sizeof(serialCommand) - 1) {
        serialCommand[serialCommandLength++] = incomingChar;
      }
    }

    lastSerialCharTime = millis();
  }

  // Support commands without newline.
  if (
    serialCommandLength > 0 &&
    millis() - lastSerialCharTime >= COMMAND_GAP_MS
  ) {
    serialCommand[serialCommandLength] = '\0';

    processCommand(serialCommand);

    serialCommandLength = 0;
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(9600);
  BTSerial.begin(38400);

  // Configure pins.
  pinMode(PIN_LED_INDOOR_1, OUTPUT);
  pinMode(PIN_LED_INDOOR_2, OUTPUT);

  pinMode(PIN_FAN_RELAY, OUTPUT);

  pinMode(PIN_LED_OUTDOOR_1, OUTPUT);
  pinMode(PIN_LED_OUTDOOR_2, OUTPUT);

  pinMode(PIN_SOUND_SENSOR, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_WATER_SENSOR, INPUT);
  pinMode(PIN_LDR, INPUT);

  pinMode(PIN_DHT, INPUT);

  dht.begin();

  // Initial output states.
  digitalWrite(PIN_LED_INDOOR_1, LOW);
  digitalWrite(PIN_LED_INDOOR_2, LOW);

  digitalWrite(PIN_FAN_RELAY, LOW);

  digitalWrite(PIN_LED_OUTDOOR_1, LOW);
  digitalWrite(PIN_LED_OUTDOOR_2, LOW);

  digitalWrite(PIN_BUZZER, LOW);

  // Window continuous-rotation servo.
  servoWindow.attach(PIN_SERVO_WINDOW, 500, 2400);
  servoWindow.writeMicroseconds(SERVO_STOP_US);

  windowState = WINDOW_STOPPED;

  // Ampaian positional servo.
  servoAmpaian.attach(PIN_SERVO_AMPAIAN);
  servoAmpaian.write(AMPAIAN_CLOSE_ANGLE);

  ampaianPosition = AMPAIAN_CLOSE_ANGLE;

  // Check initial sound sensor state.
  int initialSoundState = digitalRead(PIN_SOUND_SENSOR);

  if (initialSoundState == SOUND_ACTIVE_LEVEL) {
    soundReady = false;
    soundTriggered = false;

    Serial.println(F("SOUND SENSOR ACTIVE AT STARTUP"));
    Serial.println(F("WAITING FOR SENSOR TO RETURN TO IDLE..."));
  } else {
    soundReady = true;
  }

  // ===================================================
  // ENABLE EXTERNAL INTERRUPT
  // Arduino Uno D3 = INT1
  // FALLING = HIGH-to-LOW transition
  // ===================================================

  attachInterrupt(
    digitalPinToInterrupt(PIN_SOUND_SENSOR),
    soundSensorISR,
    FALLING
  );

  // Startup messages.
  Serial.println();

  Serial.println(F("======================================"));
  Serial.println(F(" SMART HOUSE - ARDUINO UNO + HC-05"));
  Serial.println(F("======================================"));

  Serial.println(F("SERIAL = 9600"));
  Serial.println(F("BLUETOOTH = 38400"));

  Serial.println(F("Bluetooth Ready"));
  Serial.println(F("Serial Monitor Ready"));

  Serial.println(F("Sound sensor INT1 interrupt enabled on D3"));
  Serial.println(F("Buzzer duration = 5 seconds"));
  Serial.println(F("Outdoor LED blink duration = 10 seconds"));

  Serial.println(F("DHT11 readings every 2 seconds."));
  Serial.println(F("Send HELP for commands"));

  BTSerial.println(F("SMART HOUSE READY"));
  BTSerial.println(F("SEND HELP FOR COMMANDS"));
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  checkSerial();

  checkBluetooth();

  // Process interrupt events and buzzer timer.
  checkSoundSensor();

  // Update outdoor blinking independently.
  updateSoundBlink();

  // Update sensors every two seconds.
  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = millis();

    updateSensors();
  }
}