/*
  =====================================================================
  SMART HOUSE MINI PROJECT
  ARDUINO UNO + HC-05 BLUETOOTH
  =====================================================================

  SERIAL MONITOR:
    9600 baud

  HC-05:
    38400 baud

  HC-05 WIRING:
    TXD -> Arduino D10
    RXD -> Arduino D11
    VCC -> Arduino 5V
    GND -> Arduino GND


  =====================================================================
  LDR LOGIC
  =====================================================================

    0 - 2000  = DAY
    > 2000    = NIGHT

  Arduino UNO ADC = 0 - 1023

  Code scales:
    0 - 1023 -> 0 - 4095


  =====================================================================
  WATER SENSOR
  =====================================================================

    < 2000     = DRY
    >= 2000    = WET


  =====================================================================
  OUTDOOR LED AUTO
  =====================================================================

    DAY + DRY       -> OFF
    NIGHT OR WET    -> ON


  =====================================================================
  OUTDOOR LED MANUAL
  =====================================================================

    OUTDOOR ON
    OUTDOOR OFF
    OUTDOOR AUTO


  =====================================================================
  WINDOW SERVO
  =====================================================================

    Continuous rotation servo

    RIGHT = 1600 us
    STOP  = 1485 us
    LEFT  = 1350 us


  =====================================================================
  AMPAIAN SERVO
  =====================================================================

    Positional servo

    OPEN  = 0 degrees
    CLOSE = 90 degrees

    AUTO:
      DAY + DRY       -> OPEN
      NIGHT OR WET    -> CLOSE


  =====================================================================
  SOUND SENSOR
  =====================================================================

    LOW  = SOUND DETECTED
    HIGH = NORMAL

    Sound detected:
      -> Buzzer ON for 5 seconds

    After 5 seconds:
      -> Buzzer OFF

    Sensor must return to normal before
    another sound can trigger the buzzer.


  =====================================================================
  OTHER
  =====================================================================

    Indoor LED -> ON / OFF
    Fan        -> ON / OFF

  =====================================================================
*/


#include <Servo.h>
#include <SoftwareSerial.h>


// =====================================================================
// ============================== PINS ==================================
// =====================================================================

// Indoor LEDs
#define PIN_LED_INDOOR_1   2
#define PIN_LED_INDOOR_2   3

// Fan relay
#define PIN_FAN_RELAY      4

// Outdoor LEDs
#define PIN_LED_OUTDOOR_1  5
#define PIN_LED_OUTDOOR_2  6

// Sound sensor
#define PIN_SOUND_SENSOR   7

// Buzzer
#define PIN_BUZZER         8

// Window continuous servo
#define PIN_SERVO_WINDOW   9

// HC-05
#define PIN_BT_RX          10
#define PIN_BT_TX          11

// Ampaian positional servo
#define PIN_SERVO_AMPAIAN  12

// Water sensor
#define PIN_WATER_SENSOR   A0

// LDR
#define PIN_LDR            A1


// =====================================================================
// =========================== HC-05 ===================================
// =====================================================================
//
// Arduino D10 = RX
// Arduino D11 = TX
//

SoftwareSerial BTSerial(
  PIN_BT_RX,
  PIN_BT_TX
);


// =====================================================================
// =========================== SERVO ====================================
// =====================================================================

Servo servoWindow;
Servo servoAmpaian;


// =====================================================================
// ======================= WINDOW SERVO =================================
// =====================================================================

// YOUR ORIGINAL VALUES

const int SERVO_STOP_US  = 1485;
const int SERVO_RIGHT_US = 1600;
const int SERVO_LEFT_US  = 1350;


// =====================================================================
// ======================= AMPAIAN SERVO ================================
// =====================================================================
//
// REVERSED TO CORRECT YOUR AMPAIAN DIRECTION
//
// OPEN  = 0 degrees
// CLOSE = 90 degrees
//

const int AMPAIAN_OPEN_ANGLE  = 0;
const int AMPAIAN_CLOSE_ANGLE = 90;


// =====================================================================
// ======================= SENSOR THRESHOLD =============================
// =====================================================================

const int LDR_THRESHOLD   = 2000;
const int WATER_THRESHOLD = 2000;


// =====================================================================
// ========================= BUZZER =====================================
// =====================================================================

const unsigned long BUZZ_DURATION_MS = 5000;


// =====================================================================
// ======================== SENSOR TIMER =================================
// =====================================================================

unsigned long lastSensorRead = 0;

const unsigned long SENSOR_INTERVAL = 2000;


// =====================================================================
// ====================== COMMAND BUFFER =================================
// =====================================================================

String btCommand = "";
String serialCommand = "";

unsigned long lastBTCharTime = 0;
unsigned long lastSerialCharTime = 0;


// =====================================================================
// ========================= SYSTEM STATES ===============================
// =====================================================================

// Indoor LED
bool ledIndoorState = false;


// Fan
bool fanState = false;


// Window
String windowServoState = "STOP";


// Outdoor LED
bool ledOutdoorAutoState   = false;
bool ledOutdoorManualState = false;
bool ledOutdoorState       = false;

// true  = MANUAL
// false = AUTO

bool outdoorManualMode = false;


// Water sensor
int waterRawValue = 0;
int waterValue = 0;

bool waterWet = false;


// LDR
int ldrRawValue = 0;
int ldrValue = 0;

// true  = DAY
// false = NIGHT

bool isDay = true;


// Ampaian
int ampaianPosition = AMPAIAN_CLOSE_ANGLE;

// true  = AUTO
// false = MANUAL

bool ampaianAutoMode = true;


// Buzzer
bool buzzerActive = false;

// Prevent repeated triggering
bool soundTriggered = false;

// Ready for next sound
bool soundReady = true;

unsigned long buzzerStartTime = 0;


// =====================================================================
// =========================== SEND BOTH =================================
// =====================================================================

void sendMessage(String message)
{
  Serial.println(message);
  BTSerial.println(message);
}


// =====================================================================
// ========================= INDOOR LED ==================================
// =====================================================================

void setIndoorLed(bool on)
{
  ledIndoorState = on;

  digitalWrite(
    PIN_LED_INDOOR_1,
    on ? HIGH : LOW
  );

  digitalWrite(
    PIN_LED_INDOOR_2,
    on ? HIGH : LOW
  );

  sendMessage(
    "INDOOR LED = " +
    String(
      on ? "ON" : "OFF"
    )
  );
}


// =====================================================================
// ============================== FAN ====================================
// =====================================================================

void setFan(bool on)
{
  fanState = on;

  digitalWrite(
    PIN_FAN_RELAY,
    on ? HIGH : LOW
  );

  sendMessage(
    "FAN = " +
    String(
      on ? "ON" : "OFF"
    )
  );
}


// =====================================================================
// ======================== WINDOW RIGHT =================================
// =====================================================================

void servoRight()
{
  servoWindow.writeMicroseconds(
    SERVO_RIGHT_US
  );

  windowServoState =
    "RIGHT";

  sendMessage(
    "WINDOW = RIGHT"
  );

  Serial.print(
    "WINDOW SERVO = "
  );

  Serial.print(
    SERVO_RIGHT_US
  );

  Serial.println(
    " us"
  );
}


// =====================================================================
// ========================= WINDOW STOP =================================
// =====================================================================

void servoStop()
{
  servoWindow.writeMicroseconds(
    SERVO_STOP_US
  );

  windowServoState =
    "STOP";

  sendMessage(
    "WINDOW = STOP"
  );

  Serial.print(
    "WINDOW SERVO = "
  );

  Serial.print(
    SERVO_STOP_US
  );

  Serial.println(
    " us"
  );
}


// =====================================================================
// ========================== WINDOW LEFT =================================
// =====================================================================

void servoLeft()
{
  servoWindow.writeMicroseconds(
    SERVO_LEFT_US
  );

  windowServoState =
    "LEFT";

  sendMessage(
    "WINDOW = LEFT"
  );

  Serial.print(
    "WINDOW SERVO = "
  );

  Serial.print(
    SERVO_LEFT_US
  );

  Serial.println(
    " us"
  );
}


// =====================================================================
// ======================= OUTDOOR LED UPDATE ============================
// =====================================================================

void updateOutdoorLed()
{
  bool finalState;


  if (outdoorManualMode)
  {
    finalState =
      ledOutdoorManualState;
  }
  else
  {
    finalState =
      ledOutdoorAutoState;
  }


  // No change
  if (
    finalState ==
    ledOutdoorState
  )
  {
    return;
  }


  ledOutdoorState =
    finalState;


  digitalWrite(
    PIN_LED_OUTDOOR_1,
    finalState ? HIGH : LOW
  );

  digitalWrite(
    PIN_LED_OUTDOOR_2,
    finalState ? HIGH : LOW
  );


  sendMessage(
    "OUTDOOR LED = " +
    String(
      finalState
      ? "ON"
      : "OFF"
    )
  );
}


// =====================================================================
// ===================== OUTDOOR MANUAL =================================
// =====================================================================

void setOutdoorManualLed(bool on)
{
  outdoorManualMode =
    true;


  ledOutdoorManualState =
    on;


  sendMessage(
    "OUTDOOR MANUAL = " +
    String(
      on
      ? "ON"
      : "OFF"
    )
  );


  updateOutdoorLed();
}


// =====================================================================
// ======================= OUTDOOR AUTO MODE =============================
// =====================================================================

void setOutdoorAutoMode()
{
  outdoorManualMode =
    false;


  sendMessage(
    "OUTDOOR = AUTO MODE"
  );


  updateOutdoorLed();
}


// =====================================================================
// ====================== OUTDOOR AUTO LOGIC =============================
// =====================================================================

void updateOutdoorAuto()
{
  /*
    NIGHT OR WET -> ON
    DAY + DRY    -> OFF
  */

  ledOutdoorAutoState =
    waterWet ||
    !isDay;


  updateOutdoorLed();
}


// =====================================================================
// ===================== AMPAIAN POSITION ================================
// =====================================================================

void setAmpaianPosition(int angle)
{
  angle =
    constrain(
      angle,
      0,
      180
    );


  ampaianPosition =
    angle;


  servoAmpaian.write(
    angle
  );


  sendMessage(
    "AMPAIAN POSITION = " +
    String(
      angle
    ) +
    " DEG"
  );
}


// =====================================================================
// ======================== AMPAIAN OPEN =================================
// =====================================================================

void ampaianOpen()
{
  // Manual mode

  ampaianAutoMode =
    false;


  setAmpaianPosition(
    AMPAIAN_OPEN_ANGLE
  );


  sendMessage(
    "AMPAIAN = OPEN / MANUAL"
  );
}


// =====================================================================
// ======================== AMPAIAN CLOSE ================================
// =====================================================================

void ampaianClose()
{
  // Manual mode

  ampaianAutoMode =
    false;


  setAmpaianPosition(
    AMPAIAN_CLOSE_ANGLE
  );


  sendMessage(
    "AMPAIAN = CLOSE / MANUAL"
  );
}


// =====================================================================
// ========================= AMPAIAN AUTO ================================
// =====================================================================

void ampaianAuto()
{
  ampaianAutoMode =
    true;


  sendMessage(
    "AMPAIAN = AUTO MODE"
  );


  updateAmpaianAuto();
}


// =====================================================================
// ====================== AMPAIAN AUTO LOGIC =============================
// =====================================================================

void updateAmpaianAuto()
{
  // Ignore automatic control in manual mode

  if (!ampaianAutoMode)
  {
    return;
  }


  /*
    AUTO:

    DAY + DRY       -> OPEN
    NIGHT OR WET    -> CLOSE
  */


  if (
    waterWet ||
    !isDay
  )
  {
    // CLOSE

    if (
      ampaianPosition !=
      AMPAIAN_CLOSE_ANGLE
    )
    {
      setAmpaianPosition(
        AMPAIAN_CLOSE_ANGLE
      );
    }
  }


  else
  {
    // OPEN

    if (
      ampaianPosition !=
      AMPAIAN_OPEN_ANGLE
    )
    {
      setAmpaianPosition(
        AMPAIAN_OPEN_ANGLE
      );
    }
  }
}


// =====================================================================
// ======================= SOUND SENSOR =================================
// =====================================================================
//
// Sensor logic:
//
// LOW  = sound detected
// HIGH = normal
//

const int SOUND_ACTIVE_LEVEL = LOW;


// =====================================================================
// ======================= CHECK SOUND ==================================
// =====================================================================

void checkSoundSensor()
{
  int soundNow =
    digitalRead(
      PIN_SOUND_SENSOR
    );


  // ================================================================
  // SOUND DETECTED
  // ================================================================

  if (
    soundNow ==
    SOUND_ACTIVE_LEVEL
  )
  {
    if (
      soundReady &&
      !soundTriggered &&
      !buzzerActive
    )
    {
      soundTriggered =
        true;


      soundReady =
        false;


      buzzerActive =
        true;


      buzzerStartTime =
        millis();


      digitalWrite(
        PIN_BUZZER,
        HIGH
      );


      sendMessage(
        "SOUND DETECTED -> BUZZER ON"
      );
    }
  }


  // ================================================================
  // BUZZER OFF AFTER 5 SECONDS
  // ================================================================

  if (
    buzzerActive &&
    millis() - buzzerStartTime >=
    BUZZ_DURATION_MS
  )
  {
    buzzerActive =
      false;


    digitalWrite(
      PIN_BUZZER,
      LOW
    );


    sendMessage(
      "BUZZER OFF (5s)"
    );
  }


  // ================================================================
  // RE-ARM
  // ================================================================
  //
  // Sensor must return HIGH before
  // another sound can trigger the buzzer.
  //

  if (
    !buzzerActive &&
    soundTriggered &&
    soundNow !=
    SOUND_ACTIVE_LEVEL
  )
  {
    soundTriggered =
      false;


    soundReady =
      true;


    Serial.println(
      "SOUND SENSOR READY"
    );
  }
}


// =====================================================================
// =========================== LDR ======================================
// =====================================================================

void checkLDRSensor()
{
  ldrRawValue =
    analogRead(
      PIN_LDR
    );


  // Convert UNO ADC 0-1023
  // to 0-4095

  ldrValue =
    (int)(
      (long)ldrRawValue *
      4095L /
      1023L
    );


  /*
    0 - 2000 = DAY
    > 2000   = NIGHT
  */

  bool newDay =
    ldrValue <=
    LDR_THRESHOLD;


  if (
    newDay !=
    isDay
  )
  {
    isDay =
      newDay;


    sendMessage(
      "LDR STATUS = " +
      String(
        isDay
        ? "DAY"
        : "NIGHT"
      )
    );
  }


  Serial.print(
    "LDR RAW = "
  );

  Serial.print(
    ldrRawValue
  );


  Serial.print(
    " | SCALED = "
  );

  Serial.print(
    ldrValue
  );


  Serial.print(
    " | STATUS = "
  );


  Serial.println(
    isDay
    ? "DAY"
    : "NIGHT"
  );
}


// =====================================================================
// ======================== WATER SENSOR ================================
// =====================================================================

void checkWaterSensor()
{
  waterRawValue =
    analogRead(
      PIN_WATER_SENSOR
    );


  // Convert UNO ADC 0-1023
  // to 0-4095

  waterValue =
    (int)(
      (long)waterRawValue *
      4095L /
      1023L
    );


  /*
    < 2000  = DRY
    >= 2000 = WET
  */

  bool newWaterWet =
    waterValue >=
    WATER_THRESHOLD;


  if (
    newWaterWet !=
    waterWet
  )
  {
    waterWet =
      newWaterWet;


    sendMessage(
      "WATER = " +
      String(
        waterWet
        ? "WET"
        : "DRY"
      )
    );
  }


  Serial.print(
    "WATER RAW = "
  );

  Serial.print(
    waterRawValue
  );


  Serial.print(
    " | SCALED = "
  );

  Serial.print(
    waterValue
  );


  Serial.print(
    " | STATUS = "
  );


  Serial.println(
    waterWet
    ? "WET"
    : "DRY"
  );
}


// =====================================================================
// ====================== UPDATE SENSORS ================================
// =====================================================================

void updateSensors()
{
  // LDR
  checkLDRSensor();


  // Water sensor
  checkWaterSensor();


  // Outdoor automatic control
  updateOutdoorAuto();


  // Ampaian automatic control
  updateAmpaianAuto();
}


// =====================================================================
// =========================== STATUS ===================================
// =====================================================================

void sendStatus()
{
  sendMessage(
    "======================"
  );


  sendMessage(
    " SMART HOUSE STATUS"
  );


  sendMessage(
    "======================"
  );


  // Indoor LED

  sendMessage(
    "Indoor LED: " +
    String(
      ledIndoorState
      ? "ON"
      : "OFF"
    )
  );


  // Fan

  sendMessage(
    "Fan: " +
    String(
      fanState
      ? "ON"
      : "OFF"
    )
  );


  // Window

  sendMessage(
    "Window: " +
    windowServoState
  );


  // LDR

  sendMessage(
    "LDR: " +
    String(
      ldrValue
    ) +
    " = " +
    String(
      isDay
      ? "DAY"
      : "NIGHT"
    )
  );


  // Water

  sendMessage(
    "Water: " +
    String(
      waterWet
      ? "WET"
      : "DRY"
    )
  );


  // Outdoor Mode

  sendMessage(
    "Outdoor Mode: " +
    String(
      outdoorManualMode
      ? "MANUAL"
      : "AUTO"
    )
  );


  // Outdoor LED

  sendMessage(
    "Outdoor LED: " +
    String(
      ledOutdoorState
      ? "ON"
      : "OFF"
    )
  );


  // Ampaian Mode

  sendMessage(
    "Ampaian Mode: " +
    String(
      ampaianAutoMode
      ? "AUTO"
      : "MANUAL"
    )
  );


  // Ampaian position

  sendMessage(
    "Ampaian Position: " +
    String(
      ampaianPosition
    ) +
    " DEG"
  );


  // Buzzer

  sendMessage(
    "Buzzer: " +
    String(
      buzzerActive
      ? "ON"
      : "OFF"
    )
  );


  sendMessage(
    "======================"
  );
}


// =====================================================================
// ============================= HELP ==================================
// =====================================================================

void sendHelp()
{
  sendMessage(
    "====== COMMANDS ======"
  );


  sendMessage(
    "LED ON"
  );

  sendMessage(
    "LED OFF"
  );


  sendMessage(
    "FAN ON"
  );

  sendMessage(
    "FAN OFF"
  );


  sendMessage(
    "WINDOW RIGHT"
  );

  sendMessage(
    "WINDOW STOP"
  );

  sendMessage(
    "WINDOW LEFT"
  );


  sendMessage(
    "OUTDOOR ON"
  );

  sendMessage(
    "OUTDOOR OFF"
  );

  sendMessage(
    "OUTDOOR AUTO"
  );


  sendMessage(
    "AMPAIAN OPEN"
  );

  sendMessage(
    "AMPAIAN CLOSE"
  );

  sendMessage(
    "AMPAIAN AUTO"
  );


  sendMessage(
    "STATUS"
  );

  sendMessage(
    "HELP"
  );


  sendMessage(
    "======================"
  );
}


// =====================================================================
// ======================= PROCESS COMMAND ==============================
// =====================================================================

void processCommand(
  String command
)
{
  command.trim();

  command.toUpperCase();


  if (
    command.length() == 0
  )
  {
    return;
  }


  Serial.print(
    "COMMAND RECEIVED = "
  );

  Serial.println(
    command
  );


  // ================================================================
  // INDOOR LED
  // ================================================================

  if (
    command == "LED ON"
  )
  {
    setIndoorLed(true);
  }


  else if (
    command == "LED OFF"
  )
  {
    setIndoorLed(false);
  }


  // ================================================================
  // FAN
  // ================================================================

  else if (
    command == "FAN ON"
  )
  {
    setFan(true);
  }


  else if (
    command == "FAN OFF"
  )
  {
    setFan(false);
  }


  // ================================================================
  // WINDOW
  // ================================================================

  else if (
    command == "WINDOW RIGHT"
  )
  {
    servoRight();
  }


  else if (
    command == "WINDOW STOP"
  )
  {
    servoStop();
  }


  else if (
    command == "WINDOW LEFT"
  )
  {
    servoLeft();
  }


  // ================================================================
  // OUTDOOR LED
  // ================================================================

  else if (
    command == "OUTDOOR ON"
  )
  {
    setOutdoorManualLed(true);
  }


  else if (
    command == "OUTDOOR OFF"
  )
  {
    setOutdoorManualLed(false);
  }


  else if (
    command == "OUTDOOR AUTO"
  )
  {
    setOutdoorAutoMode();
  }


  // ================================================================
  // AMPAIAN
  // ================================================================

  else if (
    command == "AMPAIAN OPEN"
  )
  {
    ampaianOpen();
  }


  else if (
    command == "AMPAIAN CLOSE"
  )
  {
    ampaianClose();
  }


  else if (
    command == "AMPAIAN AUTO"
  )
  {
    ampaianAuto();
  }


  // ================================================================
  // STATUS
  // ================================================================

  else if (
    command == "STATUS"
  )
  {
    sendStatus();
  }


  // ================================================================
  // HELP
  // ================================================================

  else if (
    command == "HELP"
  )
  {
    sendHelp();
  }


  // ================================================================
  // UNKNOWN
  // ================================================================

  else
  {
    sendMessage(
      "UNKNOWN COMMAND"
    );


    sendMessage(
      "SEND HELP"
    );
  }
}


// =====================================================================
// ===================== CHECK BLUETOOTH ================================
// =====================================================================
//
// Supports:
//
// 1. App sends newline
// 2. App does not send newline
//
// If no newline, command executes after 250 ms.
//

void checkBluetooth()
{
  while (
    BTSerial.available()
  )
  {
    char incomingChar =
      BTSerial.read();


    // ---------------------------------------------------------------
    // NEW LINE
    // ---------------------------------------------------------------

    if (
      incomingChar == '\n' ||
      incomingChar == '\r'
    )
    {
      if (
        btCommand.length() > 0
      )
      {
        processCommand(
          btCommand
        );


        btCommand = "";
      }
    }


    // ---------------------------------------------------------------
    // NORMAL CHARACTER
    // ---------------------------------------------------------------

    else
    {
      if (
        btCommand.length() < 50
      )
      {
        btCommand +=
          incomingChar;
      }
    }


    lastBTCharTime =
      millis();
  }


  // ---------------------------------------------------------------
  // NO NEWLINE
  // ---------------------------------------------------------------

  if (
    btCommand.length() > 0 &&
    millis() - lastBTCharTime >= 250
  )
  {
    processCommand(
      btCommand
    );


    btCommand = "";
  }
}


// =====================================================================
// ====================== CHECK SERIAL MONITOR ==========================
// =====================================================================
//
// Serial Monitor:
//   9600 baud
//
// Newline recommended.
//

void checkSerial()
{
  while (
    Serial.available()
  )
  {
    char incomingChar =
      Serial.read();


    // ---------------------------------------------------------------
    // NEW LINE
    // ---------------------------------------------------------------

    if (
      incomingChar == '\n' ||
      incomingChar == '\r'
    )
    {
      if (
        serialCommand.length() > 0
      )
      {
        processCommand(
          serialCommand
        );


        serialCommand = "";
      }
    }


    // ---------------------------------------------------------------
    // NORMAL CHARACTER
    // ---------------------------------------------------------------

    else
    {
      if (
        serialCommand.length() < 50
      )
      {
        serialCommand +=
          incomingChar;
      }
    }


    lastSerialCharTime =
      millis();
  }


  // ---------------------------------------------------------------
  // NO NEWLINE
  // ---------------------------------------------------------------

  if (
    serialCommand.length() > 0 &&
    millis() - lastSerialCharTime >= 250
  )
  {
    processCommand(
      serialCommand
    );


    serialCommand = "";
  }
}


// =====================================================================
// ============================== SETUP =================================
// =====================================================================

void setup()
{
  // ================================================================
  // SERIAL MONITOR
  // ================================================================

  Serial.begin(
    9600
  );


  // ================================================================
  // HC-05
  // ================================================================

  BTSerial.begin(
    38400
  );


  // ================================================================
  // GPIO
  // ================================================================

  pinMode(
    PIN_LED_INDOOR_1,
    OUTPUT
  );


  pinMode(
    PIN_LED_INDOOR_2,
    OUTPUT
  );


  pinMode(
    PIN_FAN_RELAY,
    OUTPUT
  );


  pinMode(
    PIN_LED_OUTDOOR_1,
    OUTPUT
  );


  pinMode(
    PIN_LED_OUTDOOR_2,
    OUTPUT
  );


  pinMode(
    PIN_SOUND_SENSOR,
    INPUT
  );


  pinMode(
    PIN_BUZZER,
    OUTPUT
  );


  pinMode(
    PIN_WATER_SENSOR,
    INPUT
  );


  pinMode(
    PIN_LDR,
    INPUT
  );


  // ================================================================
  // INITIAL OUTPUT STATES
  // ================================================================

  digitalWrite(
    PIN_LED_INDOOR_1,
    LOW
  );


  digitalWrite(
    PIN_LED_INDOOR_2,
    LOW
  );


  digitalWrite(
    PIN_FAN_RELAY,
    LOW
  );


  digitalWrite(
    PIN_LED_OUTDOOR_1,
    LOW
  );


  digitalWrite(
    PIN_LED_OUTDOOR_2,
    LOW
  );


  digitalWrite(
    PIN_BUZZER,
    LOW
  );


  // ================================================================
  // WINDOW CONTINUOUS SERVO
  // ================================================================

  servoWindow.attach(
    PIN_SERVO_WINDOW,
    500,
    2400
  );


  servoWindow.writeMicroseconds(
    SERVO_STOP_US
  );


  windowServoState =
    "STOP";


  // ================================================================
  // AMPAIAN POSITIONAL SERVO
  // ================================================================

  servoAmpaian.attach(
    PIN_SERVO_AMPAIAN
  );


  // Start CLOSED

  servoAmpaian.write(
    AMPAIAN_CLOSE_ANGLE
  );


  ampaianPosition =
    AMPAIAN_CLOSE_ANGLE;


  // ================================================================
  // SOUND SENSOR INITIAL STATE
  // ================================================================

  int initialSoundState =
    digitalRead(
      PIN_SOUND_SENSOR
    );


  if (
    initialSoundState ==
    SOUND_ACTIVE_LEVEL
  )
  {
    soundReady =
      false;

    Serial.println(
      "SOUND SENSOR ACTIVE AT STARTUP"
    );

    Serial.println(
      "WAITING FOR SENSOR TO RETURN TO IDLE..."
    );
  }
  else
  {
    soundReady =
      true;
  }


  soundTriggered =
    false;


  // ================================================================
  // STARTUP MESSAGE
  // ================================================================

  Serial.println();

  Serial.println(
    "======================================"
  );

  Serial.println(
    " SMART HOUSE - ARDUINO UNO + HC-05"
  );

  Serial.println(
    "======================================"
  );

  Serial.println(
    "SERIAL = 9600"
  );

  Serial.println(
    "BLUETOOTH = 38400"
  );

  Serial.println(
    "Bluetooth Ready"
  );

  Serial.println(
    "Serial Monitor Ready"
  );

  Serial.println(
    "Send HELP for commands"
  );


  BTSerial.println(
    "SMART HOUSE READY"
  );

  BTSerial.println(
    "SEND HELP FOR COMMANDS"
  );
}


// =====================================================================
// =============================== LOOP =================================
// =====================================================================

void loop()
{
  // ================================================================
  // SERIAL MONITOR
  // ================================================================

  checkSerial();


  // ================================================================
  // HC-05 BLUETOOTH
  // ================================================================

  checkBluetooth();


  // ================================================================
  // SOUND SENSOR
  // ================================================================

  checkSoundSensor();


  // ================================================================
  // SENSOR UPDATE EVERY 2 SECONDS
  // ================================================================

  if (
    millis() - lastSensorRead >=
    SENSOR_INTERVAL
  )
  {
    lastSensorRead =
      millis();


    updateSensors();
  }
}