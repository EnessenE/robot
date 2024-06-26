#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#include <iostream>
#include <sstream>
#include <vector>

const char *ssid = "X_WiFi";
const char *password = "XVJGWVTB";

// Pin setup
enum PinSetup : byte
{
  enuPinSetupUnused,
  enuPinSetupInput,
  enuPinSetupInputLatchingHigh,
  enuPinSetupInputLatchingLow,
  enuPinSetupOutput,
  enuPinSetupOutputPWM,
  enuPinSetupMeccanoid
};

byte meccanoidReceiveByte(byte bytPin);
byte meccanoidCommunicate(byte bytPin, byte bytPosition);
void meccanoidSendByte(byte bytPin, byte bytData);
byte meccanoidGetServoMotorPosition(byte bytPin, byte bytPosition);
void meccanoidInitialise(byte bytPin);
void reset();
void iterateAllPins();

void setMotorPosition(byte partPin, byte pinInArray, byte angle);
void setServoLed(byte partPin, byte partInArray, byte color);
void setLed(byte partPin, byte partInArray, int red, int green, int blue, int fade);

void setAllLeds(int red, int green, int blue, int fade);
void initializeRelevantPins();

void connectWiFi();
void checkSerial();

void handleCommand(String rawCommand);

const byte relevantDigitalPins[1] = {D2};
const byte amountOfPins = sizeof(relevantDigitalPins) / sizeof(relevantDigitalPins[0]);

const byte bytLowestAnalogPin = A0;
const byte bytHighestAnalogPin = A0;

PinSetup arrDigitalPinSetup[amountOfPins];
PinSetup arrAnalogPinSetup[bytHighestAnalogPin + 1];

// Latch
const unsigned int intLatchDebounceInterval = 500;

bool arrDigitalLatchTriggered[amountOfPins];
unsigned long arrDigitalLatchDebounceStartTime[amountOfPins];
bool arrAnalogLatchTriggered[bytHighestAnalogPin + 1];
unsigned long arrAnalogLatchDebounceStartTime[bytHighestAnalogPin + 1];

// State
byte arrCurrentValue[amountOfPins][4];
byte arrStartValue[amountOfPins][4];
byte arrEndValue[amountOfPins][4];

unsigned int arrDuration[amountOfPins][4];
unsigned long arrStartTime[amountOfPins][4];

// Meccanoid
byte arrMeccanoidData[amountOfPins][4];
byte arrMeccanoidType[amountOfPins][4];

// runtime
bool dataReceived = false;

#define SendKey 0 // Button to send data Flash BTN on NodeMCU

int port = 7331; // Port number
WiFiServer server(port);

void setup()
{
  // Initialize serial port
  Serial.begin(9600);
  delay(1000);
  Serial.println("Init");
  Serial.println("Currently " + String(amountOfPins) + " pins");
  reset();
  initializeRelevantPins();
  setAllLeds(255, 0, 0, 0);
  Serial.println("Init finished");
  connectWiFi();

  Serial.println("Finished with everything");
}

long lastBlinkTime;
bool blinkToggle = false;
void loop()
{
  // Serial.println("Loop start");
  // iterateAllPins();

  // setMotorPosition(D2, 0, random(24, 233));
  // setServoLed(D2, 0, random(0, 7));
  // setMotorPosition(D2, 1, random(24, 233));
  // setServoLed(D2, 1, random(0, 7));

  // setLed(D2, 0, random(0, 255), random(0, 255), random(0, 255), 0);

  checkSerial();
  if (millis() - lastBlinkTime > 1000)
  {
    lastBlinkTime = millis();
    if (blinkToggle)
    {
      setAllLeds(0, 255, 0, 1);
    }
    else
    {
      setAllLeds(0, 0, 0, 1);
    }
    blinkToggle = !blinkToggle;
  }
  // delay(1000);
}

//******************************************************************************************************************************************************************************************************
// WiFi FUNCTIONS
//******************************************************************************************************************************************************************************************************
void connectWiFi()
{

  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  pinMode(SendKey, INPUT_PULLUP); // Btn to send data

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // Connect to wifi

  Serial.print("Connecting to WiFi:");
  Serial.println(ssid);
  long lastWiFiBlink;
  bool WiFiBlinkToggle = false;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    if (millis() - lastWiFiBlink > 1000)
    {
      lastWiFiBlink = millis();
      if (WiFiBlinkToggle)
      {
        setAllLeds(0, 0, 255, 1);
      }
      else
      {
        setAllLeds(0, 0, 0, 1);
      }
      WiFiBlinkToggle = !WiFiBlinkToggle;
    }
    delay(100);
  }

  Serial.println("");
  setAllLeds(0, 255, 0, 1);
  Serial.print("Connected to ");
  Serial.println(ssid);

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.print("Open Telnet and connect to IP:");
  Serial.print(WiFi.localIP());
  Serial.print(" on port ");
  Serial.println(port);
}

void checkSerial()
{
  WiFiClient client = server.available();

  if (client)
  {
    if (client.connected())
    {
      Serial.println("Client Connected");
      setAllLeds(0, 0, 255, 1);
    }

    while (client.connected())
    {
      while (client.available() > 0)
      {
        // read data from the connected client
        String line = client.readStringUntil('\n');
        Serial.println("new command: " + String(line));
        handleCommand(line);
        client.write("OK");
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
//******************************************************************************************************************************************************************************************************
// CUSTOM FUNCTIONS
//******************************************************************************************************************************************************************************************************
void setAllLeds(int red, int green, int blue, int fade)
{
  // Find the position of the LED

  for (int pin = 0; pin < amountOfPins; pin++)
  {
    byte bytPin = relevantDigitalPins[pin];
    for (byte partInArray = 0; partInArray < 4; partInArray++)
    {
      if (arrMeccanoidType[bytPin][partInArray] == 2)
      {
        setLed(bytPin, partInArray, red, green, blue, fade);
      }
    }
  }
}

void initializeRelevantPins()
{
  for (int pin = 0; pin < amountOfPins; pin++)
  {
    Serial.println("Starting relevant pin: " + String(relevantDigitalPins[pin]));
    meccanoidInitialise(relevantDigitalPins[pin]);
    setAllLeds(255, 255, 255, 1);
    Serial.println("Finished relevant pin: " + String(relevantDigitalPins[pin]));
  }
  Serial.println("All relevant pins started");
}

void setLed(byte partPin, byte partInArray, int red, int green, int blue, int fade)
{
  Serial.println("Requesting " + String(partPin) + " - " + String(partInArray) + " to move to " + String(red) + "/" + String(green) + "/" + String(blue) + " - " + String(fade));

  byte bytLED1 = 0;
  byte bytLED2 = 0;

  // Combine Red, Green, Blue and Fade values into two bytes: bytLED1 = 0GGGRRR; bytLED2 = 1FFFBBB
  bytLED1 = 0x3F & (((green << 3) & 0x38) | (red & 0x07));
  bytLED2 = 0x40 | (((fade << 3) & 0x38) | (blue & 0x07));

  // Set the colour
  arrMeccanoidData[partPin][partInArray] = bytLED1;
  meccanoidCommunicate(partPin, 0);
  arrMeccanoidData[partPin][partInArray] = bytLED2;
  meccanoidCommunicate(partPin, 0);
}

void setServoLed(byte partPin, byte partInArray, byte color)
{
  Serial.println("Requesting " + String(partPin) + " - " + String(partInArray) + " to color to " + String(color));

  byte bytColour = color;
  // Ensure the position values are within end stop range
  // Ensure the colour values are within range
  if (bytColour > 7)
  {
    bytColour = 7;
  }

  // Set the colour
  arrMeccanoidData[partPin][partInArray] = bytColour + 240;
  meccanoidCommunicate(partPin, partInArray);
}

void setMotorPosition(byte partPin, byte partInArray, byte angle)
{
  Serial.println("Requesting " + String(partPin) + " - " + String(partInArray) + " to move to " + String(angle));
  byte bytPosition = angle;

  int randomTime = 1000;
  // Ensure the position values are within end stop range
  if (bytPosition < 24)
  {
    bytPosition = 24;
  }
  if (bytPosition > 232)
  {
    bytPosition = 232;
  }

  // Get the servo position if it currently disabled
  if (arrMeccanoidData[partPin][partInArray] == 0xFA)
  {
    arrCurrentValue[partPin][partInArray] = meccanoidGetServoMotorPosition(partPin, partInArray);
  }

  // Store the start value, end value, start time and duration
  arrStartValue[partPin][partInArray] = arrCurrentValue[partPin][partInArray];
  arrEndValue[partPin][partInArray] = bytPosition;

  arrStartTime[partPin][partInArray] = millis();
  arrDuration[partPin][partInArray] = randomTime;
}

//******************************************************************************************************************************************************************************************************
// MECCANOID FUNCTIONS
//******************************************************************************************************************************************************************************************************

void iterateAllPins()
{
  Serial.println("Iterating through ALL pins!");
  // Iterate through digital pins
  for (int pin = 0; pin < amountOfPins; pin++)
  {
    byte bytPin = relevantDigitalPins[pin];
    Serial.println("Pin " + String(bytPin) + " - " + String(arrDigitalPinSetup[bytPin]));
    switch (arrDigitalPinSetup[bytPin])
    {
    case enuPinSetupInputLatchingHigh:
    {
      // Check input for high latch trigger
      if (digitalRead(bytPin) && millis() - arrDigitalLatchDebounceStartTime[bytPin] > intLatchDebounceInterval)
      {
        arrDigitalLatchTriggered[bytPin] = true;
        arrDigitalLatchDebounceStartTime[bytPin] = millis();
      }
      break;
    }
    case enuPinSetupInputLatchingLow:
    {
      // Check input for low latch trigger
      if (!digitalRead(bytPin) && millis() - arrDigitalLatchDebounceStartTime[bytPin] > intLatchDebounceInterval)
      {
        arrDigitalLatchTriggered[bytPin] = true;
        arrDigitalLatchDebounceStartTime[bytPin] = millis();
      }
      break;
    }
    case enuPinSetupOutputPWM:
    {
      // Change PWM values

      // Does the current value equal the end value?
      if (arrCurrentValue[bytPin][0] != arrEndValue[bytPin][0])
      {
        // Calculate the new value
        if ((millis() - arrStartTime[bytPin][0]) >= arrDuration[bytPin][0])
        {
          arrCurrentValue[bytPin][0] = arrEndValue[bytPin][0];
        }
        else
        {
          arrCurrentValue[bytPin][0] = arrStartValue[bytPin][0] + (((float)(millis() - arrStartTime[bytPin][0]) / arrDuration[bytPin][0]) * (arrEndValue[bytPin][0] - arrStartValue[bytPin][0]));
        }

        // Set the PWM output
        analogWrite(bytPin, arrCurrentValue[bytPin][0]);
      }
      break;
    }
    case enuPinSetupMeccanoid:
    {
      // Change servo values
      byte bytValue;
      bool booUpdate = false;

      for (byte bytPosition = 0; bytPosition < 4; bytPosition++)
      {

        Serial.println("Checking for update " + String(bytPin) + " - " + String(bytPosition));
        if (arrMeccanoidType[bytPin][bytPosition] == 1)
        {
          Serial.println("Servo detected " + String(bytPin) + " - " + String(bytPosition));

          // Does the current value equal the end value?
          if (arrCurrentValue[bytPin][bytPosition] != arrEndValue[bytPin][bytPosition])
          {
            Serial.println("Update found for servo " + String(bytPin) + " - " + String(bytPosition));

            // Calculate the new value
            if ((millis() - arrStartTime[bytPin][bytPosition]) >= arrDuration[bytPin][bytPosition])
            {
              bytValue = arrEndValue[bytPin][bytPosition];
            }
            else
            {
              bytValue = arrStartValue[bytPin][bytPosition] + (((float)(millis() - arrStartTime[bytPin][bytPosition]) / arrDuration[bytPin][bytPosition]) * (arrEndValue[bytPin][bytPosition] - arrStartValue[bytPin][bytPosition]));
            }

            // Set the servo value, if there has been a change
            if (bytValue != arrCurrentValue[bytPin][bytPosition])
            {
              arrCurrentValue[bytPin][bytPosition] = bytValue;
              arrMeccanoidData[bytPin][bytPosition] = bytValue;
              booUpdate = true;
            }
            else
            {
              Serial.println("No update for servo " + String(bytPin) + " - " + String(bytPosition));
            }
          }
          else
          {
            Serial.println("Current end val match " + String(arrCurrentValue[bytPin][bytPosition]) + " vs " + String(arrEndValue[bytPin][bytPosition]));
          }
        }
        else
        {
          Serial.println("Not a servo " + String(bytPin) + " - " + String(bytPosition));
        }
      }

      // Only update if there has been a change to at least one servo value
      if (booUpdate)
      {
        meccanoidCommunicate(bytPin, 0);
      }

      break;
    }
    default:
    {
      // Do nothing
    }
    }

    // Iterate through analogue pins
    for (byte bytPin = bytLowestAnalogPin; bytPin <= bytHighestAnalogPin; bytPin++)
    {
      switch (arrAnalogPinSetup[bytPin])
      {
      case enuPinSetupInputLatchingHigh:
      {
        // Check input for high latch trigger
        if (analogRead(bytPin) > 255 && millis() - arrAnalogLatchDebounceStartTime[bytPin] > intLatchDebounceInterval)
        {
          arrAnalogLatchTriggered[bytPin] = true;
          arrAnalogLatchDebounceStartTime[bytPin] = millis();
        }
        break;
      }
      case enuPinSetupInputLatchingLow:
      {
        // Check input for low latch trigger
        if (analogRead(bytPin) < 256 && millis() - arrAnalogLatchDebounceStartTime[bytPin] > intLatchDebounceInterval)
        {
          arrAnalogLatchTriggered[bytPin] = true;
          arrAnalogLatchDebounceStartTime[bytPin] = millis();
        }
        break;
      }
      default:
      {
        // Do nothing
      }
      }
    }
  }
}

void reset()
{
  Serial.println("Resetting " + String(amountOfPins) + " pins");
  // Reset digital pins
  for (int pin = 0; pin < amountOfPins; pin++)
  {
    byte bytPin = relevantDigitalPins[pin];
    Serial.println("Resetting digital pin (" + String(pin) + ")" + String(bytPin));
    // Pins default to unused inputs
    arrDigitalPinSetup[bytPin] = enuPinSetupUnused;
    pinMode(bytPin, INPUT);

    // Reset latch
    arrDigitalLatchTriggered[bytPin] = false;
    arrDigitalLatchDebounceStartTime[bytPin] = 0;

    // Reset state
    for (byte bytPosition = 0; bytPosition <= 3; bytPosition++)
    {
      arrCurrentValue[bytPin][bytPosition] = 0;
      arrStartValue[bytPin][bytPosition] = 0;
      arrEndValue[bytPin][bytPosition] = 0;
      arrDuration[bytPin][bytPosition] = 0;
      arrStartTime[bytPin][bytPosition] = 0;
    }

    Serial.println("Finished resetting digital pin " + String(bytPin));
  }
  Serial.println("Finished going through ALL pins!");
}

void meccanoidInitialise(byte bytPin)
{
  Serial.println("Initializing digital pin " + String(bytPin));
  byte bytInput;
  byte bytPosition = 0;
  byte bytTimeout = 0;
  bool booAllPositionsDiscovered;

  // Is this pin already setup for Meccanoid use?
  if (arrDigitalPinSetup[bytPin] == enuPinSetupMeccanoid)
  {
    Serial.println(F("OK"));
    return;
  }

  // Send reset
  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++)
  {
    arrMeccanoidData[bytPin][bytLoopPosition] = 0xFD;
    meccanoidCommunicate(bytPin, bytLoopPosition);
  }

  // Set positions and types as unassigned
  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++)
  {
    arrMeccanoidData[bytPin][bytLoopPosition] = 0xFE;
    arrMeccanoidType[bytPin][bytLoopPosition] = 255;
  }

  do
  {
    bytInput = meccanoidCommunicate(bytPin, bytPosition);

    // If 0xFE is received, then the module exists so get its type
    if (bytInput == 0xFE)
    {
      arrMeccanoidData[bytPin][bytPosition] = 0xFC;
    }

    // If 0x01 is received, the module is a servo, so change its colour to black
    if (bytInput == 0x01 && arrMeccanoidType[bytPin][bytPosition] == 255)
    {
      Serial.println("Detected a servo");
      arrMeccanoidData[bytPin][bytPosition] = 0xF0;
      arrMeccanoidType[bytPin][bytPosition] = 1;
    }

    // If 0x02 is received, the module is an LED
    if (bytInput == 0x02 && arrMeccanoidType[bytPin][bytPosition] == 255)
    {
      Serial.println("Detected a LED");
      arrMeccanoidData[bytPin][bytPosition] = 0x00;
      arrMeccanoidType[bytPin][bytPosition] = 2;
    }

    // If 0x00 is received, there is no module at this or higher positions
    if (bytInput == 0x00 && bytPosition > 0)
    {
      if (arrMeccanoidType[bytPin][bytPosition - 1] != 255)
      {
        arrMeccanoidData[bytPin][bytPosition] = 0xFE;
        arrMeccanoidType[bytPin][bytPosition] = 0;
      }
    }

    // See if all positions have been discovered
    booAllPositionsDiscovered = true;
    for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++)
    {
      if (arrMeccanoidType[bytPin][bytLoopPosition] == 255)
      {
        booAllPositionsDiscovered = false;
      }
    }

    // Move to the next position
    bytPosition++;
    if (bytPosition == 4)
    {
      bytPosition = 0;
    }

    bytTimeout++;
  } while (!booAllPositionsDiscovered && bytTimeout != 0);

  if (bytTimeout == 0)
  {
    // This pin is now unused
    arrDigitalPinSetup[bytPin] = enuPinSetupUnused;

    Serial.println(F("Timed out"));
  }
  else
  {
    for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++)
    {
      if (arrMeccanoidType[bytPin][bytLoopPosition] == 1)
      {
        Serial.println("Writing to servo " + String(bytPin) + " - " + String(bytLoopPosition));
        // Set servo LED to black
        arrMeccanoidData[bytPin][bytLoopPosition] = 0xF0;

        // Get the servo position
        arrCurrentValue[bytPin][bytLoopPosition] = meccanoidGetServoMotorPosition(bytPin, bytLoopPosition);
        Serial.println("Servo position: " + String(arrCurrentValue[bytPin][bytLoopPosition]));
        arrEndValue[bytPin][bytLoopPosition] = arrCurrentValue[bytPin][bytLoopPosition];
      }

      if (arrMeccanoidType[bytPin][bytLoopPosition] == 2)
      {
        Serial.println("Writing to LED " + String(bytPin) + " - " + String(bytLoopPosition));
        // Set LED to black
        arrMeccanoidData[bytPin][bytLoopPosition] = 0x00;
        meccanoidCommunicate(bytPin, bytLoopPosition);
        arrMeccanoidData[bytPin][bytLoopPosition] = 0x40;
        meccanoidCommunicate(bytPin, bytLoopPosition);
      }
    }

    // This pin is now a Meccanoid pin
    arrDigitalPinSetup[bytPin] = enuPinSetupMeccanoid;

    Serial.println("Init complete for digital pin " + String(bytPin));
  }

  return;
}

byte meccanoidCommunicate(byte bytPin, byte bytPosition)
{
  unsigned int intCheckSum;
  byte bytCheckSum;
  byte bytInput;

  // Send header
  meccanoidSendByte(bytPin, 0xFF);

  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++)
  {
    // Send 4 data bytes
    meccanoidSendByte(bytPin, arrMeccanoidData[bytPin][bytLoopPosition]);
  }

  // Calculate checksum
  intCheckSum = arrMeccanoidData[bytPin][0] + arrMeccanoidData[bytPin][1] + arrMeccanoidData[bytPin][2] + arrMeccanoidData[bytPin][3]; // Ignore overflow
  intCheckSum = intCheckSum + (intCheckSum >> 8);                                                                                      // Right shift 8 places
  intCheckSum = intCheckSum + (intCheckSum << 4);                                                                                      // Left shift 4 places
  intCheckSum = intCheckSum & 0xF0;                                                                                                    // Mask off top nibble
  bytCheckSum = intCheckSum | bytPosition;

  // Send checksum
  meccanoidSendByte(bytPin, bytCheckSum);

  // Receive input
  bytInput = meccanoidReceiveByte(bytPin);

  delay(10);

  return bytInput;
}

void meccanoidSendByte(byte bytPin, byte bytData)
{
  const unsigned int intMeccanoidBitDelay = 417;

  pinMode(bytPin, OUTPUT);
  digitalWrite(bytPin, LOW);
  delayMicroseconds(intMeccanoidBitDelay); // Start bit - 417us LOW

  for (byte bytMask = 00000001; bytMask > 0; bytMask <<= 1)
  { // Iterate through bit mask
    if (bytData & bytMask)
    { // If bitwise AND resolves to true

      digitalWrite(bytPin, HIGH); // Send 1
    }
    else
    { // If bitwise AND resolves to false

      digitalWrite(bytPin, LOW); // Send 0
    }
    delayMicroseconds(intMeccanoidBitDelay); // Delay
  }

  digitalWrite(bytPin, HIGH);
  delayMicroseconds(intMeccanoidBitDelay); // Stop bit - 417us HIGH

  digitalWrite(bytPin, HIGH);
  delayMicroseconds(intMeccanoidBitDelay); // Stop bit - 417us HIGH
}

byte meccanoidReceiveByte(byte bytPin)
{
  byte bytTemp;
  bytTemp = 0;

  pinMode(bytPin, INPUT);

  delay(1.5);

  // Iterate through bit mask
  for (byte bytMask = 00000001; bytMask > 0; bytMask <<= 1)
  {
    if (pulseIn(bytPin, HIGH, 2500) > 400)
    {
      bytTemp = bytTemp | bytMask;
    }
  }

  return bytTemp;
}

byte meccanoidGetServoMotorPosition(byte bytPin, byte bytPosition)
{
  // Get the servo position - keep getting returned position until two readings match and are non-zero
  byte bytReturnedValue = 0;
  byte bytLastReturnedValue = 0;

  do
  {
    bytLastReturnedValue = bytReturnedValue;
    bytReturnedValue = meccanoidCommunicate(bytPin, bytPosition);
  } while (bytReturnedValue == 0 || bytReturnedValue != bytLastReturnedValue);

  return bytReturnedValue;
}

///////////////////
/////HELPERS
///////////////////

std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;

  while (getline(ss, item, delim))
  {
    result.push_back(item);
  }

  return result;
}

void handleCommand(String rawCommand)
{
  dataReceived = false;
  // In order:
  // pin
  // part in array
  // all next are values for the item
  Serial.println("Received command: " + String(rawCommand));
  if (rawCommand.indexOf(";") > 0)
  {
    std::vector<std::string> command = split(rawCommand.c_str(), ';');

    // map pin from string to byte
    byte pin = atoi(command[1].c_str());
    byte partInArray = atoi(command[2].c_str());

    if (strcmp(command[0].c_str(), "setled") == 0)
    {
      int red = atoi(command[3].c_str());
      int green = atoi(command[4].c_str());
      int blue = atoi(command[5].c_str());
      int fade = atoi(command[6].c_str());

      setLed(pin, partInArray, red, green, blue, fade);
    }
    else if (strcmp(command[0].c_str(), "setservopos") == 0)
    {
      int angle = atoi(command[3].c_str());
      setMotorPosition(pin, partInArray, angle);
    }
    else if (strcmp(command[0].c_str(), "setservoled") == 0)
    {
      int color = atoi(command[3].c_str());
      // 1-7
      setServoLed(pin, partInArray, color);
    }
    iterateAllPins();
  }
  else
  {
    Serial.println("Not a valid command");
  }
}