#include <Arduino.h>

byte arrMeccanoidData[4];
byte arrMeccanoidType[4];

byte arrCurrentValue[4];
byte arrStartValue[4];
byte arrEndValue[4];

void setMotorPosition(int angle);
byte meccanoidReceiveByte(byte bytPin);
byte meccanoidCommunicate(byte bytPin, byte bytPosition);
void meccanoidSendByte(byte bytPin, byte bytData);
byte meccanoidGetServoMotorPosition(byte bytPin, byte bytPosition);
void meccanoidInitialise(byte bytPin);
void motorAndSuch(int steps);

void setup()
{
  // Initialize serial port
  Serial.begin(9600);
  delay(1000);
  Serial.println("Init");

  // Send reset
  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++)
  {
    arrMeccanoidData[bytLoopPosition] = 0xFD;
    meccanoidCommunicate(PIN2, bytLoopPosition);
  }
  meccanoidInitialise(PIN2);
  pinMode(PIN2, OUTPUT);
  Serial.println("Init finished");
}

void loop()
{
  Serial.println("OK");

  setMotorPosition(random(24, 233));
  Serial.println("Set motor position, waiting 5");
  delay(5000);
}

void setMotorPosition(int angle)
{
  Serial.println("Angling");
  Serial.println((int)angle);
  byte bytPin = PIN_A0;
  // Get the servo position - keep getting returned position until two readings match and are non-zero

  arrMeccanoidData[0] = angle;
  arrMeccanoidType[0] = 1;
  // arrMeccanoidData[1] = NULL;
  // arrMeccanoidData[2] = NULL;
  // arrMeccanoidData[3] = NULL;
  // arrMeccanoidType[1] = NULL;
  // arrMeccanoidType[2] = NULL;
  // arrMeccanoidType[3] = NULL;

  meccanoidCommunicate(bytPin, 0);
  // byte bytReturnedValue = 0;
  // byte bytLastReturnedValue = 0;

  // do
  // {
  //   bytLastReturnedValue = bytReturnedValue;
  //   bytReturnedValue = meccanoidCommunicate(bytPin, angle);
  // } while (bytReturnedValue == 0 || bytReturnedValue != bytLastReturnedValue);
}

byte meccanoidCommunicate(byte bytPin, byte bytPosition)
{
  unsigned int intCheckSum;
  byte bytCheckSum;
  byte bytInput;

  // Send header
  meccanoidSendByte(bytPin, 0xFF);

  Serial.println("Data:");
  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++)
  {
    // Send 4 data bytes
    meccanoidSendByte(bytPin, arrMeccanoidData[bytLoopPosition]);
    Serial.println(arrMeccanoidData[bytLoopPosition]);
  }

  // Calculate checksum
  intCheckSum = arrMeccanoidData[0] + arrMeccanoidData[1] + arrMeccanoidData[2] + arrMeccanoidData[3]; // Ignore overflow
  intCheckSum = intCheckSum + (intCheckSum >> 8);                                                      // Right shift 8 places
  intCheckSum = intCheckSum + (intCheckSum << 4);                                                      // Left shift 4 places
  intCheckSum = intCheckSum & 0xF0;                                                                    // Mask off top nibble
  bytCheckSum = intCheckSum | bytPosition;

  // Send checksum
  meccanoidSendByte(bytPin, bytCheckSum);

  // Receive input
  bytInput = meccanoidReceiveByte(bytPin);

  delay(10);

  return bytInput;
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
    {                            // If bitwise AND resolves to false
      digitalWrite(bytPin, LOW); // Send 0
    }
    delayMicroseconds(intMeccanoidBitDelay); // Delay
  }

  digitalWrite(bytPin, HIGH);
  delayMicroseconds(intMeccanoidBitDelay); // Stop bit - 417us HIGH

  digitalWrite(bytPin, HIGH);
  delayMicroseconds(intMeccanoidBitDelay); // Stop bit - 417us HIGH
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

void motorAndSuch(int steps)
{
  // // Determine step direction, number of steps and step interval
  // int intNumberOfSteps = steps;
  // bool booStepDirectionClockwise = (intNumberOfSteps > 0);
  // intNumberOfSteps = abs(intNumberOfSteps);

  // for (int intStep = 0; intStep < intNumberOfSteps; intStep++)
  // {
  //   // Determine the next step in the pattern
  //   if (booStepDirectionClockwise)
  //   {
  //     switch (arrEndValue[0])
  //     {
  //     case B00000110:
  //       arrEndValue[0] = B00000101;
  //       break;
  //     case B00000101:
  //       arrEndValue[0] = B00001001;
  //       break;
  //     case B00001001:
  //       arrEndValue[0] = B00001010;
  //       break;
  //     default:
  //       arrEndValue[0] = B00000110;
  //     }
  //   }
  //   else
  //   {
  //     switch (arrEndValue[0])
  //     {
  //     case B00000110:
  //       arrEndValue[0] = B00001010;
  //       break;
  //     case B00000101:
  //       arrEndValue[0] = B00000110;
  //       break;
  //     case B00001001:
  //       arrEndValue[0] = B00000101;
  //       break;
  //     default:
  //       arrEndValue[0] = B00001001;
  //     }
  //   }
  //   // Set the outputs to high or low depending on their position in the pattern
  //   digitalWrite(PIN2, arrEndValue[0] & B00001000);
  //   digitalWrite(PIN2, arrEndValue[0] & B00000100);
  //   digitalWrite(PIN2, arrEndValue[0] & B00000010);
  //   digitalWrite(PIN2, arrEndValue[0] & B00000001);

  //   // Wait until it is time for the next step
  //   delay(10);
  // }
  // pinMode(PIN2, OUTPUT);
  // pinMode(PIN2, HIGH);
}