#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>

const int dirPin = 2;
const int stepPin = 3;
const int releasePin = 4;

const int directionPin = 5;
const int enablePin = 6;
const int slowerPin = 7;
const int fasterPin = 8;
const int modePin = 9;

int speed = 200;
const int maxSpeed = 800;
const int speedIncr = 10;

int buttonState[] = {LOW, LOW, LOW, LOW};
int lastButtonState[] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastButtonDebounceTime[] = {0, 0, 0, 0};

int lastSwitchState = HIGH;
unsigned long lastModeDebounceTime = 0;
char trackingMode = '\0';

unsigned long debounceDelay = 50;

#define motorInterfaceType 1

AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

int modeAddress()
{
  if (trackingMode == 'm')
  {
    return sizeof(speed);
  }
  else
  {
    return 0;
  }
}

void checkMode()
{
  int value = digitalRead(modePin);

  if (value != lastSwitchState)
  {
    lastModeDebounceTime = millis();
  }

  if ((millis() - lastModeDebounceTime) > debounceDelay)
  {
    bool reloadSpeed = false;
    if (value && trackingMode != 'm')
    {
      trackingMode = 'm'; // moon
      reloadSpeed = true;
    }
    else if (!value && trackingMode != 's')
    {
      trackingMode = 's'; // star
      reloadSpeed = true;
    }

    if (reloadSpeed)
    {
      Serial.print("mode: ");
      Serial.println(trackingMode);
      EEPROM.get(modeAddress(), speed);
      myStepper.setSpeed(speed);
      Serial.print("speed: ");
      Serial.println(speed);
    }
  }

  lastSwitchState = value;
}

void setup()
{
  Serial.begin(9600);

  pinMode(releasePin, OUTPUT);
  digitalWrite(releasePin, HIGH);
  pinMode(directionPin, INPUT_PULLUP);
  pinMode(enablePin, INPUT_PULLUP);
  pinMode(slowerPin, INPUT_PULLUP);
  pinMode(fasterPin, INPUT_PULLUP);
  pinMode(modePin, INPUT_PULLUP);
  myStepper.setMaxSpeed(maxSpeed);

  checkMode();
}

boolean buttonPressed(int pin)
{
  const int idx = pin - directionPin;

  int value = digitalRead(pin);
  bool pressed = false;

  if (value != lastButtonState[idx])
  {
    lastButtonDebounceTime[idx] = millis();
  }

  if ((millis() - lastButtonDebounceTime[idx]) > debounceDelay)
  {
    if (value != buttonState[idx])
    {
      buttonState[idx] = value;

      if (buttonState[idx] == LOW)
      {
        pressed = true;
      }
    }
  }

  lastButtonState[idx] = value;

  return pressed;
}

void loop()
{
  checkMode();

  if (buttonPressed(directionPin))
  {
    Serial.println("direction button pressed");
    speed = -speed;
    myStepper.setSpeed(speed);
    EEPROM.put(modeAddress(), speed);
    Serial.print("saved: ");
    Serial.println(speed);
  }

  if (buttonPressed(enablePin))
  {
    Serial.println("enable button pressed");
    if (!digitalRead(releasePin))
    {
      Serial.println("motor running: stopping");
      myStepper.disableOutputs();
      digitalWrite(releasePin, HIGH);
    }
    else
    {
      Serial.println("motor stopped: starting");
      Serial.print("speed: ");
      Serial.println(myStepper.speed());
      digitalWrite(releasePin, LOW);
      delay(2);
      myStepper.enableOutputs();
      myStepper.setSpeed(speed);
    }
  }

  if (buttonPressed(slowerPin))
  {
    Serial.println("slower button pressed");
    if (myStepper.speed() >= 0)
    {
      speed = max(speed - speedIncr, 0);
    }
    else
    {
      speed = min(speed + speedIncr, 0);
    }

    myStepper.setSpeed(speed);
    EEPROM.put(modeAddress(), speed);
    Serial.print("saved: ");
    Serial.println(speed);
  }

  if (buttonPressed(fasterPin))
  {
    Serial.println("faster button pressed");
    if (myStepper.speed() >= 0)
    {
      speed = min(speed + speedIncr, maxSpeed);
    }
    else
    {
      speed = max(speed - speedIncr, -maxSpeed);
    }
    myStepper.setSpeed(speed);
    EEPROM.put(modeAddress(), speed);
    Serial.print("saved: ");
    Serial.println(speed);
  }

  myStepper.runSpeed();
}