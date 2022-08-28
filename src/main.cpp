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

int speed = 200;
const int maxSpeed = 800;
const int speedIncr = 10;

int buttonState[] = {LOW, LOW, LOW, LOW};
int lastButtonState[] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[] = {0, 0, 0, 0};

unsigned long debounceDelay = 50;

#define motorInterfaceType 1

AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup()
{
  Serial.begin(9600);

  EEPROM.get(0, speed);
  Serial.print("loaded speed: ");
  Serial.println(speed);

  pinMode(releasePin, OUTPUT);
  digitalWrite(releasePin, HIGH);
  pinMode(directionPin, INPUT_PULLUP);
  pinMode(enablePin, INPUT_PULLUP);
  pinMode(slowerPin, INPUT_PULLUP);
  pinMode(fasterPin, INPUT_PULLUP);
  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setSpeed(0);
}

boolean buttonPressed(int pin)
{
  const int idx = pin - directionPin;

  int value = digitalRead(pin);
  bool pressed = false;

  if (value != lastButtonState[idx])
  {
    lastDebounceTime[idx] = millis();
  }

  if ((millis() - lastDebounceTime[idx]) > debounceDelay)
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
  if (buttonPressed(directionPin))
  {
    Serial.println("direction button pressed");
    speed = -speed;
    myStepper.setSpeed(speed);
    EEPROM.put(0, speed);
    Serial.print("saved: ");
    Serial.println(speed);
  }

  if (buttonPressed(enablePin))
  {
    Serial.println("enable button pressed");
    if (myStepper.speed() != 0.0)
    {
      Serial.println("motor running: stopping");
      myStepper.setSpeed(0);
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
    EEPROM.put(0, speed);
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
    EEPROM.put(0, speed);
    Serial.print("saved: ");
    Serial.println(speed);
  }

  myStepper.runSpeed();
}