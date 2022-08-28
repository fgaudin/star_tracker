#include <Arduino.h>
#include <AccelStepper.h>

const int dirPin = 2;
const int stepPin = 3;
const int releasePin = 4;
const int directionPin = 5;
const int enablePin = 6;
const int slowerPin = 7;
const int fasterPin = 8;
int speed = 200;
const int maxSpeed = 500;
const int speedIncr = 10;

int directionButtonState;
int lastDirectionButtonState = HIGH;
unsigned long lastDirectionDebounceTime = 0;

int enableButtonState;
int lastEnableButtonState = HIGH;
unsigned long lastEnableDebounceTime = 0;

int slowerButtonState;
int lastSlowerButtonState = HIGH;
unsigned long lastSlowerDebounceTime = 0;

int fasterButtonState;
int lastFasterButtonState = HIGH;
unsigned long lastFasterDebounceTime = 0;

unsigned long debounceDelay = 50;

#define motorInterfaceType 1

AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup()
{
  Serial.begin(9600);
  pinMode(releasePin, OUTPUT);
  digitalWrite(releasePin, HIGH);
  pinMode(directionPin, INPUT_PULLUP);
  pinMode(enablePin, INPUT_PULLUP);
  pinMode(slowerPin, INPUT_PULLUP);
  pinMode(fasterPin, INPUT_PULLUP);
  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setSpeed(0);
}

void checkDirection()
{
  int direction = digitalRead(directionPin);

  if (direction != lastDirectionButtonState)
  {
    lastDirectionDebounceTime = millis();
  }

  if ((millis() - lastDirectionDebounceTime) > debounceDelay)
  {
    if (direction != directionButtonState)
    {
      directionButtonState = direction;

      if (directionButtonState == LOW)
      {
        Serial.println("direction button pressed");
        myStepper.setSpeed(-myStepper.speed());
      }
    }
  }

  lastDirectionButtonState = direction;
}

void checkEnable()
{
  int enable = digitalRead(enablePin);

  if (enable != lastEnableButtonState)
  {
    lastEnableDebounceTime = millis();
  }

  if ((millis() - lastEnableDebounceTime) > debounceDelay)
  {
    if (enable != enableButtonState)
    {
      enableButtonState = enable;

      if (enableButtonState == LOW)
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
          digitalWrite(releasePin, LOW);
          delay(2);
          myStepper.enableOutputs();
          myStepper.setSpeed(speed);
        }
      }
    }
  }

  lastEnableButtonState = enable;
}

void checkSlower()
{
  int slower = digitalRead(slowerPin);

  if (slower != lastSlowerButtonState)
  {
    lastSlowerDebounceTime = millis();
  }

  if ((millis() - lastSlowerDebounceTime) > debounceDelay)
  {
    if (slower != slowerButtonState)
    {
      slowerButtonState = slower;

      if (slowerButtonState == LOW)
      {
        Serial.println("slower button pressed");
        speed = max(speed - speedIncr, 0);
        myStepper.setSpeed(speed);
      }
    }
  }

  lastSlowerButtonState = slower;
}

void checkFaster()
{
  int faster = digitalRead(fasterPin);

  if (faster != lastFasterButtonState)
  {
    lastFasterDebounceTime = millis();
  }

  if ((millis() - lastFasterDebounceTime) > debounceDelay)
  {
    if (faster != fasterButtonState)
    {
      fasterButtonState = faster;

      if (fasterButtonState == LOW)
      {
        Serial.println("faster button pressed");
        speed = min(speed + speedIncr, maxSpeed);
        myStepper.setSpeed(speed);
      }
    }
  }

  lastFasterButtonState = faster;
}

void loop()
{
  checkDirection();
  checkEnable();
  checkSlower();
  checkFaster();

  myStepper.runSpeed();
}