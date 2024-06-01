#include <Arduino.h>
#include "Moonlite.h"
#include "StepperControl.h"

#define LED_PIN   PA0

// Stepper control constants
#define DIRECTION PA1
#define STEP      PA2
#define SLEEP     PA3
#define RESET     PA4
#define STEP_M3   PA5
#define STEP_M2   PA6
#define STEP_M1   PA7
#define ENABLE    PB0

//const int temperatureSensorPin = 3;

unsigned long timestamp;

//LM335 TemperatureSensor(temperatureSensorPin);
StepperControl Motor(STEP, DIRECTION,
                     STEP_M1, STEP_M2, STEP_M3,
                     ENABLE, SLEEP, RESET);
Moonlite SerialProtocol;

float temp = 0;
long pos = 0;
bool pageIsRefreshing = false;

void processCommand()
{
  switch (SerialProtocol.getCommand().commandID)
  {
    case ML_C:
      // Initiate temperature convertion
      // Not implemented
      break;
    case ML_FG:
      // Goto target position
      Motor.goToTargetPosition();
      break;
    case ML_FQ:
      // Motor stop movement
      Motor.stopMovement();
      break;
    case ML_GB:
      // Set the Red Led backligth value
      // Dump value necessary to run the official moonlite software
      SerialProtocol.setAnswer(2, 0x00);
      break;
    case ML_GC:
      // Return the temperature coefficient
      SerialProtocol.setAnswer(2, (long)Motor.getTemperatureCompensationCoefficient());
      break;
    case ML_GD:
      // Return the current motor speed
      switch (Motor.getSpeed())
      {
        case 500:
          SerialProtocol.setAnswer(2, (long)0x20);
          break;
        case 1000:
          SerialProtocol.setAnswer(2, (long)0x10);
          break;
        case 3000:
          SerialProtocol.setAnswer(2, (long)0x8);
          break;
        case 5000:
          SerialProtocol.setAnswer(2, (long)0x4);
          break;
        case 7000:
          SerialProtocol.setAnswer(2, (long)0x2);
          break;
        default:
          SerialProtocol.setAnswer(2, (long)0x20);
          break;
      }
      break;
    case ML_GH:
      // Return the current stepping mode (half or full step)
      SerialProtocol.setAnswer(2, (long)(Motor.getStepMode() == SC_32TH_STEP ? 0xFF : 0x00));
      break;
    case ML_GI:
      // get if the motor is moving or not
      SerialProtocol.setAnswer(2, (long)(Motor.isInMove() ? 0x01 : 0x00));
      break;
    case ML_GN:
      // Get the target position
      SerialProtocol.setAnswer(4, (long)(Motor.getTargetPosition()));
      break;
    case ML_GP:
      // Return the current position
      SerialProtocol.setAnswer(4, (long)(Motor.getCurrentPosition()));
      break;
    case ML_GT:
      // Return the temperature
      //SerialProtocol.setAnswer(4, (long)((TemperatureSensor.getTemperature() * 2)));
      SerialProtocol.setAnswer(4, (long)(20 * 2));
      break;
    case ML_GV:
      // Get the version of the firmware
      SerialProtocol.setAnswer(2, (long)(0x01));
      break;
    case ML_SC:
      // Set the temperature coefficient
      Motor.setTemperatureCompensationCoefficient(SerialProtocol.getCommand().parameter);
      break;
    case ML_SD:
      // Set the motor speed
      switch (SerialProtocol.getCommand().parameter)
      {
        case 0x02:
          Motor.setSpeed(7000);
          break;
        case 0x04:
          Motor.setSpeed(5000);
          break;
        case 0x08:
          Motor.setSpeed(3000);
          break;
        case 0x10:
          Motor.setSpeed(1000);
          break;
        case 0x20:
          Motor.setSpeed(500);
          break;
        default:
          break;
      }
      break;
    case ML_SF:
      // Set the stepping mode to full step
      Motor.setStepMode(SC_16TH_STEP);
      if (Motor.getSpeed() >= 6000)
      {
        Motor.setSpeed(6000);
      }
      break;
    case ML_SH:
      // Set the stepping mode to half step
      Motor.setStepMode(SC_32TH_STEP);
      break;
    case ML_SN:
      // Set the target position
      Motor.setTargetPosition(SerialProtocol.getCommand().parameter);
      break;
    case ML_SP:
      // Set the current motor position
      Motor.setCurrentPosition(SerialProtocol.getCommand().parameter);
      break;
    case ML_PLUS:
      // Activate temperature compensation focusing
      Motor.enableTemperatureCompensation();
      break;
    case ML_MINUS:
      // Disable temperature compensation focusing
      Motor.disableTemperatureCompensation();
      break;
    case ML_PO:
      // Temperature calibration
      //TemperatureSensor.setCompensationValue(SerialProtocol.getCommand().parameter / 2.0);
      break;
    default:
      break;
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PC13, HIGH);
  SerialProtocol.init(9600);

  // Set the motor speed to a valid value for Moonlite
  Motor.setSpeed(7000);
  Motor.setStepMode(SC_32TH_STEP);
  Motor.setMoveMode(SC_MOVEMODE_SMOOTH);

  timestamp = millis();
}

void loop()
{
  if (!Motor.isInMove())
  {
    digitalWrite(LED_PIN, HIGH);
    //TemperatureSensor.Manage();
    if (Motor.isTemperatureCompensationEnabled() && ((millis() - timestamp) > 30000))
    {
     // Motor.setCurrentTemperature(TemperatureSensor.getTemperature());
      Motor.setCurrentTemperature(20);
      Motor.compensateTemperature();
      timestamp = millis();
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  Motor.Manage();
  SerialProtocol.Manage();

  if (SerialProtocol.isNewCommandAvailable())
  {
    processCommand();
  }
}
