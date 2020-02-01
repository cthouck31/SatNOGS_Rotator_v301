#include <Arduino.h>

#include "AccelStepper.h"
#include "Easycomm.h"
#include "Endstop.h"
#include "Rotator_Pins.h"

// Status Indicators.
#define GLED  (12)
#define RLED  (13)

// Easycomm Controller.
#define SERIAL_BAUD_RATE  (115200)
Easycomm_Ctrl comms;

// Rotator.
// Stepper motors.
#define GEAR_RATIO        (108/4)
#define MAX_SPEED         (3200/2)
#define MAX_ACCELERATION  (1600/2)
#define MIN_PULSE_WIDTH   (20)
#define STEPS_PER_REV     (1600/4)
#define HOMING_DELAY      (7500)
#define MIN_AZIM_ANGLE    (0)
#define MAX_AZIM_ANGLE    (180)
#define MIN_ELEV_ANGLE    (0)
#define MAX_ELEV_ANGLE    (360)
// Map pins from 'Rotator_Pins.h'.
#define STEPPER_ENABLE    (MOTOR_EN)
#define AZIM_MOTOR_PIN1   (M1IN1)
#define AZIM_MOTOR_PIN2   (M1IN2)
#define ELEV_MOTOR_PIN1   (M2IN1)
#define ELEV_MOTOR_PIN2   (M2IN2)
// Create steppers.
AccelStepper azimMotor(0x1, AZIM_MOTOR_PIN1, AZIM_MOTOR_PIN2);
//AccelStepper elevMotor(0x1, ELEV_MOTOR_PIN1, ELEV_MOTOR_PIN2);

// End-stops.
#define DFLT_SWITCH_STATE  (HIGH)
#define AZIM_SWITCH        (SW2)
#define ELEV_SWITCH        (SW1)
Endstop azimSwitch;
// Endstop elevSwitch;

//=====================================================================
// Utility Functions.
//=====================================================================
void blinkLed(uint16_t pin, uint16_t numBlinks, uint16_t periodMs);

//=====================================================================
// Rotator Functions.
//=====================================================================
int8_t Rotator_getAzimElev(void *dev, float *azim, float *elev);
int8_t Rotator_getAzim(void *dev, float *azim);
int8_t Rotator_setAzim(void *dev, float azim);
int8_t Rotator_getElev(void *dev, float *elev);
int8_t Rotator_setElev(void *dev, float elev);
int8_t Rotator_getUplinkFreq(void *dev, float *freq);
int8_t Rotator_setUplinkFreq(void *dev, float freq);
int8_t Rotator_getDownlinkFreq(void *dev, float *freq);
int8_t Rotator_setDownlinkFreq(void *dev, float freq);
int8_t Rotator_setMoveUp(void *dev);
int8_t Rotator_setMoveDown(void *dev);
int8_t Rotator_setMoveLeft(void *dev);
int8_t Rotator_setMoveRight(void *dev);
int8_t Rotator_setBreakIn(void *dev, uint32_t msec, uint8_t fwd);
int8_t Rotator_setPark(void *dev);

void
setup()
{
  int8_t err;
  
  // Set-up status indicators.
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
  
  // Initialize Easycomm.
  err = Easycomm_init(&comms);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }
  
  // Provide Easycomm w/ status indicator pins.
  err = Easycomm_setDebugPins(&comms, GLED, RLED);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }
  
  // Set-up serial port.
  err = Easycomm_begin(&comms, SERIAL_BAUD_RATE);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }
  
  // Set the Easycomm context (passed to callback functions).
  err = Easycomm_setContext(&comms, &comms);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }
  
  // Attach rotator callbacks to the Easycomm controller.
  comms.getAzimElev     = Rotator_getAzimElev;
  comms.getAzim         = Rotator_getAzim;
  comms.setAzim         = Rotator_setAzim;
  comms.getElev         = Rotator_getElev;
  comms.setElev         = Rotator_setElev;
  comms.getUplinkFreq   = Rotator_getUplinkFreq;
  comms.setUplinkFreq   = Rotator_setUplinkFreq;
  comms.getDownlinkFreq = Rotator_getDownlinkFreq;
  comms.setDownlinkFreq = Rotator_setDownlinkFreq;
  comms.setMoveUp       = Rotator_setMoveUp;
  comms.setMoveDown     = Rotator_setMoveDown;
  comms.setMoveLeft     = Rotator_setMoveLeft;
  comms.setMoveRight    = Rotator_setMoveRight;
  comms.setBreakIn      = Rotator_setBreakIn;
  comms.setPark         = Rotator_setPark;
  
  // Initialize rotator steppers.
  azimMotor.setEnablePin(STEPPER_ENABLE);
  azimMotor.setPinsInverted(0, 0, 1);
  azimMotor.enableOutputs();
  azimMotor.setMaxSpeed(MAX_SPEED);
  azimMotor.setAcceleration(MAX_ACCELERATION);
  azimMotor.setMinPulseWidth(MIN_PULSE_WIDTH);
  
  // Initialize end-stops.
  err = Endstop_init(&azimSwitch, AZIM_SWITCH, DFLT_SWITCH_STATE);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }
  
  // Park at start up.
  err = Rotator_setPark(NULL);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }
  
  // Blink status indicators.
  blinkLed(GLED, 5, 100);
  
};

void
loop()
{
  uint8_t value;
  Easycomm_run(&comms);
  
  // TODO: Put motor stepping in this loop to prevent blocking and
  // enable continuous tracking.
};


//=====================================================================
// Utility Functions.
//=====================================================================

void
blinkLed(uint16_t pin,
         uint16_t numBlinks,
         uint16_t periodMs)
{
  uint16_t k;
  
  for (k = 0; k < numBlinks; k++)
  {
    digitalWrite(pin, HIGH);
    delay(periodMs/2);
    digitalWrite(pin, LOW);
    delay(periodMs/2);
  }
};

float
step2deg(int32_t x)
{
  return (360.0f * x / ((int32_t)STEPS_PER_REV * (int32_t)GEAR_RATIO));
};

int32_t
deg2step(float x)
{
  return (int32_t)((int32_t)GEAR_RATIO * (int32_t)STEPS_PER_REV * x / 360.0);
};

float
phaseWrap(float x)
{
  while (x < 0.0f)
  {
    x += 360.0f;
  }
  while (x >= 360.0f)
  {
    x -= 360.0f;
  }
  
  return x;
};

//=====================================================================
// Rotator Functions.
//=====================================================================
int8_t
Rotator_getAzimElev(void *dev,
                    float *azim,
                    float *elev)
{
  int8_t err;
  Rotator_getAzim(dev, azim);
  Rotator_getElev(dev, elev);
  
  return 0;
};

int8_t
Rotator_getAzim(void *dev,
                float *azim)
{
  int32_t pos = azimMotor.currentPosition();
  *azim = step2deg(pos);
  
  return 0;
};

int8_t
Rotator_setAzim(void *dev,
                float azim)
{
  uint32_t steps = 0;
  uint32_t time;
  float phase = phaseWrap(azim);
  int32_t pos = deg2step(phase);
  
  azimMotor.moveTo(pos);
  
  while (azimMotor.currentPosition() != pos)
  {
    azimMotor.run();
    steps++;
  }
  
  time = millis();
  while (((millis() - time) < HOMING_DELAY) && steps)
  {
    azimMotor.run();
  }
  
  return 0;
};

int8_t
Rotator_getElev(void *dev,
                float *elev)
{
  *elev = 180;
  
  return 0;
};

int8_t
Rotator_setElev(void *dev,
                float elev)
{
  return 0;
};

int8_t
Rotator_getUplinkFreq(void *dev,
                      float *freq)
{
  *freq = 144000;
  
  return 0;
};

int8_t
Rotator_setUplinkFreq(void *dev,
                      float freq)
{
  return 0;
};

int8_t
Rotator_getDownlinkFreq(void *dev,
                      float *freq)
{
  *freq = 145000;
  
  return 0;
};

int8_t
Rotator_setDownlinkFreq(void *dev,
                      float freq)
{
  return 0;
};

int8_t
Rotator_setBreakIn(void *dev,
                   uint32_t msec,
                   uint8_t fwd)
{
  uint32_t start;
  
  digitalWrite(RLED, HIGH);
  
  start = millis();
  while ((((uint32_t)millis()) - start) < msec)
  {
    if (fwd)
    {
      azimMotor.moveTo(azimMotor.currentPosition()+STEPS_PER_REV*4);
    }
    else
    {
      azimMotor.moveTo(azimMotor.currentPosition()-STEPS_PER_REV*4);
    }
    azimMotor.run();
  }
  
  // Home has been found, break out of loop.
  azimMotor.moveTo(azimMotor.currentPosition());
  
  start = millis();
  while ((((uint32_t)millis()) - start) < HOMING_DELAY)
  {
    azimMotor.run();
  }
  
  digitalWrite(RLED, LOW);
  
  return 0;
};

int8_t
Rotator_setMoveUp(void *dev)
{
  return 0;
};

int8_t
Rotator_setMoveDown(void *dev)
{
  return 0;
};

int8_t
Rotator_setMoveLeft(void *dev)
{
  uint8_t value;
  float currDeg = step2deg(azimMotor.currentPosition());
  float nextDeg = currDeg - 0.5f;
  nextDeg       = phaseWrap(nextDeg);
  int32_t pos   = deg2step(nextDeg);
  
  azimMotor.moveTo(pos);
  
  while (azimMotor.currentPosition() != pos)
  {
    value = Endstop_read(&azimSwitch);
    digitalWrite(RLED, value);
    azimMotor.run();
  }
  
  return 0;
};

int8_t
Rotator_setMoveRight(void *dev)
{
  uint8_t value;
  float currDeg = step2deg(azimMotor.currentPosition());
  float nextDeg = currDeg + 0.5f;
  nextDeg       = phaseWrap(nextDeg);
  int32_t pos   = deg2step(nextDeg);
  
  azimMotor.moveTo(pos);
  
  while (azimMotor.currentPosition() != pos)
  {
    value = Endstop_read(&azimSwitch);
    digitalWrite(RLED, value);
    azimMotor.run();
  }
  
  return 0;
};

int8_t
Rotator_setPark(void *dev)
{
  uint32_t steps = 0;
  uint32_t time;
  uint8_t value;
  uint8_t isAzimHome = 0;
  float currDeg = step2deg(azimMotor.currentPosition());
  // 4 revolutions.
  int32_t pos   = deg2step(currDeg+4*360);
  
  azimMotor.moveTo(pos);

  Easycomm_writePacket(&comms, float2String(currDeg, 3));  
  Easycomm_writePacket(&comms, String(pos));
  
  while (!isAzimHome)
  {
    value = Endstop_read(&azimSwitch);
    digitalWrite(RLED, value);
    if (value && !isAzimHome)
    {
      azimMotor.moveTo(azimMotor.currentPosition());
      isAzimHome = 1;
      break;
    }
    azimMotor.run();
    steps++;
  }
  
  time = millis();
  while (((millis() - time) < HOMING_DELAY) && steps)
  {
    azimMotor.run();
  }
  
  digitalWrite(RLED, LOW);
  
  azimMotor.setCurrentPosition(0);
  
  return 0;
};
