#include <Arduino.h>

#include "AccelStepper.h"
#include "Easycomm.h"
#include "Endstop.h"
#include "Rotator_Pins.h"
#include "Crc.h"

// Status Indicators.
#define GLED  (12)
#define RLED  (13)


// Version string.
static const char*
ROTATOR_VERSION = "SatNOGS-v3.0.1-Stepper-r0.0.1";

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
#define BREAKIN_STEP      (45)
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
int8_t Rotator_setZero(void *dev);
int8_t Rotator_setPark(void *dev);
int8_t Rotator_getVersion(void *dev, const char **vers);

//=====================================================================
// Rotator State Machine.
//=====================================================================
typedef struct Rotator_State
{
  /** @brief Current state. */
  Easycomm_Status state;
  
  /** @brief Set azimuth and elevation. */
  float azim;
  float elev;
  
  /** @brief Homing flags. */
  uint8_t isAzimHome;
  uint8_t isElevHome;
  
  /** @brief Break-in state. */
  uint32_t breakinStart;
  uint32_t breakinTimeout;
  uint8_t  breakinDir;
  
  /** @brief Error handling. */
  Easycomm_Error error;
  /** @brief Timeout before error returns to park (in seconds). */
  uint32_t errorTimeout;
  
} Rotator_State;

Rotator_State rotator;


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
  comms.setZero         = Rotator_setZero;
  comms.setPark         = Rotator_setPark;
  comms.getVersion      = Rotator_getVersion;
  
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
  
  // Initialize rotator state machine.
  memset(&rotator, 0, sizeof(rotator));
  rotator.state        = EASYCOMM_STATUS_IDLE;
  rotator.error        = EASYCOMM_ERROR_NONE;
  rotator.errorTimeout = 3; // seconds.
  rotator.isAzimHome   = 0;
  rotator.isElevHome   = 0;
  
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
  int8_t err;
  uint8_t azimEnd, elevEnd;
  
  Easycomm_run(&comms);

  switch (rotator.state)
  {
    case EASYCOMM_STATUS_MOVING:
      {
        // Read end-stops.
        azimEnd = Endstop_read(&azimSwitch);
        // Check if azimuth end-stop has been reached.
        rotator.isAzimHome = azimEnd;
        if (rotator.isAzimHome)
        {
          // Set the park azimuth.
          rotator.azim = phaseWrap(step2degree(azimMotor.currentPosition()));
          azimMotor.moveTo(azimMotor.currentPosition());
          rotator.state = EASYCOMM_STATUS_IDLE;
        }
      }
      // No break.
      
    case EASYCOMM_STATUS_IDLE:
      {
      }
      // No break.
      
    case EASYCOMM_STATUS_POINTING:
      {
        digitalWrite(GLED, HIGH);
        
        // Run the motors.
        azimMotor.moveTo(deg2step(rotator.azim));
        azimMotor.run();
        
        // Position has been reached.
        if (azimMotor.distanceToGo() == 0)
        {
          // Set home position if parked successfully.
          if (rotator.state == EASYCOMM_STATUS_MOVING)
          {
            // Store home position.
            azimMotor.setCurrentPosition(0);
          }
          rotator.state = EASYCOMM_STATUS_IDLE;
          digitalWrite(RLED, LOW);
        }
        else
        {
          digitalWrite(RLED, HIGH);
        }
      }
      break;
      
    case EASYCOMM_STATUS_BREAKIN:
      {
        // Run the motors.
        if (rotator.breakinDir)
        {
          azimMotor.move(deg2step(BREAKIN_STEP));
        }
        else
        {
          azimMotor.move(deg2step(-BREAKIN_STEP));
        }
        azimMotor.run();
        
        // Position has been reached.
        if (((uint32_t)millis() - rotator.breakinStart) > rotator.breakinTimeout)
        {
          // Park the rotator.
          rotator.state = EASYCOMM_STATUS_MOVING;
        }
      }
      break;
      
    default:
      // TODO: Send alarm back to host.
      // Easycomm_sendAlarm(String((uint16_t)rotator.error));
      
    case EASYCOMM_STATUS_ERROR:
      {
        // Set error indicator.

        // Sleep for a bit.
        delay((uint32_t)rotator.errorTimeout*1000);
        // Attempt to park (sets state to MOVING).
        err = Rotator_setPark(NULL);
        if (err != 0)
        {
          // Blink error indicator.
          blinkLed(RLED, 1000, 10);
          // TODO: Send alarm back to host.
          // Easycomm_sendAlarm(String((uint16_t)rotator.error));
        }
        else
        {
          digitalWrite(RLED, LOW);
        }
        
      }
      break;
  }


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
  rotator.azim  = phaseWrap(azim);
  rotator.state = EASYCOMM_STATUS_POINTING;
  
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
  *freq = 0;
  
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
  *freq = 0;
  
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
  rotator.breakinDir     = fwd;
  rotator.breakinTimeout = msec;
  rotator.breakinStart   = millis();
  rotator.state          = EASYCOMM_STATUS_BREAKIN;
  
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
  float currDeg = step2deg(azimMotor.currentPosition());
  float nextDeg = currDeg - 0.5f;
  nextDeg       = phaseWrap(nextDeg);
  
  switch(rotator.state)
  {
    case EASYCOMM_STATUS_IDLE:
      break;
      
    default:
      return -1;
  }
  
  rotator.azim  = nextDeg;
  
  return 0;
};

int8_t
Rotator_setMoveRight(void *dev)
{
  float currDeg = step2deg(azimMotor.currentPosition());
  float nextDeg = currDeg + 0.5f;
  nextDeg       = phaseWrap(nextDeg);
  
  switch(rotator.state)
  {
    case EASYCOMM_STATUS_IDLE:
      break;
      
    default:
      return -1;
  }
  
  rotator.azim  = nextDeg;
  
  return 0;
};

int8_t
Rotator_setZero(void *dev)
{
  // Store home position.
  azimMotor.setCurrentPosition(0);
  
  return 0;
};

int8_t
Rotator_setPark(void *dev)
{
  // Request 4 rotations.
  rotator.azim = deg2step(step2deg(azimMotor.currentPosition()) + 4*360.0f);
  // Go to MOVING state.
  rotator.state = EASYCOMM_STATUS_MOVING;
  
  return 0;
};

int8_t
Rotator_getVersion(void *dev,
                   const char **vers)
{
  *vers = ROTATOR_VERSION;
  
  return 0;
};
