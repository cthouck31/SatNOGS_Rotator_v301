#include <Arduino.h>

#include "AccelStepper.h"
#include "Easycomm.h"
#include "Endstop.h"
#include "Rotator_Pins.h"

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
#define MAX_SPEED         (3200/4)
#define MAX_ACCELERATION  (1600/4)
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
AccelStepper elevMotor(0x1, ELEV_MOTOR_PIN1, ELEV_MOTOR_PIN2);

// End-stops.
#define DFLT_SWITCH_STATE  (HIGH)
#define AZIM_SWITCH        (SW2)
#define ELEV_SWITCH        (SW1)
Endstop azimSwitch;
Endstop elevSwitch;

//=====================================================================
// Utility Functions.
//=====================================================================
void blinkLed(uint16_t pin, uint16_t numBlinks, uint16_t periodMs);

/**
 * @brief Failure trap. Returns alarm (AL) via serial
 * every second and strobes output pin.
 * 
 * @param[in] mask Alarm mask.
 * @param[in] msg  Message to print w/ debug info.
 * 
 * @returns None.
 */
#define failureTrap(mask, msg)                     \
do {                                               \
      String errMsg = String("AL") + String(mask); \
      /* Get the function name and line number.*/  \
      /* Must use define to get function/line. */  \
      errMsg += String(",");                       \
      errMsg += String(__func__);                  \
      errMsg += String("-line");                   \
      errMsg += String(__LINE__);                  \
      if (msg.length() > 0)                        \
      {                                            \
        errMsg += String(":") + String(msg);       \
        }                                          \
      digitalWrite(RLED, LOW);                     \
      while (1)                                    \
      {                                            \
        Serial.println(errMsg);                    \
        digitalWrite(RLED, HIGH);                  \
        delay(100);                                \
        digitalWrite(RLED, LOW);                   \
        delay(900);                                \
        }                                          \
  } while(0);

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
int8_t Rotator_setBreakIn(void *dev, uint32_t sec, uint8_t fwd);
int8_t Rotator_setZero(void *dev);
int8_t Rotator_setPark(void *dev);
int8_t Rotator_getVersion(void *dev, const char **vers);
int8_t Rotator_setSleep(void *dev);
// Control functions.
int8_t Rotator_park(void *dev);
int8_t Rotator_home(void *dev);
int8_t Rotator_breakIn(void *dev);

//=====================================================================
// Rotator State Machine.
//=====================================================================
typedef struct Rotator_State
{
  /** @brief Current state. */
  Easycomm_Status state;

  /** @brief Set azimuth and elevation. */
  uint8_t homingRequest;
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

} 
Rotator_State;

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
  err = Easycomm_setContext(&comms, &rotator);
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
  comms.setSleep        = Rotator_setSleep;

  // Initialize rotator steppers.
  // Azimuth.
  azimMotor.setEnablePin(STEPPER_ENABLE);
  azimMotor.setPinsInverted(0, 0, 1);
  azimMotor.enableOutputs();
  azimMotor.setMaxSpeed(MAX_SPEED);
  azimMotor.setAcceleration(MAX_ACCELERATION);
  azimMotor.setMinPulseWidth(MIN_PULSE_WIDTH);
  // Elevation.
  elevMotor.setEnablePin(STEPPER_ENABLE);
  elevMotor.setPinsInverted(0, 0, 1);
  elevMotor.enableOutputs();
  elevMotor.setMaxSpeed(MAX_SPEED);
  elevMotor.setAcceleration(MAX_ACCELERATION);
  elevMotor.setMinPulseWidth(MIN_PULSE_WIDTH);

  // Initialize end-stops.
  err = Endstop_init(&azimSwitch, AZIM_SWITCH, DFLT_SWITCH_STATE);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }
  err = Endstop_init(&elevSwitch, ELEV_SWITCH, DFLT_SWITCH_STATE);
  if (err != 0)
  {
    blinkLed(RLED, 3, 1000);
  }

  // Initialize rotator state machine.
  memset(&rotator, 0, sizeof(rotator));
  rotator.state         = EASYCOMM_STATUS_IDLE;
  rotator.error         = EASYCOMM_ERROR_NONE;
  rotator.errorTimeout  = 3; // seconds.
  rotator.isAzimHome    = 0;
  rotator.isElevHome    = 0;
  rotator.homingRequest = 0;

  // Park at start up.
  err = Rotator_park(NULL);
  // Trap on failure (prevent any further damage).
  if (err != 0)
  {
    failureTrap(EASYCOMM_ERROR_SENSOR | EASYCOMM_ERROR_JAM | EASYCOMM_ERROR_HOMING,
                String("PARK-FAILURE"));
  }

  // Blink status indicators.
  blinkLed(GLED, 5, 100);

};

void
loop()
{
  int8_t err;
  uint8_t azimEnd, elevEnd;

  // Run the Easycomm interface.
  // NOTE: Performs callbacks on successful reception of
  // packets. Flags should be checked after this function
  // to perform any requests.
  Easycomm_run(&comms);
  
  // Wake on move requests.
  switch (rotator.state)
  {
    case EASYCOMM_STATUS_IDLE:
      break;
      
    case EASYCOMM_STATUS_PARKING:
    case EASYCOMM_STATUS_MOVING:
    case EASYCOMM_STATUS_POINTING:
      azimMotor.enableOutputs();
      elevMotor.enableOutputs();
      break;
      
    default:
      break;
  }
  
  // Process requests.
  switch (rotator.state)
  {
    case EASYCOMM_STATUS_IDLE:
      break;
      
    case EASYCOMM_STATUS_PARKING:
      err = Rotator_park(NULL);
      rotator.state = (err == 0) ? 
                       EASYCOMM_STATUS_IDLE : EASYCOMM_STATUS_ERROR;
      break;
      
    case EASYCOMM_STATUS_MOVING:
      err = Rotator_breakIn(NULL);
      rotator.state = (err == 0) ? 
                       EASYCOMM_STATUS_IDLE : EASYCOMM_STATUS_ERROR;
      break;
      
    case EASYCOMM_STATUS_POINTING:
      err = Rotator_home(NULL);
      rotator.state = (err == 0) ? 
                       EASYCOMM_STATUS_IDLE : EASYCOMM_STATUS_ERROR;
      break;
      
    case EASYCOMM_STATUS_ERROR:
      blinkLed(RLED, 1, 1000);
      break;
      
    default:
      rotator.state = EASYCOMM_STATUS_ERROR;
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
  rotator.azim          = phaseWrap(azim);
  rotator.state         = EASYCOMM_STATUS_POINTING;

  return 0;
};

int8_t
Rotator_getElev(void *dev,
                float *elev)
{
  int32_t pos = elevMotor.currentPosition();
  *elev = step2deg(pos);

  return 0;
};

int8_t
Rotator_setElev(void *dev,
                float elev)
{
  rotator.elev          = phaseWrap(elev);
  rotator.state         = EASYCOMM_STATUS_POINTING;

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
                   uint32_t sec,
                   uint8_t fwd)
{
  rotator.breakinDir     = fwd;
  rotator.breakinTimeout = sec * 1000;
  rotator.breakinStart   = millis();
  rotator.state          = EASYCOMM_STATUS_MOVING;

  return 0;
};

int8_t
Rotator_setMoveUp(void *dev)
{
  float currDeg = step2deg(elevMotor.currentPosition());
  float nextDeg = currDeg + 0.5f;
  nextDeg       = phaseWrap(nextDeg);

  rotator.elev          = nextDeg;
  rotator.state         = EASYCOMM_STATUS_POINTING;
  
  return 0;
};

int8_t
Rotator_setMoveDown(void *dev)
{
  float currDeg = step2deg(elevMotor.currentPosition());
  float nextDeg = currDeg - 0.5f;
  nextDeg       = phaseWrap(nextDeg);

  rotator.elev          = nextDeg;
  rotator.state         = EASYCOMM_STATUS_POINTING;
  
  return 0;
};

int8_t
Rotator_setMoveLeft(void *dev)
{
  float currDeg = step2deg(azimMotor.currentPosition());
  float nextDeg = currDeg - 0.5f;
  nextDeg       = phaseWrap(nextDeg);

  rotator.azim          = nextDeg;
  rotator.state         = EASYCOMM_STATUS_POINTING;

  return 0;
};

int8_t
Rotator_setMoveRight(void *dev)
{ 
  float currDeg = step2deg(azimMotor.currentPosition());
  float nextDeg = currDeg + 0.5f;
  nextDeg       = phaseWrap(nextDeg);

  rotator.azim          = nextDeg;
  rotator.state         = EASYCOMM_STATUS_POINTING;

  return 0;
};

int8_t
Rotator_setZero(void *dev)
{
  // Store home positions.
  azimMotor.setCurrentPosition(0);
  elevMotor.setCurrentPosition(0);

  return 0;
};

int8_t
Rotator_setPark(void *dev)
{
  rotator.state = EASYCOMM_STATUS_PARKING;

  return 0;
};

int8_t
Rotator_park(void *dev)
{
  // Azimuth.
  uint8_t azimFound      = 0;
  uint8_t azimEnd        = 0;
  uint8_t prevAzimEnd    = 0;
  int32_t azimStartStep  = 0;
  int32_t azimStopStep   = 0;
  int32_t azimHomePos    = azimMotor.currentPosition() - 4*deg2step(360.0f);
  // Elevation.
  uint8_t elevFound      = 0;
  uint8_t elevEnd        = 0;
  uint8_t prevElevEnd    = 0;
  int32_t elevStartStep  = 0;
  int32_t elevStopStep   = 0;
  int32_t elevHomePos    = elevMotor.currentPosition() - 4*deg2step(360.0f);

  uint32_t start = 0;
  
  uint32_t check = millis();

  // Detect home locations (prevents parking if already parked).
  rotator.isAzimHome  = Endstop_read(&azimSwitch);
  rotator.isElevHome  = Endstop_read(&elevSwitch);

  // Indicate that the system is parking.
  digitalWrite(RLED, HIGH);

  // Homing loop.
  while (!rotator.isAzimHome || !rotator.isElevHome)
  {
    // Break if parking for over 45 seconds.
    if ((millis()-check) > 45000)
    {
      break;
    }
    
    //======================================================================
    // Azimuth loop.
    azimEnd = Endstop_read(&azimSwitch);

    // Keep checking for the end-stop until it has been found.
    if (!azimFound)
    {
      // Detect rising/falling edges of end-stop.
      if (!prevAzimEnd && azimEnd)
      {
        // Record when the end-stop is first detected (rising edge).
        azimStartStep = azimMotor.currentPosition();
//        Serial.println(String(azimStartStep));
      }
      else if (prevAzimEnd && !azimEnd)
      {
        // Record when the end-stop detection is lost (falling-edge).
        azimStopStep = azimMotor.currentPosition();
        // Calculate best estimate of end-stop center.
        azimHomePos  = (azimStopStep + azimStartStep)/2;
        // Record that the end has been found.
        azimFound = 1;
//        Serial.println(String(azimStopStep));
//        Serial.println(String(azimHomePos));
        // Start a timeout for homing (allow loop to converge).
        if (elevFound)
        {
          // Start a timeout for homing (allow loop to converge).
          start     = millis();
        }
      }
    }

    // Move to the desired position.
    azimMotor.moveTo(azimHomePos);
    azimMotor.run();

    // Store end-stop state.
    prevAzimEnd = azimEnd;

    //======================================================================
    // Elevation loop.
    elevEnd = Endstop_read(&elevSwitch);
    
    // Keep checking for the end-stop until it has been found.
    if (!elevFound)
    {
      // Detect rising/falling edges of end-stop.
      if (!prevElevEnd && elevEnd)
      {
        // Record when the end-stop is first detected (rising edge).
        elevStartStep = elevMotor.currentPosition();
      }
      else if (prevElevEnd && !elevEnd)
      {
        // Record when the end-stop detection is lost (falling-edge).
        elevStopStep = elevMotor.currentPosition();
        // Calculate best estimate of end-stop center.
        elevHomePos  = (elevStopStep + elevStartStep)/2;
        // Record that the end has been found.
        elevFound = 1;
        if (azimFound)
        {
          // Start a timeout for homing (allow loop to converge).
          start     = millis();
        }
      }
    }

    // Move to the desired position.
    elevMotor.moveTo(elevHomePos);
    elevMotor.run();
    
  // Store end-stop state.
    prevElevEnd = elevEnd;

    // After the home position is found, allow the
    // homing loop to run until steady-state.
    if (azimFound && elevFound)
    {
      // Check if timeout has been reached.
      if ((millis()-start) > HOMING_DELAY)
      {
        // Break out of the homing loop.
        rotator.isAzimHome = 1;
        rotator.isElevHome = 1;
      }
    }

  }

  // Check that the rotator is actually parked.
  if (azimMotor.distanceToGo() != 0)
  {
    return -1;
  }
  // Check that the end-stop is enabled (should be parked at the end-stop).
  if (!(azimEnd=Endstop_read(&azimSwitch)))
  {
    return -1;
  }
  if (elevMotor.distanceToGo() != 0)
  {
    return -1;
  }
  if (!(elevEnd=Endstop_read(&elevSwitch)))
  {
    return -1;
  }

  // Set the park positions.
  azimMotor.setCurrentPosition(0);
  elevMotor.setCurrentPosition(0);

  // Signal that the rotator has parked.
  digitalWrite(RLED, LOW);

  // Sleep when parked.
  return Rotator_setSleep(dev);
};

int8_t
Rotator_home(void *dev)
{
  uint8_t azHomed = 0;
  uint8_t elHomed = 0;
  int32_t azStep  = deg2step(rotator.azim);
  int32_t elStep  = deg2step(rotator.elev);

  // Set location to move to.
  azimMotor.moveTo(azStep);
  elevMotor.moveTo(elStep);

  while (!azHomed || !elHomed)
  {
    // Break out when the rotator has moved to location.
    if (azimMotor.distanceToGo() == 0)
    {
      azHomed = 1;
    }
    if (elevMotor.distanceToGo() == 0)
    {
      elHomed = 1;
    }

    azimMotor.run();
    elevMotor.run();
  }

  // Ensure the rotator is in position.
  if (azimMotor.distanceToGo() != 0)
  {
    return -1;
  }
  if (elevMotor.distanceToGo() != 0)
  {
    return -1;
  }
  
  // Sleep when done moving.
  return Rotator_setSleep(dev);
};

int8_t
Rotator_breakIn(void *dev)
{
  // Compute step size (30 degrees / call).
  int32_t moveStep = rotator.breakinDir ? deg2step(30.0f) : -deg2step(30.0f);

  // Move until timeout is reached.
  while ((millis() - rotator.breakinStart) < rotator.breakinTimeout)
  {
    // Step forward.
    azimMotor.move(moveStep);
    elevMotor.move(moveStep);
    // Run the motor.
    azimMotor.run();
    elevMotor.run();
  }

  // Park rotator.
  return Rotator_park(NULL);
};

int8_t
Rotator_getVersion(void *dev,
const char **vers)
{
  *vers = ROTATOR_VERSION;

  return 0;
};

int8_t
Rotator_setSleep(void *dev)
{
  // Sleep the motors to save power.
  azimMotor.disableOutputs();
  elevMotor.disableOutputs();
  
  return 0;
};

