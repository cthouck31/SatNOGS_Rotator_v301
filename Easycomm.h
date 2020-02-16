#ifndef EASYCOMM_H
#define EASYCOMM_H

// Arduino.
#include "Arduino.h"

// Standard.
#include <stdint.h>
#include <string.h>

// Easycomm Types.
#include "Easycomm_Types.h"

// Easycomm definitions.
#define EASYCOMM_VERSION    (0x03)

// Easycomm Class.
typedef struct Easycomm_Ctrl
{
  //=====================================
  // Variables.
  //=====================================

        //===============================
        // Easycomm I.
        float azimuth;
        float elevation;
        float uplinkFreq;
        float downlinkFreq;

        //===============================
        // Communication.
        String cmdMsg;

        //===============================
        // Debug.
        uint8_t  debugEnable;
        uint16_t passLed;
        uint16_t failLed;

        //===============================        
         // Configurable context.
        void *dev;

        //===============================
        // Callback functions.
        int8_t (*getAzimElev)(void *dev, float *azim, float *elev);
        int8_t (*getAzim)(void *dev, float *azim);
        int8_t (*setAzim)(void *dev, float azim);
        int8_t (*getElev)(void *dev, float *elev);
        int8_t (*setElev)(void *dev, float elev);
        int8_t (*getUplinkFreq)(void *dev, float *freq);
        int8_t (*setUplinkFreq)(void *dev, float freq);
        int8_t (*getDownlinkFreq)(void *dev, float *freq);
        int8_t (*setDownlinkFreq)(void *dev, float freq);
        
        int8_t (*setMoveUp)(void *dev);
        int8_t (*setMoveDown)(void *dev);
        int8_t (*setMoveLeft)(void *dev);
        int8_t (*setMoveRight)(void *dev);
        int8_t (*setZero)(void *dev);
        int8_t (*setPark)(void *dev);
        
        int8_t (*setBreakIn)(void *dev, uint32_t sec, uint8_t fwd);
        int8_t (*getVersion)(void *dev, const char **vers);

} Easycomm_Ctrl;

/*
  @brief Initialize the Easycomm controller.
  
  @param[in] ctrl  Easycomm controller object.
  
  @return 0 on success.
*/
int8_t
Easycomm_init(Easycomm_Ctrl *ctrl);

/*
  @brief Resets the Easycomm controller.
  
  @param[in] ctrl  Easycomm controller object.
  
  @return 0 on success.
*/
int8_t
Easycomm_reset(Easycomm_Ctrl *ctrl);

/*
  @brief Set the Easycomm serial port.
  
  @param[in] ctrl  Easycomm controller object.
  @param[in] comms Serial port for comms.
  
  @return 0 on success.
*/
int8_t
Easycomm_begin(Easycomm_Ctrl *ctrl,
               uint32_t baudRate);

/*
  @brief Write to the Easycomm serial port.
  
  @param[in] ctrl Easycomm controller object.
  @param[in] line Line to write via serial port.
  
  @return 0 on success.
*/
int8_t
Easycomm_write(Easycomm_Ctrl *ctrl,
               String line);

/*
  @brief Write to the Easycomm serial port (w/ line end).
  
  @param[in] ctrl Easycomm controller object.
  @param[in] line Line to write via serial port.
  
  @return 0 on success.
*/
int8_t
Easycomm_writePacket(Easycomm_Ctrl *ctrl,
                     String line);

/*
  @brief Set the context for callbacks.
  
  @param[in] ctrl Easycomm controller object.
  @param[in] dev  Context to be passed to callback functions.
  
  @return 0 on success.
*/
int8_t
Easycomm_setContext(Easycomm_Ctrl *ctrl,
                    void *dev);

/*
  @brief Set the debug pins (if requested).
  
  @param[in] ctrl    Easycomm controller object.
  @param[in] passPin Pin to strobe on successful parse.
  @param[in] failPin Pin to strobe on failed parse.
  
  @return 0 on success.
*/
int8_t
Easycomm_setDebugPins(Easycomm_Ctrl *ctrl,
                      const uint16_t passPin,
                      const uint16_t failPin);

int8_t
Easycomm_blinkLed(Easycomm_Ctrl *ctrl,
                  const uint16_t pin,
                  const uint32_t numBlinks,
                  const uint32_t periodMs);

int8_t
Easycomm_blinkPassLed(Easycomm_Ctrl *ctrl,
                      const uint32_t numBlinks,
                      const uint32_t periodMs);

int8_t
Easycomm_blinkFailLed(Easycomm_Ctrl *ctrl,
                      const uint32_t numBlinks,
                      const uint32_t periodMs);

/*
  @brief Run the processing function (w/ timeout).
  
  @param[in] ctrl Easycomm controller object.
  @param[in] msec Port read timeout.
  
  @return 0 on success.
*/
int8_t
Easycomm_runWait(Easycomm_Ctrl *ctrl,
                 const uint32_t msec);

/*
  @brief Run the processing function (no timeout).
  
  @param[in] ctrl Easycomm controller object.
  
  @return 0 on success.
*/
int8_t
Easycomm_run(Easycomm_Ctrl *ctrl);

#endif // EASYCOMM_H
