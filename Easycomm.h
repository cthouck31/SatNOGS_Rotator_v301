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
        int8_t (*setPark)(void *dev);
        
        int8_t (*setBreakIn)(void *dev, uint32_t msec, uint8_t fwd);

} Easycomm_Ctrl;

/*
  @brief Initialize the Easycomm controller.
  
  @param[in] ctrl  Easycomm controller object.
  
  @return 0 on success.
*/
int8_t
Easycomm_init(Easycomm_Ctrl *ctrl)
{
  if (ctrl == NULL)
  {
    return -1;
  }

  // Reset memory.
  memset(ctrl, 0, sizeof(Easycomm_Ctrl));

  return 0;
};

/*
  @brief Resets the Easycomm controller.
  
  @param[in] ctrl  Easycomm controller object.
  
  @return 0 on success.
*/
int8_t
Easycomm_reset(Easycomm_Ctrl *ctrl)
{
  if (ctrl == NULL)
  {
    return -1;
  }

  if (Serial)
  {
    Serial.flush();
  }

  return 0;
};

/*
  @brief Set the Easycomm serial port.
  
  @param[in] ctrl  Easycomm controller object.
  @param[in] comms Serial port for comms.
  
  @return 0 on success.
*/
int8_t
Easycomm_begin(Easycomm_Ctrl *ctrl,
               uint32_t baudRate)
{
  if (ctrl == NULL)
  {
    return -1;
  }
  
  Serial.begin(baudRate);
  while (!Serial);

  return Easycomm_reset(ctrl);
};

int8_t
Easycomm_write(Easycomm_Ctrl *ctrl,
               String line)
{
  if (ctrl == NULL)
  {
    return -1;
  }
  if (!Serial)
  {
    return -1;
  }

  Serial.print(line);

  return 0;
};

int8_t
Easycomm_writePacket(Easycomm_Ctrl *ctrl,
                     String line)
{
  return Easycomm_write(ctrl, line+String("\n"));
};

int8_t
Easycomm_setContext(Easycomm_Ctrl *ctrl,
                    void *dev)
{
  if (ctrl == NULL)
  {
    return -1;
  }

  ctrl->dev = dev;

  return 0;
};

int8_t
Easycomm_setDebugPins(Easycomm_Ctrl *ctrl,
                      const uint16_t passPin,
                      const uint16_t failPin)
{
  if (ctrl == NULL)
  {
    return -1;
  }
  
  ctrl->passLed = passPin;
  ctrl->failLed = failPin;
  ctrl->debugEnable = 1;

  return 0;
};

int8_t
Easycomm_blinkLed(Easycomm_Ctrl *ctrl,
                  const uint16_t pin,
                  const uint32_t numBlinks,
                  const uint32_t periodMs)
{
  uint32_t k;

  if (ctrl == NULL)
  {
    return -1;
  }
  
  if (ctrl->debugEnable)
  {
    for (k = 0; k < numBlinks; k++)
    {
      digitalWrite(pin, HIGH);
      delay(periodMs/2);
      digitalWrite(pin, LOW);
      delay(periodMs/2);
    }
  }

  return 0;
};

int8_t
Easycomm_blinkPassLed(Easycomm_Ctrl *ctrl,
                      const uint32_t numBlinks,
                      const uint32_t periodMs)
{
  if (ctrl == NULL)
  {
    return -1;
  }
  
  return Easycomm_blinkLed(ctrl, ctrl->passLed, numBlinks, periodMs);
};

int8_t
Easycomm_blinkFailLed(Easycomm_Ctrl *ctrl,
                      const uint32_t numBlinks,
                      const uint32_t periodMs)
{
  if (ctrl == NULL)
  {
    return -1;
  }
  
  return Easycomm_blinkLed(ctrl, ctrl->failLed, numBlinks, periodMs);
};

/*
  @brief Find and replace a substring in a string.
  
  @param[in] s  String to be searched.
  @param[in] loc  Substring to locate and replace.
  @param[in] rep  Substring to replace with if found.
  
  @return String with replaced substring (if found), otherwise the input string.
*/
String
findAndReplace(String *s,
               const char *loc,
               const char *rep)
{
  String repStr;
  String locStr;
  String newStr;
  int16_t idx;
  
  locStr = String(loc);
  
  idx = s->indexOf(locStr);
  if (idx < 0)
  {
    return (*s);
  }
  
  repStr = String(rep);
  
  if (idx == 0)
  {
    newStr = repStr + s->substring(idx+locStr.length());
  }
  else
  {
    newStr = s->substring(0, idx) + repStr + s->substring(idx+locStr.length());
  }

  return newStr;
};

/*
  @brief Convenience function to convert a float to a string.
  
  @param[in] x      Float number.
  @param[in] decPts Number of decimal points.
  
  @return String representing the float number.
*/
String
float2String(const float x,
             const uint8_t decPts)
{
  uint8_t k;
  float frac;
  int32_t fracInt;
  String number;
  
  int32_t whole = x - (((int32_t)x)-x);
  
  frac = x - whole;
  for (k = 0; k < decPts; k++)
  {
    frac = frac * 10;
  }
  fracInt = (int32_t)frac;
  
  number = String(whole) + String(".") + String(fracInt);
  
  return number;
};

/*
*/
String
parseToken(String *s,
           const char *loc)
{
  String parseStr;
  String locStr;
  int16_t idx;
  
  locStr = String(loc);
  
  // Locate substring.
  idx = s->indexOf(String(loc));
  if (idx < 0)
  {
    return String("");
  }

  // Find next whitespace.
  idx = parseStr.indexOf(' ');
  if (idx < 0)
  {
    return *s;
  }

  // Return value between the substring and the next whitespace.
  return s->substring(idx, idx+locStr.length());
};

/*
*/
String
parseValue(String *s,
           const char *loc)
{
  String parseStr;
  String locStr;
  int16_t idx;
  
  locStr = String(loc);
  
  // Locate substring.
  idx = s->indexOf(String(loc));
  if (idx < 0)
  {
    return String("");
  }
  
  // Remove substring.
  parseStr = s->substring(idx+locStr.length());
  // Find next whitespace.
  idx = parseStr.indexOf(' ');
  if (idx < 0)
  {
    // Check if there is a substring leftover.
    if (parseStr.length() > 0)
    {
      return parseStr;
    }
    // No value.
    return String("");
  }

  // Return value between the substring and the next whitespace.
  return parseStr.substring(0, idx-1);
};

int8_t
Easycomm_runWait(Easycomm_Ctrl *ctrl,
                 const uint32_t msec)
{
  char c;
  char buffer[16];
  String packet;
  String token, value;
  int32_t idx;
  uint16_t k;
  int8_t result;
  int32_t param;
  
  // Get starting time.
  uint32_t start = millis();
  
  // Process input until timeout.
  do
  {

    // Check for new serial communications.
    while (Serial.available() > 0)
    {
      // Read character.
      c = Serial.read();
      
      // Add character to buffer until EOL/CR is detected.
      if (c != '\n' && c != '\r')
      {
        // Add character to the buffer.
        ctrl->cmdMsg = String(ctrl->cmdMsg + c);
        continue;
      }
      
      //=====================================================
      // Process the packet.
      ctrl->cmdMsg.toUpperCase();
      // Indicate a packet was received.
      Easycomm_blinkPassLed(ctrl, 1, 10);
      
      Easycomm_writePacket(ctrl, ctrl->cmdMsg);
      
      //=====================================================
      // Azimuth/Elevation call.
      if (ctrl->cmdMsg.startsWith("AZ EL"))
      {
        // Get the parameters.
        result = -1;
        if (ctrl->getAzimElev)
        {
          result = ctrl->getAzimElev(ctrl->dev, &ctrl->azimuth, &ctrl->elevation);
        }
        // Return information to host.
        if (result == 0)
        {
          packet = String("AZ") + float2String(ctrl->azimuth, 1) + String(" ") + String("EL") + float2String(ctrl->elevation, 1);
          Easycomm_writePacket(ctrl, packet);
        }
        // Remove tokens from the string.
        ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, "AZ EL", "");
      }
      else
      {
        //=====================================================
        // Azimuth call.
        if ((idx=ctrl->cmdMsg.indexOf("AZ")) >= 0)
        {
          // Check the command for a parameter.
          token = parseToken(&ctrl->cmdMsg, "AZ");
          value = parseValue(&token, "AZ");
          
          // Get the parameters.
          result = -1;
          
          // Check for read or write.
          if (value.length() == 0)
          {
            // Read.
            if (ctrl->getAzim)
            {
              result = ctrl->getAzim(ctrl->dev, &ctrl->azimuth);
            }
            // Return information to host.
            if (result == 0)
            {
              packet = String("AZ") + float2String(ctrl->azimuth, 1);
              Easycomm_writePacket(ctrl, packet);
            }
            // Remove tokens from the string.
            ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, "AZ", "");
          }
          else
          {
            Easycomm_blinkFailLed(ctrl, 1, 100);
            // Write.
            if (ctrl->setAzim)
            {
              result = ctrl->setAzim(ctrl->dev, atof(value.c_str()));
            }
            // Remove tokens from the string.
            ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
          }
        }
        //=====================================================
        // Elevation call.
        if ((idx=ctrl->cmdMsg.indexOf("EL")) >= 0)
        {
          // Check the command for a parameter.
          token = parseToken(&ctrl->cmdMsg, "EL");
          value = parseValue(&token, "EL");
          
          // Get the parameters.
          result = -1;
          
          // Check for read or write.
          if (value.length() == 0)
          {
            // Read.
            if (ctrl->getElev)
            {
              result = ctrl->getElev(ctrl->dev, &ctrl->elevation);
            }
            // Return information to host.
            if (result == 0)
            {
              packet = String("EL") + float2String(ctrl->elevation, 1);
              Easycomm_writePacket(ctrl, packet);
            }
            // Remove tokens from the string.
            ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, "EL", "");
          }
          else
          {
            Easycomm_blinkFailLed(ctrl, 1, 100);
            // Write.
            if (ctrl->setElev)
            {
              result = ctrl->setElev(ctrl->dev, atof(value.c_str()));
            }
            // Remove tokens from the string.
            ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
          }
        }  
      }
      //=====================================================
      // Uplink Frequency call.
      if ((idx=ctrl->cmdMsg.indexOf("UP")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "UP");
        value = parseValue(&token, "UP");
        
        // Get the parameters.
        result = -1;
        
        // Check for read or write.
        if (value.length() == 0)
        {
          // Read.
          if (ctrl->getUplinkFreq)
          {
            result = ctrl->getUplinkFreq(ctrl->dev, &ctrl->uplinkFreq);
          }
          // Return information to host.
          if (result == 0)
          {
            packet = String("UP") + float2String(ctrl->uplinkFreq, 1);
            Easycomm_writePacket(ctrl, packet);
          }
          // Remove tokens from the string.
          ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, "UP", "");
        }
        else
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          // Write.
          if (ctrl->setUplinkFreq)
          {
            result = ctrl->setUplinkFreq(ctrl->dev, atof(value.c_str()));
          }
          // Remove tokens from the string.
          ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
        }
      }
      //=====================================================
      // Downlink Frequency call.
      if ((idx=ctrl->cmdMsg.indexOf("DN")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "DN");
        value = parseValue(&token, "DN");
        
        // Get the parameters.
        result = -1;
        
        // Check for read or write.
        if (value.length() == 0)
        {
          // Read.
          if (ctrl->getDownlinkFreq)
          {
            result = ctrl->getDownlinkFreq(ctrl->dev, &ctrl->downlinkFreq);
          }
          // Return information to host.
          if (result == 0)
          {
            packet = String("DN") + float2String(ctrl->downlinkFreq, 1);
            Easycomm_writePacket(ctrl, packet);
          }
          // Remove tokens from the string.
          ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
        }
        else
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          // Write.
          if (ctrl->setDownlinkFreq)
          {
            result = ctrl->setDownlinkFreq(ctrl->dev, atof(value.c_str()));
          }
          // Remove tokens from the string.
          ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
        }
      }
      //=====================================================
      // Break-in callback.
      if ((idx=ctrl->cmdMsg.indexOf("BREAKIN")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "BREAKIN");
        value = parseValue(&token, "BREAKIN");
        
        // Get the parameters.
        result = -1;
        
        // Check for read or write.
        if (value.length() == 0)
        {
          // Use default.
          value = String("3000");
        }
        
        Easycomm_writePacket(ctrl, value);
        
        // Write.
        if (ctrl->setBreakIn)
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          param = atoi(value.c_str());
          result = ctrl->setBreakIn(ctrl->dev,
                                    (param < 0) ? -param : param,
                                    param >= 0);
        }
        else
        {
          Easycomm_blinkPassLed(ctrl, 1, 1000);
        }
        // Remove tokens from the string.
        ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
      }
      //=====================================================
      // Move up callback.
      if ((idx=ctrl->cmdMsg.indexOf("MU")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "MU");
        
        // Get the parameters.
        result = -1;
        
        Easycomm_writePacket(ctrl, value);
        
        // Write.
        if (ctrl->setMoveUp)
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          result = ctrl->setMoveUp(ctrl->dev);
        }
        else
        {
          Easycomm_blinkPassLed(ctrl, 1, 1000);
        }
        // Remove tokens from the string.
        ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
      }
      //=====================================================
      // Move down callback.
      if ((idx=ctrl->cmdMsg.indexOf("MD")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "MD");
        
        // Get the parameters.
        result = -1;
        
        Easycomm_writePacket(ctrl, value);
        
        // Write.
        if (ctrl->setMoveDown)
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          result = ctrl->setMoveDown(ctrl->dev);
        }
        else
        {
          Easycomm_blinkPassLed(ctrl, 1, 1000);
        }
        // Remove tokens from the string.
        ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
      }
      //=====================================================
      // Move left callback.
      if ((idx=ctrl->cmdMsg.indexOf("ML")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "ML");
        
        // Get the parameters.
        result = -1;
        
        Easycomm_writePacket(ctrl, value);
        
        // Write.
        if (ctrl->setMoveLeft)
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          result = ctrl->setMoveLeft(ctrl->dev);
        }
        else
        {
          Easycomm_blinkPassLed(ctrl, 1, 1000);
        }
        // Remove tokens from the string.
        ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
      }
      //=====================================================
      // Move right callback.
      if ((idx=ctrl->cmdMsg.indexOf("MR")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "MR");
        
        // Get the parameters.
        result = -1;
        
        Easycomm_writePacket(ctrl, value);
        
        // Write.
        if (ctrl->setMoveRight)
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          result = ctrl->setMoveRight(ctrl->dev);
        }
        else
        {
          Easycomm_blinkPassLed(ctrl, 1, 1000);
        }
        // Remove tokens from the string.
        ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
      }
      //=====================================================
      // Park callback.
      if ((idx=ctrl->cmdMsg.indexOf("PARK")) >= 0)
      {
        // Check the command for a parameter.
        token = parseToken(&ctrl->cmdMsg, "PARK");
        
        // Get the parameters.
        result = -1;
        
        Easycomm_writePacket(ctrl, value);
        
        // Write.
        if (ctrl->setPark)
        {
          Easycomm_blinkFailLed(ctrl, 1, 100);
          result = ctrl->setPark(ctrl->dev);
        }
        else
        {
          Easycomm_blinkPassLed(ctrl, 1, 1000);
        }
        // Remove tokens from the string.
        ctrl->cmdMsg = findAndReplace(&ctrl->cmdMsg, token.c_str(), "");
      }
      
      Easycomm_writePacket(ctrl, ctrl->cmdMsg);
  
      // Flush the buffer.
      ctrl->cmdMsg = "";
      Serial.flush();
    }
  
  } while ((millis() - start) < msec);

  return 0;
};

int
Easycomm_run(Easycomm_Ctrl *ctrl)
{
  return Easycomm_runWait(ctrl, 0);
};

#endif // EASYCOMM_H
