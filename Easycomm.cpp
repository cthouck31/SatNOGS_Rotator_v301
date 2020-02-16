#include "Easycomm.h"
// CRC.
#include "Crc.h"

static uint16_t Easycomm_Hash_Table[EASYCOMM_NUM_CMDS];

/*
  @brief Initialize the Easycomm controller.
  
  @param[in] ctrl  Easycomm controller object.
  
  @return 0 on success.
*/
int8_t
Easycomm_init(Easycomm_Ctrl *ctrl)
{
  uint16_t hash;
  uint16_t k;
  uint16_t len;
  if (ctrl == NULL)
  {
    return -1;
  }

  // Reset memory.
  memset(ctrl, 0, sizeof(Easycomm_Ctrl));
  
  // Initialize CRC.
  Crc_init();
  
  // Compute CRC hash table.
  memset(Easycomm_Hash_Table, 0, sizeof(Easycomm_Hash_Table));
  for (k = 0; k < EASYCOMM_NUM_CMDS; k++)
  {
    // Compute hash.
    len  = strnlen(Easycomm_Cmds[k], 32);
    len  = len > 2 ? 2 : len;
    hash = Crc_process((uint8_t*)Easycomm_Cmds[k], len);
    // Store in hash table for fast look-up (index equals 'enum' of command).
    Easycomm_Hash_Table[k] = hash;
  }

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

//===========================================================
// Run Utilities.

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
  
  int32_t whole = (int32_t)x;
  
  frac = x - whole;
  for (k = 0; k < decPts; k++)
  {
    frac = frac * 10;
  }
  fracInt = (int32_t)frac;
  
  number = String(whole) + String(".") + String(fracInt);
  
  return number;
};

uint8_t
parseAllTokens(char** tokens,
               const uint8_t maxNumTokens,
               const char* line)
{
  uint8_t numTokens = 0;
  char *token;
  char buffer[256];
  strncpy(buffer, line,  sizeof(buffer));

  token = strtok(buffer, " ");
  while (token != NULL)
  {
    if (numTokens == maxNumTokens)
    {
      break;
    }
    tokens[numTokens++] = token;
    token = strtok(NULL, " ");
  }  

  return numTokens;
};

int16_t
getEasycommCmd(uint16_t cmdHash)
{
  uint16_t id;
  uint8_t  found = 0;
  
  for (id = 0; id < EASYCOMM_NUM_CMDS; id++)
  {
    if (Easycomm_Hash_Table[id] == cmdHash)
    {
      found = 1;
      break;
    }
  }
  
  return (found ? (int16_t)id : -1);
};

const char*
getEasycommValue(uint8_t cmd,
                 const char *token)
{
  uint16_t len;
  uint16_t cmdLen;
  
  if (cmd > EASYCOMM_NUM_CMDS)
  {
    return 0;
  }
  
  cmdLen = strnlen(Easycomm_Cmds[cmd], 128);
  len    = strnlen(token, 128);
  
  return (len > cmdLen) ? (token+cmdLen) : NULL;
};

//===========================================================


int8_t
Easycomm_runWait(Easycomm_Ctrl *ctrl,
                 const uint32_t msec)
{
  char c;
  String packet;
  uint16_t k;
  
  const uint8_t MAX_NUM_TOKENS = 64;
  uint8_t numTokens = 0;
  char* tokens [MAX_NUM_TOKENS];
  
  uint16_t   len;
  uint16_t   hash;
  int16_t    cmd;
  const char *valPtr;
  
  float data;
    
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
      
      //=====================================================
      // Parse all tokens.
      
      // Return packet.
      packet    = "";
      // Number of tokens in string.
      numTokens = parseAllTokens(tokens, MAX_NUM_TOKENS, ctrl->cmdMsg.c_str());
      
      // Process all tokens.
      for (k = 0; k < numTokens; k++)
      {
        // Get length of the token.
        len = strnlen(tokens[k], 64);

        // Skip short tokens.
        if (len < 2)
        {
          continue;
        }
        // Compute hash of token.
        hash     = Crc_process((uint8_t*)tokens[k], 2);
        // Get the Easycomm command ID.
        cmd      = getEasycommCmd(hash);
        // Detect invalid command.
        if (cmd < 0)
        {
          // Continue processing tokens.
          continue;
        }
        
        // Get the associated value(s).
        valPtr = getEasycommValue(cmd, tokens[k]);
        
        // Process the command.
        switch(cmd)
        {

          //==========================================================================
          // Azimuth.
          case EASYCOMM_AZ:
            {
              // Check if read/write.
              if (valPtr != NULL)
              {
                // Prevent null function call.
                if (ctrl->setAzim)
                {
                  ctrl->setAzim(ctrl->dev, atof(valPtr));
                }
              }
              else
              {
                // Create return packet.
                data = 0.0f;
                if (ctrl->getAzim)
                {
                  ctrl->getAzim(ctrl->dev, &data);
                }
                packet = String("AZ") + String(float2String(data, 1)) + String(" ") + packet;
              }
            }
            break;

          //==========================================================================
          // Elevation.
          case EASYCOMM_EL:
            {
              // Check if read/write.
              if (valPtr != NULL)
              {
                // Prevent null function call.
                if (ctrl->setElev)
                {
                  ctrl->setElev(ctrl->dev, atof(valPtr));
                }
              }
              else
              {
                // Create return packet.
                data = 0.0f;
                if (ctrl->getElev)
                {
                  ctrl->getElev(ctrl->dev, &data);
                }
                packet = packet + String("EL") + String(float2String(data, 1)) + String(" ");
              }
            }
            break;

          //==========================================================================
          // Uplink Frequency.
          case EASYCOMM_UP:
            {
              // Check if read/write.
              if (valPtr != NULL)
              {
                // Prevent null function call.
                if (ctrl->setUplinkFreq)
                {
                  ctrl->setUplinkFreq(ctrl->dev, atof(valPtr));
                }
              }
              else
              {
                // Create return packet.
                data = 0.0f;
                if (ctrl->getUplinkFreq)
                {
                  ctrl->getUplinkFreq(ctrl->dev, &data);
                }
                packet = packet + String("UP") + String(float2String(data, 1)) + String(" ");
              }
            }
            break;

          //==========================================================================
          // Downlink Frequency.
          case EASYCOMM_DN:
            {
              // Check if read/write.
              if (valPtr != NULL)
              {
                // Prevent null function call.
                if (ctrl->setDownlinkFreq)
                {
                  ctrl->setDownlinkFreq(ctrl->dev, atof(valPtr));
                }
              }
              else
              {
                // Create return packet.
                data = 0.0f;
                if (ctrl->getDownlinkFreq)
                {
                  ctrl->getDownlinkFreq(ctrl->dev, &data);
                }
                packet = packet + String("DN") + String(float2String(data, 1)) + String(" ");
              }
            }
            break;
            
          //==========================================================================
          // Move up.
          case EASYCOMM_MU:
            {
              // Prevent null function call.
              if (ctrl->setMoveUp)
              {
                ctrl->setMoveUp(ctrl->dev);
              }
            }
            break;
            
          //==========================================================================
          // Move down.
          case EASYCOMM_MD:
            {
              // Prevent null function call.
              if (ctrl->setMoveDown)
              {
                ctrl->setMoveDown(ctrl->dev);
              }
            }
            break;
            
          //==========================================================================
          // Move right.
          case EASYCOMM_MR:
            {
              // Prevent null function call.
              if (ctrl->setMoveRight)
              {
                ctrl->setMoveRight(ctrl->dev);
              }
            }
            break;
            
          //==========================================================================
          // Move left.
          case EASYCOMM_ML:
            {
              // Prevent null function call.
              if (ctrl->setMoveLeft)
              {
                ctrl->setMoveLeft(ctrl->dev);
              }
            }
            break;
            
          //==========================================================================
          // Version.
          case EASYCOMM_VE:
            {
              // Prevent null function call.
              if (ctrl->getVersion)
              {
                ctrl->getVersion(ctrl->dev, &valPtr);
              }
              packet = packet + String(valPtr);
            }
            break;
            
          //==========================================================================
          // Break-in.
          case EASYCOMM_BREAKIN:
            {
              data = 3;
              if (valPtr != NULL)
              {
                data = atoi(valPtr);
              }
              // Prevent null function call.
              if (ctrl->setBreakIn)
              {
                ctrl->setBreakIn(ctrl->dev, data > 0 ? data : -data, data > 0);
              }
            }
            break;
            
          //==========================================================================
          // Park.
          case EASYCOMM_PARK:
            {
              // Prevent null function call.
              if (ctrl->setPark)
              {
                ctrl->setPark(ctrl->dev);
              }
            }
            break;

          //==========================================================================
          // Invalid.
          default:
            // Invalid token detected.
            break;

        } // end switch.
        
      } // end token parsing.


      // Print return packet (if populated).
      if (packet.length())
      {
        Easycomm_writePacket(ctrl, packet);
      }

      // Flush the command.
      ctrl->cmdMsg = "";
      Serial.flush();
    }
  
  } while ((millis() - start) < msec);

  return 0;
};

int8_t
Easycomm_run(Easycomm_Ctrl *ctrl)
{
  return Easycomm_runWait(ctrl, 0);
};
