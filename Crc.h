#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t Crc_Type_t;

#define CRC_WIDTH (8 * sizeof(Crc_Type_t))
#define CRC_MSB   (((Crc_Type_t)1) << (CRC_WIDTH-1))

#define CRC_POLY  (0x1021)
#define CRC_SEED  (0xffff)

/** @brief CRC Variables. */
static uint8_t    Crc_Initialized = 0;
static Crc_Type_t Crc_Table[256];


static void
Crc_init()
{
  uint8_t  b;
  uint16_t val;
  Crc_Type_t remainder;
  
  for (val = 0; val < 256; val++)
  {
    remainder = val << (CRC_WIDTH - 8);
    
    for (b = 8; b > 0; b--)
    {
      if (remainder & CRC_MSB)
      {
        remainder = (remainder << 1) ^ CRC_POLY;
      }
      else
      {
        remainder = (remainder << 1);
      }
    }
    
    Crc_Table[val] = remainder;
  }
  
  Crc_Initialized = 1;
};

static Crc_Type_t
Crc_process(const uint8_t *data,
            const uint32_t len)
{
  uint32_t k;
  uint8_t  oct;
  Crc_Type_t remainder = CRC_SEED;
  if (!Crc_Initialized)
  {
    Crc_init();
  }
  
  for (k = 0; k < len; k++)
  {
    oct       = data[k] ^ (remainder >> (CRC_WIDTH-8));
    remainder = Crc_Table[oct] ^ (remainder << 8);
  }
  
  return remainder;
};

#ifdef __cplusplus
};
#endif

#endif // __CRC_H__
