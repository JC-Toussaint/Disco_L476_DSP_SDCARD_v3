
/**-----------------------------------------------------------------
    @file       btypes.h
    @brief      base types and macro definitions
    @details    
    @author     Falko Bilz
    @copyright  Copyright(c) 2016 by Falko Bilz
------------------------------------------------------------------*/

#include <stdint.h>     // uint*_t definitions

#define _BV(bit)        ( (uint32_t)1 << (bit) )
#define vsizeof(v)      ( sizeof(v) / sizeof(v[0]) )

/**-----------------------------------------------------------------
  @brief      union for 2 byte value different access
------------------------------------------------------------------*/
typedef union
{
  int16_t       s16;
  uint16_t      u16;

  int8_t        s8[2];
  uint8_t       u8[2];
} U_WORD;

/**-----------------------------------------------------------------
  @brief      union for 4 byte value different access
------------------------------------------------------------------*/
typedef union
{
  int32_t       s32;
  uint32_t      u32;

  int16_t       s16[2];
  uint16_t      u16[2];

  int8_t        s8[4];
  uint8_t       u8[4];
} U_LONG;
