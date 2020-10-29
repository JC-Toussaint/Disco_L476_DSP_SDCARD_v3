
/**-----------------------------------------------------------------
    @file       waveplayer.h
    @brief      Interface for playing wave files
    @details    
    @author     Falko Bilz
    @copyright  Copyright(c) 2016-2017 by Falko Bilz
------------------------------------------------------------------*/

#pragma once

#include "ff.h"           // FIL definition
#include <stdint.h>       // uint16_t definition

/**-----------------------------------------------------------------
  @brief      struct of RIFF chunk for simplified wave file
------------------------------------------------------------------*/
typedef struct
{
  char                        ChunkID[4];     ///< Byte 0..3
  uint32_t                    FileSize;       ///< Byte 4..7
  char                        riffType[4];    ///< Byte 8..11
  uint32_t                    SubChunk1ID;    ///< Byte 12..15
  uint32_t                    SubChunk1Size;  ///< Byte 16..19
  uint16_t                    formatTag;      ///< Byte 20, 21: Format category
  uint16_t                    channels;       ///< Byte 22, 23: Number of Channels
  uint32_t                    SampleRate;     ///< Byte 24..27: Sampling rate at which each channel should be played
  uint32_t                    AvgByteRate;    ///< Byte 28..31
  uint16_t                    BlockAlign;     ///< Byte 32, 33
  uint16_t                    BitPerSample;   ///< Byte 34, 35
  uint32_t                    SubChunk2ID;    ///< Byte 36..39
  uint32_t                    SubChunk2Size;  ///< Byte 40..43
} T_WAVE_CHUNK;

/**-----------------------------------------------------------------
  @brief      struct of RIFF chunk as union
------------------------------------------------------------------*/
typedef union
{
  T_WAVE_CHUNK                s;
  uint8_t                     u[ sizeof(T_WAVE_CHUNK) ];
} U_WAVE_CHUNK;

void  wave_play_back      ( const char* const fileName );
void  set_volume          ( const uint16_t vol );

extern FIL  fileObj;      // export for sharing because of its huge size of > 2 KB
