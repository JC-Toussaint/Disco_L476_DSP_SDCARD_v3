/**
 ******************************************************************************
 * @file    waveplayer.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    2019
 * @brief   This file includes a HAL version of the Wave Player driver
 * for the STM32072B-EVAL demonstration.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ff.h"
#include <math.h>

#include "btypes.h"               // U_LONG definition
#include "ff.h"                   // f_open(), f_read(), f_close() declarations
#include "time_f.h"               // time_*_ms() declarations
#include "waveplayer.h"           // T_WAVE_CHUNK definition
#include <string.h>               // memset() declaration

extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;

/** @defgroup WAVEPLAYER_Private_Variables
 * @{
 */
//#define AUDIO_BUFFER_SIZE     4080  /* max. 0xFFFF */ // too large for a SPI SDcard reader
#define AUDIO_BUFFER_SIZE_U8     8192 // 8192 // 2048 // 1024  /* max. 0xFFFF */
#define AUDIO_BUFFER_SIZE_U16    (AUDIO_BUFFER_SIZE_U8/2)
#define AUDIO_BUFFER_SIZE_U32    (AUDIO_BUFFER_SIZE_U8/4)

#define N_CHANNELS 2
#define BLOCK_SIZE   32
static uint32_t blockSize = BLOCK_SIZE;
static uint32_t numBlocks = (AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS)/BLOCK_SIZE; // FIR on half of the audiobuffer ONLY, per channel

#define F2Q15(x)  ((q15_t)(x *      32768UL))
#define F2Q31(x)  ((q31_t)(x * 2147483648UL))

/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 ** fir1(28, 6/24)
 ** ------------------------------------------------------------------- */
//#define NUM_TAPS                        29
//float32_t aFIR_F32_Coeffs[NUM_TAPS] = {
//-0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
//-0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
//+0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
//+0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
//};

//#define NUM_TAPS                       4
//float32_t aFIR_F32_Coeffs[NUM_TAPS] = {
//		   0.25f, 0.25f, 0.25f, 0.25f
//};

//#define NUM_TAPS                       2
//float32_t aFIR_F32_Coeffs[NUM_TAPS] = {
//		0.1f, 0.9f
//};

#define NUM_TAPS                       1
float32_t aFIR_F32_Coeffs[NUM_TAPS] = {
		1.0f
};

float32_t 	firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

float32_t 	aFIR_F32_Input_CH1 [AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS];  // FIR on half of the audiobuffer ONLY, per channel
float32_t 	aFIR_F32_Input_CH2 [AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS];  // FIR on half of the audiobuffer ONLY, per channel

float32_t 	aFIR_F32_Output_CH1[AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS];
float32_t 	aFIR_F32_Output_CH2[AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS];
/**-----------------------------------------------------------------
  @brief      helper union for audio data buffer
------------------------------------------------------------------*/
//typedef union
//{
//	uint32_t                    u32[2*AUDIO_BUFFER_SIZE / 4];
//	uint16_t                    u16[2*AUDIO_BUFFER_SIZE / 2];
//	uint8_t                     u8 [2*AUDIO_BUFFER_SIZE];
//} U_AUDIO_BUF;

typedef union
{
	uint32_t                    u32[AUDIO_BUFFER_SIZE_U32];
	uint16_t                    u16[AUDIO_BUFFER_SIZE_U16];
	uint8_t                     u8 [AUDIO_BUFFER_SIZE_U8];
} U_AUDIO_BUF;

/**-----------------------------------------------------------------
  @brief      const value defines for audio buffer state
------------------------------------------------------------------*/
typedef enum
{
	BUFFER_OFFSET_NONE = 0,
	BUFFER_OFFSET_HALF,
	BUFFER_OFFSET_FULL
} E_AUDIO_BUF_STATE;

U_AUDIO_BUF                   AudioBuf;       ///< buffer used for playing audio
volatile E_AUDIO_BUF_STATE    bufferOffset;   ///< Position in the audio play buffer
FIL                           fileObj;        ///< File object, contains buffers; size is > 2 KB

/**-----------------------------------------------------------------
  @short      Fill audio buffer with data from file
  @param[out] bufStart ... start address of buffer to be set
  @param[in]  bytesToRead  number of bytes to be read
  @param[in]  waveFormat . format of the wave file to be played
  @pre        bufStart is 4 byte aligned,\n
              needed for adjust_audio_data()
------------------------------------------------------------------*/
static inline bool read_audio_data ( uint8_t* bufStart, const uint16_t bytesToRead,
		const T_WAVE_CHUNK* const waveFormat )
{
	bool      report;
	UINT      bytesRead;

	report = f_read ( &fileObj, bufStart, bytesToRead, &bytesRead );
	if ( report == FR_OK  &&  bytesRead )
	{
		if ( bytesRead < bytesToRead )
		{
			memset ( bufStart + bytesRead, 0, bytesToRead - bytesRead );
		}
		report = true;
	}
	else
	{
		report = false;
	}

	return report;
}

/**
 * @brief  Wave player Initialization
 * @param  None
 * @retval None
 */
void WavePlayer_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

	/* USER CODE END SAI1_Init 1 */
	hsai_BlockA1.Instance = SAI1_Block_A;
	hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
	hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
	hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
	hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
	hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockA1.FrameInit.FrameLength = 32;
	hsai_BlockA1.FrameInit.ActiveFrameLength = 16;
	hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT; // SAI_FS_FIRSTBIT;
	hsai_BlockA1.SlotInit.FirstBitOffset = 0;
	hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_BlockA1.SlotInit.SlotNumber = 2;
	hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
	if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_UNLOCK(&hsai_BlockA1);     // THIS IS EXTREMELY IMPORTANT FOR hsai_BlockA1 TO WORK!!
	__HAL_SAI_ENABLE(&hsai_BlockA1); // THIS IS EXTREMELY IMPORTANT FOR hsai_BlockA1 TO WORK!!
}

/**-----------------------------------------------------------------
  @short      Play wave file from a mass storage
  @param[in]  fileName ... wave file name
  @remark     called from play_sound()
------------------------------------------------------------------*/
void wave_play_back ( const char* const fileName )
{
	UINT          bytesRead;
	T_WAVE_CHUNK  waveFormat;
	bool          run;
	arm_fir_instance_f32 FIR_F32_Struct;

	// Open the Wave file to be played
	if ( f_open(&fileObj, fileName, FA_READ) == FR_OK )
	{
		// Read sizeof(WaveFormat) bytes from file
		f_read ( &fileObj, &waveFormat, sizeof(waveFormat), &bytesRead );

		if ( waveFormat.BitPerSample != 16 )
		{
			run = false;
		}

		if ( waveFormat.channels == 1 )
		{
			run = false;
		}
		if ( sizeof(waveFormat) == bytesRead )
		{
			// Initialize wave player: I2S Codec, DMA, TWI, IOExpander and IOs
			WavePlayer_Init();
			run = true;
		}
		else
		{
			run = false;
		}

		if ( run )
		{
			printf("playing %s\n", fileName);

			// clear buffer for startup sequence
			memset ( AudioBuf.u8, 0, AUDIO_BUFFER_SIZE_U8 );

			bufferOffset = BUFFER_OFFSET_NONE;

			// Start playing Wave of AUDIO_BUFFER_SIZE/2 words
			if ( waveFormat.BitPerSample == 16 )
			{
				HAL_SAI_Transmit_DMA ( &hsai_BlockA1, (uint8_t *) AudioBuf.u16, AUDIO_BUFFER_SIZE_U16 );
			}
			else
			{
				HAL_SAI_Transmit_DMA ( &hsai_BlockA1, (uint8_t *) AudioBuf.u32, AUDIO_BUFFER_SIZE_U32 );

				// DMA buffer is mostly wrong after power up, repeat the procedure
				HAL_SAI_DMAStop ( &hsai_BlockA1 );
				HAL_SAI_Transmit_DMA ( &hsai_BlockA1, (uint8_t *) AudioBuf.u32, AUDIO_BUFFER_SIZE_U32 );
			}

			HAL_Delay ( 300 );  // TDA 7266 datasheet: 100..200 ms between Standby and Mute

			// HAL_GPIO_WritePin ( GPIOA, AMP_MUTE_Pin, GPIO_PIN_RESET );

			/* Call FIR init function to initialize the instance structure. */
			arm_fir_init_f32(&FIR_F32_Struct, NUM_TAPS, (float32_t *)&aFIR_F32_Coeffs[0], &firStateF32[0], blockSize);
		}


		while ( run )
		{
			switch ( bufferOffset )
			{
			case BUFFER_OFFSET_HALF:
				// read next AUDIO_BUFFER_SIZE/2 bytes after 1st part of buffer is played
				run = read_audio_data ( AudioBuf.u8, AUDIO_BUFFER_SIZE_U8/2, &waveFormat );


				//#define FIR
#ifdef FIR
				//#define ARM
#ifdef ARM
				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS; i++){ // FIR on half of the audiobuffer ONLY
					int16_t signal_CH1 = (int16_t) AudioBuf.u16[2*i+0]; // splitting of channel 1 and channel 2
					int16_t signal_CH2 = (int16_t) AudioBuf.u16[2*i+1]; // typecasting to signed integer

					aFIR_F32_Input_CH1[i] =(float32_t) signal_CH1;
					aFIR_F32_Input_CH2[i] =(float32_t) signal_CH2;
				}

				for (uint32_t counter_FIR_f32_p = 0; counter_FIR_f32_p < numBlocks; counter_FIR_f32_p++)
				{
					arm_fir_f32(&FIR_F32_Struct, aFIR_F32_Input_CH1 + (counter_FIR_f32_p * blockSize), aFIR_F32_Output_CH1 + (counter_FIR_f32_p * blockSize), blockSize);
					arm_fir_f32(&FIR_F32_Struct, aFIR_F32_Input_CH2 + (counter_FIR_f32_p * blockSize), aFIR_F32_Output_CH2 + (counter_FIR_f32_p * blockSize), blockSize);
				}

				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS; i++){
					AudioBuf.u16[2*i+0]=(uint16_t) (aFIR_F32_Output_CH1[i]);
					AudioBuf.u16[2*i+1]=(uint16_t) (aFIR_F32_Output_CH2[i]);

					AudioBuf.u16[2*i+0]=(uint16_t) (aFIR_F32_Input_CH1[i]);
					AudioBuf.u16[2*i+1]=(uint16_t) (aFIR_F32_Input_CH2[i]);
				}

#else

				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS-1-NUM_TAPS+1; i++){
					int16_t signal;
					float32_t fir_output;
					for (uint32_t nch=0; nch<N_CHANNELS; nch++){
						fir_output=0.0f;
						for (uint32_t j=0; j<NUM_TAPS; j++){
							uint32_t ip=i+j;
							signal=(int16_t) AudioBuf.u16[2*ip+nch];
							fir_output+=aFIR_F32_Coeffs[(NUM_TAPS-1)-j]*(float32_t)signal;
						}
						AudioBuf.u16[2*i+nch]=(uint16_t) fir_output;
					}
				}
#endif
#endif

//#define I32
#ifdef I32

				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS-1; i++){
					int16_t signal;
					int32_t fir_output; // MANDATORY int32 type value
					uint16_t *ptr=AudioBuf.u16+2*i+0;

					signal=((int16_t) *(ptr  ));
					fir_output  = (int32_t) signal;

					signal=((int16_t) *(ptr+2));
					fir_output += (int32_t) signal;
					*ptr=(uint16_t) (fir_output>>1); // division by 2

					signal=((int16_t) *(ptr+1));
					fir_output  = (int32_t) signal;

					signal=((int16_t) *(ptr+3));
					fir_output += (int32_t) signal;
					*(ptr+1)=(uint16_t) (fir_output>>1); // division by 2
				}

#endif

#define GI32

#ifdef GI32
#define NSHIFT 2
#define NTAPS (1<<NSHIFT)

				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS-NTAPS; i++){
					int16_t signal;
					int32_t fir_output; // MANDATORY int32 type value
					uint16_t *ptr=AudioBuf.u16+2*i+0;

					fir_output=0;
					for (uint32_t ntap=0; ntap<NTAPS; ntap++){
					signal=((int16_t) *(ptr+2*ntap));
					fir_output += (int32_t) signal;
					}
					*ptr=(uint16_t) (fir_output>>NSHIFT);

					fir_output=0;
					for (uint32_t ntap=0; ntap<NTAPS; ntap++){
					signal=((int16_t) *(ptr+1+2*ntap));
					fir_output += (int32_t) signal;
					}
					*(ptr+1)=(uint16_t) (fir_output>>NSHIFT);
				}

#endif
				bufferOffset = BUFFER_OFFSET_NONE;
				break;

			case BUFFER_OFFSET_FULL:
				// read next AUDIO_BUFFER_SIZE/2 bytes after 2nd part of buffer is played
				run = read_audio_data ( &AudioBuf.u8[AUDIO_BUFFER_SIZE_U8/2], AUDIO_BUFFER_SIZE_U8/2, &waveFormat );

#ifdef FIR
				//#define ARM
#ifdef ARM
				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS; i++){
					int16_t signal_CH1 = (int16_t) AudioBuf.u16[2*i+0+AUDIO_BUFFER_SIZE_U16/2]; // splitting of channel 1 and channel 2
					int16_t signal_CH2 = (int16_t) AudioBuf.u16[2*i+1+AUDIO_BUFFER_SIZE_U16/2]; // typecasting to signed integer

					aFIR_F32_Input_CH1[i] =(float32_t) signal_CH1;
					aFIR_F32_Input_CH2[i] =(float32_t) signal_CH2;
				}

				for (uint32_t counter_FIR_f32_p = 0; counter_FIR_f32_p < numBlocks; counter_FIR_f32_p++)
				{
					arm_fir_f32(&FIR_F32_Struct, aFIR_F32_Input_CH1 + (counter_FIR_f32_p * blockSize), aFIR_F32_Output_CH1 + (counter_FIR_f32_p * blockSize), blockSize);
					arm_fir_f32(&FIR_F32_Struct, aFIR_F32_Input_CH2 + (counter_FIR_f32_p * blockSize), aFIR_F32_Output_CH2 + (counter_FIR_f32_p * blockSize), blockSize);
				}

				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS; i++){
					AudioBuf.u16[2*i+0+AUDIO_BUFFER_SIZE_U16/2]=(uint16_t) (aFIR_F32_Output_CH1[i]);
					AudioBuf.u16[2*i+1+AUDIO_BUFFER_SIZE_U16/2]=(uint16_t) (aFIR_F32_Output_CH2[i]);

					AudioBuf.u16[2*i+0+AUDIO_BUFFER_SIZE_U16/2]=(uint16_t) (aFIR_F32_Input_CH1[i]);
					AudioBuf.u16[2*i+1+AUDIO_BUFFER_SIZE_U16/2]=(uint16_t) (aFIR_F32_Input_CH2[i]);
				}

#else

				for (uint32_t i=NUM_TAPS; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS-1-NUM_TAPS+1; i++){
					int16_t signal;
					float32_t fir_output;
					for (uint32_t nch=0; nch<N_CHANNELS; nch++){
						fir_output=0.0f;
						for (uint32_t j=0; j<NUM_TAPS; j++){
							uint32_t ip=i+j;
							signal=(int16_t) AudioBuf.u16[2*ip+nch+AUDIO_BUFFER_SIZE_U16/2];
							fir_output+=aFIR_F32_Coeffs[(NUM_TAPS-1)-j]*(float32_t)signal;
						}
						AudioBuf.u16[2*i+nch+AUDIO_BUFFER_SIZE_U16/2]=(uint16_t) fir_output;
					}
				}
#endif
#endif

#ifdef I32
				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS-1; i++){
					int16_t signal;
					int32_t fir_output; // MANDATORY int32 type value
					uint16_t *ptr=AudioBuf.u16+2*i+0+AUDIO_BUFFER_SIZE_U16/2;

					signal=((int16_t) *(ptr  ));
					fir_output  = (int32_t) signal;

					signal=((int16_t) *(ptr+2));
					fir_output += (int32_t) signal;
					*ptr=(uint16_t) (fir_output>>1); // division by 2

					signal=((int16_t) *(ptr+1));
					fir_output  = (int32_t) signal;

					signal=((int16_t) *(ptr+3));
					fir_output += (int32_t) signal;
					*(ptr+1)=(uint16_t) (fir_output>>1); // division by 2

				}
#endif

#ifdef GI32
				for (uint32_t i=0; i<AUDIO_BUFFER_SIZE_U16/2/N_CHANNELS-NTAPS; i++){
					int16_t signal;
					int32_t fir_output; // MANDATORY int32 type value
					uint16_t *ptr=AudioBuf.u16+2*i+0+AUDIO_BUFFER_SIZE_U16/2;

					fir_output=0;
					for (uint32_t ntap=0; ntap<NTAPS; ntap++){
					signal=((int16_t) *(ptr+2*ntap));
					fir_output += (int32_t) signal;
					}
					*ptr=(uint16_t) (fir_output>>NSHIFT);

					fir_output=0;
					for (uint32_t ntap=0; ntap<NTAPS; ntap++){
					signal=((int16_t) *(ptr+1+2*ntap));
					fir_output += (int32_t) signal;
					}
					*(ptr+1)=(uint16_t) (fir_output>>NSHIFT);
				}

#endif
				bufferOffset = BUFFER_OFFSET_NONE;
				break;

			default:
				break;
			}
		}

		//    HAL_GPIO_WritePin ( GPIOA, AMP_MUTE_Pin, GPIO_PIN_SET );
		HAL_SAI_DMAStop ( &hsai_BlockA1 );
		f_close ( &fileObj );
	}
}

/**
  @short      Handle the DMA transfer complete interrupt
  @details    This function is called when the requested data\n
              has been completely transferred
  @param[in]  hsai ... SAI1_Block handle
 */

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
	if ( hsai->Instance == SAI1_Block_A )
	{
		bufferOffset = BUFFER_OFFSET_FULL;
	}
}

/**
  @short      Handle the DMA transfer complete interrupt
  @details    This function is called when half of the\n
              requested buffer has been transferred
  @param[in]  hsai ... SAI1_Block handle
 */

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)  {
	if ( hsai->Instance == SAI1_Block_A )
	{
		bufferOffset = BUFFER_OFFSET_HALF;
	}

}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
