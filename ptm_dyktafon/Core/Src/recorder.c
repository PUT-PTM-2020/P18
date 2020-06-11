/*
 * recorder.c
 *
 *  Created on: Apr 23, 2020
 *      Author: stach
 */

#include <recorder.h>

const uint32_t SAMPLE_RATE = 8000;
const uint16_t BITS_PER_SAMPLE = 16;
const uint16_t CHUNK_SIZE = 256;
int16_t data_chunk[CHUNK_SIZE];
volatile int data_iterator = 0;

//funkcja musi być uruchomiona na początku i na końcu nagrania żeby przygotować miejsce na nagłowek
// oraz nadpisać go odpoweidnimi danymi
int AddWaveHeader(char* file_path)
{
	FIL *f;
	char* wave_header[44];

	f_open(f, file_path, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (!f)
	{
		// błąd otwarcia pliku
		return 1;
	}
	// sprawdzenie rozmiaru pliku
	uint32_t data_size = f_size(f) - 44;
	/* write chunkID, must be 'RIFF'  ------------------------------------------*/
	  wave_header[0] = 'R';
	  wave_header[1] = 'I';
	  wave_header[2] = 'F';
	  wave_header[3] = 'F';

	  /*CHUNK_SIZE - Write the file length */
	  int ChunkSize = data_size + 36;
	  wave_header[4]  = (uint8_t)((ChunkSize & 0xFF));
	  wave_header[5]  = (uint8_t)((ChunkSize >> 8) & 0xFF);
	  wave_header[6]  = (uint8_t)((ChunkSize >> 16) & 0xFF);
	  wave_header[7]  = (uint8_t)((ChunkSize >> 24) & 0xFF);


	  /*FORMAT - Write the file format, must be 'WAVE' */
	  wave_header[8]  = 'W';
	  wave_header[9]  = 'A';
	  wave_header[10] = 'V';
	  wave_header[11] = 'E';

	  /*SUBCHUNK_1_ID - Write the format chunk, must be'fmt ' */
	  wave_header[12]  = 'f';
	  wave_header[13]  = 'm';
	  wave_header[14]  = 't';
	  wave_header[15]  = ' ';

	  /*SUBCHUN_1_SIZE - Write the length of the 'fmt' data, must be 0x10 */
	  wave_header[16]  = 0x10;
	  wave_header[17]  = 0x00;
	  wave_header[18]  = 0x00;
	  wave_header[19]  = 0x00;

	  /*AUDIO_FORMAT -  Write the audio format, must be 0x01 (PCM) */
	  wave_header[20]  = 0x01;
	  wave_header[21]  = 0x00;

	  /*NUM_CHANNELS Write the number of channels, must be 0x01 (Mono) or 0x02 (Stereo) */
	  wave_header[22]  = 0x01;
	  wave_header[23]  = 0x00;

	  /*SAMPLE_RATE Write the Sample Rate 8000 Hz */
	  wave_header[24]  = (uint8_t)((SAMPLE_RATE  & 0xFF));
	  wave_header[25]  = (uint8_t)((SAMPLE_RATE  >> 8) & 0xFF);
	  wave_header[26]  = (uint8_t)((SAMPLE_RATE >> 16) & 0xFF);
	  wave_header[27]  = (uint8_t)((SAMPLE_RATE >> 24) & 0xFF);

	  /*BYTE_RATE Write the Byte Rate ( == SampleRate * NumChannels * BitsPerSample/8)*/
	  int byte_rate = SAMPLE_RATE * BITS_PER_SAMPLE / 8;
	  wave_header[28]  = (uint8_t)((byte_rate & 0xFF));
	  wave_header[29]  = (uint8_t)((byte_rate >> 8) & 0xFF);
	  wave_header[30]  = (uint8_t)((byte_rate >> 16) & 0xFF);
	  wave_header[31]  = (uint8_t)((byte_rate >> 24) & 0xFF);

	  /* Write the block alignment  == NumChannels * BitsPerSample/8*/
	  wave_header[32]  = 0x02;
	  wave_header[33]  = 0x00;

	  /* Write the number of bits per sample */
	  wave_header[34]  = 0x10;
	  wave_header[35]  = 0x00;

	  /* Write the Data chunk, must be 'data' */
	  wave_header[36]  = 'd';
	  wave_header[37]  = 'a';
	  wave_header[38]  = 't';
	  wave_header[39]  = 'a';

	  /* Write the number of sample data */
	  wave_header[40]  = (uint8_t)((data_size & 0xFF));
	  wave_header[41]  = (uint8_t)((data_size >> 8) & 0xFF);
	  wave_header[42]  = (uint8_t)((data_size >> 16) & 0xFF);
	  wave_header[43]  = (uint8_t)((data_size >> 24) & 0xFF);

	  uint16_t bw;
	  f_write(f, wave_header, 44, &bw);
	  f_close(f);
	  if (bw!=44) return 1;
	  return 0;
}

int SaveChunk(char* file_path, int16_t data[])
{
	FIL* f;
	f_open(f, file_path, FA_OPEN_APPEND | FA_WRITE);
	uint16_t bw;
	f_write(f, data, CHUNK_SIZE, &bw);
	f_close(f);
	if (CHUN_SIZE!=bw) return 1;
	return 0;
}
