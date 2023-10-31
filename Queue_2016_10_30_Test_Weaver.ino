/***********************************************************************
 * 
 * Weaver-style audio chain
 * 
 * Frank DD4WH 2016_10_30
 * 
 *  first experiment 
 */

// with optimizations by Pete El Supremo 2016_10_27, thanks Pete!
//
// this is a setup to test whether we can do real time audio processing 
// with the Teensy 3.5 in floating point math
// it takes the LINE INPUT audio, converts it from int16_t to float32_t, 
// does the audio processing in float32_t with the NEW ARM CMSIS lib (see https://forum.pjrc.com/threads/38325-Excellent-results-with-Floating-Point-FFT-IFFT-Processing-and-Teensy-3-6?p=119797&viewfull=1#post119797),
// and see here: https://forum.pjrc.com/threads/38325-Excellent-results-with-Floating-Point-FFT-IFFT-Processing-and-Teensy-3-6?p=120177&viewfull=1#post120177 
// . . . converts the audio back to int16_t and gives it back to the headphone output
//
// MIT licence

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Metro.h>
#include "font_Arial.h"
#include <ILI9341_t3.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#include "Filter_coeffs.h"

#define BACKLIGHT_PIN 0

#define TFT_DC      20
#define TFT_CS      21
#define TFT_RST     32  // 255 = unused. connect to 3.3V
#define TFT_MOSI     7
#define TFT_SCLK    14
#define TFT_MISO    12

//void sinewave(void);

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

Metro five_sec=Metro(2000); // Set up a 0.5 second Metro

// this audio comes from the codec by I2S2

AudioInputI2S            i2s_in; 
           
AudioRecordQueue         Q_in_L;    
AudioRecordQueue         Q_in_R;    

AudioPlayQueue           Q_out_L; 
AudioPlayQueue           Q_out_R; 
AudioAnalyzeFFT256  myFFT;
AudioOutputI2S           i2s_out;           
AudioConnection          patchCord1(i2s_in, 0, Q_in_L, 0);
AudioConnection          patchCord2(i2s_in, 1, Q_in_R, 0);
AudioConnection      patchCord5(Q_out_R,0,myFFT,0); 
AudioConnection          patchCord3(Q_out_L, 0, i2s_out, 1);
AudioConnection          patchCord4(Q_out_R, 0, i2s_out, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265.212

int idx_t = 0;
int idx = 0;
int64_t sum;
float32_t mean;
int n_L;
int n_R;
long int n_clear;

int peak[512];
int barm[512];

ulong samp_ptr = 0;
bool FFT_state = false;

const int myInput = AUDIO_INPUT_LINEIN;

// We're only processing one buffer at a time so the
// number of samples is fixed at 128
#define BUFFER_SIZE 128

// buffers for quadrature oscillator 1
float32_t Osc1_Q_buffer [BUFFER_SIZE];
float32_t Osc1_I_buffer [BUFFER_SIZE];
float32_t a1_buffer [BUFFER_SIZE];
float32_t b1_buffer [BUFFER_SIZE];
float32_t c1_buffer [BUFFER_SIZE];
float32_t d1_buffer [BUFFER_SIZE];
float32_t e1_buffer [BUFFER_SIZE];
float32_t f1_buffer [BUFFER_SIZE];
 
bool sine_flag1 = false;
int16_t IF_FREQ1 = AUDIO_SAMPLE_RATE_EXACT / 16;
bool dir1 = false;

// definitions for quadrature oscillator 2
float32_t NCO_FREQ = AUDIO_SAMPLE_RATE_EXACT / 16; // + 20;
float32_t NCO_INC = 2 * PI * NCO_FREQ / AUDIO_SAMPLE_RATE_EXACT;
float32_t OSC_COS = cos (NCO_INC);
float32_t OSC_SIN = sin (NCO_INC);
float32_t Osc_Vect_Q = 1.0;
float32_t Osc_Vect_I = 0.0;
float32_t Osc_Gain = 0.0;
float32_t Osc_Q = 0.0;
float32_t Osc_I = 0.0;
float32_t i_temp = 0.0;
float32_t q_temp = 0.0;
//float32_t Osc2_Q_buffer [BUFFER_SIZE];
//float32_t Osc2_I_buffer [BUFFER_SIZE];
/*
bool sine_flag2 = false;
//int16_t IF_FREQ2 = 5515;
int16_t IF_FREQ2 = AUDIO_SAMPLE_RATE_EXACT / 16; 
bool dir2 = true;
*/
// Decimation and Interpolation factor
#define DECIMATION_FACTOR 4

float32_t float_buffer_L [BUFFER_SIZE];
float32_t float_buffer_R [BUFFER_SIZE];
float32_t float_buffer_L_3 [BUFFER_SIZE];
float32_t float_buffer_R_3 [BUFFER_SIZE];

// This holds the samples output by the decimation
// and therefore there are a factor of 4 fewer samples
float32_t float_buffer_L_2 [BUFFER_SIZE / DECIMATION_FACTOR];
float32_t float_buffer_R_2 [BUFFER_SIZE / DECIMATION_FACTOR];

// determine the number of decimation and interpolation coefficients
// from the size of their arrays
const int N_DEC_COEFFS = sizeof(FIR_dec1_coeffs)/sizeof(FIR_dec1_coeffs[0]);
const int N_INT_COEFFS = sizeof(FIR_int1_coeffs)/sizeof(FIR_int1_coeffs[0]);

// decimation with FIR lowpass
arm_fir_decimate_instance_f32 FIR_dec1_L;
float32_t FIR_decim_state_L [N_DEC_COEFFS + BUFFER_SIZE - 1];
arm_fir_decimate_instance_f32 FIR_dec1_R;
float32_t FIR_decim_state_R [N_DEC_COEFFS + BUFFER_SIZE - 1];

// interpolation with FIR lowpass
arm_fir_interpolate_instance_f32 FIR_int1_L;
float32_t FIR_interp_state_L [(N_INT_COEFFS / DECIMATION_FACTOR) + BUFFER_SIZE - 1];
arm_fir_interpolate_instance_f32 FIR_int1_R;
float32_t FIR_interp_state_R [(N_INT_COEFFS / DECIMATION_FACTOR) + BUFFER_SIZE - 1];

// FIR filter
// determine the number of filter coeffs from the size of their arrays
const int N_FIR1_COEFFS = sizeof(FIR_lowpass1_coeffs)/sizeof(FIR_lowpass1_coeffs[0]);
arm_fir_instance_f32 FIR_lowpass1_L;
float32_t FIR_lowpass1_state_L [N_FIR1_COEFFS + BUFFER_SIZE - 1];
arm_fir_instance_f32 FIR_lowpass1_R;
float32_t FIR_lowpass1_state_R [N_FIR1_COEFFS + BUFFER_SIZE - 1];

/****************************************************************************************
 *  init IIR filters
 ****************************************************************************************/
// 2-pole biquad IIR - definitions and Initialisation
const int N_stages_biquad_lowpass1 = sizeof(biquad_lowpass1_coeffs)/sizeof(biquad_lowpass1_coeffs[0]) / 5;
float32_t biquad_lowpass1_state_L [N_stages_biquad_lowpass1 * 4];
float32_t biquad_lowpass1_state_R [N_stages_biquad_lowpass1 * 4];
arm_biquad_casd_df1_inst_f32 biquad_lowpass1_L = {N_stages_biquad_lowpass1, biquad_lowpass1_state_L, biquad_lowpass1_coeffs}; 
arm_biquad_casd_df1_inst_f32 biquad_lowpass1_R = {N_stages_biquad_lowpass1, biquad_lowpass1_state_R, biquad_lowpass1_coeffs}; 

// 10-pole biquad IIR - definitions and Initialisation
const int N_stages_biquad_lowpass2 = sizeof(biquad_lowpass2_coeffs)/sizeof(biquad_lowpass2_coeffs[0]) / 5;
float32_t biquad_lowpass2_state_L [N_stages_biquad_lowpass2 * 4];
float32_t biquad_lowpass2_state_R [N_stages_biquad_lowpass2 * 4];
arm_biquad_casd_df1_inst_f32 biquad_lowpass2_L = {N_stages_biquad_lowpass2, biquad_lowpass2_state_L, biquad_lowpass2_coeffs}; 
arm_biquad_casd_df1_inst_f32 biquad_lowpass2_R = {N_stages_biquad_lowpass2, biquad_lowpass2_state_R, biquad_lowpass2_coeffs}; 

// complex FFT with the new library CMSIS V4.5
const static arm_cfft_instance_f32 *S;
const int length_FFT = 256; 
float32_t FFT_buffer [length_FFT * 2] __attribute__ ((aligned (4)));

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Audio connections require memory. and the record queue
  // uses this memory to buffer incoming audio.
  AudioMemory(100);

  // Enable the audio shield. select input. and enable output
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.5);
  sgtl5000_1.adcHighPassFilterDisable(); // does not help too much!

/*  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(SD.begin(10))) {
    // stop here if no SD card. but print a message
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
*/
  pinMode( BACKLIGHT_PIN, OUTPUT );
  analogWrite( BACKLIGHT_PIN, 1023 );

  tft.begin();
  tft.setRotation( 3 );
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(10, 1);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setFont(Arial_14);
  tft.print("Floating point audio processing");


/****************************************************************************************
 *  init FIR filters
 ****************************************************************************************/
    arm_fir_init_f32(&FIR_lowpass1_L, N_FIR1_COEFFS, FIR_lowpass1_coeffs, FIR_lowpass1_state_L,BUFFER_SIZE);
    arm_fir_init_f32(&FIR_lowpass1_R, N_FIR1_COEFFS, FIR_lowpass1_coeffs, FIR_lowpass1_state_R,BUFFER_SIZE);
/****************************************************************************************
 *  init decimation and interpolation filters
 ****************************************************************************************/
  if(arm_fir_decimate_init_f32(&FIR_dec1_L, N_DEC_COEFFS, DECIMATION_FACTOR, FIR_dec1_coeffs, FIR_decim_state_L, BUFFER_SIZE)) {
    Serial.println("Init of decimation failed");
    while(1);
  }
  if(arm_fir_interpolate_init_f32(&FIR_int1_L,  DECIMATION_FACTOR, N_INT_COEFFS, FIR_int1_coeffs, FIR_interp_state_L, BUFFER_SIZE/DECIMATION_FACTOR)) {
    Serial.println("Init of interpolation failed");
    while(1);  
  }
  if(arm_fir_decimate_init_f32(&FIR_dec1_R, N_DEC_COEFFS,  DECIMATION_FACTOR, FIR_dec1_coeffs, FIR_decim_state_R, BUFFER_SIZE)) {
    Serial.println("Init of decimation failed");
    while(1);
  }
  if(arm_fir_interpolate_init_f32(&FIR_int1_R, DECIMATION_FACTOR, N_INT_COEFFS, FIR_int1_coeffs, FIR_interp_state_R, BUFFER_SIZE/DECIMATION_FACTOR)) {
    Serial.println("Init of interpolation failed");
    while(1);  
  }
 /****************************************************************************************
 *  init complex FFT
 ****************************************************************************************/
   switch (length_FFT) {
    case 16:
      S = &arm_cfft_sR_f32_len16;
      break;
    case 32:
      S = &arm_cfft_sR_f32_len32;
      break;
    case 64:
      S = &arm_cfft_sR_f32_len64;
      break;
    case 128:
      S = &arm_cfft_sR_f32_len128;
      break;
    case 256:
      S = &arm_cfft_sR_f32_len256;
      break;
    case 512:
      S = &arm_cfft_sR_f32_len512;
      break;
    case 1024:
      S = &arm_cfft_sR_f32_len1024;
      break;
    case 2048:
      S = &arm_cfft_sR_f32_len2048;
      break;
    case 4096:
      S = &arm_cfft_sR_f32_len4096;
      break;
  }
  
 /****************************************************************************************
 *  begin to queue the audio from the audio library
 ****************************************************************************************/
    delay(100);
    Q_in_L.begin();
    Q_in_R.begin();    
} // END SETUP


int16_t *sp_L;
int16_t *sp_R;

void loop() {
 
 elapsedMicros usec = 0;
/**********************************************************************************
 *  Get samples from queue buffers
 **********************************************************************************/

    // this is supposed to prevent overfilled queue buffers
    if (Q_in_L.available() > 3 || Q_in_R.available() > 3) {
      Q_in_L.clear();
      Q_in_R.clear();
      n_clear ++; // just for debugging to check how often this occurs
    }
    // is there at least one buffer in each channel available ?
    if (Q_in_L.available() >= 1 && Q_in_R.available() >= 1)
    {   
    sp_L = Q_in_L.readBuffer();
    sp_R = Q_in_R.readBuffer();

      // convert to float
     arm_q15_to_float (sp_L, float_buffer_L, BUFFER_SIZE); // convert int_buffer to float 32bit
     arm_q15_to_float (sp_L, float_buffer_R, BUFFER_SIZE); // convert int_buffer to float 32bit
     Q_in_L.freeBuffer();
     Q_in_R.freeBuffer();

/**********************************************************************************
 *  Put 128 floating point samples into FFT buffer and set flag FFT_state when it is filled
 **********************************************************************************/
        for(int i = 0; i < BUFFER_SIZE; i++)
        {
            if(FFT_state == false) // FFT buffer not yet filled
            {
                FFT_buffer [samp_ptr] = (float32_t)float_buffer_L[i];    // get floating point data for FFT for spectrum scope/waterfall display
                samp_ptr++;
                FFT_buffer [samp_ptr] = (float32_t)float_buffer_R[i];
                samp_ptr++;

                // On obtaining enough samples for spectrum scope/waterfall, update state machine, reset pointer and wait until we process what we have
                if(samp_ptr > length_FFT) 
                {
                    samp_ptr = 0;
                    FFT_state = true; // FFT buffer filled with length_FFT samples
                }
            }
        }

/**************************************************************************
 * From here, all the 32 bit float audio processing can start
 * ************************************************************************

/**************************************************************************
 *   This is the Weaver audio chain implemented by two quadrature oscillators
 *   and two IIR lowpass filters 
 *   For more info see: 
 *   http://csoundjournal.com/ezine/summer2000/processing/
 *   http://www.pa3ect.eu/start/weaver-derde-methode-ssb-visueel-uitgelegd/
 *   Summers, H. (2015): Weaver article library. – online: http://www.hanssummers.com/weaver/weaverlib [2015-12-06]
 *   Weaver, D. (1956): A Third Method of Generation of Single-Sideband Signals. – Proceedings of the IRE, Dec 1956.
 * *************************************************************************/
    // take the mono input signal and mix it with the I & Q outputs of a quadrature oscillator
    // running at (sampling frequency / 16) = 2757.4Hz 
      freq_conv1(); // complex shift by fs/16

    // I & Q are now filtered with a lowpass filter with Fstop = 2750Hz --> which will lead to a Bandwidth of 2 x 2750Hz = 5.5kHz  
    // The lowpass filters are identical IIR biquad filters with 5 stages = 10th order
      arm_biquad_cascade_df1_f32 (&biquad_lowpass2_L, float_buffer_L,float_buffer_L_3, BUFFER_SIZE);
      arm_biquad_cascade_df1_f32 (&biquad_lowpass2_R, float_buffer_R,float_buffer_R_3, BUFFER_SIZE);

    // The second quadrature oscillator runs at (sampling frequency / 16) PLUS the desired offset frequency
    // the two outputs of the oscillator (which are 90 degrees apart) are complex multiplied with I & Q
      freq_conv2(); 
    // We then take I & Q and simply add them together, the result is in float_buffer_R and is copied to float_buffer_L for mono output
      arm_add_f32(float_buffer_R, float_buffer_L, float_buffer_R, BUFFER_SIZE); // add I & Q
      arm_copy_f32 (float_buffer_R, float_buffer_L, BUFFER_SIZE); // copy to left audio chain --> Mono output of shifted signal

// THAT WAS THE WEAVER CHAIN !
// potential applications are:
// - Frequency shifting for passband tuning
// - SSB demodulation in Software defined radios without the need for phase shifting Hilbert transforms
// - 
      
/* 
  // decimation by 4 --> 44118 / 4 = 11029.5 sps, means that a 4.2kHz decimation filter is fine [should have -80dB at 5.5kHz]
      //  decimation filter FIR 80 taps, lowpass 4.2kHz, Parks McClellan, Window off, Transition width 0.1, 80dB stopband attenuation
      arm_fir_decimate_f32(&FIR_dec1_L, float_buffer_L, float_buffer_L_2, BUFFER_SIZE);
      arm_fir_decimate_f32(&FIR_dec1_R, float_buffer_R, float_buffer_R_2, BUFFER_SIZE);
      
      // test filter 1 stage
//      arm_biquad_cascade_df1_f32 (&biquad_lowpass1_L, float_buffer_L_2,float_buffer_L_3, BUFFER_SIZE / DECIMATION_FACTOR);
//      arm_biquad_cascade_df1_f32 (&biquad_lowpass1_R, float_buffer_R_2,float_buffer_R_3, BUFFER_SIZE / DECIMATION_FACTOR);

      // test filter 5 to 8 stages
//      arm_biquad_cascade_df1_f32 (&biquad_lowpass2_L, float_buffer_L_3,float_buffer_L_2, BUFFER_SIZE / DECIMATION_FACTOR);
//      arm_biquad_cascade_df1_f32 (&biquad_lowpass2_R, float_buffer_R_3,float_buffer_R_2, BUFFER_SIZE / DECIMATION_FACTOR);


        
       // test FIR filter 200 taps
//     arm_fir_f32(&FIR_lowpass1_L,float_buffer_L_3, float_buffer_L_2, BUFFER_SIZE / DECIMATION_FACTOR);
//     arm_fir_f32(&FIR_lowpass1_R,float_buffer_R_3, float_buffer_R_2, BUFFER_SIZE / DECIMATION_FACTOR);

  // interpolation by 4
      arm_fir_interpolate_f32(&FIR_int1_L, float_buffer_L_2, float_buffer_L, BUFFER_SIZE / DECIMATION_FACTOR);
      arm_fir_interpolate_f32(&FIR_int1_R, float_buffer_R_2, float_buffer_R, BUFFER_SIZE / DECIMATION_FACTOR);

    // this IIR filter works in 44118sps ! Quite expensive in terms of CPU power
//      arm_biquad_cascade_df1_f32 (&biquad_lowpass2_L, float_buffer_L_3,float_buffer_L, BUFFER_SIZE);
//      arm_biquad_cascade_df1_f32 (&biquad_lowpass2_R, float_buffer_R_3,float_buffer_R, BUFFER_SIZE);

      // scaling after interpolation ? multiply volume by DECIMATION_FACTOR seems to be a reasonable figure (output = input volume)
      arm_scale_f32 (float_buffer_R,(float32_t) DECIMATION_FACTOR,float_buffer_R, BUFFER_SIZE);
      arm_scale_f32 (float_buffer_L,(float32_t) DECIMATION_FACTOR,float_buffer_L, BUFFER_SIZE);
*/      
/**************************************************************************
 * END of 32 bit float audio processing
 * ************************************************************************
 */
    sp_L = Q_out_L.getBuffer();
    sp_R = Q_out_R.getBuffer();
    arm_float_to_q15 (float_buffer_L, sp_L, BUFFER_SIZE); 
    arm_float_to_q15 (float_buffer_R, sp_R, BUFFER_SIZE); 
      Q_out_L.playBuffer(); // play it !
      Q_out_R.playBuffer(); // play it !


/**********************************************************************************
 *  FFT
 **********************************************************************************/
     if (FFT_state) {

//     arm_cfft_f32(S, FFT_buffer, 0, 1);
      }
/**********************************************************************************
 *  PRINT ROUTINE FOR ELAPSED MICROSECONDS
 **********************************************************************************/
 
      sum = sum + usec;
      idx_t++;
      if (idx_t > 1000) {
          tft.fillRect(240,50,90,20,ILI9341_BLACK);   
          tft.setCursor(240, 50);
          mean = sum / idx_t;
          tft.print (mean);
          Serial.print (mean);
          Serial.print (" microsec for 2 stereo blocks    ");
          Serial.println();
          idx_t = 0;
          sum = 0;
         
      }

     }
/**********************************************************************************
 *  PRINT ROUTINE FOR AUDIO LIBRARY PROCESSOR AND MEMORY USAGE
 **********************************************************************************/
          if (five_sec.check() == 1)
    {
      Serial.print("Proc = ");
      Serial.print(AudioProcessorUsage());
      Serial.print(" (");    
      Serial.print(AudioProcessorUsageMax());
      Serial.print("),  Mem = ");
      Serial.print(AudioMemoryUsage());
      Serial.print(" (");    
      Serial.print(AudioMemoryUsageMax());
      Serial.println(")");
      Serial.print("Cleared the audio buffer ");    
      Serial.print(n_clear); Serial.println (" times. ");

/*      tft.fillRect(100,120,200,80,ILI9341_BLACK);
      tft.setCursor(10, 120);
      tft.setTextSize(2);
      tft.setTextColor(ILI9341_WHITE);
      tft.setFont(Arial_14);
      tft.print ("Proc = ");
      tft.setCursor(100, 120);
      tft.print (AudioProcessorUsage());
      tft.setCursor(180, 120);
      tft.print (AudioProcessorUsageMax());
      tft.setCursor(10, 150);
      tft.print ("Mem  = ");
      tft.setCursor(100, 150);
      tft.print (AudioMemoryUsage());
      tft.setCursor(180, 150);
      tft.print (AudioMemoryUsageMax());
     */ 
      AudioProcessorUsageMaxReset();
      AudioMemoryUsageMaxReset();
    }
   spectrum();
}


 void spectrum() { // spectrum analyser code by rheslip - modified
     if (myFFT.available()) {
    int scale;
    scale = 2;
  for (int16_t x=2; x < 100; x+=2) {

     int bar = (abs(myFFT.output[x]) * scale);
     if (bar >180) bar=180;
     // this is a very simple IIR filter to smooth the reaction of the bars
     bar = 0.2 * bar + 0.8 * barm[x]; 
     if (bar > peak[x]) peak[x]=bar;
//     tft.drawFastVLine(x, 210-bar,bar, ILI9341_PURPLE);
     tft.drawFastVLine(x*2+10, 210-bar,bar, ILI9341_PINK);

     tft.drawFastVLine(x*2+10, 20, 210-bar-20, ILI9341_BLACK);    

     tft.drawPixel(x*2+10,209-peak[x], ILI9341_YELLOW);

     if(peak[x]>0) peak[x]-=1;
     barm[x] = bar;
  }
  } //end if

   } // end void spectrum

/*************************************************************************************************
 *  FREQUENCY CONVERSION USING A SOFTWARE QUADRATURE OSCILLATOR
 *  
 *  THIS VERSION USES A PRECALCULATED COS AND SIN WAVE AND IS VERY FAST AND EFFICIENT
 *  
 *  MAJOR DRAWBACK: frequency conversion can only be done at sub-multiples of the sampling frequency
 * 
 *  large parts of the code taken from the mcHF code by Clint, KA7OEI, thank you!
 *    see here for more info on quadrature oscillators: 
 *  Wheatley, M. (2011): CuteSDR Technical Manual Ver. 1.01. - http://sourceforge.net/projects/cutesdr/
 *  Lyons, R.G. (2011): Understanding Digital Processing. – Pearson, 3rd edition.  
 *************************************************************************************************/
/////////////////////////////////////////////////////////////////////
// Sine/cosine generation function
// Adjust rad_calc *= 32;  p.e. ( 44100 / 128 * 32 = 11025 khz)
// this can only generate frequencies in sub-multiples of the SAMPLE_RATE !
/////////////////////////////////////////////////////////////////////
/**
 * 
 */
void freq_conv1()
{
  uint     i;
  float32_t rad_calc;

  if (!sine_flag1)  {  // have we already calculated the sine wave?
    for (i = 0; i < BUFFER_SIZE; i++) {  // No, let's do it!
      rad_calc = (float32_t)i;    // convert to float the current position within the buffer
      rad_calc /= (BUFFER_SIZE);     // make this a fraction
      rad_calc *= (PI * 2);     // convert to radians
      rad_calc *= IF_FREQ1 * 128 / AUDIO_SAMPLE_RATE_EXACT;      // multiply by number of cycles that we want within this block ( 44100 / 128 * 32 = 11025 khz)
      //
      Osc1_Q_buffer [i] = arm_cos_f32(rad_calc);  // get sine and cosine values and store in pre-calculated array
      Osc1_I_buffer [i] = arm_sin_f32(rad_calc);  // they are in float32_t format
    }
    sine_flag1 = 1; // signal that once we have generated the quadrature sine waves, we shall not do it again
  }

    // Do frequency conversion using optimized ARM math functions [KA7OEI]
    // there seems to be something wrong here: I is real! Q is imaginary --> corrected, DD4WH
    arm_mult_f32(float_buffer_L, Osc1_Q_buffer, c1_buffer, BUFFER_SIZE); // multiply products for converted I channel
    arm_mult_f32(float_buffer_R, Osc1_I_buffer, d1_buffer, BUFFER_SIZE);
    arm_mult_f32(float_buffer_L, Osc1_I_buffer, e1_buffer, BUFFER_SIZE);
    arm_mult_f32(float_buffer_R, Osc1_Q_buffer, f1_buffer, BUFFER_SIZE);    // multiply products for converted Q channel

    if(!dir1)    // Conversion is "above" on RX (LO needs to be set lower)
    {
        arm_add_f32(f1_buffer, e1_buffer, float_buffer_R, BUFFER_SIZE); // summation for I channel
        arm_sub_f32(c1_buffer, d1_buffer, float_buffer_L, BUFFER_SIZE); // difference for Q channel
    }
    else    // Conversion is "below" on RX (LO needs to be set higher)
    {
        arm_add_f32(c1_buffer, d1_buffer, float_buffer_L, BUFFER_SIZE); // summation for I channel
        arm_sub_f32(f1_buffer, e1_buffer, float_buffer_R, BUFFER_SIZE); // difference for Q channel
    }

} // end freq_conv1()

/*************************************************************************************************
 *  freq_conv2()
 *  
 *  FREQUENCY CONVERSION USING A SOFTWARE QUADRATURE OSCILLATOR
 *  
 *  THIS VERSION calculates the COS AND SIN WAVE on the fly AND IS SLOW AND INEFFICIENT
 *  
 *  MAJOR ADVANTAGE: frequency conversion can be done for any frequency !
 * 
 *  large parts of the code taken from the mcHF code by Clint, KA7OEI, thank you!
 *    see here for more info on quadrature oscillators: 
 *  Wheatley, M. (2011): CuteSDR Technical Manual Ver. 1.01. - http://sourceforge.net/projects/cutesdr/
 *  Lyons, R.G. (2011): Understanding Digital Processing. – Pearson, 3rd edition.  
 *************************************************************************************************/

// INPUT  I = float_buffer_L_3
// INPUT  Q = float_buffer_R_3
// OUTPUT I = float_buffer_L
// OUTPUT Q = float_buffer_R
/*
void set_freq_conv2(float32_t NCO_FREQ) {
//  float32_t NCO_FREQ = AUDIO_SAMPLE_RATE_EXACT / 16; // + 20;
float32_t NCO_INC = 2 * PI * NCO_FREQ / AUDIO_SAMPLE_RATE_EXACT;
float32_t OSC_COS = cos (NCO_INC);
float32_t OSC_SIN = sin (NCO_INC);
float32_t Osc_Vect_Q = 1.0;
float32_t Osc_Vect_I = 0.0;
float32_t Osc_Gain = 0.0;
float32_t Osc_Q = 0.0;
float32_t Osc_I = 0.0;
float32_t i_temp = 0.0;
float32_t q_temp = 0.0;
}
*/
void freq_conv2()
{
  uint     i;
      for(i = 0; i < BUFFER_SIZE; i++) {
        // generate local oscillator on-the-fly:  This takes a lot of processor time!
        Osc_Q = (Osc_Vect_Q * OSC_COS) - (Osc_Vect_I * OSC_SIN);  // Q channel of oscillator
        Osc_I = (Osc_Vect_I * OSC_COS) + (Osc_Vect_Q * OSC_SIN);  // I channel of oscillator
        Osc_Gain = 1.95 - ((Osc_Vect_Q * Osc_Vect_Q) + (Osc_Vect_I * Osc_Vect_I));  // Amplitude control of oscillator
        // rotate vectors while maintaining constant oscillator amplitude
        Osc_Vect_Q = Osc_Gain * Osc_Q;
        Osc_Vect_I = Osc_Gain * Osc_I;
        //
        // do actual frequency conversion
        float_buffer_L[i] = (float_buffer_L_3[i] * Osc_Q) + (float_buffer_R_3[i] * Osc_I);  // multiply I/Q data by sine/cosine data to do translation
        float_buffer_R[i] = (float_buffer_R_3[i] * Osc_Q) - (float_buffer_L_3[i] * Osc_I);
        //
      }
} // end freq_conv2()
 
