#include <driver/i2s.h>
#include "FFT.h" // include the library
#include "FFT_signal.h"
#include "hsv_to_rgb.h"
#include "led.h"


const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 1024;

#define bufferLen 64
int16_t sBuffer[bufferLen];
#define baseline 0
double maximum = 8000;
int ffMAXSIZE = 64;
double ffArr[64];
int top = 0;
int last = 0;
char print_buf[300];
float vals[3];

// float lmao_freq = 0;
// float lmao_mag = 0;

fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
HSV hsv;
RGB rgb;

int i2s_read_b(i2s_port_t i2s_num, void *dest, size_t size, TickType_t ticks_to_wait)
{
    size_t bytes_read = 0;
    int res = 0;
    res = i2s_read(i2s_num, dest, size, &bytes_read, ticks_to_wait);
    if (res != ESP_OK) {
        return ESP_FAIL;
    } else {
        return bytes_read;
    }
}

void push(double data, int last) {
  // double *everything_but_last = ffArr;
  // double *secondHalf = ffArr + (ffMAXSIZE - 1);

  // double a[] = {data};

  // double *result = new double[ffMAXSIZE];
  // std::copy(a, a + 1, result);
  // std::copy(everything_but_last, everything_but_last + (ffMAXSIZE - 1), result + 1);


  // ffArr[0] = a[0];

  // for(int i = 1; i < ffMAXSIZE; i++) {
  //   ffArr[i] = result[i];
  // }

}

int readMic() {
  int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");

  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
 
  if (result == ESP_OK)
  {
    // Read I2S data buffer
    int16_t samples_read = bytesIn / 8;
    if (samples_read > 0) {
      float mean = 0;
      for (int16_t i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }
 
      // Average the data reading
      mean /= samples_read;
 
      // Print to serial plotter
      return(mean + 9000) / 3;
    }
  }
}

void readMicNoiseLevel() {
  int32_t samples[BLOCK_SIZE];
  size_t bytes_read = 0;
  
  int num_bytes_read = i2s_read_b(
                                      I2S_PORT, 
                                      (char *)samples, 
                                      BLOCK_SIZE,     // the doc says bytes, but its elements.
                                      portMAX_DELAY); // no timeout

  int samples_read = num_bytes_read / 8;
  if (samples_read > 0) {

    float mean = 0;
    for (int i = 0; i < samples_read; ++i) {
      mean += (samples[i]);
    }
    mean /= samples_read;


    float fftDataReal_[samples_read];
    //normalize values
    for (uint16_t i = 0; i < samples_read; i++) {
        // Corrected input value: Subtract the block average from each sample in order remove the DC component
        int16_t v = samples[i] - mean;

        // Constant for normalizing int16 input values to floating point range -1.0 to 1.0
        const float kInt16MaxInv = 1.0f / __INT16_MAX__;

        // Input value in floating point representation
        float r;

        // Compute input value for FFT
        r = kInt16MaxInv * v;

        /*
        // Generate test signal
        const float k2Pi = 6.2831853f;
        const float k2PiSampleCountInv = k2Pi * kFFT_SampleCountInv;
        
        r = sinf( k2PiSampleCountInv * (testSignalFreqFactor_ * i) );
        */
        
        // Store value in FFT input array
        fftDataReal_[i] = r;
    }

    for (int k = 0 ; k < FFT_N ; k++) {
      real_fft_plan->input[k] = samples[k];
    }

      //Execute transformation
    fft_execute(real_fft_plan);

    float mag;
    float freq;
    max_magnitude = 1e-8;
    for (int k = 1 ; k < real_fft_plan->size / 2 ; k++)
    {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
      mag = sqrt(pow(real_fft_plan->output[2*k],2) + pow(real_fft_plan->output[2*k+1],2))/1;
      
      freq = k*1.0/TOTAL_TIME;
  //    sprintf(print_buf,"%f Hz : %f", freq, mag);
  //    Serial.println(print_buf);
      if(mag > max_magnitude)
      {
        max_magnitude = mag;
        fundamental_freq = freq;
      }
    }

    

    mean = 0;
    for (int i = 0; i < samples_read; ++i) {
      mean += (fftDataReal_[i]);
    }
    mean = mean / samples_read;

    /*
    Serial.print("Pressure:");
    Serial.print( mean );
    Serial.print(",");
    Serial.print("Frequency:");
    Serial.print( fundamental_freq );
    Serial.print(",");
    Serial.print("Magnitude:");
    Serial.println((max_magnitude/10000)*2/FFT_N);
 */
    vals[0] = mean;
    vals[1] = fundamental_freq;
    vals[2] = (max_magnitude/100000)*2/FFT_N;
  }
}

int readMicSoundPressure() {
  int32_t samples[BLOCK_SIZE];
  int num_bytes_read = i2s_read_b(I2S_PORT, 
                                      (char *)samples, 
                                      BLOCK_SIZE,     // the doc says bytes, but its elements.
                                      portMAX_DELAY); // no timeout
  
  int samples_read = num_bytes_read / 8;
  if (samples_read > 0) {

    float mean = 0;
    for (int i = 0; i < samples_read; ++i) {
      mean += (samples[i] >> 14);
    }
    mean /= samples_read;

    float maxsample = -1e8, minsample = 1e8;
    for (int i = 0; i < samples_read; ++i) {
      minsample = min(minsample, samples[i] - mean);
      maxsample = max(maxsample, samples[i] - mean);
    }
    return ((maxsample - minsample) / 2147483647) * 8000;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Configuring I2S...");

  led_setup();

  esp_err_t err;

  // The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
      .sample_rate = 16000,                         // 16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // although the SEL config should be left, it seems to transmit on right
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 8,                           // number of buffers
      .dma_buf_len = BLOCK_SIZE                     // samples per buffer
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = 14,   // BCKL
      .ws_io_num = 15,    // LRCL
      .data_out_num = -1, // not used (only for speakers)
      .data_in_num = 32   // DOUT
  };

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    exit(0);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    exit(0);
    while (true);
  }
  Serial.println("I2S driver installed.");

  //============================================

  max_magnitude = 1e-8;
  fundamental_freq = 0;

}

void loop() {

    //float pressure = readMicNoiseLevel();
    const double SONIC_SPEED = 343;
    const double MIN_AMP = 0;
    const double MAX_AMP = 4000;
    readMicNoiseLevel();
    float wavelength = min((SONIC_SPEED / (vals[1]) / 11.0), 300.0);
    float brightness = min((vals[2] - MIN_AMP) / (MAX_AMP - MIN_AMP), 1.0);

    // Serial.println(vals[0]);
    // Serial.println(vals[1]);
    
    
    // Serial.print("wavelength: ");
    // Serial.print(wavelength);
    // Serial.print(", brightness: ");
    // Serial.println(brightness);
    hsv.H=360.0 - wavelength;
    hsv.S=1.00;
    hsv.V=brightness;
    RGB temp = HSVToRGB(hsv);
    rgb.R = temp.R;
    rgb.G = temp.G;
    rgb.B = temp.B;
    if (brightness > 0.1) {
      Serial.print("wavelength: ");
      Serial.print(wavelength);
      Serial.print(", brightness: ");
      Serial.println(brightness);
      Serial.print("(R,G,B) = (");
      Serial.print(rgb.R);
      Serial.print(",");
      Serial.print(rgb.G);
      Serial.print(",");
      Serial.print(rgb.B);
      Serial.println(")");
      set_rgb(rgb.R, rgb.G, rgb.B);
    }
    // Serial.print("(R,G,B) = (");
    // Serial.print(rgb.R);
    // Serial.print(",");
    // Serial.print(rgb.G);
    // Serial.print(",");
    // Serial.println(rgb.B);
    // Serial.println(")");    
}
