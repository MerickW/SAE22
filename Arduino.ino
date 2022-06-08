/*
  Example of use of the FFT library to compute FFT for a signal sampled through the ADC.
        Copyright (C) 2018 Enrique Condés and Ragnar Ranøyen Homb
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1280; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

bool continuous = false;
bool binary = false;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

const int bufferLength = 64;
int count = 5;
int messageLength = 5;
int delayTime = 10;
char tab[bufferLength];
bool reading = true;


void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");

   // initialize digital pin LEDs as an output.
  for (int i = 2; i < 11; i++) {
    pinMode(i, OUTPUT);
  }

  // initialize digital pin built-in LED as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
}

void loop()
{
  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  /* Print the results of the sampling according to time */
  //WARNING, this portion doesn't work well with the tracer.
  //Use the monitor instead to get precise data.
//  Serial.println("Data:");
//  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
//  Serial.println("Weighed data:");
//  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
//  Serial.println("Computed Real values:");
//  PrintVector(vReal, samples, SCL_INDEX);
//  Serial.println("Computed Imaginary values:");
//  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  if (continuous) {
    Serial.println("Computed magnitudes:");
    PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  
    Serial.println(x, 6); //Print out what frequency is the most dominant.
    Serial.println("");
  }
   
   for (int i = 2; i < 11; i++) {
    digitalWrite(i, LOW);
   }

   if (!reading) {
    count = 0;
    tab[count] = 0;
   }

   digitalWrite(LED_BUILTIN, LOW);
   if (!binary && !continuous) {
     if (x > 145.0 && x < 150.0) { //This turns on the LED for the dominant frequency.
      digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
      tab[count] = '1';
     } else if (x > 160.0 && x < 170.0 ) {
      digitalWrite(3, HIGH);
      tab[count] = '2';
     } else if (x > 173.0 && x < 179.0 ) {
      digitalWrite(4, HIGH);
      tab[count] = '3';
     } else if (x > 195.0 && x < 200.0 ) {
      digitalWrite(5, HIGH);
      tab[count] = '4';
     } else if (x > 219.0 && x < 224.0 ) {
      digitalWrite(6, HIGH);
      tab[count] = '5';
     } else if (x > 246.0 && x < 251.0 ) {
      digitalWrite(7, HIGH);
      tab[count] = '6';
     } else if (x > 261.0 && x < 265.0 ) {
      digitalWrite(8, HIGH);
      tab[count] = '7';
      digitalWrite(LED_BUILTIN, HIGH);
     } else if (x > 293.0 && x < 297.0 ) {
      digitalWrite(9, HIGH);
      tab[count] = '8';
      digitalWrite(LED_BUILTIN, HIGH);
     } else if (x > 328.0 && x < 334.0 ) {
      digitalWrite(10, HIGH);
      tab[count] = '9';
      digitalWrite(LED_BUILTIN, HIGH);
     }
     else if (x > 430 && x < 450) {
      tab[count] = '0';
      digitalWrite(LED_BUILTIN, HIGH);
     }
   }
   
   /*
    if (!continuous && binary) {
      for (int i=12; i<50; i++) {
        if(vReal[i] > 600) {
          int freq = i * 10;
          Serial.println(freq);
          if ((freq == 150)){
//            tab[count] += 1;
            digitalWrite(2, HIGH);
          } else if ((freq == 170) ) {
//            tab[count] += 2;
            digitalWrite(3, HIGH);
          } else if ((freq == 180)) {
//            tab[count] += 4;
            digitalWrite(4, HIGH);
          } else if ((freq == 200)) {
//            tab[count] += 8;
            digitalWrite(5, HIGH);
          } else if ((freq == 220)) {
//            tab[count] += 16;
            digitalWrite(6, HIGH);
          } else if ((freq == 240)) {
//            tab[count] += 32;
            digitalWrite(7, HIGH);
          } else if ((freq == 260)) {
//            tab[count] += 64;
            digitalWrite(8, HIGH);
          } else if (freq == 290) {
            //tab[count] += 128;
            digitalWrite(9, HIGH);
          } else if (freq == 330 || freq == 340) {
            digitalWrite(10, HIGH);
          }
        }
      }
    }
*/
  if (!continuous && binary) {
    int detect = 350;
    if (vReal[14] > detect || vReal[15] > detect-200) {
      tab[count] += 1;
      digitalWrite(2, HIGH);
    }
    if (vReal[16] > detect) {
      tab[count] += 2;
      digitalWrite(3, HIGH);
    }
    if (vReal[18] > detect) {
      tab[count] += 4;
      digitalWrite(4, HIGH);
    }
    if (vReal[20] > detect) {
      tab[count] += 8;
      digitalWrite(5, HIGH);
    }
    if (vReal[22] > detect || vReal[23] > detect-200) {
      tab[count] += 16;
      digitalWrite(6, HIGH);
    }
    if (vReal[24] > detect) {
      tab[count] += 32;
      digitalWrite(7, HIGH);
    }
    if (vReal[26] > detect) {
      tab[count] += 64;
      digitalWrite(8, HIGH); 
    }
  }
    if (tab[0] != 0) {
      reading = true;
    }

    microseconds = micros();
    count++;
    if (count >= messageLength && !continuous) {
      for (int i = 0; i < messageLength; i++) {
        Serial.print(tab[i]);
      }
      Serial.println("");
      if (messageLength == 26 && binary) Serial.println("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
      count = 0;
      reading = false;
      String temp = "";
      delayTime = -1;
      Serial.println("Input delay (-1 continuous, -2 mode, -3 mLength)");
      while(delayTime < 0) {
        if (Serial.available()) {
          temp = Serial.readString();
          delayTime = max(temp.toInt(), 110); //Min delay = 110 ms
          if (temp.toInt() == -1) {
            continuous = true;
          }
          if (temp.toInt() == -2) {
            binary = !binary;
            delayTime = -1;
            if (binary) Serial.println("Binary mode");
            else Serial.println("Direct mode");
          }
          if (temp.toInt() == -3) {
            Serial.println("Enter new message length (1, 63)");
            messageLength = 0;
            while(messageLength < 1) {
              delayTime = -1;
              temp = Serial.readString();
              messageLength = temp.toInt();
              if (messageLength > 63) messageLength = 63;
            }
            Serial.println(messageLength);
          }
          temp = "";
        }
      }
      for (int i = 0; i < bufferLength; i++) {
        tab[i] = 0;
      }
      Serial.println("reading");
    }
    //Only delay when we have collected data
    if (tab[0] != 0)
      delay(delayTime-100); //Delay
    if (Serial.available()) {
      count = 4;
      continuous = false;
      Serial.readString();
    }


  //  while(1); /* Run Once */
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    
    if(scaleType==SCL_FREQUENCY && vData[i] > 300.0) {
      Serial.print(abscissa, 6);
      Serial.print("Hz");
      Serial.print(" ");
      Serial.println(vData[i], 4);
    }
  }
  Serial.println();
}
