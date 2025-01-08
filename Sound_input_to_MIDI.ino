/*  Listens to mic or audio line input, does FFT, turns that into MIDI output note
 *  
 *  OLED Screen pins on Uno: 
 *  SCL --> A5
 *  SDA --> A4
 *  
 *  OLED Screen pins on Leonardo:
 *  SCL --> D3 (digital pin3)
 *  SDA --> D2 (digital pin2)
 *  
 *  OLED Screen pins on ESP32:
 *  SCL --> 22  
 *  SDA --> 21
 *    
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <mtof.h>
#include <HardwareSerial.h>
#include "arduinoFFT.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// esp32 start TX2 for MIDI.
HardwareSerial SerialPort(2);  //if using UART2
#define RX_pin 16  
#define TX_pin 17  
#define MIC_pin 34 

#define LED_PIN 2 
#define POT_PIN 4

String start_msg = "FFT - MIDI";

int potValue = 0;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// FFT stuff======================================
#define SAMPLES 256              //SAMPLES-pt FFT. Must be a base-2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 1024  //Ts = Based on Nyquist, must be 2 times the highest expected frequency. We only want frequencies below ~500 Hz for long pad notes from synths.

arduinoFFT FFT = arduinoFFT();
 
unsigned int samplingPeriod;
unsigned long microSeconds;
   
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values
 

void setup() {
  // Various setup==============
  pinMode(LED_PIN, OUTPUT);

  // esp32 start TX2 for MIDI====
  SerialPort.begin(31250, SERIAL_8N1, 16, 17);  // baudrate, SERIAL_8N1, rx_pin, tx_pin

  // FFT setup===================
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds   

  // OLED display setup==========
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(1000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 30);
  display.println(start_msg); 
  display.display();
  delay(1000);
}

void loop() {
  potValue = analogRead(POT_PIN);  // the pot determines the length of the note
  potValue = map(potValue, 0, 4095, 1, 32);  // maps the pot value to a range of 1-32 beats at 175bpm

  int note_total_time = potValue * 343; // 343 is one beat at 175bpm

  int note_off_time = 172; // 171.5ms is length of eighth note at 175bpm
  int note_on_time = note_total_time - note_off_time;
    
  double freq = FFT_calculation();

  int midi_note = int(mtof.toPitch(freq));
  
  Serial.print(freq);
  Serial.print(" -- ");
  Serial.println(midi_note);
 
  display_info(freq, midi_note, potValue);
  
  noteOn(0x90, midi_note, 0x45);
  digitalWrite(LED_PIN, HIGH);
  delay(note_on_time);
  noteOff(0x90, midi_note);
  digitalWrite(LED_PIN, LOW);
  delay(note_off_time);
}

void display_info(int value1=0, int value2=0, int value3=0){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Freq: ");  
  if (value1 >= 10000){   // If nothing is connected to the input pin, the freq and midi values display a really large number
    display.print("NA");
  }
  else{
    display.print(value1);
  }
  display.setCursor(0, 20);
  display.print("MIDI: ");
  if (value2 >= 10000){
    display.print("NA");
  }
  else{
    display.print(value2);
  }
  display.setCursor(0, 40);
  display.print("Time: ");
  display.print(value3);
  display.print("b");
  display.display();
}

double FFT_calculation(){
  for(int   i=0; i<SAMPLES; i++) {
    microSeconds = micros();
  
    vReal[i] = analogRead(MIC_pin); //Reads the value from analog input pin, quantize it and save it as a real term.
    vImag[i] = 0; //Makes   imaginary term 0 always

    /*remaining wait time between samples if necessary*/
    while(micros() < (microSeconds + samplingPeriod)) {
        //do nothing
    }
  }

  /*Perform FFT on samples*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  /*Find peak frequency and print peak*/
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  //Serial.println(peak);     //Print out the most dominant frequency.
  return peak;
}

void noteOn(int cmd, int pitch, int velocity) {
  // send MID not on message
  SerialPort.write(cmd);
  SerialPort.write(pitch);
  SerialPort.write(velocity);
}

void noteOff(int cmd, int pitch) {
  // send MID not off message
  int velocity = 0x00;
  SerialPort.write(cmd);
  SerialPort.write(pitch);
  SerialPort.write(velocity);
}


