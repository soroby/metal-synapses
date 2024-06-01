// Import required libraries
#include <Wire.h>        // Library for I2C communication
#include <MPR121.h>      // Library for interacting with the MPR121 capacitive touch sensor
#include "MIDIUSB.h"   // Library for sending MIDI messages over USB
#include <Adafruit_NeoPixel.h> //include library

// #include <Arduino.h>


#define numElectrodes 12// Define the number of touch electrodes (unused in the code)
#define PIN        6 //the pin you will communicate with the LEDs along

// How many NeoPixels are attached to the Arduino? How many are on the strip?
#define NUMPIXELS 40

//int analogInput = MPR121.getFilteredData(1);

// variables:
int sensorValue = 0;  // the sensor value
int sensorMin = 1023;  // minimum sensor value
int sensorMax = 0;// maximum sensor value
int sensorValue2 = 0;  // the sensor value
int sensorMin2 = 1023;  // minimum sensor value
int sensorMax2 = 0;
int sensorValue3 = 0;  // the sensor value
int sensorMin3 = 1023;  // minimum sensor value
int sensorMax3 = 0;
int sensorValue4 = 0;  // the sensor value
int sensorMin4 = 1023;  // minimum sensor value
int sensorMax4 = 0;
int sensorValue5 = 0;  // the sensor value
int sensorMin5 = 1023;  // minimum sensor value
int sensorMax5 = 0;
 
// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals.
//the last parameter is the common setting for Neopixel. 
//However, this may need changing if you have different LED strip drivers for different brands or older versions
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);

//byte currentVelocity = 64; // this gives it an initial velocity so we can change the velocity

// MPR121 mpr121;

// Function to send a MIDI note-on message
// Parameters:
// - channel: MIDI channel (0-15)
// - pitch: MIDI note number (e.g. 48 for middle C)
// - velocity: Velocity of the note (e.g. 64 for normal, 127 for fastest)

void noteOn(byte channel) { // we took out byte velocity so it's only expecting these two bytes
//   // Create a MIDI event packet for the note-on message
  midiEventPacket_t noteOn = {0x09 | channel};
//   // Send the MIDI event packet over USB
   MidiUSB.sendMIDI(noteOn);
   // // this is to changes the control and number is the volume // we could do this manually in ableton 
}

// Function to send a MIDI note-off message
// Parameters:
// - channel: MIDI channel (0-15)
// - pitch: MIDI note number (e.g. 48 for middle C)
// - velocity: Velocity of the note (e.g. 64 for normal)

void noteOff(byte channel) {
//   // Create a MIDI event packet for the note-off message
   midiEventPacket_t noteOff = {0x08 | channel};
//   // Send the MIDI event packet over USB
  MidiUSB.sendMIDI(noteOff);
 }

// Setup function, called once when the program starts
void setup() {
  //int analogInput = MPR121.getFilteredData(1);
  
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
        
  strip.setBrightness(100);
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);

  // Initialize the MPR121 capacitive touch sensor
  MPR121.begin(0x5A);  // I2C address of the sensor is 0x5A
  MPR121.setInterruptPin(4); // Set interrupt pin (you can change this pin if needed)
  MPR121.setTouchThreshold(40); // Set touch threshold (sensitivity)
  MPR121.setReleaseThreshold(30); // Set release threshold
   
  
}

// Function to send a MIDI control change message
// Parameters:
// - channel: MIDI channel (0-15)
// - control: Control number (0-119)
// - value: Control value (0-127)
void controlChange(byte channel, byte control, byte value) { // this decides the control its going to affect
  // Create a MIDI event packet for the control change message
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  // Send the MIDI event packet over USB
  MidiUSB.sendMIDI(event);
}

// Loop function, called repeatedly in a loop
void loop() {
 
  strip.clear(); // Set all pixel colors to 'off'
  // Update touch data from the MPR121 sensor
  MPR121.updateTouchData();
  MPR121.updateFilteredData();

  //int analogInput = constrain(MPR121.getFilteredData(1), 550, 650); // variable to get data from pin 1
  int analogInput = MPR121.getFilteredData(1);
 
  sensorValue = MPR121.getFilteredData(1);

    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  
  sensorValue = MPR121.getFilteredData(1);
  
  sensorValue = constrain(sensorValue, sensorMin, sensorMax);
  Serial.print(sensorValue);
  Serial.print(',');
  Serial.print(sensorMin);
  Serial.print(',');
  Serial.println(sensorMax);
  int currentVelocity = map(sensorValue, sensorMin, sensorMax, 127, 0); //(analogInput, analogInput (from serial monitor) to Midi values)
  
  // Check if electrode 1 is touched
  if (MPR121.getTouchData(1)) {
     // If touched, send a MIDI note-on message (channel 0, pitch 48, velocity 64)
     noteOn (1);
     controlChange (1, 81, currentVelocity);
   } else {
     // If not touched, send a MIDI note-off message (channel 0, pitch 48, velocity 64)
     noteOff(1);
  }

  int analogInput2 = MPR121.getFilteredData(2);
 
  sensorValue2 = MPR121.getFilteredData(2);

    // record the maximum sensor value
    if (sensorValue2 > sensorMax2) {
      sensorMax2 = sensorValue2;
    }

    // record the minimum sensor value
    if (sensorValue2 < sensorMin2) {
      sensorMin2 = sensorValue2;
    }
  
  sensorValue2 = MPR121.getFilteredData(2);
  
  sensorValue2 = constrain(sensorValue2, sensorMin2, sensorMax2);
  Serial.print(sensorValue2);
  Serial.print(',');
  Serial.print(sensorMin2);
  Serial.print(',');
  Serial.println(sensorMax2);
  int currentVelocity2 = map(sensorValue2, sensorMin2, sensorMax2, 127, 0); //(analogInput, analogInput (from serial monitor) to Midi values)
  
   if (MPR121.getTouchData(2)) {
     // If touched, send a MIDI note-on message (channel 0, pitch 48, velocity 64)
    noteOn(2);
    controlChange (2, 7, currentVelocity2);}
    else {
   // If not touched, send a MIDI note-off message (channel 0, pitch 48, velocity 64)
    noteOff(2);
  }

  int analogInput3 = MPR121.getFilteredData(3);
 
  sensorValue3 = MPR121.getFilteredData(3);

    // record the maximum sensor value
    if (sensorValue3 > sensorMax3) {
      sensorMax3 = sensorValue3;
    }

    // record the minimum sensor value
    if (sensorValue3 < sensorMin3) {
      sensorMin3 = sensorValue3;
    }
  
  sensorValue3 = MPR121.getFilteredData(3);
  
  sensorValue3 = constrain(sensorValue3, sensorMin3, sensorMax3);
  Serial.print(sensorValue3);
  Serial.print(',');
  Serial.print(sensorMin3);
  Serial.print(',');
  Serial.println(sensorMax3);
  int currentVelocity3 = map(sensorValue3, sensorMin3, sensorMax3, 127, 0); //(analogInput, analogInput (from serial monitor) to Midi values)
  
  // Check if electrode 1 is touched
  if (MPR121.getTouchData(3)) {
     // If touched, send a MIDI note-on message (channel 0, pitch 48, velocity 64)
     noteOn (3);
     controlChange (3, 81, currentVelocity3);
   } else {
     // If not touched, send a MIDI note-off message (channel 0, pitch 48, velocity 64)
     noteOff(3);
  }

  int analogInput4 = MPR121.getFilteredData(4);
 
  sensorValue4 = MPR121.getFilteredData(4);

    // record the maximum sensor value
    if (sensorValue4 > sensorMax4) {
      sensorMax4 = sensorValue4;
    }

    // record the minimum sensor value
    if (sensorValue4 < sensorMin4) {
      sensorMin4 = sensorValue4;
    }
  
  sensorValue4 = MPR121.getFilteredData(4);
  
  sensorValue4 = constrain(sensorValue4, sensorMin4, sensorMax4);
  Serial.print(sensorValue4);
  Serial.print(',');
  Serial.print(sensorMin4);
  Serial.print(',');
  Serial.println(sensorMax4);
  int currentVelocity4 = map(sensorValue4, sensorMin4, sensorMax4, 127, 0); //(analogInput, analogInput (from serial monitor) to Midi values)
  
  // Check if electrode 1 is touched
  if (MPR121.getTouchData(4)) {
     // If touched, send a MIDI note-on message (channel 0, pitch 48, velocity 64)
     noteOn (4);
     controlChange (4, 81, currentVelocity4);
   } else {
     // If not touched, send a MIDI note-off message (channel 0, pitch 48, velocity 64)
     noteOff(4);
  }

  int analogInput5 = MPR121.getFilteredData(5);
 
  sensorValue5 = MPR121.getFilteredData(5);

    // record the maximum sensor value
    if (sensorValue5 > sensorMax5) {
      sensorMax5 = sensorValue5;
    }

    // record the minimum sensor value
    if (sensorValue5 < sensorMin5) {
      sensorMin5 = sensorValue5;
    }
  
  sensorValue5 = MPR121.getFilteredData(5);
  
  sensorValue5 = constrain(sensorValue5, sensorMin5, sensorMax5);
  Serial.print(sensorValue5);
  Serial.print(',');
  Serial.print(sensorMin5);
  Serial.print(',');
  Serial.println(sensorMax5);
  int currentVelocity5 = map(sensorValue5, sensorMin5, sensorMax5, 127, 0); //(analogInput, analogInput (from serial monitor) to Midi values)
  
  // Check if electrode 1 is touched
  if (MPR121.getTouchData(5)) {
     // If touched, send a MIDI note-on message (channel 0, pitch 48, velocity 64)
     noteOn (5);
     controlChange (5, 81, currentVelocity5);
   } else {
     // If not touched, send a MIDI note-off message (channel 0, pitch 48, velocity 64)
     noteOff(5);
  }
  MidiUSB.flush();
  //   if (MPR121.getTouchData(8)) {
  //   // If touched, send a MIDI note-on message (channel 0, pitch 48, velocity 64)
  //   noteOn(2, 63);}
  //   else {
  //   // If not touched, send a MIDI note-off message (channel 0, pitch 48, velocity 64)
  //   noteOff(2, 63);
  // }
  //  if (MPR121.getTouchData(10)) {
  //   // If touched, send a MIDI note-on message (channel 0, pitch 48, velocity 64)
  //   noteOn(2, 63);}
  //   else {
  //   // If not touched, send a MIDI note-off message (channel 0, pitch 48, velocity 64)
  //   noteOff(2, 63);
  // }
 int rgbvals[] = {16711680, 16744448, 16776960, 65280, 65535, 255, 11141375, 16711935};
// rgbvals[0] = 16711680;
// rgbvals[1] = 16744448;
// rgbvals[2] = 16776960;
// rgbvals[3] = 65280;
// rgbvals[4] = 65535;
// rgbvals[5] = 255;
// rgbvals[6] = 11141375;
// rgbvals[7] = 16711935;
 
 //int rgb1 = map(analogInput, 550, 650, 0, 255);
// int rgb2 = map(analogInput, 550, 650, 0, 100);
 int i = map(analogInput, 250, 350, 0, 7);
 strip.fill(rgbvals[i], 0, 30);
 strip.show();
 delay(1);  
// Serial.println(i);
  // Uncomment the following line to test control change messages
  // controlChange(0, 10, 65); // Set the value of controller 10 on channel 0 to 65
  MidiUSB.flush();
}