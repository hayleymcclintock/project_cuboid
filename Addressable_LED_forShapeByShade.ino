// script written by rob 11/5, in progress to get shapes by shades. 
// the goal is to toggle sections of the LED strip once some condition (ie. a couple secondds, while the cube
// face is stable on the ground and the camera is aligned well) has been met. 
// it will flash in 4 sections, holding the light for ~5 sec, turning it off for ~2 sec, turning the next
// on for ~5 sec, and so on. 

#include "FastLED.h"

// how much power are we going to draw? we have 30 led/meter. and 1 meter. 
// putting power supply at 5v 1.5 a should be more than sufficient! 
 
// How many leds in your strip?
#define NUM_LEDS 30 // this will change once we cut it for each individual pin toy 
 
#define DATA_PIN 6
#define CLOCK_PIN 7
 
CRGB leds[NUM_LEDS]; // set up block of memory used for storing and manipulating LED data 
 
void setup() { 
     FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

}
 
void loop() {

  int section_size = 5; // number of lights in a section (wrapped on one of 4 sides of pin toy)
  int on_duration = 5000; // how long in ms we want the LED turned on 
  int off_duration = 2000; // how long is ms the LED is turned off before next one is turned on 
  int current_face_num = current_face_num++; // the number of the face on cube we've rolled to 
  int section_cnt = 0; // index of the first led in the section
  
  for(int i=0; i<4; i++){ // for each of the 4 sides of pin toy ie. sections, turn on and off the LED strips
    turn_on_led(on_duration, off_duration, section_size, section_cnt);
    turn_off_led(on_duration, off_duration, section_size, section_cnt);

    if (section_cnt < NUM_LEDS) {
      section_cnt = section_cnt + section_size; // starts at led section with first one 0, goes to first one as 5, and so on until 29. 
    }
  }
}

//////////////////////Helper Functions 

// turns on the led and then waits 
int turn_on_led(int inOnDuration, int inOffDuration, int inSectionSize, int inSectionCnt) { 
  
    for(int i=inSectionCnt; i<inSectionSize+inSectionCnt; i++){ 
        leds[i] = CHSV(0, 10, 255); //pretty bright white (HSV Color input) 
        FastLED.show();
    }
    delay(inOnDuration);
}

// turn off section and wait 2 sec 
int turn_off_led(int inOnDuration, int inOffDuration, int inSectionSize, int inSectionCnt) {

    for(int i=inSectionCnt; i<inSectionSize+inSectionCnt; i++){  
        leds[i] = CHSV(0,0,0);  // no color 
        FastLED.show(); 
    }
    delay(inOffDuration);
}
//////////////////////////////////////////////////////////////////////////////
