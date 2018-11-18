// master Arduino code with pressure regulator and LED utility. Waits for ROS controller to send it a char via serial, and then executes an action
// accoridng to that Char.
// coding in progress. Last edit: 11/12/18 at 3:32 pm : debugged in arduino IDE compiler. Have yet to test.

#include "FastLED.h" // library for LEDs 
// for the LEDs: D6 yellow  , D7  green, Orange Power Supply +5V , Black Power Supply GND, make sure to connect this and arduino GND
#include <math.h>
#include <Wire.h> // for I2C pressure reg 
//  for the pressure reg:  SLC 21 green, SLA 20 blue, black GND, red 5V



//LED CONFIGURATION  ---------------------------------------------------------------------
// how much power are we going to draw? we have 30 led/meter. and 1 meter.
// putting power supply at 5v 1.5 a should be more than sufficient!

// How many leds in your strip?
#define NUM_LEDS 60 // this will change once we string together the 4 strips and cut it... 
#define DATA_PIN 6
#define CLOCK_PIN 7
CRGB leds[NUM_LEDS]; // set up block of memory used for storing and manipulating LED data

//-----------------------------------------------------------------------------------------


//
//// P_REG CONFIGURATION  -------------------------------------------------------------------
// System configuration variables
const int NUM_ACTUATORS = 4; // **USER**
// we might end up using 8 actuators?
int pressureRegulatorAddresses[NUM_ACTUATORS] = {59,60,61,72}; // these are actuators 0, 1, 2 and 3 **USER**

// Memory Addresses
const int INDEX_SETPOINT_HIGH_BYTE = 16; // Default setpoint is atmospheric pressure (-0.147 [PSI])
const int INDEX_DESIRED_STATE = 18; // State. 0 = Release, 1 = Hold, 2 = Inflate. Our desired state. This is only active during manual mode.  Default valve state is hold (both valves off).
const int INDEX_PRESSURE_SENSOR_HIGH_BYTE = 19; //High byte of our actuator's pressure sensor
const int INDEX_HALF_BAND = 21; //  1/2*How wide our bang-bang deadband should be. Default Half-band is 10 [ADC values] = 2.94 [PSI]
const int INDEX_OPERATION_MODE = 22; // What mode should we operate in? automatic = 0. manual = 1. Default mode is manual
const int INDEX_ONLY_SEND_PRESSURE = 23; // If == 0, then the i2cArray can be read. ElseIf == 1, every read results in reading pressure. Default read mode is "only send pressure"

// 1) General variables
const int nBytes = 2; // How many bytes to read
float values[NUM_ACTUATORS]; // Array of received data
float filteredValues[NUM_ACTUATORS]; // Array of filtered sensor data
int iterState = 0; // State in the iteration (used for cycling between commands)
unsigned long currentMillis = 0.0;
unsigned long commandMillis = 1000; // How long to delay between commands
unsigned long cycleMillis = 10; // How long to delay between cycles
bool changeState = false;

// 2) Pressure Regulator Variables
const int AUTOMATIC = 0;
const int MANUAL = 1; //
int controlMode = MANUAL; // **USER** // manual is PIC waits for you to tell it with arduino or whatever command to open or close. automatic: you tell it pressure, it gets there.
bool onlySendPressure = 0; // **USER**
unsigned long command = 0;
unsigned long commandToSend = 0; // Use unsigned int to make allow bitshift operator
const int halfBand = 3;
float expConst = .9; // **USER** Decay (time) constant for the (optional) exponential filter. 0.9 is rather agressive. 0 is same as no filter.
//---------------------------------------------------------------------------------------------



void setup() {
  // LED SETUP --------------------------------------------------------------------------------
  FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS); // setup stuff for LED library
  // will have to add the other 4 leds here later...

  //-------------------------------------------------------------------------------------------

  //turn off all LED first
  turn_off_all_led();

  //   // P_REG SETUP -------------------------------------------------------------------------------
    Wire.begin();        // join i2c bus (address optional for master)
    Serial.begin(9600);  // start serial for output
  //  currentMillis = millis(); // Initial time
  //
  //  for (int actuator = 0; actuator < NUM_ACTUATORS; actuator++) {
  //    writeI2C(pressureRegulatorAddresses[actuator], INDEX_OPERATION_MODE, controlMode); // Send pic firmware mode to the actuator
  //    writeI2C(pressureRegulatorAddresses[actuator], INDEX_HALF_BAND, halfBand); // Send inHalfBand to the actuator
  //    writeI2C(pressureRegulatorAddresses[actuator], INDEX_ONLY_SEND_PRESSURE, onlySendPressure); // Send "don't only send pressure" to the actuator
  //  }
  //-------------------------------------------------------------------------------------------
}



void loop() {


  int section_size = 5; // how many lights are strung around one face of the pin toy ?

  // send data only when you receive data from ROS
  if (Serial.available() > 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    Serial.println(incomingByte);
    //P_REG SEND COMAND SECTION---------------------------------------------------------------------

    if (incomingByte == 1) {     //pressure reg 1 ON
      command = 2;
      writeI2C(pressureRegulatorAddresses[0], INDEX_DESIRED_STATE, command); // Send command to the actuator // Only valid states are 0,1,2. 2 is inflate. 1 is hold. 0 is release
    }
    else if (incomingByte == 5) { //pressure reg 1 OFF
      command = 0;
      writeI2C(pressureRegulatorAddresses[0], INDEX_DESIRED_STATE, command);
    }
    else if (incomingByte == 2) { //pressure reg 2 ON
      command = 2;
      writeI2C(pressureRegulatorAddresses[1], INDEX_DESIRED_STATE, command);
    }
    else if (incomingByte == 6) { //pressure reg 2 OFF
      command = 0;
      writeI2C(pressureRegulatorAddresses[1], INDEX_DESIRED_STATE, command);
    }
    else if (incomingByte == 3) { //pressure reg 3 ON
      command = 2;
      writeI2C(pressureRegulatorAddresses[2], INDEX_DESIRED_STATE, command);
    }
    else if (incomingByte == 7) { //pressure reg 3 OFF
      command = 0;
      writeI2C(pressureRegulatorAddresses[2], INDEX_DESIRED_STATE, command);
    }
    else if (incomingByte == 4) { //pressure reg 4 ON
      command = 2;
      writeI2C(pressureRegulatorAddresses[3], INDEX_DESIRED_STATE, command);
    }
    else if (incomingByte == 8) { //pressure reg 4 OFF
      command = 0;
      writeI2C(pressureRegulatorAddresses[3], INDEX_DESIRED_STATE, command);
    }
//    ------------------------------------------------------------------------------------------ -



    //LED SEND COMAND SECTION -------------------------------------------------------------------
    else if (incomingByte == 11) { //LED side 1 section 1 ON
      int section_size = 4; 
      turn_on_led(section_size, 0);
      turn_on_led(4, 33);
      Serial.println("called function");
      
    }
    else if (incomingByte == 15) { //LED side 1 section 1 oFF
      int section_size = 4; 
      turn_off_led(section_size, 0);
      turn_off_led(4, 33);

    }
    else if (incomingByte == 12) { //LED side 1 section 2 ON
      int section_size = 4; 
      turn_on_led(section_size, section_size * 2 - section_size - 1);
      turn_on_led(4, 37);

    }
    else if (incomingByte == 16) { //LED side 1 section 2 oFF
      int section_size = 4;
      turn_off_led(section_size, section_size * 2 - section_size - 1);
      turn_off_led(4, 37);

    }
    else if (incomingByte == 13) { //LED side 1 section 3 ON
      int section_size = 3; 
      turn_on_led(section_size, 8);//section_size * 3 - section_size - 1);
       turn_on_led(4, 41);//section_size * 3 - section_size - 1);

    }
    else if (incomingByte == 17) { //LED side 1 section 3 oFF
      int section_size = 3;       
      turn_off_led(section_size, 8);//section_size * 3 - section_size - 1);
       turn_off_led(4, 41);//section_size * 3 - section_size - 1);

    }
    else if (incomingByte == 14) { //LED side 1 section 4 ON
      int section_size = 4;
      turn_on_led(section_size, section_size * 4 - section_size - 1);
       turn_on_led(3, 30);//section_size * 3 - section_size - 1);

    }
    else if (incomingByte == 18) { //LED side 1 section 4 oFF
      int section_size = 4;
      turn_off_led(section_size, section_size * 4 - section_size - 1);
        turn_off_led(3, 30);//section_size * 3 - section_size - 1);

    }




    else if (incomingByte == 21) { //LED side 2 section 1 ON
      turn_on_led(section_size, section_size * 5 - section_size - 1);
    }
    else if (incomingByte == 25) { //LED side 2 section 1 OFF
      turn_off_led(section_size, section_size * 5 - section_size - 1);
    }
    else if (incomingByte == 22) { //LED side 2 section 2 ON
      turn_on_led(section_size, section_size * 6 - section_size - 1);
    }
    else if (incomingByte == 26) { //LED side 2 section 2 OFF
      turn_off_led(section_size, section_size * 6 - section_size - 1);
    }
    else if (incomingByte == 23) { //LED side 2 section 3 ON
      turn_on_led(section_size, section_size * 7 - section_size - 1);
    }
    else if (incomingByte == 27) { //LED side 2 section 3 OFF
      turn_off_led(section_size, section_size * 7 - section_size - 1);
    }
    else if (incomingByte == 24) { //LED side 2 section 4 ON
      turn_on_led(section_size, section_size * 8 - section_size - 1);
    }
    else if (incomingByte == 28) { //LED side 2 section 4 oFF
      turn_off_led(section_size, section_size * 8 - section_size - 1);
    }

    else if (incomingByte == 31) { //LED side 3 section 1 ON
      turn_on_led(section_size, section_size * 9 - section_size - 1);
    }
    else if (incomingByte == 35) { //LED side 3 section 1 Off
      turn_off_led(section_size, section_size * 9 - section_size - 1);
    }
    else if (incomingByte == 32) { //LED side 3 section 2 ON
      turn_on_led(section_size, section_size * 10 - section_size - 1);
    }
    else if (incomingByte == 36) { //LED side 3 section 2 Off
      turn_off_led(section_size, section_size * 10 - section_size - 1);
    }
    else if (incomingByte == 33) { //LED side 3 section 3 ON
      turn_on_led(section_size, section_size * 11 - section_size - 1);
    }
    else if (incomingByte == 37) { //LED side 3 section 3 Off
      turn_off_led(section_size, section_size * 11 - section_size - 1);
    }
    else if (incomingByte == 34) { //LED side 3 section 4 ON
      turn_on_led(section_size, section_size * 12 - section_size - 1);
    }
    else if (incomingByte == 38) { //LED side 3 section 4 Off
      turn_off_led(section_size, section_size * 12 - section_size - 1);
    }


    else if (incomingByte == 41) { //LED side 4 section 1 ON
      turn_on_led(section_size, section_size * 13 - section_size - 1);
    }
    else if (incomingByte == 45) { //LED side 4 section 1 Off
      turn_off_led(section_size, section_size * 13 - section_size - 1);
    }
    else if (incomingByte == 42) { //LED side 4 section 2 ON
      turn_on_led(section_size, section_size * 14 - section_size - 1);
    }
    else if (incomingByte == 46) { //LED side 4 section 2 Off
      turn_off_led(section_size, section_size * 14 - section_size - 1);
    }
    else if (incomingByte == 43) { //LED side 4 section 3 ON
      turn_on_led(section_size, section_size * 15 - section_size - 1);
    }
    else if (incomingByte == 47) { //LED side 4 section 3 Off
      turn_off_led(section_size, section_size * 15 - section_size - 1);
    }
    else if (incomingByte == 44) { // LED side 4 section 4 ON
      turn_on_led(section_size, section_size * 16 - section_size - 1);
    }
    else if (incomingByte == 48) { // LED side 4 section 4 off
      turn_off_led(section_size, section_size * 16 - section_size - 1);
    }
    //----------------------------------------------------------------------
  }
}


//Helper Functions LEDs ------------------------------------------------

// turns on the led section
int turn_on_led(int inSectionSize, int firstIndSec) {

  for (int i = firstIndSec; i < inSectionSize + firstIndSec; i++) {
    leds[i] = CHSV(0, 100, 100); //pretty bright white (HSV Color input) (0 , 10, 255) original white
    FastLED.show();
  }
}

// turn off led section
int turn_off_led(int inSectionSize, int firstIndSec) {

  for (int i = firstIndSec; i < inSectionSize + firstIndSec; i++) {
    leds[i] = CHSV(0, 0, 0); // no color
    FastLED.show();
  }
}


//turn off all led
int turn_off_all_led() {

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(0, 0, 0); //no color
    FastLED.show();
  }
}

//---------------------------------------------------------------------



//Helper function Pres_reg --------------------------------------------

void writeI2C(int inAddress, int inIndex, int inValue) {
  Wire.beginTransmission(inAddress); //Prepare internal variables
  Wire.write(inIndex);        // write memory locaiton to buffer
  Wire.write(inValue);        // write value to buffer
  Wire.endTransmission();    // transmit
}

//---------------------------------------------------------------------
