// This driver is for testing pneumatic regulator(s).
// Assumes your PIC firmware will send, in sequence, the memory located at [the most recent
// sent index] upon a read request.
// Writes state to and reads pressure from a pressure regulator.
// Typical parameters a user wants to edit are marked by **USER**

#include <Wire.h>

// System configuration variables
const int NUM_ACTUATORS = 1; // **USER**
int pressureRegulatorAddresses[NUM_ACTUATORS] = {38,39,40,41}; // **USER**  (we will have 4, corresponding to each face of the cube)

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

float expConst = .9; // **USER** Decay (time) constant for the (optional) exponential filte    0.9 is rather agressive. 0 is same as no filter. r

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(2000000);  // start serial for output
  currentMillis = millis(); // Initial time

  for (int actuator = 0; actuator < NUM_ACTUATORS; actuator++) {
    writeI2C(pressureRegulatorAddresses[actuator], INDEX_OPERATION_MODE, controlMode); // Send pic firmware mode to the actuator
    writeI2C(pressureRegulatorAddresses[actuator], INDEX_HALF_BAND, halfBand); // Send inHalfBand to the actuator
    writeI2C(pressureRegulatorAddresses[actuator], INDEX_ONLY_SEND_PRESSURE, onlySendPressure); // Send "don't only send pressure" to the actuator
  }
}






void loop() {


  // WILL EDIT THIS SECTION TO ACCOUNT FOR ALL DIFFERENT PRESSURE REGULATORS AND ROS INTERFACING 
  if (receive_ros_command == 1) {    // once we get the signal from ros on our subscriber here on the arduino, we will fire the pressure regulator of interest 
    changeState = true;
  }

  // find which actuator we want to pressurize and read from  
  if (someotherroscommand == 1) {
    actuator = 1
  }
  else if (someotherroscommand == 2) {
    actuator = 2
  }
  else if (someotherroscommand == 3) {
    actutor = 3
  }
  else {
    actuator = 4
  }

  // Write to and read from slaves
  readPressureRegulator(pressureRegulatorAddresses[actuator], filteredValues[actuator], expConst, values[actuator]); 

  // Change command if it is a "command cycle"
  if (changeState) {
    
      command = 2  // Only valid states are 0,1,2. 2 is inflate. 1 is hold. 0 is release

      writeI2C(pressureRegulatorAddresses[actuator], INDEX_DESIRED_STATE, command); // Send command to the actuator 
         // maybe publish that we sent a command to ros so we can keep tabs on it???? 
  }
  

  finishCycle();
}

void finishCycle() {
  changeState = false;
  delay(cycleMillis);
  Serial.print(command);
  Serial.print("\t");
  Serial.println("");
  for (int actuator = 0; actuator < NUM_ACTUATORS; actuator++) {
    Serial.print(values[actuator]); Serial.print("\t");
  }
}

// Communicate with Pressure Regulator **************************************

void readPressureRegulator(const int &inAddress, float &inFiltered, const float &inA, float &inReading) {
  // Read the pressure regulator's air-pressure sensor, and filter it. This function specifically reads pressure, not a generic memory address.

  // Tell the PIC which address to read from, if we are not in "onlySendPressure" mode
  if (!onlySendPressure) {
    Wire.beginTransmission(inAddress); //Prepare internal variables
    Wire.write(INDEX_PRESSURE_SENSOR_HIGH_BYTE);        // write memory location to buffer
    Wire.endTransmission();    // transmit
  }

  // Request data
  Wire.requestFrom(inAddress, 2); byte b1_local = Wire.read(); byte b2_local = Wire.read();
  inReading = ((float)b1_local * 256 + (float)b2_local) / pow(2, 6); // Only keep 10 high bits. Convert to voltage
  inReading = tenBitToVolt(inReading);
  inReading = voltToPsi(inReading);
  inFiltered = (inA * inFiltered + (1 - inA) * inReading); //filter the noise, exponential filter

  return inReading  // or in Filtered, whatever you woudl like to see 

}

void writeI2C(int inAddress, int inIndex, int inValue) {
  Wire.beginTransmission(inAddress); //Prepare internal variables
  Wire.write(inIndex);        // write memory locaiton to buffer
  Wire.write(inValue);        // write value to buffer
  Wire.endTransmission();    // transmit
}
// End communicate with Pressure Regulator **************************************

// Unit conversions  **************************************

float tenBitToVolt(float inTenBit) {
  // Convert a 10-bit "5V source ADC reading" to voltage.
  // Specifically, our PIC has a supply voltage of 5V and uses a 10 bit ADC.
  float volts = inTenBit * 5 / pow(2, 10);
  return volts;
}

unsigned long voltToTenBit(float inVolts) {
  // Convert from voltage to a ten bit "5V source ADC reading", scaled as 2^10 distinct values over 5 volts.
  // Specifically, our PIC has a supply voltage of 5V and uses a 10 bit ADC.
  unsigned long tenBit = inVolts * pow(2, 10) / 5;
  return tenBit;
}

float voltToPsi(float inVolts) {
  // Convert voltage reading to pressure [psi]
  // This calibration function is specific to our pressure regulator. "ASDX RR X 030PD A A 5". Read from Joran's notebook.
  float outPsi;
  float pressureMax = 30;
  float pressureMin = -30;
  float vS = 5; // supply voltage
  float transferFunctionLimits = 0.1;
  float psi = ((inVolts - transferFunctionLimits * vS) * (pressureMax - pressureMin)) / (0.8 * vS) + pressureMin;
  return psi;
}

float psiToVolt(float inPsi) {
  // Convert pressure to voltage reading [V]
  // This calibration function is specific to our pressure regulator. "ASDX RR X 030PD A A 5". Read from Joran's notebook.
  float outPsi;
  float pressureMax = 30;
  float pressureMin = -30;
  float vS = 5; // supply voltage
  float transferFunctionLimits = 0.1;
  float volt = 0.8 * vS / (pressureMax - pressureMin) * (inPsi - pressureMin) + transferFunctionLimits * vS;
  return volt;
}

float psiToPa(float inPsi) {
  //Convert [psi] to [Pa]. From Google.
  float pa = inPsi * 6894.76;
  return pa;
}

