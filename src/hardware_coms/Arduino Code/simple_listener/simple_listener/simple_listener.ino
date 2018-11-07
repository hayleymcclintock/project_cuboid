
#include <math.h>


int incomingByte = 0;


void setup() {
  Serial.begin(9600);
  
}

void loop() {

  // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                
                Serial.println(incomingByte);
                
                delay(5);
        }

}
