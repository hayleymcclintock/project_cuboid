#include <Servo.h>
#include <math.h>

Servo crane;
Servo boom;

int relay_pin = 3;

int vertical_pos = 110;
int out_pos = 170;

int crane_up = 172;
int crane_down = 0;

int incomingByte = 0;

int lastByte =49;

void setup() {
  boom.attach(10);  // attaches the servo on pin 9 to the servo object
  boom.write(vertical_pos);
  Serial.begin(9600);
  crane.attach(9);
  crane.write(crane_down);
  
  pinMode(relay_pin, OUTPUT);
}

void attach_object(){
      boom.write(out_pos);
      delay(1000);
      crane.write(crane_up);
}

void move_crane_down() {
     crane.write(crane_down);
     //There is already a delay in attach_object
}


void detach_object(){
      boom.write(vertical_pos);
      delay(700);
      crane.write(crane_down);
  
}

void loop() {

  // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                
                if (incomingByte == 48 && lastByte ==48 ) {
                   move_crane_down();
                   attach_object();
                  }
                else if (incomingByte ==48) { //send a 0 to attach to the object
                  attach_object();
                  }
                else if (incomingByte == 49) { //send a 1 to release from the object
                  detach_object();
                  }
                else if (incomingByte == 57) { //reset the power on the servos
                  digitalWrite(relay_pin, HIGH);
                  delay(3000);
                  digitalWrite(relay_pin, LOW);
                  }
                lastByte = incomingByte;
                
        }

}
