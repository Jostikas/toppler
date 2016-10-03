#include <Servo.h>
#include <Wire.h>
#include <Arduino_I2C_ESC.h>

Servo servo1; 
Arduino_I2C_ESC motor(0x29);
unsigned int j = 0;
void setup() {
  Wire.begin();
  pinMode(2,OUTPUT);
  
  servo1.attach(2); //analog pin 0
  Serial.begin(19200);
  Serial.println("R");
  
  Serial.println("Finally, it works!");
  
  motor.set(0);
  motor.set(0);
  motor.set(0);
  
}
int v2 = 0;
int autoo = 0;

void loop() {
  static int v = 0;
  //motor.update();
  //motor.rpm()
  if ( Serial.available()) {
    char ch = Serial.read();
  
    switch(ch) {
      case '0'...'9':
        v = v * 10 + ch - '0';
        Serial.print("wrote: ");
        Serial.println(v);
        break;
      case 's':
        servo1.write(v);
        motor.set(v);
        Serial.print("set: ");
        Serial.println(v);
        v2 = v;
        v = 0;
        autoo = 0;
        break;
      case '-':
        v *= -1;
        Serial.print("wrote: ");
        Serial.println(v);
        break;
      
    }
    
  }
  if(j++ %4000 == 0){
    Serial.print("rpm: ");
    motor.update();
    Serial.println(motor.rpm());
    motor.set(v2);
  }
} 
