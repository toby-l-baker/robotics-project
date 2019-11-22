#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

#define TRIG_PIN_1  5
#define ECHO_PIN_1  6

#define SERVO_PIN_1 0

class RangeSensor{
  private:
    int trig_pin, echo_pin;
    long range;
  public:
    RangeSensor(int, int);
    long get_range();
    void update_range();    
};

RangeSensor::RangeSensor(int trig_pin, int echo_pin){
  this->trig_pin = trig_pin;
  this->echo_pin = echo_pin;
  this->range = 0L;
  pinMode(this->trig_pin, OUTPUT);
  pinMode(this->echo_pin, INPUT);
}

long RangeSensor::get_range(){
  return this->range;
}
void RangeSensor::update_range(){
  digitalWrite(this->trig_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(this->trig_pin, HIGH);
  this->range = pulseIn(this->echo_pin, HIGH)/2/29.1;
}



RangeSensor rs(TRIG_PIN_1, ECHO_PIN_1);
Servo sm;

void setup() {
    Serial.begin(9600);
    sm.attach(SERVO_PIN_1);
    
}

void loop() {

  

}
