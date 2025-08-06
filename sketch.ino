/*
m1              m2
  \            /
    \        /
      \     /
        \  /
          X
        /  \
      /      \
    /          \
  /              \
m3                  m4


*/

#include <Servo.h>


#define lsig 2  // left
#define rsig 4  // right
#define fsig 7  // front
#define bsig 8  // back
Servo m1;
Servo m2;
Servo m3;
Servo m4;

const int dm1 = 9;
const int dm2 = 10;
const int dm3 = 11;
const int dm4 = 6;

void setup() {
  m1.attach(dm1);
  m2.attach(dm2);
  m3.attach(dm3);
  m4.attach(dm4);
  pinMode(lsig, INPUT);
  pinMode(rsig, INPUT);
  pinMode(fsig, INPUT);
  pinMode(bsig, INPUT);
  Serial.begin(9600);
  m1.writeMicroseconds(1000);
  m1.writeMicroseconds(2000);
  m2.writeMicroseconds(1000);
  m2.writeMicroseconds(2000);
  m3.writeMicroseconds(1000);
  m3.writeMicroseconds(2000);
  m4.writeMicroseconds(1000);
  m4.writeMicroseconds(2000);
  delay(3000);
  
}
void foront(int speed){
  m1.writeMicroseconds(speed);
  m2.writeMicroseconds(speed);
}
void back(int speed){
  m3.writeMicroseconds(speed);
  m4.writeMicroseconds(speed);
}
void right(int speed){
  m2.writeMicroseconds(speed);
  m4.writeMicroseconds(speed);
}
void left(int speed){
  m3.writeMicroseconds(speed);
  m1.writeMicroseconds(speed);
}
void forward(int hi, int lo){
  foront(lo);
  back(hi);
}
void backward(int hi, int lo){
  foront(hi);
  back(lo);
}
void rightward(int hi, int lo){
  right(lo);
  left(hi);
}
void leftward(int hi, int lo){
  right(hi);
  left(lo);
}
void ychange(int speed){
  back(speed);
  foront(speed);
}
int rbtn;
int lbtn;
int fbtn;
int bbtn;
bool rclk = false;
bool lclk = false;
bool fclk = false;
bool bclk = false;

void clockwise(int hi, int lo){
  m1.writeMicroseconds(hi);
  m2.writeMicroseconds(lo);
  m3.writeMicroseconds(hi);
  m4.writeMicroseconds(lo);
}
void anticlockwise(int hi, int lo){
  m1.writeMicroseconds(lo);
  m2.writeMicroseconds(hi);
  m3.writeMicroseconds(lo);
  m4.writeMicroseconds(hi);
}

void loop() {
  int hi = 2000;
  int lo = 1000;
  int mid = 1500;

  
}
