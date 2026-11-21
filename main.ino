#include <ESP32Servo.h>

Servo servo[5];
float sudut[5];
float x,y,z;
const float L0 = 11.0, L1 = 12.0, L2 = 13.0, L3 = 9.0; //cm???? (permisalan)

void forwardKinematics(){
  float t0 = sudut[0] * PI / 180.0; // 0 = base, 1 = shoulder, 2 = elbow, 3 = wrist
  float t1 = sudut[1] * PI / 180.0;
  float t2 = sudut[2] * PI / 180.0;
  float t3 = sudut[3] * PI / 180.0;

  float a1 = t1;
  float a2 = t1 + t2;
  float a3 = t1 + t2 + t3;

  float r = (L1 * cos(a1)) + (L2 * cos(a2)) + (L3 * cos(a3));
  z = L0 + (L1 * sin(a1)) + (L2 * sin(a2)) + (L3 * sin(a3));
  x = r * cos(t0);
  y = r * sin(t0);
}

void inverseKinematics(){

}

void nolkanPosisi(){

}

void sudutKeServo(){

}

void readInput(){

}

bool cekBatas(){
  // Mengecek apakah posisi yang diharapkan itu bisa dicapai
}

void gerakServo(){

}

void setup(){

}

void loop(){

}
