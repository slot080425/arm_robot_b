#include <ESP32Servo.h>

Servo servo[5];

const float L0 = 11.0, L1 = 12.0, L2 = 13.0, L3 = 9.0; //cm???? (permisalan)
                                                       
// Urutan: 0 = basis, 1 = bahu, 2 = siku, 3 = pergelangan, 4 = gripper
const int pinServo[5] = {13,14,27,26,25};

float servoDeg[5];

// Kalibrasi: servoDeg = arah * sudut + offset
// offset = sudut servo saat link mendatar, arah = +1 / -1 jika terbalik


const float GRIPPER_BUKA  = 60.0;
const float GRIPPER_TUTUP = 120.0;

float arah[4]   = {1, 1, 1, 1};
float offset[4] = {90, 90, 90, 90};
float sudut[5];
float servoNow[5] = {90, 90, 90, 90, 90};

float x,y,z;
float tx, ty, tz;   // target IK
float phi = 0;  // arah gripper dari horizontal: 0 mendatar, 90 atas, -90 bawah
bool  bisa = false; // true jika IK punya solusi
int mode = -1; // 0 = target, 1 = sudut kinematika, 2 = sudut servo
               
String laporan;

void forwardKinematics(){
  float t0 = sudut[0] * DEG_TO_RAD;
  float t1 = sudut[1] * DEG_TO_RAD;
  float t2 = sudut[2] * DEG_TO_RAD;
  float t3 = sudut[3] * DEG_TO_RAD;

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


void sudutKeServo(){
  for (int i = 0; i < 4; i++) servoDeg[i] = arah[i] * sudut[i] + offset[i];
}

void servoKeSudut(){
  for (int i = 0; i < 4; i++) sudut[i] = (servoDeg[i] - offset[i]) / arah[i];
}

// Servo hanya menerima 0-180; target di luar itu ditolak, servo tidak digerakkan
bool cekBatas(){
  if (!bisa) return false;
  for (int i = 0; i < 4; i++) {
    if (servoDeg[i] < 0 || servoDeg[i] > 180) return false;
  }
  return true;
}

void gerakServo(){
  // Gerakan dilakukan perlahan agar tidak terlalu cepat pergerakannya
  bool masihGerak = true;
  while (masihGerak) {
    masihGerak = false;
    for (int i = 0; i < 4; i++) {
      if (servoNow[i] < servoDeg[i]) {
        servoNow[i] = min(servoNow[i] + 1, servoDeg[i]);
        masihGerak = true;
      } else if (servoNow[i] > servoDeg[i]) {
        servoNow[i] = max(servoNow[i] - 1, servoDeg[i]);
        masihGerak = true;
      }
      servo[i].write((int)servoNow[i]);
    }
    delay(5);
  }
}

void nolkanPosisi(){
  for (int i = 0; i < 5; i++) sudut[i] = 0;
  sudutKeServo();
  for (int i = 0; i < 4; i++) {
    servoNow[i] = servoDeg[i];
    servo[i].write((int)servoNow[i]);
  }
  servoNow[4] = GRIPPER_BUKA;
  servo[4].write((int)servoNow[4]);
}

void setGripper(float g){
  g = constrain(g, 0, 180);
  servoNow[4] = g;
  servo[4].write((int)g);
}

bool proses(int m){
  if (m == 0)      { inverseKinematics(); sudutKeServo(); }
  else if (m == 1) { bisa = true; sudutKeServo(); }
  else             { bisa = true; servoKeSudut(); }

  if (!cekBatas()) {
    laporan = "Target tidak bisa dicapai (di luar jangkauan atau melebihi batas servo)";
    return false;
  }

  forwardKinematics();                      // pada mode 0, hasilnya harus ≈ target
  laporan = "Sudut kinematika: ";
  for (int i = 0; i < 4; i++) laporan += String(sudut[i], 1) + " ";
  laporan += "\nSudut servo     : ";
  for (int i = 0; i < 4; i++) laporan += String(servoDeg[i], 1) + " ";
  laporan += "\nFK              : " + String(x, 1) + ", " + String(y, 1) + ", " + String(z, 1);
  return true;
}

// Serial: "tx ty tz phi" | "s a b c d" (kinematika) | "v a b c d" (servo) | "g sudut"
void readInput(){
  mode = -1;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char c = line.charAt(0);

  if (c == 'g') {
    setGripper(line.substring(1).toFloat());
    Serial.println("Gripper: " + String(servoNow[4], 0));
    return;
  }

  if (c == 's' || c == 'v') {
    float *d = (c == 's') ? sudut : servoDeg;
    if (sscanf(line.c_str() + 1, "%f %f %f %f", &d[0], &d[1], &d[2], &d[3]) == 4) {
      mode = (c == 's') ? 1 : 2;
    } else {
      Serial.println("Format salah. Contoh: s 0 30 45 -20");
    }
    return;
  }

  if (sscanf(line.c_str(), "%f %f %f %f", &tx, &ty, &tz, &phi) == 4) {
    mode = 0;
  } else {
    Serial.println("Format salah. Contoh: 20 10 15 0");
  }
}

void setup(){
  Serial.begin(115200);
  Serial.setTimeout(50);
  for(int i = 0; i < 5; i++) servo[i].attach(pinServo[i]);
  nolkanPosisi();
  Serial.println("Siap. Perintah:");
  Serial.println("  tx ty tz phi   (target IK, contoh: 20 10 15 0)");
  Serial.println("  s a b c d      (sudut kinematika)");
  Serial.println("  v a b c d      (sudut servo langsung)");
  Serial.println("  g sudut        (gripper)");
}

void loop(){
  if (!Serial.available()) return;
  readInput();
  if (mode < 0) return;

  if (proses(mode)) {
    Serial.println(laporan);
    gerakServo();
  } else {
    Serial.println(laporan);
  }

}
