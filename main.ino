#include <ESP32Servo.h>

Servo servo[5];

// L0 = dasar-bahu, L1 = bahu-siku, L2 = siku-pergelangan, L3 = pergelangan-gripper (cm)
const float L0 = 11.0, L1 = 12.0, L2 = 13.0, L3 = 9.0;

const int pinServo[5] = {13, 14, 27, 26, 25};

// servoDeg = arah * offset + sudut
float arah[4]   = {1, 1, 1, 1};
float offset[4] = {90, 90, 90, 90};

const float GRIPPER_BUKA  = 60.0;
const float GRIPPER_TUTUP = 120.0;

float sudut[5]; // sudut fk                          
float servoDeg[5];                           
float servoNow[5] = {90, 90, 90, 90, 90};   

float x, y, z; // output fk                              
float tx, ty, tz; // input ik                           
bool  bisa = false; // ik ketemu solusi = true                  
int   mode = -1; // 0 = input, 1 = sudut fk, 2 = sudut servo                             
String laporan;

void fkPos(const float q[4], float p[3]){
  float a1 = q[1];
  float a2 = q[1] + q[2];
  float a3 = q[1] + q[2] + q[3];

  float r  = L1 * cos(a1) + L2 * cos(a2) + L3 * cos(a3);
  float zz = L0 + L1 * sin(a1) + L2 * sin(a2) + L3 * sin(a3);

  p[0] = r * cos(q[0]);
  p[1] = r * sin(q[0]);
  p[2] = zz;
}

void forwardKinematics(){
  float q[4];
  for (int i = 0; i < 4; i++) q[i] = sudut[i] * DEG_TO_RAD;
  float p[3];
  fkPos(q, p);
  x = p[0]; y = p[1]; z = p[2];
}

void jacobian(const float q[4], float J[3][4]){
  float a[4]  = {0, q[1], q[1] + q[2], q[1] + q[2] + q[3]};
  float Ls[4] = {0, L1, L2, L3};
  float c0 = cos(q[0]), s0 = sin(q[0]);

  float r = 0;
  for (int i = 1; i <= 3; i++) r += Ls[i] * cos(a[i]);

  J[0][0] = -r * s0;  J[1][0] = r * c0;  J[2][0] = 0;

  for (int j = 1; j <= 3; j++) {
    float rj = 0, zj = 0;
    for (int i = j; i <= 3; i++) {
      rj -= Ls[i] * sin(a[i]);
      zj += Ls[i] * cos(a[i]);
    }
    J[0][j] = c0 * rj;
    J[1][j] = s0 * rj;
    J[2][j] = zj;
  }
}

float det3(float A[3][3]){
  return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
       - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
       + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
}

bool solve3(float A[3][3], const float b[3], float s[3]){
  float d = det3(A);
  if (fabs(d) < 1e-9) return false;
  for (int c = 0; c < 3; c++) {
    float M[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) M[i][j] = (j == c) ? b[i] : A[i][j];
    s[c] = det3(M) / d;
  }
  return true;
}

bool iterasi(float q[4], const float target[3], const float lo[4], const float hi[4]){
  const float lambda = 0.5, tol = 0.1, langkahMax = 0.3;
  for (int it = 0; it < 300; it++) {
    float p[3];
    fkPos(q, p);
    float e[3] = {target[0] - p[0], target[1] - p[1], target[2] - p[2]};
    if (sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]) < tol) return true;

    float J[3][4];
    jacobian(q, J);

    float A[3][3];                                          
    for (int i = 0; i < 3; i++)
      for (int k = 0; k < 3; k++) {
        A[i][k] = (i == k) ? lambda * lambda : 0;
        for (int j = 0; j < 4; j++) A[i][k] += J[i][j] * J[k][j];
      }

    float s[3];
    if (!solve3(A, e, s)) return false;

    float dq[4], norma = 0;
    for (int j = 0; j < 4; j++) {
      dq[j] = J[0][j] * s[0] + J[1][j] * s[1] + J[2][j] * s[2];
      norma += dq[j] * dq[j];
    }
    norma = sqrt(norma);
    float skala = (norma > langkahMax) ? langkahMax / norma : 1.0;   

    for (int j = 0; j < 4; j++) q[j] = constrain(q[j] + dq[j] * skala, lo[j], hi[j]);
  }
  return false;
}

void inverseKinematics(){
  bisa = false;
  float target[3] = {tx, ty, tz};

  float lo[4], hi[4];
  for (int i = 0; i < 4; i++) {
    float a = (0   - offset[i]) / arah[i] * DEG_TO_RAD;
    float b = (180 - offset[i]) / arah[i] * DEG_TO_RAD;
    lo[i] = min(a, b);
    hi[i] = max(a, b);
  }

  const float awal[6][3] = {
    {0, 0, 0},
    {0.5, -1.0, 0.5}, {1.0, -1.0, -0.3}, {0.3, 1.0, -1.0},
    {0.8, -0.6, -0.8}, {1.2, -0.8, -1.2}
  };

  for (int n = 0; n < 6; n++) {
    float q[4];
    if (n == 0) {
      for (int i = 0; i < 4; i++) q[i] = (servoNow[i] - offset[i]) / arah[i] * DEG_TO_RAD;
      if (fabs(q[2]) < 0.1) { q[1] = 0.5; q[2] = -1.0; q[3] = 0.5; }
    } else {
      for (int j = 1; j <= 3; j++) q[j] = awal[n][j - 1];
    }
    q[0] = atan2(ty, tx);
    for (int i = 0; i < 4; i++) q[i] = constrain(q[i], lo[i], hi[i]);

    if (iterasi(q, target, lo, hi)) {
      for (int i = 0; i < 4; i++) sudut[i] = q[i] * RAD_TO_DEG;
      bisa = true;
      return;
    }
  }
}

void sudutKeServo(){
  for (int i = 0; i < 4; i++) servoDeg[i] = arah[i] * sudut[i] + offset[i];
}

void servoKeSudut(){
  for (int i = 0; i < 4; i++) sudut[i] = (servoDeg[i] - offset[i]) / arah[i];
}

bool cekBatas(){
  if (!bisa) return false;
  for (int i = 0; i < 4; i++) {
    if (servoDeg[i] < -0.01 || servoDeg[i] > 180.01) return false;   
  }
  return true;
}

void gerakServo(){
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
    delay(15);
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

  forwardKinematics();                      
  laporan = "Sudut kinematika: ";
  for (int i = 0; i < 4; i++) laporan += String(sudut[i], 1) + " ";
  laporan += "\nSudut servo     : ";
  for (int i = 0; i < 4; i++) laporan += String(servoDeg[i], 1) + " ";
  laporan += "\nFK              : " + String(x, 1) + ", " + String(y, 1) + ", " + String(z, 1);
  laporan += "\nArah gripper    : " + String(sudut[1] + sudut[2] + sudut[3], 1) + " deg dari horizontal";
  return true;
}

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

  if (sscanf(line.c_str(), "%f %f %f", &tx, &ty, &tz) == 3) {
    mode = 0;
  } else {
    Serial.println("Format salah. Contoh: 25 0 15");
  }
}

void setup(){
  Serial.begin(115200);
  Serial.setTimeout(50);

  for (int i = 0; i < 5; i++) servo[i].attach(pinServo[i]);
  nolkanPosisi();

  Serial.println("Siap. Perintah:");
  Serial.println("  tx ty tz       (target IK, contoh: 25 0 15)");
  Serial.println("  s a b c d      (sudut fk)");
  Serial.println("  v a b c d      (sudut servo)");
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