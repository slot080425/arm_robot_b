#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

const char* WIFI_SSID = "m1su";
const char* WIFI_PASS = "250408zz";

Servo servo[5];
WebServer server(80);

// Panjang antar poros (cm): L0 = tinggi bahu dari dasar, L1 = bahu-siku,
// L2 = siku-pergelangan, L3 = pergelangan-ujung gripper
const float L0 = 11.0, L1 = 12.0, L2 = 13.0, L3 = 9.0;

// Urutan: 0 = basis, 1 = bahu, 2 = siku, 3 = pergelangan, 4 = gripper
const int pinServo[5] = {13, 14, 27, 26, 25};

// Kalibrasi: servoDeg = arah * sudut + offset
// offset = sudut servo saat link mendatar, arah = +1 / -1 jika terbalik
float arah[4]   = {1, 1, 1, 1};
float offset[4] = {90, 90, 90, 90};

const float GRIPPER_BUKA  = 60.0;
const float GRIPPER_TUTUP = 120.0;

const unsigned long LANGKAH_MS = 15;        // jeda tiap langkah 1 derajat (makin besar makin lambat)

float sudut[5];                             // sudut kinematika (derajat); hasil hitung sementara
float servoCalon[5];                        // hasil hitung yang belum tentu aman
float servoDeg[5];                          // tujuan aktif yang dikejar servo (sudah lolos cek)
float servoNow[5] = {90, 90, 90, 90, 90};   // posisi servo saat ini

float x, y, z;                              // hasil FK
float tx, ty, tz;                           // target IK
bool  bisa = false;                         // true jika IK menemukan solusi
String laporan;


// FK: posisi ujung dari sudut sendi q (radian), konvensi horizontal
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

// Jacobian 3x4: kolom j = gerak ujung per radian sendi j
void jacobian(const float q[4], float J[3][4]){
  float a[4]  = {0, q[1], q[1] + q[2], q[1] + q[2] + q[3]};
  float Ls[4] = {0, L1, L2, L3};
  float c0 = cos(q[0]), s0 = sin(q[0]);

  float r = 0;
  for (int i = 1; i <= 3; i++) r += Ls[i] * cos(a[i]);

  // basis: memutar ujung tegak lurus jari-jari, tinggi tidak berubah
  J[0][0] = -r * s0;  J[1][0] = r * c0;  J[2][0] = 0;

  // bahu, siku, pergelangan: tiap sendi menggerakkan link ke-j sampai ujung
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

// determinan dan penyelesaian sistem 3x3 (aturan Cramer)
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

// Satu percobaan IK dari tebakan awal q. true jika galat < toleransi.
bool iterasi(float q[4], const float target[3], const float lo[4], const float hi[4]){
  const float lambda = 0.5, tol = 0.1, langkahMax = 0.3;   // tol dalam cm, langkah dalam radian

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
    float skala = (norma > langkahMax) ? langkahMax / norma : 1.0;   // batasi besar langkah

    for (int j = 0; j < 4; j++) q[j] = constrain(q[j] + dq[j] * skala, lo[j], hi[j]);
  }
  return false;
}


// IK
void inverseKinematics(){
  bisa = false;
  float target[3] = {tx, ty, tz};

  // tolak bila target lebih jauh dari panjang lengan (dari poros bahu)
  float r = sqrt(tx * tx + ty * ty);
  float jarak = sqrt(r * r + (tz - L0) * (tz - L0));
  if (jarak > L1 + L2 + L3) return;

  // batas sendi dalam sudut kinematika (radian), dari batas servo 0-180
  float lo[4], hi[4];
  for (int i = 0; i < 4; i++) {
    float a = (0   - offset[i]) / arah[i] * DEG_TO_RAD;
    float b = (180 - offset[i]) / arah[i] * DEG_TO_RAD;
    lo[i] = min(a, b);
    hi[i] = max(a, b);
  }

  // tebakan awal bahu, siku, pergelangan (radian); baris pertama tidak dipakai
  const float awal[6][3] = {
    {0, 0, 0},
    {0.5, -1.0, 0.5}, {1.0, -1.0, -0.3}, {0.3, 1.0, -1.0},
    {0.8, -0.6, -0.8}, {1.2, -0.8, -1.2}
  };

  for (int n = 0; n < 6; n++) {
    float q[4];
    if (n == 0) {
      for (int i = 0; i < 4; i++) q[i] = (servoDeg[i] - offset[i]) / arah[i] * DEG_TO_RAD;
      // lengan lurus = singular (Jacobian kehilangan arah), beri tekukan awal
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
  for (int i = 0; i < 4; i++) servoCalon[i] = arah[i] * sudut[i] + offset[i];
}

void servoKeSudut(){
  for (int i = 0; i < 4; i++) sudut[i] = (servoCalon[i] - offset[i]) / arah[i];
}

// Servo hanya menerima 0-180; calon di luar itu ditolak, tujuan aktif tidak berubah
bool cekBatas(){
  if (!bisa) return false;
  for (int i = 0; i < 4; i++) {
    if (servoCalon[i] < -0.01 || servoCalon[i] > 180.01) return false;   // toleransi pembulatan
  }
  return true;
}

// Dipanggil terus-menerus dari loop(): maju 1 derajat tiap LANGKAH_MS menuju servoDeg.
// Tidak memblokir, sehingga web tetap dilayani dan tombol Stop bisa bekerja.
void langkahServo(){
  static unsigned long terakhir = 0;
  if (millis() - terakhir < LANGKAH_MS) return;
  terakhir = millis();

  for (int i = 0; i < 4; i++) {
    if (servoNow[i] < servoDeg[i]) {
      servoNow[i] = min(servoNow[i] + 1, servoDeg[i]);
      servo[i].write((int)servoNow[i]);
    } else if (servoNow[i] > servoDeg[i]) {
      servoNow[i] = max(servoNow[i] - 1, servoDeg[i]);
      servo[i].write((int)servoNow[i]);
    }
  }
}

// Berhenti di posisi sekarang. Servo tetap menahan posisi (torsi aktif).
void berhenti(){
  for (int i = 0; i < 4; i++) servoDeg[i] = servoNow[i];
}

// Semua sudut kinematika 0 = lengan lurus mendatar (servo langsung melompat)
void nolkanPosisi(){
  for (int i = 0; i < 5; i++) sudut[i] = 0;
  sudutKeServo();
  for (int i = 0; i < 4; i++) {
    servoDeg[i] = servoCalon[i];
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

// Alur satu perintah: hitung calon -> cek batas -> jadikan tujuan aktif -> laporan
// m = 0: target (tx ty tz), m = 2: sudut servo langsung (servoCalon sudah diisi)
bool proses(int m){
  if (m == 0) { inverseKinematics(); if (bisa) sudutKeServo(); }
  else        { bisa = true; servoKeSudut(); }

  if (!cekBatas()) {
    laporan = "Tidak terjangkau (di luar jangkauan atau melebihi batas servo)";
    return false;
  }

  for (int i = 0; i < 4; i++) servoDeg[i] = servoCalon[i];   

  forwardKinematics();                      
  laporan = "Servo: ";
  for (int i = 0; i < 4; i++) laporan += String(servoCalon[i], 0) + " ";
  laporan += "\nFK   : " + String(x, 1) + ", " + String(y, 1) + ", " + String(z, 1);
  laporan += "\nGripper: " + String(sudut[1] + sudut[2] + sudut[3], 0) + " deg dari horizontal";
  return true;
}


const char HALAMAN[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lengan Robot</title>
<style>
  body { font-family: sans-serif; max-width: 420px; margin: 20px auto; padding: 0 12px; }
  input[type=range] { width: 100%; }
  input[type=number] { width: 60px; padding: 6px; margin: 2px; }
  button { padding: 10px 14px; margin: 4px 2px; }
  .stop { background: #c62828; color: white; font-weight: bold; font-size: 18px; width: 100%; padding: 14px; border: 0; border-radius: 6px; }
  pre { background: #eee; padding: 10px; white-space: pre-wrap; }
</style>
</head>
<body>
<button class="stop" onclick="berhenti()">STOP</button>

<h3>Posisi ujung (cm)</h3>
x: <b id="vx">-</b><br>
<input type="range" id="x" min="0" max="35" step="0.5" value="30" oninput="geser()">
y: <b id="vy">-</b><br>
<input type="range" id="y" min="-35" max="35" step="0.5" value="0" oninput="geser()">
z: <b id="vz">-</b><br>
<input type="range" id="z" min="0" max="45" step="0.5" value="11" oninput="geser()">

<h3>Gripper</h3>
<button onclick="kirim('/g?val=60')">Buka</button>
<button onclick="kirim('/g?val=120')">Tutup</button>

<details>
<summary>Kalibrasi: sudut servo langsung</summary>
<input type="number" id="a" value="90"><input type="number" id="b" value="90"><input type="number" id="c" value="90"><input type="number" id="d" value="90"><br>
<button onclick="kirimServo()">Kirim</button>
</details>

<pre id="out">Siap.</pre>

<script>
function v(id){ return document.getElementById(id).value; }
function tampil(t){ document.getElementById('out').textContent = t; }

var sibuk = false, tertunda = false;

function geser(){
  document.getElementById('vx').textContent = v('x');
  document.getElementById('vy').textContent = v('y');
  document.getElementById('vz').textContent = v('z');
  kirimIK();
}

// Hanya satu permintaan berjalan; nilai slider terbaru dikirim begitu yang lama selesai
function kirimIK(){
  if (sibuk) { tertunda = true; return; }
  sibuk = true;
  fetch('/ik?x=' + v('x') + '&y=' + v('y') + '&z=' + v('z'))
    .then(function(r){ return r.text(); })
    .then(tampil)
    .catch(function(){ tampil('Gagal terhubung ke ESP32'); })
    .finally(function(){
      sibuk = false;
      if (tertunda) { tertunda = false; kirimIK(); }
    });
}

function kirim(url){
  fetch(url).then(function(r){ return r.text(); }).then(tampil)
    .catch(function(){ tampil('Gagal terhubung ke ESP32'); });
}

function berhenti(){ tertunda = false; kirim('/stop'); }
function kirimServo(){ kirim('/v?a='+v('a')+'&b='+v('b')+'&c='+v('c')+'&d='+v('d')); }

// Saat halaman dibuka, isi slider sesuai posisi tujuan lengan sekarang
fetch('/pos').then(function(r){ return r.text(); }).then(function(t){
  var p = t.split(',');
  document.getElementById('x').value = p[0];
  document.getElementById('y').value = p[1];
  document.getElementById('z').value = p[2];
  document.getElementById('vx').textContent = v('x');
  document.getElementById('vy').textContent = v('y');
  document.getElementById('vz').textContent = v('z');
}).catch(function(){});
</script>
</body>
</html>
)rawliteral";

void handleIK(){
  if (!(server.hasArg("x") && server.hasArg("y") && server.hasArg("z"))) {
    server.send(400, "text/plain", "Parameter kurang");
    return;
  }
  tx = server.arg("x").toFloat();
  ty = server.arg("y").toFloat();
  tz = server.arg("z").toFloat();

  proses(0);                               
  server.send(200, "text/plain", laporan);
}

void handleServo(){
  if (!(server.hasArg("a") && server.hasArg("b") && server.hasArg("c") && server.hasArg("d"))) {
    server.send(400, "text/plain", "Parameter kurang");
    return;
  }
  servoCalon[0] = server.arg("a").toFloat();
  servoCalon[1] = server.arg("b").toFloat();
  servoCalon[2] = server.arg("c").toFloat();
  servoCalon[3] = server.arg("d").toFloat();

  proses(2);
  server.send(200, "text/plain", laporan);
}

void handleGripper(){
  setGripper(server.arg("val").toFloat());
  server.send(200, "text/plain", "Gripper: " + String(servoNow[4], 0));
}

void handleStop(){
  berhenti();
  server.send(200, "text/plain", "BERHENTI di posisi sekarang");
}

// Posisi tujuan lengan saat ini (dari servoDeg), untuk mengisi slider saat halaman dibuka
void handlePos(){
  for (int i = 0; i < 4; i++) sudut[i] = (servoDeg[i] - offset[i]) / arah[i];
  forwardKinematics();
  server.send(200, "text/plain", String(x, 1) + "," + String(y, 1) + "," + String(z, 1));
}

void setup(){
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) servo[i].attach(pinServo[i]);
  nolkanPosisi();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Menyambung WiFi");
  unsigned long mulai = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - mulai < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Buka di browser: http://");
    Serial.println(WiFi.localIP());
    server.on("/", []() { server.send_P(200, "text/html", HALAMAN); });
    server.on("/ik", handleIK);
    server.on("/v", handleServo);
    server.on("/g", handleGripper);
    server.on("/stop", handleStop);
    server.on("/pos", handlePos);
    server.begin();
  } else {
    Serial.println("WiFi gagal. Periksa SSID dan password, lalu reset.");
  }
}

void loop(){
  server.handleClient();
  langkahServo();
}