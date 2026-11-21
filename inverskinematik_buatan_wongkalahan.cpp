#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

int main() {
  const double PI = 3.14159265358979323846;
  double L2, L3;
  double x, y, z;

  // 1. INPUT PANJANG LENGAN
  cout << "=== INPUT PANJANG LENGAN ===" << endl;
  cout << "Masukkan panjang L2: ";
  cin >> L2;
  cout << "Masukkan panjang L3: ";
  cin >> L3;

  if (L2 <= 0 || L3 <= 0) {
    cout << "\n[ERROR] Panjang L2 dan L3 harus lebih dari 0." << endl;
    return 0;
  }

  // 2. INPUT KOORDINAT TARGET (END-EFFECTOR)
  cout << "\n=== INPUT KOORDINAT TARGET ===" << endl;
  cout << "Masukkan nilai X: ";
  cin >> x;
  cout << "Masukkan nilai Y: ";
  cin >> y;
  cout << "Masukkan nilai Z: ";
  cin >> z;

  // 3. PERHITUNGAN TAMPAK ATAS (Bidang X, Y)
  double theta_1 = atan2(y, x) * (180.0 / PI);
  double r = sqrt((x * x) + (y * y));

  // 4. PERHITUNGAN TAMPAK SAMPING (Bidang r, Z)
  double d = sqrt((r * r) + (z * z));

  // Cek apakah target masuk dalam jangkauan lengan
  if (d > (L2 + L3) || d < fabs(L2 - L3)) {
    cout << "\n[GAGAL] Koordinat target di luar jangkauan lengan!" << endl;
    cout << "Jarak miring d harus antara " << fabs(L2 - L3) << " sampai "
         << (L2 + L3) << endl;
    return 0;
  }

  // Hitung sudut dalam C dan Theta 3
  double cos_C = ((L2 * L2) + (L3 * L3) - (d * d)) / (2.0 * L2 * L3);
  if (cos_C > 1.0) cos_C = 1.0;
  if (cos_C < -1.0) cos_C = -1.0;

  double C = acos(cos_C) * (180.0 / PI);
  double theta_3 = 180.0 - C;

  // Hitung Theta 2
  double cos_sudut_dalam = ((L2 * L2) + (d * d) - (L3 * L3)) / (2.0 * L2 * d);
  if (cos_sudut_dalam > 1.0) cos_sudut_dalam = 1.0;
  if (cos_sudut_dalam < -1.0) cos_sudut_dalam = -1.0;

  double sudut_dalam = acos(cos_sudut_dalam) * (180.0 / PI);
  double sudut_elevasi = atan2(z, r) * (180.0 / PI);
  double theta_2 = sudut_dalam + sudut_elevasi;

  // 5. TAMPILKAN HASIL (2 angka di belakang koma)
  cout << fixed << setprecision(2);
  cout << "\n=== HASIL PERHITUNGAN INVERSE KINEMATICS ===" << endl;
  cout << "Tampak Atas:" << endl;
  cout << "  Jarak datar (r)      = " << r << endl;
  cout << "  Sudut Theta 1 (Base) = " << theta_1 << " derajat" << endl;

  cout << "\nTampak Samping:" << endl;
  cout << "  Jarak miring (d)     = " << d << endl;
  cout << "  Sudut dalam (C)      = " << C << " derajat" << endl;
  cout << "  Sudut Theta 2 (L2)   = " << theta_2 << " derajat" << endl;
  cout << "  Sudut Theta 3 (L3)   = " << theta_3 << " derajat" << endl;

  return 0;
}