/* Membuat Header File Kalman Filter     	   */
/* Referensi Kristian Lauszus, TKJ Electronics */

#include <Kalman.c>

// Variabel Kalman filter
float Q_angle; // Noise variansi untuk accelerometer
float Q_bias; // Noise variansi untuk bias gyro
float R_measure; // Nilai noise variansi yang terukur

float angle; // Sudut hasil kalkulasi Kalman filter - bagian dari matriks 2x1
float bias; // Hasil perhitungan bias gyro oleh the Kalman filter - bagian dari matriks 2x1
float rate; // Hasil perhitungan rate yang tidak bias

float P[2][2]; // Error covariance matrix - Matriks 2x2

// Deklarasi Prosedur dan Fungsi
void Kalman();

// Sudut dalam derajat, rate dalam sudut/detik, waktu delta dalam detik
float getAngle(float newAngle, float newRate, float dt);

void setAngle(float angle); // Untuk mengatur sudut (ini menjadi sudut awal)
float getRate(); // Return the unbiased rate

// Tunning Kalman filter
void setQangle(float Q_angle);
void setQbias(float Q_bias);
void setRmeasure(float R_measure);

float getQangle();
float getQbias();
float getRmeasure();
