#include <ArduinoEigen.h>
#include <Wire.h>
#include <Arduino_LSM9DS1.h>

using namespace Eigen;

float acc_x, acc_y, acc_z;   // accelerometer measurement
float mag_x, mag_y, mag_z;   // magnetometer measurement

Vector3f a_o;                // accelerometer bias

int skip = 1;

Vector3f a_mes;              // accel measurement vector
Vector3f m_mes;              // magnetometer measurement vector

Vector3f e_1;                // standard e_1 basis vector
Vector3f e_3;                // standard e_3 basis vector
Vector3f mag_ref;            // reference magnetic field

// triad construction vectors
Vector3f qr, rr, sr, qb, rb, sb;

// triad reference and body rotation
Matrix3f M_ref, M_bod;

// triad rotation matrix
Matrix3f R_triad;

Matrix3f skew(const Vector3f& v);

uint32_t loop_start_timer;
#define Tus 10000
#define Ts 0.01

void get_accel_bias()
{
  Vector3f a_sum(0, 0, 0);

  for (int i = 0; i < 2000; i++) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(acc_x, acc_y, acc_z);
    }

    Vector3f a_temp;
    a_temp << -acc_y, -acc_x, -acc_z;   // same remapping as accelMeasure

    a_sum += a_temp;
    delay(5);
  }

  Vector3f a_avg = a_sum / 2000.0;
  Vector3f a_expected(0, 0, -1);

  a_o = a_avg - a_expected;

  Serial.println("Accel bias:");
  Serial.print(a_o(0)); Serial.print(", ");
  Serial.print(a_o(1)); Serial.print(", ");
  Serial.println(a_o(2));
}

void setup() {
  Serial.begin(115200);

  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();

  Serial.print("Magnetometer sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();

  e_1 << 1, 0, 0;
  e_3 << 0, 0, 1;

  a_o << 0, 0, 0;

  // Keep board flat and still during startup
  get_accel_bias();

  // Find reference magnetic field direction
  get_ref_mag();

  // Construct reference triad
  qr = -e_3;
  rr = skew(qr) * mag_ref;
  rr = rr.normalized();
  sr = skew(qr) * rr;

  M_ref.col(0) = qr;
  M_ref.col(1) = rr;
  M_ref.col(2) = sr;

  delay(3000);
}

void loop() {
  loop_start_timer = micros();

  // Obtain calibrated accel readings
  accelMeasure(a_mes);

  // Obtain calibrated magnetometer readings
  magnetoMeasure(m_mes);

  m_mes = m_mes.normalized();
  a_mes = a_mes.normalized();

  // Construct measurement triad
  qb = a_mes;
  rb = skew(qb) * m_mes;
  rb = rb.normalized();
  sb = skew(qb) * rb;

  M_bod.col(0) = qb;
  M_bod.col(1) = rb;
  M_bod.col(2) = sb;

  // TRIAD rotation matrix
  R_triad = M_ref * M_bod.transpose();

  Quaternionf q(R_triad);
  q.normalize();

  if (skip > 0) {
    Serial.print(q.w()); Serial.print(",");
    Serial.print(q.x()); Serial.print(",");
    Serial.print(q.y()); Serial.print(",");
    Serial.println(q.z());
  }
  skip = -skip;

  while (micros() - loop_start_timer < Tus);
}

// Calibrated accelerometer measurement
void accelMeasure(Vector3f &accel)
{
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
  }

  accel << -acc_y, -acc_x, -acc_z;
  accel = accel - a_o;
}

// Calibrated magnetometer measurement
void magnetoMeasure(Vector3f &mag)
{
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mag_x, mag_y, mag_z);
  }

  calibrateMagnetometer(mag_x, mag_y, mag_z);
  mag << mag_y, -mag_x, mag_z;
}

// Get reference magnetic field
void get_ref_mag()
{
  Vector3f m_sum(0, 0, 0);

  for (int i = 0; i < 2000; i++) {
    magnetoMeasure(m_mes);
    m_sum += m_mes;
    delay(5);
  }

  mag_ref = m_sum / 2000.0;
  mag_ref = mag_ref.normalized();
}

// Magnetometer calibration
void calibrateMagnetometer(float &mx, float &my, float &mz) {
  Vector3f bias;
  bias << 8.547923, -22.503305, 1.225360;

  Matrix3f M;
  M << 1.000348, -0.003213, -0.011183,
      -0.003213,  1.010146, -0.114517,
      -0.011183, -0.114517,  0.934820;

  Vector3f mag_raw(mx, my, mz);
  Vector3f mag_calibrated = M * (mag_raw - bias);

  mx = mag_calibrated(0);
  my = mag_calibrated(1);
  mz = mag_calibrated(2);
}

Matrix3f skew(const Vector3f& v) {
  Matrix3f m;
  m <<     0,   -v(2),  v(1),
        v(2),       0, -v(0),
       -v(1),   v(0),      0;
  return m;
}
