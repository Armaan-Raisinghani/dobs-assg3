#include "eigen.h"
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Eigen/Geometry>

using namespace Eigen;

float acc_x, acc_y, acc_z; //accelerometer measurement
float mag_x, mag_y, mag_z; //magneto measurements (raw & calibrated)
float mx_o, my_o, mz_o; //reference magnetic field

int skip = 1;

Vector3f Omega; //angular vel. from gyro
Vector3f a_mes; //accel measurement vector
Vector3f m_mes; //magneto measurement vector

Vector3f e_1; // standard e_1 basis vector
Vector3f e_3; // standard e_3 basis vector (accel reference)
Vector3f mag_ref; // reference magneto vector
Vector3f w_o; // //constant gyro bias vector

//triad construction vectors: qr,rr,sr- reference, qb,rb,sb- body
Vector3f qr, rr, sr, qb, rb, sb;

//triad reference and body rotation
Matrix3f M_ref, M_bod;

//triad rotataion matrix
Matrix3f R_triad;

Matrix3f skew(const Vector3f& v);

uint32_t loop_start_timer;
// #define Tus 10000
#define Tus 10000
// #define Ts 0.01  //100Hz Kalman Iteration speed
#define Ts 0.01

void setup() {
  Serial.begin(115200);

  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //accel details
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  e_1 << 1, 0, 0; //standard vectors
  e_3 << 0, 0, 1;

  //find reference magnetic field direction m_o using 2000 readings
  get_ref_mag();

  //construct reference-triad
  qr = -e_3;
  rr = skew(qr)*mag_ref;
  rr = rr.normalized(); // Normalise
  sr = skew(qr)*rr;

  M_ref.col(0) = qr;
  M_ref.col(1) = rr;
  M_ref.col(2) = sr;
}

void loop() {
  loop_start_timer = micros();

  //obtain accel readings
  accelMeasure(a_mes);

  //obtain magneto readings
  magnetoMeasure(m_mes);

  m_mes = m_mes.normalized();
  a_mes = a_mes.normalized();

  //construct measurement triad
  qb = a_mes;
  rb = skew(qb)*m_mes;
  rb = rb.normalized(); // Normalise
  sb = skew(qb)*rb;

  M_bod.col(0) = qb;
  M_bod.col(1) = rb;
  M_bod.col(2) = sb;

  //triad rotation matrix
  R_triad = M_ref*M_bod.transpose();

  // For display
  Quaternionf q(R_triad);
  q.normalize();
  if (skip > 0)
  {
      // Print comma separated for Serial Plotter
      Serial.print(q.w()); Serial.print(",");
      Serial.print(q.x()); Serial.print(",");
      Serial.print(q.y()); Serial.print(",");
      Serial.println(q.z());
  }
  skip = -skip;
  while (micros() - loop_start_timer < Tus)
    ;
}

//function to get accel measurements
void accelMeasure(Vector3f &accel)
{
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
   }
  accel << -acc_y, -acc_x, -acc_z;
}

//function to get magnetometer measurements
void magnetoMeasure(Vector3f &mag)
{
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mag_x, mag_y, mag_z);
  }
  calibrateMagnetometer(mag_x, mag_y, mag_z);
  mag << mag_y, -mag_x, mag_z;
}

//function to get reference magnetic field
void get_ref_mag()
{
  Vector3f m_sum(0,0,0);
  for (int i = 0; i < 2000; i++) {
    magnetoMeasure(m_mes);
    m_sum = m_sum + m_mes;
  }
  mag_ref = m_sum/2000;
  mag_ref = mag_ref.normalized();
}

void calibrateMagnetometer(float &mx, float &my, float &mz) {
  // Bias
  Vector3f bias;
  bias <<  8.547923, -22.503305, 1.225360 ;

  // Soft iron correction matrix
  Matrix3f M;
  M << 1.000348, -0.003213, -0.011183,
       -0.003213,  1.010146, -0.114517,
       -0.011183, -0.114517,  0.934820; ;

  // Apply hard iron correction
 // Raw measurement as a vector
  Vector3f mag_raw(mx, my, mz);

  // Apply calibration: (mag - bias) → soft iron correction
  Vector3f mag_calibrated = M * (mag_raw - bias);

  // Output as scalars
  mx = mag_calibrated(0);
  my = mag_calibrated(1);
  mz = mag_calibrated(2);
}

Matrix3f skew(const Vector3f& v) {
  Matrix3f m;
  m <<    0,   -v(2),  v(1),
        v(2),     0,  -v(0),
       -v(1),  v(0),     0;
  return m;
}
