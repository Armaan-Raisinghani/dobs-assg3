#include "eigen.h"
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Eigen/Geometry>

using namespace Eigen;

float acc_x, acc_y, acc_z; //accelerometer measurement
float wx, wy, wz; //gyro measurement
float mag_x, mag_y, mag_z; //magneto measurements (raw & calibrated)
float k_m, k_a, k_i, k_p; //gains
float roll, pitch, yaw;
float mx_o, my_o, mz_o; //reference magnetic field
float wx_o, wy_o, wz_o; //constant gyro bias

int skip = 1;

Matrix3f R_est; //observer attitude estimate
Matrix3f R_gyro; //Rotation matrix from gyro integration
Vector3f b_est; //observer bias estimate
Vector3f a_est; //estimated accel measurement
Vector3f m_est; //estimated magneto measurement

Vector3f Omega; //angular vel. from gyro
Vector3f a_mes; //accel measurement vector
Vector3f m_mes; //magneto measurement vector

Vector3f e_1; // standard e_1 basis vector
Vector3f e_3; // standard e_3 basis vector (accel reference)
Vector3f mag_ref; // reference magneto vector
Vector3f w_o; // //constant gyro bias vector

Vector3f w_mes; // correction term (explicit)
Vector3f w_passive; // correction term (direct & passive)
Vector3f omega_update; // updated omega
Matrix3f m_skew;
Matrix3f a_skew;
Matrix3f R_E, proj_anti; //Rotation error, proejected matrix

Vector3f qr, rr, sr, qb, rb, sb; //triad construction vectors: qr,rr,sr- reference, qb,rb,sb- body
Matrix3f M_ref, M_bod; //triad reference and body rotation
Matrix3f R_triad; //triad rotataion matrix


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
  //gyro details
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  //accel details
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  //initialize R_est(0), b_est(0)
  R_est << 1, 0, 0,
           0, 1, 0,
           0, 0, 1 ;
  b_est << 0, 0, 0;
  //initialize R_gyro(0)
  R_gyro << 1, 0, 0,
           0, 1, 0,
           0, 0, 1 ;

  e_1 << 1, 0, 0; //standard vectors
  e_3 << 0, 0, 1;

  //assign gains
  k_p = 5.0;
  k_i = 1.0;
  k_m = 0.50;
  k_a = 1.0;

  //get gyro bias
  get_gyro_bias();

  //find reference magnetic field direction m_o using 2000 readings
  get_ref_mag();

  //construct reference-triad
  qr = -e_3;
  rr = skew(qr)*mag_ref;
  rr = rr.normalized();
  sr = skew(qr)*rr;
  M_ref.col(0) = qr;
  M_ref.col(1) = rr;
  M_ref.col(2) = sr;

  // loop_timer = micros();
}

void loop() {
  loop_start_timer = micros();

  //obstain gyro readings
  gyroMeasure(Omega);

  //gyro integration
  updateRotation(R_gyro, Omega, Ts);

  //obtain accel readings
  accelMeasure(a_mes);

  //obtain magneto readings
  magnetoMeasure(m_mes);

  m_mes = m_mes.normalized();
  a_mes = a_mes.normalized();

  //construct measurement triad
  qb = a_mes;
  rb = skew(qb)*m_mes;
  rb = rb.normalized();
  sb = skew(qb)*rb;
  M_bod.col(0) = qb;
  M_bod.col(1) = rb;
  M_bod.col(2) = sb;
  R_triad = M_ref*M_bod.transpose();//triad rotation matrix

  //measurement estimates
  m_est = R_est.transpose()*mag_ref;
  a_est = R_est.transpose()*(-e_3);

  m_skew = skew(m_mes);
  a_skew = skew(a_mes);

  //correction term
  // explicit
  // w_mes = ;
  // direct and passive
  R_E = R_est.transpose()*R_triad;
  proj_anti = 0.5f*(R_E - R_E.transpose());
  w_passive << proj_anti(2,1), proj_anti(0,2), proj_anti(1,0);

  // Bias Update
  b_est = b_est - Ts*k_i*w_passive;

  // Explicit
  // omega_update = ;
  // passive
  omega_update = Omega - b_est + k_p*w_passive;
  // direct
  // omega_update = Omega - b_est + k_p*(R_est*w_passive);

  // Update orientation
  updateRotation(R_est, omega_update, Ts);

  //convert to quaternions and print
  Quaternionf q1(R_gyro);
  q1.normalize();
  Quaternionf q2(R_triad);
  q2.normalize();
  Quaternionf q3(R_est);
  q3.normalize();

  if (skip > 0)
  {
      // Print comma separated for Serial Plotter
      Serial.print(q1.w()); Serial.print(",");
      Serial.print(q1.x()); Serial.print(",");
      Serial.print(q1.y()); Serial.print(",");
      Serial.print(q1.z()); Serial.print(",");
      Serial.print(q2.w()); Serial.print(",");
      Serial.print(q2.x()); Serial.print(",");
      Serial.print(q2.y()); Serial.print(",");
      Serial.print(q2.z()); Serial.print(",");
      Serial.print(q3.w()); Serial.print(",");
      Serial.print(q3.x()); Serial.print(",");
      Serial.print(q3.y()); Serial.print(",");
      Serial.println(q3.z());
  }
  skip = -skip;

  // yaw   = atan2(R_est(1,0), R_est(0,0));
  // pitch = -asin(R_est(2,0));
  // roll  = atan2(R_est(2,1), R_est(2,2));
  // Serial.print(roll, 6);   Serial.print(",");
  // Serial.print(pitch, 6);  Serial.print(",");
  // Serial.println(yaw, 6);
  //print_mtxf(R_est);
  //Serial.println(micros() - loop_timer);

  while (micros() - loop_start_timer < Tus)
    ;
}

Matrix3f skew(const Vector3f& v) {
  Matrix3f m;
  m <<    0,   -v(2),  v(1),
        v(2),     0,  -v(0),
       -v(1),  v(0),     0;
  return m;
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

// Update R using exponential map
void updateRotation(Matrix3f &R, const Vector3f &omega, float dt) {
  float theta = omega.norm() * dt;

  if(theta < 1e-6) {
    // Use first-order approx if angle small
    R = R + dt * R * skew(omega);
  } else {
    Vector3f u = omega.normalized();
    Matrix3f K = skew(u);
    Matrix3f Exp = Matrix3f::Identity()
                   + sin(theta)*K
                   + (1-cos(theta))*(K*K);
    R = R * Exp;
  }
}

void print_mtxf(const Eigen::MatrixXf& X)
{
   int i, j, nrow, ncol;
   nrow = X.rows();
   ncol = X.cols();
   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);
   Serial.println();
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}
//function to get gyro measurements
void gyroMeasure(Vector3f &Omega)
{
  //For ENU frame - exchange sensor x and y directions
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(wx, wy, wz);
    }
  Omega << wy, wx, wz;
  Omega = Omega*3.14/180.0; //change unit to rad/sec
  Omega = Omega - w_o; //substract bias
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
//function to get gyro bias
void get_gyro_bias()
{
  w_o << 0,0,0;
  Vector3f w_o_sum(0,0,0);
  for (int i = 0; i < 1000; i++) {
    gyroMeasure(Omega);
    w_o_sum = w_o_sum + Omega;
    delay(5);
  }
  w_o = w_o_sum/1000;
}
//function to get reference magnetic field
void get_ref_mag()
{
  Vector3f m_sum(0,0,0);
  for (int i = 0; i < 1000; i++) {
    magnetoMeasure(m_mes);
    m_sum = m_sum + m_mes;
    delay(5);
  }
  mag_ref = m_sum/1000;
  mag_ref = mag_ref.normalized();
}